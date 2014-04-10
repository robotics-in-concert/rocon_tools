#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import uuid

import rospy
import rosgraph
import unique_id
import rocon_interaction_msgs.msg as interaction_msgs
import rocon_interaction_msgs.srv as interaction_srvs
import rocon_uri

from .remocon_monitor import RemoconMonitor
from .interactions_table import InteractionsTable
from . import interactions
from .exceptions import MalformedInteractionsYaml, YamlResourceNotFoundException

##############################################################################
# Interactions
##############################################################################


class InteractionsManager(object):
    '''
      Manages connectivity information provided by services and provides this
      for human interactive (aka remocon) connections.
    '''
    __slots__ = [
        'interactions_table',  # Dictionary of string : interaction_msgs.RemoconApp[]
        'publishers',
        'parameters',
        'services',
        'spin',
        'platform_info',
        '_watch_loop_period',
        '_remocon_monitors'  # list of currently connected remocons.
    ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self.interactions_table = InteractionsTable()
        self.publishers = self._setup_publishers()
        self.services = self._setup_services()
        self.parameters = self._setup_parameters()
        self._watch_loop_period = 1.0
        self._remocon_monitors = {}  # topic_name : RemoconMonitor

        # Load pre-configured interactions
        for resource_name in self.parameters['interactions']:
            try:
                msg_interactions = interactions.load_msgs_from_yaml_resource(resource_name)
                msg_interactions = self._bind_dynamic_symbols(msg_interactions)
                (new_interactions, invalid_interactions) = self.interactions_table.load(msg_interactions)
                for i in new_interactions:
                    rospy.loginfo("Interactions : loading %s [%s-%s-%s]" %
                                  (i.display_name, i.name, i.role, i.namespace))
                for i in invalid_interactions:
                    rospy.logwarn("Interactions : failed to load %s [%s-%s-%s]" %
                                  (i.display_name, i.name, i.role, i.namespace))
            except YamlResourceNotFoundException as e:
                rospy.logerr("Interactions : failed to load resource %s [%s]" %
                             (resource_name, str(e)))
            except MalformedInteractionsYaml as e:
                rospy.logerr("Interactions : pre-configured interactions yaml malformed [%s][%s]" %
                             (resource_name, str(e)))

    def spin(self):
        '''
          Parse the set of /remocons/<name>_<uuid> connections.
        '''
        while not rospy.is_shutdown():
            master = rosgraph.Master(rospy.get_name())
            diff = lambda l1, l2: [x for x in l1 if x not in l2]
            try:
                # This master call returns a filtered list of [topic_name, topic_type] elemnts (list of lists)
                remocon_topics = [x[0] for x in master.getPublishedTopics(interaction_msgs.Strings.REMOCONS_NAMESPACE)]
                new_remocon_topics = diff(remocon_topics, self._remocon_monitors.keys())
                lost_remocon_topics = diff(self._remocon_monitors.keys(), remocon_topics)
                for remocon_topic in new_remocon_topics:
                    self._remocon_monitors[remocon_topic] = RemoconMonitor(remocon_topic,
                                                                           self._ros_publish_interactive_clients)
                    self._ros_publish_interactive_clients()
                    rospy.loginfo("Interactions : new remocon connected [%s]" %  # strips the /remocons/ part
                                  remocon_topic[len(interaction_msgs.Strings.REMOCONS_NAMESPACE) + 1:])
                for remocon_topic in lost_remocon_topics:
                    self._remocon_monitors[remocon_topic].unregister()
                    # careful, this mutates the dictionary
                    # http://stackoverflow.com/questions/5844672/delete-an-element-from-a-dictionary
                    del self._remocon_monitors[remocon_topic]
                    self._ros_publish_interactive_clients()
                    rospy.loginfo("Interactions : remocon left [%s]" %  # strips the /remocons/ part
                                  remocon_topic[len(interaction_msgs.Strings.REMOCONS_NAMESPACE) + 1:])
            except rosgraph.masterapi.Error:
                rospy.logerr("Interactions : error trying to retrieve information from the local master.")
            except rosgraph.masterapi.Failure:
                rospy.logerr("Interactions : failure trying to retrieve information from the local master.")
            rospy.rostime.wallsleep(self._watch_loop_period)

    def _setup_publishers(self):
        '''
          These are all public topics. Typically that will drop them into the /concert
          namespace.
        '''
        publishers = {}
        publishers['interactive_clients'] = rospy.Publisher('~interactive_clients',
                                                            interaction_msgs.InteractiveClients,
                                                            latch=True)
        return publishers

    def _setup_services(self):
        '''
          These are all public services. Typically that will drop them into the /concert
          namespace.
        '''
        services = {}
        services['get_roles'] = rospy.Service('~get_roles',
                                                     interaction_srvs.GetRoles,
                                                     self._ros_service_get_roles)
        services['get_interactions'] = rospy.Service('~get_interactions',
                                                     interaction_srvs.GetInteractions,
                                                     self._ros_service_get_interactions)
        services['get_interaction'] = rospy.Service('~get_interaction',
                                                    interaction_srvs.GetInteraction,
                                                    self._ros_service_get_interaction)
        services['set_interactions'] = rospy.Service('~set_interactions',
                                                     interaction_srvs.SetInteractions,
                                                     self._ros_service_set_interactions)
        services['request_interaction'] = rospy.Service('~request_interaction',
                                                        interaction_srvs.RequestInteraction,
                                                        self._ros_service_request_interaction)
        return services

    def _setup_parameters(self):
        param = {}
        param['rosbridge_address'] = rospy.get_param('~rosbridge_address', "")
        if param['rosbridge_address'] == "":
            param['rosbridge_address'] = 'localhost'
        param['rosbridge_port'] = rospy.get_param('~rosbridge_port', 9090)
        param['interactions'] = rospy.get_param('~interactions', [])
        return param

    ##########################################################################
    # Ros Api Functions
    ##########################################################################

    def _ros_publish_interactive_clients(self):
        interactive_clients = interaction_msgs.InteractiveClients()
        for remocon in self._remocon_monitors.values():
            if remocon.status is not None:  # i.e. we are monitoring it.
                interactive_client = interaction_msgs.InteractiveClient()
                interactive_client.name = remocon.name
                interactive_client.id = unique_id.toMsg(uuid.UUID(remocon.status.uuid))
                interactive_client.platform_info = remocon.status.platform_info
                if remocon.status.running_app:
                    interaction = self.interactions_table.find(remocon.status.hash)
                    interactive_client.app_name = interaction.display_name if interaction is not None else "unknown"
                    interactive_clients.running_clients.append(interactive_client)
                else:
                    interactive_clients.idle_clients.append(interactive_client)
        self.publishers['interactive_clients'].publish(interactive_clients)

    def _ros_service_get_interaction(self, request):
        '''
          Handle incoming requests for a single app.
        '''
        response = interaction_srvs.GetInteractionResponse()
        response.interaction = self.interactions_table.find(request.hash).msg
        response.result = False if response.interaction is None else True
        return response

    def _ros_service_get_interactions(self, request):
        '''
          Handle incoming requests to provide a role-applist dictionary
          filtered for the requesting platform.

          @param request
          @type concert_srvs.GetInteractionsRequest
        '''
        response = interaction_srvs.GetInteractionsResponse()
        response.interactions = []

        if request.roles:  # works for None or empty list
            unavailable_roles = [x for x in request.roles if x not in self.interactions_table.roles()]
            for role in unavailable_roles:
                rospy.logerr("Interactions : received request for interactions of an unregistered role [%s]" % role)

        try:
            filtered_interactions = self.interactions_table.filter(request.roles, request.uri)
        except rocon_uri.RoconURIValueError as e:
            rospy.logerr("Interactions : received request for interactions to be filtered by an invalid rocon uri [%s][%s]" % (request.uri, str(e)))
            filtered_interactions = []
        for i in filtered_interactions:
            response.interactions.append(i.msg)
        return response

    def _ros_service_get_roles(self, request):
        uri = request.uri if request.uri != '' else 'rocon:/'
        try:
            filtered_interactions = self.interactions_table.filter([], uri)
        except rocon_uri.RoconURIValueError as e:
            rospy.logerr("Interactions : received request for roles to be filtered by an invalid rocon uri [%s][%s]" % (rocon_uri, str(e)))
            filtered_interactions = []
        role_list = list(set([i.role for i in filtered_interactions]))
        role_list.sort()
        response = interaction_srvs.GetRolesResponse()
        response.roles = role_list
        return response

    def _ros_service_set_interactions(self, request):
        '''
          Add or remove interactions from the interactions table.

          Note: uniquely identifying apps by name (not very sane).

          @param request list of roles-apps to set
          @type concert_srvs.SetInteractionsRequest
        '''
        if request.load:
            interactions = self._bind_dynamic_symbols(request.interactions)
            (new_interactions, invalid_interactions) = self.interactions_table.load(interactions)
            for i in new_interactions:
                rospy.loginfo("Interactions : loading %s [%s-%s-%s]" % (i.display_name, i.name, i.role, i.namespace))
            for i in invalid_interactions:
                rospy.logwarn("Interactions : failed to load %s [%s-%s-%s]" (i.display_name,
                                                                             i.name,
                                                                             i.role,
                                                                             i.namespace))
        else:
            removed_interactions = self.interactions_table.unload(request.interactions)
            for i in removed_interactions:
                rospy.loginfo("Interactions : unloading %s [%s-%s-%s]" % (i.display_name, i.name, i.role, i.namespace))
        # send response
        response = interaction_srvs.SetInteractionsResponse()
        response.result = True
        return response

    def _ros_service_request_interaction(self, request):
        response = interaction_srvs.RequestInteractionResponse()
        response.result = True
        response.error_code = interaction_msgs.ErrorCodes.SUCCESS
        interaction = self.interactions_table.find(request.hash)
        # for interaction in self.interactions_table.interactions:
        #     rospy.logwarn("Interactions:   [%s][%s][%s]" % (interaction.name, interaction.hash, interaction.max))
        if interaction is not None:
            if interaction.max == interaction_msgs.Interaction.UNLIMITED_INTERACTIONS:
                return response
            else:
                count = 0
                for remocon_monitor in self._remocon_monitors.values():
                    if remocon_monitor.status is not None and remocon_monitor.status.running_app:
                        # Todo this is a weak check as it is not necessarily uniquely identifying the app
                        # Todo - reintegrate this using full interaction variable instead
                        pass
                        #if remocon_monitor.status.app_name == request.application:
                        #    count += 1
                if count < interaction.max:
                    return response
                else:
                    response.error_code = interaction_msgs.ErrorCodes.INTERACTION_QUOTA_REACHED
                    response.message = interaction_msgs.ErrorCodes.MSG_INTERACTION_QUOTA_REACHED
        else:
            response.error_code = interaction_msgs.ErrorCodes.INTERACTION_UNAVAILABLE
            response.message = interaction_msgs.ErrorCodes.MSG_INTERACTION_UNAVAILABLE
        response.result = False
        return response

    ##########################################################################
    # Utility functions
    ##########################################################################

    def _bind_dynamic_symbols(self, interactions):
        '''
          Provide some intelligence to the interactions specification by binding designated
          symbols at runtime.

          - interaction.compatibility - __ROSDISTRO__ (depracated - use | in the compatibility variable itself)
          - interaction.parameters - __ROSBRIDGE_ADDRESS__
          - interaction.parameters - __ROSBRIDGE_PORT__

          @param interaction : parse this interaction scanning and replacing symbols.
          @type request_interactions_msgs.Interaction[]

          @return the updated interaction list
          @rtype request_interactions_msgs.Interaction[]
        '''
        for interaction in interactions:
            interaction.parameters = interaction.parameters.replace('__ROSBRIDGE_ADDRESS__',
                                                                    self.parameters['rosbridge_address'])
            interaction.parameters = interaction.parameters.replace('__ROSBRIDGE_PORT__',
                                                                    str(self.parameters['rosbridge_port']))
            #interaction.compatibility = interaction.compatibility.replace('%ROSDISTRO%',
            #                                                              rocon_python_utils.ros.get_rosdistro())
        return interactions
