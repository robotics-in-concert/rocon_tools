#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: manager
   :platform: Unix
   :synopsis: The ros level node class that manages interactions.


This module defines the class used to execute a ros node responsible for
managing the ros api that manipulates interactions.
----

"""
##############################################################################
# Imports
##############################################################################

import re
import uuid

import rospy
import rosgraph
import unique_id
import rocon_interaction_msgs.msg as interaction_msgs
import rocon_interaction_msgs.srv as interaction_srvs
import rocon_uri
import socket

from .remocon_monitor import RemoconMonitor
from .interactions_table import InteractionsTable
from . import interactions
from .exceptions import MalformedInteractionsYaml, YamlResourceNotFoundException
from .rapp_handler import RappHandler, FailedToStartRappError, FailedToStopRappError


##############################################################################
# Interactions
##############################################################################


class InteractionsManager(object):
    '''
      Manages connectivity information provided by services and provides this
      for human interactive (aka remocon) connections.
    '''
    __slots__ = [
        '_interactions_table',  # Dictionary of string : interaction_msgs.RemoconApp[]
        '_parameters',
        '_rapp_handler',        # Interface for handling interactions-rapps pairing
        '_publishers',
        '_services',
        'spin',
        '_pair',                # interaction_msgs/Pair showing rapp and remocon that are pairing
        '_watch_loop_period',
        '_remocon_monitors'    # list of currently connected remocons.
    ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self._watch_loop_period = 1.0
        self._remocon_monitors = {}                 # topic_name : RemoconMonitor
        self._parameters = self._setup_parameters()  # important to come first since we use self._parameters['pairing'] everywhere
        self._publishers = self._setup_publishers()  # important to come early, so we can publish is_pairing below
        self._rapp_handler = None
        if self._parameters['pairing']:
            self._rapp_handler = RappHandler(self._rapp_manager_status_update_callback)
            self._pair = interaction_msgs.Pair()
            self._publishers['pairing'].publish(self._pair)
        self._interactions_table = InteractionsTable(filter_pairing_interactions=not self._parameters['pairing'])
        self._services = self._setup_services()

        # Load pre-configured interactions
        for resource_name in self._parameters['interactions']:
            try:
                msg_interactions = interactions.load_msgs_from_yaml_resource(resource_name)
                msg_interactions = self._bind_dynamic_symbols(msg_interactions)
                (new_interactions, invalid_interactions) = self._interactions_table.load(msg_interactions)
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
          Loop around parsing the status of 1) connected remocons and 2) an internal
          rapp manager if the node was configured for pairing. Reacts appropriately if it
          identifies important status changes (e.g. a rapp went down while this node
          is currently managing its associated paired interaction).
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
                                                                           self._remocon_status_update_callback)
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
            except socket.error:
                rospy.logerr("Interactions : socket error trying to retrieve information from the local master.")
            rospy.rostime.wallsleep(self._watch_loop_period)

    def _remocon_status_update_callback(self, new_interactions, finished_interactions):
        """
        Called whenever there is a status update on a remocon signifying when an interaction has been started
        or finished. This gets triggered by the RemoconMonitor instances.

        :param new_interactions int32[]: list of hashes for newly started interactions on this remocon.
        :param lost_interactions int32[]: list of hashes for newly started interactions on this remocon.
        """
        # could also possibly use the remocon id here
        if self._parameters['pairing']:
            if self.is_pairing():
                for interaction_hash in finished_interactions:
                    interaction = self._interactions_table.find(interaction_hash)
                    if interaction.is_paired_type():
                        try:
                            self._rapp_handler.stop()
                            self._pair = interaction_msgs.Pair()
                            self._publishers['pairing'].publish(self._pair)
                        except FailedToStopRappError as e:
                            rospy.logerr("Interactions : failed to stop a paired rapp [%s]" % e)
        self._ros_publish_interactive_clients()

    def _rapp_manager_status_update_callback(self):
        """
        Called if the rapp manager has a rapp that is stopping - an indication that we need to
        stop a pairing interaction if one is running.
        """
        self._pair.rapp = ""
        self._publishers['pairing'].publish(self._pair)

    def _setup_publishers(self):
        '''
          These are all public topics. Typically that will drop them into the /concert
          namespace.
        '''
        publishers = {}
        publishers['interactive_clients'] = rospy.Publisher('~interactive_clients',
                                                            interaction_msgs.InteractiveClients,
                                                            latch=True,
                                                            queue_size=5
                                                           )
        if self._parameters['pairing']:
            publishers['pairing'] = rospy.Publisher('~pairing',
                                                    interaction_msgs.Pair,
                                                    latch=True,
                                                    queue_size=5
                                                    )

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
        param['webserver_address'] = rospy.get_param('~webserver_address', 'localhost')
        param['interactions'] = rospy.get_param('~interactions', [])
        param['pairing'] = rospy.get_param('~pairing', False)
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
                interactive_client.running_interactions = []
                for interaction_hash in remocon.status.running_interactions:
                    interaction = self._interactions_table.find(interaction_hash)
                    interactive_client.running_interactions.append(interaction.display_name if interaction is not None else "unknown")
                if interactive_client.running_interactions:
                    interactive_clients.running_clients.append(interactive_client)
                else:
                    interactive_clients.idle_clients.append(interactive_client)
        self._publishers['interactive_clients'].publish(interactive_clients)

    def _ros_service_get_interaction(self, request):
        '''
          Handle incoming requests for a single app.
        '''
        response = interaction_srvs.GetInteractionResponse()
        interaction = self._interactions_table.find(request.hash)
        if interaction is None:
            response.interaction = interaction_msgs.Interaction()
            response.result = False
        else:
            response.interaction = interaction.msg
            response.result = True
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
            unavailable_roles = [x for x in request.roles if x not in self._interactions_table.roles()]
            for role in unavailable_roles:
                rospy.logerr("Interactions : received request for interactions of an unregistered role [%s]" % role)

        uri = request.uri if request.uri != '' else 'rocon:/'
        try:
            filtered_interactions = self._interactions_table.filter(request.roles, uri)
        except rocon_uri.RoconURIValueError as e:
            rospy.logerr("Interactions : received request for interactions to be filtered by an invalid rocon uri"
                         " [%s][%s]" % (uri, str(e)))
            filtered_interactions = []
        for i in filtered_interactions:
            response.interactions.append(i.msg)
        return response

    def _ros_service_get_roles(self, request):
        uri = request.uri if request.uri != '' else 'rocon:/'
        try:
            filtered_interactions = self._interactions_table.filter([], uri)
        except rocon_uri.RoconURIValueError as e:
            rospy.logerr("Interactions : received request for roles to be filtered by an invalid rocon uri"
                         " [%s][%s]" % (rocon_uri, str(e)))
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
            (new_interactions, invalid_interactions) = self._interactions_table.load(interactions)
            for i in new_interactions:
                rospy.loginfo("Interactions : loading %s [%s-%s-%s]" % (i.display_name, i.name, i.role, i.namespace))
            for i in invalid_interactions:
                rospy.logwarn("Interactions : failed to load %s [%s-%s-%s]" (i.display_name,
                                                                             i.name,
                                                                             i.role,
                                                                             i.namespace))
        else:
            removed_interactions = self._interactions_table.unload(request.interactions)
            for i in removed_interactions:
                rospy.loginfo("Interactions : unloading %s [%s-%s-%s]" % (i.display_name, i.name, i.role, i.namespace))
        # send response
        response = interaction_srvs.SetInteractionsResponse()
        response.result = True
        return response

    def _ros_service_request_interaction(self, request):
        interaction = self._interactions_table.find(request.hash)
        # for interaction in self._interactions_table.interactions:
        #     rospy.logwarn("Interactions:   [%s][%s][%s]" % (interaction.name, interaction.hash, interaction.max))
        if interaction is None:
            return _request_interaction_response(interaction_msgs.ErrorCodes.INTERACTION_UNAVAILABLE)
        if interaction.max != interaction_msgs.Interaction.UNLIMITED_INTERACTIONS:
            count = 0
            for remocon_monitor in self._remocon_monitors.values():
                if remocon_monitor.status is not None and remocon_monitor.status.running_interactions:
                    # Todo this is a weak check as it is not necessarily uniquely identifying the interaction
                    # Todo - reintegrate this using full interaction variable instead
                    pass
                    #if remocon_monitor.status.app_name == request.application:
                    #    count += 1
            if count > interaction.max:
                rospy.loginfo("Interactions : rejected interaction request [interaction quota exceeded]")
                return _request_interaction_response(interaction_msgs.ErrorCodes.INTERACTION_QUOTA_REACHED)
        if self._parameters['pairing']:
            if interaction.is_paired_type():
                # abort if already pairing
                if self.is_pairing():
                    rospy.loginfo("Interactions : rejected interaction request [already pairing]")
                    return _request_interaction_response(interaction_msgs.ErrorCodes.ALREADY_PAIRING)
                try:
                    self._rapp_handler.start(interaction.pairing.rapp, interaction.pairing.remappings)
                    self._pair = interaction_msgs.Pair(rapp=interaction.pairing.rapp, remocon=request.remocon)
                    self._publishers['pairing'].publish(self._pair)
                except FailedToStartRappError as e:
                    rospy.loginfo("Interactions : rejected interaction request [failed to start the paired rapp]")
                    response = _request_interaction_response(interaction_msgs.ErrorCodes.START_PAIRED_RAPP_FAILED)
                    response.message = "Failed to start the rapp [%s]" % str(e)  # custom response
                    return response
        # if we get here, we've succeeded.
        return _request_interaction_response(interaction_msgs.ErrorCodes.SUCCESS)

    ##########################################################################
    # Utility functions
    ##########################################################################

    def _bind_dynamic_symbols(self, interactions):
        '''
          Provide some intelligence to the interactions specification by binding designated
          symbols at runtime.

          - interaction.name - __WEBSERVER_ADDRESS__
          - interaction.compatibility - __ROSDISTRO__ (depracated - use | in the compatibility variable itself)
          - interaction.parameters - __ROSBRIDGE_ADDRESS__
          - interaction.parameters - __ROSBRIDGE_PORT__

          :param interaction: parse this interaction scanning and replacing symbols.
          :type interaction: request_interactions_msgs.Interaction[]

          :returns: the updated interaction list
          :rtype: request_interactions_msgs.Interaction[]
        '''

        # Binding runtime CONSTANTS
        for interaction in interactions:
            interaction.name = interaction.name.replace('__WEBSERVER_ADDRESS__', self._parameters['webserver_address'])
            interaction.parameters = interaction.parameters.replace('__ROSBRIDGE_ADDRESS__',
                                                                    self._parameters['rosbridge_address'])
            interaction.parameters = interaction.parameters.replace('__ROSBRIDGE_PORT__',
                                                                    str(self._parameters['rosbridge_port']))
            #interaction.compatibility = interaction.compatibility.replace('%ROSDISTRO%',
            #                                                              rocon_python_utils.ros.get_rosdistro())

        # Binding ros param values
        pattern = '\ __(.*?)__[,|\}]' # searchs for pattern '<space>__PARAMNAME__,'
        for interaction in interactions:
            for p in re.findall(pattern, interaction.parameters):
                if p.startswith('/'):
                    rparam = rospy.get_param(p)
                elif p.startswith('~'):
                    msg = '%s is invalid format for rosparam binding. See https://github.com/robotics-in-concert/rocon_tools/issues/81'%p
                    raise MalformedInteractionsYaml(str(msg))
                else:
                    # See https://github.com/robotics-in-concert/rocon_tools/issues/81 for the rule
                    ns = interaction.namespace
                    if not ns:
                        rparam = rospy.get_param('~'+p)
                    else:
                        rparam = rospy.get_param(ns+'/'+p)
                match = '__' + p + '__'
                interaction.parameters = interaction.parameters.replace(match, str(rparam))
        return interactions

    def is_pairing(self):
        """
        Whether this interactions manager is currently managing a pairing interaction
        or not.

        :returns: whether there is an active pairing or not
        :rtype bool:
        """
        # just check if either string is non-empty
        return self._pair.rapp or self._pair.remocon

##############################################################################
# Utility methods/factories
##############################################################################

request_interaction_error_messages = {
                                interaction_msgs.ErrorCodes.SUCCESS: 'Success',
                                interaction_msgs.ErrorCodes.INTERACTION_UNAVAILABLE: interaction_msgs.ErrorCodes.MSG_INTERACTION_UNAVAILABLE,
                                interaction_msgs.ErrorCodes.INTERACTION_QUOTA_REACHED: interaction_msgs.ErrorCodes.MSG_INTERACTION_QUOTA_REACHED,
                                interaction_msgs.ErrorCodes.ALREADY_PAIRING: interaction_msgs.ErrorCodes.MSG_START_PAIRED_RAPP_FAILED,
                                interaction_msgs.ErrorCodes.START_PAIRED_RAPP_FAILED: interaction_msgs.ErrorCodes.MSG_START_PAIRED_RAPP_FAILED,
                                }


def _request_interaction_response(code):
    response = interaction_srvs.RequestInteractionResponse()
    response.error_code = code
    response.message = request_interaction_error_messages[code]
    response.result = True if code == interaction_msgs.ErrorCodes.SUCCESS else False
    return response
