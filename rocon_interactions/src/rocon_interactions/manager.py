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

import uuid

import rospy
import rosgraph
import unique_id
import rocon_console.console as console
import rocon_interaction_msgs.msg as interaction_msgs
import rocon_interaction_msgs.srv as interaction_srvs
import rocon_python_comms
import rocon_uri
import socket
import std_msgs.msg as std_msgs

from .exceptions import RappNotRunningError, FailedToStartRappError, FailedToStopRappError
from .interactions_table import InteractionsTable
from .pairings import RuntimePairingSignature
from .rapp_handler import RappHandler
from .remocon_monitor import RemoconMonitor
from .ros_parameters import Parameters
from . import utils

##############################################################################
# Interactions
##############################################################################


class InteractionsManager(object):
    '''
      Manages connectivity information provided by services and provides this
      for human interactive (aka remocon) connections.
    '''
    __slots__ = [  # todo - shift these slots to sphinx ivar, vartype docs
        'parameters',
        'publishers',
        'services',
        '_interactions_table',  # Dictionary of string : interaction_msgs.RemoconApp[]
        '_rapp_handler',        # Interface for handling interactions-rapps pairing
        'spin',
        '_watch_loop_period',
        '_remocon_monitors',    # list of currently connected remocons.
        'runtime_pairing_signatures'
    ]

    ##########################################################################
    # Initialisation & Execution
    ##########################################################################

    def __init__(self):
        self._watch_loop_period = 1.0
        self._remocon_monitors = {}                  # topic_name : RemoconMonitor
        self.parameters = Parameters()              # important to come first since we use self.parameters.pairing everywhere

        # ros communications
        self.services = rocon_python_comms.utils.Services(
            [
                ('~get_roles', interaction_srvs.GetRoles, self._ros_service_get_roles),
                ('~get_interaction', interaction_srvs.GetInteraction, self._ros_service_get_interaction),
                ('~get_interactions', interaction_srvs.GetInteractions, self._ros_service_get_interactions),
                ('~set_interactions', interaction_srvs.SetInteractions, self._ros_service_set_interactions),
                ('~request_interaction', interaction_srvs.RequestInteraction, self._ros_service_request_interaction)
            ]
        )
        latched = True
        queue_size_five = 5
        self.publishers = rocon_python_comms.utils.Publishers(
            [
                ('~parameters', std_msgs.String, latched, queue_size_five),
                ('~interactive_clients', interaction_msgs.InteractiveClients, latched, queue_size_five),
                ('~pairing_events', std_msgs.Bool, not latched, queue_size_five),
                ('~pairings', std_msgs.String, latched, queue_size_five)  # for debugging, show all pairings
            ]
        )
        # small pause (convenience only) to let connections to come up
        rospy.rostime.wallsleep(0.5)
        self.publishers.parameters.publish(std_msgs.String("%s" % self.parameters))

        self._rapp_handler = RappHandler(self._rapp_changed_state_callback) if self.parameters.pairing else None
        self._interactions_table = InteractionsTable(filter_pairing_interactions=not self.parameters.pairing)
        self._interactions_table.load_from_resources(self.parameters.interactions)

        self.runtime_pairing_signatures = []
        self._ros_publish_pairings()

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

    ##########################################################################
    # Callbacks
    ##########################################################################

    def _remocon_status_update_callback(self, remocon_unique_name, new_interactions, finished_interactions):
        """
        Called whenever there is a status update on a remocon signifying when an interaction has been started
        or finished. This gets triggered by the RemoconMonitor instances.

        :param str remocon_unique_name: unique identifier for this remocon
        :param int32[] new_interactions: list of hashes for newly started interactions on this remocon.
        :param int32[] lost_interactions: list of hashes for newly started interactions on this remocon.
        """
        # could also possibly use the remocon id here
        if self.parameters.pairing:
            to_be_removed_signature = None
            for signature in self.runtime_pairing_signatures:
                if signature.interaction.hash in finished_interactions \
                        and signature.remocon_name == remocon_unique_name:
                    to_be_removed_signature = signature
                    break
            if to_be_removed_signature is not None:
                self.runtime_pairing_signatures.remove(to_be_removed_signature)
                self._ros_publish_pairings()
                if not self.runtime_pairing_signatures:
                    try:
                        self._rapp_handler.stop()
                    except FailedToStopRappError as e:
                        rospy.logerr("Interactions : failed to stop a paired rapp [%s]" % e)
        self._ros_publish_interactive_clients()

    def _rapp_changed_state_callback(self, rapp, stopped=False):
        """
        Called if a rapp toggles from start-stop or viceversa. If it's stopping, then
        remocons should use this to drop their current interactions. And whether starting
        or stopping, they should use this as a trigger to refresh their lists if they have
        pairing interactions to consider.

        :param rapp: the rapp (dict form - see :func:`.rapp_handler.rapp_msg_to_dict`) that started or stopped.
        :param stopped:
        """
        self.publishers.pairing_events.publish(std_msgs.Bool(not stopped))
        if stopped:
            self.runtime_pairing_signatures = []
            self._ros_publish_pairings()

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
        self.publishers.interactive_clients.publish(interactive_clients)

    def _ros_publish_pairings(self):
        """
        For debugging purposes only we publish the currently running pairing interactions.
        """
        s = console.bold + "\nRuntime Pairings\n" + console.reset
        for signature in self.runtime_pairing_signatures:
            s += "  %s\n" % signature
        self.publishers.pairings.publish(std_msgs.String("%s" % s))

    def _ros_service_get_interaction(self, request):
        '''
          Handle incoming requests for a single interaction's details.
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

        ################################################
        # Filter by role, rocon_uri
        ################################################
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

            rapp_list = self._rapp_handler.list()
            print rapp_list

        ################################################
        # Filter pairings by running requirements
        ################################################
        if request.runtime_pairing_requirements:
            filtered_interactions = [interaction for interaction in filtered_interactions if self._running_requirements_are_satisfied(interaction)]

        ################################################
        # Convert to response format
        ################################################
        for i in filtered_interactions:
            response.interactions.append(i.msg)
        return response

    def _running_requirements_are_satisfied(self, interaction):
        """
        Right now we only have running constraints for paired interactions.

        - fail if a rapp is running with a different signature (remappings and parameters considered).
        - fail is no rapp is running and this interaction doesn't control the rapp lifecycle

        This is used when we filter the list to provide to the user as well as when we are requested
        to start an interaction.

        :param interaction: all the details on the interaction we are checking
        :type interaction: rocon_interactions.interactions.Interaction
        :return: true if satisfied, false otherwise
        :rtype: bool
        """
        satisfied = True
        if interaction.pairing.rapp:
            try:
                satisfied = self._rapp_handler.matches_running_rapp(interaction.pairing.rapp, interaction.pairing.remappings, interaction.pairing.parameters)
                if not satisfied:
                    rospy.logdebug("Interactions : '%s' failed to meet runtime requirements [running rapp different to this interaction's pairing rapp signature]" % interaction.display_name)
            except RappNotRunningError:
                # only controlling interactions can move the rapp to start
                satisfied = interaction.pairing.control_rapp_lifecycle
                if not satisfied:
                    rospy.logdebug("Interactions : '%s' failed to meet runtime requirements [rapp is not running and this pairing interaction is not the controlling type]" % interaction.display_name)
        return satisfied

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
            (new_interactions, invalid_interactions) = self._interactions_table.load(request.interactions)
            for i in new_interactions:
                rospy.loginfo("Interactions : loading '%s' [%s-%s-%s]" % (i.display_name, i.name, i.role, i.namespace))
            for i in invalid_interactions:
                rospy.logwarn("Interactions : failed to load %s [%s-%s-%s]" % (i.display_name,
                                                                               i.name,
                                                                               i.role,
                                                                               i.namespace)
                              )
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
            return utils.generate_request_interaction_response(interaction_msgs.ErrorCodes.INTERACTION_UNAVAILABLE)
        if interaction.max != interaction_msgs.Interaction.UNLIMITED_INTERACTIONS:
            count = 0
            for remocon_monitor in self._remocon_monitors.values():
                if remocon_monitor.status is not None and remocon_monitor.status.running_interactions:
                    # Todo this is a weak check as it is not necessarily uniquely identifying the interaction
                    # Todo - reintegrate this using full interaction variable instead
                    pass
                    # if remocon_monitor.status.app_name == request.application:
                    #    count += 1
            if count > interaction.max:
                rospy.loginfo("Interactions : rejected interaction request [interaction quota exceeded]")
                return utils.generate_request_interaction_response(interaction_msgs.ErrorCodes.INTERACTION_QUOTA_REACHED)
        if self.parameters.pairing and interaction.is_paired_type():
            if not self._running_requirements_are_satisfied(interaction):
                if self._rapp_handler.is_running:
                    response = utils.generate_request_interaction_response(interaction_msgs.ErrorCodes.DIFFERENT_RAPP_IS_RUNNING)
                else:
                    response = utils.generate_request_interaction_response(interaction_msgs.ErrorCodes.REQUIRED_RAPP_IS_NOT_RUNNING)
                rospy.logwarn("Interactions : request interaction for '%s' refused [%s]" % (interaction.display_name, response.message))
                return response
            if not self._rapp_handler.is_running and interaction.pairing.control_rapp_lifecycle:
                try:
                    self._rapp_handler.start(interaction.pairing.rapp, interaction.pairing.remappings)
                except FailedToStartRappError as e:
                    rospy.loginfo("Interactions : rejected interaction request [failed to start the paired rapp]")
                    response = utils.generate_request_interaction_response(interaction_msgs.ErrorCodes.START_PAIRED_RAPP_FAILED)
                    response.message = "Failed to start the rapp [%s]" % str(e)  # custom response
                    return response
            # else rapp is running and nothing to do from here, remocon has all the work to do!
            # just list it in our pairs.
            self.runtime_pairing_signatures.append(RuntimePairingSignature(interaction, request.remocon))
            self._ros_publish_pairings()

        # if we get here, we've succeeded.
        return utils.generate_request_interaction_response(interaction_msgs.ErrorCodes.SUCCESS)
