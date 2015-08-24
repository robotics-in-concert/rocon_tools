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
from .exceptions import MalformedInteractionsYaml, YamlResourceNotFoundException
from .interactions_table import InteractionsTable
from .pairings_table import PairingsTable
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

      Currently assumes static configuration, i.e. load everything from yaml at
      startup.

      To upgrade for dynamic configuration, i.e. load from ros api, then you'll
      need to touch a bit of the logic herein.

      - set interactions service call
      - don't prefilter interactions at startup
      - instead prefilter them on get_interactions requests
    '''

    ##########################################################################
    # Initialisation & Execution
    ##########################################################################

    def __init__(self):
        self._watch_loop_period = 1.0
        self._remocon_monitors = {}     # topic_name : RemoconMonitor
        self.parameters = Parameters()  # important to come first since we use self.parameters.pairing everywhere
        self.active_pairing = None

        self._rapp_handler = RappHandler(self._rapp_changed_state_callback) if self.parameters.pairing else None
        self.interactions_table = InteractionsTable(filter_pairing_interactions=not self.parameters.pairing)
        self.pairings_table = PairingsTable()

        #############################################
        # Load pairing/interaction msgs from yaml
        #############################################
        all_pairings = []
        all_interactions = []
        for resource_name in self.parameters.interactions:
            try:
                (pairings, interactions) = utils.load_msgs_from_yaml_resource(resource_name)
            except YamlResourceNotFoundException as e:
                rospy.logerr("Interactions : failed to load resource %s [%s]" %
                             (resource_name, str(e)))
            except MalformedInteractionsYaml as e:
                rospy.logerr("Interactions : pre-configured interactions yaml malformed [%s][%s]" %
                             (resource_name, str(e)))
            all_interactions.extend(interactions)
            all_pairings.extend(pairings)

        #############################################
        # Filter pairings w/o rapps & Load
        #############################################
        available_pairings = []
        for p in all_pairings:
            rapp = self._rapp_handler.get_rapp(p.rapp)
            if rapp is not None:
                if not p.icon.resource_name:
                    p.icon = rapp["icon"]
                if not p.description:
                    p.description = rapp["description"]
                available_pairings.append(p)
            else:
                rospy.logwarn("Interactions : pairing '%s' requires rapp '%s', but it is unavailable." % (p.name, p.rapp))
        self.pairings_table.load(available_pairings)
        #############################################
        # Filter interactions w/o pairings & Load
        #############################################
        available_interactions = []
        for i in all_interactions:
            valid = True
            for required_pairing in i.required_pairings:
                if not self.pairings_table.is_available_pairing(required_pairing):
                    rospy.logwarn("Interactions : interaction '%s' requires pairing '%s' but it is unavailable." % (i.name, required_pairing))
                    valid = False
                    continue
            if valid:
                available_interactions.append(i)
        self.interactions_table.load(available_interactions)

        # ros communications
        self.services = rocon_python_comms.utils.Services(
            [
                ('~get_interaction', interaction_srvs.GetInteraction, self._ros_service_get_interaction),
                ('~get_interactions', interaction_srvs.GetInteractions, self._ros_service_get_interactions),
                ('~get_pairings', interaction_srvs.GetPairings, self._ros_service_get_pairings),
                ('~request_interaction', interaction_srvs.RequestInteraction, self._ros_service_request_interaction),
                ('~start_pairing', interaction_srvs.StartPairing, self._ros_service_start_pairing),
                ('~stop_pairing', interaction_srvs.StopPairing, self._ros_service_stop_pairing),
            ]
        )
        latched = True
        queue_size_five = 5
        self.publishers = rocon_python_comms.utils.Publishers(
            [
                ('~parameters', std_msgs.String, latched, queue_size_five),
                ('~interactive_clients', interaction_msgs.InteractiveClients, latched, queue_size_five),
                ('~pairing_status', interaction_msgs.PairingStatus, latched, queue_size_five),
                # ('~introspection/pairings', std_msgs.String, latched, queue_size_five)  # for debugging, show all pairings
            ]
        )
        # small pause (convenience only) to let connections to come up
        rospy.rostime.wallsleep(0.5)
        self.publishers.parameters.publish(std_msgs.String("%s" % self.parameters))
        self.publishers.pairing_status.publish(interaction_msgs.PairingStatus())

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
#             for signature in self.runtime_pairing_signatures:
#                 if signature.interaction.hash in finished_interactions \
#                         and signature.remocon_name == remocon_unique_name:
#                     to_be_removed_signature = signature
#                     break
#             if to_be_removed_signature is not None:
#                 self.runtime_pairing_signatures.remove(to_be_removed_signature)
#                 self._ros_publish_pairings()
#                 if not self.runtime_pairing_signatures:
#                     try:
#                         self._rapp_handler.stop()
#                     except FailedToStopRappError as e:
#                         rospy.logerr("Interactions : failed to stop a paired rapp [%s]" % e)
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
        msg = interaction_msgs.PairingStatus()
        if stopped:
            self.active_pairing = None
        elif self.active_pairing:  # we set active pairing when we start a rapp
            msg.active_pairing = self.active_pairing.name
        self.publishers.pairing_status.publish(msg)

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
                    interaction = self.interactions_table.find(interaction_hash)
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
        interaction = self.interactions_table.find(request.hash)
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
        if request.groups:  # works for None or empty list
            unavailable_groups = [x for x in request.groups if x not in self.interactions_table.groups()]
            for role in unavailable_groups:
                rospy.logerr("Interactions : received request for interactions of an unregistered role [%s]" % role)

        uri = request.uri if request.uri != '' else 'rocon:/'
        try:
            filtered_interactions = self.interactions_table.filter(request.groups, uri)
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

    def _ros_service_get_pairings(self, request):
        '''
          Handle incoming requests to provide the pairings list.

          @param request
          @type concert_srvs.GetPairingsRequest
        '''
        response = interaction_srvs.GetPairingsResponse()
        response.pairings = []
        for p in self.pairings_table.pairings:
            response.pairings.append(p.msg)
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

    def _ros_service_request_interaction(self, request):
        interaction = self.interactions_table.find(request.hash)
        # for interaction in self.interactions_table.interactions:
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

    def _ros_service_start_pairing(self, request):
        print("Got a request to start pairing [%s]" % request.name)
        if not self.parameters.pairing:
            return interaction_srvs.StartPairingResponse(interaction_msgs.ErrorCodes.NOT_PAIRING, interaction_msgs.ErrorCodes.MSG_NOT_PAIRING)
        if self._rapp_handler.is_running:
            return interaction_srvs.StartPairingResponse(interaction_msgs.ErrorCodes.ALREADY_PAIRING, interaction_msgs.ErrorCodes.MSG_ALREADY_PAIRING)
        try:
            print("Looking for pairing: %s" % request.name)
            pairing = self.pairings_table.find(request.name)
            self.active_pairing = pairing
            self._rapp_handler.start(pairing.rapp, pairing.remappings)
            return interaction_srvs.StartPairingResponse(interaction_msgs.ErrorCodes.SUCCESS, "firing up.")
        except FailedToStartRappError as e:
            rospy.loginfo("Interactions : rejected interaction request [failed to start the paired rapp]")
            response = interaction_srvs.StartPairingResponse()
            response.result = interaction_msgs.ErrorCodes.START_PAIRING_FAILED
            response.message = "failed to start the pairing [%s]" % str(e)  # custom response
            return response

    def _ros_service_stop_pairing(self, request):
        print("Got a request to stop pairing [%s]" % request.name)
        try:
            self._rapp_handler.stop()
            return interaction_srvs.StopPairingResponse(result=interaction_msgs.ErrorCodes.SUCCESS, message="stopping.")
        except FailedToStartRappError as e:
            rospy.loginfo("Interactions : rejected interaction request [failed to start the paired rapp]")
            response = interaction_srvs.StopPairingResponse()
            response.result = interaction_msgs.ErrorCodes.STOP_PAIRING_FAILED
            response.message = "failed to stop the pairing [%s]" % str(e)  # custom response
            return response
