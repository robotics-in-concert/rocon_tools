#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Module
##############################################################################

"""
.. module:: rapp_handler
   :platform: Unix
   :synopsis: Works with the rocon app manager to support pairing interactions.

This module provides a class that can be used by other nodes to introspect
and start/stop rapps on a rapp manager running on the same ros master.

"""

##############################################################################
# Imports
##############################################################################

import rospy
import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs
import rocon_python_comms
import threading
from .exceptions import(
                        FailedToStartRappError,
                        FailedToStopRappError,
                        )

##############################################################################
# Classes
##############################################################################


class RappHandler(object):
    """
    Initialises from a conductor message detailing information about a
    concert client. Once established, this instance can be used as
    a convenience to start and stop rapps on the concert client.
    """
    __slots__ = [
        'start_rapp',         # service proxy to this client's start_app service
        'stop_rapp',          # service proxy to this client's stop_app service
        'status_subscriber',  # look for status updates (particularly rapp running/not running)
        'is_running',         # flag indicating present running status of the rapp manager.
        'status_callback',    # function that handles toggling of pairing mode when a running rapp stops.
        'initialised',        # flag indicating whether the rapp manager is ready or not.
    ]

    def __init__(self, status_callback):
        """
        Initialise the class with the relevant data required to start and stop
        rapps on this concert client.

        :param status_callback function: handles toggling of pairing mode upon appropriate status updates
        """
        self.is_running = False
        """Flag indicating if there is a monitored rapp running on the rapp manager."""
        self.status_callback = status_callback
        """Callback that handles status updates of the rapp manager appropriately at a higher level (the interactions manager level)."""
        self.initialised = False
        """Flag indicating that the rapp manager has been found and services/topics connected."""
        self.start_rapp = None
        """Service proxy to the rapp manager's start_rapp service"""
        self.stop_rapp = None
        """Service proxy to the rapp manager's stop_rapp service"""
        self.status_subscriber = None
        """Subscriber to the rapp manager's status publisher"""

        thread = threading.Thread(target=self._setup_rapp_manager_connections())
        thread.start()

    def _setup_rapp_manager_connections(self):
        try:
            start_rapp_service_name = rocon_python_comms.find_service('rocon_app_manager_msgs/StartRapp', timeout=rospy.rostime.Duration(60.0), unique=True)
            stop_rapp_service_name = rocon_python_comms.find_service('rocon_app_manager_msgs/StopRapp', timeout=rospy.rostime.Duration(60.0), unique=True)
            status_topic_name = rocon_python_comms.find_topic('rocon_app_manager_msgs/Status', timeout=rospy.rostime.Duration(60.0), unique=True)
        except rocon_python_comms.NotFoundException as e:
            rospy.logerr("Interactions : timed out trying to find the rapp manager start_rapp, stop_rapp services and status topic [%s]" % str(e))

        self.start_rapp = rospy.ServiceProxy(start_rapp_service_name, rocon_app_manager_srvs.StartRapp)
        self.stop_rapp = rospy.ServiceProxy(stop_rapp_service_name, rocon_app_manager_srvs.StopRapp)
        self.status_subscriber = rospy.Subscriber(status_topic_name, rocon_app_manager_msgs.Status, self._ros_status_subscriber)

        try:
            self.start_rapp.wait_for_service(15.0)
            self.stop_rapp.wait_for_service(15.0)
            # I should also check the subscriber has get_num_connections > 0 here
            # (need to create a wait_for_publisher in rocon_python_comms)
            self.initialised = True
            rospy.loginfo("Interactions : initialised rapp handler connections for pairing.")
        except rospy.ROSException:
            rospy.logerr("Interactions : rapp manager services disappeared.")
        except rospy.ROSInterruptException:
            rospy.logerr("Interactions : ros shutdown while looking for the rapp manager services.")

    def start(self, rapp, remappings):
        """
        Start the rapp with the specified remappings.

        :param str rapp: name of the rapp to start (e.g. rocon_apps/teleop)
        :param remappings: remappings to apply to the rapp when starting.
        :type remappings: rocon_std_msgs.Remapping_ []

        .. include:: weblinks.rst

        :raises: :exc:`.FailedToStartRappError`
        """
        if not self.initialised:
            raise FailedToStartRappError("rapp manager's location not known")
        try:
            unused_response = self.start_rapp(rocon_app_manager_srvs.StartRappRequest(name=rapp, remappings=remappings))
            # todo check this response and process it
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStartRappError("%s" % str(e))

    def stop(self):
        """
        Stop a rapp on this concert client (if one should be running). This
        doesn't need a rapp specification since only one rapp can ever be
        running - it will just stop the currently running rapp.

        :raises: :exc:`.FailedToStopRappError`
        """
        if not self.initialised:
            raise FailedToStopRappError("rapp manager's location not known")
        try:
            self.stop_rapp(rocon_app_manager_srvs.StopRappRequest())
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStopRappError("%s" % str(e))

    def _ros_status_subscriber(self, msg):
        """
        Relay a notification to the status callback function if we detect that a rapp has
        stopped on the rapp manager. This is to let the higher level disable pairing mode if
        it is the rapp manager's rapp naturally terminating rather than the user terminating
        from the user's side.
        """
        old_running_status = self.is_running
        self.is_running = (msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_RUNNING)
        if old_running_status and msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_STOPPED:
            self.status_callback()  # let the higher level disable pairing mode via this
