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

This module provides a class that can be used by other nodes to introspect
and start/stop rapps on a rapp manager running on the same ros master.

"""

##############################################################################
# Imports
##############################################################################

import rospy
import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs

##############################################################################
# Exceptions
##############################################################################


class FailedToStartRappError(Exception):
    """ Failed to start rapp. """
    pass


class FailedToStopRappError(Exception):
    """ Failed to stop rapp. """
    pass


class FailedToFindRappManagerError(Exception):
    """ Failed to find the rapp manager. """
    pass

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
        'namespace',          # conductor's alias for this concert client
        'start_rapp',         # service proxy to this client's start_app service
        'stop_rapp',          # service proxy to this client's stop_app service
        'status_subscriber',  # look for status updates (particularly rapp running/not running)
        'is_running',         # flag indicating present running status of the rapp manager.
        'status_callback',    # function that handles toggling of pairing mode when a running rapp stops.
    ]

    def __init__(self, status_callback):
        """
        Initialise the class with the relevant data required to start and stop
        rapps on this concert client.

        :param status_callback function: handles toggling of pairing mode upon appropriate status updates

        :raises: :exc:`.FailedToFindRappManagerError`
        """
        self.is_running = False
        self.status_callback = status_callback
        self.start_rapp = rospy.ServiceProxy('start_rapp', rocon_app_manager_srvs.StartRapp)
        self.stop_rapp = rospy.ServiceProxy('stop_rapp', rocon_app_manager_srvs.StopRapp)
        self.status_subscriber = rospy.Subscriber('status', rocon_app_manager_msgs.Status, self._ros_status_subscriber)

        try:
            self.start_rapp.wait_for_service(15.0)
            self.stop_rapp.wait_for_service(15.0)
            # I should also check the subscriber has get_num_connections > 0 here
            # (need to create a wait_for_publisher in rocon_python_comms)
        except rospy.ROSException:
            raise FailedToFindRappManagerError("couldn't find the rapp manager (start_rapp/stop_rapp remapped?)")
        except rospy.ROSInterruptException:
            pass  # ros is shutting down.

    def start(self, rapp, remappings):
        """
        Start the rapp with the specified remappings.

        :param rapp str: name of the rapp to start (e.g. rocon_apps/teleop)
        :param remappings rocon_std_msgs/Remapping[]: remappings to apply to the rapp when starting.

        :raises: :exc:`.FailedToStartRappError`
        """
        try:
            self.start_rapp(rocon_app_manager_srvs.StartRappRequest(name=rapp, remappings=remappings))
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStartRappError("%s" % str(e))

    def stop(self):
        """
        Stop a rapp on this concert client (if one should be running). This
        doesn't need a rapp specification since only one rapp can ever be
        running - it will just stop the currently running rapp.
        """
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
