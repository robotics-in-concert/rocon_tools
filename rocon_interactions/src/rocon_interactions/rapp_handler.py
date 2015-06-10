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
                        FailedToListRappsError,
                        )
from .rapp_watcher import RappWatcher

##############################################################################
# Classes
##############################################################################

#TODO : RappHandler needs to be modified to understand namespaces.
# => currently will not work within rocon
class RappHandler(object):
    """
    Initialises from a conductor message detailing information about a
    concert client. Once established, this instance can be used as
    a convenience to start and stop rapps on the concert client.
    """
    __slots__ = [
        'rapp_watcher',       # keep rapps namespaces in check
        'is_running',         # flag indicating present running status of the rapp manager.
        'status_callback',    # function that handles toggling of pairing mode when a running rapp stops.
        'rapp_running_callback',    # function that handles signaling added/removed interactions when rapp starts/stops.
        'initialised',        # flag indicating whether the rapp manager is ready or not.
    ]

    def __init__(self, status_callback, rapp_running_callback):
        """
        Initialise the class with the relevant data required to start and stop
        rapps on this concert client.

        :param status_callback function: handles toggling of pairing mode upon appropriate status updates
        """
        self.is_running = False
        """Flag indicating if there is a monitored rapp running on the rapp manager."""
        self.status_callback = status_callback
        """Callback that handles status updates of the rapp manager appropriately at a higher level (the interactions manager level)."""
        self.rapp_running_callback=rapp_running_callback
        """Callback that handles ignaling the list of available Interactions has changed"""
        self.initialised = False
        """Flag indicating that the rapp manager has been found and services/topics connected."""
        self.rapp_watcher = RappWatcher(self._namespaces_change, self._available_rapps_list_change, self._running_rapp_status_change, self._ros_status_subscriber)
        """Rapp Watcher instance"""
        self.rapp_watcher.start()

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
            unused_response = self.rapp_watcher.start_rapp(None, rocon_app_manager_srvs.StartRappRequest(name=rapp, remappings=remappings))
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
            unused_response = self.rapp_watcher.stop_rapp(None, rocon_app_manager_srvs.StopRappRequest())
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStopRappError("%s" % str(e))

    def get_available_rapps(self):
        return self.rapp_watcher.get_available_rapps(None)

    def is_available_rapp(self, rapp_name):
        if rapp_name in self.get_available_rapps().keys():
            return True
        else:
            return False

    def get_running_rapp(self):
        return self.rapp_watcher.get_running_rapp(None)

    def is_running_rapp(self, rapp_name):
        rapp = self.get_running_rapp()
        if 'name' in rapp.keys():
            return rapp["name"] == rapp_name
        else:
            return False

    def _namespaces_change(self, added_namespaces, removed_namespaces):
        return added_namespaces  # we want to watch every added namespace

    def _available_rapps_list_change(self, namespace, added_available_rapps, removed_available_rapps):
        pass

    def _running_rapp_status_change(self, namespace, rapp_status, rapp):
        self.rapp_running_callback(rapp_status, rapp)

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
