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
from rocon_app_manager_msgs.srv import (
    StartRapp, StartRappRequest, StopRapp, StopRappRequest)

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
        'namespace',           # conductor's alias for this concert client
        'start_rapp',     # service proxy to this client's start_app service
        'stop_rapp',       # service proxy to this client's stop_app service
    ]

    def __init__(self):
        """
        Initialise the class with the relevant data required to start and stop
        rapps on this concert client.

        :param rapp_manager_namespace str: forms the root of the start/stop app service proxies.

        :raises: :exc:`.FailedToFindRappManagerError`
        """
        self.start_rapp = rospy.ServiceProxy('start_rapp', StartRapp)
        self.stop_rapp = rospy.ServiceProxy('stop_rapp', StopRapp)
        try:
            self.start_rapp.wait_for_service(15.0)
            self.stop_rapp.wait_for_service(15.0)
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
            self.start_rapp(StartRappRequest(name=rapp, remappings=remappings))
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
            self.stop_rapp(StopRappRequest())
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStopRappError("%s" % str(e))
