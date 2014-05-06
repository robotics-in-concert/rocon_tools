#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: services
   :platform: Unix
   :synopsis: Useful methods relating to ros services.


This module contains anything relating to introspection or manipulation
of ros services.

----

"""
##############################################################################
# Imports
##############################################################################

import rospy
import time
from rosservice import rosservice_find, ROSServiceIOException

# Local imports
from .exceptions import NotFoundException

##############################################################################
# Find topics/services
##############################################################################


def find_service(service_type, timeout=rospy.rostime.Duration(5.0), unique=False):
    '''
    Do a lookup to find services of the type
    specified. This will raise exceptions if it times out or returns
    multiple values. It can apply the additional logic of whether this should
    return a single unique result, or a list. Under the hood this calls out to the ros master for a list
    of registered services and it parses that to determine the result. If nothing
    is found, it loops around internally on a 10Hz loop until the result is
    found or the specified timeout is reached.

    Usage:

    .. code-block:: python

        from rocon_python_comms import find_service

        try:
            service_name = rocon_python_comms.find_service('rocon_interaction_msgs/SetInteractions',
                                                           timeout=rospy.rostime.Duration(15.0),
                                                           unique=True)
        except rocon_python_comms.NotFoundException as e:
            rospy.logwarn("failed to find the set_interactions service.")

    :param str service_type: service type specification, e.g. concert_msgs/GetInteractions
    :param rospy.Duration timeout: raise an exception if nothing is found before this timeout occurs.
    :param bool unique: flag to select the lookup behaviour (single/multiple results)

    :returns: the fully resolved name of the service (unique) or list of names (non-unique)
    :rtype: str

    :raises: :exc:`.NotFoundException`
    '''
    service_name = None
    service_names = []
    timeout_time = time.time() + timeout.to_sec()
    while not rospy.is_shutdown() and time.time() < timeout_time and not service_names:
        try:
            service_names = rosservice_find(service_type)
        except ROSServiceIOException as e:
            raise NotFoundException(str(e))
        if unique:
            if len(service_names) > 1:
                raise NotFoundException("multiple services found %s." % service_names)
            elif len(service_names) == 1:
                service_name = service_names[0]
        if not service_names:
            rospy.rostime.wallsleep(0.1)
    if not service_names:
        raise NotFoundException("timed out")
    return service_name if service_name else service_names
