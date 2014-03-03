#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
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
      return a single unique result, or a list.

      @param service_type : service type specification, e.g. concert_msgs/GetInteractions
      @type str

      @param timeout : raise an exception if nothing is found before this timeout occurs.
      @type rospy.rostime.Duration

      @param unique : flag to select the lookup behaviour (single/multiple results)
      @type bool

      @return the fully resolved name of the service (unique) or list of names (non-unique)
      @type str

      @raise rocon_python_comms.NotFoundException
    '''
    service_name = None
    service_names = []
    timeout_time = time.time() + timeout.to_sec()
    while not rospy.is_shutdown() and time.time() < timeout_time and not service_names:
        try:
            service_names = rosservice_find(service_type)
        except ROSServiceIOException:
            raise NotFoundException("ros shutdown")
        if unique:
            if len(service_names) > 1:
                raise NotFoundException("multiple services found %s." % service_names)
            elif len(service_names) == 1:
                service_name = service_names[0]
        if not service_names:
            rospy.rostime.wallsleep(0.1)
    if service_name is None:
        raise NotFoundException("timed out")
    return service_name
