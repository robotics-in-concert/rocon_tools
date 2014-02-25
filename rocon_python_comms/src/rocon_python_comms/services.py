#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import time
from rosservice import rosservice_find

# Local imports
from .exceptions import NotFoundException

##############################################################################
# Find topics/services
##############################################################################


def find_unique_service(service_type, timeout=rospy.rostime.Duration(5.0)):
    '''
      Do a lookup to find the unique service in the master table of type
      specified. This will raise exceptions if it times out or returns
      multiple values.

      @param service_type : service type specification, e.g. concert_msgs/GetInteractions
      @type str

      @param timeout : raise an exception if nothing is found before this timeout occurs.
      @type rospy.rostime.Duration

      @return service_name : the fully resolved name of the service
      @type str

      @raise rocon_python_comms.NotFoundException
    '''
    service_name = None
    timeout_time = time.time() + timeout.to_sec()
    while not rospy.is_shutdown() and time.time() < timeout_time and not service_name:
        service_names = rosservice_find(service_type)
        if len(service_names) > 1:
            raise NotFoundException("multiple services found %s." % service_names)
        elif len(service_names) == 1:
            service_name = service_names[0]
        else:
            rospy.rostime.wallsleep(0.1)
    if service_name is None:
        raise NotFoundException("timed out")
    return service_name
