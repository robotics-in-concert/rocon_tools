#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import time
import rostopic

from .exceptions import NotFoundException

##############################################################################
# Find topics/services
##############################################################################


def find_topic(topic_type, timeout=rospy.rostime.Duration(5.0), unique=False):
    '''
      Do a lookup to find topics of the type
      specified. This will raise exceptions if it times out or returns
      multiple values. It can apply the additional logic of whether this should
      return a single unique result, or a list.

      @param topic_type : topic type specification, e.g. rocon_std_msgs/MasterInfo
      @type str

      @param timeout : raise an exception if nothing is found before this timeout occurs.
      @type rospy.rostime.Duration

      @param unique : flag to select the lookup behaviour (single/multiple results)
      @type bool

      @return the fully resolved name of the topic (unique) or list of names (non-unique)
      @type str

      @raise rocon_python_comms.NotFoundException if no topic is found within the timeout
    '''
    topic_name = None
    topic_names = []
    timeout_time = time.time() + timeout.to_sec()
    while not rospy.is_shutdown() and time.time() < timeout_time and not topic_names:
        try:
            topic_names = rostopic.find_by_type(topic_type)
        except rostopic.ROSTopicException:
            raise NotFoundException("ros shutdown")
        if unique:
            if len(topic_names) > 1:
                raise NotFoundException("multiple topics found %s." % topic_names)
            elif len(topic_names) == 1:
                topic_name = topic_names[0]
        if not topic_names:
            rospy.rostime.wallsleep(0.1)
    if not topic_names:
        raise NotFoundException("timed out")
    return topic_name if topic_name else topic_names
