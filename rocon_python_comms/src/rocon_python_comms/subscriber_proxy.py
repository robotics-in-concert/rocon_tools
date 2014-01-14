#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import time
import rospy

##############################################################################
# Subscriber Proxy
##############################################################################

class SubscriberProxy():
    '''
      Works like a service proxy, but using a subscriber instead.
    '''
    def __init__(self, topic, msg_type):
        '''
          @param topic : the topic name to subscriber to
          @type str
          @param msg_type : any ros message type (typical arg for the subscriber)
          @type msg
          @param timeout : timeout on the wait operation (None = /infty)
          @type rospy.Duration()
          @return msg type data or None
        '''
        self._subscriber = rospy.Subscriber(topic, msg_type, self._callback)
        self._data = None

    def __call__(self, timeout=None):
        '''
          Returns immediately with the latest data or waits for
          incoming data.

          @param timeout : time to wait for data, polling at 10Hz.
          @type rospy.Duration
          @return latest data or None
        '''
        if timeout:
            # everything in floating point calculations
            timeout_time = time.time() + timeout.to_sec()
        while not rospy.is_shutdown() and self._data == None:
            rospy.rostime.wallsleep(0.1)
            if timeout:
                if time.time() > timeout_time:
                    return None
        return self._data

    def wait_for_next(self, timeout=None):
        '''
          Makes sure any current data is cleared and waits for new data.
        '''
        self._data = None
        return self.__call__(timeout)

    def wait_for_publishers(self):
        '''
          Blocks until publishers are seen.

          @raise rospy.exceptions.ROSInterruptException if we are in shutdown.
        '''
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._subscriber.get_num_connections() != 0:
                return
            else:
                r.sleep()
        # we are shutting down
        raise rospy.exceptions.ROSInterruptException

    def _callback(self, data):
        self._data = data

    def unregister(self):
        '''
          Unregister the subscriber so future instantiations of this class can pull a
          fresh subscriber (important if the data is latched).
        '''
        self._subscriber.unregister()
