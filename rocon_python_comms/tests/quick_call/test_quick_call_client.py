#!/usr/bin/env python

""" Testing the service pair client """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rospy
import rocon_service_pair_msgs.msg as rocon_service_pair_msgs
import rocon_python_comms
import rostest
import unique_id
import threading


def generate_request_message():
    request = rocon_service_pair_msgs.TestiesRequest()
    request.data = "hello dude"
    return request

class TestServicePairClient(unittest.TestCase):
    '''
      Make timeouts of 3.0 here (c.f. the 2.0s sleep in the server)
      so that the first gets processed in time, but holds up the second. Note that the
      publisher callbacks that queue up get processed serially, not in parallel so that
      the second callback doesn't get done in time before timing out.
      
      If you make them both 2.0, then neither gets done in time.
    '''

    def __init__(self, *args):
        super(TestServicePairClient, self).__init__(*args)
        self.testies = rocon_python_comms.ServicePairClient('testies', rocon_service_pair_msgs.TestiesPair)
        self.response_message = "I heard ya dude"
        rospy.sleep(0.5)  # rospy hack to give publishers time to setup
        
    def test_quick_call(self):
        thread = threading.Thread(target=self.send_test)
        thread.start()
        response = self.testies(generate_request_message(), timeout=rospy.Duration(3.0))
        print("******* Response: %s" % response)
        self.assertIsNotNone(response, "Response from the server is an invalid 'None'")
        self.assertEquals(response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, response.data))

    def send_test(self):
        response = self.testies(generate_request_message(), timeout=rospy.Duration(3.0))
        print("******* Response: %s" % response)
        self.assertIsNotNone(response, "Response from the server is an invalid 'None'")
        self.assertEquals(response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, response.data))
        print("******* Threaded response: %s" % response)
        
    def error_callback(self, error_message):
        """ User callback to pick up error messages. """

if __name__ == '__main__':
    rospy.init_node("test_service_proxy")
    rostest.rosrun('rocon_python_comms',
                   'test_service_pair_client',
                   TestServicePairClient) 