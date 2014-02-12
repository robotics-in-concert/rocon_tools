#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import rospy
import unique_id
import rocon_service_pair_msgs.msg as rocon_service_pair_msgs 
import rocon_python_comms
import threading

##############################################################################
# Classes
##############################################################################

class Jagi(object):
    __slots__ = [
        'server',
        'requests',
        'lock',
    ]

    def __init__(self):
        self.server = rocon_python_comms.ServicePairServer('testies', self.callback, rocon_service_pair_msgs.TestiesPair)
        self.requests = []

    def callback(self, request_id, msg):
        '''
          @param request_id
          @type uuid_msgs/UniqueID
          @param msg
          @type ServiceRequest
        '''
        #rospy.loginfo("received '%s'" % msg.data)
        rospy.logwarn("Jagi: received [%s][%s]" % (msg.data, unique_id.toHexString(request_id)))
        response = rocon_service_pair_msgs.TestiesResponse()
        response.data = "I heard ya dude" 
        rospy.sleep(2.0)
        self.server.reply(request_id, response)
    

##############################################################################
# Main
##############################################################################
    

if __name__ == '__main__':
    
    rospy.init_node('example_rocon_pair_server')
    jagi = Jagi()
    rospy.spin()
