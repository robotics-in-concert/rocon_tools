#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import threading

import collections

import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_std_msgs.srv as rocon_std_srvs
import rospy

##############################################################################
# Main
##############################################################################


if __name__ == '__main__':
    rospy.init_node('connection_cache')
    conn_cache = rocon_python_comms.ConnectionCacheNode()
    conn_cache.spin()


