#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import pyros_setup
pyros_setup.delayed_import()
import rospy
import rocon_python_comms


##############################################################################
# Main
##############################################################################


if __name__ == '__main__':
    rospy.init_node('connection_cache')
    conn_cache = rocon_python_comms.ConnectionCacheNode()
    conn_cache.spin()


