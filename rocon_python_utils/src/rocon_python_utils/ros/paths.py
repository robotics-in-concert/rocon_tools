#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: ros.paths
   :platform: Unix
   :synopsis: Helpers for finding and create cache using rocon.

This module contains helpers that find or creat cache using rocon

----

"""

##############################################################################
# Imports
##############################################################################

# system
import os

# ros
import rospkg

##############################################################################
# Paths
##############################################################################

def get_rocon_home():
    '''
      Retrieve the location of the rocon home directory for using rocon compoenets. 
      If it is not existed, create new directory and return this path

      @return the service manager home directory (path object).
      @type str
    '''
    rocon_home = os.path.join(rospkg.get_ros_home(), 'rocon')
    if not os.path.isdir(rocon_home):
        os.makedirs(rocon_home)
    return os.path.join(rospkg.get_ros_home(), 'rocon')