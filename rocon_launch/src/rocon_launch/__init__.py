#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
This is the top-level namespace of the rocon_launch_ ROS
package. It provides machinery for spawning multiple roslaunch environments.

.. _rocon_launch: http://wiki.ros.org/rocon_launch

"""
##############################################################################
# Imports
##############################################################################

from .launch import main as launch, parse_rocon_launcher, get_roslaunch_pids
