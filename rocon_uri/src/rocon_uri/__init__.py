#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
This is the top-level namespace of the rocon_uri_ ROS
package. It provides parsing and formatting rules for uri's that
describe rocon devices and robots as resources in the rocon framework.

.. _rocon_uri: http://wiki.ros.org/rocon_uri

"""
##############################################################################
# Imports
##############################################################################

default_uri_string = 'rocon://'
from .exceptions import RoconURIValueError
from uri import parse, is_compatible, RoconURI
