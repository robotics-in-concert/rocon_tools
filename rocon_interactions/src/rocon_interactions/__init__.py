#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
This is the top-level namespace of the rocon_interactions_ ROS
package. It provides python utilities for working with the console.

.. _rocon_interactions: http://wiki.ros.org/rocon_interactions

"""
##############################################################################
# Imports
##############################################################################

from .manager import InteractionsManager
from .loader import InteractionsLoader
from .interactions_table import InteractionsTable
from .interactions import load_msgs_from_yaml_resource, Interaction
from exceptions import *

import web_interactions  # the library functions
#from .web_interactions import WebInteraction  # the main class
