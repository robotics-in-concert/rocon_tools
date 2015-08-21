#
# License: BSD
#   https://raw.github.com/robotics-in-py/rocon_app_platform/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_console.console as console

###############################################################################
# Functions
###############################################################################


class Parameters:
    """
    The variables of this class are default constructed from parameters on the
    ros parameter server. Each parameter is nested in the private namespace of
    the node which instantiates this class.

    :ivar interactions: load up these .interactions `resource names`_ (e.g. 'rocon_interactions/pairing') *[[]]*
    :vartype interactions: [ str ]
    :ivar pairing: work together with a rapp manager to enable pairing interactions *[False]*
    :vartype pairing: bool
    :ivar bindings: bindings that are substituted into yaml loaded interactions
    :vartype bindings: rocon_interactions.ros_parameters.Bindings

    .. _resource names: http://wiki.ros.org/Names#Package_Resource_Names
    """
    def __init__(self):
        # see sphinx docs above for more detailed explanations of each parameter
        self.interactions = rospy.get_param('~interactions', [])
        self.pairing = rospy.get_param('~pairing', False)

    def __str__(self):
        s = console.bold + "\nParameters:\n" + console.reset
        for key in sorted(self.__dict__):
            s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (self.__dict__[key] if self.__dict__[key] is not None else '-')
        s += console.reset
        return s
