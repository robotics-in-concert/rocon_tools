#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: master
   :platform: Unix
   :synopsis: Advertising info about a ros/rocon master.


This module contains the machinery for advertising basic information about a
ros master.

----

"""
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_python_utils

##############################################################################
# Conductor
##############################################################################


class RoconMaster(object):
    """
    This class accepts a few parameters describing the ros master and then
    publishes the ros master info on a latched publisher. Publishing is necessary
    because an icon can only be represented by it's location as a parameter. It
    is easier directly publishing the icon rather than having clients go do
    the lookup themselves.
    """
    __slots__ = [
            '_publishers',
            '_parameters',
            'spin',
        ]

    def __init__(self):
        '''
        Retrieves ``name``, ``description`` and ``icon`` parameters from the
        parameter server and publishes them on a latched ``info`` topic.
        The icon parameter must be a ros resource name (pkg/filename).
        '''
        ##################################
        # Pubs, Subs and Services
        ##################################
        self._publishers = {}
        # efficient latched publisher, put in the public concert namespace.
        self._parameters = self._setup_ros_parameters()
        self._publishers["info"] = rospy.Publisher("info", rocon_std_msgs.MasterInfo, latch=True, queue_size=1)
        master_info = rocon_std_msgs.MasterInfo()
        master_info.name = self._parameters['name']
        master_info.description = self._parameters['description']
        master_info.icon = rocon_python_utils.ros.icon_resource_to_msg(self._parameters['icon'])
        master_info.version = rocon_std_msgs.Strings.ROCON_VERSION
        self._publishers['info'].publish(master_info)
        # Aliases
        self.spin = rospy.spin
        """Spin function, currently this just replicates the rospy spin function since everything is done in the constructor."""

    def _setup_ros_parameters(self):
        '''
          Parameters that are configurable (overridable) are currently set via args in the
          concert master launcher where they are published as parameters. We grab those here.

          Parameters that are fixed (not configurable), we set here so we can access the message
          string constant and use that (also to avoid roslaunch clutter).
        '''
        param = {}
        param['name'] = rospy.get_param('name', 'Cybernetic Pirate')
        param['icon'] = rospy.get_param('icon', 'rocon_icons/cybernetic_pirate.png')
        param['description'] = rospy.get_param('description', 'A rocon system.')
        # a local version
        rospy.set_param('version', rocon_std_msgs.Strings.ROCON_VERSION)
        # and a global version (useful as a ping to check for a rocon master (e.g. by androids)
        rospy.set_param('/rocon/version', rocon_std_msgs.Strings.ROCON_VERSION)
        return param
