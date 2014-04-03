#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
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
    __slots__ = [
            'publishers',
            'param',
            'spin',
        ]

    def __init__(self):
        ##################################
        # Pubs, Subs and Services
        ##################################
        self.publishers = {}
        # efficient latched publisher, put in the public concert namespace.
        self.param = self._setup_ros_parameters()
        self.publishers["info"] = rospy.Publisher("info", rocon_std_msgs.MasterInfo, latch=True)
        master_info = rocon_std_msgs.MasterInfo()
        master_info.name = self.param['name']
        master_info.description = self.param['description']
        master_info.icon = rocon_python_utils.ros.icon_resource_to_msg(self.param['icon'])
        master_info.version = rocon_std_msgs.Strings.ROCON_VERSION
        self.publishers['info'].publish(master_info)
        # Aliases
        self.spin = rospy.spin

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
