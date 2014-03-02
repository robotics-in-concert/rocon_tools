#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# RosTest
##############################################################################

""" Test loading of interactions to the interactions manager. """

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rostest
import rosunit
import rospy
import rocon_console.console as console
import rocon_interaction_msgs.srv as interaction_srvs
import rocon_uri
import rocon_interactions

##############################################################################
# Imports
##############################################################################

class TestLoader(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_loader")

    def test_loader(self):
        """ Loading... """
        try:
            rospy.wait_for_service('~get_interactions', 3.0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            self.fail("Failed to find %s" % rospy.resolve_name('~get_interactions'))
        get_interactions = rospy.ServiceProxy('~get_interactions', interaction_srvs.GetInteractions)
        request = interaction_srvs.GetInteractionsRequest(roles=[], uri=rocon_uri.default_uri_string)
        interactions_table = None
        while not rospy.is_shutdown() and not interactions_table:
            response = get_interactions(request)
            if response.interactions:
                interactions_table = rocon_interactions.InteractionsTable()
                interactions_table.load(response.interactions)
            else:
                rospy.rostime.wallsleep(0.1)
        print("%s" % interactions_table)
        roles = interactions_table.roles()
        self.assertEqual(roles, ['Rqt', 'PyQt'], 'roles of the interaction table did not return as expected [%s][%s]' % (roles, ['Rqt', 'PyQt']))
        self.assertEqual(len(interactions_table), 4, 'number of interactions incorrect [%s][%s]' % (len(interactions_table), 4))

    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('rocon_interactions', 'loader', TestLoader)
