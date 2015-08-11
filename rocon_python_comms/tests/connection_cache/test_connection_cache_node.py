#!/usr/bin/env python

""" Testing the find_node function """

from collections import deque

import unittest
import time

import rospy
import rostest
import roslaunch
import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_std_msgs.srv as rocon_std_srvs

class timeout(object):
    """
    Small useful timeout class
    """
    def __init__(self, seconds):
        self.seconds = seconds
    def __enter__(self):
        self.die_after = time.time() + self.seconds
        return self
    def __exit__(self, type, value, traceback):
        pass
    @property
    def timed_out(self):
        return time.time() > self.die_after


class TestConnectionCacheNode(unittest.TestCase):

    def _list_cb(self, data):
        self.conn_list_msgq.append(data)
        pass

    def _diff_cb(self, data):
        self.conn_diff_msgq.append(data)
        pass

    def setUp(self):
        #Init the test node
        rospy.init_node('test_node')

        # We prepare our data structure for checking messages
        self.conn_list_msgq = deque()
        self.conn_diff_msgq = deque()

        # Then we hookup to its topics and prepare a service proxy
        self.conn_list = rospy.Subscriber('/connection_cache/list', rocon_std_msgs.ConnectionsList, self._list_cb)
        self.conn_diff = rospy.Subscriber('/connection_cache/diff', rocon_std_msgs.ConnectionsDiff, self._diff_cb)
        self.spin_rate = rospy.ServiceProxy('/connection_cache/spin_rate', rocon_std_srvs.SpinRate)

        # no need to spin
        pass

    def tearDown(self):
        pass

    def chatter_detected(self, topicq_clist, conn_type, node_name):
        """
        detect the chatter publisher in a connection list
        """
        test = False
        for i, conn in enumerate(topicq_clist):  # lop through all connections in the list
            test = (conn.name == '/chatter'
                    and conn.type == conn_type
                    and conn.node.startswith(node_name)  # sometime the node gets suffixes with uuid ??
                    and conn.type_info == ''
                    and conn.xmlrpc_uri == '')
            if test:  # break right away if found
                break
        if not test:
            print "Expected : name:{name} type:{type} node:{node} type_info:{type_info} xmlrpc_uri:{xmlrpc_uri}".format(name='/chatter', type=conn_type, node=node_name, type_info='', xmlrpc_uri='')
            print "NOT FOUND IN LIST : {0}".format(topicq_clist)
        return test

    def test_detect_publisher_added_lost(self):
        # Start a dummy node
        talker_node = roslaunch.core.Node('roscpp_tutorials', 'talker')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(talker_node)

        added_publisher_detected = {'list': False, 'diff': False}

        # Loop a bit so we can detect the topic
        with timeout(5) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not added_publisher_detected['list'] and self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.PUBLISHER, '/talker'):  # if we find it
                    added_publisher_detected['list'] = True

                if not added_publisher_detected['diff'] and self.conn_diff_msgq and self.chatter_detected(self.conn_diff_msgq[-1].added, rocon_python_comms.PUBLISHER, '/talker'):  # if we find it
                    added_publisher_detected['diff'] = True

                if added_publisher_detected['list'] and added_publisher_detected['diff']:
                    break
                time.sleep(0.2)

        assert added_publisher_detected['list'] and added_publisher_detected['diff']
        process.stop()

        lost_publisher_detected = {'list': False, 'diff': False}

        # Loop a bit so we can detect the topic is gone
        with timeout(5) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not lost_publisher_detected['list'] and self.conn_list_msgq and not self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.PUBLISHER, '/talker'):  # if we DONT find it
                    lost_publisher_detected['list'] = True

                if not lost_publisher_detected['diff'] and self.conn_diff_msgq and self.chatter_detected(self.conn_diff_msgq[-1].lost, rocon_python_comms.PUBLISHER, '/talker'):  # if we find it
                    lost_publisher_detected['diff'] = True

                if lost_publisher_detected['list'] and lost_publisher_detected['diff']:
                    break
                time.sleep(0.2)

        assert lost_publisher_detected['list'] and lost_publisher_detected['diff']


    def test_detect_subscriber_added_lost(self):
        # Start a dummy node
        listener_node = roslaunch.core.Node('roscpp_tutorials', 'listener')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(listener_node)

        added_subscriber_detected = {'list': False, 'diff': False}

        # Loop a bit so we can detect the topic
        with timeout(5) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not added_subscriber_detected['list'] and self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    added_subscriber_detected['list'] = True

                if not added_subscriber_detected['diff'] and self.conn_diff_msgq and self.chatter_detected(self.conn_diff_msgq[-1].added, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    added_subscriber_detected['diff'] = True

                if added_subscriber_detected['list'] and added_subscriber_detected['diff']:
                    break
                time.sleep(0.2)

        assert added_subscriber_detected['list'] and added_subscriber_detected['diff']
        process.stop()

        lost_subscriber_detected = {'list': False, 'diff': False}

        # Loop a bit so we can detect the topic is gone
        with timeout(5) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not lost_subscriber_detected['list'] and self.conn_list_msgq and not self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we DONT find it
                    lost_subscriber_detected['list'] = True

                if not lost_subscriber_detected['diff'] and self.conn_diff_msgq and self.chatter_detected(self.conn_diff_msgq[-1].lost, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    lost_subscriber_detected['diff'] = True

                if lost_subscriber_detected['list'] and lost_subscriber_detected['diff']:
                    break
                time.sleep(0.2)

        assert lost_subscriber_detected['list'] and lost_subscriber_detected['diff']


    def test_detect_service(self):
        pass

    # TODO : detect actions server and client

    # TODO : use dynamic_reconfigure instead
    def test_change_spin_rate(self):
        pass



if __name__ == '__main__':

    # we don't really need a test file :
    # First we start our node.
    # self.cache_node = roslaunch.core.Node('rocon_python_comms', 'connection_cache')
    # self.launch = roslaunch.scriptapi.ROSLaunch()
    # self.launch.start()
    # self.process = self.launch.launch(self.cache_node)
    # NOT NEEDED : DONE IN ROSTTEST FILE

    rostest.rosrun('rocon_python_comms',
                   'test_connection_cache',
                   TestConnectionCacheNode)
