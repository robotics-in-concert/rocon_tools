#!/usr/bin/env python

import multiprocessing
from collections import deque

import os
import sys
import pprint
import subprocess
import unittest
import time
from functools import partial

import pyros_setup

try:
    import rospy
    import rostest
    import roslaunch
    import rosgraph
    import rosnode
    import rocon_python_comms
    import rocon_std_msgs.msg as rocon_std_msgs
except ImportError:
    pyros_setup = pyros_setup.delayed_import()
    import rospy
    import rostest
    import roslaunch
    import rosgraph
    import rosnode
    import rocon_python_comms
    import rocon_std_msgs.msg as rocon_std_msgs


roscore_process = None
master = None
launch = None


def setup_module():
    global master
    global roscore_process
    master, roscore_process = pyros_setup.get_master()
    assert master.is_online()

    global launch
    # initialize the launch environment first
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # Init the test node
    rospy.init_node('test_connection_cache_node')


def teardown_module():
    # finishing all process
    if roscore_process is not None:
        roscore_process.terminate()  # make sure everything is stopped
    rospy.signal_shutdown('test complete')


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

    def _spin_cb(self, data):
        self.spin_freq = data.spin_freq

    def setUp(self, cacheproxy=rocon_python_comms.ConnectionCacheProxy):
        # We prepare our data structure for checking messages
        self.conn_list_msgq = deque()
        self.conn_diff_msgq = deque()
        self.spin_freq = 0.0

        # Then we hookup to its topics and prepare a service proxy
        self.conn_list = rospy.Subscriber('/connection_cache/list', rocon_std_msgs.ConnectionsList, self._list_cb)
        self.conn_diff = rospy.Subscriber('/connection_cache/diff', rocon_std_msgs.ConnectionsDiff, self._diff_cb)
        self.set_spin = rospy.Publisher('/connection_cache/spin', rocon_std_msgs.ConnectionCacheSpin, queue_size=1)
        self.get_spin = rospy.Subscriber('/connection_cache/spin', rocon_std_msgs.ConnectionCacheSpin, self._spin_cb)

        # connecting to the master via proxy object
        self._master = rospy.get_master()

        self.proxy = cacheproxy(
                list_sub='/connection_cache/list',
                diff_sub='/connection_cache/diff'
        )

        # Verify that the Proxy except properly, because the cache node is not started
        with self.assertRaises(rocon_python_comms.UnknownSystemState):
            self.proxy.getSystemState()

        cache_node = roslaunch.core.Node('rocon_python_comms', 'connection_cache.py', name='connection_cache')
        self.cache_process = launch.launch(cache_node)

        node_api = None
        with timeout(5) as t:
            while not t.timed_out and node_api is None:
                node_api = rosnode.get_api_uri(self._master, 'connection_cache')

        assert node_api is not None  # make sure the connection cache node is started before moving on.

        # Making sure we set System state before starting test ( needed to validate diff behavior )
        ssinitdone = False
        with timeout(5) as t:
            while not t.timed_out and not ssinitdone:
                try:
                    self.proxy.getSystemState()
                except rocon_python_comms.UnknownSystemState:
                    ssinitdone = False
                    time.sleep(1)
                else:
                    ssinitdone = True
                    break

        assert not t.timed_out

    def tearDown(self):
        if self.cache_process:
            self.cache_process.stop()
            time.sleep(1)  # to make sure the process has enough time to stop.
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

    def add_two_ints_detected(self, svcq_clist, conn_type, node_name):
        """
        detect the chatter publisher in a connection list
        """
        test = False
        for i, conn in enumerate(svcq_clist):  # lop through all connections in the list
            test = (conn.name == '/add_two_ints'
                    and conn.type == conn_type
                    and conn.node.startswith(node_name)  # sometime the node gets suffixes with uuid ??
                    and conn.type_info == ''
                    and conn.xmlrpc_uri == '')
            if test:  # break right away if found
                break
        if not test:
            print "Expected : name:{name} type:{type} node:{node} type_info:{type_info} xmlrpc_uri:{xmlrpc_uri}".format(name='/add_two_ints', type=conn_type, node=node_name, type_info='', xmlrpc_uri='')
            print "NOT FOUND IN LIST : {0}".format(svcq_clist)
        return test

    def equalMasterSystemState(self, proxySS):
        masterSS = self._master.getSystemState()[2]

        same = True
        # masterSS set included in proxySS set
        for idx, t in enumerate(masterSS):  # connection type
            print "MASTER SYSTEM STATE CONNECTION TYPE {0}".format(t)
            print " -> PROXY SYSTEM STATE CONNECTION TYPE {0}".format(proxySS[idx])
            proxySS_names = [c[0] for c in proxySS[idx]]
            for c in t:  # connection list [name, [nodes]]
                print "MASTER SYSTEM STATE CONNECTION {0}".format(c)
                print " -> CHECK {0} in PROXY NAMES {1}".format(c[0], proxySS_names)
                same = same and c[0] in proxySS_names
                if not same:
                    print("ERROR : {0} not in {1}".format(c[0], proxySS_names))
                    break
                proxySS_conn_nodes = [fpc for pc in proxySS[idx] if c[0] == pc[0] for fpc in pc[1]]
                for n in c[1]:
                    print " -> CHECK {0} in PROXY NODES {1}".format(n, proxySS_conn_nodes)
                    same = same and n in proxySS_conn_nodes
                    if not same:
                        print("ERROR : {0} not in {1}".format(n, proxySS_conn_nodes))
                        break

        # proxySS set included in masterSS set
        for idx, t in enumerate(proxySS):  # connection type
            print "PROXY SYSTEM STATE CONNECTION TYPE {0}".format(t)
            print " -> MASTER SYSTEM STATE CONNECTION TYPE {0}".format(masterSS[idx])
            masterSS_names = [c[0] for c in masterSS[idx]]
            for c in t:  # connection list [name, [nodes]]
                print "PROXY SYSTEM STATE CONNECTION {0}".format(c)
                print " -> CHECK {0} in MASTER NAMES {1}".format(c[0], masterSS_names)
                same = same and c[0] in masterSS_names
                if not same:
                    print("ERROR : {0} not in {1}".format(c[0], masterSS_names))
                    break
                masterSS_conn_nodes = [fmc for mc in masterSS[idx] if c[0] == mc[0] for fmc in mc[1]]
                for n in c[1]:
                    print " -> CHECK {0} in MASTER NODES {1}".format(n, masterSS_conn_nodes)
                    same = same and n in masterSS_conn_nodes
                    if not same:
                        print("ERROR : {0} not in {1}".format(n, masterSS_conn_nodes))
                        break
        return same

    def test_detect_publisher_added_lost(self):
        # Start a dummy node
        talker_node = roslaunch.core.Node('roscpp_tutorials', 'talker')
        process = launch.launch(talker_node)
        try:
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

            assert added_publisher_detected['list']
            assert added_publisher_detected['diff']

            time.sleep(0.2)
            # asserting in proxy as well
            assert self.equalMasterSystemState(self.proxy.getSystemState())

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()

            # Start a dummy node to trigger a change
            server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
            distraction_process = launch.launch(server_node)
            try:
                still_publisher_detected = {'list': False}

                # Loop a bit so we can detect the topic
                with timeout(5) as t:
                    while not t.timed_out:
                        # Here we only check the last message received
                        if not still_publisher_detected['list'] and self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.PUBLISHER, '/talker'):  # if we find it
                            still_publisher_detected['list'] = True
                            break
                        time.sleep(0.2)

                assert still_publisher_detected['list']
                time.sleep(0.2)
                # asserting in proxy as well
                assert self.equalMasterSystemState(self.proxy.getSystemState())
            finally:
                distraction_process.stop()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()

        finally:
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

        assert lost_publisher_detected['list']
        assert lost_publisher_detected['diff']
        time.sleep(0.2)
        # asserting in proxy as well
        assert self.equalMasterSystemState(self.proxy.getSystemState())

    def test_detect_subscriber_added_lost(self):
        # Start a dummy node
        listener_node = roslaunch.core.Node('roscpp_tutorials', 'listener')
        process = launch.launch(listener_node)
        try:
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

            assert added_subscriber_detected['list']
            assert added_subscriber_detected['diff']

            # asserting in proxy as well
            time.sleep(0.2)
            assert self.equalMasterSystemState(self.proxy.getSystemState())

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()

            # Start a dummy node to trigger a change
            server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
            distraction_process = launch.launch(server_node)
            try:
                still_subscriber_detected = {'list': False}

                # Loop a bit so we can detect the topic
                with timeout(5) as t:
                    while not t.timed_out:
                        # Here we only check the last message received
                        if not still_subscriber_detected['list'] and self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                            still_subscriber_detected['list'] = True
                            break
                        time.sleep(0.2)

                assert still_subscriber_detected['list']
                time.sleep(0.2)
                # asserting in proxy as well
                assert self.equalMasterSystemState(self.proxy.getSystemState())
            finally:
                distraction_process.stop()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()

        finally:
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

        assert lost_subscriber_detected['list']
        assert lost_subscriber_detected['diff']
        time.sleep(0.2)
        # asserting in proxy as well
        assert self.equalMasterSystemState(self.proxy.getSystemState())

    def test_detect_service_added_lost(self):
        # Start a dummy node
        server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
        process = launch.launch(server_node)
        try:
            added_service_detected = {'list': False, 'diff': False}

            # Loop a bit so we can detect the service
            with timeout(5) as t:
                while not t.timed_out:
                    # Here we only check the last message received
                    if not added_service_detected['list'] and self.conn_list_msgq and self.add_two_ints_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we find it
                        added_service_detected['list'] = True

                    if not added_service_detected['diff'] and self.conn_diff_msgq and self.add_two_ints_detected(self.conn_diff_msgq[-1].added, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we find it
                        added_service_detected['diff'] = True

                    if added_service_detected['list'] and added_service_detected['diff']:
                        break
                    time.sleep(0.2)

            assert added_service_detected['list']
            assert added_service_detected['diff']
            time.sleep(0.2)
            # asserting in proxy as well
            assert self.equalMasterSystemState(self.proxy.getSystemState())

            still_service_detected = {'list': False}

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()

            # Start a dummy node
            talker_node = roslaunch.core.Node('roscpp_tutorials', 'talker')
            distraction_process = launch.launch(talker_node)
            try:
                # Loop a bit so we can detect the service
                with timeout(5) as t:
                    while not t.timed_out:
                        # Here we only check the last message received
                        if not still_service_detected['list'] and self.conn_list_msgq and self.add_two_ints_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we find it
                            still_service_detected['list'] = True
                            break
                        time.sleep(0.2)

                assert still_service_detected['list']
                time.sleep(0.2)
                # asserting in proxy as well
                assert self.equalMasterSystemState(self.proxy.getSystemState())
            finally:
                distraction_process.stop()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()

        finally:
            process.stop()

        lost_service_detected = {'list': False, 'diff': False}

        # Loop a bit so we can detect the service is gone
        with timeout(5) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not lost_service_detected['list'] and self.conn_list_msgq and not self.add_two_ints_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we DONT find it
                    lost_service_detected['list'] = True

                if not lost_service_detected['diff'] and self.conn_diff_msgq and self.add_two_ints_detected(self.conn_diff_msgq[-1].lost, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we find it
                    lost_service_detected['diff'] = True

                if lost_service_detected['list'] and lost_service_detected['diff']:
                    break
                time.sleep(0.2)

        assert lost_service_detected['list']
        assert lost_service_detected['diff']

        time.sleep(0.2)
        # asserting in proxy as well
        assert self.equalMasterSystemState(self.proxy.getSystemState())

    # TODO : detect actions server and client

    def test_change_spin_rate_detect_sub(self):
        # constant use just to prevent spinning too fast
        overspin_sleep_val= 0.02
        def prevent_overspin_sleep():
            time.sleep(overspin_sleep_val)

        # Prepare launcher
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # wait until we get current connectioncache spin
        with timeout(5) as t:
            while not t.timed_out and self.spin_freq == 0.0:
                prevent_overspin_sleep()

        assert not self.spin_freq == 0.0

        # Make rate 3 times slower ( enough to have time to create the node )
        mem_spin_freq = self.spin_freq
        rate_msg = rocon_std_msgs.ConnectionCacheSpin()
        rate_msg.spin_freq = self.spin_freq/3
        self.set_spin.publish(rate_msg)

        # Start a dummy node
        listener_node = roslaunch.core.Node('roscpp_tutorials', 'listener')
        process = launch.launch(listener_node)

        # check that we dont get any update
        added_subscriber_diff_detected = False

        # wait - only as long as a tick - until rate has been published as changed.
        # during this time we shouldnt detect the subscriber
        counter = 0
        while (counter == 0 or 1/(counter * overspin_sleep_val) < mem_spin_freq) and self.spin_freq == mem_spin_freq:
            counter += 1
            # Here we only check the last message received
            if self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                added_subscriber_diff_detected = True

            assert not added_subscriber_diff_detected
            prevent_overspin_sleep()

        # we should have waited less than one tick
        assert counter == 0 or 1/(counter * overspin_sleep_val) < mem_spin_freq

        # Start counting
        start_wait = rospy.get_time()
        while not added_subscriber_diff_detected and rospy.get_time() - start_wait < 1/self.spin_freq:
            # check that we get an update
            # Here we only check the last message received
            if self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                added_subscriber_diff_detected = True
            prevent_overspin_sleep()

        assert added_subscriber_diff_detected

        # Make rate fast again
        last_freq = self.spin_freq
        rate_msg = rocon_std_msgs.ConnectionCacheSpin()
        rate_msg.spin_freq = mem_spin_freq
        self.set_spin.publish(rate_msg)

        # restart the dummy node
        process.stop()
        process = launch.launch(listener_node)
        added_subscriber_diff_detected = False

        # wait - only for a tick - until rate has changed
        counter = 0
        while (counter == 0 or 1/(counter * overspin_sleep_val) < last_freq) and not self.spin_freq == mem_spin_freq:
            counter += 1
            prevent_overspin_sleep()

        # we should have waited less than one tick
        assert counter == 0 or 1/(counter * overspin_sleep_val) < last_freq

        # Start counting
        start_wait = rospy.get_time()
        while not added_subscriber_diff_detected and rospy.get_time() - start_wait < 1/mem_spin_freq:
            # check that we get an update
            # Here we only check the last message received
            if self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                added_subscriber_diff_detected = True

        assert added_subscriber_diff_detected

        process.stop()


class TestConnectionCacheNodeDiff(TestConnectionCacheNode):

    def setUp(self, cacheproxy=rocon_python_comms.ConnectionCacheProxy):
        super(TestConnectionCacheNodeDiff, self).setUp(partial(rocon_python_comms.ConnectionCacheProxy, diff_opt=True))

    def tearDown(self):
        super(TestConnectionCacheNodeDiff, self).tearDown()


if __name__ == '__main__':

    setup_module()
    rostest.rosrun('rocon_python_comms',
                   'test_connection_cache',
                   TestConnectionCacheNode)
    teardown_module()
