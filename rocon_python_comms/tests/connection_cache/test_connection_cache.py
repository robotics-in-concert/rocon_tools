#!/usr/bin/env python

""" Testing the find_node function """

import unittest
import rospy
import rostest
import rocon_python_comms


class TestConnectionCache(unittest.TestCase):
    def test_find_node_unique(self):
        connection_cache = rocon_python_comms.ConnectionCache()
        new_connections, lost_connections = connection_cache.update()
        # delayed connection details generators (too expensive calling every one
        # so we usually call them only individually as required)
        for connection_type in rocon_python_comms.connection_types:
            for connection in new_connections[connection_type]:
                connection.generate_type_info()
                connection.generate_xmlrpc_info()
        print("Connections: \n%s" % connection_cache)
        assert connection_cache.find("/fibonacci")

if __name__ == '__main__':
    rostest.rosrun('rocon_python_comms',
                   'test_connection_cache',
                   TestConnectionCache)
