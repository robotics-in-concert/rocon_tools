#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
##############################################################################
# Description
##############################################################################

"""
.. module:: connections
   :platform: Unix
   :synopsis: A comprehensive api for listing/handling ros connections.


This is a wrapper around the many ad-hoc modules that work with the ros master
system state list of pubs, subs, services and actions. In some cases it
just extends the functionality (severely lacking in some cases) and in others
it provides new, higher level methods (e.g. for actions).
----

"""

##############################################################################
# Imports
##############################################################################
import copy
import os
import re
import socket
import threading

import collections
import time

import rocon_std_msgs.msg as rocon_std_msgs
import rosgraph
import rospy
import rostopic
import rosservice

##############################################################################
# Aliases
##############################################################################

# Can't see an easier way to alias or import these
PUBLISHER = rocon_std_msgs.Connection.PUBLISHER
SUBSCRIBER = rocon_std_msgs.Connection.SUBSCRIBER
SERVICE = rocon_std_msgs.Connection.SERVICE
ACTION_SERVER = rocon_std_msgs.Connection.ACTION_SERVER
ACTION_CLIENT = rocon_std_msgs.Connection.ACTION_CLIENT

##############################################################################
# Constants
##############################################################################

# for help in iterating over the set of connection constants
connection_types = frozenset([PUBLISHER,
                              SUBSCRIBER,
                              SERVICE,
                              ])
connection_types_actions = frozenset([
                              ACTION_CLIENT,
                              ACTION_SERVER
                              ])
connection_types_list = [PUBLISHER,
                         SUBSCRIBER,
                         SERVICE,
                         ]
connection_types_actions_list = [
                         ACTION_CLIENT,
                         ACTION_SERVER
                         ]

action_types = ['/goal', '/cancel', '/status', '/feedback', '/result']

##############################################################################
# Classes
##############################################################################


class Connection(object):
    """
      An object that represents a connection containing all the gory details
      about a connection, allowing a connection to be passed along to other nodes.

      Note, we use a ros msg type as a data structure for the variable storage.
      This lets users spin it off in the ros world as needed as well as
      providing extra operators for manipulation and handling of connection
      types on top.
    """

    def __init__(self, connection_type, name, node, type_msg=None, type_info=None, xmlrpc_uri=None):
        """
        :param str type: type of connection from string constants in rocon_std_msgs.Connection (e.g. PUBLISHER)
        :param str name: the topic/service name or the action base name
        :param str node: the name of the node establishing this connection
        :param str type_msg: topic or service type, e.g. std_msgs/String
        :param str type_info: extra type information ( following rospy implementation ) : service uri or topic type
        :param str xmlrpc_uri: xmlrpc node uri for managing the connection
        """
        self._connection = rocon_std_msgs.Connection(connection_type, name, node, type_msg, type_info, xmlrpc_uri)

    @property
    def type(self):
        return self._connection.type if self._connection.type else None

    @type.setter
    def type(self, connection_type):
        self._connection.type = connection_type if connection_type else ""

    @property
    def name(self):
        return self._connection.name if self._connection.name else None

    @name.setter
    def name(self, connection_name):
        self._connection.name = connection_name if connection_name else ""

    @property
    def node(self):
        return self._connection.node if self._connection.node else None

    @node.setter
    def node(self, connection_node):
        self._connection.node = connection_node if connection_node else ""

    @property
    def type_msg(self):
        return self._connection.type_msg if self._connection.type_msg else None

    @type_msg.setter
    def type_msg(self, connection_type_msg):
        self._connection.type_msg = connection_type_msg if connection_type_msg else ""

    @property
    def type_info(self):
        return self._connection.type_info if self._connection.type_info else None

    @type_info.setter
    def type_info(self, connection_type_info):
        self._connection.type_info = connection_type_info if connection_type_info else ""

    @property
    def xmlrpc_uri(self):
        return self._connection.xmlrpc_uri if self._connection.xmlrpc_uri else None

    @xmlrpc_uri.setter
    def xmlrpc_uri(self, connection_xmlrpc_uri):
        self._connection.xmlrpc_uri = connection_xmlrpc_uri if connection_xmlrpc_uri else ""

    @property
    def msg(self):
        return self._connection

    @msg.setter
    def msg(self, msg):
        self._connection = msg

    def generate_type_info_msg(self):
        """
        Basic connection details are provided by get system state from the master, which is
        a one shot call to give you information about every connection possible. it does
        not however provide type info information and the only way of retrieving that from
        the master is making one xmlrpc call to the master for every single connection.
        This gets expensive, so generating this information is usually delayed until we
        need it and done via this method.
        """
        if self.type_info is None:
            if self.type == PUBLISHER or self.type == SUBSCRIBER:
                try:
                    self.type_info = rostopic.get_topic_type(self.name)[0]  # message type
                    self.type_msg = self.type_info
                except rostopic.ROSTopicIOException as topic_exc:
                    rospy.logwarn(topic_exc)
            elif self.type == SERVICE:
                try:
                    self.type_info = rosservice.get_service_uri(self.name)
                    self.type_msg = rosservice.get_service_type(self.name)
                except rosservice.ROSServiceIOException as service_exc:
                    rospy.logwarn(service_exc)
            elif self.type == ACTION_SERVER or self.type == ACTION_CLIENT:
                try:
                    goal_topic = self.name + '/goal'
                    goal_topic_type = rostopic.get_topic_type(goal_topic)
                    self.type_info = re.sub('ActionGoal$', '', goal_topic_type[0])  # Base type for action
                    self.type_msg = self.type_info
                except rostopic.ROSTopicIOException as topic_exc:
                    rospy.logwarn(topic_exc.msg)
        return self  # chaining

    def generate_xmlrpc_info(self, master=None):
        """
        As with type info, detailed xmlrpc info has to be generated on a per connection
        basis which is expensive, so it's best to delay its generation until needed.

        :param rosgraph.Master master : if you've already got a master xmlrpc client initialised, use that.
        """
        if self.xmlrpc_uri is None:
            if master is None:
                master = rosgraph.Master(self.node)
            try:
                self.xmlrpc_uri = master.lookupNode(self.node)
            except rosgraph.MasterError as exc:
                rospy.logwarn(str(exc))  # keep going even if the node is not found.
        return self  # chaining

    def __eq__(self, other):
        """
          Don't need to check every characteristic of the connection to
          uniquely identify it, just the name, node and type.
        """
        if isinstance(other, self.__class__):
            # return self.__dict__ == other.__dict__
            return (self.name == other.name and
                    self.type == other.type and
                    self.node == other.node)
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        """
        String representation of the connection, it differs a little by connection type.
        """
        if self.type == SERVICE:
            return '{type: %s, name: %s, node: %s, uri: %s, service_api: %s}' % (self.type,
                                                                                 self.name,
                                                                                 self.node,
                                                                                 self.xmlrpc_uri,
                                                                                 self.type_info
                                                                                 )
        else:
            return '{type: %s, name: %s, node: %s, uri: %s, topic_type: %s}' % (self.type,
                                                                                self.name,
                                                                                self.node,
                                                                                self.xmlrpc_uri,
                                                                                self.type_info
                                                                                )

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return hash((self.name, self.type, self.node))


##############################################################################
# Utility Methods
##############################################################################


def create_connection(ConnectionMsg):
    """
    Creates a Connection instance from a Connection message
    """
    return Connection(ConnectionMsg.type, ConnectionMsg.name, ConnectionMsg.node, ConnectionMsg.type_msg, ConnectionMsg.type_info, ConnectionMsg.xmlrpc_uri)


def create_empty_connection_type_dictionary(types=None):
    '''
      Used to initialise a dictionary with rule type keys
      and empty lists.
    '''
    types = types or connection_types
    dic = {}
    for connection_type in types:
        dic[connection_type] = set()
    return dic

##############################################################################
# Connection Cache
##############################################################################


class ConnectionCache(object):
    """
    Caches all of the connections living in a ros master. Use the 'update'
    method to force a refresh of the basic information for every connection.
    """

    def __init__(self):
        #: rosgraph.Master: master API instance
        self.master = rosgraph.Master(rospy.get_name())

        #: dict: dict structure of connections, by type.
        self.connections = create_empty_connection_type_dictionary(connection_types)

    def find(self, name):
        """
        Convenience function for finding all connections with the
        specified name.

        @TODO other find methods using a mix of name, node, type.
        """
        types = connection_types
        found_connections = []
        for connection_type in types:
            for connection in self.connections[connection_type]:
                if name == connection.name:
                    found_connections.append(connection)
        return found_connections

    def __str__(self):
        """
        String representation of the connection cache.
        """
        s = ""
        types = connection_types
        for connection_type in types:
            s += ("%s:\n" % connection_type)
            for connection in self.connections[connection_type]:
                s += "  {name: %s, node: %s, type_info: %s, xmlrpc_uri: %s}\n" % (connection.name,
                                                                                  connection.node,
                                                                                  connection.type_info,
                                                                                  connection.xmlrpc_uri
                                                                                  )
        return s

    def update(self, new_system_state=None, new_topic_types=None):
        """
          Currently completely regenerating the connections dictionary and then taking
          diffs. Could be faster if we took diffs on the system state instead, but that's
          a bit more awkward since each element has a variable list of nodes that we'd have
          to check against to get good diffs. e.g.

            old_publishers = ['/chatter', ['/talker']]
            new_publishers = ['/chatter', ['/talker', '/babbler']]
        """
        # init the variables we will return
        new_connections = create_empty_connection_type_dictionary(connection_types)
        lost_connections = create_empty_connection_type_dictionary(connection_types)

        if new_system_state is None:
            try:
                publishers, subscribers, services = self.master.getSystemState()
                topic_types = self.master.getTopicTypes()
            except socket.error:
                rospy.logerr("ConnectionCache : couldn't get system state from the master "
                             "[did you set your master uri to a wireless IP that just went down?]")
                return new_connections, lost_connections
        else:
            publishers = new_system_state[PUBLISHER]
            subscribers = new_system_state[SUBSCRIBER]
            services = new_system_state[SERVICE]
            topic_types = new_topic_types

        pubs = self._get_connections_from_pub_sub_list(publishers, PUBLISHER, topic_types)
        new_connections[PUBLISHER] = pubs - self.connections[PUBLISHER]
        for c in new_connections[PUBLISHER]:
            c.generate_xmlrpc_info()
        # lost connections already have xmlrpc_uri and it s not checked by set for unicity (__hash__)
        lost_connections[PUBLISHER] = self.connections[PUBLISHER] - pubs

        subs = self._get_connections_from_pub_sub_list(subscribers, SUBSCRIBER, topic_types)
        new_connections[SUBSCRIBER] = subs - self.connections[SUBSCRIBER]
        for c in new_connections[SUBSCRIBER]:
            c.generate_xmlrpc_info()
        # lost connections already have xmlrpc_uri and it s not checked by set for unicity (__hash__)
        lost_connections[SUBSCRIBER] = self.connections[SUBSCRIBER] - subs

        svcs = self._get_connections_from_service_list(services, SERVICE)
        new_connections[SERVICE] = svcs - self.connections[SERVICE]
        for c in new_connections[SERVICE]:
            c.generate_type_info_msg()
            c.generate_xmlrpc_info()
        # lost connections already have xmlrpc_uri and it s not checked by set for unicity (__hash__)
        # type_info is different but it is also not checked by set for unicity (__hash__)
        lost_connections[SERVICE] = self.connections[SERVICE] - svcs

        self.connections[PUBLISHER].update(new_connections[PUBLISHER])
        self.connections[PUBLISHER].difference_update(lost_connections[PUBLISHER])

        self.connections[SUBSCRIBER].update(new_connections[SUBSCRIBER])
        self.connections[SUBSCRIBER].difference_update(lost_connections[SUBSCRIBER])

        self.connections[SERVICE].update(new_connections[SERVICE])
        self.connections[SERVICE].difference_update(lost_connections[SERVICE])

        return new_connections, lost_connections

    @staticmethod
    def _get_connections_from_service_list(connection_list, connection_type):
        connections = set()
        for service in connection_list:
            service_name = service[0]
            # service_uri = rosservice.get_service_uri(service_name)
            nodes = service[1]
            for node in nodes:
                # try:
                #    node_uri = self.lookupNode(node)
                # except:
                #    continue
                connection = Connection(connection_type, service_name, node)  # service_uri, node_uri
                connections.add(connection)
        return connections

    @staticmethod
    def _get_connections_from_pub_sub_list(connection_list, connection_type, msg_type_list):
        connections = set()
        for topic in connection_list:
            topic_name = topic[0]
            topic_type = [t[1] for t in msg_type_list if t[0] == topic_name]
            topic_type = topic_type[0] if topic_type else None
            nodes = topic[1]
            for node in nodes:
                # try:
                    # node_uri = self.lookupNode(node)
                # except:
                #    continue
                connection = Connection(connection_type, topic_name, node, topic_type, topic_type)
                connections.add(connection)
        return connections

    @staticmethod
    def _get_connections_from_action_list(connection_list, connection_type):
        connections = set()
        for action in connection_list:
            action_name = action[0]
            # goal_topic = action_name + '/goal'
            # goal_topic_type = rostopic.get_topic_type(goal_topic)
            # topic_type = re.sub('ActionGoal$', '', goal_topic_type[0])  # Base type for action
            nodes = action[1]
            for node in nodes:
                # try:
                #    node_uri = self.lookupNode(node)
                # except:
                #    continue
                connection = Connection(connection_type, action_name, node)
                connections.add(connection)
        return connections


class ConnectionCacheNode(object):
    def __init__(self):

        self.spin_freq = rospy.get_param("~spin_freq", 0.1)
        self.spin_original_freq = self.spin_freq
        self.spin_timer = 0.0
        self.conn_cache = ConnectionCache()  # we want a drop in replacement for ROSmaster access

        self.conn_cache_spin_pub = rospy.Publisher("~spin", rocon_std_msgs.ConnectionCacheSpin, latch=True, queue_size=1)
        self.conn_cache_spin_sub = rospy.Subscriber("~spin", rocon_std_msgs.ConnectionCacheSpin, self.set_spin_cb)

        self.conn_list = rospy.Publisher("~list", rocon_std_msgs.ConnectionsList, latch=True, queue_size=1)  # uptodate full list
        self.conn_diff = rospy.Publisher("~diff", rocon_std_msgs.ConnectionsDiff, queue_size=5, tcp_nodelay=True)  # differences only for faster parsing.

    def set_spin_cb(self, data):
        if data.spin_freq and not data.spin_freq == self.spin_freq:  # we change the rate if needed
            self.spin_freq = data.spin_freq
            self.spin_timer = data.spin_timer

    def spin(self):
        rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
        try:
            # sensible default values
            self.spin_rate = rospy.Rate(self.spin_freq)
            last_spinmsg = None
            last_update = time.time()

            while not rospy.core.is_shutdown():
                elapsed_time = time.time() - last_update
                self.spin_timer = max(self.spin_timer - elapsed_time, 0.0)
                last_update = time.time()

                # If needed (or first time) we change our spin rate, and publish the new frequency
                if self.spin_timer > 0.0 or last_spinmsg is None or last_spinmsg.spin_timer > 0.0:
                    # if spin_timer just came back to 0.0 we use self.spin_original_freq
                    if self.spin_timer == 0.0:
                        self.spin_freq = self.spin_original_freq
                    # if timer is almost finished we need to increase rate to be back to original speed on time
                    self.spin_rate = rospy.Rate(
                        self.spin_freq if self.spin_timer == 0.0 else max(self.spin_freq, 1 / self.spin_timer)
                    )
                    spinmsg = rocon_std_msgs.ConnectionCacheSpin()
                    spinmsg.spin_freq = self.spin_freq
                    spinmsg.spin_timer = self.spin_timer
                    last_spinmsg = spinmsg
                    self.conn_cache_spin_pub.publish(spinmsg)

                try:
                    new_conns, lost_conns = self.conn_cache.update()
                    changed = False

                    diff_msg = rocon_std_msgs.ConnectionsDiff()
                    list_msg = rocon_std_msgs.ConnectionsList()
                    for ct in connection_types:
                        if new_conns[ct] or lost_conns[ct]:  # something changed
                            changed = True
                            for c in new_conns[ct]:
                                create_connection(c)
                                diff_msg.added.append(c.msg)
                                rospy.logdebug("[rocon_python_comms] Connection Cache + {0}".format(c))
                            for c in lost_conns[ct]:
                                create_connection(c)
                                rospy.logdebug("[rocon_python_comms] Connection Cache - {0}".format(c))
                                diff_msg.lost.append(c.msg)
                        # we always need all connections types in the full list
                        for c in self.conn_cache.connections[ct]:
                            create_connection(c)
                            list_msg.connections.append(c.msg)

                    if changed:
                        # rospy.loginfo("COMPLETE LIST : {0}".format(self.conn_cache.connections))

                        self.conn_diff.publish(diff_msg)  # new_conns, old_conns
                        self.conn_list.publish(list_msg)  # conn_cache.connections

                except rospy.ROSException:
                    rospy.logerr("[rocon_python_comms] Connection Cache ROS Watcher : Connections list unavailable.")
                except rospy.ROSInterruptException:
                    rospy.logerr("[rocon_python_comms] Connection Cache ROS Watcher : ros shutdown while looking for Connections .")

                self.spin_rate.sleep()

        except KeyboardInterrupt:
            rospy.logdebug("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')

