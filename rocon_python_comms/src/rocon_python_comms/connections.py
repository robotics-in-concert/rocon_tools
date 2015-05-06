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

import re
import socket

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
                              ACTION_CLIENT,
                              ACTION_SERVER
                             ])
connection_types_list = [PUBLISHER,
                         SUBSCRIBER,
                         SERVICE,
                         ACTION_CLIENT,
                         ACTION_SERVER
                        ]
action_types = ['/goal', '/cancel', '/status', '/feedback', '/result']

##############################################################################
# Classes
##############################################################################


class Connection(object):
    '''
      An object that represents a connection containing all the gory details
      about a connection, allowing a connection to be passed through to a
      foreign gateway.

      Note, we use a ros msg type as a data structure for the variable storage.
      This lets users spin it off in the ros world as needed as well as
      providing extra operators for manipulation and handling of connection
      types on top.
    '''

    def __init__(self, connection_type, name, node, type_info=None, xmlrpc_uri=None):
        '''
        :param str type: type of connection from string constants in rocon_std_msgs.Connection (e.g. PUBLISHER)
        :param str name: the topic/service name or the action base name
        :param str node: the name of the node establishing this connection
        :param str type_info: topic, service or action type, e.g. std_msgs/String
        :param str xmlrpc_uri: xmlrpc node uri for managing the connection
        '''
        self._connection = rocon_std_msgs.Connection(connection_type, name, node, type_info, xmlrpc_uri)

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

    def generate_type_info(self):
        '''
        Basic connection details are provided by get system state from the master, which is
        a one shot call to give you information about every connection possible. it does
        not however provide type info information and the only way of retrieving that from
        the master is making one xmlrpc call to the master for every single connection.
        This gets expensive, so generating this information is usually delayed until we
        need it and done via this method.
        '''
        if self.type_info is None:
            if self.type == PUBLISHER or self.type == SUBSCRIBER:
                self.type_info = rostopic.get_topic_type(self.name)[0]  # message type
            elif self.type == SERVICE:
                self.type_info = rosservice.get_service_uri(self.name)
            elif self.type == ACTION_SERVER or self.type == ACTION_CLIENT:
                goal_topic = self.name + '/goal'
                goal_topic_type = rostopic.get_topic_type(goal_topic)
                self.type_info = re.sub('ActionGoal$', '', goal_topic_type[0])  # Base type for action

    def generate_xmlrpc_info(self, master=None):
        '''
        As with type info, detailed xmlrpc info has to be generated on a per connection
        basis which is expensive, so it's best to delay its generation until needed.

        :param rosgraph.Master master : if you've already got a master xmlrpc client initialised, use that.
        '''
        if self.xmlrpc_uri is None:
            if master is None:
                master = rosgraph.Master(self.node)
            self.xmlrpc_uri = master.lookupNode(self.node)

    def __eq__(self, other):
        '''
          Don't need to check every characteristic of the connection to
          uniquely identify it, just the name, node and type.
        '''
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
        '''
        String representation of the connection, it differs a little by connection type.
        '''
        if self.type == SERVICE:
            return '{type: %s, name: %s, node: %s, uri: %s, service_api: %s}' % (
                self.type,
                self.name,
                self.node,
                self.xmlrpc_uri,
                self.type_info
                )
        else:
            return '{type: %s, name: %s, node: %s, uri: %s, topic_type: %s}' % (
                self.type,
                self.name,
                self.node,
                self.xmlrpc_uri,
                self.type_info
                )

    def __repr__(self):
        return self.__str__()

##############################################################################
# Utility Methods
##############################################################################


def create_empty_connection_type_dictionary():
    '''
      Used to initialise a dictionary with rule type keys
      and empty lists.
    '''
    dic = {}
    for connection_type in connection_types:
        dic[connection_type] = []
    return dic

##############################################################################
# Connection Cache
##############################################################################


class ConnectionCache(object):
    """
    Caches all of the connections living in a ros master. Use the 'update'
    method to force a refresh of the basic information for every connection.
    For detailed information, run 'generate_type_info' and
    'generate_xmlrpc_info' on the specific connection (this is a more
    expensive call and you don't want to do this for every connection).
    """
    __slots__ = [
                 'connections',
                 '_lookup_node',
                 '_get_system_state'
                ]

    def __init__(self):
        master = rosgraph.Master(rospy.get_name())
        self._lookup_node = master.lookupNode
        self._get_system_state = master.getSystemState
        self.connections = create_empty_connection_type_dictionary()

    def generate_type_info(self, name):
        '''
        Generate type info for all nodes with the specified name.
        '''
        for connection_type in connection_types:
            for connection in self.connections[connection_type]:
                if name == connection.name:
                    connection.generate_type_info()

    def find(self, name):
        '''
        Convenience function for finding all connections with the
        specified name.

        @TODO other find methods using a mix of name, node, type.
        '''
        found_connections = []
        for connection_type in connection_types:
            for connection in self.connections[connection_type]:
                if name == connection.name:
                    found_connections.append(connection)
        return found_connections

    def __str__(self):
        '''
        String representation of the connection cache.
        '''
        s = ""
        for connection_type in connection_types:
            s += ("%s:\n" % connection_type)
            for connection in self.connections[connection_type]:
                s += "  {name: %s, node: %s, type_info: %s, xmlrpc_uri: %s}\n" % (connection.name,
                       connection.node,
                       connection.type_info,
                       connection.xmlrpc_uri
                      )
        return s

    def update(self, new_system_state=None):
        '''
          Currently completely regenerating the connections dictionary and then taking
          diffs. Could be faster if we took diffs on the system state instead, but that's
          a bit more awkward since each element has a variable list of nodes that we'd have
          to check against to get good diffs. e.g.

            old_publishers = ['/chatter', ['/talker']]
            new_publishers = ['/chatter', ['/talker', '/babbler']]
        '''
        # init the variables we will return
        new_connections = create_empty_connection_type_dictionary()
        lost_connections = create_empty_connection_type_dictionary()

        if new_system_state is None:
            try:
                publishers, subscribers, services = self._get_system_state()
            except socket.error:
                rospy.logerr("Gateway : couldn't get system state from the master "
                             "[did you set your master uri to a wireless IP that just went down?]")
                return new_connections, lost_connections
        else:
            publishers = new_system_state[PUBLISHER]
            subscribers = new_system_state[SUBSCRIBER]
            services = new_system_state[SERVICE]
        action_servers, publishers, subscribers = self._get_action_servers(publishers, subscribers)
        action_clients, publishers, subscribers = self._get_action_clients(publishers, subscribers)
        connections = create_empty_connection_type_dictionary()
        connections[PUBLISHER] = self._get_connections_from_pub_sub_list(publishers, PUBLISHER)
        connections[SUBSCRIBER] = self._get_connections_from_pub_sub_list(subscribers, SUBSCRIBER)
        connections[SERVICE] = self._get_connections_from_service_list(services, SERVICE)
        connections[ACTION_SERVER] = self._get_connections_from_action_list(action_servers, ACTION_SERVER)
        connections[ACTION_CLIENT] = self._get_connections_from_action_list(action_clients, ACTION_CLIENT)

        # Will probably need to check not just in, but only name, node equal
        diff = lambda l1, l2: [x for x in l1 if x not in l2]
        for connection_type in connection_types:
            new_connections[connection_type] = diff(connections[connection_type], self.connections[connection_type])
            lost_connections[connection_type] = diff(self.connections[connection_type], connections[connection_type])
        self.connections = connections
        return new_connections, lost_connections

    def _is_topic_node_in_list(self, topic, node, topic_node_list):
        # check if cancel available
        available = False
        for candidate in topic_node_list:
            if candidate[0] == topic and node in candidate[1]:
                available = True
                break
        return available

    def _get_connections_from_action_list(self, connection_list, connection_type):
        connections = []
        for action in connection_list:
            action_name = action[0]
            #goal_topic = action_name + '/goal'
            #goal_topic_type = rostopic.get_topic_type(goal_topic)
            # topic_type = re.sub('ActionGoal$', '', goal_topic_type[0])  # Base type for action
            nodes = action[1]
            for node in nodes:
                # try:
                #    node_uri = self.lookupNode(node)
                # except:
                #    continue
                connection = Connection(connection_type, action_name, node, None, None)  # topic_type, node_uri
                connections.append(connection)
        return connections

    def _get_connections_from_service_list(self, connection_list, connection_type):
        connections = []
        for service in connection_list:
            service_name = service[0]
            #service_uri = rosservice.get_service_uri(service_name)
            nodes = service[1]
            for node in nodes:
                # try:
                #    node_uri = self.lookupNode(node)
                # except:
                #    continue
                connection = Connection(connection_type, service_name, node, None, None)  # service_uri, node_uri
                connections.append(connection)
        return connections

    def _get_connections_from_pub_sub_list(self, connection_list, connection_type):
        connections = []
        for topic in connection_list:
            topic_name = topic[0]
            #topic_type = rostopic.get_topic_type(topic_name)
            #topic_type = topic_type[0]
            nodes = topic[1]
            for node in nodes:
                # try:
                    #node_uri = self.lookupNode(node)
                # except:
                #    continue
                connection = Connection(connection_type, topic_name, node, None, None)  # topic_type, node_uri
                connections.append(connection)
        return connections

    def _get_actions(self, pubs, subs):
        '''
          Return actions and pruned publisher, subscriber lists.

          @param publishers
          @type list of publishers in the form returned by rosgraph.Master.get_system_state
          @param subscribers
          @type list of subscribers in the form returned by rosgraph.Master.get_system_state
          @return list of actions, pruned_publishers, pruned_subscribers
          @rtype [base_topic, [nodes]], as param type, as param type
        '''

        actions = []
        for goal_candidate in pubs:
            if re.search('\/goal$', goal_candidate[0]):
                # goal found, extract base topic
                base_topic = re.sub('\/goal$', '', goal_candidate[0])
                nodes = goal_candidate[1]
                action_nodes = []

                # there may be multiple nodes -- for each node search for the other topics
                for node in nodes:
                    is_action = True
                    is_action &= self._is_topic_node_in_list(base_topic + '/goal', node, pubs)
                    is_action &= self._is_topic_node_in_list(base_topic + '/cancel', node, pubs)
                    is_action &= self._is_topic_node_in_list(base_topic + '/status', node, subs)
                    is_action &= self._is_topic_node_in_list(base_topic + '/feedback', node, subs)
                    is_action &= self._is_topic_node_in_list(base_topic + '/result', node, subs)

                    if is_action:
                        action_nodes.append(node)

                if len(action_nodes) != 0:
                    # yay! an action has been found
                    actions.append([base_topic, action_nodes])
                    # remove action entries from publishers/subscribers
                    for connection in pubs:
                        if connection[0] in [base_topic + '/goal', base_topic + '/cancel']:
                            for node in action_nodes:
                                try:
                                    connection[1].remove(node)
                                except ValueError:
                                    rospy.logerr(
                                        "Gateway : couldn't remove an action publisher " +
                                        "from the master connections list [%s][%s]" %
                                        (connection[0], node))
                    for connection in subs:
                        if connection[0] in [base_topic + '/status', base_topic + '/feedback', base_topic + '/result']:
                            for node in action_nodes:
                                try:
                                    connection[1].remove(node)
                                except ValueError:
                                    rospy.logerr(
                                        "Gateway : couldn't remove an action subscriber " +
                                        "from the master connections list [%s][%s]" %
                                        (connection[0], node))
        pubs[:] = [connection for connection in pubs if len(connection[1]) != 0]
        subs[:] = [connection for connection in subs if len(connection[1]) != 0]
        return actions, pubs, subs

    def _get_action_servers(self, publishers, subscribers):
        '''
          Return action servers and pruned publisher, subscriber lists.

          @param publishers
          @type list of publishers in the form returned by rosgraph.Master.get_system_state
          @param subscribers
          @type list of subscribers in the form returned by rosgraph.Master.get_system_state
          @return list of actions, pruned_publishers, pruned_subscribers
          @rtype [base_topic, [nodes]], as param type, as param type
        '''
        actions, subs, pubs = self._get_actions(subscribers, publishers)
        return actions, pubs, subs

    def _get_action_clients(self, publishers, subscribers):
        '''
          Return action clients and pruned publisher, subscriber lists.

          @param publishers
          @type list of publishers in the form returned by rosgraph.Master.get_system_state
          @param subscribers
          @type list of subscribers in the form returned by rosgraph.Master.get_system_state
          @return list of actions, pruned_publishers, pruned_subscribers
          @rtype [base_topic, [nodes]], as param type, as param type
        '''
        actions, pubs, subs = self._get_actions(publishers, subscribers)
        return actions, pubs, subs
