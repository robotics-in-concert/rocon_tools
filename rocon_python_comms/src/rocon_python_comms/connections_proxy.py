#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
##############################################################################
# Description
##############################################################################

"""
.. module:: connections
   :platform: Unix
   :synopsis: A proxy to use for communicating with the connection cache.

----

"""

##############################################################################
# Imports
##############################################################################
import copy
import threading

import collections

import rocon_std_msgs.msg as rocon_std_msgs
import rospy

from .connections import (
    SUBSCRIBER,
    PUBLISHER,
)


# TODO : split that in multiple files
class ConnectionCacheProxy(object):
    class InitializationTimeout(Exception):
        pass

    class Channel(object):
        """
        Definition of a channel ( topic/service )
        => a compressed version of a list of connection with same name
        """

        def __init__(self, name, type, xmlrpc_uri, nodes=None):
            """
            Initialize a Channel instance
            :param name
            :param type
            :param nodes a set of tuple (node_name, node_uri)
            """
            self.name = name
            self.type = type
            # ROS master keeps only one URI per service ( only the last node service URI is kept )
            # So we actually have only one xmlrpc_uri per channel
            self.xmlrpc_uri = xmlrpc_uri
            self.nodes = nodes or set()

        def __eq__(self, other):  # used for manual == operator
            if not isinstance(other, ConnectionCacheProxy.Channel):
                return NotImplemented
            elif self is other:
                return True
            else:
                return (self.name == other.name and
                        self.type == other.type and
                        self.xmlrpc_uri == other.xmlrpc_uri and
                        self.nodes == other.nodes)

        def __hash__(self):  # used for comparison in sets
            return hash((self.name, self.type))

        @staticmethod
        def dict_factory(conn_list, chan_dict=None):
            """
            Merge a list of Connections in a dict of Channels
            :param conn_list: List of connections : Each different connection name will create a new Channel
            :param chan_dict: Preexisting channel dict. if merge wanted.
            :return:
            """
            chan_dict = chan_dict or {}
            for c in conn_list:
                if c.name not in chan_dict.keys():
                    chan_dict[c.name] = ConnectionCacheProxy.Channel(
                        c.name,
                        c.type_msg,  # type_msg is always the message type (topic or service)
                        c.type_info if c.type_info != c.type_msg else None)  # None for topics who don't have uri.
                chan_dict[c.name].nodes.add((c.node, c.xmlrpc_uri))  # type_info is the uri of the service or the msgtype of the topic
            return chan_dict

        @staticmethod
        def dict_slaughterhouse(conn_list, chan_dict):
            """
            Removes a list of Connections from a dict of Channels
            :param conn_list: List of connections : Each different connection name will affect one and only one connection
            :param chan_dict: Preexisting channel dict. to extract from
            :return:
            """
            chan_dict = chan_dict
            for c in conn_list:
                try:
                    nodelist = chan_dict[c.name].nodes
                    try:
                        nodelist.remove((c.node, c.xmlrpc_uri))
                    except KeyError:  # keep it working even if unexpected things happen
                        rospy.logwarn("Trying to remove inexistent ({c.node}, {c.xmlrpc_uri}) from connection {c.name} nodes : {nodelist} ".format(**locals()))
                        pass  # node not in set. no need to remove
                    if not nodelist:
                        chan_dict.pop(c.name)
                except KeyError:  # keep it working even if unexpected things happen
                    rospy.logwarn("Trying to access nodes for inexistent {c.name} in {chan_dict} ".format(**locals()))
                    pass  # node not in set. no need to remove

            return chan_dict

    class ActionChannel(object):
        """
        Extension of a Channel for Actions
        """
        def __init__(self, goal_chan, cancel_chan, status_chan, feedback_chan, result_chan):
            """
            Initialize an ActionChannel instance
            :param goal_chan
            :param cancel_chan
            :param status_chan
            :param feedback_chan
            :param result_chan
            :param nodes a set of tuple (node_name, node_uri)
            """
            assert goal_chan.name.endswith("/goal")
            self.goal_chan = goal_chan
            assert cancel_chan.name.endswith("/cancel")
            self.cancel_chan = cancel_chan
            assert status_chan.name.endswith("/status")
            self.status_chan = status_chan
            assert feedback_chan.name.endswith("/feedback")
            self.feedback_chan = feedback_chan
            assert result_chan.name.endswith("/result")
            self.result_chan = result_chan

        @property
        def name(self):
            return self.goal_chan.name[:-len("/goal")]

        @property
        def type(self):
            return self.goal_chan.type

        @property
        def xmlrpc_uri(self):
            return None

        @property
        def nodes(self):
            return self.goal_chan.nodes | self.cancel_chan.nodes | self.status_chan.nodes | self.feedback_chan.nodes | self.result_chan.nodes

        def __eq__(self, other):  # used for manual == operator
            if not isinstance(other, ConnectionCacheProxy.ActionChannel):
                return NotImplemented
            elif self is other:
                return True
            else:
                return (self.goal_chan == other.goal_chan and
                        self.cancel_chan == other.cancel_chan and
                        self.status_chan == other.status_chan and
                        self.feedback_chan == other.feedback_chan and
                        self.result_chan == other.result_chan)

        def is_server(self):
            return (
                # only one of them present is enough to tell
                self.goal_chan.type == SUBSCRIBER
                or self.cancel_chan.type == SUBSCRIBER
                or self.status_chan.type == PUBLISHER
                or self.feedback_chan.type == PUBLISHER
                or self.result_chan.type == PUBLISHER
            )

        def is_client(self):
            return (
                self.goal_chan.type == PUBLISHER
                or self.cancel_chan.type == PUBLISHER
                or self.status_chan.type == SUBSCRIBER
                or self.feedback_chan.type == SUBSCRIBER
                or self.result_chan.type == SUBSCRIBER
            )

        @staticmethod
        def dict_factory_actions_from_chan(chan_dict, chan_other_dict, action_dict=None):
            """
            Build ActionChannels from two list of Channels, and merge them in the dictionary.
            The behavior is symmetrical, pass pubs_list, subs_list, for action clients and the reverse for action servers
            :param chan_dict: Dict of channels
            :param chan_other_dict: symmetric dict of channels ( subscriber / publisher complement of chan_dict )
            :return:
            """
            chan_dict = chan_dict or {}
            action_dict = action_dict or {}
            new_chan_dict = chan_dict
            new_chan_other_dict = chan_other_dict

            goal_chan_from_dict = {n[:-len("/goal")]: pc for n, pc in chan_dict.iteritems() if n.endswith("/goal")}
            cancel_chan_from_dict = {n[:-len("/cancel")]: pc for n, pc in chan_dict.iteritems() if n.endswith("/cancel")}
            status_chan_from_dict = {n[:-len("/status")]: pc for n, pc in chan_other_dict.iteritems() if n.endswith("/status")}
            feedback_chan_from_dict = {n[:-len("/feedback")]: pc for n, pc in chan_other_dict.iteritems() if n.endswith("/feedback")}
            result_chan_from_dict = {n[:-len("/result")]: pc for n, pc in chan_other_dict.iteritems() if n.endswith("/result")}

            # since we need all the 5 topics anyway for an action,
            # checking only in goal dict is enough
            for k, v in goal_chan_from_dict.iteritems():
                action_name = k
                try:
                    goal_chan = goal_chan_from_dict[k]
                    cancel_chan = cancel_chan_from_dict[k]
                    status_chan = status_chan_from_dict[k]
                    feedback_chan = feedback_chan_from_dict[k]
                    result_chan = result_chan_from_dict[k]
                except KeyError:  # skip this
                    continue

                # here we should have the 5 connections
                if action_name in new_chan_dict.keys():
                    action_dict[action_name].goal_chan.nodes |= goal_chan.nodes
                    action_dict[action_name].cancel_chan.nodes |= cancel_chan.nodes
                    action_dict[action_name].status_chan.nodes |= status_chan.nodes
                    action_dict[action_name].feedback_chan.nodes |= feedback_chan.nodes
                    action_dict[action_name].result_chan.nodes |= result_chan.nodes
                else:
                    action_dict[action_name] = ConnectionCacheProxy.ActionChannel(
                        goal_chan, cancel_chan, status_chan, feedback_chan, result_chan
                    )

                # purging old used stuff
                new_chan_dict.pop(goal_chan.name, None)
                new_chan_dict.pop(cancel_chan.name, None)

                new_chan_other_dict.pop(status_chan.name, None)
                new_chan_other_dict.pop(feedback_chan.name, None)
                new_chan_other_dict.pop(result_chan.name, None)

            return action_dict, new_chan_dict, new_chan_other_dict

        @staticmethod
        def dict_slaughterhouse_actions_from_chan(chan_dict, chan_other_dict, action_dict):
            """
            Destroy ActionChannels from action_lost_dict, and merge lost pubs/subs in the chan_lost_dict and chan_lost_other_dict.
            The behavior is symmetrical, pass pubs_list, subs_list, for action clients and the reverse for action servers
            :param chan_dict: Dict of channels
            :param chan_other_dict: symmetric dict of channels ( subscriber / publisher complement of chan_dict )
            :return:
            """
            chan_dict = chan_dict or {}
            action_dict = action_dict or {}
            new_chan_dict = chan_dict
            new_chan_other_dict = chan_other_dict

            goal_chan_from_dict = {n: act.goal_chan for n, act in action_dict.iteritems() if act.goal_chan.name in chan_dict.keys()}
            cancel_chan_from_dict = {n: act.cancel_chan for n, act in action_dict.iteritems() if act.cancel_chan.name in chan_dict.keys()}
            status_chan_from_dict = {n: act.status_chan for n, act in action_dict.iteritems() if act.status_chan.name in chan_other_dict.keys()}
            feedback_chan_from_dict = {n: act.feedback_chan for n, act in action_dict.iteritems() if act.feedback_chan.name in chan_other_dict.keys()}
            result_chan_from_dict = {n: act.result_chan for n, act in action_dict.iteritems() if act.result_chan.name in chan_other_dict.keys()}

            # since we need all the 5 topics anyway for an action,
            # losing only one is enough to break the action
            to_del = []
            for k, v in action_dict.iteritems():
                action_name = k
                goal_chan = None
                cancel_chan = None
                status_chan = None
                feedback_chan = None
                result_chan = None
                # removing the matching channels from channel dict.
                # registering action as lost is enough.
                try:
                    goal_chan = goal_chan_from_dict[k]
                    chan_dict.pop(goal_chan.name)
                except KeyError:  # doesnt matter
                    pass
                try:
                    cancel_chan = cancel_chan_from_dict[k]
                    chan_dict.pop(cancel_chan.name)
                except KeyError:  # doesnt matter
                    pass
                try:
                    status_chan = status_chan_from_dict[k]
                    chan_other_dict.pop(status_chan.name)
                except KeyError:  # doesnt matter
                    pass
                try:
                    feedback_chan = feedback_chan_from_dict[k]
                    chan_other_dict.pop(feedback_chan.name)
                except KeyError:  # doesnt matter
                    pass
                try:
                    result_chan = result_chan_from_dict[k]
                    chan_other_dict.pop(result_chan.name)
                except KeyError:  # doesnt matter
                    pass

                if (goal_chan is not None or cancel_chan is not None) or (
                    status_chan is not None or feedback_chan is not None or result_chan is not None
                ):
                    to_del.append(action_name)

            # purging lost actions
            for action_name in to_del:
                action_dict.pop(action_name)

            return action_dict, new_chan_dict, new_chan_other_dict

    def __init__(self, list_sub=None, handle_actions=False, user_callback=None, diff_opt=False, diff_sub=None, list_wait_timeout=5):
        """
        Initialize a connection cache proxy to retrieve system state while minimizing call to the master or the cache node
        This method will block until the connection cache node has sent its system state
        :param list_sub: topic name to subscribe to to get the list from connectioncache node
        :param handle_actions: whether connectioncacheProxy does action filtering internally
        :param user_callback: user callback function for asynchronous state change management
        :param diff_opt: whether we optimise the proxy by using the differences only and rebuilding the full state from it
        :param diff_sub: topic name to subscribe to to get the diff from connectioncache node
        :param list_wait_timeout: seconds to wait for list in list topic before timing out
        :return:
        """
        self.diff_opt = diff_opt
        self.diff_sub = diff_sub or '~connections_diff'
        self.handle_actions = handle_actions
        self.user_cb = None
        self._system_state_lock = threading.Lock()  # writer lock
        if self.handle_actions:
            self.SystemState = collections.namedtuple("SystemState", "publishers subscribers services action_servers action_clients")
        else:
            self.SystemState = collections.namedtuple("SystemState", "publishers subscribers services")
        self._system_state = None

        if user_callback:
            if not hasattr(user_callback, '__call__'):
                rospy.logwarn("Connection Cache Proxy user callback not callable. Ignoring user callback.")
            else:
                self.user_cb = user_callback

        self.conn_list_called = threading.Event()
        self.conn_list = rospy.Subscriber(list_sub or '~connections_list', rocon_std_msgs.ConnectionsList, self._list_cb)

        # TODO : Here we should get the spin freq (quick) and determine timeout depending on conneciton cache frequency
        if not self.conn_list_called.wait(list_wait_timeout):  # we block until we receive a message from connection node
            # if we timeout we except to prevent using the object uninitialized
            raise ConnectionCacheProxy.InitializationTimeout("Connection Cache Proxy timed out on initialization. aborting")

        # waiting until we are sure we are plugged in connection cache node.
        # RAII : after __init__() ConnectionCache is ready to use (self._system_state is initialized).

        rospy.loginfo("ConnectionCacheProxy: started inside node {}".format(rospy.get_name()))
        rospy.loginfo("                    : with list topic at {}".format(self.conn_list.name))
        rospy.loginfo("                    : and diff topic at {}".format(rospy.resolve_name(self.diff_sub)))

    @staticmethod
    def _is_topic_node_in_list(topic, node, topic_node_list):
        # TODO : there is probably a oneliner equivalent for this
        # check if cancel available
        available = False
        for candidate in topic_node_list:
            if candidate[0] == topic and node in candidate[1]:
                available = True
                break
        return available

    def _list_cb(self, data):
        self._system_state_lock.acquire()
        # we got a new full list : reset the local value for _system_state
        pubs = [c for c in data.connections if c.type == c.PUBLISHER]
        subs = [c for c in data.connections if c.type == c.SUBSCRIBER]
        svcs = [c for c in data.connections if c.type == c.SERVICE]

        pub_chans = ConnectionCacheProxy.Channel.dict_factory(pubs)
        sub_chans = ConnectionCacheProxy.Channel.dict_factory(subs)
        svc_chans = ConnectionCacheProxy.Channel.dict_factory(svcs)

        if self.handle_actions:
            action_server_chans, unused_subs_chans, pub_chans = ConnectionCacheProxy.ActionChannel.dict_factory_actions_from_chan(sub_chans, pub_chans)
            action_client_chans, pub_chans, unused_subs_chans = ConnectionCacheProxy.ActionChannel.dict_factory_actions_from_chan(pub_chans, sub_chans)
            self._system_state = self.SystemState(pub_chans, sub_chans, svc_chans, action_server_chans, action_client_chans)
        else:
            self._system_state = self.SystemState(pub_chans, sub_chans, svc_chans)

        self._system_state_lock.release()
        # rospy.loginfo("CACHE PROXY LIST_CB PUBLISHERS : {pubs}".format(pubs=self._system_state.publishers))
        # rospy.loginfo("CACHE PROXY LIST_CB SUBSCRIBERS : {subs}".format(subs=self._system_state.subscribers))
        # rospy.loginfo("CACHE PROXY LIST_CB SERVICES : {svcs}".format(svcs=self._system_state.services))

        if self.user_cb is not None and hasattr(self.user_cb, '__call__'):
            try:
                self.user_cb(self._system_state, None, None)
            except Exception as user_exc:
                rospy.logerr("Connection Cache Proxy : Diff Callback Exception {0}".format(user_exc))

        self.conn_list_called.set()  # signaling that the list_callback has been called ( for __init__ )

        if self.diff_opt:
            # hooking up to the diff and unhooking from the list
            self.conn_diff = rospy.Subscriber(self.diff_sub, rocon_std_msgs.ConnectionsDiff, self._diff_cb)
            self.conn_list.unregister()

    def _diff_cb(self, data):  # This should only run when we want to have the diff message optimization
        # modifying the system_state ( like the one provided by ROS master)
        self._system_state_lock.acquire()

        # we got a new full list : reset the local value for _system_state
        pubs_added = [c for c in data.added if c.type == c.PUBLISHER]
        subs_added = [c for c in data.added if c.type == c.SUBSCRIBER]
        svcs_added = [c for c in data.added if c.type == c.SERVICE]

        pubs_lost = [c for c in data.lost if c.type == c.PUBLISHER]
        subs_lost = [c for c in data.lost if c.type == c.SUBSCRIBER]
        svcs_lost = [c for c in data.lost if c.type == c.SERVICE]

        added_pub_chans = ConnectionCacheProxy.Channel.dict_factory(pubs_added)
        added_sub_chans = ConnectionCacheProxy.Channel.dict_factory(subs_added)
        added_svc_chans = ConnectionCacheProxy.Channel.dict_factory(svcs_added)

        lost_pub_chans = ConnectionCacheProxy.Channel.dict_factory(pubs_lost)
        lost_sub_chans = ConnectionCacheProxy.Channel.dict_factory(subs_lost)
        lost_svc_chans = ConnectionCacheProxy.Channel.dict_factory(svcs_lost)

        # we need to copy the system_state lists to be able to get meaningful diff afterwards
        old_system_state = copy.deepcopy(self._system_state)

        # new system state for services not going to change for actions
        svc_chans = ConnectionCacheProxy.Channel.dict_slaughterhouse(svcs_lost, ConnectionCacheProxy.Channel.dict_factory(svcs_added, old_system_state.services))

        if self.handle_actions:
            # changing new pubs/subs chan to new actionchans
            new_action_servers, added_sub_chans, added_pub_chans =\
                ConnectionCacheProxy.ActionChannel.dict_factory_actions_from_chan(
                    added_sub_chans, added_pub_chans, old_system_state.action_servers
                )

            new_action_clients, added_pub_chans, added_sub_chans =\
                ConnectionCacheProxy.ActionChannel.dict_factory_actions_from_chan(
                    added_pub_chans, added_sub_chans, old_system_state.action_clients
                )

            # changing lost pubs/subs chans to lost actionchans
            new_action_servers, lost_sub_chans, lost_pub_chans =\
                ConnectionCacheProxy.ActionChannel.dict_slaughterhouse_actions_from_chan(
                    lost_sub_chans, lost_pub_chans, new_action_servers
                )

            new_action_clients, lost_pub_chans, lost_sub_chans =\
                ConnectionCacheProxy.ActionChannel.dict_slaughterhouse_actions_from_chan(
                    lost_pub_chans, lost_sub_chans, new_action_clients
                )

            # recalculating difference after actions
            _added_system_state = self.SystemState(
                added_pub_chans,
                added_sub_chans,
                added_svc_chans,
                {n: c for n, c in new_action_servers.iteritems() if not (n in self._system_state.action_servers.keys() and self._system_state.action_servers[n] == c)},
                {n: c for n, c in new_action_clients.iteritems() if not (n in self._system_state.action_clients.keys() and self._system_state.action_clients[n] == c)},
            )

            _lost_system_state = self.SystemState(
                lost_pub_chans,
                lost_sub_chans,
                lost_svc_chans,
                {n: c for n, c in self._system_state.action_servers.iteritems() if not (n in new_action_servers.keys() and new_action_servers[n] == c)},
                {n: c for n, c in self._system_state.action_clients.iteritems() if not (n in new_action_clients.keys() and new_action_clients[n] == c)},
            )

            pub_chans = old_system_state.publishers
            pub_chans.update(added_pub_chans)
            pub_chans = {k: v for k, v in pub_chans.iteritems() if k not in lost_pub_chans.keys()}

            sub_chans = old_system_state.subscribers
            sub_chans.update(added_sub_chans)
            sub_chans = {k: v for k, v in sub_chans.iteritems() if k not in lost_sub_chans.keys()}

            self._system_state = self.SystemState(
                pub_chans,
                sub_chans,
                svc_chans,
                new_action_servers,
                new_action_clients,
            )

        else:
            pub_chans = ConnectionCacheProxy.Channel.dict_slaughterhouse(pubs_lost, ConnectionCacheProxy.Channel.dict_factory(pubs_added, old_system_state.publishers))
            sub_chans = ConnectionCacheProxy.Channel.dict_slaughterhouse(subs_lost, ConnectionCacheProxy.Channel.dict_factory(subs_added, old_system_state.subscribers))

            _added_system_state = self.SystemState(added_pub_chans, added_sub_chans, added_svc_chans)
            _lost_system_state = self.SystemState(lost_pub_chans, lost_sub_chans, lost_svc_chans)
            self._system_state = self.SystemState(pub_chans, sub_chans, svc_chans)

        self._system_state_lock.release()
        # rospy.loginfo("CACHE PROXY LIST_CB PUBLISHERS : {pubs}".format(pubs=self._system_state.publishers))
        # rospy.loginfo("CACHE PROXY LIST_CB SUBSCRIBERS : {subs}".format(subs=self._system_state.subscribers))
        # rospy.loginfo("CACHE PROXY LIST_CB SERVICES : {svcs}".format(svcs=self._system_state.services))

        if self.user_cb is not None and hasattr(self.user_cb, '__call__'):
            try:
                self.user_cb(self._system_state, _added_system_state, _lost_system_state)
            except Exception as user_exc:
                rospy.logerr("Connection Cache Proxy : Diff Callback Exception {0}".format(user_exc))

        pass

    def getSystemState(self):
        # ROSmaster system_state format
        self._system_state_lock.acquire()  # block in case we re changing it at the moment
        rosmaster_ss = (
            [[name, [n[0] for n in self._system_state.publishers[name].nodes]] for name in self._system_state.publishers],
            [[name, [n[0] for n in self._system_state.subscribers[name].nodes]] for name in self._system_state.subscribers],
            [[name, [n[0] for n in self._system_state.services[name].nodes]] for name in self._system_state.services],
        )
        self._system_state_lock.release()
        return rosmaster_ss

    def getTopicTypes(self):
        # ROSmaster system_state format
        self._system_state_lock.acquire()  # block in case we re changing it at the moment
        # building set of tuples to enforce unicity
        pubset = {(name, chan.type) for name, chan in self._system_state.publishers.iteritems()}
        subset = {(name, chan.type) for name, chan in self._system_state.subscribers.iteritems()}
        rosmaster_tt = [list(t) for t in (pubset | subset)]
        self._system_state_lock.release()
        return rosmaster_tt

    # Completing Master API
    def getServiceTypes(self):
        # ROSmaster system_state format
        self._system_state_lock.acquire()  # block in case we re changing it at the moment
        # building set of tuples to enforce unicity
        svcset = {(name, chan.type) for name, chan in self._system_state.services.iteritems()}
        rosmaster_st = [list(t) for t in svcset]
        self._system_state_lock.release()
        return rosmaster_st

    # Completing Master API
    def getServiceUris(self):
        # ROSmaster system_state format
        self._system_state_lock.acquire()  # block in case we re changing it at the moment
        # building set of tuples to enforce unicity
        svcset = {(name, chan.xmlrpc_uri) for name, chan in self._system_state.services.iteritems()}
        rosmaster_su = [list(t) for t in svcset]
        self._system_state_lock.release()
        return rosmaster_su
