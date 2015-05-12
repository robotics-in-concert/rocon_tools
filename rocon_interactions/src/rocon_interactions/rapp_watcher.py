# -*- coding: utf-8 -*-
from __future__ import absolute_import

import threading

import roslib
import rospy

import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs

from rocon_app_manager_msgs.msg import Status, RappList
from rocon_app_manager_msgs.srv import StartRapp, StopRapp

import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_uri
import ast


def list_rapp_msg_to_dict(list_rapp):
    """
    convert msg to dict
    """
    dict_rapp = {}
    for rapp in list_rapp:
        name = rapp.name
        dict_rapp[name] = {}
        dict_rapp[name]["status"] = rapp.status
        dict_rapp[name]["name"] = rapp.name
        dict_rapp[name]["display_name"] = rapp.display_name
        dict_rapp[name]["description"] = rapp.description
        dict_rapp[name]["compatibility"] = rapp.compatibility
        dict_rapp[name]["preferred"] = rapp.preferred
        dict_rapp[name]["icon"] = rapp.icon
        dict_rapp[name]["implementations"] = rapp.implementations
        dict_rapp[name]["public_interface"] = rapp.public_interface
        dict_rapp[name]["public_parameters"] = rapp.public_parameters
    return dict_rapp


#TODO : find more standard/optimized/"we dont have to maintain it" way of doing this ?
def _dict_key_diff(current_dict, past_dict):
    set_current, set_past = set(current_dict.keys()), set(past_dict.keys())
    intersect = set_current.intersection(set_past)
    added = set_current - intersect
    removed = set_past - intersect
    changed = set(o for o in intersect if past_dict[o] != current_dict[o])
    unchanged = set(o for o in intersect if past_dict[o] == current_dict[o])

    return (dict((el,current_dict[el]) for el in added),
            dict((el,past_dict[el]) for el in removed),
            dict((el,current_dict[el]) for el in changed),
            dict((el,current_dict[el]) for el in unchanged))


class RappWatcher(threading.Thread):

    class WatchedNS():
        def __init__(self, name, rapp_status_change_cb, ns_status_change_cb = lambda : None):

            self.name = name
            self.rapp_status_change_cb = rapp_status_change_cb
            self.ns_status_change_cb = ns_status_change_cb
            self.is_running = False

            self.start_rapp = None
            self.stop_rapp = None
            self.list_rapps = None
            self.status_subscriber = None
            self.rapplist_subscriber = None

            self._available_rapps = {}
            self._running_rapps = {}

        def grab_start_rapp(self, start_rapp_srvs_list):
            if not self.start_rapp :
                for s in start_rapp_srvs_list :
                    if s.startswith(self.name): # if we have found the service name that match our namespace
                        self.start_rapp = rospy.ServiceProxy(s, rocon_app_manager_srvs.StartRapp)
                        return True
                return False # we couldnt grab anything
            return True # we re good, no need for it

        def grab_stop_rapp(self, stop_rapp_srvs_list):
            if not self.stop_rapp :
                for s in stop_rapp_srvs_list :
                    if s.startswith(self.name): # if we have found the service name that match our namespace
                        self.stop_rapp = rospy.ServiceProxy(s, rocon_app_manager_srvs.StopRapp)
                        return True
                return False # we couldnt grab anything
            return True # we re good, no need for it

        def grab_list_rapps(self, list_rapps_srvs_list):
            if not self.list_rapps :
                for s in list_rapps_srvs_list :
                    if s.startswith(self.name): # if we have found the service name that match the namespace
                        self.list_rapps = rospy.ServiceProxy(s, rocon_app_manager_srvs.GetRappList)
                        return True
                return False # we couldnt grab anything
            return True # we re good, no need for it

        def grab_status_subscriber(self, status_topics_list):
            if not self.status_subscriber :
                for t in status_topics_list :
                    if t.startswith(self.name): # if we have found the service name that match the namespace
                        self.status_subscriber = rospy.Subscriber(t, rocon_app_manager_msgs.Status, self._ros_status_subscriber)
                        return True
                return False # we couldnt grab anything
            return True # we re good, no need for it

        def grab_rapplist_subscriber(self, rapplist_topics_list):
            if not self.rapplist_subscriber :
                for t in rapplist_topics_list :
                    if t.startswith(self.name): # if we have found the service name that match the namespace
                        self.rapplist_subscriber = rospy.Subscriber(t, rocon_app_manager_msgs.RappList, self._process_rapp_list_msg)
                        #Latched Topic : First time we connect, we should get the list of rapps
                        return True
                return False # we couldnt grab anything
            return True # we re good, no need for it

        def _process_rapp_list_msg(self, msg):
            """
            Update the available rapp list

            @param data: information of rapps
            @type rocon_app_manager_msgs/RappList
            """

            added_available_rapps, removed_available_rapps, _, _ = _dict_key_diff(list_rapp_msg_to_dict(msg.available_rapps),self._available_rapps)
            added_running_rapps, removed_running_rapps, _, _ = _dict_key_diff(list_rapp_msg_to_dict(msg.running_rapps),self._running_rapps)

            #callback
            self.rapp_status_change_cb( self.name, added_available_rapps, removed_available_rapps, added_running_rapps, removed_running_rapps)

            rospy.logwarn('updating available rapps list : %r', [r.name for r in msg.available_rapps])
            self._available_rapps = list_rapp_msg_to_dict(msg.available_rapps)
            rospy.logwarn('updating running rapps list : %r', [r.name for r in msg.running_rapps])
            self._running_rapps = list_rapp_msg_to_dict(msg.running_rapps)

        def _ros_status_subscriber(self, msg):
            """
            Update the current ns status

            @param msg: information of status
            @type rocon_app_manager_msgs/Status
            """
            # TOCHECK : is this really useful ? duplicated information with app status...
            #callback
            self.ns_status_change_cb(msg)

        @property
        def available_rapps_list(self):#TODO : remove '_list' from name. misleading
            return self._available_rapps

        @property
        def running_rapps_list(self):#TODO : remove '_list' from name. misleading
            return self._running_rapps

    #####class WatchedNS

    def __init__(self, namespaces_change_cb, rapp_status_change_cb, ns_status_change_cb = lambda msg : None, get_rapp_list_service_name = 'list_rapps'):
        """
        @param namespaces_change_cb : callback for a change of namespaces
        @param ns_status_change_cb : callback for a change of namespace status
        @param rapp_status_change_cb : callback for a change of rapp status
        """
        super(RappWatcher, self).__init__()
        #servicename ( to be able to extract namespace )
        self.get_rapp_list_service_name = get_rapp_list_service_name
        #contains data by namespace
        self.watching_ns={}
        # all available namespace detected
        self._available_namespaces = []
        # all namespace that are completely connected
        self._watched_namespaces = []

        #TODO : check callback signature
        self.rapp_status_change_cb = rapp_status_change_cb
        self.namespaces_change_cb = namespaces_change_cb
        self.ns_status_change_cb = ns_status_change_cb

    def run(self):
        rate = rospy.Rate(1) # 1hz
        # Night gathers, and now my watch begins. It shall not end until my death.
        # I shall take no wife, hold no lands, father no children.
        # I shall wear no crowns and win no glory.
        # I shall live and die at my post.
        # I am the sword in the darkness.
        # I am the watcher on the walls.
        # I am the fire that burns against cold, the light that brings the dawn, the horn that wakes the sleepers, the shield that guards the realms of men.
        # I pledge my life and honor to the Night's Watch, for this night and all the nights to come
        while not rospy.is_shutdown():
            rate.sleep() # quick sleep for safety
            try:
                # long timeout because no point of keep going if we cannot find this one.
                list_rapps_service_names = rocon_python_comms.find_service('rocon_app_manager_msgs/GetRappList', timeout=rospy.rostime.Duration(60.0), unique=False)

                # Detecting new namespaces
                ns_added = []
                ns_removed = []

                for ns in self._available_namespaces :
                    found = False
                    for fname in list_rapps_service_names :
                        if fname.startswith(ns) :
                            found = True
                    if not found :
                        ns_removed.append(ns)
                        #stopping to watch removed namespaces
                        if ns in self.watching_ns.keys() :
                            del self.watching_ns[ns]
                            rospy.logerr('STOPPED WATCHING : %r ', ns)

                for fname in list_rapps_service_names :
                    if fname.endswith(self.get_rapp_list_service_name):
                        ns = fname[:-len(self.get_rapp_list_service_name)]
                        if ns not in self._available_namespaces :
                            self._available_namespaces.append(ns)
                            ns_added.append(ns)

                if 0 < len(ns_added) or 0 < len(ns_removed) :
                    to_watch = self.namespaces_change_cb(ns_added, ns_removed)
                    for ns in to_watch :
                        if ns in self._available_namespaces :
                            rospy.logerr('NOW WATCHING FOR RAPPS IN %r ', ns)
                            self.watching_ns[ns] = self.WatchedNS(ns, self.rapp_status_change_cb, self.ns_status_change_cb)

                #grabing all services & topics
                if set(self.watching_ns.keys()) != set(self._watched_namespaces): # if some namespaces are not fully connected
                    start_rapp_service_names = rocon_python_comms.find_service('rocon_app_manager_msgs/StartRapp', timeout=rospy.rostime.Duration(1.0), unique=False)
                    stop_rapp_service_names = rocon_python_comms.find_service('rocon_app_manager_msgs/StopRapp', timeout=rospy.rostime.Duration(1.0), unique=False)
                    status_topic_names = rocon_python_comms.find_topic('rocon_app_manager_msgs/Status', timeout=rospy.rostime.Duration(1.0), unique=False)
                    rapplist_topic_names = rocon_python_comms.find_topic('rocon_app_manager_msgs/RappList', timeout=rospy.rostime.Duration(1.0), unique=False)

                    for ns, ns_data in self.watching_ns.iteritems() :
                        if ns not in self._watched_namespaces : # if that namespace is not fully connected
                            connect = ns_data.grab_start_rapp(start_rapp_service_names)
                            connect = ns_data.grab_stop_rapp(stop_rapp_service_names) and connect
                            connect = ns_data.grab_list_rapps(list_rapps_service_names) and connect
                            connect = ns_data.grab_rapplist_subscriber(rapplist_topic_names) and connect
                            connect = ns_data.grab_status_subscriber(status_topic_names) and connect
                            if connect :
                                #if all services and topics are connected this namespace is considered connected
                                self._watched_namespaces.append(ns)


                #TODO : survive if services/topics ever go down... and be able to catch them again when they come back.

            except rospy.ROSException:
                rospy.logerr("Interactions : rapp manager services disappeared.")
            except rospy.ROSInterruptException:
                rospy.logerr("Interactions : ros shutdown while looking for the rapp manager services.")

    def get_available_rapps(self, namespace):
        if len(self.watching_ns) > 0 : # if we are already watching namespaces
            if not namespace :
                namespace = self.watching_ns.keys()[0] # FIXME hack in case namespace is not passed
            return self.watching_ns[namespace].available_rapps_list
        else :
            return {}

    def get_running_rapps(self, namespace):
        if len(self.watching_ns) > 0 : # if we are already watching namespaces
            if not namespace :
                namespace = self.watching_ns.keys()[0] # FIXME hack in case namespace is not passed
            return self.watching_ns[namespace].running_rapps_list
        else :
            return {}

    def start_rapp(self, namespace, start_rapp_request):
        if len(self.watching_ns) > 0 : # if we are already watching namespaces
            if not namespace :
                namespace = self.watching_ns.keys()[0] # FIXME hack in case namespace is not passed
            return self.watching_ns[namespace].start_rapp( start_rapp_request )
        else :
            return {} #FIXME : return empty result of same type

    def stop_rapp(self, namespace, stop_rapp_request):
        if len(self.watching_ns) > 0 : # if we are already watching namespaces
            if not namespace :
                namespace = self.watching_ns.keys()[0] # FIXME hack in case namespace is not passed
            self.watching_ns[namespace].stop_rapp( stop_rapp_request )
        else :
            return {} #FIXME : return empty result of same type


#####class RappWatcher(threading.Thread)
