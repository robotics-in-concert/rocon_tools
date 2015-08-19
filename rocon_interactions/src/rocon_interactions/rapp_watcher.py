# -*- coding: utf-8 -*-
from __future__ import absolute_import

import threading

import rospy

import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs

import rocon_python_comms
import ast


class RappWatcher(object):

    def __init__(self, running_rapp_status_change_cb, silent_timeout=False):
        """
        @param rapp_status_change_cb : callback for a change of rapp status
        """
        super(RappWatcher, self).__init__()
        self._running_rapp = None
        self._available_rapps = []

        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                ('~status', rocon_app_manager_msgs.Status, self._status_subscriber_callback),
            ]
        )
        self.service_proxies = rocon_python_comms.utils.ServiceProxies(
            [
                ('~start_rapp', rocon_app_manager_srvs.StartRapp),
                ('~stop_rapp', rocon_app_manager_srvs.StopRapp),
            ]
        )
        self._initialising_thread = threading.Thread(target=self.initialise)
        self._initialising_thread.start()

    @property
    def available_rapps(self):
        return self._available_rapps

    @property
    def running_rapp(self):
        return self._running_rapp

    def _status_subscriber_callback(self, msg):
        """
        Update the current status of the rapp manager
        @param rocon_app_manager_msgs/Status msg: detailed report of the rapp manager's current state
        """
        if msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_RUNNING:
            # FIXME : This should probably be done internally in the app_manager
            # => A we only need the published interface from a running app, without caring about the original specification
            # => Is this statement always true ?
            running_rapp = rapp_msg_to_dict(msg.rapp)
            for pubif_idx, pubif in enumerate(running_rapp['public_interface']):
                newvals = ast.literal_eval(pubif.value)
                for msgif in msg.published_interfaces:
                    if (
                        (pubif.key == 'subscribers' and msgif.interface.connection_type == 'subscriber') or
                        (pubif.key == 'publishers' and msgif.interface.connection_type == 'publisher')
                    ):
                        # rospy.loginfo('newvals %r', newvals)
                        for newval_idx, newval in enumerate(newvals):
                            # rospy.loginfo('newvals[%r] %r', newval_idx, newval)
                            if newval['name'] == msgif.interface.name and newval['type'] == msgif.interface.data_type:
                                # Careful we re changing the list in place here
                                newvals[newval_idx]['name'] = msgif.name
                                # rospy.loginfo('newvals[%r] -> %r', newval_idx, newval)
                                # Careful we re changing the list in place here
                    elif msgif.interface.connection_type not in rocon_python_comms.connections.connection_types:
                        rospy.logerr('Interactions : unsupported connection type : %r', msgif.interface.connection_type)
                # Careful we re changing the list in place here
                running_rapp['public_interface'][pubif_idx].value = str(newvals)

            # TODO : same for published parameters ?

            self._running_rapp = running_rapp
            # rospy.loginfo('Interactions : new public if : %r', self._running_rapp['public_interface'])

        elif msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_STOPPED:
            self._running_rapp = None

        # callback
        self.running_rapp_status_change_cb(msg.rapp_status, self._running_rapp)

    def initialise(self):
        """
        Loops around (indefinitely) until it makes a connection with the rapp manager and retrieves the rapp list.
        """
        rate = rospy.Rate(1)  # hz
        # get the rapp list - just loop around until catch it once - it is not dynamically changing
        while not rospy.is_shutdown():
            # msg is rocon_app_manager_msgs/RappList
            msg = rocon_python_comms.SubscriberProxy('~rapp_list', rocon_app_manager_msgs.RappList)()
            if msg is None:
                rapp_list_subname = rospy.resolve_name('~rapp_list')
                rospy.logwarn("Interactions : unable to connect with the rocon app manager : {0} not found.".format(rapp_list_subname))
                try:
                    rate.sleep()
                except rospy.exceptions.ROSInterruptException:
                    continue  # ros is shutting down, catch it on the next loop
            else:
                self._available_rapps = rapp_list_msg_to_dict(msg.available_rapps)
                return

    def start_rapp(self, start_rapp_request):
        return self.service_proxies.start_rapp(start_rapp_request)

    def stop_rapp(self, namespace, stop_rapp_request):
        return self.service_proxies.stop_rapp(stop_rapp_request)
