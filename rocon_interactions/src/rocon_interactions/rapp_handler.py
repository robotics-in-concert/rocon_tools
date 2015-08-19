#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Module
##############################################################################

"""
.. module:: rapp_handler
   :platform: Unix
   :synopsis: Works with the rocon app manager to support pairing interactions.

This module provides a class that can be used by other nodes to introspect
and start/stop rapps on a rapp manager running on the same ros master.

"""

##############################################################################
# Imports
##############################################################################

import rospy
import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs
import rocon_python_comms
import threading
from .exceptions import(
                        FailedToStartRappError,
                        FailedToStopRappError,
                        FailedToListRappsError,
                        )
from .rapp_watcher import RappWatcher

##############################################################################
# Utilities
##############################################################################


def rapp_msg_to_dict(rapp):
    dict_rapp = {}
    dict_rapp["status"] = rapp.status
    dict_rapp["name"] = rapp.name
    dict_rapp["display_name"] = rapp.display_name
    dict_rapp["description"] = rapp.description
    dict_rapp["compatibility"] = rapp.compatibility
    dict_rapp["preferred"] = rapp.preferred
    dict_rapp["icon"] = rapp.icon
    dict_rapp["implementations"] = rapp.implementations
    dict_rapp["public_interface"] = rapp.public_interface
    dict_rapp["public_parameters"] = rapp.public_parameters
    return dict_rapp


def rapp_list_msg_to_dict(list_rapp):
    dict_rapp = {}
    for rapp in list_rapp:
        name = rapp.name
        dict_rapp[name] = rapp_msg_to_dict(rapp)
    return dict_rapp

##############################################################################
# Classes
##############################################################################


class RappHandler(object):
    """
    Initialises from a conductor message detailing information about a
    concert client. Once established, this instance can be used as
    a convenience to start and stop rapps on the concert client.

    :ivar is_running: flag indicating if there is a monitored rapp running on the rapp manager.
    :vartype is_running: bool

    """
    def __init__(self, status_callback, rapp_running_callback):
        """
        Initialise the class with the relevant data required to start and stop
        rapps on this concert client.
        """
        self._running_rapp = None
        self._available_rapps = {}
        self.subscribers = rocon_python_comms.utils.Publishers(
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
    def initialised(self):
        return len(self._available_rapps) != 0

    @property
    def is_running(self):
        return self._running_rapp is not None

    @property
    def running_rapp(self):
        return self._running_rapp

    def is_running_rapp(self, rapp_name):
        try:
            return self._running_rapp['name'] == rapp_name
        except AttributeError:  # i.e. it is None
            return False

    def is_available_rapp(self, rapp_name):
        return True if rapp_name in self._available_rapps().keys() else False

    def start(self, rapp, remappings):
        """
        Start the rapp with the specified remappings.

        :param str rapp: ros package resource name of the rapp to start (e.g. rocon_apps/teleop)
        :param remappings: remappings to apply to the rapp when starting.
        :type remappings: [rocon_std_msgs.Remapping]

        .. include:: weblinks.rst

        :raises: :exc:`.FailedToStartRappError`
        """
        if not self.initialised:
            raise FailedToStartRappError("rapp manager's location unknown")
        try:
            unused_response = self.service_proxies.start_rapp(rocon_app_manager_srvs.StartRappRequest(name=rapp, remappings=remappings))
            # todo check this response and process it
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStartRappError("%s" % str(e))

    def stop(self):
        """
        Stop a rapp on this concert client (if one should be running). This
        doesn't need a rapp specification since only one rapp can ever be
        running - it will just stop the currently running rapp.

        :raises: :exc:`.FailedToStopRappError`
        """
        if not self.initialised:
            raise FailedToStopRappError("rapp manager's location not known")
        try:
            unused_response = self.service_proxies.stop_rapp(rocon_app_manager_srvs.StopRappRequest())
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStopRappError("%s" % str(e))

    def initialise(self):
        """
        Loops around (indefinitely) until it makes a connection with the rapp manager and retrieves the rapp list.
        """
        rate = rospy.Rate(1)  # hz
        # get the rapp list - just loop around until catch it once - it is not dynamically changing
        while not rospy.is_shutdown():
            # msg is rocon_app_manager_msgs/RappList
            msg = rocon_python_comms.SubscriberProxy('~rapp_list', rocon_app_manager_msgs.RappList)(rospy.Duration(0.1))
            if msg is None:
                rospy.logwarn("Interactions : unable to connect with the rocon app manager (wrong remappings?).")
                try:
                    rate.sleep()
                except rospy.exceptions.ROSInterruptException:
                    continue  # ros is shutting down, catch it on the next loop
            else:
                self._available_rapps = rapp_list_msg_to_dict(msg.available_rapps)
                return

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
#             for pubif_idx, pubif in enumerate(running_rapp['public_interface']):
#                 newvals = ast.literal_eval(pubif.value)
#                 for msgif in msg.published_interfaces:
#                     if (
#                         (pubif.key == 'subscribers' and msgif.interface.connection_type == 'subscriber') or
#                         (pubif.key == 'publishers' and msgif.interface.connection_type == 'publisher')
#                     ):
#                         # rospy.loginfo('newvals %r', newvals)
#                         for newval_idx, newval in enumerate(newvals):
#                             # rospy.loginfo('newvals[%r] %r', newval_idx, newval)
#                             if newval['name'] == msgif.interface.name and newval['type'] == msgif.interface.data_type:
#                                 # Careful we re changing the list in place here
#                                 newvals[newval_idx]['name'] = msgif.name
#                                 # rospy.loginfo('newvals[%r] -> %r', newval_idx, newval)
#                                 # Careful we re changing the list in place here
#                     elif msgif.interface.connection_type not in rocon_python_comms.connections.connection_types:
#                         rospy.logerr('Interactions : unsupported connection type : %r', msgif.interface.connection_type)
#                 # Careful we re changing the list in place here
#                 running_rapp['public_interface'][pubif_idx].value = str(newvals)
#             TODO : same for published parameters ?
            self._running_rapp = running_rapp
            # rospy.loginfo('Interactions : new public if : %r', self._running_rapp['public_interface'])
        elif msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_STOPPED:
            self._running_rapp = None

    def _running_rapp_status_change(self, namespace, rapp_status, rapp):
        self.rapp_running_callback(rapp_status, rapp)

    def _ros_status_subscriber(self, msg):
        """
        Relay a notification to the status callback function if we detect that a rapp has
        stopped on the rapp manager. This is to let the higher level disable pairing mode if
        it is the rapp manager's rapp naturally terminating rather than the user terminating
        from the user's side.
        """
        old_running_status = self.is_running
        self.is_running = (msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_RUNNING)
        if old_running_status and msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_STOPPED:
            self.status_callback()  # let the higher level disable pairing mode via this
