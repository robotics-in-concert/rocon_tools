#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_std_msgs.srv as rocon_std_srvs
import rospy

##############################################################################
# Main
##############################################################################

spin_freq = 1.0
spin_freq_changed = False


def set_spin_cb(data):
    global spin_freq
    global spin_freq_changed

    if data.spin_freq and not data.spin_freq == spin_freq:  # we change the rate if needed
        spin_freq = data.spin_freq
        spin_freq_changed = True


if __name__ == '__main__':
    rospy.init_node('connection_cache')
    spin_rate = rospy.Rate(1)
    conn_cache = rocon_python_comms.ConnectionCache()

    conn_cache_spin_pub = rospy.Publisher("~spin", rocon_std_msgs.ConnectionCacheSpin, latch=True, queue_size=1)
    conn_cache_spin_sub = rospy.Subscriber("~spin", rocon_std_msgs.ConnectionCacheSpin, set_spin_cb)

    conn_list = rospy.Publisher("~list", rocon_std_msgs.ConnectionsList, latch=True, queue_size=1)  # uptodate full list
    conn_diff = rospy.Publisher("~diff", rocon_std_msgs.ConnectionsDiff, queue_size=1)  # differences only for faster parsing.

    rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
    try:
        spin_rate = rospy.Rate(spin_freq)
        spin_freq_changed = True
        while not rospy.core.is_shutdown():

            # If needed we change our spin rate, and publish the new frequency
            if spin_freq_changed:
                spin_rate = rospy.Rate(spin_freq)
                spinmsg = rocon_std_msgs.ConnectionCacheSpin()
                spinmsg.spin_freq = spin_freq
                spin_freq_changed = False
                conn_cache_spin_pub.publish(spinmsg)

            """
            Update function to call from a looping thread.
            """
            try:
                new_conns, lost_conns = conn_cache.update()
                changed = False

                diff_msg = rocon_std_msgs.ConnectionsDiff()
                list_msg = rocon_std_msgs.ConnectionsList()
                for ct in rocon_python_comms.connection_types_list:
                    if new_conns[ct] or lost_conns[ct]:  # something changed
                        changed = True
                        for c in new_conns[ct]:
                            rocon_python_comms.create_connection(c)
                            diff_msg.added.append(c.msg)
                        for c in lost_conns[ct]:
                            rocon_python_comms.create_connection(c)
                            diff_msg.lost.append(c.msg)
                        for c in conn_cache.connections[ct]:
                            rocon_python_comms.create_connection(c)
                            list_msg.connections.append(c.msg)

                if changed:
                    print "NEW : {0}".format(new_conns)
                    print "LOST : {0}".format(lost_conns)

                    conn_diff.publish(diff_msg)  # new_conns, old_conns
                    conn_list.publish(list_msg)  # conn_cache.connections

            except rospy.ROSException:
                rospy.logerr("ROS Watcher : Connections list unavailable.")
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Watcher : ros shutdown while looking for Connections .")

            spin_rate.sleep()

    except KeyboardInterrupt:
        rospy.logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')

