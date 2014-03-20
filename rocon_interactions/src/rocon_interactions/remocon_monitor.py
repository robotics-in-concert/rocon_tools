#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_interaction_msgs.msg as interaction_msgs

##############################################################################
# Remocon Monitor
##############################################################################


class RemoconMonitor(object):
    '''
      Attaches a subscriber to a remocon publisher and monitors the
      status of the remocon.
    '''
    __slots__ = [
        'name',
        'status',  # concert_msgs.RemoconStatus
        '_subscriber',
        '_publish_interactive_clients_callback'  # publishes the list of interactive clients
    ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self, topic_name, publish_interactive_clients_callback):
        if topic_name.startswith(interaction_msgs.Strings.REMOCONS_NAMESPACE + '/'):
            uuid_postfixed_name = topic_name[len(interaction_msgs.Strings.REMOCONS_NAMESPACE) + 1:]
            (self.name, unused_separator, unused_uuid_part) = uuid_postfixed_name.rpartition('_')
        else:
            self.name = 'unknown'  # should raise an error here
            return
        self._subscriber = rospy.Subscriber(topic_name, interaction_msgs.RemoconStatus, self._callback)
        self.status = None
        self._publish_interactive_clients_callback = publish_interactive_clients_callback

    def _callback(self, msg):
        self.status = msg
        # make sure we publish whenever there is a state change (as assumed when we get a status update)
        self._publish_interactive_clients_callback()

    def unregister(self):
        self._subscriber.unregister()
