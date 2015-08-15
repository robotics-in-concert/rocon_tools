#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
##############################################################################
# Description
##############################################################################

"""
.. module:: utils
   :platform: Unix
   :synopsis: Utilities for working with python communications in ros.


Convenience utilities for ros 1.0 python communications.
----

"""

##############################################################################
# Imports
##############################################################################

import rospy
from . import namespace

##############################################################################
# Classes
##############################################################################


class Services:
    def __init__(self, services):
        """
        Converts the incoming list of service name, service type, callback function triples into proper variables of this class.

        :param services: incoming list of service specifications
        :type services: list of (str, str, function) tuples representing (service_name, service_type, callback) pairs.
        """
        self.__dict__ = {namespace.basename(service_name): rospy.Service(service_name, service_type, callback) for (service_name, service_type, callback) in services}


class Publishers:
    def __init__(self, publishers):
        """
        Converts the incoming list of publisher name, type, latched, queue_size specifications into proper variables of this class.

        :param publishers: incoming list of service specifications
        :type publishers: list of (str, str, bool, int) tuples representing (topic_name, publisher_type, latched, queue_size) specifications.
        """
        self.__dict__ = {namespace.basename(topic_name): rospy.Publisher(topic_name, publisher_type, latch=latched, queue_size=queue_size) for (topic_name, publisher_type, latched, queue_size) in publishers}


class Subscribers:
    def __init__(self, subscribers):
        """
        Converts the incoming list of publisher name, service type pairs into proper variables of this class.

        :param subscribers: incoming list of service specifications
        :type subscribers: list of (str, str, bool, int) tuples representing (topic_name, subscriber_type, latched, queue_size) specifications.
        """
        self.__dict__ = {namespace.basename(topic_name): rospy.Publisher(topic_name, subscriber_type, callback) for (topic_name, subscriber_type, callback) in subscribers}
