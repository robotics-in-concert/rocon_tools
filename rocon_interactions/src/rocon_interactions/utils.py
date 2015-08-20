#
# License: BSD
#   https://raw.github.com/robotics-in-py/rocon_app_platform/license/LICENSE
#
##############################################################################

import rospy
import roslaunch.parent

##############################################################################
# Description
##############################################################################

"""
.. module:: utils
   :platform: Unix
   :synopsis: Utilities supporting the interactions classes.


This module defines utilities supporting interactions functionality.
----
"""
##############################################################################
# Imports
##############################################################################

import rocon_interaction_msgs.msg as interaction_msgs
import rocon_interaction_msgs.srv as interaction_srvs

##############################################################################
# Request Interactions Resopnse
##############################################################################

_request_interaction_error_messages = {
    interaction_msgs.ErrorCodes.SUCCESS: 'Success',
    interaction_msgs.ErrorCodes.INTERACTION_UNAVAILABLE: interaction_msgs.ErrorCodes.MSG_INTERACTION_UNAVAILABLE,
    interaction_msgs.ErrorCodes.INTERACTION_QUOTA_REACHED: interaction_msgs.ErrorCodes.MSG_INTERACTION_QUOTA_REACHED,
    interaction_msgs.ErrorCodes.ALREADY_PAIRING: interaction_msgs.ErrorCodes.MSG_START_PAIRED_RAPP_FAILED,
    interaction_msgs.ErrorCodes.START_PAIRED_RAPP_FAILED: interaction_msgs.ErrorCodes.MSG_START_PAIRED_RAPP_FAILED,
    interaction_msgs.ErrorCodes.REQUIRED_RAPP_IS_NOT_RUNNING: interaction_msgs.ErrorCodes.MSG_REQUIRED_RAPP_IS_NOT_RUNNING,
    interaction_msgs.ErrorCodes.DIFFERENT_RAPP_IS_RUNNING: interaction_msgs.ErrorCodes.MSG_DIFFERENT_RAPP_IS_RUNNING
}


def generate_request_interaction_response(code):
    """
    Construct according to the incomign code a default response message for the request interactions service.

    :param int code: one of the interaction_msgs.ErrorCodes types relevant to the set interactions service.
    :return: the response, filled with result code and message.
    :rtype: interaction_srvs.RequestInteractionResponse
    """
    response = interaction_srvs.RequestInteractionResponse()
    response.error_code = code
    response.message = _request_interaction_error_messages[code]
    response.result = True if code == interaction_msgs.ErrorCodes.SUCCESS else False
    return response
