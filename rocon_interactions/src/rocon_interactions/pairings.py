#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: pairings
   :platform: Unix
   :synopsis: Support classes for pairing interactions


This module provides classes and utilities for dealing with pairing style
interactions.
----

"""
##############################################################################
# Imports
##############################################################################

import rocon_console.console as console

##############################################################################
# Class
##############################################################################


class RuntimePairingSignature(object):
    """
    Signature identifying a runtime pairing interaction.

    :ivar interaction: the interaction
    :vartype interaction: rocon_interactions.interactions.Interaction
    :ivar remocon_name: name of the remocon that initiated the pairing interaction
    :vartype remocon_name: str
    """
    def __init__(self, interaction, remocon_name):
        self.interaction = interaction
        self.remocon_name = remocon_name

    def __str__(self):
        s = ""
        s += console.green + "%s" % self.interaction.display_name + console.reset + "-"
        s += console.cyan + "%s" % self.interaction.pairing.rapp + console.reset + "-"
        s += console.yellow + "%s" % self.remocon_name + console.reset
        return s
