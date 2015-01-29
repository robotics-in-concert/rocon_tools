#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: interactions
   :platform: Unix
   :synopsis: Representative class and methods for an *interaction*.


This module defines a class and methods that represent the core of what
an interaction is.

----

"""
##############################################################################
# Imports
##############################################################################

import yaml
import zlib  # crc32
import os

import genpy
import rospkg
import rocon_console.console as console
import rocon_python_utils
import rocon_interaction_msgs.msg as interaction_msgs

from .exceptions import InvalidInteraction, MalformedInteractionsYaml, YamlResourceNotFoundException
from . import web_interactions

##############################################################################
# Utility Methods
##############################################################################


def generate_hash(display_name, role, namespace):
    '''
      Compute a unique hash for this interaction corresponding to the
      display_name-role-namespace triple. We use zlib's crc32 here instead of unique_id because
      of it's brevity which is important when trying to id an interaction by its hash
      from an nfc tag.

      Might be worth checking http://docs.python.org/2.7/library/zlib.html#zlib.crc32 if
      this doesn't produce the same hash on all platforms.

      :param str display_name: the display name of the interaction
      :param str role: the role the interaction is embedded in
      :param str namespace: the namespace in which to embed this interaction

      :returns: the hash
      :rtype: int32
    '''
    return zlib.crc32(display_name + "-" + role + "-" + namespace)


def load_msgs_from_yaml_file(file_path):
    """
      Load interactions from a yaml resource.

      :param str file_path: file path of a yaml formatted interactions file (ext=.interactions).

      :returns: a list of ros msg interaction specifications
      :rtype: rocon_interaction_msgs.Interaction_ []

      :raises: :exc:`.YamlResourceNotFoundException` if yaml is not found.
      :raises: :exc:`.MalformedInteractionsYaml` if yaml is malformed.

      .. include:: weblinks.rst
    """
    interactions = []
    try:
        yaml_filename = file_path
        if not os.path.isfile(yaml_filename):
            raise YamlResourceNotFoundException(str(e))
    except rospkg.ResourceNotFound as e:  # resource not found.
        raise YamlResourceNotFoundException(str(e))
    with open(yaml_filename) as f:
        # load the interactions from yaml into a python object
        interaction_yaml_objects = yaml.load(f)
        # now drop it into message format
        for interaction_yaml_object in interaction_yaml_objects:
            # convert the parameters from a freeform yaml variable to a yaml string suitable for
            # shipping off in ros msgs (where parameters is a string variable)
            if 'parameters' in interaction_yaml_object:  # it's an optional key
                # chomp trailing newlines
                interaction_yaml_object['parameters'] = yaml.dump(interaction_yaml_object['parameters']).rstrip()
            interaction = interaction_msgs.Interaction()
            try:
                genpy.message.fill_message_args(interaction, interaction_yaml_object)
            except genpy.MessageException as e:
                raise MalformedInteractionsYaml(
                    "malformed yaml preventing converting of yaml to interaction msg type [%s]" % str(e))
            interactions.append(interaction)
    return interactions


def load_msgs_from_yaml_resource(resource_name):
    """
      Load interactions from a yaml resource.

      :param str resource_name: pkg/filename of a yaml formatted interactions file (ext=.interactions).

      :returns: a list of ros msg interaction specifications
      :rtype: rocon_interaction_msgs.Interaction_ []

      :raises: :exc:`.YamlResourceNotFoundException` if yaml is not found.
      :raises: :exc:`.MalformedInteractionsYaml` if yaml is malformed.

      .. include:: weblinks.rst
    """
    interactions = []
    try:
        yaml_filename = rocon_python_utils.ros.find_resource_from_string(resource_name, extension='interactions')
        interactions = load_msgs_from_yaml_file(yaml_filename)
        return interactions
    except rospkg.ResourceNotFound as e:  # resource not found.
        raise YamlResourceNotFoundException(str(e))


##############################################################################
# Classes
##############################################################################


class Interaction(object):

    '''
      This class defines an interaction. It does so by wrapping the base
      rocon_interaction_msgs.Interaction_ msg structure with
      a few convenient variables and methods.

      .. include:: weblinks.rst
    '''
    __slots__ = [
        'msg',           # rocon_interaction_msgs.Interaction
    ]

    def __init__(self, msg):
        """
          Validate the incoming fields supplied by the interaction msg
          and populate remaining fields with proper defaults (e.g. calculate the
          unique hash for this interaction). The hash is calculated based on the
          incoming display_name-role-namespace triple.

          :param msg: underlying data structure with fields minimally filled via :func:`.load_msgs_from_yaml_resource`.
          :type msg: rocon_interaction_msgs.Interaction_

          :raises: :exc:`.InvalidInteraction` if the interaction variables were improperly defined (e.g. max = -1)

          .. include:: weblinks.rst
        """
        self.msg = msg
        """Underlying data structure (rocon_interaction_msgs.Interaction_)"""
        if self.msg.max < -1:
            raise InvalidInteraction("maximum instance configuration cannot be negative [%s]" % self.msg.display_name)
        if self.msg.max == 0:
            self.msg.max = 1
        if self.msg.role == '':
            raise InvalidInteraction("role not configured [%s]" % self.msg.display_name)
        if self.msg.icon.resource_name == "":
            self.msg.icon.resource_name = 'rocon_bubble_icons/rocon.png'
        if not self.msg.icon.data:
            try:
                self.msg.icon = rocon_python_utils.ros.icon_resource_to_msg(self.msg.icon.resource_name)
            except rospkg.common.ResourceNotFound as e: # replace with default icon if icon resource is not found.
                self.msg.icon.resource_name = 'rocon_bubble_icons/rocon.png'
                self.msg.icon = rocon_python_utils.ros.icon_resource_to_msg(self.msg.icon.resource_name)
        if self.msg.namespace == '':
            self.msg.namespace = '/'
        self.msg.hash = generate_hash(self.msg.display_name, self.msg.role, self.msg.namespace)
        # some convenient aliases - these should be properties!

    def is_paired_type(self):
        """
        Classify whether this interaction is to be paired with a rapp or not.

        :returns: whether it is a pairing interaction or not
        :rtype: bool
        """
        return True if self.msg.pairing.rapp else False

    ##############################################################################
    # Conveniences
    ##############################################################################

    @property
    def name(self):
        """Executable name for this interaction, can be a roslaunch, rosrunnable, global executable, web url or web app [int]."""
        return self.msg.name

    @property
    def role(self):
        """The group under which this interaction should be embedded [int]."""
        return self.msg.role

    @property
    def compatibility(self):
        """A rocon_uri_ string that indicates what platforms it may run on [int]."""
        return self.msg.compatibility

    @property
    def display_name(self):
        """A human friendly name that also uniquely helps uniquely identify this interaction (you can have more than one configured ``name`` instance) [int]."""
        return self.msg.display_name

    @property
    def description(self):
        return self.msg.description

    @property
    def namespace(self):
        """Default namespace under which ros services and topics should be embedded for this interaction [int]."""
        return self.msg.namespace

    @property
    def max(self):
        """
        Maximum number of instantiations that is permitted (e.g. teleop should only allow 1) [int].
        """
        return self.msg.max

    @property
    def remappings(self):
        return self.msg.remappings

    @property
    def parameters(self):
        return self.msg.parameters

    @property
    def hash(self):
        """A crc32 unique identifier key for this interaction, see also :func:`.generate_hash` [int32]."""
        return self.msg.hash

    @property
    def pairing(self):
        return self.msg.pairing

    def _eq__(self, other):
        if type(other) is type(self):
            return self.msg.hash == other.msg.hash
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        '''
          Format the interaction into a human-readable string.
        '''
        web_interaction = web_interactions.parse(self.msg.name)
        name = self.msg.name if web_interaction is None else web_interaction.url
        s = ''
        s += console.green + "%s" % self.msg.display_name + console.reset + '\n'
        s += console.cyan + "  Name" + console.reset + "         : " + console.yellow + "%s" % name + console.reset + '\n'  # noqa
        s += console.cyan + "  Description" + console.reset + "  : " + console.yellow + "%s" % self.msg.description + console.reset + '\n'  # noqa
        s += console.cyan + "  Icon" + console.reset + "         : " + console.yellow + "%s" % str(self.msg.icon.resource_name) + console.reset + '\n'  # noqa
        s += console.cyan + "  Rocon URI" + console.reset + "    : " + console.yellow + "%s" % self.msg.compatibility + console.reset + '\n'  # noqa
        s += console.cyan + "  Namespace" + console.reset + "    : " + console.yellow + "%s" % self.msg.namespace + console.reset + '\n'  # noqa
        if self.msg.max == -1:
            s += console.cyan + "  Max" + console.reset + "          : " + console.yellow + "infinity" + console.reset + '\n'  # noqa
        else:
            s += console.cyan + "  Max" + console.reset + "          : " + console.yellow + "%s" % self.msg.max + console.reset + '\n'  # noqa
        already_prefixed = False
        for remapping in self.msg.remappings:
            if not already_prefixed:
                s += console.cyan + "  Remappings" + console.reset + "   : " + console.yellow + "%s->%s" % (remapping.remap_from, remapping.remap_to) + console.reset + '\n'  # noqa
                already_prefixed = True
            else:
                s += "               : " + console.yellow + "%s->%s" % (remapping.remap_from, remapping.remap_to) + console.reset + '\n'  # noqa
        if self.msg.parameters != '':
            s += console.cyan + "  Parameters" + console.reset + "   : " + console.yellow + "%s" % self.msg.parameters + console.reset + '\n'  # noqa
        s += console.cyan + "  Hash" + console.reset + "         : " + console.yellow + "%s" % str(self.msg.hash) + console.reset + '\n'  # noqa
        if self.msg.pairing.rapp:
            s += console.cyan + "  Pairing" + console.reset + "      : " + console.yellow + "%s" % str(self.msg.pairing.rapp) + console.reset + '\n'  # noqa
            already_prefixed = False
            for remapping in self.msg.pairing.remappings:
                if not already_prefixed:
                    s += console.cyan + "    Remappings" + console.reset + " : " + console.yellow + "%s->%s" % (remapping.remap_from, remapping.remap_to) + console.reset + '\n'  # noqa
                    already_prefixed = True
                else:
                    s += "               : " + console.yellow + "%s->%s" % (remapping.remap_from, remapping.remap_to) + console.reset + '\n'  # noqa
            already_prefixed = False
            for pair in self.msg.pairing.parameters:
                if not already_prefixed:
                    s += console.cyan + "    Parameters" + console.reset + " : " + console.yellow + "%s-%s" % (pair.key, pair.value) + console.reset + '\n'  # noqa
                    already_prefixed = True
                else:
                    s += "               : " + console.yellow + "%s-%s" % (pair.key, pair.value) + console.reset + '\n'  # noqa
        return s
