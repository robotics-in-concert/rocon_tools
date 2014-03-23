#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import yaml
import zlib  # crc32

import genpy
import rospkg
import rocon_console.console as console
import rocon_python_utils
import rocon_interaction_msgs.msg as interaction_msgs

from .exceptions import InvalidInteraction, MalformedInteractionsYaml, YamlResourceNotFoundException

##############################################################################
# Utility Methods
##############################################################################


def generate_hash(name, role, namespace):
    '''
      Compute a unique hash for this interaction corresponding to the
      name-role-namespace triple. We use zlib's crc32 here instead of unique_id because
      of it's brevity which is important when trying to id a remocon app by its hash
      from an nfc tag.

      Might be worth checking here http://docs.python.org/2.7/library/zlib.html#zlib.crc32 if
      his doesn't produce the same hash on all platforms.

      :param name: the executable name of the interaction
      :type name: str
      :param role: the role the interaction is embedded in
      :type role: str
      :param namespace: the namespace in which to embed this interaction
      :type namespace: str
    '''
    return zlib.crc32(name + "-" + role + "-" + namespace)


def load_msgs_from_yaml_resource(resource_name):
    """
      Load interactions from a yaml resource.

      :param resource_name: pkg/filename of a yaml formatted interactions file (ext=.interactions).
      :type resource_name: str

      :returns: a list of ros msg interaction specifications
      :rtype: concert_msgs.Interaction[]

      :raises: :exc:`rocon_interactions.YamlResourceNotFoundException,` if yaml is not found.
      :raises: :exc:`rocon_interactions.MalformedInteractionsYaml,` if yaml is malformed.
    """
    interactions = []
    try:
        yaml_filename = rocon_python_utils.ros.find_resource_from_string(resource_name, extension='interactions')
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

##############################################################################
# Classes
##############################################################################


class Interaction(object):
    '''
      Defines an interaction. This wraps the base ros msg data structure with
      a few convenient management handles.
    '''
    __slots__ = [
        'msg',           # rocon_interaction_msgs.Interaction
        # aliases
        'name',
        'compatibility',
        'namespace',
        'display_name',
        'role',
        'hash',
        'max',
    ]

    def __init__(self, msg):
        '''
          Validate the incoming fields and populate remaining fields with sane defaults.
          Also compute a unique hash for this object based on the incoming
          name-role-namespace triple.

          @param msg
          @type rocon_interaction_msgs.Interaction

          @raise exceptions.InvalidInteraction
        '''
        self.msg = msg
        if self.msg.max < -1:
            raise InvalidInteraction("maximum instance configuration cannot be negative [%s]" % self.msg.display_name)
        if self.msg.max == 0:
            self.msg.max = 1
        if self.msg.role == '':
            raise InvalidInteraction("role not configured [%s]" % self.msg.display_name)
        if self.msg.icon.resource_name == "":
            self.msg.icon.resource_name = 'rocon_bubble_icons/rocon.png'
        if not self.msg.icon.data:
            self.msg.icon = rocon_python_utils.ros.icon_resource_to_msg(self.msg.icon.resource_name)
        if self.msg.namespace == '':
            self.msg.namespace = '/'
        self.msg.hash = generate_hash(self.msg.name, self.msg.role, self.msg.namespace)
        # some convenient aliases
        self.role = self.msg.role
        self.name = self.msg.name
        self.namespace = self.msg.namespace
        self.display_name = self.msg.display_name
        self.hash = self.msg.hash
        self.compatibility = self.msg.compatibility
        self.max = self.msg.max

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
        s = ''
        s += console.green + "%s" % self.msg.display_name + console.reset + '\n'
        s += console.cyan + "  Name" + console.reset + "         : " + console.yellow + "%s" % self.msg.name + console.reset + '\n'  # noqa
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
                s += console.cyan + "  Remapping" + console.reset + "    : " + console.yellow + "%s->%s" % (remapping.remap_from, remapping.remap_to) + console.reset + '\n'  # noqa
                already_prefixed = True
            else:
                s += "               : " + console.yellow + "%s->%s" % (remapping.remap_from, remapping.remap_to) + console.reset + '\n'  # noqa
        if self.msg.parameters != '':
            s += console.cyan + "  Parameters" + console.reset + "   : " + console.yellow + "%s" % self.msg.parameters + console.reset + '\n'  # noqa
        s += console.cyan + "  Hash" + console.reset + "         : " + console.yellow + "%s" % str(self.msg.hash) + console.reset + '\n'  # noqa
        return s
