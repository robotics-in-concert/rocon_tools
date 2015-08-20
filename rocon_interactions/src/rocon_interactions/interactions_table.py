#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: interactions_table
   :platform: Unix
   :synopsis: A database of interactions.


This module provides a class that acts as a database (dictionary style) of
some set of interactions.

----

"""
##############################################################################
# Imports
##############################################################################

import re
import rocon_console.console as console
import rocon_uri
import rospy

from . import interactions
from .exceptions import InvalidInteraction, MalformedInteractionsYaml, YamlResourceNotFoundException
from .ros_parameters import Bindings

##############################################################################
# Classes
##############################################################################


class InteractionsTable(object):
    '''
      The runtime populated interactions table along with methods to
      manipulate it.

      .. include:: weblinks.rst

      :ivar interactions: list of objects that form the elements of the table
      :vartype interactions: rocon_interactions.interactions.Interaction[]
      :ivar filter_pairing_interactions: flag for indicating whether pairing interactions should be filtered when loading.
      :vartype filter_pairing_interactions: bool
      :ivar bindings: special symbols that can be substituted into the interactions specification
      :vartype rocon_interactions.ros_parameters.Bindings
    '''
    def __init__(self,
                 filter_pairing_interactions=False
                 ):
        """
        Constructs an empty interactions table.

        :param bool filter_pairing_interactions: do not load any paired interactions
        """
        self.interactions = []
        self.filter_pairing_interactions = filter_pairing_interactions
        self.bindings = Bindings()

    def roles(self):
        '''
          List all roles for the currently stored interactions.

          :returns: a list of all roles
          :rtype: str[]
        '''
        # uniquify the list
        return list(set([i.role for i in self.interactions]))

    def __len__(self):
        return len(self.interactions)

    def __str__(self):
        """
        Convenient string representation of the table.
        """
        s = ''
        role_view = self.generate_role_view()
        for role, interactions in role_view.iteritems():
            s += console.bold + role + console.reset + '\n'
            for interaction in interactions:
                s += "\n".join("  " + i for i in str(interaction).splitlines()) + '\n'
        return s

    def generate_role_view(self):
        '''
          Creates a temporary copy of the interactions and sorts them into a dictionary
          view classified by role.

          :returns: A role based view of the interactions
          :rtype: dict { role(str) : :class:`.interactions.Interaction`[] }
        '''
        # there's got to be a faster way of doing this.
        interactions = list(self.interactions)
        role_view = {}
        for interaction in interactions:
            if interaction.role not in role_view.keys():
                role_view[interaction.role] = []
            role_view[interaction.role].append(interaction)
        return role_view

    def filter(self, roles=None, compatibility_uri='rocon:/'):
        '''
          Filter the interactions in the table according to role and/or compatibility uri.

          :param roles: a list of roles to filter against, use all roles if None
          :type roles: str []
          :param str compatibility_uri: compatibility rocon_uri_, eliminates interactions that don't match this uri.

          :returns interactions: subset of all interactions that survived the filter
          :rtype: :class:`.Interaction` []

          :raises: rocon_uri.RoconURIValueError if provided compatibility_uri is invalid.
        '''
        if roles:   # works for classifying non-empty list vs either of None or empty list
            role_filtered_interactions = [i for i in self.interactions if i.role in roles]
        else:
            role_filtered_interactions = list(self.interactions)
        filtered_interactions = [i for i in role_filtered_interactions
                                 if rocon_uri.is_compatible(i.compatibility, compatibility_uri)]
        return filtered_interactions

    def load_from_resources(self, interactions_resource_list):
        """
        :param [str] interactiosn_resource_list: list of ros `resource names`_

        .. _resource names: http://wiki.ros.org/Names#Package_Resource_Names
        """
        for resource_name in interactions_resource_list:
            try:
                msg_interactions = interactions.load_msgs_from_yaml_resource(resource_name)
                (new_interactions, invalid_interactions) = self.load(msg_interactions)
                for i in new_interactions:
                    rospy.loginfo("Interactions : loading %s [%s-%s-%s]" %
                                  (i.display_name, i.name, i.role, i.namespace))
                for i in invalid_interactions:
                    rospy.logwarn("Interactions : failed to load %s [%s-%s-%s]" %
                                  (i.display_name, i.name, i.role, i.namespace))
            except YamlResourceNotFoundException as e:
                rospy.logerr("Interactions : failed to load resource %s [%s]" %
                             (resource_name, str(e)))
            except MalformedInteractionsYaml as e:
                rospy.logerr("Interactions : pre-configured interactions yaml malformed [%s][%s]" %
                             (resource_name, str(e)))

    def load(self, msgs):
        '''
          Load some interactions into the table. This involves some initialisation
          and validation steps.

          :param msgs: a list of interaction specifications to populate the table with.
          :type msgs: rocon_interaction_msgs.Interaction_ []
          :returns: list of all additions and any that were flagged as invalid
          :rtype: (:class:`.Interaction` [], rocon_interaction_msgs.Interaction_ []) : (new, invalid)
        '''
        msgs = self._bind_dynamic_symbols(msgs)
        new = []
        invalid = []
        for msg in msgs:
            # paired check
            if self.filter_pairing_interactions and msg.pairing.rapp:
                invalid.append(msg)
            else:
                try:
                    interaction = interactions.Interaction(msg)
                    self.interactions.append(interaction)
                    self.interactions = list(set(self.interactions))  # uniquify the list, just in case
                    new.append(interaction)
                except InvalidInteraction:
                    invalid.append(msg)
        return new, invalid

    def unload(self, msgs):
        '''
          Removed the specified interactions interactions table. This list is typically
          the same list as the user might initially send - no hashes yet generated.

          :param msgs: a list of interactions
          :type msgs: rocon_interaction_msgs.Interaction_ []

          :returns: a list of removed interactions
          :rtype: rocon_interaction_msgs.Interaction_ []
        '''
        removed = []
        for msg in msgs:
            msg_hash = interactions.generate_hash(msg.display_name, msg.role, msg.namespace)
            found = self.find(msg_hash)
            if found is not None:
                removed.append(msg)
                self.interactions.remove(found)
        return removed

    def find(self, interaction_hash):
        '''
          Find the specified interaction.

          :param str interaction_hash: in crc32 format

          :returns: interaction if found, None otherwise.
          :rtype: :class:`.Interaction`
        '''
        interaction = next((interaction for interaction in self.interactions
                            if interaction.hash == interaction_hash), None)
        return interaction

    def _bind_dynamic_symbols(self, interaction_msgs):
        '''
          Provide some intelligence to the interactions specification by binding designated
          symbols at runtime.

          - interaction.name - __WEBSERVER_ADDRESS__
          - interaction.compatibility - __ROSDISTRO__ (depracated - use | in the compatibility variable itself)
          - interaction.parameters - __ROSBRIDGE_ADDRESS__
          - interaction.parameters - __ROSBRIDGE_PORT__

          :param interaction_msgs: parse this interaction scanning and replacing symbols.
          :type interaction_msgs: rocon_interactions_msgs.Interaction[]

          :returns: the updated interaction list
          :rtype: rocon_interactions_msgs.Interaction[]
        '''

        # Binding runtime CONSTANTS
        for interaction in interaction_msgs:
            interaction.name = interaction.name.replace('__WEBSERVER_ADDRESS__', self.bindings.webserver_address)
            interaction.parameters = interaction.parameters.replace('__ROSBRIDGE_ADDRESS__',
                                                                    self.bindings.rosbridge_address)
            interaction.parameters = interaction.parameters.replace('__ROSBRIDGE_PORT__',
                                                                    str(self.bindings.rosbridge_port))
            # interaction.compatibility = interaction.compatibility.replace('%ROSDISTRO%',
            #                                                              rocon_python_utils.ros.get_rosdistro())

        # search for patterns of the form '<space>__PARAMNAME__,'
        # and if found, look to see if there is a rosparam matching that pattern that we can substitute
        pattern = '\ __(.*?)__[,|\}]'
        for interaction in interaction_msgs:
            for p in re.findall(pattern, interaction.parameters):
                if p.startswith('/'):
                    rparam = rospy.get_param(p)
                elif p.startswith('~'):
                    msg = '%s is invalid format for rosparam binding. See https://github.com/robotics-in-concert/rocon_tools/issues/81' % p
                    raise MalformedInteractionsYaml(str(msg))
                else:
                    # See https://github.com/robotics-in-concert/rocon_tools/issues/81 for the rule
                    ns = interaction.namespace
                    if not ns:
                        rparam = rospy.get_param('~' + p)
                    else:
                        rparam = rospy.get_param(ns + '/' + p)
                match = '__' + p + '__'
                interaction.parameters = interaction.parameters.replace(match, str(rparam))
        return interaction_msgs
