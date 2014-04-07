#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rocon_console.console as console
import rocon_uri

from . import interactions
from .exceptions import InvalidInteraction

##############################################################################
# Classes
##############################################################################


class InteractionsTable(object):
    '''
      The runtime populated interactions table along with methods to
      manipulate it.
    '''
    __slots__ = [
        'interactions',  # rocon_interactions.interactions.Interaction[]
    ]

    def __init__(self):
        self.interactions = []

    def roles(self):
        '''
          List all roles for the currently stored interactions.

          @return a list of all roles
          @rtype str[]
        '''
        # uniquify the set
        return list(set([i.role for i in self.interactions]))

    def __len__(self):
        return len(self.interactions)

    def __str__(self):
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

          Note: there's got to be a faster way of doing this.

          @return A role based view of the interactions
          @rtype dict role(str) : interactions.Interaction[]
        '''
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

          @param roles : a list of roles to filter against, use all roles if None
          @type str[]

          @param compatibility_uri : compatibility rocon uri, eliminates interactions that don't match this uri.
          @type str (rocon_uri)

          @return interactions : subset of all interactions that survived the filter
          @rtype interactions.Interactions[]

          @raise RoconURIValueError
        '''
        if roles:   # works for classifying non-empty list vs either of None or empty list
            role_filtered_interactions = [i for i in self.interactions if i.role in roles]
        else:
            role_filtered_interactions = list(self.interactions)
        filtered_interactions = [i for i in role_filtered_interactions
                                 if rocon_uri.is_compatible(i.compatibility, compatibility_uri)]
        return filtered_interactions

    def load(self, msgs):
        '''
          Load some interactions into the interaction table. This involves some initialisation
          and validation steps.

          @param msgs : a list of interaction specifications to populate the table with.
          @type rocon_interaction_msgs.msg.Interaction[]

          @return list of all additions and any that were flagged as invalid
          @rtype (interactions.Interaction[], rocon_interaction_msgs.msg..Interaction[]) : (new, invalid)
        '''
        new = []
        invalid = []
        for msg in msgs:
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

          @param msgs : a list of interactions
          @type rocon_interaction_msgs.msg..Interaction[]

          @return a list of removed interactions
          @rtype rocon_interaction_msgs.msg..Interaction[]
        '''
        removed = []
        for msg in msgs:
            msg_hash = interactions.generate_hash(msg.name, msg.role, msg.namespace)
            found = self.find(msg_hash)
            if found is not None:
                removed.append(msg)
                self.interactions.remove(found)
        return removed

    def find(self, interaction_hash):
        '''
          Find the specified interaction.

          @param interaction_hash in crc32 format
          @type str

          @return interaction if found, None otherwise.
          @rtype rocon_interactions.Interaction
        '''
        interaction = next((interaction for interaction in self.interactions
                            if interaction.hash == interaction_hash), None)
        return interaction
