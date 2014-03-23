#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE

import rosnode
from .exceptions import NotFoundException


def find_node(wanted_node_name, unique=False):
    '''
      Do a lookup to find a node with the given name. The given name is treated as the unresolved node name.
      Hence this lookup will find nodes with the same name, but different namespaces.

      This will raise exceptions, if the node couldn't be found
      or in case unique is set multiple nodes with the same name are found.

      @param wanted_node_name : name of the node looked for
      @type str

      @return the fully resolved name of the node (unique) or list of fully resolved names (non-unique)
      @type str

      @raise rocon_python_comms.NotFoundException
    '''
    available_nodes = rosnode.get_node_names()
    found_nodes = []
    for resolved_node_name in available_nodes:
        node_name = resolved_node_name[(resolved_node_name.rfind('/') + 1):len(resolved_node_name)]
        if node_name == wanted_node_name:
            found_nodes.append(resolved_node_name)

    if len(found_nodes) == 0:
            raise NotFoundException("Node '" + str(wanted_node_name) + "' not found.")
    if unique:
        if len(found_nodes) > 1:
            raise NotFoundException('More then one node with the same name found:' + str(found_nodes))

    return found_nodes
