RosApi
======

These are the specifications for the ros api of the interaction node. This is
of interest for people using the api to (un)load interactions and also for the
remocon implementation developer.

Published Topics
----------------

 * ``info`` (`rocon_std_msgs`_/MasterInfo) : indirectly used via `rocon_master_info`_ to publish the interaction master's details to remocons, latched.
 * ``~interactive_clients`` (`rocon_interaction_msgs`_/InteractiveClients) : introspect the list of connected remocons, latched.
 * ``~roles`` (`rocon_interaction_msgs`_/Roles) : introspect the currently loaded roles.

Services
--------

 * ``~get_interaction`` (`rocon_interaction_msgs`_/GetInteraction) : used by the android headless launcher to get a single interactions details by hash.
 * ``~get_interactions`` (`rocon_interaction_msgs`_/GetInteractions) : used by all remocons to get a filtered set of compatible interactions by role and `rocon_uri`_.
 * ``~set_interactions`` (`rocon_interaction_msgs`_/SetInteractions) : used to load and unload interactions on the interaction manager node.

.. _`rocon_interaction_msgs`: http://wiki.ros.org/rocon_interaction_msgs
.. _`rocon_master_info`: http://wiki.ros.org/rocon_master_info
.. _`rocon_std_msgs`: http://wiki.ros.org/rocon_std_msgs
.. _`rocon_uri`: http://wiki.ros.org/rocon_uri

Parameters
----------

 * ``~rosbridge_address`` : if serving web apps, pop the rosbridge address here (default: localhost).
 * ``~rosbridge_address`` : if serving web apps, pop the rosbridge port here (default: 9090).
  