^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rocon_interactions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2014-04-09)
------------------
* rocon_uri variable name clashes with module name, fixed.
* get roles moved to a service.
* more precise rocon uri os fields for pc interactions, also added trusty to the os rules.
* catch invalid uri's when filtering.
* bugfix the request interactions hash handling, had legacy msg code still being used.
* Contributors: Daniel Stonier

0.1.1 (2014-04-01)
------------------
* testing dependencies.
* Contributors: Daniel Stonier

0.1.0 (2014-03-31)
------------------
* stack changelogs.
* minor bugfixes to tests.
* documentation for rocon_uri.
* use noqa instead of pydev's custom macro to avoid pep8.
* roslint for rocon_interactions
* install rule for interactions and javascript.
* added parameters
* updated docs with rosapi and interaction specifications.
* rocon icon for listener - chimek doesn't make sense.
* add the web url's to the pc demo
* update rviz launching.
* tutorial setup.
* web apps finalised.
* interactions javascript and listener html.
* added javascript functions to parse interaction queries for web apps.
* allow users to enter yaml structures instead of strings for parameters in interaction yamls.
* interactions formatting for web apps.
* do not add the schema to the rosbridge argument
* a rocon interactions tutorial setup.
* Switch default icon from concert_master -> rocon_bubble_icons.
* publish roles after pre-loading, fixes `#7 <https://github.com/robotics-in-concert/rocon_tools/issues/7>`_.
* icon packs.
* fake remocons test.
* symbol binding for rosbridge parameters, `#6 <https://github.com/robotics-in-concert/rocon_tools/issues/6>`_
* Contributors: Daniel Stonier, Jihoon Lee
