^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rocon_uri
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2014-04-09)
------------------
* more precise rocon uri os fields for pc interactions, also added trusty to the os rules.
* Contributors: Daniel Stonier

0.1.2 (2014-04-02)
------------------
* a rocon_uri command line tool.
* Contributors: Daniel Stonier

0.1.0 (2014-03-31)
------------------
* stack changelogs.
* better comments around enbf failures.
* sort the elements alphabetically and in reverse so matching doesn't come
  up short as in `#17 <https://github.com/robotics-in-concert/rocon_tools/issues/17>`_.
* install rule for the rules.
* minor updates.
* typo fix for rocon uri docs.
* documentation for rocon_uri.
* adds robots (robosem, turtlebot) and sorts robots alphabetically
* rocon uri documentation started.
* nosetest for checking thrown exceptions on compatibility checking.
* adding kobuki
* redis, wifi and interactions moved in, python_utils now official.
* rocon uri support for indigo.
* rapp_name -> rapp in line with Resource.msg style
* adjustments to drop heir-part of uri if no concert name.
* Revert "switch wildcard to 'any'"
  This reverts commit 3cad70f34bf988c3bce941ddcae301f476fd8519. Using 'any' just looks really awkward and far less instantly recognisable than *.
* switch wildcard to 'any'
* wildcards moved out of the yaml specification
* add android operating systems.
* rocon_uri now loads from yaml.
* first iteration of full rocon uri class and test program.
* strange test results
* ebnf rule parser moved to its own package.
* adding rule parser package and a test.
* starting rocon_uri
* Contributors: Daniel Stonier, Marcus Liebhardt
