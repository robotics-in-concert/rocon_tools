#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises
import rocon_uri
import rocon_console.console as console
import rocon_std_msgs.msg as rocon_std_msgs

##############################################################################
# Tests
##############################################################################

# def test_experiments():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Experiments" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
#     rocon_uri_string = 'rocon://'
#     rocon_uri_string = 'rocon:///turtlebot2/dude/hydro/precise#rocon_apps/chirp'
#     rocon_uri_object = rocon_uri.parse(rocon_uri_string)
#     print("Rocon URI Object: %s" % rocon_uri_object)  
#     rocon_uri_object2 = rocon_uri.parse('rocon:///turtlebot2|waiterbot/dude/hydro/precise#rocon_apps/chirp')
#     print("Rocon URI Object: %s" %  str(rocon_uri_object.hardware_platform))
#     print("Rocon URI Object : %s" %  rocon_uri_object.hardware_platform.string)
#     print("Rocon URI Object : %s" %  rocon_uri_object.hardware_platform.list)
#     print("Rocon URI Object2: %s" %  rocon_uri_object2.hardware_platform.string)
#     print("Rocon URI Object2: %s" %  rocon_uri_object2.hardware_platform.list)

def test_invalid_elements():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Raising on invalid elements" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)

    rocon_uri_string = 'rocon:///turtlebot2/dude/hydro/precise#rocon_apps/chirp'

    invalid_schema = rocon_uri_string.replace('rocon', 'http')
    print(console.cyan + " - %s" % invalid_schema + console.reset)
    assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_schema)

    invalid_hardware_platform = rocon_uri_string.replace('turtlebot2', 'foobar')
    print(console.cyan + " - %s" % invalid_hardware_platform + console.reset)
    assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_hardware_platform)

    invalid_application_framework = rocon_uri_string.replace('hydro', 'dont_box_me_in')
    print(console.cyan + " - %s" % invalid_application_framework + console.reset)
    assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_application_framework)
 
    invalid_operating_system = rocon_uri_string.replace('precise', 'bados')
    print(console.cyan + " - %s" % invalid_operating_system + console.reset)
    assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_operating_system)

def test_stringify():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* String representation" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
 
    rocon_uri_string = 'rocon:///turtlebot2/dude/hydro/precise#rocon_apps/chirp'
 
    print(console.cyan + " - %s" % rocon_uri_string + console.reset)
    rocon_uri_object = rocon_uri.parse(rocon_uri_string)
    assert str(rocon_uri_object) == rocon_uri_string
      
def test_multiple_elements():
    '''
      Test the OR functionality.
    '''
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Parsing multiple elements" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
 
    rocon_uri_string = 'rocon:///turtlebot2/dude/hydro/precise#rocon_apps/chirp'
  
    multiple_hardware_platforms = rocon_uri_string.replace('turtlebot2', 'turtlebot2|pr2|waiterbot')
    print(console.cyan + " - %s" % multiple_hardware_platforms + console.reset)
    rocon_uri_object = rocon_uri.parse(multiple_hardware_platforms)
    print(console.yellow + "  - %s" % rocon_uri_object.hardware_platform.list + console.reset)
    assert len(rocon_uri_object.hardware_platform.list) == 3
  
    multiple_operating_systems = rocon_uri_string.replace('precise', 'quantal|precise')
    print(console.cyan + " - %s" % multiple_operating_systems + console.reset)
    rocon_uri_object = rocon_uri.parse(multiple_operating_systems)
    print(console.yellow + "  - %s" % rocon_uri_object.operating_system.list + console.reset)
    assert len(rocon_uri_object.operating_system.list) == 2
  
    multiple_application_frameworks = rocon_uri_string.replace('hydro', 'hydro|application_framework_other')
    print(console.cyan + " - %s" % multiple_application_frameworks + console.reset)
    rocon_uri_object = rocon_uri.parse(multiple_application_frameworks)
    print(console.yellow + "  - %s" % rocon_uri_object.application_framework.list + console.reset)
    assert len(rocon_uri_object.application_framework.list) == 2
  
def test_wildcards():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Wildcards" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    rocon_uri_string = 'rocon:///turtlebot2/dude/hydro/precise#rocon_apps/chirp'
    hardware_platform_uri = rocon_uri_string.replace('turtlebot2', rocon_std_msgs.Strings.URI_WILDCARD)
    print(console.cyan + " - %s" % hardware_platform_uri + console.reset)
    rocon_uri_object = rocon_uri.parse(hardware_platform_uri)
    assert rocon_uri_object.hardware_platform.string == rocon_std_msgs.Strings.URI_WILDCARD
  
    operating_systems_uri = rocon_uri_string.replace('precise', rocon_std_msgs.Strings.URI_WILDCARD)
    print(console.cyan + " - %s" % operating_systems_uri + console.reset)
    rocon_uri_object = rocon_uri.parse(operating_systems_uri)
    assert rocon_uri_object.operating_system.string == rocon_std_msgs.Strings.URI_WILDCARD
  
    application_framework_uri = rocon_uri_string.replace('hydro', rocon_std_msgs.Strings.URI_WILDCARD)
    print(console.cyan + " - %s" % application_framework_uri + console.reset)
    rocon_uri_object = rocon_uri.parse(application_framework_uri)
    assert rocon_uri_object.application_framework.string == rocon_std_msgs.Strings.URI_WILDCARD
  
    name_uri = rocon_uri_string.replace('dude', rocon_std_msgs.Strings.URI_WILDCARD)
    print(console.cyan + " - %s" % name_uri + console.reset)
    rocon_uri_object = rocon_uri.parse(name_uri)
    assert rocon_uri_object.name.string == rocon_std_msgs.Strings.URI_WILDCARD
  
def test_missing_fields():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Missing Fields" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
 
    rocon_uri_string = 'rocon:///turtlebot2/dude/hydro/precise'
 
    no_operating_system = 'rocon:///turtlebot2/dude/hydro'
    rocon_uri_object = rocon_uri.parse(no_operating_system)
    print(console.cyan + " - %s -> %s" % (no_operating_system, rocon_uri_object) + console.reset)
    assert(rocon_uri_object.operating_system.list[0] == rocon_std_msgs.Strings.URI_WILDCARD)
 
    no_application_framework = 'rocon:///turtlebot2/dude'
    rocon_uri_object = rocon_uri.parse(no_application_framework)
    print(console.cyan + " - %s -> %s" % (no_application_framework, rocon_uri_object) + console.reset)
    assert(rocon_uri_object.application_framework.list[0] == rocon_std_msgs.Strings.URI_WILDCARD)
 
    no_name = 'rocon:///turtlebot2'
    rocon_uri_object = rocon_uri.parse(no_name)
    print(console.cyan + " - %s -> %s" % (no_name, rocon_uri_object) + console.reset)
    assert(rocon_uri_object.name.list[0] == rocon_std_msgs.Strings.URI_WILDCARD)
  
def test_compatibility():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Compatibility" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
  
    rocon_uri_string = 'rocon:///turtlebot2/dude/hydro/precise'
    print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(rocon_uri_string, rocon_uri_string) == True)
    # Missing operating system
    modified_rocon_uri_string = 'rocon:///turtlebot2/dude/hydro'
    print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
    # Missing application_framework/operating system
    modified_rocon_uri_string = 'rocon:///turtlebot2/dude'
    print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
    # Missing everything (equivalent to full wildcards)
    modified_rocon_uri_string = 'rocon://'
    print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
    # Wildcards
    modified_rocon_uri_string = 'rocon:///%s/%s/%s/%s' % (rocon_std_msgs.Strings.URI_WILDCARD, rocon_std_msgs.Strings.URI_WILDCARD, rocon_std_msgs.Strings.URI_WILDCARD, rocon_std_msgs.Strings.URI_WILDCARD)
    print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
    # Regex names
    modified_rocon_uri_string = 'rocon:///turtlebot2/dud*/hydro/precise'
    print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
    modified_rocon_uri_string = 'rocon:///turtlebot2/dud*/hydro/precise'
    print(console.cyan + " - %s  ~ %s" % (modified_rocon_uri_string, rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(modified_rocon_uri_string, rocon_uri_string) == True)
    doubly_modified_rocon_uri_string = 'rocon:///turtlebot2/dudette*/hydro/precise'
    print(console.cyan + " - %s  ~ %s" % (modified_rocon_uri_string, doubly_modified_rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(modified_rocon_uri_string, doubly_modified_rocon_uri_string) == True)
    # No matching hardware platform
    modified_rocon_uri_string = 'rocon:///pr2|waiterbot/dude'
    print(console.cyan + " - %s !~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == False)
    # Modified field
    modified_rocon_uri_string = 'rocon:///turtlebot2/dudette/hydro/precise'
    print(console.cyan + " - %s !~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
    assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == False)
