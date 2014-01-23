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

##############################################################################
# Tests
##############################################################################

def test_invalid_elements():
    rocon_uri_string = 'rocon:///adams/turtlebot/precise/hydro/dude#rocon_apps/chirp'
    invalid_schema = rocon_uri_string.replace('rocon', 'http')
    invalid_hardware_platform = rocon_uri_string.replace('turtlebot', 'turtle_foobar')
    invalid_operating_system = rocon_uri_string.replace('precise', 'bados')
    invalid_application_framework = rocon_uri_string.replace('hydro', 'dont_box_me_in')
    # http not rocon
    assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, invalid_schema)
    assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, invalid_hardware_platform)
    assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, invalid_operating_system)
    assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, invalid_application_framework)

    rocon_uri_object = rocon_uri.parse(rocon_uri_string)
    assert str(rocon_uri_object) == rocon_uri_string
    print("%s" % rocon_uri_object)

def test_multiple_elements():
    '''
      Test the OR functionality.
    '''
    rocon_uri_string = 'rocon:///adams/turtlebot/precise/hydro/dude#rocon_apps/chirp'
    multiple_hardware_platforms = rocon_uri_string.replace('turtlebot', 'turtlebot2|pr2|waiterbot')
    print("Testing %s" % multiple_hardware_platforms)
    rocon_uri_object = rocon_uri.parse(multiple_hardware_platforms)
    print("  List %s" % rocon_uri_object.hardware_platforms)
    assert len(rocon_uri_object.hardware_platforms) == 3
    multiple_operating_systems = rocon_uri_string.replace('precise', 'quantal|precise')
    print("Testing %s" % multiple_operating_systems)
    rocon_uri_object = rocon_uri.parse(multiple_operating_systems)
    assert len(rocon_uri_object.operating_systems) == 2
    multiple_application_frameworks = rocon_uri_string.replace('hydro', 'hydro|application_other')
    print("Testing %s" % multiple_application_frameworks)
    rocon_uri_object = rocon_uri.parse(multiple_application_frameworks)
    print("Apps: %s" % rocon_uri_object.application_frameworks)
    assert len(rocon_uri_object.application_frameworks) == 2
