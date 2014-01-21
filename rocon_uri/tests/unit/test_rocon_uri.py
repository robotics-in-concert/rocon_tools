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

def test_message_to_string():
    rocon_uri_string = 'rocon:///precise/hydro/turtlebot/dude#rocon_apps/chirp'
    rocon_uri_object = rocon_uri.parse(rocon_uri_string)
    # http not rocon
    assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, 'http:///precise/hydro/turtlebot/dude#rocon_apps/chirp')
    assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, 'rocon:///precise//turtlebot/dude#rocon_apps/chirp')
    assert(str(rocon_uri_object), rocon_uri_string)
        
