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
import rocon_python_comms
import rocon_console.console as console
import rospy

##############################################################################
# Tests
##############################################################################

def test_basename():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Basename" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    private_name = '~foo'
    absolute_name = '/foo/bar'
    private_basename = rocon_python_comms.basename(private_name)
    absolute_basename = rocon_python_comms.basename(absolute_name)
    assert (private_basename == "foo")
    assert (absolute_basename == "bar")
    print(console.cyan + private_name + console.reset + " -> " + console.yellow + private_basename + console.reset)
    print(console.cyan + absolute_name + console.reset + " -> " + console.yellow + absolute_basename + console.reset)

