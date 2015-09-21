#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

from __future__ import print_function
import os
import rocon_console.console as console
from rocon_python_utils.iterables import lookahead

##############################################################################
# Tests
##############################################################################

def test_lookahead():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Lookahead" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    last_results = []
    for i, last in lookahead(range(3)):
        print("%s, %s" % (i, last))
        last_results.append(last)
    assert(not last_results[0])
    assert(not last_results[1])
    assert(last_results[2])
