#!/usr/bin/env python
from __future__ import absolute_import

# ROS SETUP if needed

import multiprocessing
import time
import cProfile



try:
    import rospy
    import rosgraph
    import roslaunch
    import rocon_python_comms
    import pyros_setup
except ImportError as exc:
    import os
    import pyros_setup
    import sys
    sys.modules["pyros_setup"] = pyros_setup.delayed_import_auto(base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..'))
    import rospy
    import rosgraph
    import roslaunch
    import rocon_python_comms

roscore_process = None
master = None

if not rosgraph.masterapi.is_online():
    master, roscore_process = pyros_setup.get_master()
    assert master.is_online()

# Start roslaunch
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

cache_node = rocon_python_comms.ConnectionCache()

def update_loop():
    count = 255
    start = time.time()
    while count > 0:
        # time is ticking
        now = time.time()
        timedelta = now - start
        start = now

        cache_node.update()

        count -= 1

cProfile.run('update_loop()')

rospy.signal_shutdown('test complete')

if roscore_process is not None:
    roscore_process.terminate()  # make sure everything is stopped