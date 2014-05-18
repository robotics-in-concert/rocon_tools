#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: launch
   :platform: Unix
   :synopsis: Machinery for spawning multiple roslaunchers.


This module contains the machinery for spawning and managing multiple terminals
that execute a pre-configured roslaunch inside each.

----

"""
##############################################################################
# Imports
##############################################################################

import os
import argparse
import signal
import sys
from time import sleep
import roslaunch
import tempfile
import rocon_python_comms
import rocon_python_utils
import rosgraph
import rocon_console.console as console

from .exceptions import InvalidRoconLauncher, UnsupportedTerminal
from . import terminals
from . import utils

##############################################################################
# Methods
##############################################################################


def parse_arguments():
    parser = argparse.ArgumentParser(description="Rocon's multi-roslauncher.")
    terminal_group = parser.add_mutually_exclusive_group()
    terminal_group.add_argument('-k', '--konsole', default=False, action='store_true', help='spawn individual ros systems via multiple konsole terminals')
    terminal_group.add_argument('-g', '--gnome', default=False, action='store_true', help='spawn individual ros systems via multiple gnome terminals')
    parser.add_argument('--screen', action='store_true', help='run each roslaunch with the --screen option')
    parser.add_argument('--no-terminals', action='store_true', help='do not spawn terminals for each roslaunch')
    parser.add_argument('--hold', action='store_true', help='hold terminals open after upon completion (incompatible with --no-terminals)')
    # Force package, launcher pairs, I like this better than roslaunch style which is a bit vague
    parser.add_argument('package', nargs='?', default='', help='name of the package in which to find the concert launcher')
    parser.add_argument('launcher', nargs=1, help='name of the concert launch configuration (xml) file')
    #parser.add_argument('launchers', nargs='+', help='package and concert launch configuration (xml) file configurations, roslaunch style')
    mappings = rosgraph.names.load_mappings(sys.argv)  # gets the arg mappings, e.g. scheduler_type:=simple
    argv = rosgraph.myargv(sys.argv[1:])  # strips the mappings
    args = parser.parse_args(argv)
    if args.no_terminals:
        args.terminal_name = terminals.active
    elif args.konsole:
        args.terminal_name = terminals.konsole
    elif args.gnome:
        args.terminal_name = terminals.gnome_terminal
    else:
        args.terminal_name = None
    return (args, mappings)


class RoconLaunch(object):
    __slots__ = [
                 'terminal',
                 'processes',
                 'hold'  # keep terminals open when sighandling them
                ]

    def __init__(self, terminal_name, hold=False):
        """
        Initialise empty of processes, but make sure we set the hold argument.

        :param bool hold: whether or not to hold windows open or not.
        """
        self.processes = []
        self.hold = hold
        try:
            self.terminal = terminals.create_terminal(terminal_name)
        except (UnsupportedTerminal, rocon_python_comms.NotFoundException) as e:
            console.error("Cannot find a suitable terminal [%s]" % str(e))
            sys.exit(1)

    def signal_handler(self, sig, frame):
        '''
          Special handler that gets triggered if someone hits CTRL-C in the original terminal that executed
          'rocon_launch'. We catch the interrupt here, search and eliminate all child roslaunch processes
          first (give them time to gracefully quit) and then finally close the terminals themselves.
          closing down the terminals themselves.

          :param str sig: signal id (usually looking for SIGINT - 2 here)
          :param frame: frame
        '''
        self.terminal.shutdown_roslaunch_windows(self.processes, self.hold)

    def spawn_roslaunch_window(self, launch_configuration):
        """
        :param launch_configuration:
        :type launch_configuration :class:`.RosLaunchConfiguration`
        """
        p = self.terminal.spawn_roslaunch_window(launch_configuration)
        self.processes.append(p)


def main():
    (args, mappings) = parse_arguments()
    rocon_launch = RoconLaunch(args.terminal_name, args.hold)
    signal.signal(signal.SIGINT, rocon_launch.signal_handler)

    if args.package == '':
        rocon_launcher = roslaunch.rlutil.resolve_launch_arguments(args.launcher)[0]
    else:
        rocon_launcher = roslaunch.rlutil.resolve_launch_arguments([args.package] + args.launcher)[0]
    if args.screen:
        roslaunch_options = "--screen"
    else:
        roslaunch_options = ""
    launchers = utils.parse_rocon_launcher(rocon_launcher, roslaunch_options, mappings)
    temporary_launchers = []
    for launcher in launchers:
        console.pretty_println("Launching %s on port %s" % (launcher.path, launcher.port), console.bold)
        ##########################
        # Customise the launcher
        ##########################
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        print("Launching %s" % temp.name)
        launch_text = '<launch>\n'
        if args.screen:
            launch_text += '  <param name="rocon/screen" value="true"/>\n'
        else:
            launch_text += '  <param name="rocon/screen" value="false"/>\n'
        launch_text += '  <include file="%s">\n' % launcher.path
        for (arg_name, arg_value) in launcher.args:
            launch_text += '    <arg name="%s" value="%s"/>\n' % (arg_name, arg_value)
        launch_text += '  </include>\n'
        launch_text += '</launch>\n'
        #print launch_text
        temp.write(launch_text)
        temp.close()  # unlink it later
        temporary_launchers.append(temp)
        launcher.path = temp.name  # replace the path to the original launcher with this one
        ##########################
        # Start the terminal
        ##########################
        rocon_launch.spawn_roslaunch_window(launcher)
    signal.pause()
    # Have to unlink them here rather than in the for loop above, because the whole gnome-terminal
    # subprocess takes a while to kick in (in the background) and the unlinking may occur before
    # it actually runs the roslaunch that needs the file.
    for temporary_launcher in temporary_launchers:
        os.unlink(temporary_launcher.name)
