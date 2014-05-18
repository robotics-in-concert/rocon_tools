#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: terminals
   :platform: Unix
   :synopsis: Terminal specific support.


This module provides a list of supported terminals and methods to handle them.

----

"""
##############################################################################
# Imports
##############################################################################

import os
import rocon_console.console as console
import rocon_python_comms
import rocon_python_utils
import signal
import subprocess
import time

from .exceptions import UnsupportedTerminal
from . import utils

##############################################################################
# Supported Terminals
##############################################################################

active = "active"  # the currently open terminal
"""String identifier for the currently open (active) terminal"""
konsole = "konsole"
"""String identifier for KDE's konsole terminal"""
gnome_terminal = "gnome-terminal"
"""String identifier for Gnome's terminal."""
gnome_terminal_wrapper = "gnome-terminal.wrapper"  # some systems use this for gnome-terminal
"""String identifier for an oft used representation of gnome's terminal on desktops like KDE."""

##############################################################################
# Terminal
##############################################################################


class Terminal(object):
    __slots__ = ['name']

    def __init__(self, name):
        """
        Creates a manager for the terminal with supporting methods and variables.
        :param str name: name of this terminal.
        """
        self.name = name

    def shutdown_roslaunch_windows(self, processes, hold):
        """
        Shuts down a roslaunch window cleanly, i.e. it first kills the roslaunch
        processes, then kills the terminal itself.
        """
        roslaunch_pids = []
        for process in processes:
            roslaunch_pids.extend(utils.get_roslaunch_pids(process.pid))
        # kill roslaunch's
        for pid in roslaunch_pids:
            try:
                os.kill(pid, signal.SIGHUP)
            except OSError:
                continue
        for pid in roslaunch_pids:
            console.pretty_println("Terminating roslaunch [pid: %d]" % pid, console.bold)
            rocon_python_utils.system.wait_pid(pid)
            #console.pretty_println("Terminated roslaunch [pid: %d]" % pid, console.bold)
        time.sleep(1)
        if hold:
            try:
                raw_input("Press <Enter> to close terminals...")
            except RuntimeError:
                pass  # this happens when you ctrl-c again instead of enter
        # now kill the terminal itself
        for process in processes:
            os.killpg(process.pid, signal.SIGTERM)
            #process.terminate()

##############################################################################
# Active
##############################################################################


class Active(Terminal):
    """
    A pseudo representation of the currently open terminal.
    """

    def __init__(self):
        """Dude"""
        super(Active, self).__init__(active)

    def spawn_roslaunch_window(self, roslaunch_configuration, postexec_fn=None):
        """
        :param roslaunch_configuration: required roslaunch info
        :type roslaunch_configuration: :class:`.RosLaunchConfiguration`
        :param func postexec_fn: run this after the subprocess finishes

        :returns: the subprocess handle
        :rtype: :class:subprocess.Popen
        """
        cmd = ["roslaunch"]
        if roslaunch_configuration.options:
            cmd.append(roslaunch_configuration.options)
        cmd.extend(["--port", roslaunch_configuration.port, roslaunch_configuration.path])
        return rocon_python_utils.system.Popen(cmd, postexec_fn=postexec_fn)

##############################################################################
# Konsole
##############################################################################


class Konsole(Terminal):
    """
    Responsible for handling of kde konsole terminals.
    """

    def __init__(self):
        super(Konsole, self).__init__(konsole)

    def spawn_roslaunch_window(self, roslaunch_configuration, postexec_fn=None):
        """
        :param roslaunch_configuration: required roslaunch info
        :type roslaunch_configuration: :class:`.RosLaunchConfiguration`
        :param func postexec_fn: run this after the subprocess finishes

        :returns: the subprocess handle
        :rtype: :class:subprocess.Popen
        """
        cmd = [self.name,
               '-p',
               'tabtitle=%s' % roslaunch_configuration.title,
               '--nofork',
               '--hold',
               '-e',
               "/bin/bash",
               "-c",
               "roslaunch %s --disable-title --port %s %s" %
                   (roslaunch_configuration.options,
                    roslaunch_configuration.port,
                    roslaunch_configuration.path)]
        return rocon_python_utils.system.Popen(cmd, postexec_fn=postexec_fn)

##############################################################################
# Gnome Terminal
##############################################################################


class GnomeTerminal(Terminal):
    """
    Responsible for handling of gnome-terminal terminals.
    """

    def __init__(self):
        super(GnomeTerminal, self).__init__(gnome_terminal)

    def spawn_roslaunch_window(self, roslaunch_configuration, postexec_fn=None):
        """
        :param roslaunch_configuration: required roslaunch info
        :type roslaunch_configuration: :class:`.RosLaunchConfiguration`
        :param func postexec_fn: run this after the subprocess finishes

        :returns: the subprocess handle
        :rtype: :class:subprocess.Popen
        """
        cmd = [self.name,
               '--title=%s' % roslaunch_configuration.title,
               '--disable-factory',
               "-e",
               "/bin/bash -c 'roslaunch %s --disable-title --port %s %s';/bin/bash" %
                   (roslaunch_configuration.options,
                    roslaunch_configuration.port,
                    roslaunch_configuration.path)
              ]
        return rocon_python_utils.system.Popen(cmd, postexec_fn=postexec_fn)

##############################################################################
# Factory
##############################################################################

supported_terminals = {active: Active,
                       konsole: Konsole,
                       gnome_terminal: GnomeTerminal,
                       gnome_terminal_wrapper: GnomeTerminal
                       }


def create_terminal(name=None):
    """
    Creates a manager for the terminal with supporting methods and variables.

    If name is None, it will try to auto-detect the user's terminal. We're currently
    using ubuntu's x-terminal-emulator to choose the shell.

    :param str name: name of the terminal manager to create (None to auto-detect).
    :returns: one of the suported terminal classes
    :rtype: one of the children of :class:.`.Terminal`

    :raises :exc:`.UnsupportedTerminal` if the name is not in the supported terminals list.
    :raises :exc:`rocon_python_comms.NotFoundException` if the specified/auto-detected terminal is not found on the system.
    """
    if name is not None and name not in supported_terminals.keys():
        raise UnsupportedTerminal("%s is not a supported terminal type [%s]" %
                         (name, supported_terminals.keys()))
    if name == konsole:
        if not rocon_python_utils.system.which('konsole'):
            msg = "cannot find 'konsole' (hint: try --gnome for gnome-terminal instead)"
            raise rocon_python_comms.NotFoundException(msg)
    elif name == gnome_terminal or name == gnome_terminal_wrapper:
        if not rocon_python_utils.system.which('konsole'):
            msg = "cannot find 'gnome' (hint: try --konsole for konsole instead)"
            raise rocon_python_comms.NotFoundException(msg)
    # elif name is active:  # nothing to do
    elif name is None:
        # auto-detect
        if not rocon_python_utils.system.which('x-terminal-emulator'):
            msg = "tried to auto-detect, but cannot find 'x-terminal-emulator' (hint: try --gnome or --konsole instead)"
            raise rocon_python_comms.NotFoundException(msg)
        p = subprocess.Popen([rocon_python_utils.system.which('update-alternatives'), '--query', 'x-terminal-emulator'], stdout=subprocess.PIPE)
        for line in p.stdout:
            if line.startswith("Value:"):
                auto_detected_name = os.path.basename(line.split()[1])
                break
        if auto_detected_name not in supported_terminals.keys():
            msg = "you are %s, an esoteric and unsupported terminal" % (auto_detected_name)
            console.warning(msg.capitalize())
            fallbacks = [konsole, gnome_terminal]
            for terminal_name in fallbacks:
                if rocon_python_utils.system.which(terminal_name):
                    name = terminal_name
                    console.warning(" --> falling back to '%s'" % terminal_name)
            if name is None:
                raise UnsupportedTerminal(msg + " (hint: try --gnome or --konsole instead)[%s]" % supported_terminals.keys())
        else:
            name = auto_detected_name
    return supported_terminals[name]()
