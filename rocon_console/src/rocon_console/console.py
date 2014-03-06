#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import sys

##############################################################################
# Methods
##############################################################################


def console_has_colours(stream):
    if not hasattr(stream, "isatty"):
        return False
    if not stream.isatty():
        return False  # auto color only on TTYs
    try:
        import curses
        curses.setupterm()
        return curses.tigetnum("colors") > 2
    except:
        # guess false in case of error
        return False

has_colours = console_has_colours(sys.stdout)
if has_colours:
    #reset = "\x1b[0;0m"
    reset = "\x1b[0m"
    bold = "\x1b[%sm" % '1'
    black, red, green, yellow, blue, magenta, cyan, white = ["\x1b[%sm" % str(i) for i in range(30, 38)]
    bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white = ["\x1b[%sm" % ('1;' + str(i)) for i in range(30, 38)]
else:
    reset = ""
    bold = ""
    black, red, green, yellow, blue, magenta, cyan, white = ["" for i in range(30, 38)]
    bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white = ["" for i in range(30, 38)]

colours = [
           bold,
           black, red, green, yellow, blue, magenta, cyan, white,
           bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white
          ]


def pretty_print(msg, colour=white):
    if has_colours:
        seq = colour + msg + reset
        sys.stdout.write(seq)
    else:
        sys.stdout.write(msg)


def pretty_println(msg, colour=white):
    if has_colours:
        seq = colour + msg + reset
        sys.stdout.write(seq)
        sys.stdout.write("\n")
    else:
        sys.stdout.write(msg)


##############################################################################
# Console
##############################################################################


def debug(msg):
    print(green + msg + reset)


def warning(msg):
    print(yellow + msg + reset)


def error(msg):
    print(red + msg + reset)


def logdebug(message):
    print(green + "[debug] " + message + reset)


def loginfo(message):
    print("[info] " + message)


def logwarn(message):
    print(yellow + "[warning] " + message + reset)


def logerror(message):
    print(red + "[error] " + message + reset)


def logfatal(message):
    print(bold_red + "[error] " + message + reset)


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    for colour in colours:
        pretty_print("dude\n", colour)
    logdebug("info message")
    logwarn("warning message")
    logerror("error message")
    logfatal("fatal message")
    pretty_print("red\n", red)
    print("some normal text")
    print(cyan + "    Name" + reset + ": " + yellow + "Dude" + reset)
