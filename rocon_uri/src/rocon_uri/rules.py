#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

# Local imports

##############################################################################
# Rules
##############################################################################


def hardware_platforms():
    hardware_platforms_rule = [
             'init hardware_platforms_list=[] ',
             'pattern           ::= hp_zero hardware_platforms*',
             'hp_zero           ::= hp                          @hardware_platforms_list.append("$hp")',
             'hardware_platforms::= "|" hp                      @hardware_platforms_list.append("$hp")',
             'hp                ::= "*" | "pc" | robot | mobile_device',
               'robot           ::= "turtlebot2" | "pr2" | "waiterbot" | "robot_other"',
               'mobile_device   ::= smart_phone | tablet',
               'smart_phone     ::= "galaxy" | "mega" | "note3" | "smart_phone_other"',
               'tablet          ::= "xoom" | "note10" | "tablet_other"',
              ]
    return hardware_platforms_rule


def operating_systems():
    operating_systems_rule = [
             'init operating_systems_list=[] ',
             'pattern           ::= os_zero operating_systems*',
             'os_zero           ::= os                          @operating_systems_list.append("$os")',
             'operating_systems ::= "|" os                      @operating_systems_list.append("$os")',
             'os                ::= "*" | windoze | linux | "osx" | "freebsd"',
               'windoze         ::= "winxp" | "windows7"',
               'linux           ::= "arch" | "debian" | "fedora" | "gentoo" | "opensuse" | ubuntu | "linux"',
                 'ubuntu        ::= "precise" | "quantal" | "raring"'
              ]
    return operating_systems_rule


def application_frameworks():
    application_frameworks_rule = [
             'init application_frameworks_list=[] ',
             'pattern           ::= af_zero application_frameworks*',
             'af_zero           ::= af                           @application_frameworks_list.append("$af")',
             'application_frameworks ::= "|" af                  @application_frameworks_list.append("$af")',
             'af                 ::= "*" | ros | "opros" | "application_other"',
               'ros              ::= "groovy" | "hydro" | "ros_other"',
              ]
    return application_frameworks_rule


def names():
    names_rule = [
             'init names_list=[] ',
             'pattern           ::= name_zero names*',
             'name_zero         ::= name                          @names_list.append("$name")',
             'names             ::= "|" name                      @names_list.append("$name")',
             'name              ::= "*" | r"\S"*',
              ]
    return names_rule
