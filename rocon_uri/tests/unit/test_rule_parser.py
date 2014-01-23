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
import rocon_uri.rule_parser as rule_parser

##############################################################################
# Tests
##############################################################################

def test_message_to_string():
    # about this rule:
    #   - accomodate a trailing slash (sep? at the end)
    operating_systems_rule = [
             'init operating_systems_list=[] ',
             'pattern           ::= os_zero operating_systems*',
             'os_zero           ::= os                          @operating_systems_list.append("$os")', 
             'operating_systems ::= "|" os                      @operating_systems_list.append("$os")', 
             'os                ::= "*" | windows | linux | "osx" | "freebsd"',
               'windows         ::= "winxp" | "windows7"',
               'linux           ::= "arch" | "debian" | "fedora" | "gentoo" | "opensuse" | ubuntu | "linux"',
               'ubuntu          ::= "precise" | "quantal" | "raring" | "ubuntu"' 
              ]
    operating_systems_input = "precise|quantal"
    result = rule_parser.rp.match(operating_systems_rule, operating_systems_input)
    print("Input: %s" % operating_systems_input)
    if result is not None:
        print("  OS list: %s" % result.operating_systems_list)
        print("  Ubuntu : %s" % result.ubuntu)
        print("  Linux  : %s" % result.linux)
        #print("  Windows: %s" % result.windows) throws an AttributeError
    else:
        print("Error in parsing")
    
    rule = [ 'uri      ::= sep os* sep system* sep platform* sep name* sep?',
             'sep      ::= r"/"',
             'os       ::= "windows" | "linux" | "precise"',
             'system   ::= "opros" | ros',
               'ros      ::= "groovy" | "hydro" | "ros"', 
             'platform ::= r"." ^sep',
             'name     ::= r"." ^sep',
             ]
    rocon_uri = "/precise/ros/turtlebot/dude"
    print("Input: %s" % rocon_uri)
    result = rule_parser.rp.match(rule, rocon_uri)
    if result is not None:
        print("  os      : %s" % result.os)
        print("  system  : %s" % result.system)
        print("  platform: %s" % result.platform)
        print("  name    : %s" % result.name)
    else:
        print("Error in parsing")
    
#     rule = [ 'rosdistro ::= "groovy" | "hydro"' ]
#     text = 'groovy'
#     result = rule_parser.rp.match(rule, text)
#     print("\nResult: %s" % result)
#     print("rosdistro: %s" % result.rosdistro)

#     assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, 'http:///precise/hydro/turtlebot/dude#rocon_apps/chirp')
#     assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, 'rocon:///precise//turtlebot/dude#rocon_apps/chirp')
#     assert(str(rocon_uri_object), rocon_uri_string)
        
# #
# # we define the rule as a list of (sub)rules
# #
# # r"\S"*   means regular expression specifying 
# #          any character except blank 
# rule=['sqs  ::=  parms  fileid ', 
#       'parms::=  r"\S"* ',        
#       'fileid::= r"\S"* ']        
# #
# # we concatenate arguments ... as words
# parms=' '.join(sys.argv[1:])
# #
# # we make the parsing 
# cmp=rule_parser.rp.match(rule,parms)
# # 
# #as re module, if the result object is None, 
# # the parsing is unsuccessful
# if cmp==None:
#     print "Error in parsing:"   
# else:
#     #
#     # now, to get values from parsing, 
#     # we use rule names as parser arguments.
#     # cmp.sqs    will contain input parameters
#     # cmp.parms  will contain string to locate
#     # cmp.fileid will contain fileid to search in
#     try:
#         id=open(cmp.fileid)
#         for l in id.readlines():
#             if l.find(cmp.parms)>-1: 
#                 print l[:-1]
#     except Exception,e:
#         print e
#     else:
#         id.close()

