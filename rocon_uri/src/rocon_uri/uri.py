#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

# this needs updating to urllib.parse for python3
import urlparse
# register properties of the rocon scheme, otherwise fragments don't get allocated properly, refer to:
#   http://stackoverflow.com/questions/1417958/parse-custom-uris-with-urlparse-python
getattr(urlparse, 'uses_fragment').append('rocon')
getattr(urlparse, 'uses_netloc').append('rocon')
#getattr(urlparse, 'uses_params').append('rocon')
#getattr(urlparse, 'uses_query').append('rocon')
#getattr(urlparse, 'uses_relative').append('rocon')
import rocon_ebnf.rule_parser as rule_parser

# Local imports
from .exceptions import RoconURIInvalidException
import rules

##############################################################################
# RoconURI Class
##############################################################################


def parse(rocon_uri_string):
    """
      @param rocon_uri_string : a rocon uri in string format.
      @type str
      @return a validated rocon uri object
      @rtype RoconURI
      @raise RoconUriInvalidException
    """
    return RoconURI(rocon_uri_string)


class RoconURI(object):
    '''
      A rocon uri container.
    '''
    __slots__ = [
            'concert_name',    # urlparse scheme element
            'concert_version', # urlparse path elements
            'hardware_platforms',
            'operating_systems',
            'application_frameworks',
            'names',
            'rapp_name',       # urlparse fragment
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self, rocon_uri_string):
        """
          @param rocon_uri_string : a rocon uri in string format.
          @type str
          @raise RoconUriInvalidException
        """
        parsed_url = urlparse.urlparse(rocon_uri_string)
        if parsed_url.scheme != 'rocon':
            raise RoconURIInvalidException("uri scheme '%s' != 'rocon'" % parsed_url.scheme)
        self.concert_name = parsed_url.netloc
        uri_path_elements = [element for element in parsed_url.path.split('/') if element]
        if len(uri_path_elements) < 3 or len(uri_path_elements) > 5:
            raise RoconURIInvalidException("uri path element invalid, need at least concert_version/platform and at most concert_version/platform/os/system/name fields [%s]" % parsed_url.path)
        self.concert_version = uri_path_elements[0]
        try:
            self.hardware_platforms = rule_parser.match(rules.hardware_platforms(), uri_path_elements[1]).hardware_platforms_list
        except AttributeError: # result of match is None
            raise RoconURIInvalidException("hardware platforms specification is invalid [%s]" % uri_path_elements[1])
        try:
            self.operating_systems = rule_parser.match(rules.operating_systems(), uri_path_elements[2]).operating_systems_list
        except AttributeError: # result of match is None
            raise RoconURIInvalidException("operating system specification is invalid [%s]" % uri_path_elements[2])
        try:
            self.application_frameworks = rule_parser.match(rules.application_frameworks(), uri_path_elements[3]).application_frameworks_list
        except IndexError:
            self.application_frameworks = ["*"]
        except AttributeError: # result of match is None
            raise RoconURIInvalidException("application framework specification is invalid [%s]" % uri_path_elements[3])
        try:
            self.names = rule_parser.match(rules.names(), uri_path_elements[4]).names_list
        except IndexError:
            self.names = ["*"]
        except AttributeError: # result of match is None
            raise RoconURIInvalidException("name specification is invalid [%s]" % uri_path_elements[4])
        self.rapp_name = parsed_url.fragment

    def __str__(self):
        hardware_platforms = _collapse_string_list(self.hardware_platforms)
        operating_systems = _collapse_string_list(self.operating_systems)
        application_frameworks = _collapse_string_list(self.application_frameworks)
        names = _collapse_string_list(self.names)
        return "rocon://%s/%s/%s/%s/%s/%s#%s" % (self.concert_name, self.concert_version, hardware_platforms, operating_systems, application_frameworks, names, self.rapp_name)

def _collapse_string_list(string_list):
    # len = 0 already guaranteed before here
    s = ""
    if len(string_list) == 1:
        return string_list[0]
    else:
        s += string_list[0]
    for l in string_list[1:]:
        s += "|%s" % l
    return s
