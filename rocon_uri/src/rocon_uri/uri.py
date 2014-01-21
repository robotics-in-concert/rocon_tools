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

# Local imports
from .exceptions import RoconURIInvalidException

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
            'location',
            'os',
            'system',
            'platform',
            'name',
            'rapp',
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
        self.location = parsed_url.netloc
        uri_path_elements = [element for element in parsed_url.path.split('/') if element]
        if len(uri_path_elements) != 4:
            raise RoconURIInvalidException("uri path element invalid, requires four fields [%s]" % parsed_url.path)
        self.os = uri_path_elements[0]
        self.platform = uri_path_elements[1]
        self.system = uri_path_elements[2]
        self.name = uri_path_elements[3]
        self.rapp = parsed_url.fragment

    def __str__(self):
        return "rocon://" + self.location + "/" + self.os + "/" + self.platform + "/" + self.system + "/" + self.name + "/" + self.rapp

