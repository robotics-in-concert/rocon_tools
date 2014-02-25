#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Exceptions
##############################################################################


class ServicePairException(Exception):
    pass


class ServicePairIOException(Exception):
    pass


class NotFoundException(IOError):
    """
      Raised when a requested entity cannot be found, or didn't return the correct result.
    """
    pass
