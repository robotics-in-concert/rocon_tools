#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Exceptions
##############################################################################


class InvalidInteraction(Exception):
    """
      Whenever an interaction has been specified incorrectly.
    """


class YamlResourceNotFoundException(IOError):
    """
      The requested yaml resource could not be found.
    """


class MalformedInteractionsYaml(Exception):
    """
      Whenever malformed yaml is used in loading a set of interactions.
    """
