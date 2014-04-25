#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
"""
.. module:: web_interactions

Module for parsing web interaction strings provided to the interactions
manager.
"""
##############################################################################
# Imports
##############################################################################

#import urlparse
import re

##############################################################################
# Methods
##############################################################################


def parse(interaction):
    """
    Tries to parse the specified string to see if it is a valid web interaction
    (web app or web url). If it is, it passes back a web interaction object,
    or None if it is not valid.

    :param interaction str: the string to parse.

    :returns: the web interaction object if parsed
    :rtype: WebInteraction or None
    """
    # handle quotes or non quotes
    web_app_with_quotes = (WebInteraction.WEB_APP, re.compile(r"web_app\(\"(.+)\"\)"))
    web_app_without_quotes = (WebInteraction.WEB_APP, re.compile(r"web_app\((.+)\)"))
    web_url_with_quotes = (WebInteraction.WEB_URL, re.compile(r"web_url\(\"(.+)\"\)"))
    web_url_without_quotes = (WebInteraction.WEB_URL, re.compile(r"web_url\((.+)\)"))
    for (web_interaction_type, compiled_regular_expression) in [
            web_app_with_quotes, web_app_without_quotes, web_url_with_quotes, web_url_without_quotes]:
        result = compiled_regular_expression.match(interaction)
        if result:
            return WebInteraction(web_interaction_type, result.group(1))
    return None


class WebInteraction(object):

    WEB_APP = "web_app"
    WEB_URL = "web_url"

    def __init__(self, interaction_type, interaction_url):
        """
        :param interaction str: the string to parse

        :todo: should do some validation of the incoming url with urlparse here.
        """
        self._type = interaction_type
        self._url = interaction_url

    def is_web_app(self):
        return True if self._type == WebInteraction.WEB_APP else False

    def is_web_url(self):
        return True if self._type == WebInteraction.WEB_URL else False

    @property
    def url(self):
        return self._url
