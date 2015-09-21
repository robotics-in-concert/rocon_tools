#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: iterables
   :platform: Unix
   :synopsis: Useful functions for working on interables, loops and stuff.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Methods
##############################################################################


def lookahead(iterable):
    """
    Lets you run over a loop where you are interested in catching the 'last'
    element in that loop.

    .. code:: python
       for i, last in lookahead(range(3)):
           print("%s, %s" % (i, last))

       # outputs:
       #
       # 0 False
       # 1 False
       # 2 True
    """
    it = iter(iterable)
    last = it.next()  # next(it) in Python 3
    for val in it:
        yield last, False
        last = val
    yield last, True
