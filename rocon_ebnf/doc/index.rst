.. rocon_ebnf documentation master file, created by
   sphinx-quickstart on Sat May  3 12:35:57 2014.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Rocon EBNF
==========

This is a ros-wrapped implementation of a python package that establishes
rules written in `ebnf notation`_ for parsing of specially formatted strings.
It is used in the `Robotics in Concert`_ framework for parsing and
manipulation of `rocon_uri`_ strings.

.. _`ebnf notation`: http://en.wikipedia.org/wiki/Extended_Backus%E2%80%93Naur_Form
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page
.. _`rocon_uri`: http://wiki.ros.org/rocon_uri

Upstream
--------

Details of LParis' original package:

- `Source v0.91 Tarball`_
- `Documentation`_

.. _`Source v0.91 Tarball` : http://lparis45.free.fr/rp-0.91.zip
.. _`Documentation` : http://lparis45.free.fr/rp.html

Patches
-------

- Renamed from ``ebnf`` to ``rocon_ebnf`` to avoid conflict with an original version on the python path.
- Two patches, `fdec1e9a`_ and `ac568daf`_ to allow underscores (oft used in ros names) to be matched
- Another patch, `18a79ca7`_ to correctly mix lowercase and uppercase together for matching.

.. _`fdec1e9a`: https://github.com/robotics-in-concert/rocon_tools/commit/fdec1e9a9fd9bc2a205e3d5ef1b8a084919351c7
.. _`ac568daf`: https://github.com/robotics-in-concert/rocon_tools/commit/ac568dafacddd0947f30de4899f14a8da7f656f5
.. _`18a79ca7`: https://github.com/robotics-in-concert/rocon_tools/commit/18a79ca796ca4eefb9e6b8fee94e772f98ae9267


.. Contents:

.. .. tocdtree::
..    :maxdepth: 2

..    upstream
..    patches

 
.. Indices and tables
.. ==================

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`

