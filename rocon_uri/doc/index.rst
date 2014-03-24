.. rocon_uri documentation master file, created by
   sphinx-quickstart on Sun Mar 23 23:19:28 2014.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Rocon Uri
=========

Various entities in the `Robotics in Concert`_ framework require a specification for
describing the kind of entity it is, or the kind of entity it is compatible with. In
essence, it is about identifying what kind of resource (e.g. robot) something is, what
kind of resource is needed (e.g. the robot required by a service), or what kind of
resource it is compatible with (e.g. a rocon app that can run on certain robots). We
defer to the standard specifications for `universal resource identifiers`_ to
format strings representing our resources and `ebnf`_ as a notation to express
the exact format required for these strings.

The `rocon_uri`_ package enables this for the by providing a specification, rules and
an api for constructing and manipulating resource identifiers in the rocon framework.

.. _`universal resource identifiers`: http://en.wikipedia.org/wiki/Uniform_resource_identifier
.. _`ebnf`: http://en.wikipedia.org/wiki/Extended_Backus%E2%80%93Naur_Form
.. _`rocon_uri`: http://wiki.ros.org/rocon_uri
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page

Contents:

.. toctree::
   :maxdepth: 2

   specification
   rules
   usage
   modules
   CHANGELOG

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

