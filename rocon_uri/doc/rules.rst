.. _rules-section-label:

Rules
=====

A yaml file is used by the `rocon_uri`_ module to generate
the ebnf rules used to parse rocon uri strings. If rules need to be added
or modified, make a pull request against this file on the relevant branch.

* rules.yaml [`Indigo`_]
* rules.yaml [`Hydro-Devel`_]

.. _`Indigo`: https://github.com/robotics-in-concert/rocon_tools/tree/indigo/rocon_uri/yaml/rules.yaml
.. _`Hydro-Devel`: https://github.com/robotics-in-concert/rocon_tools/tree/hydro-devel/rocon_uri/yaml/rules.yaml

A snapshot of the current rules yaml:

.. literalinclude:: ../yaml/rules.yaml
   :language: yaml
   :linenos:

.. _`rocon_uri`: http://wiki.ros.org/rocon_uri
