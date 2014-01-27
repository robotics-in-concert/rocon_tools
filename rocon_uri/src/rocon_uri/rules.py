#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import yaml
import roslib

# Local imports

##############################################################################
# Rules
##############################################################################


def load_rules_into_dictionary():
    yaml_filename = os.path.join(roslib.packages.get_pkg_dir('rocon_uri'), 'yaml', 'rules.yaml')
    with open(yaml_filename) as f:
        yaml_rules = yaml.load(f)
    return yaml_rules


def walk_yaml_rules(name, root=load_rules_into_dictionary()):
    '''
      A generator which walks through the yaml list of rules.
      Works in almost exactly the same way as os.path.walk.
      Usage:

        for name, group, elements in walk_yaml('hardware_platform', yaml_rules['hardware_platform']):
        print("Name: %s" % name)
        print("  Group: %s" % group)
        print("  Elements: %s" % elements)

      @param name : a name to attach to the current 3-tuple that gets returned.
      @type str
      @param yaml_root : the yaml structure, defaults to the root yaml file.
      @type list
      one of the fields ['hardware_platform', 'name', 'application_framework', 'operating_system']

    '''
    #print("Walking %s" % name)
    groups = {}
    elements = []
    for element in root:
        if isinstance(element, dict):
            groups.update(element)
        else:
            elements.append(element)
    #print("  Groups: %s" % groups.keys())
    #print("  Elements: %s" % elements)
    yield (name, groups.keys(), elements)
    if not groups.keys():
        return
    for key, value in groups.iteritems():
        for x in walk_yaml_rules(name + '/' + key, value):
            yield x
    return


def load_ebnf_rules():
    yaml_rule_sets = {}
    yaml_rules = load_rules_into_dictionary()
    for yaml_rule_set in yaml_rules:  # merge each of hardware_platform, application_framework, os into one dictionary
        yaml_rule_sets.update(yaml_rule_set)
    # special case, add the names as an empty list
    yaml_rule_sets['name'] = []
    for yaml_rule_set_name, yaml_rule_set in yaml_rule_sets.iteritems():
        rules = []
        #rules.append('option verbose')
        rules.append('init %s_list=[]' % yaml_rule_set_name)
        rules.append('pattern ::= zero element*')
        rules.append('zero    ::= %s  @%s_list.append("$%s")' % (yaml_rule_set_name, yaml_rule_set_name, yaml_rule_set_name))
        rules.append('element ::= "|" %s   @%s_list.append("$%s")' % (yaml_rule_set_name, yaml_rule_set_name, yaml_rule_set_name))
        for name, groups, elements in walk_yaml_rules(yaml_rule_set_name, yaml_rule_set):
            # Accept a wildcard for each
            rule = '%s ::= "*"' % name.split('/')[-1]
            element_rules = ' | '.join(['"%s"' % element for element in elements])
            group_rules = ' | '.join(groups)
            if groups:
                rule += " | " + group_rules
            if elements:
                rule += " | " + element_rules
            # special case - let anything through for names.
            if yaml_rule_set_name == "name":
                rule += ' | r"\S"*'
            rules.append(rule)
        yaml_rule_sets[yaml_rule_set_name] = rules
    return yaml_rule_sets
