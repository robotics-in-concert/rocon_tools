#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

from catkin_pkg.packages import find_packages

##############################################################################
# Resources
##############################################################################


def package_index_from_package_path(package_paths):
    """Find all packages on the given list of paths

    Iterates over the given list of paths in reverse order so that packages
    found in the paths at the beginning of the list get overlaid onto packages
    with the same name which were found in paths farther back in the list.

    The resulting dictionary is keyed by the package name (so packages with
    duplicate names are overlaid) and the values are the
    :py:class:`catkin_pkg.package.Package` class

    @note Is this actually implemented as a function in a general ros package?

    :param ros_package_path: list of paths to search
    :type ros_package_path: list
    :returns: dictionary of package objects keyed by name of the package
    :rtype: dict
    """
    result = {}
    for path in reversed(package_paths):
        for unused_package_path, package in find_packages(path).items():
            result[package.name] = package
    return result
