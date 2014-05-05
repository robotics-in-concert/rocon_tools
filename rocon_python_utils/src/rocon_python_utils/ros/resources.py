#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: ros.resources
   :platform: Unix
   :synopsis: Helpers for working with ros resource names.

This module contains helpers that lookup or collect an index of ros resource
names for various purposes.

----

"""

##############################################################################
# Imports
##############################################################################

# system
import os

# ros
import rospkg
import roslib.names

from .catkin import package_index_from_package_path

##############################################################################
# Resources
##############################################################################


def find_resource_from_string(resource, rospack=None, extension=None):
    '''
      Convenience wrapper around roslib to find a resource (file) inside
      a package. This function passes off the work to find_resource
      once the input string is split.

      Pass it a shared rospack (:class:`.rospkg.RosPack`) object to accelerate
      the crawling across the ROS_PACKAGE_PATH when you are calling this
      function for many resources consecutively.

      .. code-block:: python

           rospack = rospkg.RosPack()
           for ros_resource_name in ['rocon_interactions/pc.interactions', 'rocon_interactions/demo.interactions']
               filename = find_resource_from_string(ros_resource_name, rospack)
               # do something

      :param str resource: ros resource name (in the form package/filename)
      :param rospack: a caching utility to help accelerate catkin filesystem lookups
      :type rospack: :class:`.rospkg.RosPack`
      :param str extension: file name extension to look for/expect

      :returns: full pathname to the resource
      :rtype: str

      :raises: :exc:`.rospkg.ResourceNotFound` raised if the resource is not found or has an inappropriate extension.
    '''
    if extension is not None:
        filename_extension = os.path.splitext(resource)[-1]
        if filename_extension == '':  # no ext given
            resource += ".%s" % extension
        elif filename_extension != "." + extension and filename_extension != extension:
            raise rospkg.ResourceNotFound("resource with invalid filename extension specified [%s][%s]" % (resource, extension))
    package, filename = roslib.names.package_resource_name(resource)
    if not package:
        raise rospkg.ResourceNotFound("resource could not be split with a valid leading package name [%s]" % (resource))
    return find_resource(package, filename, rospack)


def find_resource(package, filename, rospack=None):
    '''
      Convenience wrapper around roslib to find a resource (file) inside
      a package. It checks the output, and provides the appropriate
      error if there is one.

      :param str package: ros package
      :param str filename: some file inside the specified package
      :returns: absolute path to the file
      :rtype: str

      :raises: :exc:`.rospkg.ResourceNotFound` : raised if there is nothing found or multiple objects found.
    '''
    try:
        resolved = roslib.packages.find_resource(package, filename, rospack=rospack)
        if not resolved:
            raise rospkg.ResourceNotFound("cannot locate [%s] in package [%s]" % (filename, package))
        elif len(resolved) == 1:
            return resolved[0]
        elif len(resolved) > 1:
            raise rospkg.ResourceNotFound("multiple resources named [%s] in package [%s]:%s\nPlease specify full path instead" % (filename, package, ''.join(['\n- %s' % r for r in resolved])))
    except rospkg.ResourceNotFound:
        raise rospkg.ResourceNotFound("[%s] is not a package or launch file name [%s]" % (package, package + '/' + filename))
    return None


def resource_index_from_package_exports(export_tag, package_paths=None, package_whitelist=None, package_blacklist=[]):
    '''
      Scans the package path looking for exports and grab the ones we are interested in.

      :param str export_tag: export tagname
      :param package_paths: list of package paths to scan over
      :type package_paths: str[]
      :param package_whitelist: list of packages to include (and no other)
      :type package_paths: str[]
      :param package_blacklist: list of packages to exclude
      :type package_paths: str[]

      :returns: the dictionary of resource and its absolute path
      :rtype: dict { resource_name : os.path }
    '''
    package_index = _get_package_index(package_paths)
    resources = {}
    invalid_resources = {}
    for package in package_index.values():

        if package_whitelist:
            if package.name not in package_whitelist:
                continue
        elif package.name in package_blacklist:
            continue
        for export in package.exports:
            if export.tagname == export_tag:
                filename_relative_path = export.content
                resource_name = package.name + '/' + os.path.splitext(os.path.basename(filename_relative_path))[0]
                resource_filename = os.path.join(os.path.dirname(package.filename), filename_relative_path)
                if not os.path.isfile(resource_filename):
                    invalid_resources[resource_name] = resource_filename
                else:
                    resources[resource_name] = (resource_filename, package)
    return (resources, invalid_resources)


def _get_package_index(package_paths):
    # should make use of rospkg.get_ros_paths here.
    # http://docs.ros.org/independent/api/rospkg/html/rospkg_environment.html
    ros_package_path = package_paths if package_paths else os.getenv('ROS_PACKAGE_PATH', '')
    ros_package_path = [x for x in ros_package_path.split(':') if x]
    package_index = package_index_from_package_path(ros_package_path)
    return package_index
