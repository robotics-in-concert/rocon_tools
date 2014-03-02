#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# system
import os

# ros
import rospkg
import roslib.names

##############################################################################
# Resources
##############################################################################


def find_resource_from_string(resource, rospack=None, extension=None):
    '''
      Convenience wrapper around roslib to find a resource (file) inside
      a package. This function passes off the work to find_resource
      once the input string is split.

      @param package : ros package
      @param resource : string resource identifier of the form package/filename

      @param extension : file name extension to look for/expect
      @type string

      @return full pathname to the resource
      @rtype str

      @raise rospkg.ResourceNotFound : raised if the resource is not found or has an inappropriate extension.
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

      @param package : ros package
      @param filename : some file inside the specified package
      @return str : absolute path to the file

      @raise rospkg.ResourceNotFound : raised if there is nothing found or multiple objects found.
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
        raise rospkg.ResourceNotFound("[%s] is not a package or launch file name" % package)
    return None
