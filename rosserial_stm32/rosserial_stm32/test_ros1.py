#!/usr/bin/env python
import sys
import rospkg
import roslib
import roslib.msgs
import roslib.srvs
import roslib.gentools

def get_dependencies(spec, package, compute_files=True, stdout=sys.stdout, stderr=sys.stderr, rospack=None):
    """
    Compute dependencies of the specified Msgs/Srvs
    @param spec: message or service instance
    @type  spec: L{roslib.msgs.MsgSpec}/L{roslib.srvs.SrvSpec}
    @param package: package name
    @type  package: str
    @param stdout: (optional) stdout pipe
    @type  stdout: file
    @param stderr: (optional) stderr pipe
    @type  stderr: file
    @param compute_files: (optional, default=True) compute file
    dependencies of message ('files' key in return value)
    @type  compute_files: bool
    @return: dict:
      * 'files': list of files that \a file depends on
      * 'deps': list of dependencies by type
      * 'spec': Msgs/Srvs instance.
      * 'uniquedeps': list of dependencies with duplicates removed,
      * 'package': package that dependencies were generated relative to.
    @rtype: dict
    """

    # #518: as a performance optimization, we're going to manually control the loading
    # of msgs instead of doing package-wide loads.

    # we're going to manipulate internal apis of msgs, so have to manually init
    roslib.msgs._init()

    deps = []
    try:
        if not rospack:
            rospack = rospkg.RosPack()
        if isinstance(spec, roslib.msgs.MsgSpec):
            _add_msgs_depends(rospack, spec, deps, package)
        elif isinstance(spec, roslib.srvs.SrvSpec):
            _add_msgs_depends(rospack, spec.request, deps, package)
            _add_msgs_depends(rospack, spec.response, deps, package)
        else:
            raise MsgSpecException('spec does not appear to be a message or service')
    except KeyError as e:
        raise MsgSpecException('Cannot load type %s.  Perhaps the package is missing a dependency.' % (str(e)))

    # convert from type names to file names

    if compute_files:
        files = {}
        for d in set(deps):
            d_pkg, t = roslib.names.package_resource_name(d)
            d_pkg = d_pkg or package  # convert '' -> local package
            files[d] = roslib.msgs.msg_file(d_pkg, t)
    else:
        files = None

    # create unique dependency list
    uniquedeps = []
    for d in deps:
        if d not in uniquedeps:
            uniquedeps.append(d)

    if compute_files:
        return {'files': files, 'deps': deps, 'spec': spec, 'package': package, 'uniquedeps': uniquedeps}
    else:
        return {'deps': deps, 'spec': spec, 'package': package, 'uniquedeps': uniquedeps}

def rosmsg_md5(mode, type_):
    package, base_type = roslib.names.package_resource_name(type_)
    print("package = %s, base_type = %s" % (package, base_type))

    roslib.msgs.load_package_dependencies(package, load_recursive=True)
    roslib.msgs.load_package(package)
    if mode == roslib.msgs.EXT:
        f = roslib.msgs.msg_file(package, base_type)
        print("f = %s" % f)
        name, spec = roslib.msgs.load_from_file(f, package)
    else:
        f = roslib.srvs.srv_file(package, base_type)
        print("f = %s" % f)
        name, spec = roslib.srvs.load_from_file(f, package)
    
    print("spec = ", spec)
    print("typeof spec = ", type(spec))
    #print("package = ", package)
    gendeps_dict = roslib.gentools.get_dependencies(spec, package, compute_files=False)
    print(gendeps_dict)
    return roslib.gentools.compute_md5(gendeps_dict)

if __name__ == '__main__':
    md5 = rosmsg_md5(roslib.msgs.EXT, "geometry_msgs/Vector3Stamped")
    print("md5 = ", md5)
