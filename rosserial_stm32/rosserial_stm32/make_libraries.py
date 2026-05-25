#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken), 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

THIS_PACKAGE = "rosserial_stm32"

__usage__ = """
make_libraries.py generates the STM32 rosserial library files for SW4STM32.
It requires the location of your SWSTM32 project folder.

rosrun rosserial_stm32 make_libraries.py <output_path>
"""

from .interfaces import get_interface_spec, get_package_interfaces, get_message_hash, get_service_hash
from ament_index_python.packages import get_package_share_directory
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
import traceback
import hashlib
import os, sys, re
import json
import shutil
from colorama import Fore, Style

#####################################################################
# Data Types

class EnumerationType:
    """ For data values. """

    def __init__(self, name, ty, value):
        self.name = name
        self.type = ty
        self.value = value

    def make_declaration(self, f):
        f.write('      enum { %s = %s };\n' % (self.name, self.value))

class PrimitiveDataType:
    """ Our datatype is a C/C++ primitive. """

    def __init__(self, name, ty, bytes):
        self.name = name
        self.type = ty
        self.bytes = bytes

    def make_initializer(self, f, trailer):
        f.write('      %s(0)%s\n' % (self.name, trailer))

    def make_declaration(self, f):
        #f.write('      typedef %s _%s_type;\n      _%s_type %s;\n' % (self.type, self.name, self.name, self.name) )
        f.write('      %s %s;\n' % (self.type, self.name) )

    def serialize(self, f):
        #f.write('        cdr << %s;\n' % self.name)
        f.write('        ucdr_serialize_%s(cdr, this->%s);\n\n' % (self.type, self.name))

    def deserialize(self, f):
        f.write('        ucdr_deserialize_%s(cdr, &this->%s);\n\n' % (self.type, self.name))


class MessageDataType(PrimitiveDataType):
    """ For when our data type is another message. """

    def make_initializer(self, f, trailer):
        f.write('      %s()%s\n' % (self.name, trailer))

    def serialize(self, f):
        f.write('        this->%s.cdr_serialize(cdr);\n\n' % self.name)

    def deserialize(self, f):
        f.write('        this->%s.cdr_deserialize(cdr);\n\n' % self.name)

class AVR_Float64DataType(PrimitiveDataType):
    """ AVR C/C++ has no native 64-bit support, we automatically convert to 32-bit float. """

    def make_initializer(self, f, trailer):
        f.write('      %s(0)%s\n' % (self.name, trailer))

    def make_declaration(self, f):
        f.write('      typedef float _%s_type;\n      _%s_type %s;\n' % (self.name, self.name, self.name) )

    def serialize(self, f):
        f.write('      offset += serializeAvrFloat64(outbuffer + offset, this->%s);\n' % self.name)

    def deserialize(self, f):
        f.write('      offset += deserializeAvrFloat64(inbuffer + offset, &(this->%s));\n' % self.name)


class StringDataType(PrimitiveDataType):
    """ Need to convert to signed char *. """

    def make_initializer(self, f, trailer):
        f.write('      %s("")%s\n' % (self.name, trailer))

    def make_declaration(self, f):
        #f.write('      typedef const char* _%s_type;\n      _%s_type %s;\n' % (self.name, self.name, self.name) )
        f.write('      const char* %s;\n' % (self.name) )

    def serialize(self, f):
        #f.write('        ucdr_serialize_uint32_t(cdr, strlen(this->%s)+1)\n;' % (self.name))
        #f.write('        ucdr_serialize_array_char(cdr, this->%s, strlen(this->%s)+1);\n\n' % (self.name, self.name))
        f.write('        ucdr_serialize_string(cdr, this->%s);\n\n' % (self.name))

    def deserialize(self, f):
        # Cannot use ucdr_deserialize_string() because we cannot alloc strings in the embedded platform
        # So instead, we simply set the char pointer to point to the beggining of the string in the
        # original buffer used by the CDR
        cn = self.name.replace("[","").replace("]","")
        f.write('        uint32_t %s_str_len = 0;\n' % cn)
        f.write('        if (!ucdr_deserialize_uint32_t(cdr, &%s_str_len))\n' % cn)
        f.write('        {\n')
        f.write('            return false; // Failed to read string length\n')
        f.write('        }\n')
        f.write('\n')
        f.write('        this->%s = reinterpret_cast<const char*>(cdr->iterator);\n' % cn)
        f.write('        // check that the string is null-terminated\n')
        f.write('        if (this->%s[%s_str_len-1]) {\n' % (cn, cn))
        f.write('            return false;\n')
        f.write('        }\n')
        f.write('        ucdr_advance_buffer(cdr, %s_str_len);\n\n' % cn)


class ArrayDataType(PrimitiveDataType):

    def __init__(self, name, ty, bytes, cls, array_size=None):
        print("[ArrayDataType] name = %s, ty = %s, bytes = %d, cls = %s" % (name, ty, bytes, cls))
        self.name = name
        self.type = ty
        self.bytes = bytes
        self.size = array_size
        self.cls = cls

    def make_initializer(self, f, trailer):
        if self.size == None:
            f.write('      %s_length(0), st_%s(), %s(nullptr)%s\n' % (self.name, self.name, self.name, trailer))
        else:
            f.write('      %s()%s\n' % (self.name, trailer))

    def make_declaration(self, f):
        if self.size == None:
            f.write('      uint32_t %s_length;\n' % self.name)
            f.write('      typedef %s _%s_type;\n' % (self.type, self.name))
            f.write('      _%s_type st_%s;\n' % (self.name, self.name)) # static instance for copy
            f.write('      _%s_type * %s;\n' % (self.name, self.name))
        else:
            f.write('      %s %s[%d];\n' % (self.type, self.name, self.size))

    def serialize(self, f):
        c = self.cls(self.name+"[i]", self.type, self.bytes)
        if self.size == None:
            # serialize length
            f.write('      ucdr_serialize_uint32_t(cdr, this->%s_length);\n' % self.name)
            f.write('      for (uint32_t i = 0; i < %s_length; i++){\n' % self.name)
            c.serialize(f)
            f.write('      }\n')
        else:
            #f.write('        ucdr_serialize_sequence_%s(cdr, this->%s, %d);\n' % (self.type, self.name, self.size))
            f.write('      for (uint32_t i = 0; i < %d; i++){\n' % (self.size) )
            c.serialize(f)
            f.write('      }\n')

    def deserialize(self, f):
        if self.size == None:
            c = self.cls("st_"+self.name, self.type, self.bytes)
            # deserialize length
            f.write('      uint32_t %s_lengthT;\n' % self.name)
            f.write('      ucdr_deserialize_uint32_t(cdr, &%s_lengthT);\n' % self.name)
            #f.write('      uint32_t %s_lengthT = ((uint32_t) (*(inbuffer + offset))); \n' % self.name)
            #f.write('      %s_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); \n' % self.name)
            #f.write('      %s_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); \n' % self.name)
            #f.write('      %s_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); \n' % self.name)
            #f.write('      offset += sizeof(this->%s_length);\n' % self.name)
            f.write('      if(%s_lengthT > this->%s_length)\n' % (self.name, self.name))
            f.write('        this->%s = (%s*)realloc(this->%s, %s_lengthT * sizeof(%s));\n' % (self.name, self.type, self.name, self.name, self.type))
            f.write('      %s_length = %s_lengthT;\n' % (self.name, self.name))
            # copy to array
            f.write('      for( uint32_t i = 0; i < %s_length; i++){\n' % (self.name) )
            c.deserialize(f)
            f.write('        memcpy( &(this->%s[i]), &(this->st_%s), sizeof(%s));\n' % (self.name, self.name, self.type))
            f.write('      }\n')
        else:
            c = self.cls(self.name+"[i]", self.type, self.bytes)
            f.write('      for( uint32_t i = 0; i < %d; i++){\n' % (self.size) )
            c.deserialize(f)
            f.write('      }\n')




#####################################################################
# Mapping

ROS_TO_EMBEDDED_TYPES = {
    'bool'    :   ('bool',              1, PrimitiveDataType, []),
    'byte'    :   ('int8_t',            1, PrimitiveDataType, []),
    'int8'    :   ('int8_t',            1, PrimitiveDataType, []),
    'char'    :   ('uint8_t',           1, PrimitiveDataType, []),
    'uint8'   :   ('uint8_t',           1, PrimitiveDataType, []),
    'int16'   :   ('int16_t',           2, PrimitiveDataType, []),
    'uint16'  :   ('uint16_t',          2, PrimitiveDataType, []),
    'int32'   :   ('int32_t',           4, PrimitiveDataType, []),
    'uint32'  :   ('uint32_t',          4, PrimitiveDataType, []),
    'int64'   :   ('int64_t',           8, PrimitiveDataType, []),
    'uint64'  :   ('uint64_t',          8, PrimitiveDataType, []),
    'float32' :   ('float',             4, PrimitiveDataType, []),
    'float64' :   ('double',            8, PrimitiveDataType, []),
    'string'  :   ('char*',             0, StringDataType,    [])
}


#####################################################################
# Messages

class Message:
    """ Parses message definitions into something we can export. """
    global ROS_TO_EMBEDDED_TYPES

    def __init__(self, name: str, package: str, definition: str, md5: str):

        self.name = name            # name of message/class
        self.package = package      # package we reside in
        self.md5 = md5              # checksum
        self.includes = list()      # other files we must include

        self.data = list()          # data types for code generation
        self.enums = list()

        #print("type of definition: %s" % type(definition))
        #print("Parsing definition:\n%s" % definition)

        # parse definition
        for line in definition.splitlines():

            # prep work
            line = line.strip().rstrip()

            if len(line) == 0:
                continue

            # check if the whole line is a comment
            if line[0] == "#":
                continue

            #print("Analyzing line [%s]" % line)

            # remove trailing comments
            if line.find("#") > -1:
                #print("found # at %d" % line.find("#"))
                line = line[0:line.find("#")-1]
            line = line.strip().rstrip()

#            print("Analyzing line [%s]" % line)

            equal_pos = line.find("=")
            value = None
            if equal_pos > -1:
                try:
                    value = line[equal_pos+1:]
                except:
                    value = '"' + line[equal_pos+1:] + '"';
                
                value = value.strip()
                #print("found equal, value = %s" % value)
                line = line[0:equal_pos]

            # find package/class name
            line = line.replace("\t", " ")
            l = line.split(" ")
            while "" in l:
                l.remove("")
            if len(l) < 2:
                print("line contains < 2 fields: %s" % line)
                continue

            ty, name = l[0:2]
            if value != None:
                self.enums.append( EnumerationType(name, ty, value))
                #print("New enum [%s] [%s] = [%s]" % (ty, name, value))
                continue

            try:
                type_package, type_name = ty.split("/")
            except:
                type_package = None
                type_name = ty

            #print("var type = %s  var name = %s" % (type_name, name))
            type_array = False
            if type_name.find('[') > 0:
                type_array = True
                try:
                    type_array_size = int(type_name[type_name.find('[')+1:type_name.find(']')])
                except:
                    type_array_size = None
                type_name = type_name[0:type_name.find('[')]

            # convert to C type if primitive, expand name otherwise
            try:
                code_type = ROS_TO_EMBEDDED_TYPES[type_name][0]
                size = ROS_TO_EMBEDDED_TYPES[type_name][1]
                cls = ROS_TO_EMBEDDED_TYPES[type_name][2]
                for include in ROS_TO_EMBEDDED_TYPES[type_name][3]:
                    if include not in self.includes:
                        #print("include1 = %s" % include)
                        self.includes.append(include)
            except:
                if type_package == None:
                    type_package = self.package

                full_include = type_package + "/msg/" + convert_camel_case_to_lower_case_underscore(type_name)
                #original: full_include = type_package + "/msg/" + type_name
                #print("adding include %s" % full_include)
                if full_include not in self.includes:
                    self.includes.append(full_include)
                cls = MessageDataType
                code_type = type_package + "::msg::" + type_name
                size = 0
            if type_array:
                self.data.append( ArrayDataType(name, code_type, size, cls, type_array_size ) )
            else:
                self.data.append( cls(name, code_type, size) )

    def _write_serializer(self, f):
        # serializer
        f.write('    bool cdr_serialize(ucdrBuffer* cdr) const override\n')
        f.write('    {\n')
        for d in self.data:
            d.serialize(f)
        f.write('        return true;\n');
        f.write('    }\n')
        f.write('\n')

    def _write_deserializer(self, f):
        # deserializer
        f.write('    bool cdr_deserialize(ucdrBuffer* cdr) override\n')
        f.write('    {\n')
        for d in self.data:
            d.deserialize(f)
        f.write('        return true;\n');
        f.write('    }\n')
        f.write('\n')

    def _write_std_includes(self, f):
        f.write('#include <stdint.h>\n')
        f.write('#include <string.h>\n')
        f.write('#include <stdlib.h>\n')
        f.write('#include <ros/msg.h>\n')
        f.write('#include <ucdr/microcdr.h>\n')

    def _write_msg_includes(self,f):
        for i in self.includes:
            f.write('#include <%s.hpp>\n' % i)

    def _write_constructor(self, f):
        f.write('    %s()%s\n' % (self.name, ':' if self.data else ''))
        if self.data:
            for d in self.data[:-1]:
                d.make_initializer(f, ',')
            self.data[-1].make_initializer(f, '')
        f.write('    {\n    }\n\n')

    def _write_data(self, f):
        for d in self.data:
            d.make_declaration(f)
        for e in self.enums:
            e.make_declaration(f)
        f.write('\n')

    def _write_getType(self, f):
        f.write('    const char* getType() override { return "%s/msg/%s"; };\n'%(self.package, self.name))

    def _write_getMD5(self, f):
        f.write('    const char* getMD5() override { return "%s"; };\n'%self.md5)

    def _write_impl(self, f):
        f.write('  class %s : public ros::Msg\n' % self.name)
        f.write('  {\n')
        f.write('    public:\n')
        self._write_data(f)
        self._write_constructor(f)
        self._write_serializer(f)
        self._write_deserializer(f)
        self._write_getType(f)
        self._write_getMD5(f)
        f.write('\n')
        f.write('  };\n')

    def make_header(self, f):
        ifdef_name = '%s__MSG__%s_HPP_' % (self.package.upper(), self.name.upper()) 
        f.write('#ifndef %s\n' % ifdef_name)
        f.write('#define %s\n' % ifdef_name)
        f.write('\n')
        self._write_std_includes(f)
        self._write_msg_includes(f)
        f.write('\n')
        f.write('namespace %s\n' % self.package)
        f.write('{\n')
        f.write('\n')
        f.write('namespace msg\n')
        f.write('{\n')
        f.write('\n')
        self._write_impl(f)
        f.write('\n')
        f.write('} // namespace msg\n')
        f.write('\n')
        f.write('} // namespace %s\n' % self.package)
        f.write('\n')
        f.write('#endif // %s\n' % ifdef_name)

class Service:
    def __init__(self, name, package, definition, md5req, md5res):
        """
        @param name -  name of service
        @param package - name of service package
        @param definition - list of lines of  definition
        """

        self.name = name
        self.package = package

        """
        sep_line = len(definition.splitlines())
        sep = re.compile('---*')
        for i in range(0, len(definition)):
            if (None!= re.match(sep, definition[i]) ):
                sep_line = i
                break
        self.req_def = definition[0:sep_line]
        self.resp_def = definition[sep_line+1:]
        """
        self.req_def, self.resp_def = definition.split("---\n")

        self.req = Message(name+"Request", package, self.req_def, md5req)

        #print("type of definition = %s" % type(definition))
        #print(definition)
        #print("type of req_def = %s" % type(self.req_def))
        #print(self.req_def)

        self.resp = Message(name+"Response", package, self.resp_def, md5res)

    def make_header(self, f):
        ifdef_name = '%s__SRV__%s_HPP_' % (self.package.upper(), self.name.upper()) 
        f.write('#ifndef %s\n' % ifdef_name)
        f.write('#define %s\n' % ifdef_name)

        self.req._write_std_includes(f)
        includes = self.req.includes
        includes.extend(self.resp.includes)
        includes = list(set(includes))
        for inc in includes:
            f.write('#include "%s.h"\n' % inc)

        f.write('\n')
        f.write('namespace %s\n' % self.package)
        f.write('{\n')
        f.write('\n')
        f.write('namespace srv\n')
        f.write('{\n')
        f.write('\n')
        f.write('  static const char %s[] = "%s/srv/%s";\n'%(self.name.upper(), self.package, self.name))

        def write_type(out, name):
            out.write('    virtual const char * getType() override { return %s; };\n'%(name))
        _write_getType = lambda out: write_type(out, self.name.upper())
        self.req._write_getType = _write_getType
        self.resp._write_getType = _write_getType

        f.write('\n')
        self.req._write_impl(f)
        f.write('\n')
        self.resp._write_impl(f)
        f.write('\n')
        f.write('  class %s {\n' % self.name )
        f.write('    public:\n')
        f.write('    typedef %s Request;\n' % self.req.name )
        f.write('    typedef %s Response;\n' % self.resp.name )
        f.write('  };\n')
        f.write('\n')

        f.write('} // namespace srv\n')
        f.write('\n')
        f.write('} // namespace %s\n' % self.package)
        f.write('\n')

        f.write('#endif\n')



class MessageClass:
    def __init__(self):
        self._md5sum = None
        self._type = None

class ServiceClass:
    def __init__(self):
        self._request_class = MessageClass()
        self._response_class = MessageClass()


"""
def get_service_class(service_type):
    sc = ServiceClass()
    hash_req = hashlib.md5()
    hash_resp = hashlib.md5()
    
    spec = get_interface_spec(service_type)
    req, resp = spec.split("---") 

    hash_req.update(req.strip().encode())
    hash_resp.update(resp.strip().encode())
    sc._request_class._md5sum = hash_req.hexdigest()
    sc._request_class._type = service_type + "Request"
    sc._response_class._md5sum = hash_resp.hexdigest()
    sc._response_class._type = service_type + "Response"
    return sc
"""

# For ROS1 messages, MD5 sum is calculated here:
# https://github.com/ros/ros/blob/noetic-devel/core/roslib/src/roslib/gentools.py
# function: def compute_md5()

#####################################################################
# Make a Library

def MakeLibrary(package: str, output_path: str):
    ENABLE_SRVS = True
    interfaces = get_package_interfaces(package)

    messages = list()
    services = list()
    for interface in interfaces:
        if "/msg/" in interface:
            print("[MSG] %s" % interface)
            _, _, msg_name = interface.split('/')
            definition = get_interface_spec(interface)
            rihs01_hash = get_message_hash(interface)
            if rihs01_hash:
                messages.append( Message(msg_name, package, definition, rihs01_hash) )
            else:
                err_msg = "Unable to build message: %s/msg/%s\n" % (package, msg_name)
                sys.stderr.write(err_msg)
        elif "/srv/" in interface:
            if not ENABLE_SRVS:
                print(Fore.RED + "[SRV] %s (disabled)" % (interface))
                print(Style.RESET_ALL, end='')
                continue

            print("[SRV] %s" % (interface))
            _, _, srv_name = interface.split('/')
            definition = get_interface_spec(interface)
            md5req, md5res = get_service_hash(interface)
            if md5req and md5res:
                services.append( Service(srv_name, package, definition, md5req, md5res ) )
            else:
                err_msg = "Unable to build service: %s/srv/%s\n" % (package, srv_name)
                sys.stderr.write(err_msg)

    # generate for each message
    msgs_output_path = os.path.join(output_path, package, "msg")
    for msg in messages:
        if not os.path.exists(msgs_output_path):
            os.makedirs(msgs_output_path)
        header = open(os.path.join(msgs_output_path, convert_camel_case_to_lower_case_underscore(msg.name) + ".hpp"), "w")
        msg.make_header(header)
        header.close()

    # generate for each service
    srvs_output_path = os.path.join(output_path, package, "srv")
    for srv in services:
        if not os.path.exists(srvs_output_path):
            os.makedirs(srvs_output_path)
        header = open(os.path.join(srvs_output_path, convert_camel_case_to_lower_case_underscore(srv.name) + ".hpp"), "w")
        srv.make_header(header)
        header.close()


def make_msg(interface: str, output_path:str):
    message = None
    print("[MSG] ", interface)
    package, _, msg_name = interface.split('/')
    definition = get_interface_spec(interface)
    rihs01_hash = get_message_hash(interface)
    if rihs01_hash:
        message = Message(msg_name, package, definition, rihs01_hash)
    else:
        err_msg = "Unable to build message: %s/msg/%s\n" % (package, msg_name)
        sys.stderr.write(err_msg)

    msgs_output_path = os.path.join(output_path, package, "msg")
    if not os.path.exists(msgs_output_path):
        os.makedirs(msgs_output_path)
    header = open(os.path.join(msgs_output_path, message.name + ".h"), "w")
    message.make_header(header)
    header.close()


def rosserial_generate(pkg_list, path):
    failed = []
    for p in sorted(pkg_list):
        try:
            MakeLibrary(p, path)
        except Exception as e:
            failed.append(p + " ("+str(e)+")")
            print('[%s]: Unable to build messages: %s\n' % (p, str(e)))
            print(traceback.format_exc())
    print('\n')
    if len(failed) > 0:
        print('*** Warning, failed to generate libraries for the following packages: ***')
        for f in failed:
            print('    %s'%f)
        raise Exception("Failed to generate libraries for: " + str(failed))
    print('\n')

def rosserial_client_copy_files(rospack, path):
    if not os.path.exists(os.path.join(path, "ros")):
        os.makedirs(os.path.join(path, "ros"))
    if not os.path.exists(os.path.join(path, "tf")):
        os.makedirs(os.path.join(path, "tf"))
    files = ['duration.cpp',
             'time.cpp',
             os.path.join('ros', 'duration.h'),
             os.path.join('ros', 'msg.h'),
             os.path.join('ros', 'node_handle.h'),
             os.path.join('ros', 'publisher.h'),
             os.path.join('ros', 'service_client.h'),
             os.path.join('ros', 'service_server.h'),
             os.path.join('ros', 'subscriber.h'),
             os.path.join('ros', 'time.h'),
             os.path.join('tf', 'tf.h'),
             os.path.join('tf', 'transform_broadcaster.h')]
    mydir = rospack.get_path("rosserial_client")
    for f in files:
        shutil.copyfile(os.path.join(mydir, "src", "ros_lib", f), os.path.join(path, f))


def main(args=None):
    # need correct inputs
    if len(sys.argv) < 2:
        print(__usage__)
        exit(1)
        
    # get output path
    path = sys.argv[1]
    if path[-1] == "/":
        path = path[0:-1]
    path = path + "/Inc/"
    #print("\nExporting to %s" % path)

    #rospack = rospkg.RosPack()
    rospack = None

    # copy ros_lib stuff in
    """
    rosserial_stm32_dir = get_package_share_directory(THIS_PACKAGE)
    files = os.listdir(rosserial_stm32_dir+"/ros_lib")
    for f in files:
        if os.path.isfile(rosserial_stm32_dir+"/ros_lib/"+f):
            shutil.copy(rosserial_stm32_dir+"/ros_lib/"+f, path)
        else:
            shutil.copytree(rosserial_stm32_dir+"/ros_lib/"+f, path, dirs_exist_ok = True)
    """


    #rosserial_client_copy_files(rospack, path)

    # generate messages
    #rosserial_generate(['std_msgs', 'std_srvs'], path)
    #rosserial_generate(['std_msgs','builtin_interfaces','rosserial_msgs', 'std_srvs', 'introspection_interfaces'], path)
    rosserial_generate(['introspection_interfaces'], path)
    #make_msg("std_msgs/msg/Int32", path)

    #interface = "std_msgs/msg/Int32"
    #interface = sys.argv[2]
    #package, _, msg_name = interface.split('/')
    #definition = get_interface_spec(interface)
    #rihs01_hash = get_message_hash(interface)
    #message = Message(msg_name, package, definition, rihs01_hash)

    #print("\n====================================")
    #f = sys.stderr
    #message._write_msg_includes(f)
    #message._write_data(f)
    #message._write_constructor(f)
    #message._write_deserializer(f)
    #message.make_header(f)


if __name__ == '__main__':
    main()
