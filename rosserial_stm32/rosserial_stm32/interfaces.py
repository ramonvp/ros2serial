#!/usr/bin/env python

import typing
import sys
from ros2interface.verb.show import InterfaceTextLine
from rosidl_runtime_py import get_interface_path, get_interfaces
import collections
import json
import os

def get_message_hash(message_type: str):
    hash = None
    path = get_interface_path(message_type)
    json_path = os.path.splitext(path)[0] + ".json"
    #print("[get_message_hash] json_path = %s" % json_path)
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
            hash = data["type_hashes"][0]["hash_string"]
    except Exception as e:
        print(e)

    return hash

def get_service_hash(service_type: str):
    hash_request = None
    hash_response = None
    path = get_interface_path(service_type)
    json_path = os.path.splitext(path)[0] + ".json"
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
        for type_hash in data["type_hashes"]:
            if "Request" in type_hash["type_name"]:
                hash_request = type_hash["hash_string"]
            if "Response" in type_hash["type_name"]:
                hash_response = type_hash["hash_string"]
    except Exception as e:
        print(e)

    return hash_request, hash_response

def _get_interface_lines(interface_identifier: str) -> typing.Iterable[InterfaceTextLine]:
    parts: typing.List[str] = interface_identifier.split('/')
    if len(parts) != 3:
        raise ValueError(
            f"Invalid name '{interface_identifier}'. Expected three parts separated by '/'"
        )
    pkg_name, _, msg_name = parts

    file_path = get_interface_path(interface_identifier)
    #print("[_get_interface_lines] interface_id = %s, file_path = %s" % (interface_identifier, file_path))
    with open(file_path) as file_handler:
        for line in file_handler:
            #print("interface line: %s" % line)
            yield InterfaceTextLine(
                pkg_name=pkg_name,
                msg_name=msg_name,
                line_text=line.rstrip(),
            )

def _print_interface_line(
    line: InterfaceTextLine,
    is_show_comments: bool,
    indent_level: int,
    print_line: bool
) -> str:
    line_to_print = ""
    text = str(line)
    if not is_show_comments:
        if not text or line.is_comment():
            return line_to_print
        elif line.is_trailing_comment():
            #print("line trailing comment = ",line.trailing_comment)
            #comment_start_idx = text.find(line.trailing_comment)
            comment_start_idx = text.find('#')
            text = text[:comment_start_idx - 1].strip()
    if text:
        indent_string = indent_level * '\t'
        line_to_print = f'{indent_string}{text}'
        if print_line:
            print("AFTER: ",line_to_print)
    elif print_line:
        print()

    line_to_print += "\n"

    return line_to_print

def get_interface_spec_ros2(
    interface_identifier: str,
    is_show_comments: bool = False,
    is_show_nested_comments: bool = False,
    indent_level: int = 0,
    expand_nesting: bool = False,
    print_line: bool = False
):
    spec : str = ""
    for line in _get_interface_lines(interface_identifier):
        #print("BEFORE: ",line)
        spec += _print_interface_line(line, is_show_comments=is_show_comments, indent_level=indent_level, print_line=print_line)
        if line.nested_type and expand_nesting:
            spec += get_interface_spec(
                line.nested_type,
                is_show_comments=is_show_nested_comments,
                is_show_nested_comments=is_show_nested_comments,
                indent_level=indent_level+1,
                expand_nesting=expand_nesting
            )
    
    return spec

def get_interface_spec(interface_identifier: str):
    file_path = get_interface_path(interface_identifier)
    with open(file_path, 'r') as file:
        spec = file.read()
    return spec

def get_package_interfaces(interface_identifier: str,) -> typing.List[str]:
    interfaces = collections.defaultdict(list)
    try:
        interfaces = get_interfaces([interface_identifier])
    except LookupError as e:
        return []

    interfaces_list = []
    for package_name in sorted(interfaces):
        for interface_name in interfaces[package_name]:
            full_name = f'{package_name}/{interface_name}'
            interfaces_list.append(full_name)
            #print(full_name)
    return interfaces_list

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Interface identifier required, i.e. geometry_msgs/msg/Vector3")
        exit(1)

    try:
        spec = get_interface_spec(sys.argv[1])
        req, resp = spec.split("---") 
        print("req=", req.strip())
        print("resp=", resp.strip())
        #print(get_package_interfaces(sys.argv[1]))
    except ValueError as e:
        print(e)
    except LookupError as e:
        print(e)
