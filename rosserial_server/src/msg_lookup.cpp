/**
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2019, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */
#include <algorithm>
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <iostream>
#include <vector>
#include "rosserial_server/msg_lookup.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>


namespace rosserial_server
{

struct PackageInfo
{
    std::string package_name;   // package name (i.e std_msgs, geometry_msgs, ...)
    std::string interface_type; // msg or srv
    std::string type_name;      // type name (i.e. Int32, PoseStamped, etc...)
};

std::ostream& operator<<(std::ostream& os, PackageInfo const& info)
{
    os << "package_name = " << info.package_name << ", interface_type = " << info.interface_type << ", type_name = " << info.type_name;
    return os;
}

PackageInfo split(const std::string str)
{
    std::vector<std::string> fields;
    const char del = '/';
    std::size_t pos = str.find(del);
    std::size_t prev = 0;

    while (pos != std::string::npos) {
        fields.push_back(str.substr(prev, pos-prev));
        prev = pos+1;
        pos = str.find(del, prev);
    }

    if (prev < str.size()) {
        fields.push_back(str.substr(prev));
    }

    if (fields.size() != 3)
    {
        throw std::invalid_argument("interface name does not have 3 parts");
    }

    return PackageInfo {fields[0], fields[1], fields[2]};
}

const MsgInfo lookupMessage(const std::string& interface_name, const std::string &suffix)
{
    MsgInfo msginfo;

    if (std::count(interface_name.begin(), interface_name.end(), '/') != 2)
    {
        std::cerr << "Invalid interface name, should have 3 parts" << std::endl;
        return msginfo;
    }

    PackageInfo package_info = split(interface_name);
    //std::cout << package_info << std::endl;
    std::string package_share_directory;
    try {
        package_share_directory = ament_index_cpp::get_package_share_directory(package_info.package_name);
    } catch (const ament_index_cpp::PackageNotFoundError & exception) {
        std::cerr << "Package not found: " << package_info.package_name << std::endl;
        std::cerr << exception.what() << std::endl;
        return msginfo;
    }

    //fprintf(stderr, "package_path = %s\n", package_share_directory.c_str());

    using namespace rapidjson;
    std::string msg_json = package_share_directory + "/" + package_info.interface_type + "/" + package_info.type_name + ".json";
    FILE* fp = fopen(msg_json.c_str(), "r");
	
	if(nullptr == fp)
    {
        std::cerr << "Json file not found: " << msg_json << std::endl;
		return msginfo;
    }
 
	try
	{
		char readBuffer[65536];
		FileReadStream is(fp, readBuffer, sizeof(readBuffer));	 
		fclose(fp);
		 
		Document document;
		document.ParseStream(is);
		
        //std::cout << "Json parsed successfully" << std::endl;
        //std::cout << document["type_hashes"][0]["hash_string"].GetString() << std::endl;
        const std::string full_type_name = interface_name + suffix;
        for (auto it = document["type_hashes"].Begin(); it != document["type_hashes"].End(); ++it)
        {
            if( full_type_name == it->GetObject()["type_name"].GetString() )
            {
                msginfo.md5sum = it->GetObject()["hash_string"].GetString();
                break;
            }
        }
	}
	catch(std::exception& exp)
	{
		std::cerr << exp.what() << std::endl;
		return msginfo;
	}

    return msginfo;
}

}  // namespace rosserial_server
