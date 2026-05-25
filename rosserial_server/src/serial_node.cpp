/**
 *
 *  \file
 *  \brief      Main entry point for the serial node.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
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
 *
 */

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosserial_server/serial_session.h>

#include "rosserial_server/msg_lookup.h"

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>

#include <introspection_interfaces/srv/test.hpp>

#define ENABLE_RATE 0

void run_io_service(boost::asio::io_service* io_service)
{
    io_service->run();
}

int test_lookup_interfaces()
{
    rosserial_server::MsgInfo msginfo = rosserial_server::lookupMessage("std_msgs/msg/Int32");
    std::cout << "message_hash = " << msginfo.md5sum << std::endl;

    rosserial_server::MsgInfo srvinfo;
    rosserial_server::MsgInfo reqinfo;
    rosserial_server::MsgInfo respinfo;

    std::string message_type = "std_srvs/srv/SetBool";

    srvinfo = rosserial_server::lookupMessage(message_type);
    reqinfo = rosserial_server::lookupMessage(message_type, "_Request");
    respinfo = rosserial_server::lookupMessage(message_type, "_Response");

    std::cout << "service_md5 = " << srvinfo.md5sum << std::endl;
    std::cout << "request_message_md5_ = " << reqinfo.md5sum << std::endl;
    std::cout << "response_message_md5_ = " << respinfo.md5sum << std::endl;

    return 0;
}

rosserial_server::ConversionMap createConversionMap()
{
    rosserial_server::ConversionMap converter_map;

    //ADD_MAP_ENTRY(converter_map, introspection_interfaces::srv::Test);
    ADD_MAP_ENTRY(converter_map, std_srvs::srv::SetBool);
    ADD_MAP_ENTRY(converter_map, std_srvs::srv::Trigger);
    ADD_MAP_ENTRY(converter_map, std_srvs::srv::Empty);

    ADD_MAP_ENTRY(converter_map, introspection_interfaces::srv::Test);

    return converter_map;
}


int main(int argc, char* argv[])
{
    //return test_lookup_interfaces();

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("rosserial_server_serial_node");
    //TODO (if required): ros_helper::waitForRosout();
    node->declare_parameter("port", "/dev/ttyACM0");
    node->declare_parameter("baud", 57600);

    std::string port;
    int baud;

    node->get_parameter("port", port);
    node->get_parameter("baud", baud);

    boost::asio::io_service io_service;
    auto serial_session = std::make_shared<rosserial_server::SerialSession>(node, io_service, port, baud);
    serial_session->setServicesMap(createConversionMap());
    std::thread t(run_io_service, &io_service);

#if ENABLE_RATE
  rclcpp::Rate loop_rate(1);
  while (rclcpp::ok())
  {
    fprintf(stderr, "Inside the loop...\n");
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
#else
    //rclcpp::executors::MultiThreadedExecutor executor;
    //executor.add_node(node);
    //executor.spin();
    rclcpp::spin(node);
#endif
  t.join();
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
