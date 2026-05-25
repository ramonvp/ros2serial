/**
 *
 *  \file
 *  \brief      Single, reconnecting class for a serial rosserial session.
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

#ifndef ROSSERIAL_SERVER_SERIAL_SESSION_H
#define ROSSERIAL_SERVER_SERIAL_SESSION_H

#include <iostream>
#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>

#include "rosserial_server/session.h"
#include <linux/serial.h>

namespace rosserial_server
{

class SerialSession : public Session<boost::asio::serial_port>
{
public:
  SerialSession(std::shared_ptr<rclcpp::Node> node, boost::asio::io_service& io_service, std::string port, int baud)
    : Session(node, io_service), port_(port), baud_(baud), timer_(io_service)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "rosserial_server session configured for " << port_ << " at " << baud << "bps.");

    failed_connection_attempts_ = 0;
    check_connection();
  }

private:
  void check_connection()
  {
    if (!is_active())
    {
      attempt_connection();
    }

    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (rclcpp::ok())
    {
      timer_.expires_from_now(boost::posix_time::milliseconds(2000));
      timer_.async_wait(boost::bind(&SerialSession::check_connection, this));
    }
    else
    {
      shutdown();
    }
  }

  void attempt_connection()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Opening serial port.");

    boost::system::error_code ec;
    socket().open(port_, ec);
    if (ec) {
      failed_connection_attempts_++;
      if (failed_connection_attempts_ == 1) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to open port " << port_ << ": " << ec);
      } else {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unable to open port " << port_ << " (" << failed_connection_attempts_ << "): " << ec);
      }
      return;
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Opened " << port_);
    failed_connection_attempts_ = 0;

    typedef boost::asio::serial_port_base serial;
    socket().set_option(serial::baud_rate(baud_));
    socket().set_option(serial::character_size(8));
    socket().set_option(serial::stop_bits(serial::stop_bits::one));
    socket().set_option(serial::parity(serial::parity::none));
    socket().set_option(serial::flow_control(serial::flow_control::none));

    boost::asio::serial_port::native_handle_type native = socket().native_handle();
    struct serial_struct serial_ops;
    ioctl(native, TIOCGSERIAL, &serial_ops);
    serial_ops.flags |= ASYNC_LOW_LATENCY; // (0x2000)
    ioctl(native, TIOCSSERIAL, &serial_ops);

    // Required to sleep and flush under Ubuntu 24 after opening file descriptor
    // see: https://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
    // Additionally, Arduino Uno boards gets reset with DTR signal every time the serial
    // port is opened, hence, we need to wait a bit until the firmware on the Arduino is
    // really running and attending the serial port communication.
    unsigned int wait_seconds = 2;
    RCLCPP_INFO(node_->get_logger(), "Waiting %u seconds for driver ready", wait_seconds);
    sleep(wait_seconds);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Flushing serial port");
    tcflush(native,TCIOFLUSH);

    // Kick off the session.
    start();
  }

  std::string port_;
  int baud_;
  boost::asio::deadline_timer timer_;
  int failed_connection_attempts_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_SERIAL_SESSION_H
