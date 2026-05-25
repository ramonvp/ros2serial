/**
 *
 *  \file
 *  \brief      Class representing a session between this node and a
 *              templated rosserial client.
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

#ifndef ROSSERIAL_SERVER_SESSION_H
#define ROSSERIAL_SERVER_SESSION_H

#include <map>
#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/core/noncopyable.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <rosserial_msgs/msg/topic_info.hpp>
#include <rosserial_msgs/msg/log.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "rosserial_server/async_read_buffer.h"
#include "rosserial_server/topic_handlers.h"
#include "rosserial_server/services.h"

#include <rcpputils/asserts.hpp>

namespace rosserial_server
{

typedef std::vector<uint8_t> Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;


template<typename Socket>
class Session : public boost::noncopyable /*, public rclcpp::Node*/
{
public:
  Session(std::shared_ptr<rclcpp::Node> node, boost::asio::io_service& io_service)
    : node_(node),
      io_service_(io_service),
      socket_(io_service),
      sync_timer_(io_service),
      require_check_timer_(io_service),
      ros_spin_timer_(io_service),
      async_read_buffer_(node_, socket_, buffer_max,
                         boost::bind(&Session::read_failed, this,
                                     boost::asio::placeholders::error))
  {
    active_ = false;

    timeout_interval_ = boost::posix_time::milliseconds(5000);
    attempt_interval_ = boost::posix_time::milliseconds(1500); //was 1 second, could be not enough  1500
    require_check_interval_ = boost::posix_time::milliseconds(1500); //was 1 second, could be not enough
    require_param_name_ = "~require";

    node->declare_parameter("unrecognised_topic_retry_threshold", 1);
    node->declare_parameter("services_no_response_timeout_ms", 2000);


    node->get_parameter("unrecognised_topic_retry_threshold", unrecognised_topic_retry_threshold_); // It was 0 previously
    node->get_parameter("services_no_response_timeout_ms", services_no_response_timeout_ms_); // 2seg by default, previously was 5seg.


    // Intermittent callback to service ROS callbacks. To avoid polling like this,
    // CallbackQueue could in the future be extended with a scheme to monitor for
    // callbacks on another thread, and then queue them up to be executed on this one.
    /*ros_spin_timer_.expires_from_now(ros_spin_interval_);
    ros_spin_timer_.async_wait(boost::bind(&Session::ros_spin_timeout, this,
                                           boost::asio::placeholders::error));*/
  }

  Socket& socket()
  {
    return socket_;
  }

  void setServicesMap(const ConversionMap& services_map)
  {
        services_map_ = services_map;
  }

  void start()
  {
    using namespace rosserial_msgs::msg;
    using namespace std::placeholders;

    RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::start] Starting session.");

    callbacks_[TopicInfo::ID_PUBLISHER] = std::bind(&Session::setup_publisher, this, _1);
    callbacks_[TopicInfo::ID_SUBSCRIBER] = std::bind(&Session::setup_subscriber, this, _1);
    callbacks_[TopicInfo::ID_LOG] = std::bind(&Session::handle_log, this, _1);
    callbacks_[TopicInfo::ID_TIME] = std::bind(&Session::handle_time, this, _1);

    callbacks_[TopicInfo::ID_SERVICE_CLIENT+TopicInfo::ID_PUBLISHER]  = std::bind(&Session::setup_service_client_publisher, this, _1);
    callbacks_[TopicInfo::ID_SERVICE_CLIENT+TopicInfo::ID_SUBSCRIBER] = std::bind(&Session::setup_service_client_subscriber, this, _1);
    callbacks_[TopicInfo::ID_SERVICE_SERVER+TopicInfo::ID_PUBLISHER]  = std::bind(&Session::setup_service_server_publisher, this, _1);
    callbacks_[TopicInfo::ID_SERVICE_SERVER+TopicInfo::ID_SUBSCRIBER] = std::bind(&Session::setup_service_server_subscriber, this, _1);

    active_ = true;
    attempt_sync();
    read_sync_header();
  }

  void stop()
  {
    if(is_active())
    {
      RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::stop] Request stop");
      active_ = false;
      // Abort any pending ROS callbacks.
      //ros_callback_queue_.clear();

      // Abort active session timer callbacks, if present.
      sync_timer_.cancel();
      require_check_timer_.cancel();

      // Reset the state of the session, dropping any publishers or subscribers
      // we currently know about from this client.
      callbacks_.clear();
      topics_names_.clear();
      subscribers_.clear();
      publishers_.clear();
      service_clients_.clear();
      service_servers_.clear();

      // Send disconnection message
      std::vector<uint8_t> message(1);
      write_message(message, rosserial_msgs::msg::TopicInfo::ID_TX_STOP);

      // Close the socket.
      socket_.close();
    }
  }

  void shutdown()
  {
    if (is_active())
    {
      stop();
    }
    io_service_.stop();
  }

  bool is_active() const
  {
    return active_;
  }

  /**
   * This is to set the name of the required topics parameter from the
   * default of ~require. You might want to do this to avoid a conflict
   * with something else in that namespace, or because you're embedding
   * multiple instances of rosserial_server in a single process.
   */
  void set_require_param(const std::string & param_name)
  {
    require_param_name_ = param_name;
  }

protected:
  std::shared_ptr<rclcpp::Node> node_;

private:
  /**
   * Periodic function which handles calling ROS callbacks, executed on the same
   * io_service thread to avoid a concurrency nightmare.
   */
  void ros_spin_timeout(const boost::system::error_code& error) {
    //ros_callback_queue_.callAvailable();

    if (rclcpp::ok())
    {
      // Call again next interval.
      ros_spin_timer_.expires_from_now(ros_spin_interval_);
      ros_spin_timer_.async_wait(boost::bind(&Session::ros_spin_timeout, this,
                                             boost::asio::placeholders::error));
    }
    else
    {
      shutdown();
    }
  }

  //// RECEIVING MESSAGES ////
  // TODO: Total message timeout, implement primarily in ReadBuffer.

  void read_sync_header() {
    async_read_buffer_.read(1, boost::bind(&Session::read_sync_first, this, boost::placeholders::_1));
  }

  void read_sync_first(SimpleCdr& stream) {
    uint8_t sync;
    stream >> sync;
    /**/RCLCPP_DEBUG(node_->get_logger(), "[read_sync_first] sync byte = 0x%02X", sync);
    if (sync == 0xff) {
        //RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::read_sync_first] Received sync first");
        async_read_buffer_.read(1, boost::bind(&Session::read_sync_second, this, boost::placeholders::_1));
    } else {
        //RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::read_sync_first] Byte is not sync first %02X", sync);
        read_sync_header();
    }
  }

  void read_sync_second(SimpleCdr& stream) {
    uint8_t sync;
    stream >> sync;
    RCLCPP_DEBUG(node_->get_logger(), "[read_sync_second] sync byte = 0x%02X", sync);
    if (sync == 0xfe) {
        //RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::read_sync_second] Received sync second");
        async_read_buffer_.read(5, boost::bind(&Session::read_id_length, this, boost::placeholders::_1));
    } else {
        read_sync_header();
    }
  }

  void read_id_length(SimpleCdr& stream) {
    uint16_t length;
    uint8_t length_checksum;

    // Check header checksum byte for length field.
    stream >> length >> length_checksum;
    uint8_t csl = checksum(length);
    if (length_checksum + csl != 0xff) {
      RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::read_id_length] Bad message header length checksum. Dropping message from client. L%d C%02X %02X", length, length_checksum, csl);
      read_sync_header();
      return;
    }
    
    uint16_t topic_id;
    stream >> topic_id;
    
    RCLCPP_DEBUG(node_->get_logger(), "[Rosserial::Session::read_id_length] Received message header with length %d and topic_id=%d", length, topic_id);

    // Read message length + checksum byte.
    async_read_buffer_.read(length + 1, boost::bind(&Session::read_body, this,
                                                    boost::placeholders::_1, topic_id));
  }

  void read_body(SimpleCdr& stream, uint16_t topic_id) {

    if (!is_active()) {
      RCLCPP_DEBUG(node_->get_logger(), "[Rosserial::Session::read_body] Request to shutdown while receiving msg, aborting read_body...");
      return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "[Rosserial::Session::read_body] Received body of length %d for message on topic %d.", stream.getLength(), topic_id);

    uint8_t msg_checksum = checksum(stream) + checksum(topic_id);
    if (msg_checksum != 0xff) {
      RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::read_body] Rejecting message on topicId=%d, name = %s, length=%d with bad checksum.", topic_id, topics_names_[topic_id].c_str(), stream.getLength());
    } else {
      if (callbacks_.count(topic_id) == 1) {
        try {
          // stream includes the check sum byte. 
          SimpleCdr msg_stream(stream.getData(), stream.getLength()-1);
          //fprintf(stderr, "==>>> Callback to topic_id %d\n", topic_id);
          callbacks_[topic_id](msg_stream);
        //} catch(ros::serialization::StreamOverrunException e) {
        } catch(std::exception &e) {
            fprintf(stderr, "EXCEPTION: %s\n", e.what());
          if (topic_id < 100) {
            RCLCPP_ERROR(node_->get_logger(), "[Rosserial::Session::read_body] Buffer overrun when attempting to parse setup message topic_id=%d.", topic_id);
            RCLCPP_ERROR_ONCE(node_->get_logger(), "[Rosserial::Session::read_body] Is this firmware from a pre-Groovy rosserial?");
          } else {
            RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::read_body] Buffer overrun when attempting to parse user message.");
          }
        }
      } else {
        if(topics_names_.size() > topic_id)
        {
          RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::read_body] Received message with unrecognized topicId (%d), number of counts is %u, name is '%s'.", topic_id, static_cast<uint32_t>(callbacks_.count(topic_id)), topics_names_[topic_id].c_str());
        }
        else
        {
          RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::read_body] Received message with unrecognized topicId (%d), number of counts is %u, name not available.", topic_id, static_cast<uint32_t>(callbacks_.count(topic_id)));
        }

        if ((unrecognised_topic_retry_threshold_ > 0) && ++unrecognised_topics_ >= unrecognised_topic_retry_threshold_)
        {
          // The threshold for unrecognised topics has been exceeded.
          // Attempt to request the topics from the client again
          RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::read_body] Unrecognised topic threshold exceeded. Requesting topics from client");
          attempt_sync();
          unrecognised_topics_ = 0;
        }
      }
    }

    // Kickoff next message read.
    read_sync_header();
  }

  void read_failed(const boost::system::error_code& error) {
    if (error == boost::system::errc::no_buffer_space) {
      // No worries. Begin syncing on a new message.
      RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::read_failed] Overrun on receive buffer. Attempting to regain rx sync.");
      read_sync_header();
    } else if (error) {
      // When some other read error has occurred, stop the session, which destroys
      // all known publishers and subscribers.
      RCLCPP_WARN_STREAM(node_->get_logger(), "[Rosserial::Session::read_failed] Socket asio error, closing socket: " << error);
      stop();
    }
  }

  //// SENDING MESSAGES ////
  void write_buffer(BufferPtr buffer_ptr) {  
    boost::asio::async_write(socket_, boost::asio::buffer(*buffer_ptr),
          boost::bind(&Session::write_completion_cb, this, boost::asio::placeholders::error, buffer_ptr));
  }

  void write_message(Buffer& message, const uint16_t topic_id) {
    const uint8_t overhead_bytes = 8; // introduced by rosserial protocol
    uint16_t length = overhead_bytes + message.size();

    BufferPtr buffer_ptr(new Buffer(length));

    uint8_t msg_checksum = 255 - (checksum(message) + checksum(topic_id));
    SimpleCdr stream(buffer_ptr->data(), buffer_ptr->size());
    uint8_t msg_len_checksum = 255 - checksum(message.size());

    stream << (uint16_t)0xfeff << (uint16_t)message.size() << msg_len_checksum << topic_id << message << msg_checksum;
    //fprintf(stderr, "[topic=%d] ", topic_id); stream.print("[write_message]");

    /*
    fprintf(stderr, "[write_message] ");
    for (size_t i = 0; i < buffer_ptr->size(); i++)
    {
        fprintf(stderr, "%02hhX ", buffer_ptr->data()[i]);
    }
    fprintf(stderr, "\n");
    */

    RCLCPP_DEBUG(node_->get_logger(), "[Rosserial::Session::write_message] Sending buffer of %d bytes to client.", length);
    boost::lock_guard<boost::mutex> lock(mutex_); 
    io_service_.dispatch(boost::bind(&Session::write_buffer, this, buffer_ptr));
  }

  void write_completion_cb(const boost::system::error_code& error,
                           BufferPtr buffer_ptr) {
    if (error) {
      if (error == boost::system::errc::io_error) {
        //ROS_WARN_THROTTLE(1, "[Rosserial::Session::write_completion_cb] Socket write operation returned IO error.");
      } else if (error == boost::system::errc::no_such_device) {
        //ROS_WARN_THROTTLE(1, "[Rosserial::Session::write_completion_cb] Socket write operation returned no device.");
      } else {
        //ROS_WARN_STREAM_THROTTLE(1, "[Rosserial::Session::write_completion_cb] Unknown error returned during write operation: " << error);
      }
      stop();
    }
    // Buffer is destructed when this function exits and buffer_ptr goes out of scope.
  }

  //// SYNC WATCHDOG ////
  void attempt_sync() {
    request_topics();
    set_sync_timeout(attempt_interval_);
  }

  void set_sync_timeout(const boost::posix_time::time_duration& interval) {
    if (rclcpp::ok())
    {
      sync_timer_.cancel();
      sync_timer_.expires_from_now(interval);
      sync_timer_.async_wait(boost::bind(&Session::sync_timeout, this,
            boost::asio::placeholders::error));
    }
    else
    {
      shutdown();
    }
  }

  void sync_timeout(const boost::system::error_code& error) {
    if (error != boost::asio::error::operation_aborted) {
      RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::sync_timeout] Sync with device lost.*********");
      stop();
    }
  }

  //// HELPERS ////
  void request_topics() {
    std::vector<uint8_t> message(0);
    RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::request_topics] Sending request topics message for VER2 protocol.");
    write_message(message, rosserial_msgs::msg::TopicInfo::ID_PUBLISHER);

    // Set timer for future point at which to verify the subscribers and publishers
    // created by the client against the expected set given in the parameters.
    require_check_timer_.expires_from_now(require_check_interval_);
    require_check_timer_.async_wait(boost::bind(&Session::required_topics_check, this,
          boost::asio::placeholders::error));
  }

  void required_topics_check(const boost::system::error_code& error) {
    /*
    if (error != boost::asio::error::operation_aborted) {
      if (ros::param::has(require_param_name_)) {
        if (!check_set(require_param_name_ + "/publishers", publishers_) ||
            !check_set(require_param_name_ + "/subscribers", subscribers_)) {
          RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::required_topics_check] Connected client failed to establish the publishers and subscribers dictated by require parameter. Re-requesting topics.");
          request_topics();
        }
      }
    }
    */
  }

  /*
  template<typename M>
  bool check_set(const std::string & param_name, M & map) {
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    rcpputils::assert_true(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < param_list.size(); ++i) {
      rcpputils::assert_true(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      std::string required_topic((std::string(param_list[i])));
      // Iterate through map of registered topics, to ensure that this one is present.
      bool found = false;
      for (typename M::iterator j = map.begin(); j != map.end(); ++j) {
        if (nh_.resolveName(j->second->get_topic()) ==
            nh_.resolveName(required_topic)) {
          found = true;
          RCLCPP_INFO_STREAM(node_->get_logger(), "[Rosserial::Session::check_set] Verified connection to topic " << required_topic << ", given in parameter " << param_name);
          break;
        }
      }
      if (!found) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[Rosserial::Session::check_set] Missing connection to topic " << required_topic << ", required by parameter " << param_name);
        return false;
      }
    }
    return true;
  }
  */
 
  static uint8_t checksum(SimpleCdr& stream) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < stream.getLength(); ++i) {
      sum += stream.getData()[i];
    }
    return sum;
  }

  static uint8_t checksum(Buffer& stream) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < stream.size(); ++i) {
      sum += stream[i];
    }
    return sum;
  }
  

  static uint8_t checksum(uint16_t val) {
    return (val >> 8) + val;
  }

  //// RECEIVED MESSAGE HANDLERS ////

  void setup_publisher(SimpleCdr& stream)
  {
    try {
        rosserial_msgs::msg::TopicInfo topic_info;
        rclcpp::SerializedMessage serialized_msg = stream.asSerializedMessage();
        rclcpp::Serialization<rosserial_msgs::msg::TopicInfo> serializer;
        serializer.deserialize_message(&serialized_msg, &topic_info);

        RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::setup_publisher] Setting up publisher for topic: %s  message type: %s", topic_info.topic_name.c_str(), topic_info.message_type.c_str());

        PublisherPtr pub(new Publisher(node_, topic_info));
        callbacks_[topic_info.topic_id] = std::bind(&Publisher::handle, pub, std::placeholders::_1);
        publishers_[topic_info.topic_id] = pub;
        topics_names_[topic_info.topic_id] = topic_info.topic_name;

        set_sync_timeout(timeout_interval_);
    } catch(...) {
        RCLCPP_ERROR(node_->get_logger(), "Error trying to deserialize TopicInfo buffer");
    }
  }

  void setup_subscriber(SimpleCdr& stream)
  {
    rosserial_msgs::msg::TopicInfo topic_info;
    rclcpp::SerializedMessage serialized_msg = stream.asSerializedMessage();
    rclcpp::Serialization<rosserial_msgs::msg::TopicInfo> serializer;
    serializer.deserialize_message(&serialized_msg, &topic_info);

    RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::setup_subscriber] Setting up subscriber to topic: %s  message type: %s", topic_info.topic_name.c_str(), topic_info.message_type.c_str());

    try
    {
        SubscriberPtr sub(new Subscriber(node_, topic_info,
            boost::bind(&Session::write_message, this, boost::placeholders::_1, topic_info.topic_id)));
        subscribers_[topic_info.topic_id] = sub;
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "[Rosserial::Session::setup_subscriber] Setting up subscriber to topic failed: %s  message type: %s", topic_info.topic_name.c_str(), topic_info.message_type.c_str());
    }


    set_sync_timeout(timeout_interval_);
  }

  // When the rosserial client creates a ServiceClient object (and/or when it registers that object with the NodeHandle)
  // it creates a publisher (to publish the service request message to us) and a subscriber (to receive the response)
  // the service client callback is attached to the *subscriber*, so when we receive the service response
  // and wish to send it over the socket to the client,
  // we must attach the topicId that came from the service client subscriber message

  void setup_service_client_publisher(SimpleCdr& stream) {
    rosserial_msgs::msg::TopicInfo topic_info;
    rclcpp::SerializedMessage serialized_msg = stream.asSerializedMessage();
    rclcpp::Serialization<rosserial_msgs::msg::TopicInfo> serializer;
    serializer.deserialize_message(&serialized_msg, &topic_info);

    if (!service_clients_.count(topic_info.topic_name))
    {
        RCLCPP_DEBUG(node_->get_logger(), "[Rosserial::Session::setup_service_client_publisher] Creating service client for topic %s",topic_info.topic_name.c_str());
        ServiceClientPtr srv(new ServiceClient(
            node_,topic_info,boost::bind(&Session::write_message, this, boost::placeholders::_1, boost::placeholders::_2), services_map_));
        service_clients_[topic_info.topic_name] = srv;
        callbacks_[topic_info.topic_id] = std::bind(&ServiceClient::handle, srv, std::placeholders::_1);
        topics_names_[topic_info.topic_id] = topic_info.topic_name;
    }

    std::string hostHash = service_clients_[topic_info.topic_name]->getRequestMessageMD5();
    if (hostHash != topic_info.md5sum)
    {
        RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::setup_service_client_publisher] Service client setup: Request message MD5 mismatch between rosserial client and ROS: %s\nhost:%s\nembedded:%s", topic_info.topic_name.c_str(), hostHash.c_str(), topic_info.md5sum.c_str());
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "[Rosserial::Session::setup_service_client_publisher] Service client %s: request message MD5 successfully validated as %s",
            topic_info.topic_name.c_str(),topic_info.md5sum.c_str());
    }

    set_sync_timeout(timeout_interval_);
  }

  void setup_service_client_subscriber(SimpleCdr& stream) {
    rosserial_msgs::msg::TopicInfo topic_info;
    rclcpp::SerializedMessage serialized_msg = stream.asSerializedMessage();
    rclcpp::Serialization<rosserial_msgs::msg::TopicInfo> serializer;
    serializer.deserialize_message(&serialized_msg, &topic_info);


    if (!service_clients_.count(topic_info.topic_name)) {
      RCLCPP_DEBUG(node_->get_logger(), "[Rosserial::Session::setup_service_client_subscriber] Creating service client for topic %s",topic_info.topic_name.c_str());
      ServiceClientPtr srv(new ServiceClient(
        node_,topic_info,boost::bind(&Session::write_message, this, boost::placeholders::_1, boost::placeholders::_2), services_map_));
      service_clients_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = std::bind(&ServiceClient::handle, srv, std::placeholders::_1);
      topics_names_[topic_info.topic_id] = topic_info.topic_name;
    }
    // see above comment regarding the service client callback for why we set topic_id here
    service_clients_[topic_info.topic_name]->setTopicId(topic_info.topic_id);
    if (service_clients_[topic_info.topic_name]->getResponseMessageMD5() != topic_info.md5sum) {
      RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::setup_service_client_subscriber] Service client setup: Response message MD5 mismatch between rosserial client and ROS: %s", topic_info.topic_name.c_str());
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "[Rosserial::Session::setup_service_client_subscriber] Service client %s: response message MD5 successfully validated as %s",
        topic_info.topic_name.c_str(),topic_info.md5sum.c_str());
    }
    set_sync_timeout(timeout_interval_);
  }

  // When the rosserial client creates a ServiceServer object (and/or when it registers that object with the NodeHandle)
  // it creates a "proxy" service server
  void setup_service_server_publisher(SimpleCdr& stream) {
    rosserial_msgs::msg::TopicInfo topic_info;
    rclcpp::SerializedMessage serialized_msg = stream.asSerializedMessage();
    rclcpp::Serialization<rosserial_msgs::msg::TopicInfo> serializer;
    serializer.deserialize_message(&serialized_msg, &topic_info);

    if (!service_servers_.count(topic_info.topic_name)) {
      RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::setup_service_server_publisher] Creating service server publisher for topic %s",topic_info.topic_name.c_str());
      ServiceServerPtr srv(new ServiceServer(node_, topic_info, buffer_max, boost::bind(&Session::write_message, this, boost::placeholders::_1, boost::placeholders::_2), services_no_response_timeout_ms_, services_map_));
      service_servers_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = std::bind(&ServiceServer::response_handle, srv, std::placeholders::_1);
      topics_names_[topic_info.topic_id] = topic_info.topic_name;
    }
    if (service_servers_[topic_info.topic_name]->getResponseMessageMD5() != topic_info.md5sum) {
      RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::setup_service_server_publisher] Service server setup: Response message MD5 mismatch between rosserial server and ROS: %s", topic_info.topic_name.c_str());
      RCLCPP_WARN(node_->get_logger(), "%s  and %s", service_servers_[topic_info.topic_name]->getResponseMessageMD5().c_str(), topic_info.md5sum.c_str());
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "[Rosserial::Session::setup_service_server_publisher] Service server %s: response message MD5 successfully validated as %s",
               topic_info.topic_name.c_str(),topic_info.md5sum.c_str());
    }
    set_sync_timeout(timeout_interval_);
  }

  void setup_service_server_subscriber(SimpleCdr& stream) {
    rosserial_msgs::msg::TopicInfo topic_info;
    rclcpp::SerializedMessage serialized_msg = stream.asSerializedMessage();
    rclcpp::Serialization<rosserial_msgs::msg::TopicInfo> serializer;
    serializer.deserialize_message(&serialized_msg, &topic_info);

    if (!service_servers_.count(topic_info.topic_name)) {
      RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::setup_service_server_subscriber] Creating service server subscriber for topic %s",topic_info.topic_name.c_str());
      ServiceServerPtr srv(new ServiceServer(node_, topic_info, buffer_max, boost::bind(&Session::write_message, this, boost::placeholders::_1, boost::placeholders::_2), services_no_response_timeout_ms_, services_map_));
      service_servers_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = std::bind(&ServiceServer::response_handle, srv, std::placeholders::_1);
      topics_names_[topic_info.topic_id] = topic_info.topic_name;
    }

    // see above comment regarding the service server callback for why we set topic_id here
    service_servers_[topic_info.topic_name]->setTopicId(topic_info.topic_id);
    if (service_servers_[topic_info.topic_name]->getRequestMessageMD5() != topic_info.md5sum) {
      RCLCPP_WARN(node_->get_logger(), "[Rosserial::Session::setup_service_server_subscriber] Service server setup: Request message MD5 mismatch between rosserial server and ROS: %s", topic_info.topic_name.c_str());
      RCLCPP_WARN(node_->get_logger(), "%s  and %s", service_servers_[topic_info.topic_name]->getRequestMessageMD5().c_str(), topic_info.md5sum.c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(), "[Rosserial::Session::setup_service_server_subscriber] Service server %s: request message MD5 successfully validated as %s",
               topic_info.topic_name.c_str(),topic_info.md5sum.c_str());
    }
    set_sync_timeout(timeout_interval_);
  }


  void handle_log(SimpleCdr& stream) {
    rosserial_msgs::msg::Log l;
    rclcpp::SerializedMessage serialized_msg = stream.asSerializedMessage();
    rclcpp::Serialization<rosserial_msgs::msg::Log> serializer;
    serializer.deserialize_message(&serialized_msg, &l);

    if(l.level == rosserial_msgs::msg::Log::ROSDEBUG) RCLCPP_DEBUG(node_->get_logger(), "%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::msg::Log::INFO) RCLCPP_INFO(node_->get_logger(), "%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::msg::Log::WARN) RCLCPP_WARN(node_->get_logger(), "%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::msg::Log::ERROR) RCLCPP_ERROR(node_->get_logger(), "%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::msg::Log::FATAL) RCLCPP_FATAL(node_->get_logger(), "%s", l.msg.c_str());
  }

  void handle_time(SimpleCdr& stream) {
    /*
    fprintf(stderr, "[handle_time::received] ");
    for (size_t i = 0; i < stream.getLength(); ++i) {
      fprintf(stderr, "%02x ", stream.getData()[i]);
    }
    fprintf(stderr, "\n");
    */

    rclcpp::Time time = node_->get_clock()->now();
    builtin_interfaces::msg::Time time_msg(time);
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<builtin_interfaces::msg::Time> serializer;
    serializer.serialize_message(&time_msg, &serialized_msg);

    /*
    fprintf(stderr, "[handle_time::send    ] ");
    const rcl_serialized_message_t & sm = serialized_msg.get_rcl_serialized_message();
    for (size_t i = 0; i < serialized_msg.size(); ++i) {
      fprintf(stderr, "%02x ", sm.buffer[i]);
    }
    fprintf(stderr, "\n");
    */

    std::vector<uint8_t> message(serialized_msg.size());
    memcpy(message.data(), serialized_msg.get_rcl_serialized_message().buffer, message.size());
    write_message(message, rosserial_msgs::msg::TopicInfo::ID_TIME);

    // The MCU requesting the time from the server is the sync notification. This
    // call moves the timeout forward.
    set_sync_timeout(timeout_interval_);
  }

  boost::asio::io_service& io_service_;
  Socket socket_;
  AsyncReadBuffer<Socket> async_read_buffer_;
  enum { buffer_max = 8192-1 };
  bool active_;
  int services_no_response_timeout_ms_;

  boost::mutex mutex_;

  boost::posix_time::time_duration timeout_interval_;
  boost::posix_time::time_duration attempt_interval_;
  boost::posix_time::time_duration require_check_interval_;
  boost::posix_time::time_duration ros_spin_interval_;
  boost::asio::deadline_timer sync_timer_;
  boost::asio::deadline_timer require_check_timer_;
  boost::asio::deadline_timer ros_spin_timer_;
  std::string require_param_name_;
  int unrecognised_topic_retry_threshold_{ 0 };
  int unrecognised_topics_{ 0 };

  std::map<uint16_t, std::function<void(SimpleCdr&)> > callbacks_;
  std::map<uint16_t, PublisherPtr> publishers_;
  std::map<uint16_t, SubscriberPtr> subscribers_;
  std::map<std::string, ServiceClientPtr> service_clients_;
  std::map<std::string, ServiceServerPtr> service_servers_;
  std::map<uint16_t, std::string> topics_names_;

  ConversionMap services_map_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_SESSION_H
