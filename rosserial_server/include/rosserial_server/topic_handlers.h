/**
 *
 *  \file
 *  \brief      Classes which manage the Publish and Subscribe relationships
 *              that a Session has with its client.
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

#ifndef ROSSERIAL_SERVER_TOPIC_HANDLERS_H
#define ROSSERIAL_SERVER_TOPIC_HANDLERS_H

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <rosserial_msgs/msg/topic_info.hpp>
#include <rosserial_server/msg_lookup.h>
#include <rosserial_server/simplecdr.h>
#include <rosserial_server/services.h>

namespace rosserial_server
{

class Publisher {
public:
  Publisher(std::shared_ptr<rclcpp::Node> node, const rosserial_msgs::msg::TopicInfo& topic_info)
  {
    rosserial_server::MsgInfo msginfo;
    try
    {
      msginfo = lookupMessage(topic_info.message_type);
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "[Publisher] Unable to look up message: " << e.what());
    }

    if (!msginfo.md5sum.empty() && msginfo.md5sum != topic_info.md5sum)
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "[Publisher] Message" << topic_info.message_type  << "MD5 sum from client does not "
                      "match that in system. Will avoid using system's message definition.");
      msginfo.full_text = "";
    }

    /*if (!topics_group_) {
      topics_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    rclcpp::PublisherOptions publisher_options;
    publisher_options.callback_group = topics_group_;
    */
    publisher_ = node->create_generic_publisher(topic_info.topic_name, topic_info.message_type, rclcpp::QoS(1)/*, publisher_options*/);
  }

  void handle(SimpleCdr& stream) {
    rcutils_uint8_array_t rcutil_array;
    rcutil_array.buffer = stream.getData();
    rcutil_array.buffer_length = stream.getLength();
    rcutil_array.buffer_capacity = stream.getLength();
    rcutil_array.allocator = rcl_get_default_allocator();
    rclcpp::SerializedMessage message(rcutil_array);
    publisher_->publish(message);
  }

  std::string get_topic() {
    return std::string(publisher_->get_topic_name());
  }

private:
   std::shared_ptr<rclcpp::GenericPublisher> publisher_;
   //rclcpp::CallbackGroup::SharedPtr topics_group_;
};

typedef std::shared_ptr<Publisher> PublisherPtr;


class Subscriber {
public:
  Subscriber(std::shared_ptr<rclcpp::Node> node, rosserial_msgs::msg::TopicInfo& topic_info,
      boost::function<void(std::vector<uint8_t>& buffer)> write_fn)
    : write_fn_(write_fn)
  {
    rosserial_server::MsgInfo msginfo;
    try
    {
      msginfo = lookupMessage(topic_info.message_type);
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "[Subscriber] Unable to look up message: " << e.what());
    }

    if (!msginfo.md5sum.empty() && msginfo.md5sum != topic_info.md5sum)
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "[Subscriber] Message" << topic_info.message_type  << "MD5 sum from client does not "
                      "match that in system. Will avoid using system's message definition.");
      msginfo.full_text = "";
    }

    RCLCPP_INFO(node->get_logger(), "[Subscriber] Creating generic subscription to topic %s", topic_info.topic_name.c_str());
    subscriber_ = node->create_generic_subscription(
        topic_info.topic_name, topic_info.message_type, rclcpp::QoS(1), std::bind(&Subscriber::handle, this, std::placeholders::_1));

  }

  std::string get_topic() {
    return std::string(subscriber_->get_topic_name());
  }

private:
  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
  {
    std::vector<uint8_t> message(serialized_msg->size());
    memcpy(message.data(), serialized_msg->get_rcl_serialized_message().buffer, serialized_msg->size());
    write_fn_(message);
  }
  
  std::shared_ptr<rclcpp::GenericSubscription> subscriber_{nullptr};
  boost::function<void(std::vector<uint8_t>& buffer)> write_fn_;
};

typedef std::shared_ptr<Subscriber> SubscriberPtr;


class ServiceClient {
public:
    ServiceClient(std::shared_ptr<rclcpp::Node> node, rosserial_msgs::msg::TopicInfo& topic_info,
        boost::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn,
        const ConversionMap &services_map)
        : node_(node), write_fn_(write_fn)
    {
        topic_id_ = -1;

        service_type_ = topic_info.message_type;

        // First, attempt to find the service converter in the table. If not present
        // we cannot convert the serialized message into the correct request and response
        if (services_map.find(service_type_) == services_map.end()) {
            RCLCPP_WARN_STREAM(node_->get_logger(), "[ServiceClient] No service converted defined for service: " << service_type_);
            return;
        }

        service_converter_ = services_map.at(service_type_);
        valid_ = true;

        rosserial_server::MsgInfo srvinfo;
        rosserial_server::MsgInfo reqinfo;
        rosserial_server::MsgInfo respinfo;
        try
        {
            srvinfo = lookupMessage(topic_info.message_type);
            reqinfo = lookupMessage(topic_info.message_type, "_Request");
            respinfo = lookupMessage(topic_info.message_type, "_Response");
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "[ServiceClient] Unable to look up service definition: " << e.what());
        }
        service_md5_ = srvinfo.md5sum;
        request_message_md5_ = reqinfo.md5sum;
        response_message_md5_ = respinfo.md5sum;

        //ros::ServiceClientOptions opts;
        //opts.service = topic_info.topic_name;
        //opts.md5sum = srvinfo.md5sum;
        //opts.persistent = false; // always false for now
        //service_client_ = nh.serviceClient(opts);

        //if (!services_group_) {
        //  services_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        //}

        RCLCPP_INFO(node_->get_logger(), "[ServiceClient] Setting up service client %s of type %s", topic_info.topic_name.c_str(), topic_info.message_type.c_str());
        service_client_ = node_->create_generic_client(topic_info.topic_name, topic_info.message_type/*, rclcpp::ServicesQoS(), services_group_*/);
    }

    void setTopicId(uint16_t topic_id) {
        topic_id_ = topic_id;
    }

    std::string getServiceMD5() {
        return service_md5_;
    }

    std::string getRequestMessageMD5() {
        return request_message_md5_;
    }

    std::string getResponseMessageMD5() {
        return response_message_md5_;
    }

  void handle(SimpleCdr & stream) {

    using namespace std::chrono_literals;

    if (!valid_)
        return;

    rclcpp::SerializedMessage serialized_request = stream.asSerializedMessage();
    rclcpp::GenericClient::Request request = service_converter_->request_from_serialized_message(serialized_request);
    rclcpp::GenericClient::FutureAndRequestId result_future = service_client_->async_send_request(request);
    std::future_status status = result_future.wait_for(3s);  // timeout to guarantee a graceful finish
    if (status == std::future_status::ready) {
        rclcpp::GenericClient::SharedResponse response = result_future.get();
        rclcpp::SerializedMessage serialized_response = service_converter_->serialized_message_from_response(response.get());
        std::vector<uint8_t> response_message(serialized_response.size());
        memcpy(response_message.data(), serialized_response.get_rcl_serialized_message().buffer, serialized_response.size());
        write_fn_(response_message, topic_id_);
    } else {
        // handle timeout
        service_client_->remove_pending_request(result_future.request_id);
        RCLCPP_ERROR(node_->get_logger(), "[ServiceClient::handle] Timeout invoking service %s", service_client_->get_service_name());
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::GenericClient::SharedPtr service_client_{nullptr};
  boost::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn_;
  std::string service_md5_;
  std::string request_message_md5_;
  std::string response_message_md5_;
  std::string service_type_;
  uint16_t topic_id_;
  bool valid_{false};
  ConverterInterfacePtr service_converter_;
  //rclcpp::CallbackGroup::SharedPtr services_group_;
};

typedef std::shared_ptr<ServiceClient> ServiceClientPtr;


class ServiceServer {
public:
  ServiceServer(std::shared_ptr<rclcpp::Node> node, rosserial_msgs::msg::TopicInfo& topic_info, size_t capacity,
      boost::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn, int timeout_no_response_ms,
      const ConversionMap &services_map)
    : node_(node), write_fn_(write_fn) {
    response_buffer_.resize(capacity, 0);
    topic_id_ = -1;
    get_response_ = false;
    service_name_ = topic_info.topic_name;
    service_type_ = topic_info.message_type;
    capacity_ = capacity;
    timeout_no_response_ms_ = timeout_no_response_ms;

    // First, attempt to find the service converter in the table. If not present
    // we cannot convert the serialized message into the correct request and response
    if (services_map.find(service_type_) == services_map.end()) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[ServiceServer] No service converted defined for service: " << service_type_);
        return;
    }

    service_converter_ = services_map.at(service_type_);

    rosserial_server::MsgInfo srvinfo;
    rosserial_server::MsgInfo reqinfo;
    rosserial_server::MsgInfo respinfo;
    try
    {
      srvinfo = lookupMessage(topic_info.message_type);
      reqinfo = lookupMessage(topic_info.message_type, "_Request");
      respinfo = lookupMessage(topic_info.message_type, "_Response");
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[ServiceServer] Unable to look up service definition: " << e.what());
    }

    service_md5_ = srvinfo.md5sum;
    request_message_md5_ = reqinfo.md5sum;
    response_message_md5_ = respinfo.md5sum;

    using namespace std::placeholders;
    service_converter_->start_server(node, service_name_, std::bind(&ServiceServer::service_callback, this, _1, _2));
  }

  void setTopicId(uint16_t topic_id) {
    topic_id_ = topic_id;
  }

  std::string getServiceMD5() {
    return service_md5_;
  }

  std::string getRequestMessageMD5() {
    return request_message_md5_;
  }

  std::string getResponseMessageMD5() {
    return response_message_md5_;
  }

  const std::string& getServiceName() {
    return service_name_;
  }

  bool service_callback(const rclcpp::SerializedMessage& serialized_request, rclcpp::SerializedMessage &serialized_response)
  {
        using namespace std::chrono_literals;

        get_response_ = false;
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "[ServiceServer::service_callback] service: " << getServiceName() << " request");

        std::vector<uint8_t> request_buffer(serialized_request.size());
        memcpy(request_buffer.data(), serialized_request.get_rcl_serialized_message().buffer, serialized_request.size());
        write_fn_(request_buffer, topic_id_);

        //wait for the response
        int cnt_ms = 0;
        while (!get_response_) {
            rclcpp::sleep_for(1ms);
            if (cnt_ms++ > timeout_no_response_ms_) {
                //printf("Service Name address: %X\n", service_name_.c_str());
                //printf("Service Name is: %s\n", service_name_.c_str());
                RCLCPP_WARN_STREAM(node_->get_logger(), "[ServiceServer::request_handle]" << getServiceName() << ": no response!!");
                return false;
            }
        }

        //ROS_INFO("buffer_len = %zu  response_buffer_size = %zu", buffer_len_, response_buffer_.size());
        if (buffer_len_ > capacity_)
        {
            RCLCPP_ERROR(node_->get_logger(), "[ServiceServer::request_handle] Buffer length (%zu) is bigger than response buffer size (%zu), ignoring", buffer_len_, response_buffer_.size());
            return false;
        }

        rcutils_uint8_array_t rcutil_array;
        rcutil_array.buffer = response_buffer_.data();
        rcutil_array.buffer_length = buffer_len_;
        rcutil_array.buffer_capacity = buffer_len_;
        rcutil_array.allocator = rcl_get_default_allocator();
        serialized_response = rclcpp::SerializedMessage(rcutil_array);

        return true;
  }

  void response_handle(SimpleCdr & stream) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "[ServiceServer::response_handle] response from remote: " << getServiceName() );

    if (stream.getLength() > capacity_)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "[ServiceServer::response_handle] ****** Service receive " << getServiceName() << "unexpected big data stream");
      get_response_ = false;
    }
    else
    {
      const uint8_t* data = stream.getData();
      buffer_len_ = stream.getLength();
      if ( (data == nullptr) || (buffer_len_ == 0) )
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "[ServiceServer::response_handle] Received null data stream for service " << getServiceName());
        get_response_ = false;
      }
      else
      {
        std::memcpy(response_buffer_.data(), data, buffer_len_);
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "[ServiceServer::response_handle] service receive " << getServiceName() << " response");
        get_response_ = true;
      }
    }
  }

private:
  std::vector<uint8_t> response_buffer_;
  size_t buffer_len_;
  boost::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn_;
  std::string service_md5_;
  std::string request_message_md5_;
  std::string response_message_md5_;
  std::string service_name_;
  std::string service_type_;
  uint16_t topic_id_;
  size_t capacity_;
  bool get_response_;
  int timeout_no_response_ms_;
  std::shared_ptr<rclcpp::Node> node_;
  ConverterInterfacePtr service_converter_;
};

typedef std::shared_ptr<ServiceServer> ServiceServerPtr;

}  // namespace rosserial_server

#endif  // ROSSERIAL_SERVER_TOPIC_HANDLERS_H
