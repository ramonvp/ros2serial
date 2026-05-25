#ifndef ROSSERIAL_SERVICES_HPP
#define ROSSERIAL_SERVICES_HPP

#include <map>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

namespace rosserial_server
{

class ConverterInterface
{
public:

    typedef std::function<bool(const rclcpp::SerializedMessage& serialized_request, rclcpp::SerializedMessage &serialized_response)> ServiceCallbackT;

    virtual ~ConverterInterface() {}

    virtual void print_something(const char* message) = 0;

    virtual void* request_from_serialized_message(const rclcpp::SerializedMessage & serialized_request) = 0;

    virtual rclcpp::SerializedMessage serialized_message_from_request(const void* request) = 0;

    virtual void* response_from_serialized_message(const rclcpp::SerializedMessage & serialized_request) = 0;

    virtual rclcpp::SerializedMessage serialized_message_from_response(const void* response) = 0;

    virtual void start_server(std::shared_ptr<rclcpp::Node> node, const std::string &service_name, ServiceCallbackT service_callback) = 0;
};

typedef std::shared_ptr<ConverterInterface> ConverterInterfacePtr;

template <typename ServiceT>
class Converter : public ConverterInterface
{
public:

    virtual ~Converter() {}

    void start_server(std::shared_ptr<rclcpp::Node> node, const std::string &service_name, ServiceCallbackT callback) override
    {
        using namespace std::placeholders;

        callback_ = callback;

        service_server_ = node->create_service<ServiceT>(service_name, 
            std::bind(&Converter::service_callback, this, _1, _2));
    }

    void service_callback(const std::shared_ptr<typename ServiceT::Request> request,
                                std::shared_ptr<typename ServiceT::Response> response)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[Converter::service_callback] Incoming request");
        rclcpp::SerializedMessage serialized_request = serialized_message_from_request(request.get());
        rclcpp::SerializedMessage serialized_response;
        
        bool success = callback_(serialized_request, serialized_response);
        if (!success) {
            fprintf(stderr, "Service call failed: %s\n", service_server_->get_service_name());
        } else {
            typename ServiceT::Response* remote_response = static_cast<typename ServiceT::Response*>(response_from_serialized_message(serialized_response));
            *response = *remote_response;
        }
    }

    void print_something(const char* message) override
    {
        fprintf(stderr, "Converter: %s\n", message);
    }

    void* request_from_serialized_message(const rclcpp::SerializedMessage & serialized_request) override
    {
        static typename ServiceT::Request request;

        rclcpp::Serialization<typename ServiceT::Request> serializer;
        serializer.deserialize_message(&serialized_request, &request);

        return &request;
    }

    rclcpp::SerializedMessage serialized_message_from_request(const void* request) override
    {
        rclcpp::SerializedMessage serialized_request;
        rclcpp::Serialization<typename ServiceT::Request> serializer;
        serializer.serialize_message(request, &serialized_request);
        return serialized_request;
    }

    void* response_from_serialized_message(const rclcpp::SerializedMessage & serialized_response) override
    {
        static typename ServiceT::Response response;

        rclcpp::Serialization<typename ServiceT::Response> serializer;
        serializer.deserialize_message(&serialized_response, &response);

        return &response;
    }

    rclcpp::SerializedMessage serialized_message_from_response(const void* response) override
    {
        rclcpp::SerializedMessage serialized_response;
        rclcpp::Serialization<typename ServiceT::Response> serializer;
        serializer.serialize_message(response, &serialized_response);
        return serialized_response;
    }

    typename rclcpp::Service<ServiceT>::SharedPtr service_server_;

    ServiceCallbackT callback_;

};

// interface_type is something like std_msgs::msg::Int32, std_srvs::srv::SetBool
// expected output: std_msgs/msg/Int32
std::string replace_colon(std::string interface_type)
{
    size_t index = 0;
    while (true) {
        /* Locate the substring to replace. */
        index = interface_type.find("::");
        if (index == std::string::npos) break;

        /* Make the replacement. */
        interface_type.replace(index, 2, "/");

        /* Advance index forward so the next iteration doesn't pick it up as well. */
        index += 3;
    }

    return interface_type;
}

typedef std::map<std::string, ConverterInterfacePtr> ConversionMap;

} // namespace rosserial_server

// Some useful macros for filling quickly a map table
#define MAKE_CONVERTER(SRV_TYPE) std::make_shared<rosserial_server::Converter<SRV_TYPE>>()
#define ADD_MAP_ENTRY(map, srv_entry) map[ rosserial_server::replace_colon(#srv_entry) ] = MAKE_CONVERTER(srv_entry)


#endif
