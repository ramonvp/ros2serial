#include "rosserial_hardware/rosserial_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/serialization.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rosserial_hardware
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("RosserialSystem");
using BoolMsg = std_msgs::msg::Bool;

std::string param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}
}  // namespace

hardware_interface::CallbackReturn RosserialSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  std::vector<std::string> states, commands;
  for (const auto & c : info_.joints) {
    for (const auto & i : c.state_interfaces) {states.push_back(c.name + "/" + i.name);}
    for (const auto & i : c.command_interfaces) {commands.push_back(c.name + "/" + i.name);}
  }
  for (const auto & c : info_.gpios) {
    for (const auto & i : c.state_interfaces) {states.push_back(c.name + "/" + i.name);}
    for (const auto & i : c.command_interfaces) {commands.push_back(c.name + "/" + i.name);}
  }
  if (commands.size() != 1 || states.size() > 1) {
    RCLCPP_ERROR(
      kLogger, "Expected exactly 1 command interface and at most 1 state interface, got %zu / %zu",
      commands.size(), states.size());
    return CallbackReturn::ERROR;
  }
  command_name_ = commands[0];
  if (!states.empty()) {state_name_ = states[0];}

  const auto & p = info_.hardware_parameters;
  int baud = 57600;
  try {
    baud = std::stoi(param(p, "baud", "57600"));
  } catch (const std::exception &) {
    RCLCPP_ERROR(kLogger, "Invalid 'baud' parameter");
    return CallbackReturn::ERROR;
  }

  link_ = std::make_unique<SerialLink>(
    param(p, "port", "/dev/ttyACM0"), baud, get_node()->get_clock());
  link_->set_rx_topic(
    param(p, "state_topic", "led_state"),
    [this](const SerialLink::Payload & cdr) {on_state_payload(cdr);});
  link_->set_tx_topic(param(p, "command_topic", "led"));
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RosserialSystem::on_configure(const rclcpp_lifecycle::State &)
{
  link_->start();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RosserialSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  link_->stop();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RosserialSystem::on_activate(const rclcpp_lifecycle::State &)
{
  set_command(command_name_, 0.0);
  if (!state_name_.empty()) {set_state(state_name_, 0.0);}
  last_sent_valid_ = false;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RosserialSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void RosserialSystem::on_state_payload(const SerialLink::Payload & cdr)
{
  rclcpp::SerializedMessage serialized(cdr.size());
  std::memcpy(serialized.get_rcl_serialized_message().buffer, cdr.data(), cdr.size());
  serialized.get_rcl_serialized_message().buffer_length = cdr.size();

  BoolMsg msg;
  rclcpp::Serialization<BoolMsg>().deserialize_message(&serialized, &msg);
  state_value_ = msg.data;
  state_received_ = true;
}

hardware_interface::return_type RosserialSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!link_->is_connected()) {
    RCLCPP_WARN_THROTTLE(kLogger, *get_node()->get_clock(), 5000, "Board not connected");
    return hardware_interface::return_type::OK;
  }
  if (!state_name_.empty() && state_received_) {
    set_state(state_name_, state_value_ ? 1.0 : 0.0);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RosserialSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!link_->is_connected()) {
    last_sent_valid_ = false;  // resend as soon as the board is back
    return hardware_interface::return_type::OK;
  }
  const double v = get_command<double>(command_name_);
  const bool value = !std::isnan(v) && v > 0.5;

  // Send on change, and once a second so a board that reset still gets the value.
  const auto now = std::chrono::steady_clock::now();
  if (last_sent_valid_ && value == last_sent_ &&
    now - last_sent_time_ < std::chrono::seconds(1))
  {
    return hardware_interface::return_type::OK;
  }

  BoolMsg msg;
  msg.data = value;
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<BoolMsg>().serialize_message(&msg, &serialized);
  const auto & raw = serialized.get_rcl_serialized_message();
  if (link_->send(SerialLink::Payload(raw.buffer, raw.buffer + raw.buffer_length))) {
    last_sent_valid_ = true;
    last_sent_ = value;
    last_sent_time_ = now;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace rosserial_hardware

PLUGINLIB_EXPORT_CLASS(rosserial_hardware::RosserialSystem, hardware_interface::SystemInterface)
