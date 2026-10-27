#ifndef ROSSERIAL_HARDWARE__ROSSERIAL_SYSTEM_HPP_
#define ROSSERIAL_HARDWARE__ROSSERIAL_SYSTEM_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "rosserial_hardware/serial_link.hpp"

namespace rosserial_hardware
{

/**
 * ros2_control SystemInterface for a rosserial MCU with one digital output (e.g. an LED).
 * It only translates values: the double of the command interface becomes a
 * std_msgs/Bool for the board (> 0.5 is true) and the Bool the board reports
 * becomes the double of the state interface (1.0 / 0.0).
 *
 * Hardware parameters (URDF <ros2_control><hardware>):
 *   port          serial device               (default /dev/ttyACM0)
 *   baud          baud rate                   (default 57600)
 *   state_topic   Bool the board publishes    (default led_state)
 *   command_topic Bool the board subscribes   (default led)
 *
 * Declare exactly one command interface (joint or gpio) and at most one state interface.
 */
class RosserialSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void on_state_payload(const SerialLink::Payload & cdr);

  std::unique_ptr<SerialLink> link_;

  std::string state_name_;    // "gpio/interface", empty if no state interface
  std::string command_name_;  // "gpio/interface"

  std::atomic<bool> state_received_{false};
  std::atomic<bool> state_value_{false};

  bool last_sent_valid_{false};
  bool last_sent_{false};
  std::chrono::steady_clock::time_point last_sent_time_;
};

}  // namespace rosserial_hardware

#endif  // ROSSERIAL_HARDWARE__ROSSERIAL_SYSTEM_HPP_
