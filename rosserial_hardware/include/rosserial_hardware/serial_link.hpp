#ifndef ROSSERIAL_HARDWARE__SERIAL_LINK_HPP_
#define ROSSERIAL_HARDWARE__SERIAL_LINK_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/clock.hpp"

namespace rosserial_hardware
{

/**
 * Minimal rosserial (protocol v2) endpoint over a serial port.
 *
 * Same wire protocol as rosserial_server::Session, but topics announced by the
 * MCU are not turned into ROS publishers/subscribers: payloads are handed to
 * callbacks and written with send(). A background thread owns the port.
 */
class SerialLink
{
public:
  using Payload = std::vector<uint8_t>;  // full CDR message, including 4 byte header
  using RxCallback = std::function<void(const Payload &)>;

  SerialLink(std::string port, int baud, rclcpp::Clock::SharedPtr clock);
  ~SerialLink();

  // Topic names as seen by the MCU. rx: MCU publishes it; tx: MCU subscribes to it.
  void set_rx_topic(const std::string & name, RxCallback cb);
  void set_tx_topic(const std::string & name);

  void start();
  void stop();

  // true once the MCU has announced both the rx and tx topics and is in sync.
  bool is_connected() const { return connected_.load(); }

  // Sends a full CDR payload on the tx topic. Returns false if not connected.
  bool send(const Payload & cdr);

private:
  void run();
  bool open_port();
  void close_port();
  bool write_frame(uint16_t topic_id, const Payload & body);
  void request_topics();
  bool read_frame(uint16_t & topic_id, Payload & body, int timeout_ms);
  void handle(uint16_t topic_id, const Payload & body);
  void handle_topic_info(uint16_t kind, const Payload & body);
  void handle_time();
  void handle_log(const Payload & body);

  std::string port_;
  int baud_;
  rclcpp::Clock::SharedPtr clock_;

  int fd_{-1};
  std::mutex write_mutex_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};

  std::string rx_name_, tx_name_;
  RxCallback rx_cb_;
  std::atomic<int32_t> rx_id_{-1};
  std::atomic<int32_t> tx_id_{-1};

  std::chrono::steady_clock::time_point last_sync_;
  std::chrono::steady_clock::time_point last_request_;
  Payload rx_buf_;
};

}  // namespace rosserial_hardware

#endif  // ROSSERIAL_HARDWARE__SERIAL_LINK_HPP_
