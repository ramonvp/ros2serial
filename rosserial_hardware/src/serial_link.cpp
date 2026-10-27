#include "rosserial_hardware/serial_link.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosserial_msgs/msg/log.hpp"
#include "rosserial_msgs/msg/topic_info.hpp"

namespace rosserial_hardware
{

namespace
{
using rosserial_msgs::msg::TopicInfo;

const rclcpp::Logger kLogger = rclcpp::get_logger("rosserial_hardware.serial_link");

constexpr auto kSyncTimeout = std::chrono::milliseconds(5000);
constexpr auto kRequestInterval = std::chrono::milliseconds(1500);

speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return B0;
  }
}

std::string normalize(const std::string & name)
{
  return (!name.empty() && name.front() == '/') ? name.substr(1) : name;
}

rclcpp::SerializedMessage to_serialized(const std::vector<uint8_t> & data)
{
  rclcpp::SerializedMessage msg(data.size());
  std::memcpy(msg.get_rcl_serialized_message().buffer, data.data(), data.size());
  msg.get_rcl_serialized_message().buffer_length = data.size();
  return msg;
}

uint8_t sum(const std::vector<uint8_t> & v, uint8_t acc = 0)
{
  for (auto b : v) {acc += b;}
  return acc;
}
}  // namespace

SerialLink::SerialLink(std::string port, int baud, rclcpp::Clock::SharedPtr clock)
: port_(std::move(port)), baud_(baud), clock_(std::move(clock))
{
}

SerialLink::~SerialLink() {stop();}

void SerialLink::set_rx_topic(const std::string & name, RxCallback cb)
{
  rx_name_ = normalize(name);
  rx_cb_ = std::move(cb);
}

void SerialLink::set_tx_topic(const std::string & name) {tx_name_ = normalize(name);}

void SerialLink::start()
{
  if (running_.exchange(true)) {return;}
  thread_ = std::thread(&SerialLink::run, this);
}

void SerialLink::stop()
{
  if (!running_.exchange(false)) {return;}
  if (thread_.joinable()) {thread_.join();}
  connected_ = false;
  close_port();
}

bool SerialLink::open_port()
{
  speed_t speed = to_speed(baud_);
  if (speed == B0) {
    RCLCPP_ERROR(kLogger, "Unsupported baud rate %d", baud_);
    return false;
  }
  int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_DEBUG(kLogger, "Unable to open %s: %s", port_.c_str(), std::strerror(errno));
    return false;
  }
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    ::close(fd);
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    ::close(fd);
    return false;
  }
  RCLCPP_INFO(kLogger, "Opened %s at %d bps, waiting for the board to boot", port_.c_str(), baud_);
  // Arduino boards reset when DTR toggles on open; give the firmware time to start.
  for (int i = 0; i < 20 && running_; ++i) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
  tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  rx_buf_.clear();
  last_request_ = std::chrono::steady_clock::time_point{};
  last_sync_ = std::chrono::steady_clock::now();
  return true;
}

void SerialLink::close_port()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialLink::run()
{
  while (running_) {
    if (fd_ < 0) {
      if (!open_port()) {
        for (int i = 0; i < 10 && running_; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        continue;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (!connected_ && now - last_request_ > kRequestInterval) {
      request_topics();
      last_request_ = now;
    }
    if (connected_ && now - last_sync_ > kSyncTimeout) {
      RCLCPP_WARN(kLogger, "Sync with the board lost, requesting topics again");
      connected_ = false;
      rx_id_ = -1;
      tx_id_ = -1;
      last_sync_ = now;
    }

    uint16_t topic_id;
    Payload body;
    if (!read_frame(topic_id, body, 100)) {
      if (fd_ < 0) {connected_ = false; rx_id_ = -1; tx_id_ = -1;}
      continue;
    }
    handle(topic_id, body);
  }
}

// Returns true when a complete, checksum-valid frame was parsed. Closes the port
// on I/O errors. Bytes are accumulated in rx_buf_, so partial frames survive timeouts.
bool SerialLink::read_frame(uint16_t & topic_id, Payload & body, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (true) {
    // Resync on the 0xff 0xfe header.
    while (rx_buf_.size() >= 2 && !(rx_buf_[0] == 0xff && rx_buf_[1] == 0xfe)) {
      rx_buf_.erase(rx_buf_.begin());
    }
    if (rx_buf_.size() >= 7) {
      const uint16_t len = rx_buf_[2] | (rx_buf_[3] << 8);
      const uint8_t len_chk = rx_buf_[4];
      if (static_cast<uint8_t>(rx_buf_[2] + rx_buf_[3] + len_chk) != 0xff) {
        RCLCPP_WARN(kLogger, "Bad length checksum, dropping byte");
        rx_buf_.erase(rx_buf_.begin());
        continue;
      }
      if (rx_buf_.size() >= static_cast<size_t>(len) + 8) {
        topic_id = rx_buf_[5] | (rx_buf_[6] << 8);
        body.assign(rx_buf_.begin() + 7, rx_buf_.begin() + 7 + len);
        const uint8_t chk = rx_buf_[7 + len];
        const uint8_t total = static_cast<uint8_t>(rx_buf_[5] + rx_buf_[6] + sum(body) + chk);
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 8 + len);
        if (total != 0xff) {
          RCLCPP_WARN(kLogger, "Bad message checksum on topic %u, dropped", topic_id);
          continue;
        }
        return true;
      }
    }

    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (left <= 0) {return false;}
    pollfd pfd{fd_, POLLIN, 0};
    int r = ::poll(&pfd, 1, static_cast<int>(left));
    if (r <= 0) {return false;}
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      RCLCPP_WARN(kLogger, "Serial port error, closing %s", port_.c_str());
      close_port();
      return false;
    }
    uint8_t tmp[512];
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n < 0 && errno != EAGAIN && errno != EINTR) {
      RCLCPP_WARN(kLogger, "Read failed (%s), closing %s", std::strerror(errno), port_.c_str());
      close_port();
      return false;
    }
    if (n > 0) {rx_buf_.insert(rx_buf_.end(), tmp, tmp + n);}
  }
}

bool SerialLink::write_frame(uint16_t topic_id, const Payload & body)
{
  Payload frame;
  frame.reserve(body.size() + 8);
  const uint16_t len = static_cast<uint16_t>(body.size());
  const uint8_t len_chk = 255 - static_cast<uint8_t>((len >> 8) + len);
  const uint8_t chk = 255 - static_cast<uint8_t>((topic_id >> 8) + topic_id + sum(body));
  frame.insert(
    frame.end(), {0xff, 0xfe, static_cast<uint8_t>(len & 0xff), static_cast<uint8_t>(len >> 8),
      len_chk, static_cast<uint8_t>(topic_id & 0xff), static_cast<uint8_t>(topic_id >> 8)});
  frame.insert(frame.end(), body.begin(), body.end());
  frame.push_back(chk);

  std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd_ < 0) {return false;}
  size_t off = 0;
  int stalls = 0;
  while (off < frame.size()) {
    ssize_t n = ::write(fd_, frame.data() + off, frame.size() - off);
    if (n > 0) {
      off += n;
      stalls = 0;
    } else if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
      // Bounded wait so a stuck port can never block the control loop for long.
      pollfd pfd{fd_, POLLOUT, 0};
      ::poll(&pfd, 1, 2);
      if (++stalls > 5) {return false;}
    } else {
      return false;
    }
  }
  return true;
}

void SerialLink::request_topics()
{
  RCLCPP_INFO(kLogger, "Requesting topics from the board");
  write_frame(TopicInfo::ID_PUBLISHER, {});
}

bool SerialLink::send(const Payload & cdr)
{
  const int32_t id = tx_id_;
  if (!connected_ || id < 0) {return false;}
  return write_frame(static_cast<uint16_t>(id), cdr);
}

void SerialLink::handle(uint16_t topic_id, const Payload & body)
{
  try {
    if (topic_id == TopicInfo::ID_PUBLISHER || topic_id == TopicInfo::ID_SUBSCRIBER) {
      handle_topic_info(topic_id, body);
    } else if (topic_id == TopicInfo::ID_TIME) {
      handle_time();
    } else if (topic_id == TopicInfo::ID_LOG) {
      handle_log(body);
    } else if (static_cast<int32_t>(topic_id) == rx_id_.load()) {
      last_sync_ = std::chrono::steady_clock::now();
      if (rx_cb_) {rx_cb_(body);}
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(kLogger, "Error handling message on topic %u: %s", topic_id, e.what());
  }
}

void SerialLink::handle_topic_info(uint16_t kind, const Payload & body)
{
  TopicInfo info;
  auto serialized = to_serialized(body);
  rclcpp::Serialization<TopicInfo>().deserialize_message(&serialized, &info);
  const std::string name = normalize(info.topic_name);

  if (kind == TopicInfo::ID_PUBLISHER && name == rx_name_) {
    rx_id_ = info.topic_id;
  } else if (kind == TopicInfo::ID_SUBSCRIBER && name == tx_name_) {
    tx_id_ = info.topic_id;
  } else {
    RCLCPP_WARN(
      kLogger, "Ignoring %s '%s' announced by the board (expected '%s' / '%s')",
      kind == TopicInfo::ID_PUBLISHER ? "publisher" : "subscriber", name.c_str(),
      rx_name_.c_str(), tx_name_.c_str());
    return;
  }
  last_sync_ = std::chrono::steady_clock::now();
  if (!connected_ && rx_id_ >= 0 && tx_id_ >= 0) {
    connected_ = true;
    RCLCPP_INFO(kLogger, "Board connected (state id %d, command id %d)", rx_id_.load(), tx_id_.load());
  }
}

void SerialLink::handle_time()
{
  const rclcpp::Time now = clock_->now();
  builtin_interfaces::msg::Time msg(now);
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<builtin_interfaces::msg::Time>().serialize_message(&msg, &serialized);
  const auto & raw = serialized.get_rcl_serialized_message();
  write_frame(TopicInfo::ID_TIME, Payload(raw.buffer, raw.buffer + raw.buffer_length));
  // A time request is the board's heartbeat.
  last_sync_ = std::chrono::steady_clock::now();
}

void SerialLink::handle_log(const Payload & body)
{
  rosserial_msgs::msg::Log log;
  auto serialized = to_serialized(body);
  rclcpp::Serialization<rosserial_msgs::msg::Log>().deserialize_message(&serialized, &log);
  using L = rosserial_msgs::msg::Log;
  switch (log.level) {
    case L::ROSDEBUG: RCLCPP_DEBUG(kLogger, "[board] %s", log.msg.c_str()); break;
    case L::INFO: RCLCPP_INFO(kLogger, "[board] %s", log.msg.c_str()); break;
    case L::WARN: RCLCPP_WARN(kLogger, "[board] %s", log.msg.c_str()); break;
    default: RCLCPP_ERROR(kLogger, "[board] %s", log.msg.c_str()); break;
  }
}

}  // namespace rosserial_hardware
