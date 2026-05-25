/**
 *
 *  \file
 *  \brief      Helper object for successive reads from a ReadStream.
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


// Check this for more investigation on how to serialize:
// https://answers.ros.org/question/371866/

#ifndef ROSSERIAL_SERVER_ASYNC_READ_BUFFER_H
#define ROSSERIAL_SERVER_ASYNC_READ_BUFFER_H

#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <rclcpp/rclcpp.hpp>

// ssize_t is POSIX-only type. Use make_signed for portable code.
#include <cstdint> // size_t
#include <type_traits> // std::make_signed
typedef std::make_signed<size_t>::type signed_size_t;

#include <rosserial_server/simplecdr.h>

#if 0
#define ROSCPP_SERIALIZATION_DECL
#define ROS_FORCE_INLINE inline

namespace ros {
namespace serialization {

/*
class ROSCPP_SERIALIZATION_DECL StreamOverrunException : public std::exception
{
public:
  StreamOverrunException(const std::string& what)
  : std::exception(what)
  {}
};
*/  

struct StreamOverrunException : std::exception
{
    using std::exception::exception;
};

namespace stream_types
 {
 enum StreamType
 {
   Input,
   Output,
   Length
 };
 }
 typedef stream_types::StreamType StreamType;
 
 struct ROSCPP_SERIALIZATION_DECL Stream
 {
   /*
    * \brief Returns a pointer to the current position of the stream
    */
   inline uint8_t* getData() { return data_; }
   ROS_FORCE_INLINE uint8_t* advance(uint32_t len)
   {
     uint8_t* old_data = data_;
     data_ += len;
     if (data_ > end_)
     {
       // Throwing directly here causes a significant speed hit due to the extra code generated
       // for the throw statement
       //throwStreamOverrun();
       throw std::overflow_error("Stream Overrun!");
     }
     return old_data;
   }
 
   inline uint32_t getLength() { return (uint32_t)(end_ - data_); }
 
 protected:
   Stream(uint8_t* _data, uint32_t _count)
   : data_(_data)
   , end_(_data + _count)
   {}
 
 private:
   uint8_t* data_;
   uint8_t* end_;
 };
 
 struct ROSCPP_SERIALIZATION_DECL IStream : public Stream
 {
   static const StreamType stream_type = stream_types::Input;
 
   IStream(uint8_t* data, uint32_t count)
   : Stream(data, count)
   {}
 
   template<typename T>
   ROS_FORCE_INLINE void next(T& t)
   {
     //deserialize(*this, t);
   }
 
   template<typename T>
   ROS_FORCE_INLINE IStream& operator>>(T& t)
   {
     //deserialize(*this, t);
     return *this;
   }
 };

/**
 * \brief Output stream
 */
struct ROSCPP_SERIALIZATION_DECL OStream : public Stream
{
  static const StreamType stream_type = stream_types::Output;

  OStream(uint8_t* data, uint32_t count)
  : Stream(data, count)
  {}

  /**
   * \brief Serialize an item to this output stream
   */
  template<typename T>
  ROS_FORCE_INLINE void next(const T& t)
  {
    //serialize(*this, t);
  }

  template<typename T>
  ROS_FORCE_INLINE OStream& operator<<(const T& t)
  {
    //serialize(*this, t);
    return *this;
  }
};


}



}
#endif



namespace rosserial_server
{

template<typename AsyncReadStream>
class AsyncReadBuffer
{
public:
  AsyncReadBuffer(std::shared_ptr<rclcpp::Node> node, AsyncReadStream& s, size_t capacity,
                  boost::function<void(const boost::system::error_code&)> error_callback)
       : node_(node), stream_(s), read_requested_bytes_(0), error_callback_(error_callback) {
    reset();
    mem_.resize(capacity);

    if (!error_callback_) {
      //RCLCPP_FATAL_STREAM_NAMED(get_logger(), "async_read", "[AsyncReadBuffer] Bad error callback passed to read buffer.");
      return;
    }
  }

  /* rclcpp::Logger& get_logger() {
    static auto logger = rclcpp::make_logger("AsyncReadBuffer");
    return logger;
  } */

  /**
   * @brief Commands a fixed number of bytes from the buffer. This may be fulfilled from existing
   *        buffer content, or following a hardware read if required.
   */
  void read(size_t requested_bytes, boost::function<void(SimpleCdr&)> callback) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AsyncReadBuffer::read] Buffer read of " << requested_bytes << " bytes, " <<
                           "wi: " << write_index_ << ", ri: " << read_index_);

    static size_t count {0};
    
    if (read_requested_bytes_ != 0) {
      count++;
      if(count > 3)
      {
        read_requested_bytes_ = 0;
        reset();
      }
      RCLCPP_FATAL_STREAM(node_->get_logger(), "[AsyncReadBuffer::read] Bytes requested is nonzero, is there an operation already pending?");
      error_callback_(boost::system::errc::make_error_code(boost::system::errc::operation_not_permitted));
      return;
    }

    if (!callback) {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "[AsyncReadBuffer::read] Bad read success callback function.");
      error_callback_(boost::system::errc::make_error_code(boost::system::errc::invalid_argument));
      return;
    }
    count = 0;
    read_success_callback_ = callback;
    read_requested_bytes_ = requested_bytes;

    if (read_requested_bytes_ > mem_.size())
    {
      // Insufficient room in the buffer for the requested bytes,
      RCLCPP_ERROR_STREAM(node_->get_logger(), "[AsyncReadBuffer::read] Requested to read " << read_requested_bytes_ <<
                             " bytes, but buffer capacity is only " << mem_.size() << ".");
      error_callback_(boost::system::errc::make_error_code(boost::system::errc::no_buffer_space));
      return;
    }

    // Number of bytes which must be transferred to satisfy the request.
    signed_size_t transfer_bytes = read_requested_bytes_ - bytesAvailable();

    if (transfer_bytes > 0)
    {
      // If we don't have enough headroom in the buffer, we'll have to shift what's currently in there to make room.
      if (bytesHeadroom() < transfer_bytes)
      {
        memmove(&mem_[0], &mem_[read_index_], bytesAvailable());
        write_index_ = bytesAvailable();
        read_index_ = 0;
      }

      // Initiate a read from hardware so that we have enough bytes to fill the user request.
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AsyncReadBuffer::read] Requesting transfer of at least " << transfer_bytes << " byte(s).");
      boost::asio::async_read(stream_,
          boost::asio::buffer(&mem_[write_index_], bytesHeadroom()),
          boost::asio::transfer_at_least(transfer_bytes),
          boost::bind(&AsyncReadBuffer::callback, this,
                      boost::asio::placeholders::error,
                      boost::asio::placeholders::bytes_transferred));
    }
    else
    {
      // We have enough in the buffer already, can fill the request without going to hardware.
      callSuccessCallback();
    }
  }

private:
  void reset()
  {
    read_index_ = 0;
    write_index_ = 0;
  }

  inline size_t bytesAvailable()
  {
    return write_index_ - read_index_;
  }

  inline size_t bytesHeadroom()
  {
    return mem_.size() - write_index_;
  }

  /**
   * @brief The internal callback which is called by the boost::asio::async_read invocation
   *        in the public read method above.
   */
  void callback(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error)
    {
      read_requested_bytes_ = 0;
      read_success_callback_.clear();
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AsyncReadBuffer::callback] Read operation failed with: " << error);

      if (error == boost::asio::error::operation_aborted)
      {
        // Special case for operation_aborted. The abort callback comes when the owning Session
        // is in the middle of teardown, which means the callback is no longer valid.
      }
      else
      {
        error_callback_(error);
      }
      return;
    }

    //printf("[callback] ");
    //for(int i = 0; i < bytes_transferred; i++)
    //  printf("%02X ", mem_[i+write_index_]);
    //printf("\n");

    write_index_ += bytes_transferred;
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AsyncReadBuffer::callback] Successfully read " << bytes_transferred << " byte(s), now " << bytesAvailable() << " available.");

    /*
    fprintf(stderr, "[callback] bytes_transferred A: ");
    for(int i = 0; i < bytes_transferred; i++)
    {
        char c = mem_[i+write_index_];
        fprintf(stderr, "%02X ", c);
    }
    fprintf(stderr, "\n");

    fprintf(stderr, "[callback] bytes_transferred B: ");
    for(int i = 0; i < bytes_transferred; i++)
    {
        char c = mem_[i+write_index_];
        fprintf(stderr, "%c", c >= 0x20 && c < 0x7f ? c : '#');
    }
    fprintf(stderr, "\n");
    */


    callSuccessCallback();
  }

  /**
   * @brief Calls the user's callback. This is a separate function because it gets called from two
   *        places, depending whether or not an actual HW read is required to fill the request.
   */
  void callSuccessCallback()
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AsyncReadBuffer::callSuccessCallback] Invoking success callback with buffer of requested size " <<
                           read_requested_bytes_ << " byte(s).");

    //ros::serialization::IStream stream(&mem_[read_index_], read_requested_bytes_);
    //eprosima::fastcdr::FastBuffer fast_buffer(&mem_[read_index_], read_requested_bytes_);
    //SimpleCdr stream(fast_buffer);
    SimpleCdr stream(&mem_[read_index_], read_requested_bytes_);


    read_index_ += read_requested_bytes_;

    // Post the callback rather than executing it here so, so that we have a chance to do the cleanup
    // below prior to it actually getting run, in the event that the callback queues up another read.
#if BOOST_VERSION >= 107000
    boost::asio::post(stream_.get_executor(), boost::bind(read_success_callback_, stream));
#else
    stream_.get_io_service().post(boost::bind(read_success_callback_, stream));
#endif

    // Resetting these values clears our state so that we know there isn't a callback pending.
    read_requested_bytes_ = 0;
    read_success_callback_.clear();

    if (bytesAvailable() == 0)
    {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AsyncReadBuffer::callSuccessCallback] Buffer is empty, resetting indexes to the beginning.");
      reset();
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  AsyncReadStream& stream_;
  std::vector<uint8_t> mem_;

  size_t write_index_;
  size_t read_index_;
  boost::function<void(const boost::system::error_code&)> error_callback_;

  boost::function<void(SimpleCdr&)> read_success_callback_;
  size_t read_requested_bytes_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_ASYNC_READ_BUFFER_H
