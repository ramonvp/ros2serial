#ifndef ROSSERIAL_SERVER_SIMPLECDR_H_
#define ROSSERIAL_SERVER_SIMPLECDR_H_


#include <cstdint>
#include <vector>
#include <stdexcept>
#include <rclcpp/serialization.hpp>

namespace rosserial_server
{

class SimpleCdr
{
public:
    explicit SimpleCdr(uint8_t *buffer, uint32_t size) : buffer_(buffer), size_(size)
    {}

    // Serializers
    SimpleCdr& operator <<(const uint8_t& value)
    {
        check_space (sizeof(value));

        buffer_[offset_++] = value;
        return *this;
    }

    SimpleCdr& operator <<(const uint16_t& value)
    {
        check_space (sizeof(value));

        buffer_[offset_++] = (uint8_t)(value & 0xff);
        buffer_[offset_++] = (uint8_t)(value >> 8);
        return *this;
    }

    SimpleCdr& operator <<(const std::vector<uint8_t>& buffer)
    {
        check_space (buffer.size());

        std::memcpy (buffer_+offset_, buffer.data(), buffer.size());
        offset_ += buffer.size();
        return *this;
    }

    // Deserializers
    SimpleCdr& operator >>(uint8_t& value)
    {
        value = buffer_[offset_++];
        return *this;
    }

    SimpleCdr& operator >>(uint16_t& value)
    {
        value = buffer_[offset_++];
        value |= buffer_[offset_++]<<8;
        return *this;
    }

    void print(const char* label)
    {
        fprintf(stderr, "%s: ", label);
        for (size_t i = 0; i < offset_; i++)
        {
            fprintf(stderr, "%02hhX ", buffer_[i]);
        }
        fprintf(stderr, "\n");
    }

    uint8_t* getData() const { return buffer_; }

    uint32_t getLength() const { return size_;}

    uint8_t* getCurrentPosition() const { return buffer_+offset_;}

    void advance(uint32_t num_bytes) {
        check_space(num_bytes);
        offset_ += num_bytes;
    }

    // Set the pointer from the beginning
    void reset() { offset_ = 0; }

    rclcpp::SerializedMessage asSerializedMessage() const {
        rcutils_uint8_array_t rcutil_array;
        rcutil_array.buffer = buffer_;
        rcutil_array.buffer_length = size_;
        rcutil_array.buffer_capacity = size_;
        rcutil_array.allocator = rcl_get_default_allocator();

        return rclcpp::SerializedMessage(rcutil_array);
    }

private:
    void check_space(uint32_t num_bytes) {
        if (size_ - offset_ < num_bytes)
            throw std::out_of_range("Not enough space in buffer");
    }

    uint8_t *buffer_{nullptr};
    uint32_t size_{0};
    uint32_t offset_{0};
};

} // namespace rosserial_server

#endif
