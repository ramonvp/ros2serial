#ifndef ROSSERIAL_SERVER_FASTCDR_H_
#define ROSSERIAL_SERVER_FASTCDR_H_

#include <memory>
#include <fastcdr/Cdr.h>

// From plotjuggler Github
/* make sure to remain compatible with previous version of fastCdr */
 //#if ((FASTCDR_VERSION_MAJOR < 2))
 //#define get_buffer_pointer() getBufferPointer()
 //#define get_current_position() getCurrentPosition()
 //#define CdrVersion Cdr
 //#endif

namespace rosserial_server
{

class SimpleCdr /*: public eprosima::fastcdr::Cdr*/
{
public:
    typedef std::shared_ptr<eprosima::fastcdr::FastBuffer> FastBufferPtr;
    typedef std::shared_ptr<eprosima::fastcdr::Cdr> CdrPtr;

/*
    explicit SimpleCdr(eprosima::fastcdr::FastBuffer & fastbuffer)
#if FASTCDR_VERSION_MAJOR == 1
    : cdr_(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR)
#else
    : cdr_(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::DDS_CDR)
#endif
    {
    }*/

    explicit SimpleCdr(uint8_t* buffer, size_t size)
    {
        using namespace eprosima::fastcdr;

        fastbuffer_ = std::make_shared<FastBuffer>((char*)buffer, size);
        cdr_ = std::make_shared<Cdr>(*fastbuffer_.get(), Cdr::DEFAULT_ENDIAN, DDS_CDR);
    }

    explicit SimpleCdr(const SimpleCdr& other) 
    {
        using namespace eprosima::fastcdr;

        //const uint8_t* data = other.const_getData();
        //char* buffer_ptr = reinterpret_cast<char*>(const_cast<uint8_t*>(data));
        char* buffer_ptr = reinterpret_cast<char*>(other.getData());
 
        fastbuffer_ = std::make_shared<FastBuffer>(buffer_ptr, other.getLength());
        cdr_ = std::make_shared<Cdr>(*fastbuffer_.get(), Cdr::DEFAULT_ENDIAN, DDS_CDR);
    }


    /*
    inline uint8_t* const_getData() const { 
        #if FASTCDR_VERSION_MAJOR == 1        
        return reinterpret_cast<const uint8_t*>(cdr_->getBufferPointer());
        #else
        return reinterpret_cast<const uint8_t*>(cdr_->get_buffer_pointer());
        #endif
    }
        */


    inline uint8_t* getData() const { 
        #if FASTCDR_VERSION_MAJOR == 1        
        return reinterpret_cast<uint8_t*>(cdr_->getBufferPointer());
        #else
        return reinterpret_cast<uint8_t*>(cdr_->get_buffer_pointer());
        #endif
    }

    inline uint32_t getLength() const {
#if FASTCDR_VERSION_MAJOR == 1
    return cdr_->getSerializedDataLength();
#else
    return cdr_->get_serialized_data_length();
#endif
    }

    template<class _T>
    inline SimpleCdr& operator <<(const _T& value)
    {
        *cdr_.get() << value;
        return *this;
    }

    template<class _T>
    inline SimpleCdr& operator >>(_T& value)
    {
        *cdr_.get() >> value;
        return *this;
    }
private:

    FastBufferPtr fastbuffer_{nullptr};
    CdrPtr cdr_{nullptr};
};

} // namespace rosserial_server

#endif
