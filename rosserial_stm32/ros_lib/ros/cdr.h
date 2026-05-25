#ifndef CDR_H
#define CDR_H

#include <ucdr/microcdr.h>
//#include <stdint.h>

namespace cdr
{

// Values from Fast-CDR/src/cpp/CdrEncoding.hpp
//! @brief This enumeration represents the kinds of CDR serialization supported by eprosima::fastcdr::CDR.
typedef enum
{
    //! @brief Common CORBA CDR serialization.
    CORBA_CDR = 0,
    //! @brief DDS CDR serialization.
    DDS_CDR = 1,
    //! @brief XCDRv1 encoding defined by standard DDS X-Types 1.3
    XCDRv1 = 2,
    //! @brief XCDRv2 encoding defined by standard DDS X-Types 1.3
    XCDRv2 = 3
} CdrVersion;

//! @brief This enumeration represents the supported XCDR encoding algorithms.
typedef enum : uint8_t
{
    //! @brief Specifies that the content is PLAIN_CDR.
    PLAIN_CDR = 0x0,
    //! @brief Specifies that the content is PL_CDR,
    PL_CDR = 0x2,
    //! @brief Specifies that the content is PLAIN_CDR2.
    PLAIN_CDR2 = 0x6,
    //! @brief Specifies that the content is DELIMIT_CDR2.
    DELIMIT_CDR2 = 0x8,
    //! @brief Specifies that the content is PL_CDR2.
    PL_CDR2 = 0xa
} EncodingAlgorithmFlag;

/*!
    * @brief This enumeration represents endianness types.
    */
typedef enum : uint8_t
{
    //! @brief Big endianness.
    BIG_ENDIANNESS = 0x0,
    //! @brief Little endianness.
    LITTLE_ENDIANNESS = 0x1
} Endianness;

constexpr CdrVersion cdr_version_ = DDS_CDR;
constexpr EncodingAlgorithmFlag encoding_flag_ = PLAIN_CDR;
constexpr Endianness endianness_ = LITTLE_ENDIANNESS;

int deserialize_encapsulation(ucdrBuffer* buffer)
{
    // read encapsulation as defined in CDR spec
    uint8_t dummy = 0, encapsulation = 0;

    ucdr_deserialize_uint8_t(buffer, &dummy);
    if (dummy != 0x00) {
        printf("Invalid encapsulation\n");
        return -1;
    }

    ucdr_deserialize_uint8_t(buffer, &encapsulation);

    //const uint8_t endianness = encapsulation & (uint8_t)0x1;
    //printf("Endianness = 0x%02x (%s)\n", endianness, (endianness == 0x00) ? "BIG_ENDIANNESS" : "LITTLE_ENDIANNESS");

    //const uint8_t encoding_flag = encapsulation & static_cast<uint8_t>(~0x1);
    //printf("Encoding CDR = 0x%02x\n", encoding_flag);

    uint8_t options_[2];
    ucdr_deserialize_array_uint8_t(buffer, options_, 2);
    //uint8_t option_align {static_cast<uint8_t>(options_[1] & 0x3u)};
    //printf("Option align = 0x%02x\n", option_align);

    return 4;
}

int serialize_encapsulation(ucdrBuffer* buffer)
{
    uint8_t dummy = 0;
    uint8_t encapsulation = 0;
    uint8_t options_[2] = {0x00, 0x00};

    int size_encapsulation = 0;
    //state state_before_error(*this);

    // If it is DDS_CDR, the first step is to serialize the dummy byte.
    if (CdrVersion::CORBA_CDR < cdr_version_)
    {
        ucdr_serialize_uint8_t(buffer, dummy);
        size_encapsulation++;
    }

    // Construct encapsulation byte.
    encapsulation = (encoding_flag_ | endianness_);

    // Serialize the encapsulation byte.
    ucdr_serialize_uint8_t(buffer, encapsulation);
    size_encapsulation++;

    if (CdrVersion::CORBA_CDR < cdr_version_)
    {
        ucdr_serialize_array_uint8_t(buffer, options_, sizeof(options_));
        size_encapsulation += 2;
    }

    //reset_alignment();
    //encapsulation_serialized_ = true;
    //return *this;
    return size_encapsulation;
}

}   // namespace cdr

#endif
