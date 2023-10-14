/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: Someip Serialize help util file
 * Create: 2021-05-05
 */
#ifndef VRTF_SOMEIP_SERIALIZE_HELPER_H
#define VRTF_SOMEIP_SERIALIZE_HELPER_H
#include "vrtf/vcc/serialize/serialize_config.h"
#include <map>
#include <arpa/inet.h>
#include <climits>
namespace vrtf {
namespace serialize {
namespace someip {
const int g_fourBytePos {32};
const int g_threeBytePos {24};
const int g_oneBytePos {8};
const size_t ONE_LENGTH_FIELD {1};
const size_t TWO_LENGTH_FIELD {2};
const size_t FOUR_LENGTH_FIELD {4};
const size_t ONE_BYTES_LENGTH {1};
const size_t TWO_BYTES_LENGTH {2};
const size_t FOUR_BYTES_LENGTH {4};
const size_t EIGHT_BYTES_LENGTH {8};
const std::uint32_t g_firstByteValue32 {0xFF000000U};
const std::uint32_t g_secondByteValue32 {0x00FF0000U};
const std::uint32_t g_thirdByteValue32 {0x0000FF00U};
const std::uint32_t g_fourthByteValue32 {0x000000FFU};
const std::uint16_t g_firstByteValue16 {0xFF00U};
const std::uint16_t g_secondByteValue16 {0x00FFU};
const std::uint64_t MAX_UINT32_T {0xFFFFFFFFU};
const std::uint64_t MAX_UINT8_T {0xFFU};
const std::uint64_t MAX_UINT16_T {0xFFFFU};
inline bool isLittleEndian()
{
    union {
        int a;
        char b;
    } c;
    c.a = 1;
    return (c.b == 1);
}

inline bool CheckStructLengthField(const vrtf::serialize::SerializeConfig &config)
{
    if (!(config.wireType == vrtf::serialize::WireType::DYNAMIC && config.isTopStruct)
        && !config.isIngoreOutLength) {
        return true;
    }
    return false;
}

inline std::uint32_t htonlEx(const std::uint32_t host, const vrtf::serialize::ByteOrder order)
{
    std::uint32_t result {host};
    if (order == vrtf::serialize::ByteOrder::LITTLEENDIAN) {
        if (!isLittleEndian()) {
            result = ((host & g_firstByteValue32) >> g_threeBytePos) | ((host & g_secondByteValue32) >> g_oneBytePos) |
                ((host & g_thirdByteValue32) << g_oneBytePos) | ((host & g_fourthByteValue32) << g_threeBytePos);
        }
    } else {
        result = htonl(host);
    }
    return result;
}

inline std::uint32_t ntohlEx(const std::uint32_t host, const vrtf::serialize::ByteOrder order)
{
    std::uint32_t result {host};
    if (order == vrtf::serialize::ByteOrder::LITTLEENDIAN) {
        if (!isLittleEndian()) {
            result = ((host & g_firstByteValue32) >> g_threeBytePos) | ((host & g_secondByteValue32) >> g_oneBytePos) |
                ((host & g_thirdByteValue32) << g_oneBytePos) | ((host & g_fourthByteValue32) << g_threeBytePos);
        }
    } else {
        result = ntohl(host);
    }
    return result;
}

inline std::uint16_t htonsEx(const std::uint16_t host, const vrtf::serialize::ByteOrder order)
{
    std::uint16_t result {host};
    if (order == vrtf::serialize::ByteOrder::LITTLEENDIAN) {
        if (!isLittleEndian()) {
            result = ((host & g_firstByteValue16) >> g_oneBytePos) | ((host & g_secondByteValue16) << g_oneBytePos);
        }
    } else {
        result = htons(host);
    }
    return result;
}

inline std::uint16_t ntohsEx(const std::uint16_t host, const vrtf::serialize::ByteOrder order)
{
    std::uint16_t result {host};
    if (order == vrtf::serialize::ByteOrder::LITTLEENDIAN) {
        if (!isLittleEndian()) {
            result = ((host & g_firstByteValue16) >> g_oneBytePos) | ((host & g_secondByteValue16) << g_oneBytePos);
        }
    } else {
        result = ntohs(host);
    }
    return result;
}

template<typename T>
T htonl64(const T& host, const vrtf::serialize::ByteOrder byteOrder) noexcept
{
    T result {host};
    if (((byteOrder == vrtf::serialize::ByteOrder::LITTLEENDIAN) && isLittleEndian()) ||
        ((byteOrder == vrtf::serialize::ByteOrder::BIGENDIAN) && !isLittleEndian())) {
        return result;
    } else {
        std::uint32_t low {static_cast<std::uint32_t>(host & MAX_UINT32_T)};
        std::uint32_t high {static_cast<std::uint32_t>((host >> g_fourBytePos) & MAX_UINT32_T)};
        low = htonlEx(low, byteOrder);
        high = htonlEx(high, byteOrder);
        result = low;
        result <<= g_fourBytePos; // 32: half of 64 bits data
        result |= high;
    }
    return result;
}

template<typename T>
T ntohl64(const T& host, const vrtf::serialize::ByteOrder byteOrder) noexcept
{
    T result {host};
    if (((byteOrder == vrtf::serialize::ByteOrder::LITTLEENDIAN) && isLittleEndian()) ||
        ((byteOrder == vrtf::serialize::ByteOrder::BIGENDIAN) && !isLittleEndian())) {
        return result;
    } else {
        std::uint32_t low {static_cast<std::uint32_t>(host & MAX_UINT32_T)};
        std::uint32_t high {static_cast<std::uint32_t>((host >> g_fourBytePos) & MAX_UINT32_T)};
        low = ntohlEx(low, byteOrder);
        high = ntohlEx(high, byteOrder);
        result = low;
        result <<= g_fourBytePos; // 32: half of 64 bits data
        result |= high;
    }
    return result;
}
}
}
}
#endif
