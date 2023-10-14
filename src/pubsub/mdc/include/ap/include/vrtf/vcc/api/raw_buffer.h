/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: provided RawBuffer dataType
 * Create: 2020-08-10
 */
#ifndef VRTF_VCC_API_INTERNAL_RAWBUFFER_H
#define VRTF_VCC_API_INTERNAL_RAWBUFFER_H
#include "vrtf/vcc/api/types.h"
#include <securec.h>
namespace vrtf {
namespace vcc {
namespace api {
namespace types {
template<typename T>
struct IsRawBufferSupport {
    static const bool value = false;
};

template<>
struct IsRawBufferSupport<uint16_t> {
    static const bool value = true;
};

template<>
struct IsRawBufferSupport<uint8_t> {
    static const bool value = true;
};

template<>
struct IsRawBufferSupport<uint32_t> {
    static const bool value = true;
};

template<>
struct IsRawBufferSupport<uint64_t> {
    static const bool value = true;
};

template<>
struct IsRawBufferSupport<std::vector<uint8_t>> {
    static const bool value = true;
};

template<>
struct IsRawBufferSupport<std::vector<uint16_t>> {
    static const bool value = true;
};

template<>
struct IsRawBufferSupport<std::vector<uint64_t>> {
    static const bool value = true;
};

template<>
struct IsRawBufferSupport<std::vector<uint32_t>> {
    static const bool value = true;
};

template<typename T>
struct IsVector {
    static const bool value = false;
};

template<typename T>
struct IsVector<std::vector<T>> {
    static const bool value = true;
};

class RawBuffer {
public:
    RawBuffer(uint8_t* data, const uint64_t& size);
    ~RawBuffer();
    uint8_t* GetRawBufferPtr();
    std::uint64_t GetRawBufferSize() const;
    template<typename T>
    typename std::enable_if<IsRawBufferSupport<T>::value && !IsVector<T>::value, RawBuffer>::type& operator<<(
        const T& data)
    {
        size_t size = sizeof(T);
        if (rawDataSize < pos_ + size) {
            size_t const remainSize = rawDataSize - pos_;
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->warn("vrtf::vcc::api::types::RawBuffer::Rawbuffer is full", DEFAULT_LOG_LIMIT) <<
                "Rawbuffer is full, cannot add data[remainSize=" << remainSize << "]";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return *this;
        }
        errno_t memcpyResult = 0;
        switch (size) {
            case 1: { // 1 mean one byte dataType
                const std::uint8_t* value = reinterpret_cast<const std::uint8_t* >(&data);
                memcpyResult = memcpy_s(rawDataPtr + pos_, rawDataSize - pos_, value, size);
                break;
            }

            case 2: { // 2 mean two byte dataType
                const std::uint16_t* value = reinterpret_cast<const std::uint16_t* >(&data);
                memcpyResult = memcpy_s(rawDataPtr + pos_, rawDataSize - pos_, value, size);
                break;
            }

            case 4: { // 4 mean four byte dataType
                const std::uint32_t* value = reinterpret_cast<const std::uint32_t* >(&data);
                memcpyResult = memcpy_s(rawDataPtr + pos_, rawDataSize - pos_, value, size);
                break;
            }

            case 8: { // 8 mean eight byte dataType
                const std::uint64_t* value = reinterpret_cast<const std::uint64_t* >(&data);
                memcpyResult = memcpy_s(rawDataPtr + pos_, rawDataSize - pos_, value, size);
                break;
            }
            default: {
                break;
            }
        }
        if (memcpyResult != 0) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->error("vrtf::vcc::api::types::RawBuffer::Rawbuffer memcpy failed", DEFAULT_LOG_LIMIT)
                << "Rawbuffer memcpy failed";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return *this;
        }
        pos_ += size;
        return *this;
    }

    template<typename T>
    typename std::enable_if<IsRawBufferSupport<T>::value && IsVector<T>::value, RawBuffer>::type& operator<<(T& data)
    {
        size_t const vecSize = data.size();
        if (vecSize == 0) {
            return *this;
        }
        size_t const size = sizeof(typename T::value_type);
        size_t const actualNumber = ((rawDataSize - pos_) / (size) > vecSize) ? vecSize : (rawDataSize - pos_) / size;
        if (rawDataSize < pos_ + size * vecSize) {
            size_t const remainSize = rawDataSize - size * actualNumber - pos_;
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->warn("vrtf::vcc::api::types::RawBuffer::Rawbuffer is full", DEFAULT_LOG_LIMIT)
                << "Rawbuffer is full, cannot add vector data[remainSize=" << remainSize << "]";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        }
        if (actualNumber == 0) {
            return *this;
        }
        errno_t memcpyResult = 0;
        switch (size) {
            case 1: { // 1 mean one byte dataType
                const std::uint8_t* value = reinterpret_cast<const std::uint8_t* >(&data[0]);
                memcpyResult = memcpy_s(rawDataPtr + pos_, rawDataSize - pos_, value, actualNumber * size);
                break;
            }

            case 2: { // 2 mean two byte dataType
                const std::uint16_t* value = reinterpret_cast<const std::uint16_t* >(&data[0]);
                memcpyResult = memcpy_s(rawDataPtr + pos_, rawDataSize - pos_, value, actualNumber * size);
                break;
            }
            case 4: { // 4 mean four byte dataType
                const std::uint32_t* value = reinterpret_cast<const std::uint32_t* >(&data[0]);
                memcpyResult = memcpy_s(rawDataPtr + pos_, rawDataSize - pos_, value, actualNumber * size);
                break;
            }
            case 8: { // 8 mean eight byte dataType
                const std::uint64_t* value = reinterpret_cast<const std::uint64_t* >(&data[0]);
                memcpyResult = memcpy_s(rawDataPtr + pos_, rawDataSize - pos_, value, actualNumber * size);
                break;
            }
            default: {
                break;
            }
        }
        if (memcpyResult != 0) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->error("vrtf::vcc::api::types::RawBuffer::Rawbuffer memcpy failed", DEFAULT_LOG_LIMIT)
                << "Rawbuffer memcpy vector failed";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return *this;
        }
        pos_ += actualNumber * size;
        return *this;
    }

    template<typename T>
    typename std::enable_if<!IsRawBufferSupport<T>::value>::type operator<<(T& data)
    {
        (void)data;
        /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
        /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
        logInstance_->error("vrtf::vcc::api::types::RawBuffer::Rawbuffer not support this data type", DEFAULT_LOG_LIMIT)
            << "Rawbuffer not support this data type";
        /* AXIVION enable style AutosarC++19_03-A5.0.1 */
        /* AXIVION enable style AutosarC++19_03-A5.1.1 */
    }
    uint8_t* rawDataPtr;
private:
    uint64_t rawDataSize;
    uint64_t pos_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_ = nullptr;
};
}
}
}
}
#endif
