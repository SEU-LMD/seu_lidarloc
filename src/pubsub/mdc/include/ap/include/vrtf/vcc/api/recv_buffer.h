/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Define types in communication mannger
 * Create: 2020-08-10
 */
#ifndef VRTF_VCC_API_INTERNAL_RECVBUFFER_H
#define VRTF_VCC_API_INTERNAL_RECVBUFFER_H
#include "vrtf/vcc/api/types.h"
#include "vrtf/vcc/api/raw_buffer.h"
#include "vrtf/vcc/driver/event_handler.h"
#include <securec.h>
namespace vrtf {
namespace vcc {
namespace api {
namespace types {
class RecvBuffer;
using RecvBufferHandler = std::function<void(RecvBuffer)>;
template<typename T>
struct IsRecvBuffer {
    static const bool value = false;
};

template<>
struct IsRecvBuffer<RecvBuffer> {
    static const bool value = true;
};
class ReturnLoanControl {
public:
    ReturnLoanControl(const std::shared_ptr<vrtf::vcc::driver::EventHandler>& driver, const uint8_t* data);
    ~ReturnLoanControl();
    ReturnLoanControl(const ReturnLoanControl& other) = delete;
    ReturnLoanControl& operator=(const ReturnLoanControl& other) = delete;
private:
    std::shared_ptr<vrtf::vcc::driver::EventHandler> driver_;
    const uint8_t* rawDataPtr;
};

class RecvBuffer {
public:
    RecvBuffer(
        const uint8_t* data, const uint64_t& size, const std::shared_ptr<vrtf::vcc::driver::EventHandler>& driver);
    RecvBuffer(
        const uint8_t* data, const uint64_t& size, const std::shared_ptr<ReturnLoanControl>& returnLoanPtr);
    RecvBuffer() = default;
    RecvBuffer(const RecvBuffer& other) = delete;
    RecvBuffer& operator=(const RecvBuffer& other) = delete;
    RecvBuffer(RecvBuffer && recvBuffer) noexcept;
    RecvBuffer& operator=(RecvBuffer && recvBuffer) noexcept;
    ~RecvBuffer();
    template<typename T>
    typename std::enable_if<IsRawBufferSupport<T>::value && !IsVector<T>::value, RecvBuffer>::type& operator>>(T& data)
    {
        size_t size = sizeof(T);
        if (rawDataSize < pos_ + size) {
            size_t remainSize = rawDataSize - pos_;
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->warn("vrtf::vcc::api::types::RecvBuffer::Recvbuffer is to end", DEFAULT_LOG_LIMIT) <<
                "Recvbuffer is to end, none ostream[remainSize=" << remainSize << "]";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return *this;
        }
        errno_t memcpyResult = 0;
        switch (size) {
            case 1: { // 1 mean one byte dataType
                std::uint8_t* value = reinterpret_cast<std::uint8_t* >(&data);
                memcpyResult = memcpy_s(value, size, rawDataPtr + pos_, size);
                break;
            }

            case 2: { // 2 mean two byte dataType
                std::uint16_t* value = reinterpret_cast<std::uint16_t* >(&data);
                memcpyResult = memcpy_s(value, size, rawDataPtr + pos_, size);
                break;
            }
            case 4: { // 4 mean four byte dataType
                std::uint32_t* value = reinterpret_cast<std::uint32_t* >(&data);
                memcpyResult = memcpy_s(value, size, rawDataPtr + pos_, size);
                break;
            }
            case 8: { // 8 mean eight byte dataType
                std::uint64_t* value = reinterpret_cast<std::uint64_t* >(&data);
                memcpyResult = memcpy_s(value, size, rawDataPtr + pos_, size);
                break;
            }
            default: {
                break;
            }
        }
        if (memcpyResult != 0) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->error("vrtf::vcc::api::types::RecvBuffer::Recvbuffer memcpy failed", DEFAULT_LOG_LIMIT) <<
                "Recvbuffer memcpy failed";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return *this;
        }
        pos_ += size;
        return *this;
    }

    template<typename T>
    typename std::enable_if<IsRawBufferSupport<T>::value && IsVector<T>::value, RecvBuffer>::type& operator>>(T& data)
    {
        size_t size = sizeof(typename T::value_type);
        size_t vecSize = (rawDataSize - pos_) / size;
        if (vecSize == 0) {
            size_t remainSize = (rawDataSize - pos_) % size;
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->warn("vrtf::vcc::api::types::RecvBuffer::Recvbuffer is to end", DEFAULT_LOG_LIMIT) <<
                "Recvbuffer is to end, none ostream[remainSize=" << remainSize << "]";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return *this;
        }
        data.resize(vecSize);
        errno_t memcpyResult = 0;
        switch (size) {
            case 1: { // 1 mean one byte dataType
                std::uint8_t* value = reinterpret_cast<std::uint8_t* >(&data[0]);
                memcpyResult = memcpy_s(value, vecSize * size, rawDataPtr + pos_, vecSize * size);
                break;
            }

            case 2: { // 2 mean two byte dataType
                std::uint16_t* value = reinterpret_cast<std::uint16_t* >(&data[0]);
                memcpyResult = memcpy_s(value, vecSize * size, rawDataPtr + pos_, vecSize * size);
                break;
            }

            case 4: { // 4 mean four byte dataType
                std::uint32_t* value = reinterpret_cast<std::uint32_t* >(&data[0]);
                memcpyResult = memcpy_s(value, vecSize * size, rawDataPtr + pos_, vecSize * size);
                break;
            }

            case 8: { // 8 mean eight byte dataType
                std::uint64_t* value = reinterpret_cast<std::uint64_t* >(&data[0]);
                memcpyResult = memcpy_s(value, vecSize * size, rawDataPtr + pos_, vecSize * size);
                break;
            }
            default: {
                break;
            }
        }
        if (memcpyResult != 0) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->error("vrtf::vcc::api::types::RecvBuffer::Recvbuffer memcpy failed", DEFAULT_LOG_LIMIT) <<
                "Recvbuffer memcpy vector failed";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return *this;
        }
        pos_ += vecSize * size;
        return *this;
    }

    template<typename T>
    typename std::enable_if<!IsRawBufferSupport<T>::value>::type operator>>(T& data)
    {
        (void)data;
        /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
        /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
        logInstance_->error(
            "vrtf::vcc::api::types::RecvBuffer::Recvbuffer not support this data type", DEFAULT_LOG_LIMIT)
            << "Recvbuffer not support this data type";
        /* AXIVION enable style AutosarC++19_03-A5.0.1 */
        /* AXIVION enable style AutosarC++19_03-A5.1.1 */
    }

    const std::uint8_t* Get() const;
    std::uint64_t GetSize() const;
    const std::shared_ptr<ReturnLoanControl> GetReturnLoan() const;
private:
    uint64_t rawDataSize;
    uint64_t pos_;
    const uint8_t* rawDataPtr;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_ = nullptr;
    std::shared_ptr<ReturnLoanControl> returnLoanPtr_ = nullptr;
};
}
}
}
}
#endif
