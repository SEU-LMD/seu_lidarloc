/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: Someip Serialize util file
 * Create: 2019-11-19
 */
#ifndef VSOMEIP_SERIALIZE_H
#define VSOMEIP_SERIALIZE_H

#include "ara/core/map.h"
#include "ara/core/vector.h"
#include "ara/core/string.h"
#include "ara/core/array.h"
#include "ara/hwcommon/log/log.h"
#include "vrtf/vcc/api/method_error.h"
#ifdef RTFCM_ENABLE
#include "vrtf/vcc/api/raw_data.h"
#endif
#include "vrtf/vcc/api/shape_shifter.h"
#include "ara/core/error_code.h"
#include "vrtf/vcc/serialize/someip_serialize_helper.h"
#include "vrtf/vcc/serialize/tlv_serialize_helper.h"
#include "vrtf/vcc/serialize/ros_serialize.h"
#include <securec.h>
#include <type_traits>
#include <cstdint>
#include <cstring>
#include <memory>
#include <arpa/inet.h>
/*
Note: Serialize max size is 16M (equal to 0x100000), cannot be result in integer overflow with type of size_t
*/
namespace vrtf {
namespace serialize {
namespace someip {
const std::size_t MAX_SOMEIP_SERIALIZE_SIZE {0xFFFFFFFFU};
template <typename T, typename Tag = void>
struct IsStruct {
    static const bool value = false;
};

template <typename T>
struct IsStruct<T, typename T::IsEnumerableTag> {
    static const bool value = true;
};

template <typename T>
constexpr bool IsSerializable()
{
    return IsStruct<T>::value ||
        std::is_trivially_copyable<T>::value ||
        vrtf::serialize::ros::IsRosMsg<T>::value ||
        vrtf::serialize::ros::IsRosBuiltinMsg<T>::value ||
        ::tlv::serialize::is_smart_pointer_helper<T>::value;
}

class StructSerializeHelper;

template <typename T>
class Serializer;

class SerializeSizeCounter {
public:
    SerializeSizeCounter(vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        :config_(config)
    {}

    ~SerializeSizeCounter() = default;

    template <typename T>
    void operator()(const T& value)
    {
        Serializer<T> serializer(value, config_);
        std::size_t size = serializer.GetSize();
        size_ += size;
    }

    std::size_t GetSize() const
    {
        return size_;
    }

private:
    const vrtf::serialize::SerializeConfig config_;
    std::size_t size_ = 0;
};

template <typename T>
class Serializer {
public:
    using value_type = typename std::decay<T>::type;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : value_(value), config_(config), pos_(pos)
    {
        static_assert(IsSerializable<T>(), "Not support serialization for this type!");
    }

    void Serialize(std::uint8_t* c)
    {
        SerializeHelper(c);
    }

    std::size_t GetSize()
    {
        size_ = GetSizeHelper();
        return size_;
    }
    ~Serializer() {};
private:
    const value_type& value_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t pos_;
    std::size_t size_ = MAX_SOMEIP_SERIALIZE_SIZE;
    template <typename U = value_type>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<!IsStruct<U>::value
        && !vrtf::serialize::ros::IsRosMsg<U>::value
        && !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value
        && std::is_trivially_copyable<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        std::size_t size = sizeof(T);
        value_type value_temp = value_;
        switch (size) {
            case TWO_BYTES_LENGTH: { // 2: data bytes length, like short int
                std::uint16_t* value_p16 = reinterpret_cast<std::uint16_t* >(&value_temp);
                *value_p16 = htonsEx(*value_p16, config_.byteOrder);
                break;
            }
            case FOUR_BYTES_LENGTH: { // 4: data bytes length, like int
                std::uint32_t* value_p32 = reinterpret_cast<std::uint32_t* >(&value_temp);
                *value_p32 = htonlEx(*value_p32, config_.byteOrder);
                break;
            }
            case EIGHT_BYTES_LENGTH: { // 8: data bytes length, like long long
                std::uint64_t* value_p64 = reinterpret_cast<std::uint64_t* >(&value_temp);
                *value_p64 = htonl64(*value_p64, config_.byteOrder);
                break;
            }
            default: {
                break;
            }
        }
        const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(&value_temp);
        auto memcpySuccess = memcpy_s(c + pos_, size_, data, sizeof(T));
        if (memcpySuccess != 0) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[Someip Serializer] Memory copy return error, Invalid serialization.";
        }
    }

    template <typename U = value_type>
    void SerializeHelper(std::uint8_t* c, typename std::enable_if<IsStruct<U>::value>::type* = 0)
    {
        std::shared_ptr<StructSerializeHelper> serializingStruct {
            std::make_shared<StructSerializeHelper>(c, config_, pos_)};
        (const_cast<value_type&>(value_)).enumerate(*serializingStruct);
    }

    template <typename U = value_type>
    void SerializeHelper(std::uint8_t* c,
                         typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value
                         && !vrtf::serialize::ros::IsRosTlv<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        vrtf::serialize::ros::OStream<vrtf::serialize::someip::Serializer> stream {c + pos_, config_};
        ::ros::serialization::Serializer<U>::allInOne(stream, value_);
    }

    template <typename U = value_type>
    void SerializeHelper(std::uint8_t* c,
                         typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value
                         && vrtf::serialize::ros::IsRosTlv<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        vrtf::serialize::SerializeConfig config = config_;
        config.isTopStruct = false;
        config.isIngoreOutLength = false;
        if (config_.someipSerializeType == vrtf::serialize::SomeipSerializeType::DISABLETLV) {
            vrtf::serialize::ros::OStream<vrtf::serialize::someip::Serializer> stream {c + pos_, config};
            ::ros::serialization::Serializer<U>::allInOne(stream, value_);
            return;
        }
        if (CheckStructLengthField(config_)) {
            std::size_t size {GetSize() - config_.structLength};
            tlv::serialize::TlvSerializeHelper::CopyLengthField(
                c + pos_, size, config_.structLength, config_.byteOrder);
            pos_ += config_.structLength;
        }
        vrtf::serialize::ros::OStream<vrtf::serialize::someip::Serializer> stream {c + pos_, config};
        ::ros::serialization::Serializer<U>::allInOneTlv(stream, value_);
    }

    template <typename U = value_type>
    void SerializeHelper(std::uint8_t* c,
                         typename std::enable_if<vrtf::serialize::ros::IsRosBuiltinMsg<U>::value>::type* = 0)
    {
        using ostreamType = vrtf::serialize::ros::OStream<vrtf::serialize::someip::Serializer>;
        std::shared_ptr<ostreamType> stream {std::make_shared<ostreamType>(c + pos_, config_)};
        ::ros::serialization::Serializer<U>::write(*stream, value_);
    }

    template <typename U = value_type>
    typename std::enable_if<!IsStruct<U>::value
    && !vrtf::serialize::ros::IsRosMsg<U>::value
    && !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value
    && std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T);
    }

    template <typename U = value_type>
    typename std::enable_if<IsStruct<U>::value, std::size_t>::type GetSizeHelper() const
    {
        SerializeSizeCounter serializesizeCounter {config_};
        (const_cast<value_type&>(value_)).enumerate(serializesizeCounter);
        return serializesizeCounter.GetSize();
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value &&
        !vrtf::serialize::ros::IsRosTlv<U>::value, std::size_t>::type GetSizeHelper() const
    {
        vrtf::serialize::ros::OSizeStream<vrtf::serialize::someip::Serializer> stream {config_};
        ::ros::serialization::Serializer<U>::allInOne(stream, value_);
        return stream.GetSize();
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value &&
        vrtf::serialize::ros::IsRosTlv<U>::value, std::size_t>::type GetSizeHelper() const
    {
        // next nest struct is not topStruct
        vrtf::serialize::SerializeConfig config {config_};
        config.isTopStruct = false;
        config.isIngoreOutLength = false;
        vrtf::serialize::ros::OSizeStream<vrtf::serialize::someip::Serializer> stream {config};
        if (config_.someipSerializeType == vrtf::serialize::SomeipSerializeType::DISABLETLV) {
            ::ros::serialization::Serializer<U>::allInOne(stream, value_);
            return stream.GetSize();
        } else {
            size_t size {0};
            ::ros::serialization::Serializer<U>::allInOneTlv(stream, value_);
            if (CheckStructLengthField(config_)) {
                size = stream.GetSize() + config_.structLength;
            } else {
                size = stream.GetSize();
            }
            if (CheckSize(size, config_.structLength)) {
                return size;
            } else {
                return MAX_SOMEIP_SERIALIZE_SIZE;
            }
        }
        return stream.GetSize();
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type GetSizeHelper() const
    {
        value_type value;
        return ::ros::serialization::Serializer<U>::serializedLength(value);
    }

    bool CheckSize(const size_t bufferSize, const size_t lengthField) const
    {
        bool check {false};
        switch (lengthField) {
            case ONE_LENGTH_FIELD: {
                if (bufferSize <= MAX_UINT8_T) {
                    check = true;
                }
                break;
            }
            case TWO_LENGTH_FIELD: {
                if (bufferSize <= MAX_UINT16_T) {
                    check = true;
                }
                break;
            }
            case FOUR_LENGTH_FIELD: {
                if (bufferSize <= MAX_UINT32_T) {
                    check = true;
                }
                break;
            }
            default: {}
        }
        return check;
    }
};

/**
 * @brief Specialized template of Serializer with ara::core::ErrorCode
 */
template <>
class Serializer<ara::core::ErrorCode> {
public:
    using value_type = ara::core::ErrorCode;

    /**
     * @brief Constructor of Serializer
     */
    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : errorCode_(value), config_(config), pos_(pos)
    {
    }
    ~Serializer() = default;
    /**
     * @brief Serialize the value by someip union
     * @param c Store the serialized code stream address
     */
    void Serialize(std::uint8_t* c)
    {
        SerializeHelper(c, pos_);
    }

    /**
     * @brief Get the size of serialize value
     * @return Size of value
     */
    std::size_t GetSize()
    {
        size_ = sizeof(std::uint32_t) + sizeof(std::uint8_t) + sizeof(uint64_t) + sizeof(int32_t);
        return size_;
    }
private:
    const ara::core::ErrorCode& errorCode_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t pos_;
    std::size_t size_ {MAX_SOMEIP_SERIALIZE_SIZE};
    /**
     * @brief Serialize the error code
     * @param c Store the serialized code stream address
     * @param pos Data storage offset address
     */
    void SerializeHelper(std::uint8_t* c, size_t pos)
    {
        using namespace ara::godel::common::log;
        /* Serialize type and length field */
        std::uint8_t type {0x01};

        std::uint32_t len = sizeof(uint64_t) + sizeof(int32_t);
        std::uint32_t lenTmp = len;
        std::uint32_t* lenTmpPtr = reinterpret_cast<std::uint32_t* >(&lenTmp);
        *lenTmpPtr = htonlEx(*lenTmpPtr, config_.byteOrder);
        const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&lenTmp);
        const std::uint8_t* typePtr = reinterpret_cast<std::uint8_t* >(&type);
        bool memcpyResult {true};
        memcpyResult = (!memcpy_s(c, size_, dataLen, sizeof(std::uint32_t))) &&
                        (!memcpy_s(c + sizeof(std::uint32_t), size_ - sizeof(std::uint32_t),
                            typePtr, sizeof(std::uint8_t)));
        if (memcpyResult == false) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[Someip Serializer] Memory copy return error, Invalid serialization.";
        }

        /* Serialize ErrorCode field */
        uint64_t valueTempDomain = errorCode_.Domain().Id();
        std::uint64_t* valueP64 = reinterpret_cast<std::uint64_t* >(&valueTempDomain);
        *valueP64 = htonl64(*valueP64, config_.byteOrder);

        int32_t valueTempCode {errorCode_.Value()};
        std::int32_t* valueP32 = reinterpret_cast<std::int32_t* >(&valueTempCode);
        *valueP32 = static_cast<std::int32_t>(htonlEx(static_cast<std::uint32_t>(*valueP32), config_.byteOrder));
        const std::uint8_t* dataDomain {reinterpret_cast<const std::uint8_t* >(&valueTempDomain)};
        const std::uint8_t* dataCode {reinterpret_cast<const std::uint8_t* >(&valueTempCode)};
        /* sizeof(std::uint32_t) is size of length field, sizeof(std::uint8_t) is size of type field */
        pos = sizeof(std::uint32_t) + sizeof(std::uint8_t);
        memcpyResult = (!memcpy_s(c + pos, size_ - pos, dataDomain, sizeof(uint64_t))) &&
                        (!memcpy_s(c + pos + sizeof(uint64_t), size_ - pos - sizeof(uint64_t),
                            dataCode, sizeof(int32_t)));
        if (memcpyResult == false) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[Someip Serializer] Memory copy return error, Invalid serialization.";
        }
    }
};

template <>
class Serializer<ara::core::String> {
public:
    using value_type = ara::core::String;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : str_(value), config_(config), pos_(pos)
    {
    }

    void Serialize(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::size_t offSet = 0;
        std::uint32_t len = static_cast<std::uint32_t>(str_.size());
        // UTF-8 have 3 byte and "\0" have 1 byte, altogether 4 byte
        std::uint32_t lenTmp = len + FOUR_LENGTH_FIELD;
        std::uint32_t* lenTmpPtr = reinterpret_cast<std::uint32_t* >(&lenTmp);
        *lenTmpPtr = htonlEx(*lenTmpPtr, config_.byteOrder);
        const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&lenTmp);
        auto memcpySuccess = memcpy_s(c + pos_, size_, dataLen, sizeof(std::uint32_t));
        offSet += sizeof(std::uint32_t);
        auto memcpyUtf8Success = memcpy_s(c + pos_ + offSet, size_ - offSet, utf8_, sizeof(utf8_));
        if (memcpySuccess != 0 || memcpyUtf8Success != 0) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[Someip Serializer String] Memory copy return error, Invalid serialization.";
        }
        offSet += sizeof(utf8_);
        const char* data {reinterpret_cast<const char* >(str_.c_str())};
        memcpySuccess = memcpy_s(c + pos_ + offSet, size_ - offSet, data, len);
        offSet += len;
        auto memcpyTerminateSuccess = memcpy_s(c + pos_ + offSet, size_ - offSet,
            &terminate_, sizeof(terminate_));
        if ((memcpySuccess != 0 && len != 0) || memcpyTerminateSuccess != 0) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[Someip Serializer String] Memory copy return error, Invalid serialization.";
        }
    }

    ~Serializer() = default;
    std::size_t GetSize()
    {
        // UTF-8 have 3 byte and "\0" have 1 byte, altogether 4 byte
        size_ = sizeof(std::uint32_t) + str_.size() + FOUR_LENGTH_FIELD;
        return size_;
    }
private:
    const ara::core::String& str_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
    std::size_t size_ {MAX_SOMEIP_SERIALIZE_SIZE};
    // utf8_[3] is use to distinguish utf-8 encoding format.
    const uint8_t utf8_[3] {0xEF, 0xBB, 0xBF};
    const uint8_t terminate_ {0x00};
};
// std::string use for ros dataType serialize
template <>
class Serializer<std::string> {
public:
    using value_type = std::string;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : str_(value), config_(config), pos_(pos)
    {
    }

    void Serialize(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::size_t offSet {0};
        std::uint32_t len {static_cast<std::uint32_t>(str_.size())};
        // UTF-8 have 3 byte and "\0" have 1 byte, altogether 4 byte
        std::uint32_t lenTmp {static_cast<std::uint32_t>(GetSize() - config_.stringLength)};
        tlv::serialize::TlvSerializeHelper::CopyLengthField(c + pos_, lenTmp, config_.stringLength, config_.byteOrder);
        offSet += config_.stringLength;
        if (config_.implementsLegencyStringSerialization == false) {
            auto memcpyUtf8Success = memcpy_s(c + pos_ + offSet, size_ - offSet, utf8_, sizeof(utf8_));
            if (memcpyUtf8Success != 0) {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error()
                    << "[Someip Serializer std::string] Memory copy return error, Invalid serialization.";
            }
            offSet += sizeof(utf8_);
        }
        const char* data {reinterpret_cast<const char* >(str_.c_str())};
        auto memcpySuccess = memcpy_s(c + pos_ + offSet, size_ - offSet, data, len);
        offSet += len;
        if (config_.implementsLegencyStringSerialization == false) {
            auto memcpyTerminateSuccess = memcpy_s(c + pos_ + offSet, size_ - offSet,
                &terminate_, sizeof(terminate_));
            if ((memcpySuccess != 0 && len != 0) || memcpyTerminateSuccess != 0) {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error()
                    << "[Someip Serializer std::string] Memory copy return error, Invalid serialization.";
            }
        }
    }

    ~Serializer() = default;
    std::size_t GetSize()
    {
        // UTF-8 have 3 byte and "\0" have 1 byte, altogether 4 byte
        size_t size {str_.size() + config_.stringLength};
        if (config_.implementsLegencyStringSerialization == false) {
            size += FOUR_LENGTH_FIELD; // // UTF-8 have 3 byte and "\0" have 1 byte, altogether 4 byte
        }
        size_ = size;
        return size_;
    }
private:
    const std::string& str_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
    std::size_t size_ {MAX_SOMEIP_SERIALIZE_SIZE};
    // utf8_[3] is use to distinguish utf-8 encoding format.
    const uint8_t utf8_[3] {0xEF, 0xBB, 0xBF};
    const uint8_t terminate_ {0x00};
};

template <typename T, std::size_t N>
class Serializer<ara::core::Array<T, N>> {
public:
    using value_type = ara::core::Array<T, N>;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t  pos = 0)
        : value_(value), config_(config), pos_(pos)
    {
    }

    void Serialize(std::uint8_t* c)
    {
        SerializeHelper(c);
    }

    std::size_t GetSize()
    {
        size_ = GetSizeHelper();
        return size_;
    }

private:
    const value_type& value_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
    std::size_t size_ {MAX_SOMEIP_SERIALIZE_SIZE};
    void TraverseSerializeArray(std::uint8_t* c)
    {
        std::size_t posCnt = {pos_};
        for (std::size_t i {0}; i < N; ++i) {
            Serializer<T> serializer {value_[i], config_, posCnt};
            posCnt += serializer.GetSize();
            serializer.Serialize(c);
        }
    }

    template <typename U = T>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<!IsStruct<U>::value && std::is_trivially_copyable<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        bool isCopyable {(sizeof(U) == 1 || !isLittleEndian())};
        tlv::serialize::TlvSerializeHelper::CopyLengthField(c + pos_, size_ - config_.arrayLength, config_.arrayLength,
            config_.byteOrder);
        if (isCopyable) {
            const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(value_.data());
            std::size_t cpySize {sizeof(T) * N};
            auto memcpySuccess = memcpy_s(c + pos_ + config_.arrayLength, size_ - config_.arrayLength, data, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[Someip Serializer Array] Memory copy return error, Invalid serialization.";
            }
            return;
        }
        TraverseSerializeArray(c + config_.arrayLength);
    }

    template <typename U = T>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<IsStruct<U>::value || !std::is_trivially_copyable<U>::value>::type* = 0)
    {
        tlv::serialize::TlvSerializeHelper::CopyLengthField(c + pos_, size_ - config_.arrayLength, config_.arrayLength,
            config_.byteOrder);
        TraverseSerializeArray(c + config_.arrayLength);
    }

    template<typename U = T>
    typename std::enable_if<!IsStruct<U>::value && std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T) * N + config_.arrayLength;
    }

    template<typename U = T>
    typename std::enable_if<IsStruct<U>::value || !std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        std::size_t totalSize = config_.arrayLength;
        for (const T& item : value_) {
            Serializer<T> serializer(item, config_);
            totalSize += serializer.GetSize();
        }
        return totalSize;
    }
};

template <typename First, typename Second>
class Serializer<std::pair<First, Second>> {
public:
    using value_type = std::pair<First, Second>;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : value_(value), config_(config), pos_(pos)
    {
    }
    ~Serializer() {}
    void Serialize(std::uint8_t* c)
    {
        Serializer<typename std::decay<First>::type> firstSerializer(value_.first, config_, pos_);
        Serializer<typename std::decay<Second>::type> secondSerializer(
            value_.second, config_, pos_ + firstSerializer.GetSize());
        firstSerializer.Serialize(c);
        (void)secondSerializer.GetSize();
        secondSerializer.Serialize(c);
    }

    std::size_t GetSize() const
    {
        Serializer<typename std::decay<First>::type> firstSerializer(value_.first, config_);
        Serializer<typename std::decay<Second>::type> secondSerializer(value_.second, config_);
        return firstSerializer.GetSize() + secondSerializer.GetSize();
    }
private:
    const value_type& value_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
};


template <typename Container>
class ContainerSerializeHelper {
public:
    using value_type = Container;

    ContainerSerializeHelper(const Container& container,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
        std::size_t pos = 0)
        : value_(container), config_(config), pos_(pos)
    {
    }

    virtual ~ContainerSerializeHelper() {}
    void Serialize(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::uint32_t len = GetSize();
        std::uint32_t lengthOfLengthField = config_.vectorLength;
        len -= lengthOfLengthField;
        tlv::serialize::TlvSerializeHelper::CopyLengthField(c + pos_, len, config_.vectorLength, config_.byteOrder);
        std::size_t posCnt = config_.vectorLength;
        for (const typename Container::value_type& item : value_) {
            Serializer<typename Container::value_type> serializer(item, config_, pos_ + posCnt);
            posCnt += serializer.GetSize();
            serializer.Serialize(c);
        }
    }

    std::size_t GetSize()
    {
        size_ = config_.vectorLength;
        for (const typename Container::value_type& item : value_) {
            Serializer<typename Container::value_type> serializer(item, config_);
            size_ += serializer.GetSize();
        }
        return size_;
    }

    void SetSize(const std::size_t& size)
    {
        size_ = size;
    }

private:
    const Container& value_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t pos_;
    std::size_t size_ = MAX_SOMEIP_SERIALIZE_SIZE;
};

template <typename Key, typename Value>
class Serializer<ara::core::Map<Key, Value>>
    : public ContainerSerializeHelper<ara::core::Map<Key, Value>> {
public:
    using ContainerSerializeHelper<ara::core::Map<Key, Value>>::ContainerSerializeHelper;
    ~Serializer() = default;
};

#ifdef RTFCM_ENABLE
template <>
class Serializer<vrtf::core::RawBuffer> {
public:
    using value_type = vrtf::core::RawBuffer;
    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : container_(value), config_(config)
    {
        (void)pos;
    }

    void Serialize(std::uint8_t* c)
    {
        SerializeHelper(c);
    }

    std::size_t GetSize()
    {
        size_ = GetSizeHelper();
        return size_;
    }

private:
    const value_type& container_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t size_ = MAX_SOMEIP_SERIALIZE_SIZE;
    void SerializeHelper(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::size_t cpySize = sizeof(std::uint8_t) * container_.size();
        const std::uint8_t *data = reinterpret_cast<const std::uint8_t* >(container_.data());
        auto memcpySuccess = memcpy_s(c, size_, data, cpySize);
        if (memcpySuccess != 0 && cpySize != 0) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[SOMEIP Serializer Vector] Memory copy return error, Invalid serialization.";
        }
    }

    std::size_t GetSizeHelper() const
    {
        return sizeof(std::uint8_t) * container_.size();
    }
};
#endif

template <>
class Serializer<vrtf::vcc::api::types::ShapeShifter> {
public:
    using value_type = vrtf::vcc::api::types::ShapeShifter;
    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : value_(value), config_(config)
    {
        (void)pos;
    }

    void Serialize(std::uint8_t* c)
    {
        SerializeHelper(c);
    }

    std::size_t GetSize()
    {
        size_ = GetSizeHelper();
        return size_;
    }

private:
    const value_type& value_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t size_ = MAX_SOMEIP_SERIALIZE_SIZE;
    void SerializeHelper(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        vrtf::vcc::api::types::Stream stream(c, value_.Size());
        value_.Write(stream);
    }

    std::size_t GetSizeHelper() const
    {
        return value_.Size();
    }
};

template <>
class Serializer<ara::core::Vector<bool>> {
public:
    using value_type = ara::core::Vector<bool>;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : container_(value), config_(config), pos_(pos)
    {
    }

    void Serialize(std::uint8_t* c)
    {
        SerializeHelper(c);
    }

    std::size_t GetSize()
    {
        size_ = GetSizeHelper();
        return size_;
    }

private:
    const value_type& container_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t pos_;
    std::size_t size_ = MAX_SOMEIP_SERIALIZE_SIZE;
    void SerializeHelper(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::uint32_t len = static_cast<uint32_t>(GetSizeHelper() - sizeof(uint32_t));
        std::size_t cpySize = sizeof(std::uint32_t);
        std::uint32_t lenTmp = len;
        std::uint32_t* lenTmpPtr = reinterpret_cast<std::uint32_t* >(&lenTmp);
        *lenTmpPtr = htonlEx(*lenTmpPtr, config_.byteOrder);

        const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&lenTmp);
        auto memcpySuccess = memcpy_s(c + pos_, size_, dataLen, cpySize);
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance {Log::GetLog("CM")};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[Someip Serializer Vector<bool>] Memory copy return error, Invalid serialization.";
        }

        cpySize = sizeof(bool);
        for (std::size_t i {0}; i < len; i++) {
            bool valueTmp = container_[i];
            const std::uint8_t* dataPtrTmp = reinterpret_cast<const std::uint8_t* >(&valueTmp);
            std::size_t size {sizeof(bool)};
            switch (size) {
                case TWO_BYTES_LENGTH: { // 2: data bytes length
                    std::uint16_t* value_p16 = reinterpret_cast<std::uint16_t* >(&valueTmp);
                    *value_p16 = htonsEx(*value_p16, config_.byteOrder);
                    break;
                }
                case FOUR_BYTES_LENGTH: { // 4: data bytes length
                    std::uint32_t* value_p32 = reinterpret_cast<std::uint32_t* >(&valueTmp);
                    *value_p32 = htonlEx(*value_p32, config_.byteOrder);
                    break;
                }
                case EIGHT_BYTES_LENGTH: { // 8: data bytes length
                    std::uint64_t* value_p64 = reinterpret_cast<std::uint64_t* >(&valueTmp);
                    *value_p64 = htonl64(*value_p64, config_.byteOrder);
                    break;
                }
                default: {
                    break;
                }
            }

            memcpySuccess = memcpy_s(c + pos_ + sizeof(std::uint32_t), size_ - sizeof(std::uint32_t),
                dataPtrTmp, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::shared_ptr<Log> logInstance {Log::GetLog("CM")};
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance->error() <<
                    "[Someip Serializer Vector<bool>] Memory copy return error, Invalid serialization.";
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
                break;
            }
            pos_ += sizeof(bool);
        }
    }

    std::size_t GetSizeHelper() const
    {
        return sizeof(bool) * container_.size() + sizeof(std::uint32_t);
    }
};

template <typename T>
class Serializer<ara::core::Vector<T>> {
public:
    using value_type = ara::core::Vector<T>;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : value_(value),  config_(config), pos_(pos)
    {
    }
    ~Serializer() = default;
    void Serialize(std::uint8_t* c)
    {
        SerializeHelper(c);
    }

    std::size_t GetSize()
    {
        size_ = GetSizeHelper();
        return size_;
    }

private:
    const value_type& value_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
    std::size_t size_ = MAX_SOMEIP_SERIALIZE_SIZE;
    template <typename U = T>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<!IsStruct<U>::value && std::is_trivially_copyable<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        bool isCopyable = (sizeof(U) == 1 || !isLittleEndian());
        if (isCopyable) {
            const std::size_t& lengthField = config_.vectorLength;
            std::uint32_t len = GetSize() - lengthField;
            tlv::serialize::TlvSerializeHelper::CopyLengthField(c + pos_, len, lengthField, config_.byteOrder);
            size_t cpySize = len;
            const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(value_.data());
            auto memcpySuccess = memcpy_s(c + pos_ + lengthField, size_ - lengthField, data, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[Someip Serializer Vector] Memory copy return error, Invalid serialization.";
            }
            return;
        }
        ContainerSerializeHelper<ara::core::Vector<T>> serializer(value_, config_, pos_);
        serializer.SetSize(size_);
        serializer.Serialize(c);
    }

    template <typename U = T>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<IsStruct<U>::value || !std::is_trivially_copyable<U>::value>::type* = 0)
    {
        ContainerSerializeHelper<ara::core::Vector<T>> serializer(value_, config_, pos_);
        serializer.SetSize(size_);
        serializer.Serialize(c);
    }

    template<typename U = T>
    typename std::enable_if<!IsStruct<U>::value &&
                            std::is_trivially_copyable<U>::value &&
                            !vrtf::serialize::ros::IsRosMsg<U>::value &&
                            !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        size_t size = sizeof(T) * value_.size() + config_.vectorLength;
        return size;
    }

    template<typename U = T>
    typename std::enable_if<IsStruct<U>::value ||
                            !std::is_trivially_copyable<U>::value ||
                            vrtf::serialize::ros::IsRosMsg<U>::value ||
                            vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        std::size_t totalSize = 0;
        for (const T& item : value_) {
            Serializer<T> serializer(item, config_);
            totalSize += serializer.GetSize();
        }
        totalSize += config_.vectorLength;
        return totalSize;
    }
};

template <typename T>
class Serializer<std::shared_ptr<T>> {
public:
    using value_type = std::shared_ptr<T>;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig(),
               std::size_t pos = 0)
        : value_(value),  config_(config), pos_(pos)
    {
        if (value != nullptr) {
            serialize_ = std::make_unique<Serializer<T>>(*value, config, pos);
        }
    }
    ~Serializer() = default;
    void Serialize(std::uint8_t* c)
    {
        if (config_.someipSerializeType != vrtf::serialize::SomeipSerializeType::ENABLETLV) {
            return;
        }
        SerializeHelper(c);
    }

    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        if (config_.someipSerializeType != vrtf::serialize::SomeipSerializeType::ENABLETLV) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Include optional but not enable tlv config, serialize failed";
            return MAX_SOMEIP_SERIALIZE_SIZE;
        }
        return GetSizeHelper();
    }

private:
    const value_type& value_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
    std::size_t size_ = MAX_SOMEIP_SERIALIZE_SIZE;
    std::unique_ptr<Serializer<T>> serialize_;
    template <typename U = T>
    void SerializeHelper(std::uint8_t* c)
    {
        if (serialize_ != nullptr) {
            return serialize_->Serialize(c);
        }
    }

    size_t GetSizeHelper() const
    {
        if (serialize_ != nullptr) {
            return serialize_->GetSize();
        }
        return 0;
    }
};

class StructSerializeHelper {
public:
    StructSerializeHelper(std::uint8_t* payload,
                          vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
                          std::size_t pos = 0)
        : payload_(payload), config_(config), pos_(pos)
    {
    }

    ~StructSerializeHelper() = default;
    template <typename T>
    void operator()(const T& value)
    {
        Serializer<T> serializer(value, config_, pos_);
        pos_ += serializer.GetSize();
        serializer.Serialize(payload_);
    }

private:
    std::uint8_t* payload_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t pos_;
};

template <typename T>
class Deserializer;

template <typename Container>
class ContainerDeserializeHelper {
public:
    using result_type = Container;

    ContainerDeserializeHelper(const std::uint8_t* data, std::size_t size, std::size_t len,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), len_(len), config_(config)
    {
    }
    ~ContainerDeserializeHelper() = default;

    result_type GetValue() const
    {
        using namespace ara::godel::common::log;
        result_type result;
        const std::uint8_t* currentPos = data_ + config_.vectorLength;
        std::size_t remainingSize = size_ - config_.vectorLength;
        std::size_t sizeTmp = len_;
        while (sizeTmp > 0) {
            Deserializer<typename Container::value_type> deserializer(currentPos, remainingSize, config_);
            std::size_t size = deserializer.GetSize();
            if (size == 0) {
                break;
            }
            if (remainingSize >= size) {
                if (sizeTmp < size) {
                    std::string const ctxId {"CM"};
                    std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance->error() << "GetValue failed for Vector deserialization, insufficient data!";
                    return result;
                } else {
                    sizeTmp -= size;
                    currentPos += size;
                    remainingSize -= size;
                    result.push_back(deserializer.GetValue());
                }
            } else {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "GetValue failed for Vector deserialization, insufficient data!";
            }
        }
        return result;
    }

    std::size_t GetSize() const
    {
        using namespace ara::godel::common::log;
        std::size_t result = config_.vectorLength;
        const std::uint8_t* currentPos = data_ + config_.vectorLength;
        std::size_t remainingSize = size_ - config_.vectorLength;
        std::size_t sizeTmp = len_;
        while (sizeTmp > 0) {
            Deserializer<typename Container::value_type> deserializer(currentPos, remainingSize, config_);
            std::size_t size = deserializer.GetSize();
            if (remainingSize >= size) {
                if (sizeTmp < size) {
                    std::string const ctxId {"CM"};
                    std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance->error() << "GetSize failed for Vector deserialization, insufficient data!";
                    return MAX_SOMEIP_SERIALIZE_SIZE;
                } else {
                    currentPos += size;
                    remainingSize -= size;
                    result += size;
                    sizeTmp -= size;
                }
            } else {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "GetSize failed for Vector deserialization, insufficient data!";
                return MAX_SOMEIP_SERIALIZE_SIZE;
            }
        }
        return result;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t len_;
    const vrtf::serialize::SerializeConfig config_;
};

#ifdef RTFCM_ENABLE
class DeserializingEnumerator;
template <>
class Deserializer<vrtf::core::RawBuffer> {
public:
    using result_type = vrtf::core::RawBuffer;
    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
    }
    ~Deserializer() = default;
    result_type GetValue() const
    {
        return GetValueHelper();
    }

    std::size_t GetSize() const
    {
        return GetSizeHelper();
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
    result_type GetValueHelper() const
    {
        result_type result;
        result.reserve(size_);
        result.insert(result.begin(), data_, data_ + size_);
        return result;
    }
    std::size_t GetSizeHelper() const
    {
        return size_;
    }
};
#endif

template <>
class Deserializer<vrtf::vcc::api::types::ShapeShifter> {
public:
    using result_type = vrtf::vcc::api::types::ShapeShifter;
    Deserializer(const std::uint8_t* data,
                 std::size_t size,
                 vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
    }
    ~Deserializer() = default;
    result_type GetValue() const
    {
        return GetValueHelper();
    }

    std::size_t GetSize() const
    {
        return GetSizeHelper();
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
    result_type GetValueHelper() const
    {
        using namespace ara::godel::common::log;
        result_type result;
        vrtf::vcc::api::types::Stream stream(const_cast<uint8_t *>(data_), size_);
        result.Read(stream);
        return result;
    }
    std::size_t GetSizeHelper() const
    {
        return size_;
    }
};

class StructDeserializeHelper;

/**
 * @brief Template specialization for ara::core::Vector<bool>.
 */
template<>
class Deserializer<ara::core::Vector<bool>> {
public:
    using result_type = ara::core::Vector<bool>;

    /**
     * @brief Creates a deserializer using the given payload.
     * @details The constructor will initialize the len_ with MAX_SOMEIP_SERIALIZE_SIZE, if the given size is too
     *          short to hold the container, or the len_ will be intialized with the container's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), len_(), config_(config)
    {
        if (size_ >= sizeof(std::uint32_t)) {
            std::size_t lenTmp = *reinterpret_cast<const std::uint32_t* >(data_);
            std::uint32_t* lenTmpPtr = reinterpret_cast<std::uint32_t* >(&lenTmp);
            *lenTmpPtr = ntohlEx(*lenTmpPtr, config_.byteOrder);
            len_ = lenTmp;
        } else {
            len_ = MAX_SOMEIP_SERIALIZE_SIZE;
        }
    }
    ~Deserializer() = default;
    result_type GetValue() const
    {
        return GetValueHelper();
    }

    /**
     * @brief Returns the number of bytes the container and its contents occupy.
     * @details This method may return MAX_SOMEIP_SERIALIZE_SIZE if the length of the container does not
     *          fit the array size.
     * @return Number of bytes the container occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        // Check if len_ is initialized successfully
        if (size_ < MAX_SOMEIP_SERIALIZE_SIZE) {
            std::size_t sizeTmp = GetSizeHelper();
            if (sizeTmp <= size_) {
                return sizeTmp;
            }
        }
        std::string const ctxId {"CM"};
        std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->error() << "Deserialization of Vector<bool> failed, insufficient data.";
        return MAX_SOMEIP_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t len_;
    const vrtf::serialize::SerializeConfig config_;
    result_type GetValueHelper() const
    {
        using namespace ara::godel::common::log;
        result_type result;
        std::size_t cpySize = sizeof(bool);
        std::size_t lenTmp = len_;
        while (lenTmp > 0) {
            bool valueTmp;
            if (lenTmp < sizeof(bool)) {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error()<< "[SomeIP Deserializer Vector] Memory copy error, Invalid deserialization.";
                break;
            }
            auto memcpySuccess = memcpy_s(reinterpret_cast<std::uint8_t* >(&valueTmp),
                cpySize, data_ + sizeof(std::uint32_t) + len_ - lenTmp, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[SomeIP Deserializer Vector] Memory copy error, Invalid deserialization.";
                break;
            }
            bool valueTmpChange = valueTmp;

            switch (sizeof(bool)) {
                case TWO_BYTES_LENGTH: { // 2: data bytes length
                    std::uint16_t *value_p16 = reinterpret_cast<std::uint16_t* >(&valueTmp);
                    *value_p16 = htonsEx(*value_p16, config_.byteOrder);
                    valueTmpChange = static_cast<bool>(*value_p16);
                    break;
                }
                case FOUR_BYTES_LENGTH: { // 4: data bytes length
                    std::uint32_t* value_p32 = reinterpret_cast<std::uint32_t* >(&valueTmp);
                    *value_p32 = htonlEx(*value_p32, config_.byteOrder);
                    valueTmpChange = static_cast<bool>(*value_p32);
                    break;
                }
                case EIGHT_BYTES_LENGTH: { // 8: data bytes length
                    std::uint64_t* value_p64 = reinterpret_cast<std::uint64_t* >(&valueTmp);
                    *value_p64 = htonl64(*value_p64, config_.byteOrder);
                    valueTmpChange = static_cast<bool>(*value_p64);
                    break;
                }
                default: {}
            }
            lenTmp -= sizeof(bool);
            result.push_back(valueTmpChange);
        }
        return result;
    }
    std::size_t GetSizeHelper() const
    {
        return len_ + sizeof(std::uint32_t);
    }
};

/**
 * @brief Template specialization for ara::core::Vector<T>.
 */
template <typename T>
class Deserializer<ara::core::Vector<T>> {
public:
    using result_type = ara::core::Vector<T>;
    /**
     * @brief Creates a deserializer using the given payload. if the given size is too short to hold the container,
     *        or the len_ will be intialized with the container's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data,
                 std::size_t size,
                 vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), len_(), config_(config)
    {
        if (size_ >= config_.vectorLength) {
            len_ = tlv::serialize::TlvSerializeHelper::GetDeserializeLength(config, data);
        } else {
            len_ = MAX_SOMEIP_SERIALIZE_SIZE;
        }
    }
    ~Deserializer() = default;

    result_type GetValue() const
    {
        return GetValueHelper();
    }

    /**
     * @brief Returns the number of bytes the container and its contents occupy.
     * @details This method may return MAX_SOMEIP_SERIALIZE_SIZE if the length of the container does not fit the array
     *          size.
     * @return Number of bytes the container occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        // Check if len_ is initialized successfully
        if (size_ < MAX_SOMEIP_SERIALIZE_SIZE) {
            std::size_t sizeTmp = GetSizeHelper();
            if (sizeTmp <= size_) {
                return sizeTmp;
            }
        }
        std::string const ctxId {"CM"};
        std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->error() << "Construct deserialization for Vector failed, invalid size parameter!";
        return  MAX_SOMEIP_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t len_;
    const vrtf::serialize::SerializeConfig config_;
    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<!IsStruct<U>::value &&
        std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        using namespace ara::godel::common::log;
        bool isCopyable = (sizeof(U) == 1 || !isLittleEndian());
        if (isCopyable) {
            result_type result;
            std::size_t cpySize = len_;
            result.resize(len_ / sizeof(T));
            if (cpySize >= 0 && cpySize < MAX_SOMEIP_SERIALIZE_SIZE) {
                auto memcpySuccess = memcpy_s(result.data(), cpySize, data_ + config_.vectorLength, cpySize);
                if (memcpySuccess != 0 && cpySize != 0) {
                    std::string const ctxId {"CM"};
                    std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                    /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                    /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                    logInstance->error() <<
                        "[Someip Deserializer Vector] Memory copy return error, Invalid deserialization.";
                    /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                    /* AXIVION enable style AutosarC++19_03-A5.1.1 */
                }
                return result;
            }
        }
        return ContainerDeserializeHelper<ara::core::Vector<T>>(data_, size_, len_, config_).GetValue();
    }

    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<IsStruct<U>::value ||
        !std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        return ContainerDeserializeHelper<ara::core::Vector<T>>(data_, size_, len_, config_).GetValue();
    }

    template<typename U = T>
    typename std::enable_if<!IsStruct<U>::value && std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return len_ + config_.vectorLength;
    }

    template<typename U = T>
    typename std::enable_if<IsStruct<U>::value || !std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        size_t size = ContainerDeserializeHelper<ara::core::Vector<T>>(data_, size_, len_, config_).GetSize();
        return size;
    }
};

template <typename T>
class Deserializer<std::shared_ptr<T>> {
public:
    using result_type = std::shared_ptr<T>;
    /**
     * @brief Creates a deserializer using the given payload. if the given size is too short to hold the container,
     *        or the len_ will be intialized with the container's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data,
                 std::size_t size,
                 vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), len_(), config_(config)
    {
    }
    ~Deserializer() = default;

    result_type GetValue() const
    {
        if (config_.someipSerializeType != vrtf::serialize::SomeipSerializeType::ENABLETLV) {
            return nullptr;
        }
        return GetValueHelper();
    }

    /**
     * @brief Returns the number of bytes the container and its contents occupy.
     * @details This method may return MAX_SOMEIP_SERIALIZE_SIZE if the length of the container does not fit the array
     *          size.
     * @return Number of bytes the container occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        if (config_.someipSerializeType != vrtf::serialize::SomeipSerializeType::ENABLETLV) {
            return MAX_SOMEIP_SERIALIZE_SIZE;
        }
        // Check if len_ is initialized successfully
        if (size_ < MAX_SOMEIP_SERIALIZE_SIZE) {
            std::size_t sizeTmp = GetSizeHelper();
            if (sizeTmp <= size_) {
                return sizeTmp;
            }
        }
        std::string const ctxId {"CM"};
        std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->error() << "Construct deserialization for option, invalid size parameter!";
        return  MAX_SOMEIP_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t len_;
    const vrtf::serialize::SerializeConfig config_;
    template <typename U = T>
    result_type GetValueHelper() const
    {
        using namespace ara::godel::common::log;
        result_type result;
        Deserializer<U> deserializer(data_, size_, config_);
        if (deserializer.GetSize() > size_) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Construct deserialization for shared pointer failed, invalid size parameter!";
            return result;
        }
        result = std::make_shared<U>(deserializer.GetValue());
        return result;
    }
    template <typename U = T>
    size_t GetSizeHelper(typename std::enable_if<
        !vrtf::serialize::ros::IsRosMsg<U>::value
        && !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value
        && std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        return sizeof(T);
    }

    template <typename U = T>
    size_t GetSizeHelper(typename std::enable_if<
        vrtf::serialize::ros::IsRosMsg<U>::value
        || vrtf::serialize::ros::IsRosBuiltinMsg<U>::value
        || !std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        size_t len = tlv::serialize::TlvSerializeHelper::GetDeserializeLength(config_, data_);
        return len + tlv::serialize::TlvSerializeHelper::GetLengthFieldLength(config_); // Fix Me Now get by type
    }
};

template <typename T, std::size_t N>
class Deserializer<ara::core::Array<T, N>> {
public:
    using result_type = ara::core::Array<T, N>;

    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
    }

    result_type GetValue() const
    {
        return GetValueHelper();
    }

    std::size_t GetSize() const
    {
        return GetSizeHelper();
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
    result_type TraverseDeserializeArray() const
    {
        result_type result;
        std::size_t pos = config_.arrayLength;
        for (std::size_t i {0}; i < N; ++i) {
            Deserializer<T> deserializer(data_ + pos, size_ - pos, config_);
            size_t size = deserializer.GetSize();
            pos += size;
            result[i] = deserializer.GetValue();
        }
        return result;
    }

    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<!IsStruct<U>::value &&
        std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        using namespace ara::godel::common::log;
        bool isCopyable = (sizeof(U) == 1 || !isLittleEndian());
        if (isCopyable) {
            result_type result;
            std::size_t cpySize = N * sizeof(T);
            auto memcpySuccess = memcpy_s(result.data(), cpySize, data_, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance->error() <<
                    "[Someip Deserializer Array] Memory copy return error, Invalid deserialization.";
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            }
            return result;
        }
        return TraverseDeserializeArray();
    }

    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<IsStruct<U>::value ||
        !std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        return TraverseDeserializeArray();
    }

    template<typename U = T>
    typename std::enable_if<!IsStruct<U>::value && std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T) * N + config_.arrayLength;
    }

    template<typename U = T>
    typename std::enable_if<IsStruct<U>::value || !std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        std::size_t result = config_.arrayLength;
        for (std::size_t i {0}; i < N; ++i) {
            Deserializer<T> deserializer(data_ + result, size_ - result, config_);
            std::size_t sizeTmp = deserializer.GetSize();
            if (sizeTmp == MAX_SOMEIP_SERIALIZE_SIZE) {
                return MAX_SOMEIP_SERIALIZE_SIZE;
            } else {
                result += sizeTmp;
            }
        }
        return result;
    }
};


template <typename First, typename Second>
class Deserializer<std::pair<First, Second>> {
public:
    using result_type = std::pair<First, Second>;

    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
    }

    result_type GetValue() const
    {
        using namespace ara::godel::common::log;
        Deserializer<typename std::decay<First>::type> first_deserializer(data_, size_, config_);
        std::size_t first_size = first_deserializer.GetSize();
        if (first_size < size_) {
            Deserializer<typename std::decay<Second>::type> second_deserializer(
                data_ + first_size, size_ - first_size, config_);
            std::size_t sizeTmp = second_deserializer.GetSize();
            if (sizeTmp != MAX_SOMEIP_SERIALIZE_SIZE) {
                return result_type(first_deserializer.GetValue(), second_deserializer.GetValue());
            } else {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "GetValue failed for pair deserialization, insufficient data.";
                return result_type();
            }
        } else {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "GetValue failed for pair deserialization, insufficient data.";
            return result_type();
        }
    }

    std::size_t GetSize() const
    {
        using namespace ara::godel::common::log;
        Deserializer<typename std::decay<First>::type> first_deserializer(data_, size_, config_);
        std::size_t first_size = first_deserializer.GetSize();
        if (first_size < size_) {
            Deserializer<typename std::decay<Second>::type> second_deserializer(
                data_ + first_size, size_ - first_size, config_);
            std::size_t sizeTmp = second_deserializer.GetSize();
            if (sizeTmp != MAX_SOMEIP_SERIALIZE_SIZE) {
                return first_size + sizeTmp;
            } else {
                std::string const ctxId {"CM"};
                std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "GetValue failed for pair deserialization, insufficient data.";
                return  MAX_SOMEIP_SERIALIZE_SIZE;
            }
        } else {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "GetSize failed for pair deserialization, insufficient data.";
            return  MAX_SOMEIP_SERIALIZE_SIZE;
        }
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
};

/**
 * @brief Template specialization for container.
 */
template <typename Container>
class AssociativeDeserializeHelper {
public:
    using result_type = Container;
    /**
     * @brief Creates a deserializer using the given payload.
     * @details The constructor will initialize the len_ with MAX_SOMEIP_SERIALIZE_SIZE, if the given size is too short
     *          to hold the container, or the len_ will be intialized with the container's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    AssociativeDeserializeHelper(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), len_(), config_(config)
    {
        if (size_ >= sizeof(std::uint32_t)) {
            std::size_t lenTmp = *reinterpret_cast<const std::uint32_t* >(data_);
            std::uint32_t* lenTmpPtr = reinterpret_cast<std::uint32_t* >(&lenTmp);
            *lenTmpPtr = ntohlEx(*lenTmpPtr, config_.byteOrder);
            len_ = lenTmp;
            data_ += sizeof(std::uint32_t);
            size_ -= sizeof(std::uint32_t);
        } else {
            len_ = MAX_SOMEIP_SERIALIZE_SIZE;
        }
    }

    virtual ~AssociativeDeserializeHelper() {}
    result_type GetValue() const
    {
        result_type result;
        const std::uint8_t* pos = data_;
        std::size_t remaining = size_;

        std::size_t lenTmp = len_;
        while (lenTmp > 0) {
            Deserializer<typename Container::value_type> deserializer(pos, remaining, config_);

            std::size_t size = deserializer.GetSize();
            if (lenTmp < size) {
                break;
            }
            lenTmp -= size;
            result.insert(deserializer.GetValue());
            pos += size;
            remaining -= size;
        }
        return result;
    }

    /**
     * @brief Returns the number of bytes the AssociativeContainer and its contents occupy.
     * @details This method may return MAX_SOMEIP_SERIALIZE_SIZE if the length of the container does not fit the array
     *          size.
     * @return Number of bytes the AssociativeContainer occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        // Check if len_ is initialized successfully
        if (size_ >= MAX_SOMEIP_SERIALIZE_SIZE) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Construct deserialization for associative container failed, insufficient data.";
            return MAX_SOMEIP_SERIALIZE_SIZE;
        }
        std::size_t result = sizeof(std::uint32_t);
        const std::uint8_t* pos = data_;
        std::size_t remaining = size_;

        std::size_t lenTmp = len_;
        while (lenTmp > 0) {
            Deserializer<typename Container::value_type> deserializer(pos, remaining, config_);

            std::size_t size = deserializer.GetSize();
            if (size == MAX_SOMEIP_SERIALIZE_SIZE) {
                return MAX_SOMEIP_SERIALIZE_SIZE;
            } else {
                if (size > lenTmp) {
                    return MAX_SOMEIP_SERIALIZE_SIZE;
                }

                lenTmp = lenTmp - size;
                pos += size;
                remaining -= size;
                result += size;
            }
        }
        return result;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t len_;
    const vrtf::serialize::SerializeConfig config_;
};

template <typename Key, typename Value>
class Deserializer<ara::core::Map<Key, Value>>
    : public AssociativeDeserializeHelper<ara::core::Map<Key, Value>> {
public:
    using AssociativeDeserializeHelper<ara::core::Map<Key, Value>>::AssociativeDeserializeHelper;
};

/**
 * @brief Template specialization for string.
 */
template <>
class Deserializer<ara::core::String> {
public:
    using result_type = ara::core::String;

    /**
     * @brief Creates a deserializer using the given payload.
     * @details The constructor will initialize the len_ with MAX_SOMEIP_SERIALIZE_SIZE, if the given size is too
     *          short to hold the string, or the len_ will be intialized with the string's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
        if (size_ >= (sizeof(std::uint32_t) + sizeof(utf8_) + sizeof(terminate_))) {
            std::size_t lenTmp = *reinterpret_cast<const std::uint32_t* >(data_);
            std::uint32_t* lenTmpPtr = reinterpret_cast<std::uint32_t* >(&lenTmp);
            *lenTmpPtr = ntohlEx(*lenTmpPtr, config_.byteOrder);
            len_ = lenTmp - (sizeof(utf8_) / sizeof(std::uint8_t) + sizeof(terminate_) / sizeof(std::uint8_t));
            const uint8_t* c = reinterpret_cast<const uint8_t* >(data_ + sizeof(std::uint32_t));
            for (std::size_t i {0}; i < sizeof(utf8_) / sizeof(std::uint8_t); i++) {
                if (utf8_[i] != unsigned(*(c + i))) {
                    len_ = MAX_SOMEIP_SERIALIZE_SIZE;
                    break;
                }
            }
            if (len_ != MAX_SOMEIP_SERIALIZE_SIZE && unsigned(*(c + sizeof(utf8_) + len_)) != 0x00) {
                len_ = MAX_SOMEIP_SERIALIZE_SIZE;
            }
        } else {
            len_ = MAX_SOMEIP_SERIALIZE_SIZE;
        }
    }
    ~Deserializer() = default;
    result_type GetValue() const
    {
        const char* c = reinterpret_cast<const char* >(data_ + sizeof(std::uint32_t) + sizeof(utf8_));
        return {c, len_};
    }

    /**
     * @brief Returns the number of bytes the string occupy.
     * @details This method may return MAX_SOMEIP_SERIALIZE_SIZE if the length of the string (or its contents) does not
     *          fit the given size.
     * @return Number of bytes the string occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        // Check if len_ is initialized successfully
        if (len_ < MAX_SOMEIP_SERIALIZE_SIZE) {
            std::size_t tmp = sizeof(std::uint32_t) + len_ * sizeof(char) + sizeof(utf8_) + sizeof(terminate_);
            if (tmp <= size_) {
                return tmp;
            }
        }
        std::string const ctxId {"CM"};
        std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->error() << "Construct deserialization for String failed, insufficient data.";
        return MAX_SOMEIP_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t len_ = 0;
    // utf8_[3] is use to distinguish utf-8 encoding format.
    const uint8_t utf8_[3] = {0xEF, 0xBB, 0xBF};
    const uint8_t terminate_ = 0x00;
    const vrtf::serialize::SerializeConfig config_;
};

/**
 * @brief Template specialization for std::string.
 */
template <>
class Deserializer<std::string> {
public:
    using result_type = std::string;

    /**
     * @brief Creates a deserializer using the given payload.
     * @details The constructor will initialize the len_ with MAX_SOMEIP_SERIALIZE_SIZE, if the given size is too
     *          short to hold the string, or the len_ will be intialized with the string's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig())
        : data_(data), size_(size), config_(config)
    {
        if (CheckIsValidSize()) {
            size_t lenTmp = tlv::serialize::TlvSerializeHelper::GetDeserializeLength(config_, data_);
            if (config_.implementsLegencyStringSerialization == true) {
                len_ = lenTmp;
            } else {
                len_ = lenTmp - (sizeof(utf8_) / sizeof(std::uint8_t) + sizeof(terminate_) / sizeof(std::uint8_t));
                const uint8_t* c = reinterpret_cast<const uint8_t* >(data_ + config_.stringLength);
                for (std::size_t i {0}; i < sizeof(utf8_) / sizeof(std::uint8_t); i++) {
                    if (utf8_[i] != unsigned(*(c + i))) {
                        len_ = MAX_SOMEIP_SERIALIZE_SIZE;
                        break;
                    }
                }
                if (len_ != MAX_SOMEIP_SERIALIZE_SIZE && unsigned(*(c + sizeof(utf8_) + len_)) != 0x00) {
                    len_ = MAX_SOMEIP_SERIALIZE_SIZE;
                }
            }
        } else {
            len_ = MAX_SOMEIP_SERIALIZE_SIZE;
        }
    }
    ~Deserializer() = default;
    result_type GetValue() const
    {
        if (config_.implementsLegencyStringSerialization == true) {
            const char* c = reinterpret_cast<const char* >(data_ + config_.stringLength);
            return {c, len_};
        } else {
            const char* c = reinterpret_cast<const char* >(data_ + config_.stringLength + sizeof(utf8_));
            return {c, len_};
        }
    }

    /**
     * @brief Returns the number of bytes the string occupy.
     * @details This method may return MAX_SOMEIP_SERIALIZE_SIZE if the length of the string (or its contents) does not
     *          fit the given size.
     * @return Number of bytes the string occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        // Check if len_ is initialized successfully
        if (len_ < MAX_SOMEIP_SERIALIZE_SIZE) {
            std::size_t tmp = config_.stringLength + len_ * sizeof(char);
            if (config_.implementsLegencyStringSerialization == false) {
                tmp += sizeof(utf8_) + sizeof(terminate_);
            }
            if (tmp <= size_) {
                return tmp;
            }
        }
        std::string const ctxId {"CM"};
        std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->error() << "Construct deserialization for std::string failed, insufficient data.";
        return MAX_SOMEIP_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t len_ = 0;
    // utf8_[3] is use to distinguish utf-8 encoding format.
    const uint8_t utf8_[3] = {0xEF, 0xBB, 0xBF};
    const uint8_t terminate_ = 0x00;
    const vrtf::serialize::SerializeConfig config_;
    bool CheckIsValidSize()
    {
        if (config_.someipSerializeType == vrtf::serialize::SomeipSerializeType::ENABLETLV &&
            config_.implementsLegencyStringSerialization == true) {
            if (size_ < config_.stringLength) {
                return false;
            }
        } else {
            if (size_ < config_.stringLength + sizeof(utf8_) + sizeof(terminate_)) {
                return false;
            }
        }
        return true;
    }
};

/**
 * @brief Template specialization for ara::core::ErrorCode.
 */
template <>
class Deserializer<ara::core::ErrorCode> {
public:
    using result_type = ara::core::ErrorCode;

    /**
     * @brief Creates a deserializer using the given payload.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
    }
    ~Deserializer() = default;
    vrtf::vcc::api::types::MethodError GetValue() const
    {
        return GetValueHelper();
    }

    /**
     * @brief Returns the number of bytes the ara::core::ErrorCode occupy.
     * @details This method may return MAX_SOMEIP_SERIALIZE_SIZE if the length of the string (or its contents) does not
     *          fit the given size or the type is not 0x01.
     * @return Number of bytes the ara::core::ErrorCode occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        std::uint8_t type = 0x00;
        auto memcpySuccess = memcpy_s(&type, sizeof(uint8_t), data_ + sizeof(uint32_t), sizeof(uint8_t));
        if (memcpySuccess != 0) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance->error() <<
                "[Someip Deserializer ErrorCode] Memory copy return error, Invalid deserialization.";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        }
        if (type != 0x01) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance->error() <<
                "Construct deserialization for ErrorCode failed, invalid type of ErrorCode!";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return  MAX_SOMEIP_SERIALIZE_SIZE;
        }
        if (size_ >= (sizeof(std::uint8_t) + sizeof(std::uint32_t))) {
            std::size_t sizeTmp = sizeof(std::uint8_t) + sizeof(std::uint32_t) +
                                    sizeof(std::uint64_t) + sizeof(std::int32_t);
            if (sizeTmp == size_) {
                return sizeTmp;
            }
        }
        std::string const ctxId {"CM"};
        std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
        /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
        /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
        logInstance->error() <<
            "Construct deserialization for ErrorCode failed, invalid size parameter!";
        /* AXIVION enable style AutosarC++19_03-A5.0.1 */
        /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        return  MAX_SOMEIP_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
    vrtf::vcc::api::types::MethodError GetValueHelper() const
    {
        using namespace ara::godel::common::log;
        std::uint64_t errorDomain = 0x00000000;
        auto memcpySuccess = memcpy_s(&errorDomain, sizeof(uint64_t),
            data_ + sizeof(uint32_t) + sizeof(uint8_t), sizeof(uint64_t));
        if (memcpySuccess != 0) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance->error() <<
                "[Someip Deserializer ErrorCode] Memory copy return error, Invalid deserialization.";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        }
        auto valueDomian = *reinterpret_cast<const uint64_t* >(&errorDomain);
        std::uint64_t* valueP64 = reinterpret_cast<std::uint64_t* >(&valueDomian);
        *valueP64 = ntohl64(*valueP64, config_.byteOrder);

        std::int32_t errorCodeValue = 0x0000;
        memcpySuccess = memcpy_s(&errorCodeValue, sizeof(int32_t),
            data_ + sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint64_t), sizeof(int32_t));
        if (memcpySuccess != 0) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance->error() <<
                "[Someip Deserializer ErrorCode] Memory copy return error, Invalid deserialization.";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        }
        auto valueCodeValue = *reinterpret_cast<const int32_t* >(&errorCodeValue);
        std::int32_t* valueP32 = reinterpret_cast<std::int32_t* >(&valueCodeValue);
        *valueP32 = static_cast<std::int32_t>(ntohlEx(static_cast<std::uint32_t>(*valueP32), config_.byteOrder));

        vrtf::vcc::api::types::MethodError errorData;
        errorData.domainValue = valueDomian;
        errorData.errorCode = valueCodeValue;
        return errorData;
    }
};

class DeserializeSizeCounter {
public:
    DeserializeSizeCounter(const std::uint8_t* data, std::size_t bufSize,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : bufSize_(bufSize), data_(data), config_(config)
    {
    }
    ~DeserializeSizeCounter() = default;

    template <typename T>
    void operator()(const T&)
    {
        using namespace ara::godel::common::log;
        Deserializer<T> deserializer(data_, bufSize_, config_);
        std::size_t size = deserializer.GetSize();
        if (size == MAX_SOMEIP_SERIALIZE_SIZE || size_ == MAX_SOMEIP_SERIALIZE_SIZE || bufSize_ < size) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Deserialization[Someip] size counter failed, insufficient data.";
            size_ = MAX_SOMEIP_SERIALIZE_SIZE;
        } else {
            size_ += size;
            data_ += size;
            bufSize_ -= size;
        }
    }

    std::size_t GetSize() const
    {
        return size_;
    }

private:
    std::size_t size_ = 0;
    std::size_t bufSize_;
    const std::uint8_t* data_;
    const vrtf::serialize::SerializeConfig config_;
};

class StructDeserializeHelper {
public:
    StructDeserializeHelper(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
    }
    ~StructDeserializeHelper() = default;

    template <typename T>
    void operator()(T& value)
    {
        using namespace ara::godel::common::log;
        Deserializer<T> deserializer(data_ + pos_, size_ - pos_, config_);
        pos_ += deserializer.GetSize();
        if (pos_ > size_) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Invalid parameter: payload size, it's insufficient!";
        } else {
            value = deserializer.GetValue();
        }
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t pos_{0};
};


template <typename T>
class Deserializer {
public:
    using value_type = typename std::decay<T>::type;
    Deserializer(const std::uint8_t* data,
                 std::size_t size,
                 vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
        static_assert(IsSerializable<value_type>(), "Not support deserialization for this type!");
    }
    ~Deserializer() = default;

    value_type GetValue() const
    {
        return GetValueHelper();
    }

    std::size_t GetSize() const
    {
        return GetSizeHelper();
    }
private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
    template <typename U = value_type>
    value_type GetValueHelper(
        typename std::enable_if<!IsStruct<U>::value
        && !vrtf::serialize::ros::IsRosMsg<U>::value
        && !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value
        && std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        using namespace ara::godel::common::log;
        if (sizeof(value_type) > size_) {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Invalid parameter for trivially copyable type: size, it's insufficient!";
        }

        auto value = *reinterpret_cast<const value_type* >(data_);
        std::size_t size = sizeof(U);

        switch (size) {
            case TWO_BYTES_LENGTH: { // 2: data bytes length, like short int
                std::uint16_t* value_p16 = reinterpret_cast<std::uint16_t* >(&value);
                *value_p16 = ntohsEx(*value_p16, config_.byteOrder);
                break;
            }
            case FOUR_BYTES_LENGTH: { // 4: data bytes length, like int
                std::uint32_t* value_p32 = reinterpret_cast<std::uint32_t* >(&value);
                *value_p32 = ntohlEx(*value_p32, config_.byteOrder);
                break;
            }
            case EIGHT_BYTES_LENGTH: { // 8: data bytes length, like long long
                std::uint64_t* value_p64 = reinterpret_cast<std::uint64_t* >(&value);
                *value_p64 = ntohl64(*value_p64, config_.byteOrder);
                break;
            }
            default: {}
        }
        return value;
    }

    template <typename U = value_type>
    value_type GetValueHelper(typename std::enable_if<IsStruct<U>::value>::type* = 0) const
    {
        value_type result = value_type();
        StructDeserializeHelper deserializer(data_, size_, config_);
        result.enumerate(deserializer);
        return result;
    }

    template <typename U = value_type>
    value_type GetValueHelper(typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value
        && !vrtf::serialize::ros::IsRosTlv<U>::value>::type* = 0) const
    {
        value_type result = value_type();
        vrtf::serialize::ros::IStream<vrtf::serialize::someip::Deserializer> stream(data_, size_, config_);
        ::ros::serialization::Serializer<U>::template allInOne<
            vrtf::serialize::ros::IStream<vrtf::serialize::someip::Deserializer>, U&>(stream, result);
        return result;
    }

    template <typename U = value_type>
    value_type GetValueHelper(typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value
        && vrtf::serialize::ros::IsRosTlv<U>::value>::type* = 0) const
    {
        value_type result = value_type();
        size_t pos = 0;
        if (CheckStructLengthField(config_)
            && config_.someipSerializeType == vrtf::serialize::SomeipSerializeType::ENABLETLV) {
            pos = config_.structLength;
        }
        vrtf::serialize::SerializeConfig config = config_;
        config.structDeserializeLength = GetSize() - pos;
        config.isTopStruct = false;
        config.isIngoreOutLength = false;
        vrtf::serialize::ros::IStream<vrtf::serialize::someip::Deserializer> stream(data_ + pos, size_ - pos, config);
        if (config_.someipSerializeType == vrtf::serialize::SomeipSerializeType::ENABLETLV) {
            ::ros::serialization::Serializer<U>::template allInOneTlv<
                vrtf::serialize::ros::IStream<vrtf::serialize::someip::Deserializer>, U&>(stream, result);
        } else {
            ::ros::serialization::Serializer<U>::template allInOne<
                vrtf::serialize::ros::IStream<vrtf::serialize::someip::Deserializer>, U&>(stream, result);
        }
        return result;
    }

    template <typename U = value_type>
    value_type GetValueHelper(
        typename std::enable_if<vrtf::serialize::ros::IsRosBuiltinMsg<U>::value>::type* = 0) const
    {
        value_type result = value_type();
        using istreamType = vrtf::serialize::ros::IStream<vrtf::serialize::someip::Deserializer>;
        std::shared_ptr<istreamType> stream =
            std::make_shared<istreamType>(const_cast<uint8_t *>(data_), size_, config_);
        ::ros::serialization::Serializer<U>::read(*stream, result);
        return result;
    }

    template <typename U = value_type>
    typename std::enable_if<!IsStruct<U>::value
        && !vrtf::serialize::ros::IsRosMsg<U>::value
        && std::is_trivially_copyable<U>::value
        && !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T);
    }

    template <typename U = value_type>
    typename std::enable_if<IsStruct<U>::value, std::size_t>::type GetSizeHelper() const
    {
        DeserializeSizeCounter size_counter(data_, size_, config_);
        value_type* x = nullptr;
        x->enumerate(size_counter);
        return size_counter.GetSize();
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value
        && !vrtf::serialize::ros::IsRosTlv<U>::value, std::size_t>::type GetSizeHelper() const
    {
        vrtf::serialize::ros::ISizeStream<vrtf::serialize::someip::Deserializer> stream(data_, size_, config_);
        ::ros::serialization::Serializer<U>::template allInOne<
            vrtf::serialize::ros::ISizeStream<vrtf::serialize::someip::Deserializer>, U>(stream, value_type());
        return stream.GetSize();
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value
        && vrtf::serialize::ros::IsRosTlv<U>::value, std::size_t>::type GetSizeHelper() const
    {
        if (config_.someipSerializeType == vrtf::serialize::SomeipSerializeType::DISABLETLV) {
            vrtf::serialize::ros::ISizeStream<vrtf::serialize::someip::Deserializer> stream(data_, size_, config_);
            ::ros::serialization::Serializer<U>::template allInOne<
                vrtf::serialize::ros::ISizeStream<vrtf::serialize::someip::Deserializer>, U>(stream, value_type());
            return stream.GetSize();
        }
        size_t pos = 0;
        size_t len = size_;
        if (CheckStructLengthField(config_)) {
            pos = config_.structLength;
            len = tlv::serialize::TlvSerializeHelper::GetDeserializeLength(config_, data_);
        }
        if (len + pos > size_) {
            std::string const ctxId {"CM"};
            std::shared_ptr<ara::godel::common::log::Log> logInstance {ara::godel::common::log::Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Get deserialize length is : " << len + pos
                << " actual deserialize length is : " << size_;
            return MAX_SOMEIP_SERIALIZE_SIZE;
        }
        vrtf::serialize::SerializeConfig config {config_};
        config.isTopStruct = false;
        config.isIngoreOutLength = false;
        config.structDeserializeLength = len;
        vrtf::serialize::ros::ISizeStream<vrtf::serialize::someip::Deserializer> stream {
            data_ + pos, size_ - pos, config};
        ::ros::serialization::Serializer<U>::template allInOneTlv<
            vrtf::serialize::ros::ISizeStream<vrtf::serialize::someip::Deserializer>, U>(stream, value_type());
        size_t size {stream.GetSize() + pos};
        return size;
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type GetSizeHelper() const
    {
        value_type value = value_type();
        return ::ros::serialization::Serializer<U>::serializedLength(value);
    }
};
} // someip
} // serialize
} // vrtf

#endif
