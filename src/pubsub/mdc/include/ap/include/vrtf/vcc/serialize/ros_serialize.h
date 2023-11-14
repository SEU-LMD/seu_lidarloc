/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file provides an an interface to support ros serialize.
 * Create: 2020-11-04
 */
#ifndef VRTF_ROS_SERIALIZE_H
#define VRTF_ROS_SERIALIZE_H
#include "vrtf/vcc/serialize/serialize_config.h"
#include "vrtf/vcc/serialize/tlv_serialize_helper.h"
#include <arpa/inet.h>
/*
Note: Serialize max size is 16M (equal to 0x100000), cannot be result in integer overflow with type of size_t
*/
namespace ros {
namespace serialization {
    template<class T>
    class Serializer;

    class IStream;

    class OStream;
}
}

namespace ros {
    class Time;
    class Duration;
}

namespace vrtf {
namespace serialize {
namespace ros {
const std::size_t MAX_ROS_SERIALIZE_SIZE {0xFFFFFFFFU};
const std::uint16_t DEFAULT_DATA_ID {0xFFFFU};

template<typename T>
struct IsArray : public std::false_type {
};

template<typename T, std::size_t N>
struct IsArray<std::array<T, N>> : public std::true_type {};

template<typename T>
struct HasFuncAllInOne {
public:
    static constexpr uint32_t tmpstream {1}; // Used for allInOne
    template<typename U>
    static auto Check(uint32_t) noexcept -> decltype(U::allInOne(tmpstream, U()), std::true_type());
    template<typename U>
    static auto Check(...) noexcept -> std::false_type;
    enum { value = std::is_same<decltype(Check<T>(0)), std::true_type>::value };
};

template<typename T>
struct HasFuncAllInOneTlv {
public:
    static constexpr uint32_t tmpstream {1}; // Used for allInOne
    template<typename U>
    static auto Check(uint32_t) noexcept -> decltype(U::allInOneTlv(tmpstream, U()), std::true_type());
    template<typename U>
    static auto Check(...) noexcept -> std::false_type;
    enum { value = std::is_same<decltype(Check<T>(0)), std::true_type>::value };
};

template<typename T>
struct CheckRosBuiltinMsg {
public:
    enum {
        value = (std::is_same<typename std::decay<T>::type, ::ros::Time>::value ||
            std::is_same<typename std::decay<T>::type, ::ros::Duration>::value)
    };
};

template<typename T>
struct CheckShapeShifterMsg {
public:
    enum {
        value = (std::is_same<typename std::decay<T>::type, vrtf::vcc::api::types::ShapeShifter>::value)
    };
};

template<typename T>
struct HasMemberRequest {
public:
    template<typename U>
    static auto Check(U) -> typename std::decay<decltype(U::request)>::type;
    static void Check(...);
    enum { value = !std::is_void<decltype(Check(std::declval<T>()))>::value };
};

template<typename T>
using IsRosMsg = HasFuncAllInOne<::ros::serialization::Serializer<T>>;

template<typename T>
using IsRosTlv = HasFuncAllInOneTlv<::ros::serialization::Serializer<T>>;

template<typename T>
using IsRosBuiltinMsg = CheckRosBuiltinMsg<T>;

template<typename T>
using IsShapeShifterMsg = CheckShapeShifterMsg<T>;

template<typename T>
using IsRosMethodMsg = HasMemberRequest<T>;


template <typename U>
class RosWireTypeHelper {
public:
    template <typename T = U>
    static std::uint16_t GetWireType(const vrtf::serialize::SerializeConfig& config, const std::uint16_t dataId,
        typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<T>::value
        && !vrtf::serialize::ros::IsRosBuiltinMsg<T>::value
        && (std::is_trivially_copyable<T>::value && !IsArray<T>::value)>::type* = 0) noexcept
    {
        static_cast<void>(config);
        std::uint16_t id {dataId};
        switch (sizeof(T)) {
            case vrtf::serialize::someip::ONE_BYTES_LENGTH: {
                id = static_cast<std::uint16_t>(id | (tlv::serialize::WIRETYPE_ZERO << tlv::serialize::WIRETYPE_POS));
                break;
            }
            case vrtf::serialize::someip::TWO_BYTES_LENGTH: {
                id = static_cast<std::uint16_t>(id | (tlv::serialize::WIRETYPE_ONE << tlv::serialize::WIRETYPE_POS));
                break;
            }
            case vrtf::serialize::someip::FOUR_BYTES_LENGTH: {
                id = static_cast<std::uint16_t>(id | (tlv::serialize::WIRETYPE_TWO << tlv::serialize::WIRETYPE_POS));
                break;
            }
            case vrtf::serialize::someip::EIGHT_BYTES_LENGTH: {
                id = static_cast<std::uint16_t>(id | (tlv::serialize::WIRETYPE_THREE << tlv::serialize::WIRETYPE_POS));
                break;
            }
            default: {}
        }
        return id;
    }

    template <typename T = U>
    static std::uint16_t GetWireType(const vrtf::serialize::SerializeConfig& config, const std::uint16_t dataId,
        typename std::enable_if<vrtf::serialize::ros::IsRosMsg<T>::value
        || vrtf::serialize::ros::IsRosBuiltinMsg<T>::value
        || (!std::is_trivially_copyable<T>::value || IsArray<T>::value)>::type* = 0) noexcept
    {
        std::uint16_t id {dataId};
        if (config.wireType == vrtf::serialize::WireType::STATIC) {
            id = static_cast<std::uint16_t>(id | (tlv::serialize::WIRETYPE_FOUR << tlv::serialize::WIRETYPE_POS));
        } else {
            id = static_cast<std::uint16_t>(id | (tlv::serialize::WIRETYPE_SEVEN << tlv::serialize::WIRETYPE_POS));
        }
        return id;
    }
};

template <typename T>
class RosWireTypeHelper<std::shared_ptr<T>> {
public:
    static std::uint16_t GetWireType(
        const vrtf::serialize::SerializeConfig& config, const std::uint16_t dataId) noexcept
    {
        return RosWireTypeHelper<T>::GetWireType(config, dataId);
    }
};

template <template<typename ValueType> class DeserializerType>
class IStream {
public:
    IStream(const std::uint8_t* address, const std::size_t length, const vrtf::serialize::SerializeConfig& config)
        : tlvDataInit_(false), initAddress_(address), address_(address), length_(length), config_(config) {}
    ~IStream() = default;
    template<typename T>
    void next(T& value, std::uint16_t dataId = DEFAULT_DATA_ID) noexcept
    {
        if (dataId != DEFAULT_DATA_ID) {
            if (tlvDataInit_ == false) {
                tlv::serialize::RecordDataIdResult res {tlv::serialize::TlvSerializeHelper::RecordDataId(
                    config_, initAddress_, config_.structDeserializeLength)};
                dataIdMap_ = std::move(res.dataIdMap);
                tlvDataInit_ = true;
            }
            auto iter {dataIdMap_.find(dataId)};
            if (iter != dataIdMap_.end()) {
                DeserializerType<T> deserializer {initAddress_ + iter->second.totalSize,
                    config_.structDeserializeLength - iter->second.totalSize, config_};
                value = deserializer.GetValue();
            }
        } else {
            DeserializerType<T> deserializer {address_, length_, config_};
            auto size {deserializer.GetSize()};
            if ((length_ - size) < 0) {
                return;
            }
            address_ += size;
            length_ += size;
            value = deserializer.GetValue();
        }
    }
private:
    bool tlvDataInit_;
    const uint8_t* initAddress_;
    const uint8_t* address_;
    std::size_t length_;
    vrtf::serialize::SerializeConfig config_;
    std::unordered_map<std::uint16_t, tlv::serialize::dataIdParams> dataIdMap_ {};
};

template <template<typename ValueType> class SerializerType>
class OStream {
public:
    OStream(std::uint8_t* address, const vrtf::serialize::SerializeConfig& config)
        : address_(address), config_(config) {}
    ~OStream() = default;
    template<typename T>
    void next(const T& value, std::uint16_t dataId = DEFAULT_DATA_ID) noexcept
    {
        using namespace ara::godel::common::log;
        SerializerType<T> serializer {value, config_};
        tlv::serialize::GetSizeResult result {GetSize<T>(value, serializer, dataId)};
        if (result.isSkip == true) {
            // is optional
            return;
        }
        if (dataId != DEFAULT_DATA_ID) {
            dataId = RosWireTypeHelper<T>::GetWireType(config_, dataId);
            dataId = vrtf::serialize::someip::htonsEx(dataId, config_.byteOrder);
            auto memcpyResult = memcpy_s(address_, result.size, &dataId, sizeof(std::uint16_t));
            if (memcpyResult != 0) {
                std::shared_ptr<Log> logInstance {Log::GetLog("CM")};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "Serialization[Ros] copy dataId : " << dataId << " failed";
            }
        }
        serializer.Serialize(address_ + result.tagPos_);
        address_ += result.size;
    }
private:
    template <typename T>
    tlv::serialize::GetSizeResult GetSize(const T& value, SerializerType<T>& serializer, const std::uint16_t dataId,
        typename std::enable_if<tlv::serialize::is_smart_pointer_helper<T>::value>::type* = 0) noexcept
    {
        tlv::serialize::GetSizeResult result {false, 0, 0};
        if (value == nullptr) {
            result.isSkip = true;
            result.size = 0;
            return result;
        }
        if (dataId != DEFAULT_DATA_ID) {
            result.tagPos_ += sizeof(std::uint16_t);
        }
        result.size = serializer.GetSize() + result.tagPos_;
        return result;
    }

    template <typename T>
    tlv::serialize::GetSizeResult GetSize(const T& value, SerializerType<T>& serializer, const std::uint16_t dataId,
        typename std::enable_if<!tlv::serialize::is_smart_pointer_helper<T>::value>::type* = 0) noexcept
    {
        tlv::serialize::GetSizeResult result {false, 0, 0};
        if (dataId != DEFAULT_DATA_ID) {
            result.tagPos_ += sizeof(std::uint16_t);
        }
        result.size = serializer.GetSize() + result.tagPos_;
        return result;
    }
    uint8_t* address_;
    vrtf::serialize::SerializeConfig config_;
};

template <template<typename ValueType> class DeserializerType>
class ISizeStream {
public:
    ISizeStream(const std::uint8_t* address, const std::size_t length, const vrtf::serialize::SerializeConfig& config)
        : tlvDataInit_(false), address_(address), initAddress_(address), length_(length), config_(config) {}
    ~ISizeStream() = default;
    template<typename T>
    void next(const T&, std::uint16_t dataId = DEFAULT_DATA_ID) noexcept
    {
        using namespace ara::godel::common::log;
        size_t sizeTmp {0};
        if (size_ == MAX_ROS_SERIALIZE_SIZE) {
            return;
        }
        if ((config_.someipSerializeType == vrtf::serialize::SomeipSerializeType::ENABLETLV)
            && (dataId != DEFAULT_DATA_ID)) {
            if (tlvDataInit_ == false) {
                tlv::serialize::RecordDataIdResult res {tlv::serialize::TlvSerializeHelper::RecordDataId(
                    config_, initAddress_, config_.structDeserializeLength)};
                dataIdMap_ = std::move(res.dataIdMap);
                totalSize_ = res.size;
                tlvDataInit_ = true;
            }
            auto typeIter = dataIdMap_.find(dataId);
            if (typeIter == dataIdMap_.end()) {
                if (!tlv::serialize::is_smart_pointer_helper<T>::value) {
                    std::string const ctxId {"CM"};
                    std::shared_ptr<Log> logInstance = Log::GetLog(ctxId);
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance->error() << "Deserialization[ROS] cannot find necessary dataId " << dataId;
                    size_ = MAX_ROS_SERIALIZE_SIZE;
                }
            } else {
                size_t typeSize = DeserializerType<T>(address_ + typeIter->second.totalSize,
                    typeIter->second.paramsSize, config_).GetSize();
                if (typeSize > typeIter->second.paramsSize) {
                    std::string const ctxId {"CM"};
                    std::shared_ptr<Log> logInstance = Log::GetLog(ctxId);
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance->error() << "Deserialization[ROS] check has the same serialize config";
                    size_ = MAX_ROS_SERIALIZE_SIZE;
                }
            }
        } else {
            sizeTmp = DeserializerType<T>(address_, length_, config_).GetSize();
        }
        if ((sizeTmp < MAX_ROS_SERIALIZE_SIZE) && (size_ < MAX_ROS_SERIALIZE_SIZE) && (length_ >= sizeTmp)) {
            address_ += sizeTmp;
            size_ += sizeTmp;
            length_ -= sizeTmp;
        } else {
            std::string const ctxId {"CM"};
            std::shared_ptr<Log> logInstance {Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Deserialization[ROS] size counter failed, insufficient data.";
            size_ = MAX_ROS_SERIALIZE_SIZE;
        }
    }

    std::size_t GetSize() const noexcept
    {
        if (size_ >= MAX_ROS_SERIALIZE_SIZE) {
            return size_;
        }
        if (totalSize_ != 0) {
            return totalSize_;
        }
        return size_;
    }
private:
    bool tlvDataInit_;
    std::size_t size_ = 0;
    const uint8_t* address_;
    const uint8_t* initAddress_;
    std::size_t length_;
    vrtf::serialize::SerializeConfig config_;
    std::unordered_map<std::uint16_t, tlv::serialize::dataIdParams> dataIdMap_ {};
    size_t totalSize_ = 0;
};

template <template<typename ValueType> class SerializerType>
class OSizeStream {
public:
    OSizeStream(const vrtf::serialize::SerializeConfig& config) : config_(config) {}
    ~OSizeStream() = default;

    template<typename T>
    void next(const T& value, std::uint16_t dataId = DEFAULT_DATA_ID) noexcept
    {
        using namespace ara::godel::common::log;
        size_t dataSize {0};
        if (dataId == DEFAULT_DATA_ID) { // if dataId not equal to 0xFFFF, mean not use tlv dataType
            dataSize = SerializerType<T>(value, config_).GetSize();
        } else {
            dataSize = SerializerType<T>(value, config_).GetSize();
            if (dataSize == 0) { // dataSize = 0 mean nullptr
                return;
            }
            dataSize += sizeof(std::uint16_t);  // Add tag length
        }
        if ((dataSize < MAX_ROS_SERIALIZE_SIZE) && (size_ < MAX_ROS_SERIALIZE_SIZE)) {
            size_ += dataSize;
        } else {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Serialization[ROS] size counter failed, insufficient data.";
            size_ = MAX_ROS_SERIALIZE_SIZE;
        }
    }

    std::size_t GetSize() const noexcept
    {
        return size_;
    }
private:
    std::size_t size_ = 0;
    vrtf::serialize::SerializeConfig config_;
};
} // ros
} // serialize
} // vrtf
#endif // VRTF_ROS_SERIALIZE_H
