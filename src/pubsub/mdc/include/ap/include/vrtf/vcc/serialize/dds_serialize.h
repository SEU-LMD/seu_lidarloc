/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: DdsSerialize in vcc
 * Create: 2019-11-19
 */
#ifndef DDS_SERIALIZE_H
#define DDS_SERIALIZE_H

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
#include "vrtf/vcc/serialize/serialize_config.h"
#include "vrtf/vcc/serialize/ros_serialize.h"
#include <securec.h>
#include <type_traits>
#include <cstdint>
#include <cstring>
#include <memory>
/*
Note: Serialize max size is 16M (equal to 0x100000), cannot be result in integer overflow with type of size_t
*/
namespace vrtf {
namespace serialize {
namespace dds {
const std::size_t MAX_DDS_SERIALIZE_SIZE = 0xFFFFFFFFU;
template <typename T, typename Tag = void>
struct is_enumerable {
    static const bool value = false;
};

template <typename T>
struct is_enumerable<T, typename T::IsEnumerableTag> {
    static const bool value = true;
};

template <typename T, typename Tag = void>
struct is_dp_raw_data {
    static const bool value = false;
};

template <typename T>
struct is_dp_raw_data<T, typename T::IsDpRawDataTag> {
    static const bool value = true;
};

template <typename T>
class Serializer;

template <typename T>
class Deserializer;

template <typename T>
constexpr bool IsSerializable()
{
    return std::is_trivially_copyable<T>::value ||
        is_enumerable<T>::value ||
        vrtf::serialize::ros::IsRosMsg<T>::value ||
        vrtf::serialize::ros::IsRosBuiltinMsg<T>::value ||
        ::tlv::serialize::is_smart_pointer_helper<T>::value;
}

class SerializingEnumerator {
public:
    SerializingEnumerator(std::uint8_t* payload_data,
                          vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
                          std::size_t pos = 0)
        : payload_data_(payload_data), config_(config), pos_(pos)
    {
    }
    ~SerializingEnumerator() = default;
    template <typename T>
    void operator()(const T& value)
    {
        Serializer<T> serializer(value, config_, pos_);
        pos_ += serializer.GetSize();
        serializer.Serialize(payload_data_);
    }

private:
    std::uint8_t* payload_data_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t pos_;
};

class SerializeSizeCounter {
public:
    SerializeSizeCounter(vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : config_(config)
    {}

    ~SerializeSizeCounter() = default;

    std::size_t GetSize() const
    {
        return size_;
    }

    template <typename T>
    void operator()(const T& value)
    {
        Serializer<T> serializer(value, config_);
        std::size_t size = serializer.GetSize();
        size_ += size;
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
        static_assert(IsSerializable<T>(), "No appropriate marshalling defined for this type!");
    }
    ~Serializer() {};
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
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
    template <typename U = value_type>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<!is_enumerable<U>::value
        && !vrtf::serialize::ros::IsRosMsg<U>::value
        && !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value
        && std::is_trivially_copyable<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(&value_);
        auto memcpySuccess = memcpy_s(c + pos_, size_, data, sizeof(T));
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error()<< "[DDS Serializer] Memory copy return error, invalid data.";
        }
    }

    template <typename U = value_type>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<is_enumerable<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        if (value_type::IsPlane() &&
            config_.structPolicy == vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT) {
            const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(&value_);
            auto memcpySuccess = memcpy_s(c + pos_, size_, data, sizeof(T));
            if (memcpySuccess != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error()<< "[DDS Serializer] Memory copy return error, invalid data.";
            }
            return;
        }
        vrtf::serialize::SerializeConfig config = config_;
        if (config_.structPolicy == vrtf::serialize::StructSerializationPolicy::DISABLE_OUT_LAYER_ALIGNMENT) {
            config.structPolicy = vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT;
        }
        SerializingEnumerator enumerator(c, config, pos_);
        (const_cast<value_type&>(value_)).enumerate(enumerator);
    }

    template <typename U = value_type>
    void SerializeHelper(std::uint8_t* c,
                         typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        if (std::is_trivially_copyable<U>::value && (config_.type == vrtf::serialize::SerializationType::CM) &&
            config_.structPolicy == vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT) {
            const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(&value_);
            auto memcpySuccess = memcpy_s(c + pos_, size_, data, sizeof(T));
            if (memcpySuccess != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error()<< "[DDS Serializer] Memory copy return error, invalid data.";
            }
            return;
        }
        vrtf::serialize::SerializeConfig config = config_;
        if (config_.structPolicy == vrtf::serialize::StructSerializationPolicy::DISABLE_OUT_LAYER_ALIGNMENT) {
            config.structPolicy = vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT;
        }
        vrtf::serialize::ros::OStream<vrtf::serialize::dds::Serializer> stream(c + pos_, config);
        ::ros::serialization::Serializer<U>::allInOne(stream, value_);
    }

    template <typename U = value_type>
    void SerializeHelper(std::uint8_t* c,
                         typename std::enable_if<vrtf::serialize::ros::IsRosBuiltinMsg<U>::value>::type* = 0)
    {
        std::shared_ptr<vrtf::serialize::ros::OStream<vrtf::serialize::dds::Serializer>> stream
            = std::make_shared<vrtf::serialize::ros::OStream<vrtf::serialize::dds::Serializer>>(c + pos_, config_);
        ::ros::serialization::Serializer<U>::write(*stream, value_);
    }

    template <typename U = value_type>
    typename std::enable_if<!is_enumerable<U>::value
        && std::is_trivially_copyable<U>::value
        && !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value
        && !vrtf::serialize::ros::IsRosMsg<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T);
    }

    template <typename U = value_type>
    typename std::enable_if<is_enumerable<U>::value, std::size_t>::type GetSizeHelper() const
    {
        if (value_type::IsPlane() &&
            config_.structPolicy == vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT) {
            return sizeof(T);
        }
        vrtf::serialize::SerializeConfig config = config_;
        if (config_.structPolicy == vrtf::serialize::StructSerializationPolicy::DISABLE_OUT_LAYER_ALIGNMENT) {
            config.structPolicy = vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT;
        }
        SerializeSizeCounter sizeCounter(config);
        (const_cast<value_type&>(value_)).enumerate(sizeCounter);
        return sizeCounter.GetSize();
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value, std::size_t>::type GetSizeHelper() const
    {
        if (std::is_trivially_copyable<U>::value && (config_.type == vrtf::serialize::SerializationType::CM) &&
            config_.structPolicy == vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT) {
            return sizeof(T);
        }
        vrtf::serialize::SerializeConfig config = config_;
        if (config_.structPolicy == vrtf::serialize::StructSerializationPolicy::DISABLE_OUT_LAYER_ALIGNMENT) {
            config.structPolicy = vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT;
        }
        vrtf::serialize::ros::OSizeStream<vrtf::serialize::dds::Serializer> stream(config);
        ::ros::serialization::Serializer<U>::allInOne(stream, value_);
        return stream.GetSize();
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type GetSizeHelper() const
    {
        value_type value;
        return ::ros::serialization::Serializer<U>::serializedLength(value);
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

    /**
     * @brief Serialize the value by someip union
     * @param c Store the serialized code stream address
     */
    void Serialize(std::uint8_t* c)
    {
        SerializeHelper(c, pos_);
    }
    ~Serializer() = default;
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
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
    /**
     * @brief Serialize the error code
     * @param c Store the serialized code stream address
     * @param pos Data storage offset address
     */
    void SerializeHelper(std::uint8_t* c, size_t pos)
    {
        using namespace ara::godel::common::log;
        std::uint8_t type = 0x01;
        std::uint32_t len = sizeof(uint64_t) + sizeof(int32_t);
        /* Serialize type and length field */
        const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&len);
        const std::uint8_t* typePtr = reinterpret_cast<std::uint8_t* >(&type);
        bool memcpyResult = true;
        memcpyResult = (!memcpy_s(c, size_, dataLen, sizeof(std::uint32_t))) &&
                        (!memcpy_s(c + sizeof(std::uint32_t),
                            size_ - sizeof(std::uint32_t), typePtr, sizeof(std::uint8_t)));
        if (memcpyResult == false) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer] Memory copy return error, Invalid serialization.";
        }

        /* Serialize ErrorCode field */
        uint64_t valueTempDomain = errorCode_.Domain().Id();
        int32_t valueTempCode = errorCode_.Value();
        const std::uint8_t* dataDomain = reinterpret_cast<const std::uint8_t* >(&valueTempDomain);
        const std::uint8_t* dataCode = reinterpret_cast<const std::uint8_t* >(&valueTempCode);
        /* sizeof(std::uint32_t) is size of length field, sizeof(std::uint8_t) is size of type field */
        pos = sizeof(std::uint32_t) + sizeof(std::uint8_t);
        memcpyResult = (!memcpy_s(c + pos, size_ - pos, dataDomain, sizeof(uint64_t))) &&
                        (!memcpy_s(c + pos + sizeof(uint64_t),
                            size_ - pos - sizeof(uint64_t), dataCode, sizeof(int32_t)));
        if (memcpyResult == false) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer] Memory copy return error, Invalid serialization.";
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
    ~Serializer() = default;
    std::size_t GetSize()
    {
        size_ = sizeof(std::uint32_t) + str_.size();
        return size_;
    }

    void Serialize(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::uint32_t len = static_cast<std::uint32_t>(str_.size());
        const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&len);
        auto memcpySuccess = memcpy_s(c + pos_, size_, dataLen, sizeof(std::uint32_t));
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer String] Memory copy return error, Invalid serialization.";
        }

        const char* data = reinterpret_cast<const char* >(str_.c_str());
        memcpySuccess = memcpy_s(c + pos_ + sizeof(std::uint32_t), size_ - sizeof(std::uint32_t), data, len);
        if (memcpySuccess != 0 && len != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer String] Memory copy return error, Invalid serialization.";
        }
    }

private:
    const ara::core::String& str_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
};

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
    ~Serializer() = default;
    std::size_t GetSize()
    {
        size_ = sizeof(std::uint32_t) + str_.size();
        return size_;
    }

    void Serialize(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::uint32_t len = static_cast<std::uint32_t>(str_.size());
        const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&len);
        auto memcpySuccess = memcpy_s(c + pos_, size_, dataLen, sizeof(std::uint32_t));
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer std::string] Memory copy return error, Invalid serialization.";
        }

        const char* data = reinterpret_cast<const char* >(str_.c_str());
        memcpySuccess = memcpy_s(c + pos_ + sizeof(std::uint32_t), size_ - sizeof(std::uint32_t), data, len);
        if (memcpySuccess != 0 && len != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer std::string] Memory copy return error, Invalid serialization.";
        }
    }

private:
    const std::string& str_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
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
    ~Serializer() = default;
    std::size_t GetSize()
    {
        size_ = GetSizeHelper();
        return size_;
    }

private:
    const value_type& value_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
    void TraverseSerializeArray(std::uint8_t* c)
    {
        std::size_t posCnt = pos_;
        for (std::size_t i = 0; i < N; ++i) {
            Serializer<T> serializer(value_[i], config_, posCnt);
            posCnt += serializer.GetSize();
            serializer.Serialize(c);
        }
    }

    template <typename U = T>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<!is_enumerable<U>::value && std::is_trivially_copyable<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(value_.data());
        std::size_t cpySize = sizeof(T) * N;
        auto memcpySuccess = memcpy_s(c + pos_, size_, data, cpySize);
        if (memcpySuccess != 0 && cpySize != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer Array] Memory copy return error, Invalid serialization.";
        }
    }

    template <typename U = T>
    void SerializeHelper(std::uint8_t* c, typename std::enable_if<is_enumerable<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        if (T::IsPlane()) {
            const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(value_.data());
            std::size_t cpySize = sizeof(T) * N;
            auto memcpySuccess = memcpy_s(c + pos_, size_, data, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[DDS Serializer Array] Memory copy return error, Invalid serialization.";
            }
            return;
        }

        TraverseSerializeArray(c);
    }

    template <typename U = T>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<!is_enumerable<U>::value && !std::is_trivially_copyable<U>::value>::type* = 0)
    {
        TraverseSerializeArray(c);
    }

    std::size_t GetSequenceSize() const
    {
        std::size_t totalSize = 0;
        for (const T& item : value_) {
            Serializer<T> serializer(item, config_);
            totalSize += serializer.GetSize();
        }
        return totalSize;
    }

    template<typename U = T>
    typename std::enable_if<!is_enumerable<U>::value && std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T) * N;
    }

    template<typename U = T>
    typename std::enable_if<is_enumerable<U>::value, std::size_t>::type GetSizeHelper() const
    {
        if (T::IsPlane()) {
            return sizeof(T) * N;
        }
        return GetSequenceSize();
    }

    template<typename U = T>
    typename std::enable_if<!is_enumerable<U>::value && !std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return GetSequenceSize();
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
    ~Serializer() = default;
    std::size_t GetSize() const
    {
        Serializer<typename std::decay<First>::type> firstSerializer(value_.first, config_);
        Serializer<typename std::decay<Second>::type> secondSerializer(value_.second, config_);
        return firstSerializer.GetSize() + secondSerializer.GetSize();
    }

    void Serialize(std::uint8_t* c)
    {
        Serializer<typename std::decay<First>::type> firstSerializer(value_.first, config_, pos_);
        Serializer<typename std::decay<Second>::type> secondSerializer(
            value_.second, config_, pos_ + firstSerializer.GetSize());
        firstSerializer.Serialize(c);
        (void)secondSerializer.GetSize();
        secondSerializer.Serialize(c);
    }

private:
    const value_type& value_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
};

template <typename Sequence>
class SequenceContainerSerializer {
public:
    using value_type = Sequence;

    SequenceContainerSerializer(const Sequence& container,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
        std::size_t pos = 0)
        : container_(container), config_(config), pos_(pos)
    {
    }

    virtual ~SequenceContainerSerializer() {}
    void Serialize(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::uint32_t len = static_cast<std::uint32_t>(container_.size());
        const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(&len);
        auto memcpySuccess = memcpy_s(c + pos_, size_, data, sizeof(std::uint32_t));
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer Sequence] Memory copy return error, Invalid serialization.";
        }
        std::size_t posCnt = sizeof(std::uint32_t);
        for (const typename Sequence::value_type& item : container_) {
            Serializer<typename Sequence::value_type> serializer(item, config_, pos_ + posCnt);
            posCnt += serializer.GetSize();
            serializer.Serialize(c);
        }
    }

    void SetSize(const std::size_t& size)
    {
        size_ = size;
    }

    std::size_t GetSize()
    {
        size_ = sizeof(std::uint32_t);
        for (const typename Sequence::value_type& item : container_) {
            Serializer<typename Sequence::value_type> serializer(item, config_);
            size_ += serializer.GetSize();
        }
        return size_;
    }

private:
    const Sequence& container_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t pos_;
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
};

template <typename K, typename V>
class Serializer<ara::core::Map<K, V>>
    : public SequenceContainerSerializer<ara::core::Map<K, V>> {
public:
    using SequenceContainerSerializer<ara::core::Map<K, V>>::SequenceContainerSerializer;
    ~Serializer() = default;
};

template <typename T>
class Serializer<ara::core::Vector<T>> {
public:
    using value_type = ara::core::Vector<T>;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig (),
               std::size_t pos = 0)
        : container_(value), config_(config), pos_(pos)
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
    const value_type& container_;
    const vrtf::serialize::SerializeConfig config_;
    const std::size_t pos_;
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
    template <typename U = T>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                                !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                                !is_enumerable<U>::value &&
                                std::is_trivially_copyable<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        std::uint32_t len = static_cast<std::uint32_t>(container_.size());
        std::size_t cpySize = sizeof(std::uint32_t);
        const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&len);
        auto memcpySuccess = memcpy_s(c + pos_, size_, dataLen, cpySize);
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer Vector] Memory copy return error, Invalid serialization.";
        }
        cpySize = sizeof(T) * container_.size();
        const std::uint8_t *data = reinterpret_cast<const std::uint8_t* >(container_.data());
        memcpySuccess = memcpy_s(c + pos_ + sizeof(std::uint32_t), size_ - sizeof(std::uint32_t), data, cpySize);
        if (memcpySuccess != 0 && cpySize != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer Vector] Memory copy return error, Invalid serialization.";
        }
    }

    template <typename U = T>
    void SerializeHelper(std::uint8_t* c, typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                                                                  !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                                                                  is_enumerable<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        if (T::IsPlane()) {
            std::uint32_t len = static_cast<std::uint32_t>(container_.size());
            std::size_t cpySize = sizeof(std::uint32_t);
            const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&len);
            auto memcpySuccess = memcpy_s(c + pos_, size_, dataLen, cpySize);
            if (memcpySuccess != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[DDS Serializer Vector] Memory copy return error, Invalid serialization.";
            }

            cpySize = sizeof(T) * container_.size();
            const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(container_.data());
            memcpySuccess = memcpy_s(c + pos_ + sizeof(std::uint32_t), size_ - sizeof(std::uint32_t), data, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[DDS Serializer Vector] Memory copy return error, Invalid serialization.";
            }
            return;
        }

        SequenceContainerSerializer<ara::core::Vector<T>> serializer(container_, config_, pos_);
        serializer.SetSize(size_);
        serializer.Serialize(c);
    }

    template <typename U = T>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                                !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                                !is_enumerable<U>::value &&
                                !std::is_trivially_copyable<U>::value>::type* = 0)
    {
        SequenceContainerSerializer<ara::core::Vector<T>> serializer(container_, config_, pos_);
        serializer.SetSize(size_);
        serializer.Serialize(c);
    }

    template <typename U = T>
    void SerializeHelper(std::uint8_t* c,
        typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value ||
                                vrtf::serialize::ros::IsRosBuiltinMsg<U>::value>::type* = 0)
    {
        using namespace ara::godel::common::log;
        if (config_.type == vrtf::serialize::SerializationType::CM && std::is_trivially_copyable<U>::value) {
            std::uint32_t len = static_cast<std::uint32_t>(container_.size());
            std::size_t cpySize = sizeof(std::uint32_t);
            const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&len);
            auto memcpySuccess = memcpy_s(c + pos_, size_, dataLen, cpySize);
            if (memcpySuccess != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[DDS Serializer Vector] Memory copy return error, Invalid serialization.";
            }

            cpySize = sizeof(T) * container_.size();
            const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(container_.data());
            memcpySuccess = memcpy_s(c + pos_ + sizeof(std::uint32_t), size_ - sizeof(std::uint32_t), data, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[DDS Serializer Vector] Memory copy return error, Invalid serialization.";
            }
            return;
        }
        SequenceContainerSerializer<ara::core::Vector<T>> serializer(container_, config_, pos_);
        serializer.SetSize(size_);
        serializer.Serialize(c);
    }

    std::size_t GetSequenceSize() const
    {
        std::size_t totalSize = 0;
        for (const T& item : container_) {
            Serializer<T> serializer(item, config_);
            totalSize += serializer.GetSize();
        }
        return totalSize + sizeof(std::uint32_t);
    }

    template<typename U = T>
    typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                            !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                            !is_enumerable<U>::value &&
                            std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T) * container_.size() + sizeof(std::uint32_t);
    }

    template<typename U = T>
    typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                            !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                            is_enumerable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        if (T::IsPlane()) {
            return sizeof(T) * container_.size() + sizeof(std::uint32_t);
        }
        return GetSequenceSize();
    }

    template<typename U = T>
    typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                            !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                            !is_enumerable<U>::value &&
                            !std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return GetSequenceSize();
    }

    template<typename U = T>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value ||
                            vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        if (config_.type == vrtf::serialize::SerializationType::CM && std::is_trivially_copyable<U>::value) {
            return sizeof(T) * container_.size() + sizeof(std::uint32_t);
        }
        return GetSequenceSize();
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
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
    void SerializeHelper(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::uint32_t len = static_cast<std::uint32_t>(container_.size());
        std::size_t cpySize = sizeof(std::uint32_t);
        const std::uint8_t* dataLen = reinterpret_cast<const std::uint8_t* >(&len);
        auto memcpySuccess = memcpy_s(c + pos_, size_, dataLen, cpySize);
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer Vector<bool>] Memory copy return error, Invalid serialization.";
        }
        cpySize = sizeof(bool);
        for (std::size_t i = 0; i < len; i++) {
            bool valueTmp = container_[i];
            const std::uint8_t *dataPtrTmp = reinterpret_cast<const std::uint8_t* >(&valueTmp);
            memcpySuccess = memcpy_s(c + pos_ + sizeof(std::uint32_t), size_ - sizeof(std::uint32_t) - cpySize * i,
                dataPtrTmp, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error()<< "[DDS Serializer Vector<bool>] Memory copy return error, Invalid serialization.";
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
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
    void SerializeHelper(std::uint8_t* c)
    {
        using namespace ara::godel::common::log;
        std::size_t cpySize = sizeof(std::uint8_t) * container_.size();
        const std::uint8_t* data = reinterpret_cast<const std::uint8_t* >(container_.data());
        auto memcpySuccess = memcpy_s(c, cpySize, data, cpySize);
        if (memcpySuccess != 0 && cpySize != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Serializer Vector] Memory copy return error, Invalid serialization.";
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
    std::size_t size_ = MAX_DDS_SERIALIZE_SIZE;
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

// use for support build
template <typename T>
class Serializer<std::shared_ptr<T>> {
public:
    using value_type = std::shared_ptr<T>;

    Serializer(const value_type& value,
               vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig(),
               std::size_t pos = 0) {}
    ~Serializer() = default;
    void Serialize(std::uint8_t* c) {}

    std::size_t GetSize()
    {
        // if dds use shared_ptr, return invalid size
        return MAX_DDS_SERIALIZE_SIZE;
    }
};

template <typename T>
class Deserializer;

template <typename Sequence>
class SequenceContainerDeserializer {
public:
    using result_type = Sequence;

    SequenceContainerDeserializer(const std::uint8_t* data, std::size_t size, std::size_t len,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), len_(len), config_(config)
    {
    }
    ~SequenceContainerDeserializer() = default;
    result_type GetValue() const
    {
        using namespace ara::godel::common::log;
        result_type result;
        const std::uint8_t* current_pos = data_ + sizeof(std::uint32_t);
        std::size_t remaining_size = size_ - sizeof(std::uint32_t);
        for (std::size_t cur_item = 0; cur_item < len_; ++cur_item) {
            Deserializer<typename Sequence::value_type> item_deserializer(current_pos, remaining_size, config_);
            std::size_t s = item_deserializer.GetSize();
            if (remaining_size >= s) {
                current_pos += s;
                remaining_size -= s;
                result.push_back(item_deserializer.GetValue());
            } else {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "GetValue(): Deserialization of sequence container failed, insufficient data.";
            }
        }

        return result;
    }

    std::size_t GetSize() const
    {
        using namespace ara::godel::common::log;
        std::size_t result = sizeof(std::uint32_t);
        const std::uint8_t* current_pos = data_ + sizeof(std::uint32_t);
        std::size_t remaining_size = size_ - sizeof(std::uint32_t);
        for (std::size_t cur_item = 0; cur_item < len_; ++cur_item) {
            Deserializer<typename Sequence::value_type> item_deserializer(current_pos, remaining_size, config_);
            std::size_t s = item_deserializer.GetSize();
            if (remaining_size >= s) {
                current_pos += s;
                remaining_size -= s;
                result += s;
            } else {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "GetSize():Deserialization of sequence container failed, insufficient data.";
                return MAX_DDS_SERIALIZE_SIZE;
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
    const std::uint8_t* data_ = nullptr;
    std::size_t size_ = 0;
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

/**
 * @brief Template specialization for ara::core::Vector<bool>.
 */
template<>
class Deserializer<ara::core::Vector<bool>> {
public:
    using result_type = ara::core::Vector<bool>;

    /**
     * @brief Creates a deserializer using the given payload.The constructor will initialize the len_ with
     *        MAX_DDS_SERIALIZE_SIZE, if the given size is too short to hold the container, or the len_ will be
     *        intialized with the container's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), len_(), config_(config)
        {
            // Inialize len_, if initialize failed, len_ equals to MAX_DDS_SERIALIZE_SIZE
            if (size_ >= sizeof(std::uint32_t)) {
                len_ = *reinterpret_cast<const std::uint32_t* >(data_);
            } else {
                len_ = MAX_DDS_SERIALIZE_SIZE;
            }
        }

    ~Deserializer() = default;
    result_type GetValue() const
    {
        return GetValueHelper();
    }

    /**
     * @brief Returns the number of bytes the container and its contents occupy.
     * @details This method may return MAX_DDS_SERIALIZE_SIZE if the length of the container (or its contents) does not
     *          fit the array size.
     * \return Number of bytes the container occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        std::size_t sizeTmp = MAX_DDS_SERIALIZE_SIZE;
        // Check if len_ is initialized successfully
        if (len_ < MAX_DDS_SERIALIZE_SIZE) {
            sizeTmp = GetSizeHelper();
            if (sizeTmp <= size_) {
                return sizeTmp;
            }
        }
        std::shared_ptr<Log> logInstance = Log::GetLog("CM");
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->error() << "Deserialization of Vector<bool> failed, insufficient data.";
        return MAX_DDS_SERIALIZE_SIZE;
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
        for (std::size_t i = 0; i < len_; i++) {
            bool valueTmp;
            auto memcpySuccess = memcpy_s(reinterpret_cast<std::uint8_t* >(&valueTmp),
                cpySize, data_ + sizeof(std::uint32_t) + (i * sizeof(bool)), cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[DDS Deserializer Vector] Memory copy return error, Invalid deserialization.";
                break;
            }
            result.push_back(valueTmp);
        }
        return result;
    }

    std::size_t GetSizeHelper() const
    {
        return sizeof(bool) * len_ + sizeof(std::uint32_t);
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
     * @brief Creates a deserializer using the given payload.
     * @details The constructor will initialize the len_ with MAX_DDS_SERIALIZE_SIZE, if the given size is too short
     *          to hold the container, or the len_ will be intialized with the container's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data,
                 std::size_t size,
                 vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), len_(), config_(config)
    {
        // Inialize len_, if initialize failed, len_ equals to MAX_DDS_SERIALIZE_SIZE
        if (size_ >= sizeof(std::uint32_t)) {
            len_ = *reinterpret_cast<const std::uint32_t* >(data_);
        } else {
            len_ = MAX_DDS_SERIALIZE_SIZE;
        }
    }
    ~Deserializer() = default;
    result_type GetValue() const
    {
        return GetValueHelper();
    }

    /**
     * @brief Returns the number of bytes the container and its contents occupy.
     * @details This method may return MAX_DDS_SERIALIZE_SIZE if the length of the container (or its contents) does not
     *          fit the array size.
     * \return Number of bytes the container occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        std::size_t sizeTmp = MAX_DDS_SERIALIZE_SIZE;
        // Check if len_ is initialized successfully
        if (len_ < MAX_DDS_SERIALIZE_SIZE) {
            sizeTmp = GetSizeHelper();
            if (sizeTmp <= size_) {
                return sizeTmp;
            }
        }
        std::shared_ptr<Log> logInstance = Log::GetLog("CM");
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->verbose() << "Deserialization of Vector failed, insufficient data.";
        return MAX_DDS_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t len_;
    const vrtf::serialize::SerializeConfig config_;
    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                                                       !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                                                       !is_enumerable<U>::value &&
                                                       std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        using namespace ara::godel::common::log;
        result_type result;
        U *value_ptr = reinterpret_cast<U *>(const_cast<uint8_t *>(data_) + sizeof(std::uint32_t));
        result.reserve(len_);
        result.insert(result.begin(), value_ptr, value_ptr + len_);
        return result;
    }

    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                                                       !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                                                       is_enumerable<U>::value>::type* = 0) const
    {
        using namespace ara::godel::common::log;
        if (T::IsPlane()) {
            result_type result;
            U *value_ptr = reinterpret_cast<U *>(const_cast<uint8_t *>(data_) + sizeof(std::uint32_t));
            result.reserve(len_);
            result.insert(result.begin(), value_ptr, value_ptr + len_);
            return result;
        }
        return SequenceContainerDeserializer<ara::core::Vector<T>>(data_, size_, len_, config_).GetValue();
    }

    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                                                       !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                                                       !is_enumerable<U>::value &&
                                                       !std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        return SequenceContainerDeserializer<ara::core::Vector<T>>(data_, size_, len_, config_).GetValue();
    }

    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value ||
        vrtf::serialize::ros::IsRosBuiltinMsg<U>::value>::type* = 0) const
    {
        if (config_.type == vrtf::serialize::SerializationType::CM && std::is_trivially_copyable<U>::value) {
            result_type result;
            U *value_ptr = reinterpret_cast<U *>(const_cast<uint8_t *>(data_) + sizeof(std::uint32_t));
            result.reserve(len_);
            result.insert(result.begin(), value_ptr, value_ptr + len_);
            return result;
        }
        return SequenceContainerDeserializer<ara::core::Vector<T>>(data_, size_, len_, config_).GetValue();
    }

    template<typename U = T>
    typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                            !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                            !is_enumerable<U>::value &&
                            std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T) * len_ + sizeof(std::uint32_t);
    }

    template<typename U = T>
    typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                            !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                            is_enumerable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        if (T::IsPlane()) {
            return sizeof(T) * len_ + sizeof(std::uint32_t);
        }
        return SequenceContainerDeserializer<ara::core::Vector<T>>(data_, size_, len_, config_).GetSize();
    }

    template<typename U = T>
    typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
                            !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value &&
                            !is_enumerable<U>::value &&
                            !std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return SequenceContainerDeserializer<ara::core::Vector<T>>(data_, size_, len_, config_).GetSize();
    }

    template<typename U = T>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value ||
                            vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        if (config_.type == vrtf::serialize::SerializationType::CM && std::is_trivially_copyable<U>::value) {
            return sizeof(T) * len_ + sizeof(std::uint32_t);
        }
        return SequenceContainerDeserializer<ara::core::Vector<T>>(data_, size_, len_, config_).GetSize();
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
    ~Deserializer() = default;
    std::size_t GetSize() const
    {
        return GetSizeHelper();
    }

    result_type GetValue() const
    {
        return GetValueHelper();
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
    result_type TraverseDeserializeArray() const
    {
        result_type result;

        std::size_t pos = 0;
        for (std::size_t i = 0; i < N; ++i) {
            Deserializer<T> deserializer(data_ + pos, size_ - pos, config_);
            pos += deserializer.GetSize();
            result[i] = deserializer.GetValue();
        }

        return result;
    }

    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<!is_enumerable<U>::value
        && std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        using namespace ara::godel::common::log;
        result_type result;
        std::size_t cpySize = N * sizeof(T);
        auto memcpySuccess = memcpy_s(result.data(), cpySize, data_, cpySize);
        if (memcpySuccess != 0 && cpySize != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Deserializer Array] Memory copy return error, Invalid deserialization.";
        }
        return result;
    }

    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<is_enumerable<U>::value>::type* = 0) const
    {
        using namespace ara::godel::common::log;
        if (T::IsPlane()) {
            result_type result;
            std::size_t cpySize = N * sizeof(T);
            auto memcpySuccess = memcpy_s(result.data(), cpySize, data_, cpySize);
            if (memcpySuccess != 0 && cpySize != 0) {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "[DDS Deserializer Array] Memory copy return error, Invalid deserialization.";
            }
            return result;
        }
        return TraverseDeserializeArray();
    }

    template <typename U = T>
    result_type GetValueHelper(typename std::enable_if<!is_enumerable<U>::value
        && !std::is_trivially_copyable<U>::value>::type* = 0) const
    {
        return TraverseDeserializeArray();
    }

    std::size_t GetArraySize() const
    {
        std::size_t pos = 0;
        for (std::size_t i = 0; i < N; ++i) {
            Deserializer<T> deserializer(data_ + pos, size_ - pos, config_);
            std::size_t sizeTmp = deserializer.GetSize();
            if (sizeTmp == MAX_DDS_SERIALIZE_SIZE) {
                return MAX_DDS_SERIALIZE_SIZE;
            } else {
                pos += sizeTmp;
            }
        }

        return pos;
    }

    template<typename U = T>
    typename std::enable_if<!is_enumerable<U>::value && std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T) * N;
    }

    template<typename U = T>
    typename std::enable_if<is_enumerable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        if (T::IsPlane()) {
            return sizeof(T) * N;
        }
        return GetArraySize();
    }

    template<typename U = T>
    typename std::enable_if<!is_enumerable<U>::value && !std::is_trivially_copyable<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return GetArraySize();
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
    ~Deserializer() = default;
    result_type GetValue() const
    {
        using namespace ara::godel::common::log;
        Deserializer<typename std::decay<First>::type> first_deserializer(data_, size_, config_);
        std::size_t first_size = first_deserializer.GetSize();
        if (first_size < size_) {
            Deserializer<typename std::decay<Second>::type> second_deserializer(
                data_ + first_size, size_ - first_size, config_);
            std::size_t secondSize = second_deserializer.GetSize();
            if (secondSize != MAX_DDS_SERIALIZE_SIZE) {
                return result_type(first_deserializer.GetValue(), second_deserializer.GetValue());
            }
            return result_type();
        } else {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Deserialization of pair failed, insufficient data.";
            return result_type();
        }
    }

    std::size_t GetSize() const
    {
        using namespace ara::godel::common::log;
        Deserializer<typename std::decay<First>::type> first_deserializer(data_, size_, config_);
        std::size_t firstSize = first_deserializer.GetSize();
        if (firstSize != MAX_DDS_SERIALIZE_SIZE && firstSize < size_) {
            Deserializer<typename std::decay<Second>::type> second_deserializer(
                data_ + firstSize, size_ - firstSize, config_);
            std::size_t secondSize = second_deserializer.GetSize();
            if (secondSize != MAX_DDS_SERIALIZE_SIZE) {
                return firstSize + secondSize;
            } else {
                std::shared_ptr<Log> logInstance = Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "Deserialization of pair failed, insufficient second data.";
                return MAX_DDS_SERIALIZE_SIZE;
            }
        } else {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Deserialization of pair failed, insufficient first data.";
            return MAX_DDS_SERIALIZE_SIZE;
        }
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
};

template <typename AssociativeContainer>
class AssociativeContainerDeserializer {
public:
    using result_type = AssociativeContainer;

    /**
     * @brief Creates a deserializer using the given payload.
     * @details The constructor will initialize the len_ with MAX_DDS_SERIALIZE_SIZE, if the given size is too short to
     *          hold the container, or the len_ will be intialized with the container's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    AssociativeContainerDeserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), len_(), config_(config)
    {
        // Inialize len_, if initialize failed, len_ equals to MAX_DDS_SERIALIZE_SIZE
        if (size_ >= sizeof(std::uint32_t)) {
            len_ = *reinterpret_cast<const std::uint32_t* >(data_);
            data_ += sizeof(std::uint32_t);
            size_ -= sizeof(std::uint32_t);
        } else {
            len_ = MAX_DDS_SERIALIZE_SIZE;
        }
    }

    virtual ~AssociativeContainerDeserializer() {}
    result_type GetValue() const
    {
        result_type result;

        const std::uint8_t* pos = data_;
        std::size_t remaining = size_;

        for (std::size_t entry = 0; entry < len_; ++entry) {
            Deserializer<typename AssociativeContainer::value_type> key_deserializer(pos, remaining, config_);
            std::size_t sizeTmp = key_deserializer.GetSize();

            result.insert(key_deserializer.GetValue());
            pos += sizeTmp;
            remaining -= sizeTmp;
        }

        return result;
    }

    /**
     * @brief Returns the number of bytes the AssociativeContainer and its contents occupy.
     * @details This method may return MAX_DDS_SERIALIZE_SIZE if the length of the container (or its contents) does not
     *          fit the array size.
     * \return Number of bytes the AssociativeContainer occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        std::size_t result_size = sizeof(std::uint32_t);
        // Check if len_ is initialized successfully
        if (len_ >= MAX_DDS_SERIALIZE_SIZE) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->verbose() << "Deserialization of associative container failed, insufficient data.";
            return MAX_DDS_SERIALIZE_SIZE;
        }

        const std::uint8_t* pos = data_;
        std::size_t remaining = size_;

        for (std::size_t entry = 0; entry < len_; ++entry) {
            Deserializer<typename AssociativeContainer::value_type> key_deserializer(pos, remaining, config_);

            std::size_t size_entry = key_deserializer.GetSize();
            if (size_entry != MAX_DDS_SERIALIZE_SIZE) {
                pos += size_entry;
                remaining -= size_entry;
                result_size += size_entry;
            } else {
                return MAX_DDS_SERIALIZE_SIZE;
            }
        }

        return result_size;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t len_;
    const vrtf::serialize::SerializeConfig config_;
};

template <typename K, typename V>
class Deserializer<ara::core::Map<K, V>>
    : public AssociativeContainerDeserializer<ara::core::Map<K, V>> {
public:
    using AssociativeContainerDeserializer<ara::core::Map<K, V>>::AssociativeContainerDeserializer;
};

template <>
class Deserializer<ara::core::String> {
public:
    using result_type = ara::core::String;
    using char_type = ara::core::String::value_type;

    /**
     * @brief Creates a deserializer using the given payload.
     * @details The constructor will initialize the len_ with MAX_DDS_SERIALIZE_SIZE, if the given size is too short
     *          to hold the string, or the len_ will be intialized with the string's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
        // Inialize len_, if initialize failed, len_ equals to MAX_DDS_SERIALIZE_SIZE
        if (size_ >= sizeof(std::uint32_t)) {
            len_ = *reinterpret_cast<const std::uint32_t* >(data_);
        } else {
            len_ = MAX_DDS_SERIALIZE_SIZE;
        }
    }
    ~Deserializer() = default;
    result_type GetValue() const
    {
        const char_type* c = reinterpret_cast<const char_type* >(data_ + sizeof(std::uint32_t));
        return {c, len_};
    }

    /**
     * @brief Returns the number of bytes the string occupy.
     * @details This method may return MAX_DDS_SERIALIZE_SIZE if the length of the string (or its contents) does not
     *          fit the given size.
     * \return Number of bytes the string occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        // Check if len_ is initialized successfully
        std::size_t sizeTmp = MAX_DDS_SERIALIZE_SIZE;
        if (len_ < MAX_DDS_SERIALIZE_SIZE) {
            sizeTmp = sizeof(std::uint32_t) + len_ * sizeof(char_type);
            if (sizeTmp <= size_) {
                return sizeTmp;
            }
        }
        std::shared_ptr<Log> logInstance = Log::GetLog("CM");
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->error() << "Deserialization of String failed, insufficient data.";
        return MAX_DDS_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t len_ = 0;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
};

template <>
class Deserializer<std::string> {
public:
    using result_type = std::string;
    using char_type = std::string::value_type;

    /**
     * @brief Creates a deserializer using the given payload.
     * @details The constructor will initialize the len_ with MAX_DDS_SERIALIZE_SIZE, if the given size is too short
     *          to hold the string, or the len_ will be intialized with the string's size.
     * @param[in] data start of the payload address.
     * @param[in] size size of payload.
     */
    Deserializer(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
        // Inialize len_, if initialize failed, len_ equals to MAX_DDS_SERIALIZE_SIZE
        if (size_ >= sizeof(std::uint32_t)) {
            len_ = *reinterpret_cast<const std::uint32_t* >(data_);
        } else {
            len_ = MAX_DDS_SERIALIZE_SIZE;
        }
    }
    ~Deserializer() = default;
    result_type GetValue() const
    {
        const char_type* c = reinterpret_cast<const char_type* >(data_ + sizeof(std::uint32_t));
        return {c, len_};
    }

    /**
     * @brief Returns the number of bytes the string occupy.
     * @details This method may return MAX_DDS_SERIALIZE_SIZE if the length of the string (or its contents) does not
     *          fit the given size.
     * \return Number of bytes the string occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        // Check if len_ is initialized successfully
        std::size_t sizeTmp = MAX_DDS_SERIALIZE_SIZE;
        if (len_ < MAX_DDS_SERIALIZE_SIZE) {
            sizeTmp = sizeof(std::uint32_t) + len_ * sizeof(char_type);
            if (sizeTmp <= size_) {
                return sizeTmp;
            }
        }
        std::shared_ptr<Log> logInstance = Log::GetLog("CM");
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->error() << "Deserialization of std::string failed, insufficient data.";
        return MAX_DDS_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t len_ = 0;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
};

/**
 * @brief Specialized template of Deserializer with ara::core::ErrorCode
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
     * @brief Returns the number of bytes the ara::core::ErrorCode.
     * @details This method may return MAX_DDS_SERIALIZE_SIZE if the length of the string (or its contents) does not
     *          fit the given size.
     * @return Number of bytes the string occupies.
     */
    std::size_t GetSize()
    {
        using namespace ara::godel::common::log;
        std::uint8_t type = 0x00;
        auto memcpySuccess = memcpy_s(&type, sizeof(uint8_t), data_ + sizeof(uint32_t), sizeof(uint8_t));
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Deserializer ErrorCode] Memory copy return error, Invalid deserialization.";
        }
        if (type != 0x01) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Construct deserialization for ErrorCode failed, invalid type of ErrorCode!";
            return  MAX_DDS_SERIALIZE_SIZE;
        }
        if (size_ >= (sizeof(std::uint8_t) + sizeof(std::uint32_t))) {
            std::size_t sizeTmp = sizeof(std::uint8_t) + sizeof(std::uint32_t) +
                                    sizeof(std::uint64_t) + sizeof(std::int32_t);
            if (sizeTmp == size_) {
                return sizeTmp;
            }
        }
        std::shared_ptr<Log> logInstance = Log::GetLog("CM");
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance->error() << "Construct deserialization for ErrorCode failed, invalid size parameter!";
        return  MAX_DDS_SERIALIZE_SIZE;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
    /**
     * @brief Return ErrorCode with vrtf::vcc::api::types::MethodError.
     * @details Get domainValue and errorCode from data_, and use them make a vrtf::vcc::api::types::MethodError.
     * @return a struct with domainValue and errorCode.
     */
    vrtf::vcc::api::types::MethodError GetValueHelper() const
    {
        using namespace ara::godel::common::log;
        std::uint64_t errorDomain = 0x00000000;
        auto memcpySuccess = memcpy_s(&errorDomain, sizeof(uint64_t),
            data_ + sizeof(uint32_t) + sizeof(uint8_t), sizeof(uint64_t));
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Deserializer ErrorCode] Memory copy return error, Invalid deserialization.";
        }
        auto valueDomian = *reinterpret_cast<const uint64_t* >(&errorDomain);

        std::int32_t errorCodeValue = 0x0000;
        memcpySuccess = memcpy_s(&errorCodeValue, sizeof(int32_t),
            data_ + sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint64_t), sizeof(int32_t));
        if (memcpySuccess != 0) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[DDS Deserializer ErrorCode] Memory copy return error, Invalid deserialization.";
        }
        auto valueCodeValue = *reinterpret_cast<const int32_t* >(&errorCodeValue);

        vrtf::vcc::api::types::MethodError errorData;
        errorData.domainValue = valueDomian;
        errorData.errorCode = valueCodeValue;
        return errorData;
    }
};

class DeserializeSizeCounter {
public:
    DeserializeSizeCounter(const std::uint8_t* data, std::size_t buf_size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : buf_size_(buf_size), data_(data), config_(config)
    {
    }
    ~DeserializeSizeCounter() = default;
    template <typename T>
    void operator()(const T&)
    {
        using namespace ara::godel::common::log;
        Deserializer<T> deserializer(data_, buf_size_, config_);
        std::size_t size = deserializer.GetSize();
        if (size != MAX_DDS_SERIALIZE_SIZE && size_ != MAX_DDS_SERIALIZE_SIZE && buf_size_ >= size) {
            size_ += size;
            data_ += size;
            buf_size_ -= size;
        } else {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Deserialization[DDS] size counter failed, insufficient data.";
            size_ = MAX_DDS_SERIALIZE_SIZE;
        }
    }

    std::size_t GetSize() const
    {
        return size_;
    }

private:
    std::size_t size_ {0};
    std::size_t buf_size_;
    const std::uint8_t* data_;
    const vrtf::serialize::SerializeConfig config_;
};

class DeserializingEnumerator {
public:
    DeserializingEnumerator(const std::uint8_t* data, std::size_t size,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : data_(data), size_(size), config_(config)
    {
    }
    ~DeserializingEnumerator() = default;
    template <typename T>
    void operator()(T& value)
    {
        using namespace ara::godel::common::log;
        Deserializer<T> deserializer(data_ + pos_, size_ - pos_, config_);
        pos_ += deserializer.GetSize();
        if (pos_ > size_) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Insufficient arguments from remote end";
        } else {
            value = deserializer.GetValue();
        }
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    const vrtf::serialize::SerializeConfig config_;
    std::size_t pos_ = 0;
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
                 vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ()) {}
    ~Deserializer() = default;

    result_type GetValue() const
    {
        return nullptr;;
    }
    std::size_t GetSize()
    {
        return MAX_DDS_SERIALIZE_SIZE;
    }
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
        static_assert(IsSerializable<value_type>(), "No appropriate marshalling defined for this type!");
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
        typename std::enable_if<!is_enumerable<U>::value
        && std::is_trivially_copyable<U>::value
        && !vrtf::serialize::ros::IsRosMsg<U>::value
        && !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value>::type* = 0) const
    {
        using namespace ara::godel::common::log;
        if (sizeof(value_type) > size_) {
            std::shared_ptr<Log> logInstance = Log::GetLog("CM");
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "Deserialization of trivially copyable type failed, insufficient data!";
        }

        return *reinterpret_cast<const value_type* >(data_);
    }

    template <typename U = value_type>
    value_type GetValueHelper(typename std::enable_if<is_enumerable<U>::value>::type* = 0) const
    {
        if (value_type::IsPlane() &&
            config_.structPolicy == vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT) {
            return *reinterpret_cast<const value_type* >(data_);
        }
        value_type result = value_type();
        vrtf::serialize::SerializeConfig config = config_;
        if (config_.structPolicy == vrtf::serialize::StructSerializationPolicy::DISABLE_OUT_LAYER_ALIGNMENT) {
            config.structPolicy = vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT;
        }
        DeserializingEnumerator deserializer(data_, size_, config);
        result.enumerate(deserializer);
        return result;
    }

    template <typename U = value_type>
    value_type GetValueHelper(typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value>::type* = 0) const
    {
        value_type result = value_type();
        if (std::is_trivially_copyable<U>::value && (config_.type == vrtf::serialize::SerializationType::CM) &&
            config_.structPolicy == vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT) {
            return *reinterpret_cast<const value_type* >(data_);
        }
        vrtf::serialize::SerializeConfig config = config_;
        if (config_.structPolicy == vrtf::serialize::StructSerializationPolicy::DISABLE_OUT_LAYER_ALIGNMENT) {
            config.structPolicy = vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT;
        }
        vrtf::serialize::ros::IStream<vrtf::serialize::dds::Deserializer> stream(data_, size_, config);
        ::ros::serialization::Serializer<U>::template allInOne<
            vrtf::serialize::ros::IStream<vrtf::serialize::dds::Deserializer>, U&>(stream, result);
        return result;
    }

    template <typename U = value_type>
    value_type GetValueHelper(
        typename std::enable_if<vrtf::serialize::ros::IsRosBuiltinMsg<U>::value>::type* = 0) const
    {
        value_type result = value_type();
        using istreamType = vrtf::serialize::ros::IStream<vrtf::serialize::dds::Deserializer>;
        std::shared_ptr<istreamType> stream =
            std::make_shared<istreamType>(const_cast<uint8_t *>(data_), size_, config_);
        ::ros::serialization::Serializer<U>::read(*stream, result);
        return result;
    }

    template <typename U = value_type>
    typename std::enable_if<!is_enumerable<U>::value
        && std::is_trivially_copyable<U>::value
        && !vrtf::serialize::ros::IsRosMsg<U>::value
        && !vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type
    GetSizeHelper() const
    {
        return sizeof(T);
    }

    template <typename U = value_type>
    typename std::enable_if<is_enumerable<U>::value, std::size_t>::type GetSizeHelper() const
    {
        if (value_type::IsPlane() &&
            config_.structPolicy == vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT) {
            return sizeof(T);
        }
        vrtf::serialize::SerializeConfig config = config_;
        if (config_.structPolicy == vrtf::serialize::StructSerializationPolicy::DISABLE_OUT_LAYER_ALIGNMENT) {
            config.structPolicy = vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT;
        }
        DeserializeSizeCounter size_counter(data_, size_, config);
        value_type* x = nullptr;
        x->enumerate(size_counter);
        return size_counter.GetSize();
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value, std::size_t>::type GetSizeHelper() const
    {
        if (std::is_trivially_copyable<U>::value && (config_.type == vrtf::serialize::SerializationType::CM) &&
            config_.structPolicy == vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT) {
            return sizeof(T);
        }
        vrtf::serialize::SerializeConfig config = config_;
        if (config_.structPolicy == vrtf::serialize::StructSerializationPolicy::DISABLE_OUT_LAYER_ALIGNMENT) {
            config.structPolicy = vrtf::serialize::StructSerializationPolicy::OUT_LAYER_ALIGNMENT;
        }
        vrtf::serialize::ros::ISizeStream<vrtf::serialize::dds::Deserializer> stream(data_, size_, config);
        ::ros::serialization::Serializer<U>::template allInOne<
            vrtf::serialize::ros::ISizeStream<vrtf::serialize::dds::Deserializer>, U>(stream, value_type());
        return stream.GetSize();
    }

    template <typename U = value_type>
    typename std::enable_if<vrtf::serialize::ros::IsRosBuiltinMsg<U>::value, std::size_t>::type GetSizeHelper() const
    {
        value_type value = value_type();
        return ::ros::serialization::Serializer<U>::serializedLength(value);
    }
};
} // dds
} // serialize
} // vrtf

#endif
