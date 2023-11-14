/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: MethodParamDeserializer in vcc
 * Create: 2019-11-19
 */
#ifndef VRTF_VCC_DRIVER_METHOD_PARAMS_DESERLIZE_H
#define VRTF_VCC_DRIVER_METHOD_PARAMS_DESERLIZE_H
#include "vrtf/vcc/api/types.h"
#include "vrtf/vcc/serialize/dds_serialize.h"
#include "vrtf/vcc/serialize/someip_serialize.h"
#include "ara/hwcommon/log/log.h"
#include <type_traits>
#include <tuple>
namespace vrtf {
namespace vcc {
namespace serialize {
/// \brief Method parameters deserialization
template<class... Args>
class ParamsDeserializer {
public:
    template<std::size_t I, class... AArgs>
    class Parameters;

    template<std::size_t I, class Head, class... PArgs>
    class Parameters<I, Head, PArgs...> {
    public:
        using Type = typename Parameters<I - 1, PArgs...>::Type;
    };

    template<class Head, class... PArgs>
    class Parameters<0, Head, PArgs...> {
    public:
        using Type = Head;
    };

    template<class Head>
    class Parameters<0, Head> {
    public:
        using Type = Head;
    };

    /// \brief ArgType<I> is used to get the Ith parameter's type
    template<std::size_t I>
    using ArgType = typename Parameters<I, Args...>::Type;
    ParamsDeserializer(const vrtf::vcc::api::types::MethodMsg& msg,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : msg_(msg),
          currentPos_(msg.GetPayload()),
          remainingSize_(msg.GetSize()),
          serializeType_(msg.GetSerializeType()),
          serializeConfig_(config)
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
    }
    ~ParamsDeserializer() = default;

    /**
     * @brief Get the Ith parameter
     * @details before this method, IndexMethodParameter must have been called
     * @return The Ith parameter
     */
    template<std::size_t I>
    typename std::decay<ArgType<I>>::type GetValue()
    {
        if (serializeType_ == vrtf::serialize::SerializeType::SHM) {
            vrtf::serialize::dds::Deserializer<typename std::decay<ArgType<I>>::type> deserializer(
                posIndex_[I], argsSize_[I], serializeConfig_);
            typename std::decay<ArgType<I>>::type tmp = deserializer.GetValue();
            return tmp;
        } else if (serializeType_ == vrtf::serialize::SerializeType::SOMEIP) {
            vrtf::serialize::someip::Deserializer<typename std::decay<ArgType<I>>::type> deserializer(
                posIndex_[I], argsSize_[I], serializeConfig_);
            typename std::decay<ArgType<I>>::type tmp = deserializer.GetValue();
            return tmp;
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() <<  "Wrong serialize type, SHM deserialize type used!";
            vrtf::serialize::dds::Deserializer<typename std::decay<ArgType<I>>::type> deserializer(
                posIndex_[I], argsSize_[I], serializeConfig_);
            typename std::decay<ArgType<I>>::type tmp = deserializer.GetValue();
            return tmp;
        }
    }

    /**
     * @brief Initializes posIndex_ with the starting position of each argument in the given payload.
     *        Initializes argsSize_ with the size of each argument.
     * @details During this process the deserialization validation will be checked.
     * @param[in] Head the first parameter in recursive template.
     * @param[in] Tail the other parameters in recursive template.
     * @return true if the payload can be deserialized successfully, or false.
     */
    template<typename Head, typename... Tail>
    bool IndexMethodParameter(Head head, Tail... args)
    {
        std::size_t size = GetArgSize(head);
        if (size > remainingSize_) {
            return false;
        } else {
            posIndex_.push_back(currentPos_);
            argsSize_.push_back(size);
            currentPos_ += size;
            remainingSize_ -= size;
            return IndexMethodParameter(args...);
        }
    }

    template<typename Head>
    bool IndexMethodParameter(Head head)
    {
        std::size_t size = GetArgSize(head);
        if (size > remainingSize_) {
            return false;
        } else {
            posIndex_.push_back(currentPos_);
            argsSize_.push_back(size);
            return true;
        }
    }

    bool IndexMethodParameter()
    {
        return true;
    }

private:
    /**
     * @brief Get the size of Ith parameter.
     * @param[in] arg the Ith parameter.
     * @return the size of Ith parameter.
     */
    template<typename T>
    std::size_t GetArgSize(const T& arg)
    {
        if (serializeType_ == vrtf::serialize::SerializeType::SHM) {
            vrtf::serialize::dds::Deserializer<typename std::decay<T>::type> deserializer(
                currentPos_, remainingSize_, serializeConfig_);
            return deserializer.GetSize();
        } else if (serializeType_ == vrtf::serialize::SerializeType::SOMEIP) {
            vrtf::serialize::someip::Deserializer<typename std::decay<T>::type> deserializer(
                currentPos_, remainingSize_, serializeConfig_);
            return deserializer.GetSize();
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() <<  "Wrong serialize type, SHM deserialize type used!";
            vrtf::serialize::dds::Deserializer<typename std::decay<T>::type> deserializer(
                currentPos_, remainingSize_, serializeConfig_);
            return deserializer.GetSize();
        }
    }

    const vrtf::vcc::api::types::MethodMsg& msg_;
    const std::uint8_t* currentPos_;
    std::size_t remainingSize_;
    vrtf::serialize::SerializeType serializeType_;
    vrtf::serialize::SerializeConfig serializeConfig_;
    std::vector<const std::uint8_t*> posIndex_;
    std::vector<std::size_t> argsSize_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};

class ParamsSerializer {
public:

    ParamsSerializer(uint8_t* payloadData, vrtf::serialize::SerializeType type,
        vrtf::serialize::SerializeConfig config = vrtf::serialize::SerializeConfig ())
        : currentPos_(payloadData), serializeType_(type), serializeConfig_(config)
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
    }
    ~ParamsSerializer() = default;
    template <typename... Args>
    void Serialize(Args &&... args)
    {
        DoSerialize(currentPos_, std::forward<Args>(args)...);
    }

    template <typename... Args>
    const std::size_t& GetSize(Args &&... args)
    {
        DoGetSize(std::forward<Args>(args)...);
        return size_;
    }

    void SetBuffer(uint8_t* payloadData)
    {
        currentPos_ = payloadData;
    }

private:
    uint8_t* currentPos_;  // Data vector the class will serialize into.
    vrtf::serialize::SerializeType serializeType_;
    vrtf::serialize::SerializeConfig serializeConfig_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;

    std::size_t size_ = 0;

    void DoSerialize(const uint8_t* payload_data) const
    {
        (void)payload_data;
    }
    void DoGetSize() const
    {
    }

    template <typename HEAD, typename... TAIL>
    void DoSerialize(uint8_t *payload_data, HEAD && head, TAIL &&... tail)
    {
        if (serializeType_ == vrtf::serialize::SerializeType::SHM) {
            vrtf::serialize::dds::Serializer<typename std::decay<HEAD>::type> serializer(head, serializeConfig_);
            std::size_t size = serializer.GetSize();
            serializer.Serialize(currentPos_);
            currentPos_ += size;
            DoSerialize(currentPos_, std::forward<TAIL>(tail)...);
        } else if (serializeType_ == vrtf::serialize::SerializeType::SOMEIP) {
            vrtf::serialize::someip::Serializer<typename std::decay<HEAD>::type> serializer(head, serializeConfig_);
            std::size_t size = serializer.GetSize();
            serializer.Serialize(currentPos_);
            currentPos_ += size;
            DoSerialize(currentPos_, std::forward<TAIL>(tail)...);
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() <<  "Wrong serialize type, serialize error!";
        }
    }
    template <typename HEAD, typename... TAIL>
    void DoGetSize(HEAD && head, TAIL &&... tail)
    {
        if (serializeType_ == vrtf::serialize::SerializeType::SHM) {
            vrtf::serialize::dds::Serializer<typename std::decay<HEAD>::type> serializer(head, serializeConfig_);
            size_ += serializer.GetSize();
            DoGetSize(std::forward<TAIL>(tail)...);
        } else if (serializeType_ == vrtf::serialize::SerializeType::SOMEIP) {
            vrtf::serialize::someip::Serializer<typename std::decay<HEAD>::type> serializer(head, serializeConfig_);
            size_ += serializer.GetSize();
            DoGetSize(std::forward<TAIL>(tail)...);
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() <<  "Wrong serialize type, serialize error!";
        }
    }
};
}
}
}

#endif
