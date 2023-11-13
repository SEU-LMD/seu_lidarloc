/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: 持久化模块序列化和反序列化实现接口
 * Author: d00299008
 * Create: 2020-11-12
 * Modify: 2020-11-12
 * Notes: 无
 */
#ifndef ARA_PER_SERIALIZER_
#define ARA_PER_SERIALIZER_

#include <algorithm>

#include "ara/per/kvs_type.h"
#include "ara/per/per_base_type.h"
#include "ara/per/per_error_domain.h"

namespace ara {
namespace per {
namespace serialization {
template <typename Type>
class Serializer {
public:
    Serializer() = default;
    virtual ~Serializer() = default;
    virtual void ReadProcess(Type& data);

    virtual void WriteProcess(const Type& data);

    void KvsWriter(const ara::core::StringView key)
    {
        m_writeKvs.SetKey(ara::core::String(key));
    }

    ara::per::kvstype::KvsType const& GetWriteValue() const
    {
        return m_writeKvs;
    }
    template <class T,
        typename std::enable_if<std::is_same<T, ara::core::String>::value || std::is_integral<T>::value
            || std::is_floating_point<T>::value, T>::type* = nullptr>
    ara::core::Result<void> WriteHandle(const ara::core::StringView name, const T& value)
    {
        ara::per::kvstype::KvsType element(value);
        element.SetKey(ara::core::String(name));
        return m_writeKvs.AddKvsArrayItem(element);
    }

    template <class T,
        typename std::enable_if<(std::is_class<T>::value && !std::is_same<T, ara::core::String>::value),
            T>::type* = nullptr>
    ara::core::Result<void> WriteHandle(const ara::core::StringView name, const T& value)
    {
        auto s = std::make_unique<Serializer<T> >();
        s->KvsWriter(name);
        s->WriteProcess(value);
        return m_writeKvs.AddKvsArrayItem(s->GetWriteValue());
    }

    void KvsReader(const ara::per::kvstype::KvsType& parentElement)
    {
        m_readKvs = parentElement;
    }
    template <class T, typename std::enable_if<std::is_same<T, ara::core::String>::value, T>::type* = nullptr>
    ara::core::Result<void> ReadHandle(const ara::core::StringView name, T& value)
    {
        using ara::per::kvstype::KvsType;
        // firstly find needed element in parent child elements
        const auto resChildElements = m_readKvs.GetKvsArray<ara::per::kvstype::KvsType>();
        if (!resChildElements) {
            return ara::core::Result<void>::FromError(resChildElements.Error());
        }

        const auto it = std::find_if(resChildElements.Value().begin(), resChildElements.Value().end(),
            [&name](const KvsType& child) { return child.GetKey() == ara::core::String(name); });
        KvsType foundChild;
        if (it != resChildElements.Value().end()) {
            foundChild = *it;
        }
        // if child element not found, just return - not an error
        if (foundChild.GetStatus() != KvsType::Status::kSuccess) {
            return ara::core::Result<void>::FromValue();
        }

        if (foundChild.GetType() != KvsType::Type::kString) {
            return ara::core::Result<void>::FromError(PerErrc::kDataTypeMismatchError);
        }

        const auto kvsVal = foundChild.GetKvsString();
        if (kvsVal) {
            value = kvsVal.Value();
            return ara::core::Result<void>::FromValue();
        } else {
            return ara::core::Result<void>::FromError(kvsVal.Error());
        }
    }

    template <class T, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
    ara::core::Result<void> ReadHandle(const ara::core::StringView name, T& value)
    {
        using ara::per::kvstype::KvsType;
        // firstly find needed element in parent child elements
        const auto resChildElements = m_readKvs.GetKvsArray<KvsType>();
        if (!resChildElements) {
            return ara::core::Result<void>::FromError(resChildElements.Error());
        }

        const auto it = std::find_if(resChildElements.Value().begin(), resChildElements.Value().end(),
            [&name](const KvsType& child) { return child.GetKey() == ara::core::String(name); });
        KvsType foundChild;
        if (it != resChildElements.Value().end()) {
            foundChild = *it;
        }

        // if child element not found, just return - not an error
        if (foundChild.GetStatus() != KvsType::Status::kSuccess) {
            return ara::core::Result<void>::FromValue();
        }

        if (foundChild.GetType() != KvsType::Type::kDouble
            && foundChild.GetType() != KvsType::Type::kFloat) {
            return ara::core::Result<void>::FromError(PerErrc::kDataTypeMismatchError);
        }

        const auto kvsVal = foundChild.GetKvsDouble();
        if (kvsVal) {
            value = kvsVal.Value();
            return ara::core::Result<void>::FromValue();
        } else {
            return ara::core::Result<void>::FromError(kvsVal.Error());
        }
    }

    template <class T, typename std::enable_if<std::is_integral<T>::value, T>::type* = nullptr>
    ara::core::Result<void> ReadHandle(const ara::core::StringView name, T& value)
    {
        const auto resChildElements = m_readKvs.GetKvsArray<kvstype::KvsType>();
        if (!resChildElements) {
            return ara::core::Result<void>::FromError(resChildElements.Error());
        }
        const auto it = std::find_if(resChildElements.Value().begin(), resChildElements.Value().end(),
            [&name](const kvstype::KvsType& child) { return child.GetKey() == ara::core::String(name); });
        kvstype::KvsType foundChild;
        if (it != resChildElements.Value().end()) {
            foundChild = *it;
        }
        if (foundChild.GetStatus() != kvstype::KvsType::Status::kSuccess) {
            return ara::core::Result<void>::FromValue();
        }
        return ReadIntegralType(foundChild, value);
    }

    template <class T, typename std::enable_if<(std::is_class<T>::value &&
        !std::is_same<T, ara::core::String>::value), T>::type* = nullptr>
    ara::core::Result<void> ReadHandle(const ara::core::StringView name, T& value)
    {
        using ara::per::kvstype::KvsType;
        const auto resChildElements = m_readKvs.GetKvsArray<KvsType>();
        if (!resChildElements) {
            return ara::core::Result<void>::FromError(resChildElements.Error());
        }

        const auto it = std::find_if(resChildElements.Value().begin(), resChildElements.Value().end(),
            [&name](const KvsType& child) { return child.GetKey() == ara::core::String(name); });
        KvsType foundChild;
        if (it != resChildElements.Value().end()) {
            foundChild = *it;
        }

        // if child element not found, just return - not an error
        if (foundChild.GetStatus() != KvsType::Status::kSuccess) {
            return ara::core::Result<void>::FromValue();
        }
        auto ds = std::make_unique<Serializer<T> >();
        ds->KvsReader(foundChild);
        ds->ReadProcess(value);

        return ara::core::Result<void>::FromValue();
    }
private:
    template <class T, typename std::enable_if<std::is_integral<T>::value, T>::type* = nullptr>
    static ara::core::Result<void> ReadIntegralType(const kvstype::KvsType& kvstype, T& value)
    {
        ara::core::Result<void> result;
        switch (kvstype.GetType()) {
            case kvstype::KvsType::Type::kSInt8:
            case kvstype::KvsType::Type::kSInt16:
            case kvstype::KvsType::Type::kSInt32:
            case kvstype::KvsType::Type::kSInt64: {
                const auto kvs = kvstype.GetKvsSInt64();
                if (kvs) {
                    value = static_cast<T>(kvs.Value());
                }
                result = kvs ? ara::core::Result<void>::FromValue() : ara::core::Result<void>::FromError(kvs.Error());
                break;
            }
            case kvstype::KvsType::Type::kUInt8:
            case kvstype::KvsType::Type::kUInt16:
            case kvstype::KvsType::Type::kUInt32:
            case kvstype::KvsType::Type::kUInt64: {
                const auto kvs = kvstype.GetKvsUInt64();
                if (kvs) {
                    value = static_cast<T>(kvs.Value());
                }
                result = kvs ? ara::core::Result<void>::FromValue() : ara::core::Result<void>::FromError(kvs.Error());
                break;
            }
            case kvstype::KvsType::Type::kBoolean: {
                const auto kvs = kvstype.GetKvsBool();
                if (kvs) {
                    value = kvs.Value();
                }
                result = kvs ? ara::core::Result<void>::FromValue() : ara::core::Result<void>::FromError(kvs.Error());
                break;
            }
            default:
                result = ara::core::Result<void>::FromError(PerErrc::kDataTypeMismatchError);
                break;
        }
        return result;
    }
    ara::per::kvstype::KvsType m_writeKvs;
    ara::per::kvstype::KvsType m_readKvs;
};
}  // namespace serialization
}  // namespace per
}  // namespace ara
#endif // ARA_PER_SERIALIZER_
