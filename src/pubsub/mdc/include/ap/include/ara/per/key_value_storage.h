/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: KV文件存储接口
 * Author: s00447235
 * Create: 2019-6-26
 * Modify: 2020-6-12
 */

#ifndef ARA_PER_KEY_VALUE_STORAGE_H_
#define ARA_PER_KEY_VALUE_STORAGE_H_

#include <memory>

#include "ara/per/per_error_domain.h"
#include "ara/per/kvs_type.h"
#include "ara/per/unique_handle.h"
#include "ara/per/serializer.h"

namespace ara {
namespace per {
class KeyValueStorage {
public:
    KeyValueStorage() = default;
    virtual ~KeyValueStorage() = default;

    virtual ara::core::Result<ara::core::Vector<ara::core::String>> GetAllKeys() const noexcept = 0;
    virtual ara::core::Result<bool> HasKey(const ara::core::String& key) const noexcept = 0;
    virtual ara::core::Result<void> RemoveKey(const ara::core::StringView key) noexcept = 0;
    virtual ara::core::Result<void> RemoveAllKeys() noexcept = 0;
    virtual ara::core::Result<void> SyncToStorage() noexcept = 0;
    template <class T>
    ara::core::Result<void> GetValue(const ara::core::StringView& key, T& value) const noexcept;
    template <class T>
    ara::core::Result<void> SetValue(const ara::core::StringView& key, const T& value) noexcept;
    virtual ara::core::Result<void> GetKeyValue(const ara::core::StringView& key,
        ara::core::Vector<ara::core::String>& value) noexcept = 0;
    template <class T,
        typename std::enable_if<std::is_class<serialization::Serializer<T> >::value, T>::type* = nullptr>
    ara::core::Result<void> GetSerialValue(const ara::core::StringView key, T& value) const noexcept
    {
        ara::per::kvstype::KvsType kvs = GetInternalValue(ara::core::String(key));
        if (kvs.GetStatus() == ara::per::kvstype::KvsType::Status::kSuccess) {
            ara::per::serialization::Serializer<T> ds;
            ds.KvsReader(kvs);
            ds.ReadProcess(value);
            return ara::core::Result<void>::FromValue();
        }
        return ara::core::Result<void>::FromError(ara::per::PerErrc::kInternalError);
    }

    template <class T,
        typename std::enable_if<std::is_class<serialization::Serializer<T> >::value, T>::type* = nullptr>
    ara::core::Result<void> GetSerialValue(const ara::core::StringView key, ara::core::Vector<T>& value) const noexcept
    {
        ara::per::kvstype::KvsType kvs = GetInternalValue(ara::core::String(key));
        if (kvs.GetStatus() == ara::per::kvstype::KvsType::Status::kSuccess) {
            auto result = kvs.GetKvsArray<ara::per::kvstype::KvsType>();
            if (!result) {
                return ara::core::Result<void>::FromError(result.Error());
            }
            ara::core::Vector<ara::per::kvstype::KvsType> array = std::move(result).Value();
            for (auto it = array.begin(); it != array.end(); it++) {
                value.emplace_back();
                ara::per::serialization::Serializer<T> ds;
                ds.KvsReader(*it);
                ds.ReadProcess(value.back());
            }
            return ara::core::Result<void>::FromValue();
        }
        return ara::core::Result<void>::FromError(ara::per::PerErrc::kInternalError);
    }

    template <class T,
        typename std::enable_if<std::is_class<serialization::Serializer<T> >::value, T>::type* = nullptr>
    ara::core::Result<void> SetSerialValue(const ara::core::StringView key, const T& value) noexcept
    {
        ara::per::serialization::Serializer<T> serial;
        serial.KvsWriter(key);
        serial.WriteProcess(value);

        const ara::per::kvstype::KvsType kvs = serial.GetWriteValue();
        return SetInternalValue(ara::core::String(key), kvs);
    }

    template <class T,
        typename std::enable_if<std::is_class<serialization::Serializer<T> >::value, T>::type* = nullptr>
    ara::core::Result<void> SetSerialValue(const ara::core::StringView key, const ara::core::Vector<T>& value) noexcept
    {
        ara::per::kvstype::KvsType kvs;
        for (uint32_t i = 0U; i < value.size(); ++i) {
            const ara::core::StringView name = (ara::core::String(key) + "_" + std::to_string(i)).c_str();
            ara::per::serialization::Serializer<T> serial;
            serial.KvsWriter(name);
            serial.WriteProcess(value[i]);
            const ara::per::kvstype::KvsType type = serial.GetWriteValue();
            kvs.AddKvsArrayItem(type);
        }
        return SetInternalValue(ara::core::String(key), kvs);
    }
protected:
    virtual ara::per::kvstype::KvsType GetInternalValue(const ara::core::String& key) const noexcept = 0;
    virtual ara::core::Result<void> SetInternalValue(const ara::core::String& key,
       ara::per::kvstype::KvsType const& value) noexcept = 0;
    KeyValueStorage(const KeyValueStorage& obj) = delete;
    KeyValueStorage& operator=(const KeyValueStorage& obj) = delete;
};

ara::core::Result<ara::per::UniqueHandle<KeyValueStorage>> OpenKeyValueStorage(const ara::core::StringView kvs)
    noexcept;
ara::core::Result<void> RecoverKeyValueStorage(const ara::core::StringView kvs) noexcept;
ara::core::Result<ara::per::UniqueHandle<KeyValueStorage>> ParseKeyValueString(const ara::core::String kvs) noexcept;
}  // namespace per
}  // namespace ara
#endif  // ARA_PER_KEY_VALUE_STORAGE_H_
