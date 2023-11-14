/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: 持久化模块的非共享访问类
 * Author: s00515168
 * Create: 2020-05-13
 * Modify: 2020-06-10
 * Notes: 无
 */

#ifndef ARA_PER_UNIQUE_HANDLE_H_
#define ARA_PER_UNIQUE_HANDLE_H_

#include <memory>

namespace ara {
namespace per {
template <typename T>
class UniqueHandle final {
public:
    UniqueHandle() = default;

    explicit UniqueHandle(std::unique_ptr<T>&& objectValue)
        : object_(std::move(objectValue))
    {}

    UniqueHandle(UniqueHandle const& other) = delete;

    UniqueHandle(UniqueHandle&& other) = default;

    UniqueHandle& operator=(UniqueHandle const& other) = delete;

    UniqueHandle& operator=(UniqueHandle&& other) & = default;

    T* operator->() const noexcept
    {
        return object_.get();
    }

    T& operator*() const noexcept
    {
        return *object_.get();
    }

    explicit operator bool() const noexcept
    {
        return !!object_;
    }

    std::unique_ptr<T> GetValue() noexcept
    {
        return std::move(object_);
    }

    ~UniqueHandle() = default;
private:
    std::unique_ptr<T> object_;
};
}  // namespace per
}  // namespace ara
#endif  // ARA_PER_UNIQUE_HANDLE_H_