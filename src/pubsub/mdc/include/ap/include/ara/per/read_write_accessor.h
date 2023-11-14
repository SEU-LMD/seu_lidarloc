/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: 持久化模块的读写操作
 * Author: s00515168
 * Create: 2019-12-10
 * Modify: 2020-06-10
 * Notes: 无
 */

#ifndef ARA_PER_READ_WRITE_ACCESSOR_H_
#define ARA_PER_READ_WRITE_ACCESSOR_H_

#include "read_accessor.h"

namespace ara {
namespace per {
class ReadWriteAccessor : public ReadAccessor {
public:
    ReadWriteAccessor() = default;
    virtual ~ReadWriteAccessor() = default;
    virtual ara::core::Result<void> fsync() noexcept = 0;
    virtual std::fstream::pos_type write(const ara::core::Span<char_t> s) noexcept = 0;
    virtual ReadWriteAccessor& operator<<(ara::core::String const& streamString) noexcept = 0;
    virtual void flush() noexcept = 0;
private:
    ReadWriteAccessor(const ReadWriteAccessor& obj) = delete;
    ReadWriteAccessor& operator=(const ReadWriteAccessor& obj) = delete;
};
}  // namespace per
}  // namaspace ara
#endif  // ARA_PER_READ_WRITE_ACCESSOR_H_
