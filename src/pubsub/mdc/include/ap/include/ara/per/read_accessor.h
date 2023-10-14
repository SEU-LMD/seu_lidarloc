/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: 持久化模块的读操作
 * Author: s00515168
 * Create: 2019-12-10
 * Modify: 2020-06-10
 * Notes: 无
 */

#ifndef ARA_PER_READ_ACCESSOR_H_
#define ARA_PER_READ_ACCESSOR_H_

#include "ara/per/basic_operations.h"

namespace ara {
namespace per {
class ReadAccessor : public BasicOperations {
public:
    ReadAccessor() = default;
    virtual ~ReadAccessor() = default;
    virtual std::fstream::int_type peek() const noexcept = 0;
    virtual std::fstream::int_type get() const noexcept = 0;
    virtual std::fstream::pos_type read(const ara::core::Span<char_t> s) const noexcept = 0;
    virtual ReadAccessor& getline(ara::core::String& streamString, char_t const delim = '\n') noexcept = 0;
private:
    ReadAccessor(const ReadAccessor& obj) = delete;
    ReadAccessor& operator=(const ReadAccessor& obj) = delete;
};
}  // namespace per
}  // namaspace ara
#endif  // ARA_PER_READ_ACCESSOR_H_
