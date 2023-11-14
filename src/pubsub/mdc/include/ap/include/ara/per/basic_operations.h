/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: 文件的基础操作
 * Author: s00515168
 * Create: 2019-12-10
 * Modify: 2020-06-10
 * Notes: 无
 */

#ifndef ARA_PER_BASIC_OPERATIONS_H_
#define ARA_PER_BASIC_OPERATIONS_H_
#include <fstream>
#include "ara/per/per_base_type.h"
namespace ara {
namespace per {
class BasicOperations {
public:
    enum class SeekDirection : uint8_t {
        kBeg = 0U, // Seek from the beginning.
        kEnd = 1U, // Seek from the end.
        kCur = 2U // Seek from the current position.
    };

    enum class OpenMode : uint8_t {
        kApp = 1 << 0,    // append to the end
        kBinary = 1 << 1, // open as binary, will be opened as text if missing
        kTrunc = 1 << 2,  // delete existing data at open
        kAte = 1 << 3     // put seek pointer at the end
    };
    BasicOperations() = default;
    virtual ~BasicOperations() = default;

    virtual std::fstream::pos_type tell() const noexcept = 0;

    virtual void seek(std::fstream::pos_type const pos) noexcept = 0;

    virtual void seek(std::fstream::off_type const off, SeekDirection const direction) noexcept = 0;

    virtual bool good() const noexcept = 0;

    virtual bool eof() const noexcept = 0;

    virtual bool fail() const noexcept = 0;

    virtual bool bad() const noexcept = 0;

    virtual bool operator!() const noexcept = 0;

    explicit virtual operator bool() const noexcept = 0;

    virtual void clear() noexcept = 0;
private:
    BasicOperations(const BasicOperations& obj) = delete;
    BasicOperations& operator=(const BasicOperations& obj) = delete;
};

BasicOperations::OpenMode operator|(BasicOperations::OpenMode const& left,
                                    BasicOperations::OpenMode const& right) noexcept;

BasicOperations::OpenMode operator&(BasicOperations::OpenMode const& left,
                                    BasicOperations::OpenMode const& right) noexcept;
}  // namespace per
}  // namaspace ara

#endif  // ARA_PER_BASIC_OPERATIONS_H_
