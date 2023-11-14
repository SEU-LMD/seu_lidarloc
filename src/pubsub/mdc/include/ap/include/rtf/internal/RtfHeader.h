/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 *      This file is the implement of class BagFile.
 *      BagFile will create a bag file and provide read or write operation
 * Author: xujunbin 00514454
 * Create: 2019-11-30
 * Notes: NA
 * History: 2019-12-04 xujunbin 00514454 revised this file.
 */
#ifndef RTF_HEADER_H
#define RTF_HEADER_H

#include "ara/core/string.h"
#include "ara/core/map.h"

namespace rtf {
namespace rtfbag {
using ReadMap = ara::core::Map<ara::core::String, ara::core::String>;

class RtfHeader {
public:
    RtfHeader() = default;
    ~RtfHeader() = default;

    bool Parse(const uint8_t* buffer, const uint32_t& size);
    bool GetValue(const ara::core::String& key, ara::core::String& value) const;
    ReadMap GetValues() const;

private:
    ReadMap readMap_;
};
}  // namespace rtfbag
}  // namespace rtf
#endif
