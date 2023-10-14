/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 *      This file is the implement of class RtfBuffer.
 *      RtfBuffer is for store messages
 * Author: xujunbin 00514454
 * Create: 2019-11-30
 * Notes: NA
 * History: 2019-12-17 xujunbin 00514454 revised this file.
 */
#ifndef RTF_BUFFER_H
#define RTF_BUFFER_H

#include <stdint.h>

namespace rtf {
namespace rtfbag {
class RtfBuffer {
public:
    RtfBuffer();
    ~RtfBuffer();

    bool SetSize(const uint32_t& size);
    uint32_t GetSize() const;

    const uint8_t* GetData() const;

private:
    uint8_t* buffer_;
    uint32_t size_;
};
}  // namespace rtfbag
}  // namespace rtf
#endif
