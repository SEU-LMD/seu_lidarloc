/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file provides an abstract class to support ros serialize for mbuf.
 * Create: 2020-11-04
 */
#ifndef RAW_BUFFER_HELPER_H
#define RAW_BUFFER_HELPER_H
#include <string>
#include "vrtf/driver/dds/mbuf.h"
namespace vrtf {
namespace vcc {
class RawBufferHelper {
public:
    virtual bool IsBuffMsg(const std::string& dataType) = 0;
    virtual Mbuf *GetMbufFromMsg(const std::string& dataType, const void* msg) = 0;
    virtual bool SetMbufToMsg(const std::string& dataType, const Mbuf *mbuf, void * const msg) = 0;
};
}
}
#endif
