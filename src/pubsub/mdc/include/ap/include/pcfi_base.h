/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: PCFI base
 *              User should not use the header file directly.
 * Author: c00527270
 * Create: 2020-04-05
 */

#ifndef _PCFI_BASE_H_
#define _PCFI_BASE_H_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include "securec.h"

#define PCFI_CHECK_RET(cond, ret, msg) \
    do {  \
        if (cond) {  \
            PCFI_LOG_ERROR << (msg); \
            return (ret); \
        } \
    } while (false) \

namespace mdc {
namespace pcfi {
using String = std::string;
using char_t = char;
template<typename T>
using Vector = std::vector<T>;

enum ReturnCode : std::int32_t {
    PCFI_OK = 0,
    PCFI_ERROR = -1
};

template <typename ...PcfiArgs>
inline String FmtStr(const String& format, const PcfiArgs... logArgs)
{
    const size_t buffLen = 256U;
    char_t buffer[buffLen];
    char_t* const ptr = buffer;
    const int32_t reallen = snprintf_s(ptr, buffLen, buffLen - 1U, format.c_str(), logArgs...);
    return String(buffer, static_cast<std::uint32_t>(std::max(reallen, 0)));
}

template <typename T, typename... PcfiArgs>
std::shared_ptr<T> PcfiCreateObject(PcfiArgs&& ... logArgs)
{
    std::shared_ptr<T> ptr;
    try {
        ptr = std::make_shared<T>(std::forward<PcfiArgs>(logArgs)...);
    } catch (...) {
        return nullptr;
    }
    return std::move(ptr);
}
}
}

#endif
