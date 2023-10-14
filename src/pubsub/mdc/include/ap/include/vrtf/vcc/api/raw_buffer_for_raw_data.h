/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: the implementation of RawBufferForMbuf class
 * Create: 2021-01-21
 */
#ifndef VRTF_CORE_RAW_BUFFER_FOR_RAW_DATA_H
#define VRTF_CORE_RAW_BUFFER_FOR_RAW_DATA_H

#include "vrtf/vcc/api/raw_data.h"
#include "vrtf/driver/dds/mbuf.h"

namespace vrtf {
namespace core {
class RawBufferForRawData {
public:
    vrtf::core::RawBuffer privateData;
    Mbuf* mbuf = nullptr;

    static bool IsPlane()
    {
        return false;
    }

    Mbuf* GetMbufPtr() const
    {
        return mbuf;
    }

    void SetMbufPtr(Mbuf *p)
    {
        mbuf = p;
    }

    using IsDpRawDataTag = void;
    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(privateData);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(privateData);
    }

    bool operator == (const RawBufferForRawData& t) const
    {
        return (privateData == t.privateData);
    }
}
;
}
}
#endif
