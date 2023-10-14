/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: 基本数据类型定义
 * Author: o00288819
 * Create: 2020-03-26
 */
#ifndef DEVM_BASE_TYPE_H
#define DEVM_BASE_TYPE_H
#include <string>
#include <map>
#include <vector>
#include <list>

namespace mdc {
namespace devm {
using String = std::string;
template <typename T, typename Y>
using Map = std::map<T, Y>;
template <typename T, typename Allocator = std::allocator<T>>
using Vector = std::vector<T, Allocator>;
template <typename T, typename Allocator = std::allocator<T>>
using List = std::list<T, Allocator>;
using char_t = char;

struct DevmCanIdInfo {
    std::uint32_t canIdType {0U};
    std::uint32_t canIdRangeType {0U};
    Vector<std::uint32_t> canIdRange;

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(canIdRange);
        fun(canIdType);
        fun(canIdRangeType);
    }

    static bool IsPlane()
    {
        return false;
    }
    bool operator ==(const mdc::devm::DevmCanIdInfo& t) const
    {
        return ((canIdRange == t.canIdRange) && (canIdType == t.canIdType)) && (canIdRangeType == t.canIdRangeType);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(canIdRange);
        fun(canIdType);
        fun(canIdRangeType);
    }
};

struct DevmCanCheckList {
    std::uint8_t idNumber {0U};
    std::uint8_t controlMethod {0U};
    Vector<DevmCanIdInfo> canIdInfos;

    static bool IsPlane()
    {
        return false;
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(idNumber);
        fun(controlMethod);
        fun(canIdInfos);
    }

    bool operator ==(const mdc::devm::DevmCanCheckList& t) const
    {
        return ((idNumber == t.idNumber) && (controlMethod == t.controlMethod) &&
                (canIdInfos == t.canIdInfos));
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(idNumber);
        fun(controlMethod);
        fun(canIdInfos);
    }
};

struct DevmCtrlConfig {
    std::uint8_t channelId {0U};
    DevmCanCheckList upStream;
    DevmCanCheckList downStream;

    static bool IsPlane()
    {
        return false;
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(channelId);
        fun(upStream);
        fun(downStream);
    }

    bool operator ==(const mdc::devm::DevmCtrlConfig& t) const
    {
        return ((channelId == t.channelId) && (upStream == t.upStream) &&
                (downStream == t.downStream));
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(channelId);
        fun(upStream);
        fun(downStream);
    }
};
}
}
#endif
