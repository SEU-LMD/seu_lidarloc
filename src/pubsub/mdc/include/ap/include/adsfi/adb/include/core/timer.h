/* *
 * FUNCTION: Define Timer
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *  */

#ifndef HAF_CORE_TIMER_H
#define HAF_CORE_TIMER_H

#include <cstdint>
#include "core/basic_types.h"
#include "core/logger.h"
namespace Adsfi {
const uint32_t NANOSECOND_MULTIPLE = 1000U * 1000U * 1000U;
struct HafTime;

/* ******************************************************************************
    函 数 名		:  CreateAbnormalTime
    功能描述		:  创建一个初始化值异常的时间戳
    输入参数		:  None
    返 回 值		:  HafTime : 初始化值异常的时间戳
****************************************************************************** */
HafTime CreateAbnormalTime();

struct HafTime {
    HafTime(const uint32_t s, const uint32_t ns) : sec(s), nsec(ns)
    {}

    HafTime() = default;
    ~HafTime() = default;

    uint32_t sec{0U};
    uint32_t nsec{0U};

    /* ******************************************************************************
        函 数 名		:  -
        功能描述		:  -运算符重载，可用于计算两个时间戳之间的时延
        输入参数		:  start : 起始时间戳
        返 回 值		:  HafTime结构体表示的时延
    ****************************************************************************** */
    HafTime operator -(const HafTime &start) const
    {
        if ((sec < start.sec) || ((sec == start.sec) && (nsec < start.nsec))) {
            HAF_LOG_ERROR << "the input timestamp should be less than this";
            return CreateAbnormalTime();
        }
        HafTime elapsedTime;
        if (nsec < start.nsec) {
            elapsedTime.nsec = NANOSECOND_MULTIPLE - (start.nsec - nsec);
            elapsedTime.sec = sec - start.sec - 1U;
        } else {
            elapsedTime.nsec = nsec - start.nsec;
            elapsedTime.sec = sec - start.sec;
        }
        return elapsedTime;
    }
};

/* ******************************************************************************
    函 数 名		:  ToSecond
    功能描述		:  将时间戳换算成秒数
    输入参数		:  time : 时间戳
    返 回 值		:  时间戳对应的秒数
****************************************************************************** */
float128_t ToSecond(const HafTime time);

/* ******************************************************************************
    函 数 名		:  ToMillisecond
    功能描述		:  将时间戳换算成毫秒数
    输入参数		:  time : 时间戳
    返 回 值		:  时间戳对应的毫秒数
****************************************************************************** */
float128_t ToMillisecond(const HafTime time);

/* ******************************************************************************
    函 数 名		:  ToMicrosecond
    功能描述		:  将时间戳换算成微秒数
    输入参数		:  time : 时间戳
    返 回 值		:  时间戳对应的微秒数
****************************************************************************** */
float128_t ToMicrosecond(const HafTime time);

/* ******************************************************************************
    函 数 名		:  SecondsToHafTime
    功能描述		:  将秒数换算成时间戳
    输入参数		:  seconds : 秒数
    返 回 值		:  秒数对应的时间戳，若秒数小于则返回未初始化的时间戳
****************************************************************************** */
HafTime SecondsToHafTime(const float128_t seconds);

/* ******************************************************************************
    函 数 名		:  MilliSecondsToHafTime
    功能描述		:  将毫秒数换算成时间戳
    输入参数		:  milliseconds : 毫秒数
    返 回 值		:  毫秒数对应的时间戳，若毫秒数小于则返回未初始化的时间戳
****************************************************************************** */
HafTime MilliSecondsToHafTime(const float128_t milliseconds);

/* ******************************************************************************
    函 数 名		:  MicroSecondsToHafTime
    功能描述		:  将微秒数换算成时间戳
    输入参数		:  microseconds : 微秒数
    返 回 值		:  微秒数对应的时间戳，若微秒数小于则返回未初始化的时间戳
****************************************************************************** */
HafTime MicroSecondsToHafTime(const float128_t microseconds);
}
#endif // HAF_CORE_TIMER_H
