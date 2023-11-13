/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: implement of class RtfTimeTranslator
 *              RtfTimeTranslator trans timestamp recorded in bag file to system clock
 * Create: 2019-12-17
 */
#ifndef RTF_TIME_TRANSLATOR_H
#define RTF_TIME_TRANSLATOR_H

#include <chrono>

namespace rtf {
namespace rtfbag {
using MilliTimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds>;
class RtfTimeTranslator {
public:
    RtfTimeTranslator();
    ~RtfTimeTranslator();
    void SetTimeRate(const double& rate);
    void SetBagStartTime(const uint64_t& time);
    void SetSysClockStartTime(const MilliTimePoint& time);
    void ShiftSysClock(const std::chrono::milliseconds& deltaTime);
    MilliTimePoint Translate(const uint64_t& time) const;

private:
    double rate_;                  // rate of sys_time/bag_time
    uint64_t bagStartTime_;        // start time of timestamp in the bag file
    MilliTimePoint sysStartTime_;  // start time of system colock
};
}  // namespace rtfbag
}  // namespace rtf
#endif
