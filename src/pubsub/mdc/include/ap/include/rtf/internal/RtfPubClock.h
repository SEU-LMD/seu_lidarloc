/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: implement of class RtfPubClock
 *              RtfPubClock time clock for rtfbag palyer to public events
 * Create: 2019-12-17
 */
#ifndef RTF_PUB_CLOCK_H
#define RTF_PUB_CLOCK_H

#include <chrono>

#include "rtf/internal/RtfTimeTranslator.h"

namespace rtf {
namespace rtfbag {
class RtfPubClock {
public:
    RtfPubClock();
    ~RtfPubClock();

    void SetTimeRate(const double& rate);
    void SetBagHorizon(const uint64_t& horizon);
    void SetSysHorizon(const MilliTimePoint& horizon);
    void SetCurrentBagTime(const uint64_t& time);
    uint64_t GetTime() const;

    // run duration until sysHorizon_ is reached
    void RunClock(const std::chrono::milliseconds& duration);

    // step to horizon time
    void StepClock();

    bool HorizonReached() const;

private:
    double rate_;                          // rate of sys_time/bag_time
    std::chrono::milliseconds clockStep_;  // time long of one clock step
    MilliTimePoint sysHorizon_;            // horizon sys time
    uint64_t bagHorizon_;                  // horizon bag time
    uint64_t currentBagTime_;              // current bag timestamp
};
}  // namespace rtfbag
}  // namespace rtf
#endif
