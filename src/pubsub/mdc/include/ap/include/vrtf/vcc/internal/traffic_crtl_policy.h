/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file provides an interface to support traffic control.
 * Create: 2020-08-06
 */
#ifndef TRAFFICCTRLPOLICY_H_
#define TRAFFICCTRLPOLICY_H_

#include <cstdint>
#include <queue>
#include <memory>
#include "ara/hwcommon/log/log.h"

namespace rtf {
enum class TrafficCtrlUnit : uint32_t {
    TRAFFIC_CTRL_UNIT_SEC = 1000,
    TRAFFIC_CTRL_UNIT_TENTH_OF_SEC = 100,
    TRAFFIC_CTRL_UNIT_HUNDRED_OF_SEC = 10
};

enum class TrafficCtrlAction : uint32_t {
    DISCARD,
    ALLOWED,
    BLOCK
};

class TrafficCtrlPolicy {
public:
    TrafficCtrlPolicy() = default;
    virtual ~TrafficCtrlPolicy() = default;
    virtual TrafficCtrlAction GetTrafficCtrlAction() = 0;
    virtual bool UpdateTrafficInfo(const TrafficCtrlAction& action) = 0;
};

class BasicPolicy : public TrafficCtrlPolicy {
public:
    BasicPolicy();
    ~BasicPolicy() = default;
    BasicPolicy(const TrafficCtrlUnit& unit, const uint32_t& count);
    void SetWindowUnit(const TrafficCtrlUnit& unit);
    TrafficCtrlUnit GetWindowUnit() const;
    bool SetTrafficCount(const uint32_t& count);
    uint32_t GetTrafficCount() const;
    TrafficCtrlAction GetTrafficCtrlAction() override;
    bool UpdateTrafficInfo(const TrafficCtrlAction& action) override;

private:
    bool valid_ = true;
    TrafficCtrlUnit unitTime_;
    uint32_t count_;
    double currentTime_ = 0;
    bool allowedPrintFlag_ = false;
    uint32_t discardCount_ = 0;
    std::queue<double> timeInfoQueue_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    std::mutex mutex_;
    struct timespec discardTimeSpec_;
    struct timespec allowedTimeSpec_;
};
}
#endif

