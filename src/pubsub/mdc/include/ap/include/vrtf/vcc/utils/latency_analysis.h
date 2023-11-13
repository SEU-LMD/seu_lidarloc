/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: use in delay analysis mode
 * Create: 2020-11-26
 */
#ifndef VRTF_VCC_UTILS_LATENCY_ANALYSIS_H
#define VRTF_VCC_UTILS_LATENCY_ANALYSIS_H
#include <mutex>
#include <condition_variable>
#include <climits>
#include <cmath>
#include <securec.h>
#include <functional>
#include "ara/hwcommon/log/log.h"
#include "vrtf/vcc/utils/rtf_spin_lock.h"
namespace vrtf {
namespace vcc {
namespace utils {
struct LatencyResult {
uint64_t avgTime;
uint64_t maxTime;
uint64_t minTime;
};
enum class LatencyAnalysisMode: uint8_t {
    DISABLE = 0,
    ENABLE = 1
};
using SetLatencyModeHandler = std::function<void(const vrtf::vcc::utils::LatencyAnalysisMode&)>;
using LatencyQueryHandler = std::function<LatencyResult()>;
const timespec DEFAULT_TIME_STAMP {0, 0};
class LatencyAnalysis {
public:
    LatencyAnalysis();
    ~LatencyAnalysis() = default;
    /**
     * @brief Add server send time and calculate time
     * @param[in] serverTime server timestamp receive
     */
    void AddServerSendTime(const timespec& serverTime);

    /**
     * @brief Get delay result and clear above delay info
     * @return LatencyResult delay info include avg/max/min time
     */
    LatencyResult GetLatencyResult();
private:
    size_t totalNum_;
    uint64_t totalTime_;
    vrtf::vcc::utils::RtfSpinLock spinLock_;
    LatencyResult latencyResult_ {0, 0, ULLONG_MAX};
    static std::uint8_t baseNum;
    static std::uint8_t powerNum;
    static std::uint64_t maxToleranceSecond;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}
}
}
#endif
