/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: use to generate type-length-value(tlv) buffer and analysis buffer
 * Create: 2020-12-07
 */
#ifndef VRTF_VCC_UTILS_TLV_HELPER_H
#define VRTF_VCC_UTILS_TLV_HELPER_H
#include <mutex>
#include <securec.h>
#include "ara/hwcommon/log/log.h"
namespace vrtf {
namespace vcc {
namespace utils {
const std::uint8_t TIME_STAMP_LENGTH = 16;
const std::uint16_t TIME_STAMP_TYPE_ID = 0xFFFA;
const size_t TIME_STAMP_SIZE = sizeof(time_t) + sizeof(long);
const size_t TLV_TIME_TOTAL_SIZE = sizeof(TIME_STAMP_LENGTH) + sizeof(TIME_STAMP_TYPE_ID) + TIME_STAMP_SIZE;
enum class TlvType: std::uint8_t {
    TIME_STAMP = 0x00
};
static const std::unordered_map<TlvType, uint16_t> typeToIdMap = {
    {TlvType::TIME_STAMP, 0x01}
};
enum class TlvAnalysisResult: std::uint8_t {
    SUCCESS = 0X00,
    MEMCPY_FAIL,
    ANALYSIS_FAIL
};
class TlvHelper {
public:
    /**
     * @brief Send event/field By skeleton applications, skeleton should initialize field first.
     * @param[in] buffer data buffer add time stamp info.
     * @param[in] ts user use send interface.
     */
    static void AddTlvTimeStamp(std::uint8_t* buffer, const timespec& ts);

    /**
     * @brief Check data buffer is tlv timeStample label and get send time.
     * @param[in] data receive buffer include tlv time label
     * @param[in] ts analysis tlv time
     * @return TlvAnalysisResult analysis result enum
     */
    static TlvAnalysisResult AnalysisTlvTime(const std::uint8_t * const data, timespec& ts);
};
}
}
}
#endif
