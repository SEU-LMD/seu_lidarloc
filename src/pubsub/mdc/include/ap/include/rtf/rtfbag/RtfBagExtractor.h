/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:
 *      This file is the header of class RtfBagExtractor.
 *      RtfBagExtractor will read a bag file and extract specific event(s) from a bagfile
 * Author: liuxiaobo 00453303
 * Create: 2020-07-16
 * Notes: NA
 * History: 2020-07-18 liuxiaobo 00453303 created this file.
 */
#ifndef RTF_BAG_EXTRACTOR_H
#define RTF_BAG_EXTRACTOR_H

#include "rtf/internal/RtfBagFile.h"
#include "rtf/internal/RtfView.h"
#include "rtf/internal/RtfMsgEntity.h"

namespace rtf {
namespace rtfbag {
class RtfBagExtractor {
public:
    enum class ExtractResult : uint8_t {
        RET_SUCCEED,
        RET_CONFLICT_FILE,
        RET_START_TIME_EXCCEED_LIMIT,
        RET_END_TIME_EXCCEED_LIMIT,
        RET_INVALID_TIME_RANGE,
        RET_NO_ENOUGH_SPACE,
        RET_NO_MESSAGE_FOUND,
        RET_READ_FAILED,
        RET_WRITE_FAILED,
        RET_FAILED,
        RET_INVALID_INPUT_FILE_NAME,
        RET_INVALID_OUTPUT_FILE_NAME
    };

    enum class ExtractStatus : uint8_t {
            RET_IDLE,             // IDLE state, extract has not started or has ended
            RET_PREPARING,        // PREPAR state, preparation phase
            RET_EXTRACTING,       // EXTRACT state, split phase
            RET_STOPPING          // STOPPING state, stopping extract
    };

    struct ExtractProgress {
            ExtractStatus status;    // Current state
            double progress;         // Extract progress, [0, 1]
    };

    RtfBagExtractor(const ara::core::String& inputFile, const ara::core::String& outputFile,
        const ara::core::Vector<ara::core::String>& events,
        const uint64_t& startTime, const uint64_t& endTime);
    ~RtfBagExtractor(void);
    ExtractResult Extract();
    ExtractProgress GetExtractProgress();
    void Stop() noexcept;
private:

    ExtractResult CheckParam(
        const ara::core::String& inputFile, const ara::core::String& outputFile,
        const uint64_t& startTime, const uint64_t& endTime);
    RtfView QueryViewFromBag(
        const RtfBagFile& bagFile, const ara::core::Vector<ara::core::String>& events,
        const uint64_t& startTime, const uint64_t& endTime) noexcept;
    ExtractResult WriteViewToFile(RtfView& view, const std::string& fileName) noexcept;
    ExtractResult WriteMsgEntityToBag(const RtfMsgEntity& msgEntity, RtfBagFile& bagFile) noexcept;
    const ara::core::String UpdateOutputName(const ara::core::String& outputFile);
    void InitProgress();
    ExtractResult DoExtract();

    ara::core::String inputFile_;
    ara::core::String outputFile_;
    ara::core::Vector<ara::core::String> events_;
    uint64_t startTime_;
    uint64_t endTime_;
    ExtractProgress extractProgress_;
};
}
}

#endif

