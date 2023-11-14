/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: A base class representing a record in plog
 */

#ifndef PLOG_LOGRECORD_HPP
#define PLOG_LOGRECORD_HPP

#include <vector>
#include <memory>

#include "plog/PLogDefsAndLimits.hpp"


namespace rbs {
namespace plog {
struct LogRecord {
    LogRecord() = default;

    virtual uint64_t Serialize2RawData(uint8_t* dataHead, uint64_t destLen) const = 0;

    virtual uint64_t CalSize() const = 0;

    /**
     * @return the deserialized len, if fail, return 0
     */
    virtual uint64_t DeserializeFromRawData(const uint8_t* dataHead, uint16_t theLenShouldBe) = 0;

    /**
     * @brief convert to a csv string
     * @return the string
     */
    virtual std::string ToString() const = 0;

    virtual void Write2Stream(std::ostream& os) = 0;

    virtual ~LogRecord() = default;

protected:
    mutable uint16_t len_{0};
};
}
}

#endif // PLOG_LOGRECORD_HPP
