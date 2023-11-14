/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: A reader class to read info from plog's shm files
 */

#ifndef PLOG_SHMLOGSPACEREADER_HPP
#define PLOG_SHMLOGSPACEREADER_HPP

#include "LogRecord.hpp"

#include <memory>

namespace rbs {
namespace plog {
enum class ShmLogReaderRtn {
    OK,
    ERR,
    IS_CLOSED,
    NOT_ATTACHED,
    NOTHING_TO_READ
};

class ShmLogSpaceReaderImpl;

class ShmLogSpaceReader {
public:
    struct RecordsReadResult {
        ShmLogReaderRtn rtnValue;
        std::vector<std::unique_ptr<LogRecord>> records;
    };

    ShmLogSpaceReader();

    RecordsReadResult ReadRecords();

    /**
     * @brief
     * @param shmFileName the file name should be like ${ProcessName}.${PID} , write.6473
     * @param nBlock
     * @param blockSize
     * @return
     */
    bool AttachToShmSpace(std::string shmFileName, size_t nBlock, size_t blockSize);

    static size_t DefaultBlockSize();
    static size_t DefaultBlockNum();

    std::string GetVersionString(MoudleID moudleId);

    StageNameList GetStageNames(MoudleID moudleId);

    const std::string& GetProcessName() const;

    ~ShmLogSpaceReader();

private:
    std::unique_ptr<ShmLogSpaceReaderImpl> impl_;
};
}
}


#endif // PLOG_SHMLOGSPACEREADER_HPP
