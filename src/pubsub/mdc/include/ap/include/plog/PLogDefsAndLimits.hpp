/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: some defs and limits of plog
 */

#ifndef PLOG_PLOGDEFSANDLIMITS_HPP
#define PLOG_PLOGDEFSANDLIMITS_HPP

#include <vector>
#include <unordered_map>


namespace rbs {
namespace plog {
/**
 * @brief the ID of the moudle, because we may support user moudle, so we dont make it an enum
 */
using MoudleID = uint8_t;
using StageID = uint8_t;
/* using unordered map, so that stageID won't repeat at the beginning */
using StageNameList = std::unordered_map<StageID, std::string>;
using UserRawDataType = std::string;

using PlogUid = uint64_t;
static constexpr PlogUid PLOG_UID_MAX = UINT64_MAX;

static constexpr uint64_t STAGE_NUM_MAX = 16;

/** some id is used in C header, cplog.h , note the relationship! */
static constexpr MoudleID MOUDLE_CM_SEND = 1;
static constexpr MoudleID MOUDLE_CM_RECV = 2;
static constexpr MoudleID MOUDLE_SOMEIP_SEND = 3;
static constexpr MoudleID MOUDLE_SOMEIP_RECV = 4;
static constexpr MoudleID MOUDLE_DDS_SEND = 5;
static constexpr MoudleID MOUDLE_DDS_RECV = 6;
/** to achieve nolock get, we set a max num of moudle */
static constexpr MoudleID MOUDLE_MAX_NUM = 8;

/** all size max includes the null (\0) End, these two has fix size in the shm */
static constexpr uint64_t VERSION_STR_SIZE_MAX = 32;
static constexpr uint64_t STAGE_NAME_SIZE_MAX = 64;
static constexpr uint64_t USER_GUID_SIZE_MAX = 128;

/** all len doesn't count the null (\0) end */
static constexpr uint64_t VERSION_STR_LEN_MAX = VERSION_STR_SIZE_MAX - 1;
static constexpr uint64_t STAGE_NAME_LEN_MAX = STAGE_NAME_SIZE_MAX - 1;
static constexpr uint64_t USER_GUID_LEN_MAX = USER_GUID_SIZE_MAX - 1;
static constexpr uint64_t USER_DEFINED_DATA_LEN_MAX = 32;

static constexpr char PLOG_ENV_SWITCH_STR[] = "DDS_PLOG_SWITCH";
static constexpr char PLOG_ENV_ENABLED_STR[] = "ON";
static constexpr char PLOG_SHM_FILE_PREFIX[] = "__RBS_PLOG_";

static constexpr uint64_t DEFAULT_BLOCK_SIZE = 1024;
static constexpr uint64_t DEFAULT_BLOCK_NUM = 128;

static constexpr uint64_t TIMESTAMP_CURRENT = 0;

/** We assign the value to be compact with C API */
enum class PlogReturnValue : int {
    OK = 0,
    ERR = -1,
    INVALID_INPUT = -2,
    NOT_INITED = -3,
    MODULE_NOT_MATCH = -4,
    SWITCH_IS_OFF = 1,
    ALREADY_INITED = 2,
    RELATED_WRITER_NOT_READY = 3
};
}
}

#endif // PLOG_PLOGDEFSANDLIMITS_HPP
