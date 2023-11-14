/*
* Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
* Description: LogStreamBuffer class header
* Create: 2019-7-2
*/
#ifndef INC_ARA_GODEL_COMMON_LOG_COMMON_H
#define INC_ARA_GODEL_COMMON_LOG_COMMON_H
namespace ara    {
namespace godel  {
namespace common {
namespace log    {
enum class LogType : uint8_t {
    SCREEN_LOG = 0,
    ARA_LOG,
    SYS_LOG
};
enum class LogLevel : uint8_t {
    VRTF_COMMON_LOG_OFF      = 0x00,
    VRTF_COMMON_LOG_FATAL    = 0x01,
    VRTF_COMMON_LOG_ERROR    = 0x02,
    VRTF_COMMON_LOG_WARN     = 0x03,
    VRTF_COMMON_LOG_INFO     = 0x04,
    VRTF_COMMON_LOG_DEBUG    = 0x05,
    VRTF_COMMON_LOG_VERBOSE  = 0x06
};
enum class LogMode : uint8_t {
    LOG_REMOTE = 0,
    LOG_FILE,
    LOG_CONSOLE
};
} // end log
} // end common
} // end godel
} // end ara
#endif

