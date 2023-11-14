/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 *      This is the head file of class RtfEventCommand.
 *      RtfEventCommand will provide all of the Rtf event command. Command format is like this:
 *      rftevent help, rtfevent list, rtfevent info [event name], and so on ...
 * Author: yangdongdong 00371884
 * Create: 2019-11-19
 * Notes: NA
 * History: 2019-11-19 yangdongdong 00371884 Create this file.
 */

#ifndef _RTF_METHOD_COMMAND_H
#define _RTF_METHOD_COMMAND_H

#include "rtf/internal/RtfCommand.h"

namespace rtf {
namespace rtfmethod {
// 错误码定义
const int RTF_METHOD_RET_OK      = 0;
const int RTF_METHOD_RET_FAILED  = -1;

// 命令等级
const int RTF_METHOD_FIRST_CMD   = 1; // 一级命令字，如：rtfevent
const int RTF_METHOD_SECOND_CMD  = 2; // 二级命令字，如：rtfevent list中的list
const int RTF_METHOD_THIRD_CMD   = 3; // 三级命令字，如：rtfevent list -h中的-h

// 命令名存储下标
const int RTF_METHOD_FIRST_CMD_IDX   = 0; // 一级命令字在vector中存储的下标，如：rtfevent
const int RTF_METHOD_SECOND_CMD_IDX  = 1; // 二级命令字在vector中存储的下标，如：rtfevent list中的list
const int RTF_METHOD_THIRD_CMD_IDX   = 2; // 二级命令字在vector中存储的下标，如：rtfevent list -h中的-h

// 命令行列表
enum RtfMethodCmds : int {
    RTF_METHOD_CMD_HELP = 1, // Print rtfevent command help information
    RTF_METHOD_CMD_LIST,     // Print all event list
    RTF_METHOD_CMD_INFO,     // Print event publish and subscribe information
    RTF_METHOD_CMD_TYPE,     // Show the detail information of an event
    RTF_METHOD_CMD_CALL,

    RTF_METHOD_CMD_UNKNOWN
};

// 类定义
class RtfMethodCommand : public rtf::RtfCommand {
public:
    RtfMethodCommand();
    virtual ~RtfMethodCommand();

    // 将命令行参数输入，执行命令并输出答应结果
    virtual int ExecuteCommand(const ara::core::Vector<ara::core::String>& inputList);

protected:
    // 打印命令帮助信息
    virtual void PrintHelpInfo();

private:
    // 用于存储命令行输入参数字符串与对应的枚举值
    ara::core::Map<ara::core::String, int> parameterMap_ = {
        {"--help",  RTF_METHOD_CMD_HELP},
        {"-h",      RTF_METHOD_CMD_HELP},
    };
};
} // end of namespace rtfevent
} // end of namespace rtf
#endif
