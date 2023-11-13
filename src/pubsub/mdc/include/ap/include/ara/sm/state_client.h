/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
* Description: SM api
* Author: xizhipeng x00500238
* Create: 2020-05-05
* Notes: NA
*/
#ifndef ARA_SM_STATE_CLIENT_H
#define ARA_SM_STATE_CLIENT_H

#include <functional>
#include "ara/core/string.h"
#include "ara/core/vector.h"
#include "ara/core/map.h"

namespace ara {
namespace sm {
struct StateChange {
    ara::core::String functionGroupName;
    ara::core::String stateName;
};

struct FunctionGroupStates {
    ara::core::String functionGroupName;
    ara::core::Vector<ara::core::String> stateNames;
};

enum class SmResultCode : uint8_t {
    kSuccess = 0U,  /* 执行成功 */
    kInvalid,       /* 入参非法 */
    kFailed,        /* 执行失败 */
    kTimeout,       /* 请求超时 */
    kCommError,      /* 通信错误 */
    kFileError,     /* 文件操作错误 */
    kRejected,      /* 请求拒绝 */
    kEnd            /* 枚举终止位 */
};

enum class SysActionType : uint8_t {
    kSoft = 0U, /* 软复位 */
    kHard       /* 硬复位 */
};

enum class SysResetCause : uint8_t {
    kNormal = 0U,
    kUpdate
};

struct SysResetCode {
    SysActionType actionType;
    uint32_t actionTime;
};

class StateClient {
public:
    StateClient();
    virtual ~StateClient();

    /* 初始化StateClient服务, 返回结果标识其可用性 */
    SmResultCode Init();

    /* 获取所有功能组名及其状态名 */
    SmResultCode AcquireFunctionGroupInfo(ara::core::Vector<ara::sm::FunctionGroupStates>& functionGroupsInfo);

    /* 查询全部功能组的当前状态 */
    SmResultCode InquireState(ara::core::Vector<ara::sm::StateChange>& functionGroupCurrStates);

    /* 查询指定功能组的当前状态 */
    SmResultCode InquireState(const ara::core::String targetFgName, ara::core::String& currState);

    /* 请求状态转换, 返回值为执行结果, 应用程序可通过async异步获取执行结果 */
    SmResultCode RequestStates(const ara::core::Vector<ara::sm::StateChange> stateChangeList);

    /* 注册notify处理函数, 作为StateClient类的成员函数, 默认不具备对类外数据的操作权限, notify对象是单一FGS */
    SmResultCode RegisterNotifyHandler(const std::function<void (ara::sm::StateChange stateChangeReq,
        ara::sm::SmResultCode returnType)> handler);

    /* 注册notify处理函数, 允许应用程序只接收自身感兴趣的功能组的状态转换 */
    SmResultCode RegisterNotifyHandler(const std::function<void (ara::sm::StateChange stateChangeReq,
        ara::sm::SmResultCode returnType)> handler,
        const ara::core::Vector<ara::core::String> fgNames);

    /* 查询当前MDC平台是否位于开工状态 */
    SmResultCode IsMdcPlatformReady();

    /* 复位MDC系统 */
    SmResultCode SystemReset(const SysResetCode& resetCode,
                               const ara::core::String& user = "mdc",
                               const SysResetCause& resetCause = SysResetCause::kNormal);
};
}
}
#endif