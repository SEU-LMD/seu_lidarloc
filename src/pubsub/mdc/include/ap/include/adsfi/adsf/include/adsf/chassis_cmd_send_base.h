/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  ChassisCmdSendBase.h 底盘信息的AP消息发送
 */

#ifndef HAF_ADSF_CHASSISCMDSENDBASE_H
#define HAF_ADSF_CHASSISCMDSENDBASE_H

#include <shared_mutex>
#include "ara/chassiscmd/chassiscommandserviceinterface_skeleton.h"
#include "data_send_base.h"
#include "core/types.h"
#include "control/chassis_command.h"

namespace Adsfi {
class ChassisCmdSendBase
    : public DataSendBase<ara::chassiscmd::skeleton::ChassisCommandServiceInterfaceSkeleton, HafChassisCommand> {
public:
    explicit ChassisCmdSendBase(const uint32_t instanceIdxIn) : DataSendBase(instanceIdxIn){};
    template <typename T1, typename T2>
    static void ChassisCmdHeaderTranslate(const T1 &fromHeader, T2 &endHeader);
    template <typename T1, typename T2>
    static void ChassisCmdSteerTranslate(const T1 &fromData, T2 &endData);
    template <typename T1, typename T2>
    static void ChassisCmdBrakeAndThrottleTranslate(const T1 &fromData, T2 &endData);
    void SendingData() override;
    virtual ~ChassisCmdSendBase();
};
}
#endif //  HAF_ADSF_CHASSISCMDSENDBASE_H
