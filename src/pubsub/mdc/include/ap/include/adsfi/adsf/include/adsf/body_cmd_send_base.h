/*
* Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  BodyCmdSendBase.h 车身信息的AP消息发送
 */


#ifndef HAF_ADSF_BODYCMDSENDBASE_H
#define HAF_ADSF_BODYCMDSENDBASE_H

#include <shared_mutex>
#include <unistd.h>
#include "ara/bodycmd/bodycommandserviceinterface_skeleton.h"
#include "data_send_base.h"
#include "core/types.h"
#include "control/body_command.h"

namespace Adsfi {
class BodyCmdSendBase
    : public DataSendBase<ara::bodycmd::skeleton::BodyCommandServiceInterfaceSkeleton, HafBodyCommand> {
public:
    explicit BodyCmdSendBase(const uint32_t instanceIdxIn) : DataSendBase(instanceIdxIn){};
    template <typename T1, typename T2>
    static void BodyHeaderTranslate(const T1 &fromHeader, T2 &endHeader);
    void SendingData() override;
    virtual ~BodyCmdSendBase();
};
}
#endif // HAF_ADSF_BODYCMDSENDBASE_H
