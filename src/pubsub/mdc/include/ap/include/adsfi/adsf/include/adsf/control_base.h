/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  ControlBase.h Planning规划框架基类
 */

#ifndef HAF_ADSF_CONTROL_BASE_H
#define HAF_ADSF_CONTROL_BASE_H

#include <shared_mutex>
#include "adsfi_base.h"
#include "adsf/body_report_receive_base.h"
#include "adsf/chassis_report_receive_base.h"
#include "adsf/ego_trajectory_receive_base.h"
#include "adsf/location_receive_base.h"
#include "adsf/body_cmd_send_base.h"
#include "adsf/chassis_cmd_send_base.h"
namespace Adsfi {
class ControlBase : public AdsfiBase {
public:
    explicit ControlBase(const std::string configFile)
        : AdsfiBase(configFile),
          bodyReportInstanceIdx_(0U),
          chassisReportInstanceIdx_(0U),
          egoTrajectoryInstanceIdx_(0U),
          locationInstanceIdx_(0U),
          bodyCmdInstanceIdx_(0U),
          chassisCmdInstanceIdx_(0U),
          subThreads(){};
    ~ControlBase() override;
    void Stop() override;
    std::vector<std::shared_ptr<HafLocation>> GetLocation(const size_t &length) const;
    std::vector<std::shared_ptr<HafEgoTrajectory>> GetEgoTrajectory(const size_t &length) const;
    std::vector<std::shared_ptr<HafBodyReport>> GetBodyReport(const size_t &length) const;
    std::vector<std::shared_ptr<HafChassisReport>> GetChassisReport(const size_t &length) const;
    HafStatus SendBodyCommand(std::shared_ptr<HafBodyCommand> &bodyCmd) const;
    HafStatus SendChassisCommand(std::shared_ptr<HafChassisCommand> &chassisCmd) const;

protected:
    HafStatus StartSubThread() override;
    HafStatus StartPubThread() override;
    HafStatus ParsingInstanceId(const HafYamlNode &config) override;
private:
    bool IsInsIdxValid() const;
    const uint32_t defaultBodyReportInstanceIdx_ = 1U;
    const uint32_t defaultChassisReportInstanceIdx_ = 1U;
    const uint32_t defaultEgoTrajectoryInstanceIdx_ = 103U;
    const uint32_t defaultLocationInstanceIdx__ = 113U;
    const uint32_t defaultBodyCmdInstanceIdx_ = 1U;
    const uint32_t defaultChassisCmdInstanceIdx_ = 1U;
    uint32_t bodyReportInstanceIdx_;
    uint32_t chassisReportInstanceIdx_;
    uint32_t egoTrajectoryInstanceIdx_;
    uint32_t locationInstanceIdx_;
    uint32_t bodyCmdInstanceIdx_;
    uint32_t chassisCmdInstanceIdx_;
    std::unique_ptr<BodyReportReceiveBase> bodyReportRec;
    std::unique_ptr<ChassisReportReceiveBase> chassisReportRec;
    std::unique_ptr<LocationReceiveBase> locationRec;
    std::unique_ptr<EgoTrajectoryReceiveBase> egoTrajectoryRec;
    std::unique_ptr<BodyCmdSendBase> bodyCmdSend;
    std::unique_ptr<ChassisCmdSendBase> chassisCmdSend;
    std::vector<std::thread> subThreads;
};
}
#endif  // HAF_ADSF_CONTROL_BASE_H
