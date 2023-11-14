/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:  SceneManageBase.h 场景管理的基类
 * Create: 2020-09-15
 */
#ifndef HAF_ADSF_SCENE_MANAGE_BASE_H
#define HAF_ADSF_SCENE_MANAGE_BASE_H

#include <shared_mutex>
#include "adsf/adsfi_base.h"
#include "core/core.h"
#include "adsf/scene_feature_receive_base.h"
#include "adsf/samm_scene_send_base.h"
#include "adsf/chassis_report_receive_base.h"

namespace Adsfi {
    class SceneManageBase : public AdsfiBase {
    public:
        explicit SceneManageBase(const std::string configFile)
            : AdsfiBase(configFile),
            sammFeatureInstanceIdx_(6663U),
            sammSceneOutInstanceIdx_(6664U),
            chassisReportInstanceIdx_(1U) {};
        ~SceneManageBase() override;
        void Stop() override;
        std::vector<std::shared_ptr<HafSammFeature>> GetFeature(const size_t &length) const;
        std::vector<std::shared_ptr<HafChassisReport>> GetChassis(const size_t &length) const;
        HafStatus SendResult(std::shared_ptr<HafSammScene> &data);
        uint32_t GetFeatureIdx() const;
        uint32_t GetChassisIdx() const;
        uint32_t GetSceneIdx() const;
    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
        bool IsInstanceIdxValid() const;
        std::vector<std::thread> threadPool_;
        const uint32_t defaultSammFeatureInstanceIdx_{6663U};
        const uint32_t defaultSammSceneOutInstanceIdx_{6664U};
        const uint32_t defaultChassisReportInstanceIdx_{1U};
        uint32_t sammFeatureInstanceIdx_;    // 读取特征instanceId
        uint32_t sammSceneOutInstanceIdx_;   // 发送场景instanceId
        uint32_t chassisReportInstanceIdx_;  // 读取车身instanceId

        std::unique_ptr<SceneFeatureReceiveBase> featureRecv_{nullptr};
        std::unique_ptr<ChassisReportReceiveBase> chassisRecv_{nullptr};
        std::unique_ptr<SammSceneSendBase> resultSend_{nullptr};
    };
}
#endif // HAF_ADSF_SCENE_MANAGE_BASE_H
