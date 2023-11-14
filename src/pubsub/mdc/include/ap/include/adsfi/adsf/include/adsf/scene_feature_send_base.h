/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: SceneFeatureSendBase.h SAMM特征发送AP消息基类
 * Author: xuke x00521274
 * Create: 2020-09-07
 */

#ifndef HAF_ADSF_SAMM_FEATURE_SEND_BASE_H
#define HAF_ADSF_SAMM_FEATURE_SEND_BASE_H

#include <shared_mutex>
#include "data_send_base.h"
#include "samm/samm_feature.h"
#include "ara/samm/sammfeatureserviceinterface_skeleton.h"
namespace Adsfi {
    class SceneFeatureSendBase : public DataSendBase<ara::samm::skeleton::SammFeatureServiceInterfaceSkeleton,
        HafSammFeature> {
    public:
        explicit SceneFeatureSendBase(const uint32_t instanceIdxIn) : DataSendBase(instanceIdxIn) {};
        virtual ~SceneFeatureSendBase();
        void SendingData() override;
    private:
        template<typename FromHeader, typename EndHeader>
        static void HeaderSendTranslate(const FromHeader &fromHeader, EndHeader &endHeader);
        template<typename FromHeader, typename EndHeader>
        static void FeatureSendTranslate(const FromHeader &fromFeature, EndHeader &endFeature);
    };
} // namespace Adsfi

#endif //  HAF_ADSF_SAMM_FEATURE_SEND_BASE_H