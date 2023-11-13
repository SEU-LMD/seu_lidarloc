/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:  SceneFeatureReceiveBase 用于接收SAMM场景管理的特征数据
 * Create: 2020-06-05
 */


#ifndef ADSF_SCENE_FEATURE_RECEIVE_BASE_H
#define ADSF_SCENE_FEATURE_RECEIVE_BASE_H

#include <shared_mutex>
#include "data_receive_base.h"
#include "samm/samm_feature.h"
#include "ara/samm/sammfeatureserviceinterface_proxy.h"
namespace Adsfi {
    class SceneFeatureReceiveBase : public DataReceiveBase<ara::samm::proxy::SammFeatureServiceInterfaceProxy,
        ara::samm::proxy::SammFeatureServiceInterfaceProxy::HandleType, HafSammFeature> {
    public:
        explicit SceneFeatureReceiveBase(const uint32_t instanceID) : DataReceiveBase(instanceID) {};
        void RegisterHandle() override;
        void OnDataReceive();
        virtual ~SceneFeatureReceiveBase();
    private:
        template <typename FromHeader, typename EndHeader>
        static void SceneHeaderTranslate(const FromHeader &fromHeader, EndHeader &endHeader);
        template <typename FromHeader, typename EndHeader>
        static void FeatureReceiveTranslate(const FromHeader &fromFeature, EndHeader &endFeature);
    };
}
#endif // ADSF_SCENE_FEATURE_RECEIVE_BASE_H
