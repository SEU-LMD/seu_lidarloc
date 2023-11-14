/* *
 * FUNCTION: Define LanesSend Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
*  */

#ifndef HAF_ADSF_SAMM_SCENE_SEND_BASE_H
#define HAF_ADSF_SAMM_SCENE_SEND_BASE_H

#include <shared_mutex>
#include "core/types.h"
#include "data_send_base.h"
#include "samm/samm_scene.h"
#include "ara/samm/sammsceneserviceinterface_skeleton.h"
namespace Adsfi {
class SammSceneSendBase : public DataSendBase<ara::samm::skeleton::SammSceneServiceInterfaceSkeleton,
    HafSammScene> {
public:
    explicit SammSceneSendBase(const uint32_t instanceIdxIn) : DataSendBase(instanceIdxIn) {};
    virtual ~SammSceneSendBase();
    void SendingData() override;
private:
    template<typename SammScene, typename CmScene> void UpdateDagData(SammScene &sammScene, CmScene &data) const;
};
}
#endif // HAF_ADSF_SAMM_SCENE_SEND_BASE_H
