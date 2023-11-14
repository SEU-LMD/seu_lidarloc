/* *
 * FUNCTION: Define FusionObjectSend Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * */
#ifndef ADSF_FUSIONOBJECTSENDBASE_H
#define ADSF_FUSIONOBJECTSENDBASE_H

#include <shared_mutex>
#include "adsf/data_send_base.h"
#include "core/types.h"
#include "object/object.h"
#include "mdc/adsfi/objectarrayinterface_skeleton.h"
namespace Adsfi {
class FusionObjectSendBase
    : public DataSendBase<mdc::adsfi::skeleton::ObjectArrayInterfaceSkeleton, HafFusionOutArray<float32_t>> {
public:
    explicit FusionObjectSendBase(const uint32_t idx) : DataSendBase(idx) {};
    ~FusionObjectSendBase() override;
    void SendingData() override;

private:
    const size_t CORNERS_NUM = 8U;
    void SetData(ara::adsfi::Object &resultTmp, const HafFusionOut<float32_t> it) const;
    void SetBox(ara::adsfi::Object &resultTmp, const HafFusionOut<float32_t> it) const;
    void SetVel(ara::adsfi::Object &resultTmp, const HafFusionOut<float32_t> it) const;
    void ObjectDataAssign(ara::adsfi::ObjectArray &objArray,
        const std::shared_ptr<HafFusionOutArray<float32_t>> &data) const;
};
}
#endif
