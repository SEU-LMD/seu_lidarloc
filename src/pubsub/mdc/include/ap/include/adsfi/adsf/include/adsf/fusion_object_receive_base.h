/* *
 * FUNCTION: Define FusionObjectReceiveBase Class.
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * */
#ifndef ADSF_FUSIONOBJECTRECEIVEBASE_H
#define ADSF_FUSIONOBJECTRECEIVEBASE_H

#include "adsf/data_receive_base.h"
#include "core/types.h"
#include "object/object.h"
#include "mdc/adsfi/objectarrayinterface_proxy.h"

#include <shared_mutex>
namespace Adsfi {
class FusionObjectReceiveBase : public DataReceiveBase<mdc::adsfi::proxy::ObjectArrayInterfaceProxy,
    mdc::adsfi::proxy::ObjectArrayInterfaceProxy::HandleType, HafFusionOutArray<float32_t>> {
public:
    explicit FusionObjectReceiveBase(const uint32_t instanceIdx) : DataReceiveBase(instanceIdx) {};
    virtual ~FusionObjectReceiveBase();
    void RegisterHandle() override;
    void OnDataReceive();

private:
    void ObjectDataAssign(const ara::adsfi::ObjectArray &fusionSample, HafFusionOutArray<float32_t> &data) const;
    void SetVel(HafFusionOut<float32_t> &resultTmp, const ara::adsfi::Object it) const;
    void SetData(HafFusionOut<float32_t> &resultTmp, const ara::adsfi::Object it) const;
    void SetRect(HafFusionOut<float32_t> &resultTmp, const ara::adsfi::Object it) const;
};
}
#endif
