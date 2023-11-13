/* *
 * FUNCTION: Define ObjectReceive Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *   */
#ifndef ADSF_OBJECT3DRECEIVEBASE_H
#define ADSF_OBJECT3DRECEIVEBASE_H

#include "data_receive_base.h"
#include "core/types.h"
#include "object/object.h"

#include <shared_mutex>

#include "mdc/adsfi/objectarrayinterface_proxy.h"
namespace Adsfi {
class Object3DReceiveBase
    : public DataReceiveBase<mdc::adsfi::proxy::ObjectArrayInterfaceProxy,
          mdc::adsfi::proxy::ObjectArrayInterfaceProxy::HandleType, Haf3dDetectionOutArray<float32_t>> {
public:
    explicit Object3DReceiveBase(const uint32_t instanceIdx0, const int64_t time2live = 0, const size_t capLimit = 5U)
        : DataReceiveBase(instanceIdx0, time2live, capLimit){};
    virtual ~Object3DReceiveBase();
    void RegisterHandle() override;
    void OnDataReceive();

private:
    void ObjectDataAssign(const ara::adsfi::ObjectArray& objSample, Haf3dDetectionOutArray<float32_t>& data);
};
}  // namespace Adsfi
#endif
