/* *
 * FUNCTION: Define Object3DSend Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *   */
#ifndef ADSF_OBJECT3DSENDBASE_H
#define ADSF_OBJECT3DSENDBASE_H

#include <functional>
#include <shared_mutex>
#include "data_send_base.h"
#include "core/types.h"
#include "object/object.h"
#include "mdc/adsfi/objectarrayinterface_skeleton.h"

namespace Adsfi {
class Object3DSendBase
    : public DataSendBase<mdc::adsfi::skeleton::ObjectArrayInterfaceSkeleton, Haf3dDetectionOutArray<float32_t>> {
public:
    explicit Object3DSendBase(const uint32_t idx) : DataSendBase(idx){};
    ~Object3DSendBase() override;
    void SendingData() override;
};
}  // namespace Adsfi

#endif
