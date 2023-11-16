
#ifndef LIDAR_RECEIVE_BASE_H
#define LIDAR_RECEIVE_BASE_H

#include <shared_mutex>
#include "core/types.h"
#include "ara/lidar/lidarserviceinterface_proxy.h"
#include "data_receive_base_v1.h"
#include "sa_types.h"

using Adsfi::LidarFrameV2;
using Adsfi::DataReceiveBaseV1;

class LidarReceiveBase
    : public DataReceiveBaseV1<ara::lidar::proxy::LidarServiceInterfaceProxy,
                            ara::lidar::proxy::LidarServiceInterfaceProxy::HandleType, LidarFrameV2> {
public:
    explicit LidarReceiveBase(const uint32_t instanceIdx)
        : DataReceiveBaseV1(instanceIdx) {};
    virtual ~LidarReceiveBase();
    void RegisterHandle() override;
    void OnDataReceive();
};

#endif
