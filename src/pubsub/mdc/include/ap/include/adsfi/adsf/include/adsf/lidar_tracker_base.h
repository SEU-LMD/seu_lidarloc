/* *
 * FUNCTION: Define Lidar Tracker Base Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *    */
#ifndef ADSF_LIDARTRACKERBASE_H
#define ADSF_LIDARTRACKERBASE_H

#include <string>
#include <map>
#include <vector>
#include <set>
#include <shared_mutex>
#include "core/core.h"
#include "object/object.h"
#include "adsfi_base.h"
#include "lidar_receive_base.h"
#include "object_3d_receive_base.h"
#include "object_3d_send_base.h"
#include "location/location.h"
#include "location_receive_base.h"

namespace Adsfi {
class LidarTrackerBase : public AdsfiBase {
public:
    explicit LidarTrackerBase(const std::string configFile) : AdsfiBase(configFile) {};
    ~LidarTrackerBase() override;
    void Stop() override;
    HafStatus GetDetectionObstacles(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data);
    HafStatus GetPointCloud(std::shared_ptr<LidarFrame<PointXYZIRT>> &data, const uint32_t instanceId);
    HafStatus GetLocation(std::shared_ptr<HafLocation> &locationData);
    HafStatus SendResult(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data);
    uint32_t GetObjInsIdx() const;
    std::vector<uint32_t> GetLidarInsIdx() const;
    uint32_t GetLocationInsIdx() const;
    uint32_t GetResultObjInsIdx() const;
    bool IsLidarActivated() const;
    bool IsLocationActivated() const;

protected:
    HafStatus StartSubThread() override;
    HafStatus StartPubThread() override;
    HafStatus ParsingInstanceId(const HafYamlNode &config) override;

private:
    bool IsInsIdxValid() const;
    const uint32_t obj3DInsIdxMin_ = 2001U;
    const uint32_t obj3DInsIdxMax_ = 2005U;
    const uint32_t object3DInsIdxMiniMin_ = 2011U;
    const uint32_t object3DInsIdxMiniMax_ = 2014U;
    const uint32_t trackOutInsIdxMin_ = 2101U;
    const uint32_t trackOutInsIdxMax_ = 2105U;
    const uint32_t lidarIdxMin_ = 3U;
    const uint32_t lidarIdxMax_ = 7U;
    const uint32_t defaultLocationIdx_ = 113U;

    uint32_t object3DIns_ {obj3DInsIdxMax_};
    std::vector<uint32_t> lidarIdx_;
    uint32_t trackingOutInstanceIdx_ {trackOutInsIdxMax_};
    uint32_t locationInstanceIdx_ {defaultLocationIdx_};
    bool isLidarActivate_ {true};
    bool isLocationActivate_ {true};

    std::unique_ptr<Object3DReceiveBase> objectRe_ {nullptr};
    std::map<uint32_t, std::unique_ptr<LidarReceiveBase>> lidarRe_;
    std::unique_ptr<Object3DSendBase> resultSe_ {nullptr};
    std::unique_ptr<LocationReceiveBase> locationRecv_ {nullptr};
    std::vector<std::thread> threadPool_ {};
};
}
#endif
