/*
  * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
+ * Description: Lidar detection
+ */
#ifndef ADSF_LIDARDETBASE_H
#define ADSF_LIDARDETBASE_H

#include <shared_mutex>
#include <string>
#include <set>
#include "adsfi_base.h"
#include "core/core.h"
#include "lidar_receive_base.h"
#include "location/location.h"
#include "location_receive_base.h"
#include "object/object.h"
#include "object_3d_send_base.h"

namespace Adsfi {
    class LidarDetBase : public AdsfiBase {
    public:
        explicit LidarDetBase(const std::string configFile)
            : AdsfiBase(configFile) {};
        ~LidarDetBase() override;
        void Stop() override;
        HafStatus GetLidar(std::shared_ptr<LidarFrame<PointXYZIRT>> &data, const uint32_t instanceId,
            const uint32_t timeout = UINT32_MAX);
        HafStatus GetLocation(std::shared_ptr<HafLocation> &data);
        HafStatus SendObject(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data);

        std::vector<uint32_t> GetLidarInsIdx() const;
        uint32_t GetLocationInsIdx() const;
        uint32_t GetSendObjInsIdx() const;

    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
        bool IsIntanceIdValid() const;

        const uint32_t lidarInsIdxMin_ = 3U;
        const uint32_t lidarInsIdxMax_ = 7U;
        const uint32_t detOutInsIdxMin_ = 2001U;
        const uint32_t detOutInsIdxMax_ = 2005U;
        const uint32_t defaultLocationIdx_ = 113U;
        std::vector<uint32_t> lidarInstanceIdx_;
        uint32_t locationInstanceIdx_ = 113U;
        uint32_t detectionOutInstanceIdx_ = 2005U;

        bool isLocationActivate_ = true;

        std::map<uint32_t, std::unique_ptr<LidarReceiveBase>> lidarRecv_;
        std::unique_ptr<Object3DSendBase> objectSend_{nullptr};
        std::unique_ptr<LocationReceiveBase> locationRecv_{nullptr};
        std::vector<std::thread> threadPool_;
    };
}
#endif
