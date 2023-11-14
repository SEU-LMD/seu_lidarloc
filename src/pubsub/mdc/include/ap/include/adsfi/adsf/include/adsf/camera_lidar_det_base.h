/* *
 * FUNCTION: Define CameraLidar Detection Base Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *  */
#ifndef ADSF_CAMERALIDARDETBASE_H
#define ADSF_CAMERALIDARDETBASE_H
#include <shared_mutex>
#include <string>
#include <map>
#include "adsfi_base.h"
#include "core/core.h"
#include "camera_receive_base.h"
#include "lidar_receive_base.h"
#include "location/location.h"
#include "location_receive_base.h"
#include "object/object.h"
#include "object_3d_receive_base.h"
#include "object_3d_send_base.h"
namespace Adsfi {
    class CameraLidarDetBase : public AdsfiBase {
    public:
        explicit CameraLidarDetBase(const std::string configFile)
            :   AdsfiBase(configFile) {};
        ~CameraLidarDetBase() override;
        void Stop() override;
        // 阻塞式调用
        HafStatus GetImage(std::shared_ptr<ImageFrameV2>& data, const uint32_t instanceIdx);
        HafStatus GetLidar(std::shared_ptr<LidarFrame<PointXYZIRT> > &data, const uint32_t instanceIdx);
        HafStatus GetObject(std::shared_ptr<Haf3dDetectionOutArray<float32_t> > &data);
        HafStatus GetLocation(std::shared_ptr<HafLocation> &data);
        HafStatus SendObject(std::shared_ptr<Haf3dDetectionOutArray<float32_t> > &data);
        std::vector<uint32_t> GetLidarInsIdx() const;
        std::vector<uint32_t> GetCameraInsIdx() const;
        uint32_t GetObjInsIdx() const;
        uint32_t GetLocInsIdx() const;
        uint32_t GetResultObjInsIdx() const;
        bool IsCameraActivate() const;
        bool IsLidarActivate() const;
        bool IsObjectActivate() const;
        bool IsLocationActivate() const;
    protected:
        HafStatus StartSubThread() override;
        HafStatus StartPubThread() override;
        HafStatus ParsingInstanceId(const HafYamlNode& config) override;
    private:
        HafStatus ParsingCameraIdx(const HafYamlNode& recvConfig);
        HafStatus ParsingLidarIdx(const HafYamlNode& recvConfig);
        HafStatus ParsingActivate(const HafYamlNode& config);
        bool IsInsIdxValid() const;
        bool IsObjectInsIdxValid() const;
        bool IsLidarInsIdxValid() const;
        bool IsCameraInsIdxValid() const;
        const uint32_t camInsIdxMin_{21U};
        const uint32_t camInsIdxMax_{35U};
        const uint32_t objectIdxMin_ = 1001U;
        const uint32_t objectIdxMax_ = 1015U;
        const uint32_t objectTrackIdxMin_ = 1101U;
        const uint32_t objectTrackIdxMax_ = 1115U;
        const uint32_t detOutInsIdxMin_ = 1501U;
        const uint32_t detOutInsIdxMax_ = 1515U;
        const uint32_t lidarIdxMin_ = 3U;
        const uint32_t lidarIdxMax_ = 7U;
        const uint32_t defaultLocationIdx_ = 113U;

        std::vector<uint32_t> lidarIdx_;
        std::vector<uint32_t> cameraIdx_;
        uint32_t objectIdx_{objectTrackIdxMin_};
        uint32_t detectionOutInstanceIdx_{detOutInsIdxMin_};
        uint32_t locationIdx_{defaultLocationIdx_};

        bool isCameraActivate_ = false;
        bool isLidarActivate_ = true;
        bool isObjectActivate_ = true;
        bool isLocationActivate_ = true;

        std::map<uint32_t, std::unique_ptr<LidarReceiveBase>> lidarRecv_;
        std::map<uint32_t, std::unique_ptr<CameraReceiveBase>> cameraRecv_;
        std::unique_ptr<Object3DReceiveBase> objectRecv_{nullptr};
        std::unique_ptr<LocationReceiveBase> locationRecv_{nullptr};
        std::unique_ptr<Object3DSendBase> resultSend_{nullptr};
        std::vector<std::thread> threadPool_;
    };
}
#endif