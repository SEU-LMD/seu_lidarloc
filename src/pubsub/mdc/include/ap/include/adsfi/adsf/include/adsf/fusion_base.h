/* *
 * FUNCTION: Define FusionBase Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * */
#ifndef ADSF_FUSIONBASE_H
#define ADSF_FUSIONBASE_H

#include <string>
#include <vector>
#include "core/core.h"
#include "object/object.h"
#include "adsf/adsfi_base.h"
#include "location/location.h"
#include "adsf/location_receive_base.h"
#include "adsf/fusion_object_send_base.h"
#include "adsf/object_3d_receive_base.h"
namespace Adsfi {
class FusionBase : public AdsfiBase {
public:
    explicit FusionBase(const std::string configFile) : AdsfiBase(configFile) {};
    ~FusionBase() override;
    void Stop() override;

    HafStatus GetLocation(std::shared_ptr<HafLocation> &data);
    HafStatus GetPrefusion(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data, const uint32_t instanceIdx);
    HafStatus GetRadar(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data, const uint32_t instanceIdx);
    HafStatus GetLidar(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data, const uint32_t instanceIdx);
    HafStatus GetCamera(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data, const uint32_t instanceIdx);
    HafStatus SendObject(std::shared_ptr<HafFusionOutArray<float32_t>> &data);

    uint32_t GetLocInsIdx() const;
    std::vector<uint32_t> GetPrefusionInsIdx() const;
    std::vector<uint32_t> GetLidarInsIdx() const;
    std::vector<uint32_t> GetCameraInsIdx() const;
    std::vector<uint32_t> GetRadarInsIdx() const;
    uint32_t GetResultObjInsIdx() const;

protected:
    HafStatus StartSubThread() override;
    HafStatus StartPubThread() override;
    HafStatus ParsingInstanceId(const HafYamlNode &config) override;
private:
    bool IsInsIdxValid() const;
    void ValidInputNum();
    HafStatus GetIns(const HafYamlNode &config);
    HafStatus GetRecvIns(const HafYamlNode &config, std::vector<uint32_t> &instanceIdx, const int32_t sensorNum,
        const std::vector<uint32_t> instanceRange, const std::string sensorIdName);
    const uint32_t radarInsIdxMin_ = 3001U;
    const uint32_t radarInsIdxMax_ = 3006U;
    const uint32_t lidarInsIdxMin_ = 2001U;
    const uint32_t lidarInsIdxMax_ = 2005U;
    const uint32_t lidarInsIdxMiniMin_ = 2011U;
    const uint32_t lidarInsIdxMiniMax_ = 2014U;
    const uint32_t lidarTrackInsIdxMin_ = 2101U;
    const uint32_t lidarTrackInsIdxMax_ = 2105U;
    const uint32_t cameraInsIdxMin_ = 1001U;
    const uint32_t cameraInsIdxMax_ = 1015U;
    const uint32_t cameraTrackInsIdxMin_ = 1101U;
    const uint32_t cameraTrackInsIdxMax_ = 1115U;
    const uint32_t preFusionInsIdxMin_ = 1501U;
    const uint32_t preFusionInsIdxMax_ = 1515U;
    const uint32_t defaultLocationInstanceIdx_ = 113U;
    const uint32_t defaultFusionOutInstanceIdx_ = 4001U;

    const int32_t radarNumMax_ = 6;
    const int32_t lidarNumMax_ = 5;
    const int32_t cameraNumMax_ = 15;
    const int32_t prefusionNumMax_ = 15;

    uint32_t locationInstanceIdx_ { defaultLocationInstanceIdx_ };
    std::vector<uint32_t> prefusionInstanceIdx_;
    std::vector<uint32_t> lidarInstanceIdx_;
    std::vector<uint32_t> cameraInstanceIdx_;
    std::vector<uint32_t> radarInstanceIdx_;
    uint32_t fusionOutInstanceIdx_ { defaultFusionOutInstanceIdx_ };

    int32_t radarNum_ {radarNumMax_};
    int32_t lidarNum_ {lidarNumMax_};
    int32_t cameraNum_ {cameraNumMax_};
    int32_t prefusionNum_ {prefusionNumMax_};

    std::unique_ptr<LocationReceiveBase> locationRecv_ { nullptr };
    std::map<uint32_t, std::unique_ptr<Object3DReceiveBase>> radarRecv_;
    std::map<uint32_t, std::unique_ptr<Object3DReceiveBase>> lidarRecv_;
    std::map<uint32_t, std::unique_ptr<Object3DReceiveBase>> cameraRecv_;
    std::map<uint32_t, std::unique_ptr<Object3DReceiveBase>> prefusionRecv_;
    std::unique_ptr<FusionObjectSendBase> resultSend_ { nullptr };

    std::vector<std::thread> threadPool_;
};
}
#endif
