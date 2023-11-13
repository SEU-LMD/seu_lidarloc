/*
 * Description: Camera 目标跟踪框架头文件
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 */
#ifndef HAF_ADSF_CAMERA_TRACKER_BASE_H
#define HAF_ADSF_CAMERA_TRACKER_BASE_H
#include <string>
#include <shared_mutex>
#include "adsfi_base.h"
#include "object_3d_receive_base.h"
#include "object_3d_send_base.h"
#include "adsf/camera_receive_base.h"
#include "adsf/location_receive_base.h"
#include "core/core.h"
#include "object/object.h"
namespace Adsfi {
class CameraTrackerBase : public AdsfiBase {
public:
    explicit CameraTrackerBase(const std::string configFile) : AdsfiBase(configFile) {};
    ~CameraTrackerBase() override;
    void Stop() override;
    HafStatus GetObject(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data);
    HafStatus GetImage(std::shared_ptr<ImageFrameV2> &data);
    HafStatus GetLocation(std::shared_ptr<HafLocation> &data);
    HafStatus SendObject(std::shared_ptr<Haf3dDetectionOutArray<float32_t>> &data);

    uint32_t GetLocationInsIdx() const;
    uint32_t GetObjInsIdx() const;
    uint32_t GetCameraInsIdx() const;
    uint32_t GetResultObjInsIdx() const;

    bool IsCameraActivate() const;
    bool IsLocationActivate() const;

protected:
    HafStatus StartSubThread() override;
    HafStatus StartPubThread() override;
    HafStatus ParsingInstanceId(const HafYamlNode &config) override;
private:
    bool IsInsIdxValid() const;
    const uint32_t defaultLocationInstanceIdx_ = 113U;
    const uint32_t inputCameraInstanceIdxMin_ = 21U;
    const uint32_t inputCameraInstanceIdxMax_ = 35U;
    const uint32_t trackRecInsIdxMin_ = 1001U;
    const uint32_t trackRecInsIdxMax_ = 1015U;
    const uint32_t trackSendInsIdxMin_ = 1101U;
    const uint32_t trackSendInsIdxMax_ = 1115U;

    uint32_t locationInstanceIdx_ {defaultLocationInstanceIdx_};
    uint32_t trackReceiveInstanceIdx_ { trackRecInsIdxMin_ };
    uint32_t trackSendInstanceIdx_ { trackSendInsIdxMin_ };
    uint32_t inputCameraInstanceIdx_ { inputCameraInstanceIdxMin_ };

    bool isCameraActivate_  { true };
    bool isLocationActivate_ { true };

    std::unique_ptr<LocationReceiveBase> locationRec {nullptr};
    std::unique_ptr<Object3DReceiveBase> objectRec { nullptr };
    std::unique_ptr<CameraReceiveBase> cameraRec { nullptr };
    std::unique_ptr<Object3DSendBase> objectSend { nullptr };
    std::vector<std::thread> threadPool_;
};
}
#endif
