/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: camera detection
 */
#ifndef ADSF_CAMERADETBASE_H
#define ADSF_CAMERADETBASE_H

#include "core/core.h"
#include "object/object.h"
#include "adsfi_base.h"
#include "camera_receive_base.h"
#include "object_3d_send_base.h"
#include "data_send_base.h"
#include "yaml/haf_yaml.h"
#include <string>
#include <shared_mutex>
#include <vector>
#include <map>
#include <memory>

namespace Adsfi {
class CameraDetBase : public AdsfiBase {
public:
    explicit CameraDetBase(const std::string configFile)
        : AdsfiBase(configFile), detectionOutInstanceIdx_(detOutInsIdxMin_){};
    ~CameraDetBase() override;
    void Stop() override;
    /**
     * @brief 获取接收到的Image数据，阻塞式接收，当没有新的消息到来时，线程会一直阻塞。
     * 使用时建议单独启用一路线程，结合IsStop()进行使用，确保线程可以获取到停止信号正常退出。
     * 获取到图片时，是最新图片。
     *
     * @param data 图片指针，返回状态为HAF_SUCCESS时，指针有效
     * @param instanceIdx
     * @return HafStatus
     */
    HafStatus GetImage(
        std::shared_ptr<ImageFrameV2>& data, const uint32_t instanceIdx, const uint32_t timeout = UINT32_MAX);
    /**
     * @brief 接收图片数据，采用阻塞式调用。获取到图片时，是最新图片。
     *
     * @param instanceIdx 接收的instance id
     * @return std::shared_ptr<ImageFrameV2> 图片指针。异常时，指针为nullptr
     */
    std::shared_ptr<ImageFrameV2> GetImage(const uint32_t instanceIdx, const uint32_t timeout = UINT32_MAX);
    /**
     * @brief 发送Object数据，该函数会通知内部的线程进行AP消息的发送。
     *
     * @param data 数据指针
     * @param instanceIdx 发出的instance id
     * @return HafStatus
     */
    HafStatus SendObject(std::shared_ptr<Haf3dDetectionOutArray<float32_t>>& data, const uint32_t instanceIdx);
    /**
     * @brief 获取配置中接收的Camera消息对应的Instance ID列表
     *
     * @return std::vector<uint32_t>
     */
    std::vector<uint32_t> GetImageInsIdx() const;
    /**
     * @brief 获取发送的object消息对应的Instance ID
     *
     * @return uint32_t
     */
    uint32_t GetResultObjInsIdx() const;

protected:
    HafStatus StartSubThread() override;
    HafStatus StartPubThread() override;
    HafStatus ParsingInstanceId(const HafYamlNode& config) override;

private:
    bool IsInsIdxValid() const;
    const uint32_t camInsIdxMin_{21U};
    const uint32_t camInsIdxMax_{35U};
    const uint32_t detOutInsIdxMin_{1001U};
    const uint32_t detOutInsIdxMax_{1015U};

    int64_t cameraTimeoutThreshold{0};
    size_t cameraBufferSize{5U};

    std::vector<uint32_t> cameraInstanceIdx_;
    // detectionOutInstanceIdx_ was used in both object detection and traffic light detection
    uint32_t detectionOutInstanceIdx_{detOutInsIdxMin_};

    std::map<uint32_t, std::unique_ptr<CameraReceiveBase>> cameraRecv_;
    std::map<uint32_t, std::unique_ptr<Object3DSendBase>> objectSend_;

    std::vector<std::thread> threadPool_;
};
};  // namespace Adsfi
#endif
