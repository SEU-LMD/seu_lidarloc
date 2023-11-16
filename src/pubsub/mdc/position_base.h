/*
 * @FilePath: position_base.h
 * @Author: yujie
 * @Date: 2023-05-06 10:42:35
 * @LastEditors: yujie yujie@shineauto.com.cn
 * @LastEditTime: 2023-10-27 13:53:00
 * Copyright: 2023 ShineAuto CO.,LTD. All Rights Reserved.
 * @Descripttion:
 */
#ifndef POSITION_BASE_H
#define POSITION_BASE_H

#include "adsf/adsfi_base.h"
#include "core/core.h"
#include "base/can_base.h"
#include "base/location_send_base.h"
#include "base/lidar_receive_base.h"

using namespace Adsfi;
using namespace shared;

namespace shineauto {
namespace position {

class PositionBase : public AdsfiBase {
  public:
    explicit PositionBase(const std::string configFile) : AdsfiBase(configFile){};
    ~PositionBase()
    {
      for (auto& it : threadPool_) {
          if (it.joinable()) {
              it.join();
          }
      }
    }
    void Stop()
    {
        stopFlag_ = true;
        canReceiver_->Stop();
        canEPSReceiver_->Stop();
        lidarRecv_->Stop();
        locationSender_->Stop();
    }

HafStatus GetPosition(std::shared_ptr<HafCanRxFrame>& data)
{
    return canReceiver_->GetLastOneBlocking(data, true);
}

HafStatus  GetCanEPSData(std::shared_ptr<HafCanRxFrame>& data)
{
    return canEPSReceiver_->GetLastOneBlocking(data, true);
}

HafStatus  GetLidar(std::shared_ptr<LidarFrameV2>& data, const uint32_t timeout)
{
    return lidarRecv_->GetOneData(data, timeout);
}

HafStatus  SendLocation(std::shared_ptr<HafLocationV2>& data)
{
    return locationSender_->SendOneData(data);
}

protected:
HafStatus StartSubThread()
{
    canReceiver_->StartFindService();
    canEPSReceiver_->StartFindService();
    lidarRecv_->StartFindService();
    return HAF_SUCCESS;
}

HafStatus StartPubThread()
{
    threadPool_.push_back(std::thread(&LocationSendBase::SendingData, locationSender_.get()));
    return HAF_SUCCESS;
}

HafStatus ParsingInstanceId(const HafYamlNode& config)
{
    canRecvInsIdx_ = 7;
    recvLidarInstanceIdx_ = 13;
    locationResultInstanceIdx_ = 113;

    auto recvInstanceID = config["recvInstanceID"];
    recvInstanceID.GetValue("canInputInstanceID", canRecvInsIdx_);
    recvInstanceID.GetValue("recvLidarInstanceID", recvLidarInstanceIdx_);

    auto sendInstanceID = config["sendInstanceID"];
    sendInstanceID.GetValue("locationResultInstanceID", locationResultInstanceIdx_);

    canReceiver_ = std::make_unique<CanBase>(canRecvInsIdx_);
    canEPSReceiver_ = std::make_unique<CanBase>(8);
    lidarRecv_ = std::make_unique<LidarReceiveBase>(recvLidarInstanceIdx_);
    locationSender_ = std::make_unique<LocationSendBase>(locationResultInstanceIdx_);
    return HAF_SUCCESS;
}

  private:
    uint32_t canRecvInsIdx_ = 7;
    uint32_t recvLidarInstanceIdx_ = 13;
    uint32_t locationResultInstanceIdx_ = 113;

    std::unique_ptr<CanBase> canReceiver_{ nullptr };
    std::unique_ptr<CanBase> canEPSReceiver_{ nullptr };
    std::unique_ptr<LidarReceiveBase> lidarRecv_{ nullptr };
    std::unique_ptr<LocationSendBase> locationSender_{ nullptr };

    std::vector<std::thread> threadPool_;
};

}  // namespace position
}  // namespace shineauto

#endif
