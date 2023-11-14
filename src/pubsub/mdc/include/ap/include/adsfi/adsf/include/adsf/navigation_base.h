/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  NavigationBase.h 全局导航框架基类
 */

#ifndef HAF_ADSF_NAVIGATION_RESULT_H
#define HAF_ADSF_NAVIGATION_RESULT_H

#include <shared_mutex>
#include "adsfi_base.h"
#include "navigation/navigation_result.h"
#include "navigation/select_point.h"
#include "location/location.h"
#include "adsf/navigation_result_send_base.h"
#include "adsf/location_receive_base.h"
#include "adsf/select_point_receive_base.h"
namespace Adsfi {
class NavigationBase : public AdsfiBase {
public:
    explicit NavigationBase(const std::string configFile)
        : AdsfiBase(configFile),
        locationInstanceIdx_(113U),
        selectPointInstanceIdx_(198U),
        navigationResultOutInstanceIdx_(110U) {};
    ~NavigationBase() override;
    void Stop() override;
    std::vector<std::shared_ptr<HafLocation>> GetNLocationData(const size_t& num);
    std::vector<std::shared_ptr<HafSelectPoint>> GetNSelectPointData(const size_t& num);
    HafStatus SendNavigationResult(std::shared_ptr<HafNavigationResult>& data);
    uint32_t GetLocationInsIdx() const;
    uint32_t GetSelectPointInsIdx() const;
    uint32_t GetNavigationResultInsIdx() const;

protected:
    HafStatus StartSubThread() override;
    HafStatus StartPubThread() override;
    HafStatus ParsingInstanceId(const HafYamlNode& config) override;
private:
    bool IsInsIdxValid() const;
    const uint32_t defaultLocationInstanceIdx_{113U};
    const uint32_t defaultSelectPointInstanceIdx_{198U};
    const uint32_t defaultNavigationResultOutInstanceIdx_{110U};

    uint32_t locationInstanceIdx_;
    uint32_t selectPointInstanceIdx_;
    uint32_t navigationResultOutInstanceIdx_;

    std::unique_ptr<LocationReceiveBase> locationRecv_;
    std::unique_ptr<SelectPointReceiveBase> selectPointRecv_;
    std::unique_ptr<NavigationResultSendBase> navigationResultSend_;
    std::vector<std::thread> threadPool_;
};
}
#endif // HAF_ADSF_NAVIGATION_RESULT_H
