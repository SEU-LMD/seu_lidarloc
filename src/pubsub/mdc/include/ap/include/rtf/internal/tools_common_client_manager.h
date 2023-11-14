/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: The definition of ToolsCommonClientManager, which is use to create common proxy of rtftools
 * Create: 2021-03-11
 */
#ifndef RTF_TOOLS_COMMON_CLIENT_MANAGER_H
#define RTF_TOOLS_COMMON_CLIENT_MANAGER_H

#include <memory>
#include <mutex>
#include "rtf/internal/RtfLog.h"
#include "rtf/maintaind/rtfmaintaindtoolsservice_proxy.h"
namespace rtf {
namespace rtftools {
namespace common {
using RTFMaintaindToolsServiceProxy = rtf::maintaind::proxy::RTFMaintaindToolsServiceProxy;
class ToolsCommonClientManager {
public:
    ToolsCommonClientManager();
    ~ToolsCommonClientManager();
    static std::shared_ptr<ToolsCommonClientManager>& GetInstance();
    void FindMaintaindService();
    ara::core::Vector<std::shared_ptr<RTFMaintaindToolsServiceProxy>>  GetCurrentProxyList();
    size_t GetProxyListSize();
private:
    void ServiceAvailabilityCallback(
        const ara::com::ServiceHandleContainer<RTFMaintaindToolsServiceProxy::HandleType>& handles,
        const ara::com::FindServiceHandle& handler);
    void PrintOnlineMaintaindService();
    void StopFindMaintaindService() noexcept;
    void UpdateAndCreateProxyList() noexcept;
    ara::core::Vector<std::shared_ptr<RTFMaintaindToolsServiceProxy>> proxyList_;
    std::set<RTFMaintaindToolsServiceProxy::HandleType> handles_;
    std::mutex proxyListMutex_;
    std::once_flag flag_;
    bool isCallFindService_ {false};
    ara::com::FindServiceHandle findServiceHandle_;
    std::shared_ptr<rtf::RtfLog::Log> logger_ = nullptr;
};
}
}
}
#endif
