/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This is use to implement global shut down.
 * Create: 2020-10-30
 */
#ifndef RTF_COM_GLOBAL_SHUTDOWN_H_
#define RTF_COM_GLOBAL_SHUTDOWN_H_
#include <mutex>
#include <map>
#include "rtf/com/adapter/ros_proxy_adapter.h"
#include "rtf/com/adapter/ros_skeleton_adapter.h"
namespace rtf {
namespace com {
namespace adapter {
class GlobalShutDown {
public:
    static std::shared_ptr<GlobalShutDown> &GetInstance();
    void AddRosSkeleton(const std::string &uri, const std::shared_ptr<adapter::RosSkeletonAdapter> &adapter);
    void AddRosProxy(const std::string &uri, const std::shared_ptr<adapter::RosProxyAdapter> &adapter);
    void DeleteRosSkeleton(const std::string &uri);
    void DeleteRosProxy(const std::string &uri);
    void Shutdown();
private:
    std::mutex globalSkeletonMapMutex_;
    std::mutex globalProxyMapMutex_;
    // The map to store all skeleton adn proxy in the process, key = nodeNamspace + nodeHandleNamespace
    std::unordered_map<std::string, std::shared_ptr<adapter::RosSkeletonAdapter>>  globalSkeletonMap_;
    std::unordered_map<std::string, std::shared_ptr<adapter::RosProxyAdapter>>  globalProxyMap_;
};
}
}
}
#endif