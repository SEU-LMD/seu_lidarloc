/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: define Log info print
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_UTILS_LOG_H
#define VRTF_VCC_UTILS_LOG_H

#include <string>
#include <map>
#include <memory>
#include "vrtf/vcc/api/types.h"
namespace vrtf {
namespace vcc {
namespace utils {
std::string SwitchDriverToString(api::types::DriverType driverType) noexcept;
void PrintServiceDiscoveryInfo(
    const std::map<api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::ServiceDiscoveryInfo>>& info);
void PrintEventInfo(
    const std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::EventInfo>>& info);
void PrintMethodInfo(
    const std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::MethodInfo>>& info);
void PrintFieldInfo(
    const std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::FieldInfo>>& info);
}
}
}
#endif
