/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: HealthChannel definition
 * Create: 2020-3-27
 */
#ifndef ARA_PHM_HEALTH_CHANNEL_H
#define ARA_PHM_HEALTH_CHANNEL_H

#include <string>
#include <rtf/com/rtf_com.h>

#include "ara/core/instance_specifier.h"
#include "ara/core/result.h"

namespace ara {
namespace phm {
using HealthStatus = uint32_t;
using CommunicationMode = std::set<rtf::com::TransportMode>;
class HealthChannel {
public:
    explicit HealthChannel(ara::core::InstanceSpecifier const &instance);

    HealthChannel(const core::InstanceSpecifier &instance, std::string const &processName,
                  CommunicationMode const &communicationMode);

    ~HealthChannel() = default;

    ara::core::Result<void> ReportHealthStatus(HealthStatus healthStatusId);
private:
    ara::core::String instance_ {};
    std::string processName_ {};
    rtf::com::InstanceId instanceId_ {};
};
} // namespace phm
} // namespace ara
#endif

