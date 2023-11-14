/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: SupervisedEntity definition
 * Create: 2020-3-27
 */
#ifndef ARA_PHM_SUPERVISED_ENTITY_H
#define ARA_PHM_SUPERVISED_ENTITY_H

#include <string>
#include <rtf/com/rtf_com.h>

#include "ara/core/instance_specifier.h"
#include "ara/core/result.h"

namespace ara {
namespace phm {
using Checkpoint = uint32_t;

enum class LocalSupervisionStatus : uint32_t {
    DEINIT      = 0U,
    K_DEACTIVATED = 1U,
    K_OK          = 2U,
    K_FAILED      = 3U,
    K_EXPIRED     = 4U
};

enum class GlobalSupervisionStatus : uint32_t {
    DEINIT      = 0U,
    K_DEACTIVATED = 1U,
    K_OK          = 2U,
    K_FAILED      = 3U,
    K_EXPIRED     = 4U,
    K_STOPPED     = 5U
};
using CommunicationMode = std::set<rtf::com::TransportMode>;
class SupervisedEntity {
public:
    explicit SupervisedEntity(ara::core::InstanceSpecifier const &instance);

    SupervisedEntity(const core::InstanceSpecifier &instance, std::string const &processName,
                     CommunicationMode const &communicationMode);

    ~SupervisedEntity() = default;

    ara::core::Result<void> ReportCheckpoint(Checkpoint checkpointId);

    ara::core::Result<LocalSupervisionStatus> GetLocalSupervisionStatus() const;

    ara::core::Result<GlobalSupervisionStatus> GetGlobalSupervisionStatus() const;
private:
    ara::core::String instance_ {};
    std::string processName_ {};
    rtf::com::InstanceId instanceId_ {};
};
} // namespace phm
} // namespace ara
#endif

