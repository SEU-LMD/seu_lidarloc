/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_CM_ADAPTER_INTERNAL_TYPES_H_
#define ARA_COM_CM_ADAPTER_INTERNAL_TYPES_H_
#include <cstdint>
#include "vrtf/vcc/api/types.h"

namespace ara {
namespace com {
namespace internal {
using InstanceId = vrtf::vcc::api::types::InstanceId;
using ServiceId = vrtf::vcc::api::types::ServiceId;
using EntityId = vrtf::vcc::api::types::EntityId;
using MajorVersionId = vrtf::vcc::api::types::MajorVersionId;
using MinorVersionId = vrtf::vcc::api::types::MinorVersionId;
using VersionInfo = vrtf::vcc::api::types::VersionInfo;
using ServiceNameType = std::string;
InstanceId const UNDEFINED_INSTANCEID = vrtf::vcc::api::types::UNDEFINED_INSTANCEID;
InstanceId const ANY_INSTANCEID = vrtf::vcc::api::types::ANY_INSTANCEID;
constexpr ServiceId UNDEFINED_SERVICEID = vrtf::vcc::api::types::UNDEFINED_SERVICEID;
constexpr ServiceId ANY_SERVICEID = vrtf::vcc::api::types::ANY_SERVICEID;
constexpr std::uint32_t UNDEFINED_UID = vrtf::vcc::api::types::UNDEFINED_UID;
constexpr EntityId UNDEFINED_ENTITYID = vrtf::vcc::api::types::UNDEFINED_ENTITYID;
constexpr const char* UNDEFINED_SERVICE_NAME = "UNDEFINED_SERVICE";
constexpr MajorVersionId ANY_MAJOR_VERSIONID = vrtf::vcc::api::types::ANY_MAJOR_VERSIONID;
constexpr MinorVersionId ANY_MINOR_VERSIONID = vrtf::vcc::api::types::ANY_MINOR_VERSIONID;

using ServiceAvailableHandler = vrtf::vcc::api::types::ServiceAvailableHandler;
class BindIndex {
public:
    ServiceNameType serviceName_ = std::string(UNDEFINED_SERVICE_NAME);
    ServiceId serviceId_ = UNDEFINED_SERVICEID;
    InstanceId instanceId_ = UNDEFINED_INSTANCEID;
    BindIndex(const ServiceNameType& serviceName, const InstanceId& instanceId,
              ServiceId serviceId = UNDEFINED_SERVICEID)
        : serviceName_(serviceName), serviceId_(serviceId), instanceId_(instanceId)
        {}
    BindIndex() = delete;
    ~BindIndex(void) = default;
    bool operator<(const BindIndex& index) const
    {
        if (serviceName_ < index.serviceName_) {
            return true;
        } else if (serviceName_ == index.serviceName_) {
            if (instanceId_ < index.instanceId_) {
                return true;
            }
        } else {
            // do nothing
        }
        return false;
    }
    bool operator==(const BindIndex& index) const
    {
        if ((serviceName_ == index.serviceName_) &&
            (instanceId_ == index.instanceId_)) {
            return true;
        }
        return false;
    }
};
}
}
}
#endif
