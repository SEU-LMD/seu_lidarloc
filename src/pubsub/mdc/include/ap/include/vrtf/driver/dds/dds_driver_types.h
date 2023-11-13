/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: Types of dds driver
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_DRIVER_DDS_DDSDRIVERTYPES_H
#define VRTF_VCC_DRIVER_DDS_DDSDRIVERTYPES_H
#include <functional>
#include <string>
#include <memory>
#include "vrtf/vcc/api/types.h"
#include "vrtf/driver/dds/dds_qos_store.h"
namespace vrtf {
namespace driver {
namespace dds {
using DomainId = std::int16_t;
using NetworkIp = std::pair<std::string, bool>;
class DDSServiceDiscoveryInfo : public vcc::api::types::ServiceDiscoveryInfo {
public:
    DDSServiceDiscoveryInfo() : domainId_(0) {}
    ~DDSServiceDiscoveryInfo(void) = default;
    DDSServiceDiscoveryInfo(const vcc::api::types::ServiceId& serviceId,
                            const vcc::api::types::InstanceId& instanceId,
                            const vcc::api::types::MethodCallProcessingMode& methodMode,
                            const DomainId& domainId,
                            const std::string& qosPath)
        : ServiceDiscoveryInfo(serviceId, instanceId, methodMode), domainId_(domainId), qosPath_(qosPath) {}
    DomainId GetDomainId() const
    {
        return domainId_;
    }
    void SetDomainId(DomainId id)
    {
        domainId_ = id;
    }
    std::string GetQosPath() const
    {
        return qosPath_;
    }
    void SetQosPath(const std::string& path)
    {
        qosPath_ = path;
    }

    void SetParticipantTransportQos(const std::set<vrtf::driver::dds::qos::TransportQos>& transportMode)
    {
        transportQos_ = transportMode;
    }

    std::set<vrtf::driver::dds::qos::TransportQos> GetParticipantTransportQos() const
    {
        return transportQos_;
    }

    void SetParticipantQos(const qos::ParticipantQos& participantQos) noexcept
    {
        participantQos_ = participantQos;
    }

    qos::ParticipantQos GetParticipantQos() const noexcept
    {
        return participantQos_;
    }
private:
    DomainId domainId_;
    std::string qosPath_;
    std::set<vrtf::driver::dds::qos::TransportQos> transportQos_ = {vrtf::driver::dds::qos::TransportQos::UDP};
    qos::ParticipantQos participantQos_ = qos::ParticipantQos(qos::DiscoveryFilter(0, "UNDEFINED_DISCOVERY_FILTER"));
};

class DDSEventInfo : public vcc::api::types::EventInfo {
public:
    DDSEventInfo() : domainId_(0) {}
    DDSEventInfo(const std::string& topName, const std::string& qosProfile)
        : domainId_(0),
          topicName_(topName),
          qosProfile_(qosProfile),
          scheduleMode_(qos::ScheduleMode::DETERMINATE) {}
    ~DDSEventInfo(void) = default;
    qos::DDSEventQosStore ddsQos_;
    inline std::string GetTopicName() const
    {
        return topicName_;
    }
    inline void SetTopicName(const std::string& name)
    {
        topicName_ = name;
    }
    inline DomainId GetDomainId() const
    {
        return domainId_;
    }
    inline void SetDomainId(DomainId id)
    {
        domainId_ = id;
    }
    inline std::string GetQosProfile() const
    {
        return qosProfile_;
    }
    inline void SetQosProfile(const std::string& profile)
    {
        qosProfile_ = profile;
    }

    void SetAttribute(const std::map<std::string, std::string>& attributeValueList)
    {
        ddsAttributeList_ = attributeValueList;
    }
    std::map<std::string, std::string> GetAtrribute() const
    {
        return ddsAttributeList_;
    }

    inline void SetScheduleMode(const qos::ScheduleMode& scheduleMode)
    {
        scheduleMode_ = scheduleMode;
    }

    inline qos::ScheduleMode GetScheduleMode()
    {
        return scheduleMode_;
    }
    void SetParticipantQos(const qos::ParticipantQos& participantQos) noexcept
    {
        participantQos_ = participantQos;
    }

    qos::ParticipantQos GetParticipantQos() const noexcept
    {
        return participantQos_;
    }

private:
    DomainId domainId_;
    std::string topicName_;
    std::string qosProfile_;
    qos::ScheduleMode scheduleMode_ = qos::ScheduleMode::DETERMINATE;
    std::map<std::string, std::string> ddsAttributeList_;
    qos::ParticipantQos participantQos_ = qos::ParticipantQos(qos::DiscoveryFilter(0, "UNDEFINED_DISCOVERY_FILTER"));
};

class DDSMethodInfo : public vcc::api::types::MethodInfo {
public:
    DDSMethodInfo() : domainId_(0), methodNum_(1) {};
    ~DDSMethodInfo(void) = default;
    qos::DDSMethodQosStore ddsMethodQos_;
    inline std::string GetRequestTopicName() const
    {
        return requestTopicName_;
    }
    inline void SetRequestTopicName(const std::string& name)
    {
        requestTopicName_ = name;
    }
    inline std::string GetReplyTopicName() const
    {
        return replyTopicName_;
    }
    inline void SetReplyTopicName(const std::string& name)
    {
        replyTopicName_ = name;
    }
    inline std::string GetQosProfile() const
    {
        return qosProfile_;
    }
    inline void SetQosProfile(const std::string& profile)
    {
        qosProfile_ = profile;
    }
    DomainId GetDomainId() const
    {
        return domainId_;
    }
    void SetDomainId(DomainId id)
    {
        domainId_ = id;
    }
    void SetMethodNum(uint32_t methodNum)
    {
        methodNum_ = methodNum;
    }
    uint32_t GetMethodNum() const
    {
        return methodNum_;
    }
    void SetParticipantQos(const qos::ParticipantQos& participantQos) noexcept
    {
        participantQos_ = participantQos;
    }

    qos::ParticipantQos GetParticipantQos() const noexcept
    {
        return participantQos_;
    }
private:
    std::string requestTopicName_;
    std::string replyTopicName_;
    std::string qosProfile_ = vrtf::vcc::api::types::UNDEFINED_QOS_PROFILE;
    DomainId domainId_;
    uint32_t methodNum_;
    qos::ParticipantQos participantQos_ = qos::ParticipantQos(qos::DiscoveryFilter(0, "UNDEFINED_DISCOVERY_FILTER"));
};
}
}
}

#endif

