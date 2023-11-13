/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-06-03
 */

#ifndef RTF_COM_CONFIG_DDS_ENTITY_CONFIG_H_
#define RTF_COM_CONFIG_DDS_ENTITY_CONFIG_H_

#include <set>
#include <string>
#include "rtf/com/types/ros_types.h"
#include "rtf/com/config/interface/config_info_interface.h"
#include "rtf/com/config/interface/service_maintain_interface.h"
#include "rtf/com/config/dds/dds_role_config.h"
#include "rtf/com/config/interface/e2e_config_info_interface.h"

namespace rtf {
namespace com {
namespace utils {
class Logger;
}
namespace config {
class DDSEntityConfig : public ConfigInfoInterface,
                        public ServiceMaintainInterface,
                        public E2EConfigInfoInterface {
public:
    /**
     * @brief DDSEntityConfig constructor
     *
     * @param[in] domainId       Domain id of the entity
     * @param[in] transportModes Transport modes of the entity
     */
    DDSEntityConfig(const dds::DomainId& domainId, const std::set<TransportMode>& transportModes);

    /**
     * @brief DDSEntityConfig constructor
     */
    DDSEntityConfig(void);

    /**
     * @brief DDSEntityConfig destructor
     */
    virtual ~DDSEntityConfig(void) = default;

    /**
     * @brief Set entity service id
     * @param[in] serviceId New service id
     */
    void SetServiceId(const ServiceId& serviceId) noexcept;

    /**
     * @brief Return entity service id
     * @return Service id of the entity
     */
    ServiceId GetServiceId(void) const noexcept;

    /**
     * @brief Set entity instance id
     * @param[in] instanceId New instance id
     */
    void SetInstanceId(const InstanceId& instanceId) noexcept;

    /**
     * @brief Return entity instance id
     * @return Instance id of the entity
     */
    InstanceId GetInstanceId(void) const noexcept;

    /**
     * @brief Set entity domain id
     * @param[in] domainId New domain id
     */
    void SetDomainId(const dds::DomainId& domainId) noexcept;

    /**
     * @brief Return entity domain id
     * @return Domain id of the entity
     */
    dds::DomainId GetDomainId(void) const noexcept;

    /**
     * @brief Set entity network ip address
     * @param[in] network New network ip address
     */
    void SetNetwork(const std::string& network) noexcept;

    /**
     * @brief Return entity network ip address
     * @return Network ip address of the entity
     */
    std::string GetNetwork(void) const noexcept;

    /**
     * @brief Set entity traffic control policy
     * @param[in] policy traffic control policy
     */
    void SetTrafficCtrl(const std::shared_ptr<rtf::TrafficCtrlPolicy>& policy) noexcept;

    /**
     * @brief Get entity traffic control policy
     * @return traffic control policy
     */
    std::shared_ptr<rtf::TrafficCtrlPolicy> GetTrafficCtrl(void) const noexcept;
    // ServiceMaintainInterface
    /**
     * @brief Set entity instance short name
     * @note This is a maintain configuration
     * @param[in] instanceShortName New instance short name
     */
    virtual void SetInstanceShortName(const std::string& instanceShortName) noexcept override;

    /**
     * @brief Return entity instance short name
     * @note This is a maintain configuration
     * @return The instance short name of the entity
     */
    virtual std::string GetInstanceShortName(void) const noexcept override;

    /**
     * @brief Set serialization type
     * @param[in] serializationType New serialization type
     */
    virtual void SetSerializationType (const rtf::com::SerializationType& serializationType) noexcept;

    /**
     * @brief Return serialization type
     * @return serialization type
     */
    virtual rtf::com::SerializationType GetSerializationType(void) const noexcept;

    /**
     * @brief Set entity transport mode
     *
     * @param[in] transportModes transport modes of the entity
     * @param[in] role           choose transport mode role config
     */
    void SetTransportMode(const std::set<TransportMode>& transportModes, const Role& role = Role::BOTH) noexcept;

    /**
     * @brief Get entity transport mode
     *
     * @param[in] role         choose transportMode role config
     * @return std::set<TransportMode> transport mode of this role, default both will return total transport mode config
     */
    std::set<TransportMode> GetTransportMode(const Role& role = Role::BOTH) const noexcept;

    // E2EConfigInfoInterface
    /**
     * @brief Set an E2EConfig
     *
     * @param[in] e2eConfig  The E2EConfig will be used
     */
    virtual void SetE2EConfig(const std::shared_ptr<E2EConfig>& e2eConfig) noexcept override;

    /**
     * @brief Get an E2EConfig
     *
     * @return std::shared_ptr<E2EConfig>   The instance of E2EConfig
     */
    virtual std::shared_ptr<E2EConfig> GetE2EConfig() const noexcept override;

    /**
     * @brief Set a ResourceAttr
     *
     * @param[in] resourceAttr File's user and group
     */
    virtual void SetResourceAttr(const rtf::com::ResourceAttr& resourceAttr) noexcept;

    /**
     * @brief Get a ResourceAttr
     *
     * @return ResourceAttr File's user and group
     */
    virtual rtf::com::ResourceAttr GetResourceAttr() const noexcept;

    /**
     * @brief Set the participantQos to distinguish discovery filter
     * @param[in] participantQos participantQos
     */
    void SetParticipantQos(const dds::ParticipantQos& participantQos) noexcept;

    /**
     * @brief Return the participantQos to distinguish discovery filter
     * @return ParticipantQos
     */
    rtf::com::dds::ParticipantQos GetParticipantQos() const noexcept;

    /**
     * @brief Get a role type
     *
     * @return Role type set
     */
    std::set<rtf::com::Role> GetRoleType() const noexcept;

    /**
     * @brief Set resource create handler
     * @param[in] handler resource create handler
     */
    void RegResourceCreateHandler(const ResourceCreateHandler &handler);

    /**
     * @brief Return resource create handler
     * @return ResourceCreateHandler
     */
    ResourceCreateHandler GetResourceCreateHandler() const noexcept;
protected:
    static ServiceId const DEFAULT_SERVICE_ID;
    static InstanceId const DEFAULT_INSTANCE_ID;
    static dds::DomainId const DEFAULT_DOMAIN_ID;
    static const std::set<TransportMode>     DEFAULT_TRANSPORT_MODES;
    static const rtf::com::SerializationType DEFAULT_SERIALIZATION_TYPE;
private:
    ServiceId               serviceId_;
    InstanceId              instanceId_;
    dds::DomainId           domainId_;
    Network                 network_;
    std::shared_ptr<rtf::TrafficCtrlPolicy> trafficCrtlPolicy_;
    // ServiceMaintainInterface
    std::string instanceShortName_;
    rtf::com::SerializationType serializationType_;
    DDSRoleConfig roleConfig_;
    std::shared_ptr<E2EConfig> e2eConfig_;
    rtf::com::ResourceAttr resourceAttr_;
    rtf::com::dds::ParticipantQos participantQos_
        = rtf::com::dds::ParticipantQos(rtf::com::dds::DiscoveryFilter(0, "UNDEFINED_DISCOVERY_FILTER"));
    std::shared_ptr<rtf::com::utils::Logger> logger_;
    ResourceCreateHandler resHandler_;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_DDS_ENTITY_CONFIG_H_
