/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-06-03
 */

#ifndef RTF_COM_CONFIG_SOMEIP_ENTITY_CONFIG_H_
#define RTF_COM_CONFIG_SOMEIP_ENTITY_CONFIG_H_

#include <string>
#include "rtf/com/types/ros_types.h"
#include "rtf/com/config/e2e/e2e_config.h"
#include "rtf/com/config/someip/someip_serialize_tlv_config.h"
#include "rtf/com/config/interface/config_info_interface.h"
#include "rtf/com/config/interface/service_maintain_interface.h"
#include "rtf/com/config/interface/e2e_config_info_interface.h"

namespace rtf {
namespace com {
namespace config {
class SOMEIPEntityConfig : public ConfigInfoInterface,
                           public ServiceMaintainInterface,
                           public E2EConfigInfoInterface {
public:
    /**
     * @brief SOMEIPEntityConfig constructor
     * @param[in] serviceId  ServiceId
     * @param[in] instanceId instanceId
     */
    SOMEIPEntityConfig(const ServiceId& serviceId, const InstanceId& instanceId);

    /**
     * @brief SOMEIPEntityConfig destructor
     */
    virtual ~SOMEIPEntityConfig(void) = default;

    /**
     * @brief Return the service id of the service
     * @return Service id of the service
     */
    ServiceId GetServiceId(void) const noexcept;

    /**
     * @brief Return the instance id of the service
     * @return Instance id of the service
     */
    InstanceId GetInstanceId(void) const noexcept;

    /**
     * @brief Set the transport modes
     * @param[in] transportMode Transport modes that service uees
     */
    void SetTransportMode(const TransportMode& transportMode) noexcept;

    /**
     * @brief Return the transport modes of the service
     * @return Transport modes of the service
     */
    TransportMode GetTransportMode(void) const noexcept;

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
     * @brief Set the instance short name
     * @note This is a maintain configuration
     * @param[in] instanceShortName The shortName of the current process
     */
    virtual void SetInstanceShortName(const std::string& instanceShortName) noexcept override;

    /**
     * @brief Return the instance short name of the current process
     * @note This is a maintain configuration
     * @return The shortName of the current process
     */
    virtual std::string GetInstanceShortName(void) const noexcept override;

    /**
     * @brief Set serialization type
     * @param[in] serializationType New serialization type
     */
    void SetSerializationType (const rtf::com::SerializationType& serializationType) noexcept;

    /**
     * @brief Return serialization type
     * @return serialization type
     */
    virtual rtf::com::SerializationType GetSerializationType(void) const noexcept;

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

    void SetSomeipSerializeConfig(const std::shared_ptr<SOMEIPSerializeBaseConfig>& config);
    std::shared_ptr<SOMEIPSerializeBaseConfig> GetSomeipSerializeConfig() const noexcept;
protected:
    static const TransportMode DEFAULT_TRANSPORT_MODE;
    static const rtf::com::SerializationType DEFAULT_SERIALIZATION_TYPE;
private:
    ServiceId     serviceId_;
    InstanceId    instanceId_;
    TransportMode transportMode_;
    // ServiceMaintainInterface
    std::string   instanceShortName_;
    std::shared_ptr<rtf::TrafficCtrlPolicy> trafficCrtlPolicy_;
    rtf::com::SerializationType serializationType_;
    std::shared_ptr<E2EConfig> e2eConfig_;
    std::shared_ptr<SOMEIPSerializeBaseConfig> serializeConfig_;
    std::shared_ptr<rtf::com::utils::Logger> logger_;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_SOMEIP_ENTITY_CONFIG_H_
