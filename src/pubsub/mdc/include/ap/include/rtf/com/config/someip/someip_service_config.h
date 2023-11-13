/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-21
 */

#ifndef RTF_COM_CONFIG_SOMEIP_SERVICE_CONFIG_H_
#define RTF_COM_CONFIG_SOMEIP_SERVICE_CONFIG_H_

#include <unordered_map>

#include "rtf/com/types/ros_types.h"
#include "rtf/com/utils/logger.h"
#include "rtf/com/config/interface/config_info_interface.h"

namespace rtf {
namespace com {
namespace config {
class SOMEIPServiceConfig : public ConfigInfoInterface {
public:
    /**
     * @brief SOMEIPServiceConfig constructor
     *
     * @param[in] serviceId Service id
     * @param[in] instanceId Instance id
     */
    SOMEIPServiceConfig(const ServiceId& serviceId, const InstanceId& instanceId);

    /**
     * @brief SOMEIPServiceConfig desstructor
     */
    virtual ~SOMEIPServiceConfig(void) = default;

    // ConfigInfoInterface
    /**
     * @brief Return the entity name of the config
     * @note This is an interface implementation
     * @return Entity name of the config
     */
    virtual std::string GetEntityName(void) const noexcept override;

    /**
     * @brief Return config type of the config
     * @note This is an interface implementation
     * @return Type of the config
     */
    virtual ConfigType GetConfigType(void) const noexcept override;

    // SOMEIPServiceConfig
    /**
     * @brief Return service service id
     * @return Service id of the service
     */
    ServiceId GetServiceId(void) const noexcept;

    /**
     * @brief Return service instance id
     * @return Instance id of the service
     */
    InstanceId GetInstanceId(void) const noexcept;

    /**
     * @brief Set service major version
     * @param[in] majorVersion Major version of the service
     */
    void SetMajorVersion(const someip::MajorVersion& majorVersion) noexcept;

    /**
     * @brief Return service major version
     * @return Major version of the service
     */
    someip::MajorVersion GetMajorVersion(void) const noexcept;

    /**
     * @brief Set the minor version
     * @param[in] minorVersion Minor version of the service
     */
    void SetMinorVersion(const someip::MinorVersion& minorVersion) noexcept;

    /**
     * @brief Return serivce minor version
     * @return Minor version of the service
     */
    someip::MinorVersion GetMinorVersion(void) const noexcept;

    /**
     * @brief Set service network connector
     * @param[in] network Network connector
     */
    void SetNetwork(const std::string& network) noexcept;

    /**
     * @brief Return servicce network connector
     * @return Network connector of the service
     */
    std::string GetNetwork(void) const noexcept;

    /**
     * @brief Set service port
     * @param[in] transportMode Transport mode of the service
     * @param[in] portNum       Port number
     */
    void SetPort(const TransportMode& transportMode, const someip::Port& portNum) noexcept;

    /**
     * @brief Return service instance id
     * @param[in] transportMode Transport mode of the service
     * @return Port number
     */
    someip::Port GetPort(const TransportMode& transportMode) const noexcept;

    /**
     * @brief Set the filtered address
     * @param[in] address Filtering address for playing event on pub online
     */
    void SetFilterAddress(const std::string& address) noexcept;

    /**
     * @brief Return the filtered address
     * @return The filtered address
     */
    std::string GetFilterAddress() const noexcept;

    /**
     * @brief Set config info about composing packet
     * @param[in] info Config info about composing packet
     */
    void SetComposePacketInfo(const someip::ComposePacketInfo& info) noexcept;

    /**
     * @brief Return the data type of the event
     * @return Config info about composing packet
     */
    someip::ComposePacketInfo GetComposePacketInfo(void) const noexcept;

private:
    ServiceId            serviceId_;
    InstanceId           instanceId_;
    someip::MajorVersion majorVersion_;
    someip::MinorVersion minorVersion_;
    Network              network_;
    std::unordered_map<TransportMode, someip::Port> ports_;
    std::string filteredAddress_;
    someip::ComposePacketInfo composePacketInfo_;
    std::shared_ptr<rtf::com::utils::Logger> logger_;

    static someip::MajorVersion const DEFAULT_MAJOR_VERSION;
    static someip::MinorVersion const DEFAULT_MINOR_VERSION;
    static someip::Port         const DEFAULT_PORT;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_SOMEIP_SERVICE_CONFIG_H_
