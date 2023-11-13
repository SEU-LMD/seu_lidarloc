 /*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-18
 */

#ifndef RTF_COM_UTILS_CONFIG_HELPER_H_
#define RTF_COM_UTILS_CONFIG_HELPER_H_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rtf/com/config/dds/dds_event_config.h"
#include "rtf/com/config/dds/dds_method_config.h"
#include "rtf/com/config/someip/someip_event_config.h"
#include "rtf/com/config/someip/someip_serialize_tlv_config.h"
#include "rtf/com/config/someip/someip_method_config.h"
#include "rtf/com/config/someip/someip_service_config.h"
#include "rtf/com/config/e2e/e2e_config.h"

#include "rtf/com/types/ros_types.h"
#include "vrtf/vcc/api/types.h"
#include "json_parser/json_writer.h"

namespace rtf {
namespace com {
namespace utils {
class ConfigHelper {
public:
    /**
     * @brief Get the corresponding DDS event info by entity URI
     *
     * @param[in] entityURI    The uri of the dds event entity
     * @param[in] role    The role of the dds event entity
     * @return Shared pointer of dds::EventInfo
     */
    static std::shared_ptr<dds::EventInfo> ParseDDSEventInfo(const std::string& entityURI, const Role& role) noexcept;

    /**
     * @brief Get the corresponding DDS method info by entity URI
     *
     * @param[in] entityURI    The uri of the dds method entity
     * @param[in] role    The role of the dds event entity
     * @return Shared pointer of dds::MethodInfo
     */
    static std::shared_ptr<dds::MethodInfo> ParseDDSMethodInfo(const std::string& entityURI, const Role& role) noexcept;

    /**
     * @brief Get the corresponding SOME/IP method info by entity URI
     *
     * @param[in] entityURI    The uri of the someip event entity
     * @return Shared pointer of someip::EventInfo
     */
    static std::shared_ptr<someip::EventInfo> ParseSOMEIPEventInfo(const std::string& entityURI) noexcept;

    /**
     * @brief Get the corresponding SOME/IP method info by entity URI
     *
     * @param[in] entityURI    The uri of the someip method entity
     * @return Shared pointer of someip::MethodInfo
     */
    static std::shared_ptr<someip::MethodInfo> ParseSOMEIPMethodInfo(const std::string& entityURI) noexcept;

    /**
     * @brief Get the corresponding service discovery info by dds::EventInfo
     *
     * @param[in] eventInfo    Shared pointer of DDS event info
     * @return Shared pointer of dds::SdInfo
     */
    static std::shared_ptr<dds::SdInfo> ParseSdInfo(const std::shared_ptr<dds::EventInfo>& eventInfo) noexcept;

    /**
     * @brief Get the corresponding service discovery info by dds::MethodInfo
     *
     * @param[in] methodInfo    Shared pointer of DDS method info
     * @return Shared pointer of dds::SdInfo
     */
    static std::shared_ptr<dds::SdInfo> ParseSdInfo(const std::shared_ptr<dds::MethodInfo>& methodInfo) noexcept;

    /**
     * @brief Get the corresponding service discovery info by someip::EventInfo
     *
     * @param[in] eventInfo    Shared pointer of SOMEIP event info
     * @return Shared pointer of someip::SdInfo
     */
    static std::shared_ptr<someip::SdInfo> ParseSdInfo(const std::shared_ptr<someip::EventInfo>& eventInfo) noexcept;

    /**
     * @brief Get the corresponding service discovery info by someip::MethodInfo
     *
     * @param[in] methodInfo    Shared pointer of SOME/IP method info
     * @return Shared pointer of someip::SdInfo
     */
    static std::shared_ptr<someip::SdInfo> ParseSdInfo(
        const std::shared_ptr<someip::MethodInfo>& methodInfo) noexcept;

    /**
     * @brief Get the service name by dds::SdInfo
     *
     * @param[in] sdInfo    Shared pointer of dds::SdInfo
     * @return Service name
     */
    static std::string ParseServiceName(const std::shared_ptr<dds::SdInfo>& sdInfo) noexcept;

    /**
     * @brief Get the service name by someip::SdInfo
     *
     * @param[in] sdInfo    Shared pointer of SomeipServiceDiscoveryInfo
     * @return Service name
     */
    static std::string ParseServiceName(const std::shared_ptr<someip::SdInfo>& sdInfo) noexcept;

    /**
     * @brief Get traffic crtl policy
     *
     * @param[in] sdInfo    Shared pointer of dds::SdInfo
     * @return Service name
     */
    static std::shared_ptr<rtf::TrafficCtrlPolicy> ParseTrafficCrtlPolicy(const std::string& entityURI) noexcept;

    /**
     * @brief Get the entity config by parameters
     *
     * @param[in] type         The type of transimision, event or method
     * @param[in] protocol     The used protocol type
     * @param[in] entityURI    The uri of the entity
     * @param[in] role         The role of the entity
     * @return Shared pointer of EntityConfig
     */
    static std::shared_ptr<EntityConfig> ParseEntityConfig(const AdapterType&     type,
                                                           const AdapterProtocol& protocol,
                                                           const std::string&     entityURI,
                                                           const Role&            role) noexcept;

    /**
     * @brief Create a default event config
     * @note Default config is in DDS protocol
     * @param[in] entityURI    The uri of the entity
     * @return Unique pointer of EntityConfig
     */
    static std::shared_ptr<EntityConfig> CreateDefaultEventConfig(const std::string& entityURI) noexcept;

    /**
     * @brief Create a default method config
     * @note Default config is in DDS protocol
     * @param[in] entityURI    The uri of the entity
     * @return Unique pointer of EntityConfig
     */
    static std::shared_ptr<EntityConfig> CreateDefaultMethodConfig(const std::string& entityURI) noexcept;
private:
    struct E2EParsingResult {
        std::shared_ptr<vrtf::com::e2e::E2EXf_Object> e2eObject;
        std::shared_ptr<vrtf::com::e2e::E2EXf_CMConfig> e2eXfCmConfig;
    };
    /**
     * @brief Construct a new Config Helper object
     *
     */
    ConfigHelper(void) = default;

    /**
     * @brief Destroy the Config Helper object
     *
     */
    ~ConfigHelper(void) = default;

    /**
     * @brief Construct a new Config Helper object(delete)
     *
     * @param other
     */
    ConfigHelper(const ConfigHelper& other) = delete;

    /**
     * @brief the copy constructor of ConfigHelper(delete)
     * @param[in] other    Other instance
     * @return ConfigHelper&
     */
    ConfigHelper& operator=(const ConfigHelper& other) = delete;

    /**
     * @brief the move constructor of ConfigHelper(delete)
     * @param[in] other    Other instance
     * @return ConfigHelper&
     */
    ConfigHelper& operator=(ConfigHelper && other) = delete;

    /**
     * @brief Parse TransportMode set to DDS TransportQos set
     * @param[in] transportModes    Transport mode set
     * @return DDS TransportQos set
     */
    static std::set<dds::TransportQos> ParseDDSTransportQosSet(const std::set<TransportMode>& transportModes) noexcept;

    /**
     * @brief Parse ScheduleMode to CM ScheduleMode
     * @param[in] scheduleMode   schedule mode set
     * @return CM schedule mode
     */
    static rtf::com::dds::ScheduleMode ParseScheduleMode(const rtf::com::ScheduleMode& scheduleMode) noexcept;

    /**
     * @brief Parse SerializationType to CM SerializationType
     * @param[in] serializationType   serialization type set
     * @return serialization type
     */
    static rtf::com::VccSerializationType ParseSerializationType(
        const rtf::com::SerializationType& serializationType) noexcept;

    /**
     * @brief Print the Event info of DDS
     *
     * @param[in] entityURI             The uri of entity
     * @param[in] instanceShortName     The instance short name of the entity
     * @param[in] eventInfo             The event info will be printed
     */
    static void PrintDDSEventInfo(const std::string& entityURI, const std::string& instanceShortName,
                                  const std::shared_ptr<dds::EventInfo>& eventInfo) noexcept;

    /**
     * @brief Print the Method info of DDS
     *
     * @param[in] entityURI             The uri of entity
     * @param[in] instanceShortName     The instance short name of the entity
     * @param[in] methodInfo            The method info will be printed
     */
    static void PrintDDSMethodInfo(const std::string& entityURI, const std::string& instanceShortName,
                                   const std::shared_ptr<dds::MethodInfo>& methodInfo) noexcept;
    /**
     * @brief Set the Method NetWork info of DDS
     *
     * @param[in] methodInfo          The method info will be set network
     * @param[in] ddsMethodConfig     The dds method configuration set by the user
     */
    static void SetMethodNetWork(const std::shared_ptr<dds::MethodInfo> &methodInfo,
        const std::shared_ptr<config::DDSMethodConfig> &ddsMethodConfig);

    /**
     * @brief Set the Method qos info of DDS
     *
     * @param[in] role                The role of the dds method entity
     * @param[in] methodInfo          The method info will be set network
     * @param[in] ddsMethodConfig     The dds method configuration set by the user
     */
    static void SetMethodQos(const Role& role, const std::shared_ptr<dds::MethodInfo> &methodInfo,
        const std::shared_ptr<config::DDSMethodConfig> &ddsMethodConfig);

    /**
     * @brief Set the Event qos info of DDS
     *
     * @param[in] role                The role of the dds event entity
     * @param[in] eventInfo           The event info will be set network
     * @param[in] ddsEventConfig     The dds event configuration set by the user
     */
    static void SetEventQos(Role const &role, std::shared_ptr<dds::EventInfo> const &eventInfo,
                             std::shared_ptr<config::DDSEventConfig> const &ddsEventConfig);

    /**
     * @brief Print the Event info of SOMEIP
     *
     * @param[in] entityURI             The uri of entity
     * @param[in] instanceShortName     The instance short name of the entity
     * @param[in] eventInfo             The event info will be printed
     */
    static void PrintSOMEIPEventInfo(const std::string& entityURI, const std::string& instanceShortName,
                                     const std::shared_ptr<someip::EventInfo>& eventInfo) noexcept;

    /**
     * @brief Print the Method info of SOMEIP
     *
     * @param[in] entityURI             The uri of entity
     * @param[in] instanceShortName     The instance short name of the entity
     * @param[in] methodInfo            The method info will be printed
     */
    static void PrintSOMEIPMethodInfo(const std::string& entityURI, const std::string& instanceShortName,
                                      const std::shared_ptr<someip::MethodInfo>& methodInfo) noexcept;

    /**
     * @brief Get E2EInfo which contains E2EXf_Object and corresponding E2EXf_Profile
     *
     * @param[in] entityURI  The uri of entity
     * @param[in] offset     The offset of E2E header only used in E2E Protect
     * @param[in] e2eConfig   The E2EConfig will be used
     * @return the unique pointer of E2EParsingResult
     *      @retval nullptr         Parsing failed, shouldn't create instance
     *      @retval non-nullptr     Pasing success
     */
    static std::unique_ptr<E2EParsingResult> ParseE2EInfo(const std::string& entityURI,
        const std::uint32_t defaultOffset, const std::string& networkName,
        const std::shared_ptr<rtf::com::config::E2EConfig> e2eConfig) noexcept;
    /**
     * @brief Print profile configurations of E2EConfig Info
     *
     * @param[in] entityURI   The uri of entity
     * @param[in] e2eConfig   The E2EConfig will be used
     */
    static void PrintE2EProfileConfigInfo(const std::string& entityURI,
                                          const std::shared_ptr<rtf::com::config::E2EConfig>& e2eConfig) noexcept;

    /**
     * @brief Set dds method topic info
     *
     * @param[in] entityURI         The event uri
     * @param[in] ddsMethodConfig   The dds method config
     * @param[in] methodInfo        The dds method info
     */
    static void SetDDSMethodTopicInfo(const std::string& entityURI,
                                      const std::shared_ptr<rtf::com::config::DDSMethodConfig>& ddsMethodConfig,
                                      const std::shared_ptr<rtf::com::dds::MethodInfo>& methodInfo) noexcept;

    /**
     * @brief Set VccDDSEventInfo
     *
     * @param[in]  entityURI       The uri of entity
     * @param[in]  ddsEventConfig  The Config of DDS Event
     * @param[in]  e2eInfo         The info of e2e will set to VccDDSEventInfo
     * @param[in]  role            The role of the config will be used
     */
    static std::shared_ptr<dds::EventInfo> SetDDSEventInfo(const std::string& entityURI,
        const std::shared_ptr<config::DDSEventConfig>& ddsEventConfig,
        const std::unique_ptr<E2EParsingResult>& e2eInfo, const Role& role) noexcept;

    static std::shared_ptr<someip::EventInfo> SetSOMEIPEventInfo(const std::string& entityURI,
        const std::shared_ptr<config::SOMEIPEventConfig>& ddsEventConfig) noexcept;

    static void SetSOMEIPEventBasicInfo(const std::shared_ptr<someip::EventInfo>& eventInfo,
        const std::string& entityURI, const std::shared_ptr<config::SOMEIPEventConfig>& someipEventConfig) noexcept;

    /**
     * @brief Print E2E StateMachine configurations of E2EConfig Info
     *
     * @param[in] entityURI   The uri of entity
     * @param[in] e2eConfig   The E2EConfig will be used
     */
    static void PrintE2ESMConfigInfo(const std::string& entityURI,
                                     const std::shared_ptr<rtf::com::config::E2EConfig>& e2eConfig) noexcept;
    static VccSerializeConfig ParseSOMEIPSerializeConfig(
        const std::shared_ptr<rtf::com::config::SOMEIPSerializeBaseConfig>& config,
        const rtf::com::SerializationType& type, const rtf::com::VccStructSerializationPolicy& structPolicy);
};
} // namespace utils
} // namespace com
} // namespace rtf
#endif // RTF_COM_UTILS_CONFIG_HELPER_H_
