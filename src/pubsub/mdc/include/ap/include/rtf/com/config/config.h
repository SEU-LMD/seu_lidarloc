/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-18
 */

#ifndef RTF_COM_CONFIG_CONFIG_H_
#define RTF_COM_CONFIG_CONFIG_H_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rtf/com/config/dds/dds_event_config.h"
#include "rtf/com/config/dds/dds_method_config.h"
#include "rtf/com/config/someip/someip_event_config.h"
#include "rtf/com/config/someip/someip_method_config.h"
#include "rtf/com/config/someip/someip_service_config.h"
#include "rtf/com/config/e2e/e2e_config.h"

namespace rtf {
namespace com {
namespace config {
class Config {
public:
    /**
     * @brief Add a DDSEventConfig
     * @param[in] eventConfig DDSEventConfig instance
     * @return Operation result
     */
    static bool AddDDSEventConfig(std::shared_ptr<DDSEventConfig> const &eventConfig) noexcept;

    /**
     * @brief Delete the DDSEventConfig by event name
     * @param[in] eventName Event name
     * @return Operation result
     */
    static bool DeleteDDSEventConfig(std::string const &eventName) noexcept;

    /**
     * @brief Get the DDSEventConfig by event name
     * @param[in] eventName Event name
     * @return DDSEventConfig instance
     */
    static std::shared_ptr<DDSEventConfig> GetDDSEventConfig(std::string const &eventName) noexcept;

    /**
     * @brief Add a DDSMethodConfig
     * @param[in] methodConfig DDSMethodConfig instance
     * @return Operation result
     */
    static bool AddDDSMethodConfig(std::shared_ptr<DDSMethodConfig> const &methodConfig) noexcept;

    /**
     * @brief Delete the DDSMethodConfig by methodName
     * @param[in] methodName Method name
     * @return Operation result
     */
    static bool DeleteDDSMethodConfig(std::string const &methodName) noexcept;

    /**
     * @brief Get the DDSMethodConfig by method name
     * @param[in] methodName Method name
     * @return DDSMethodConfig instance
     */
    static std::shared_ptr<DDSMethodConfig> GetDDSMethodConfig(std::string const &methodName) noexcept;

    /**
     * @brief Add a SOMEIPServiceConfig
     * @param[in] serviceConfig SOMEIPServiceConfig instance
     * @return Operation result
     */
    static bool AddSOMEIPServiceConfig(std::shared_ptr<SOMEIPServiceConfig> const &serviceConfig) noexcept;

    /**
     * @brief Delete the SOMEIPServiceConfig by serviceId & instanceId
     * @param[in] serviceId Service id
     * @param[in] instanceId Instance id
     * @return Operation result
     */
    static bool DeleteSOMEIPServiceConfig(ServiceId const &serviceId, InstanceId const &instanceId) noexcept;

    /**
     * @brief Get the SOMEIPServiceConfig by serviceId & instanceId
     * @param[in] serviceId Service id
     * @param[in] serviceId Instance id
     * @return SOMEIPServiceConfig instance
     */
    static std::shared_ptr<SOMEIPServiceConfig> GetSOMEIPServiceConfig(ServiceId const &serviceId,
                                                                       InstanceId const &instanceId) noexcept;

    /**
     * @brief Add a SOMEIPEventConfig
     * @param[in] config SOMEIPEventConfig instance
     * @return Operation result
     */
    static bool AddSOMEIPEventConfig(std::shared_ptr<SOMEIPEventConfig> const &eventConfig) noexcept;

    /**
     * @brief Delete the SOMEIPEventConfig by event name
     * @param[in] eventName Event name
     * @return Operation result
     */
    static bool DeleteSOMEIPEventConfig(std::string const &eventName) noexcept;

    /**
     * @brief Get the SOMEIPEventConfig by event name
     * @param[in] eventName Event name
     * @return SOMEIPEventConfig instance
     */
    static std::shared_ptr<SOMEIPEventConfig> GetSOMEIPEventConfig(std::string const &eventName) noexcept;

    /**
     * @brief Add a SOMEIPMethodConfig
     * @param[in] methodConfig SOMEIPMethodConfig instance
     * @return Operation result
     */
    static bool AddSOMEIPMethodConfig(std::shared_ptr<SOMEIPMethodConfig> const &methodConfig) noexcept;

    /**
     * @brief Delete the SOMEIPMethodConfig by method name
     * @param[in] methodName Method name
     * @return Operation result
     */
    static bool DeleteSOMEIPMethodConfig(std::string const &methodName) noexcept;

    /**
     * @brief Get the SOMEIPMethodConfig by method name
     * @param[in] methodName Method name
     * @return SOMEIPMethodConfig instance
     */
    static std::shared_ptr<SOMEIPMethodConfig> GetSOMEIPMethodConfig(std::string const &methodName) noexcept;
private:
    static std::mutex accessMutex_;
    static std::unordered_map<std::string, std::shared_ptr<ConfigInfoInterface>> configMap_;
    /**
     * @brief Config default constructor
     */
    Config(void) = delete;

    /**
     * @brief Config destructor
     */
    ~Config(void) = delete;

    /**
     * @brief Config copy constructor
     * @note Deleted
     */
    Config(Config const &other) = delete;

    /**
     * @brief Config copy assignment operator
     * @note Deleted
     */
    Config& operator=(Config const &other) = delete;

    /**
     * @brief Add a config
     * @param[in] config Config instance
     * @return Operation result
     */
    static bool AddConfig(std::shared_ptr<ConfigInfoInterface> const &config) noexcept;

    /**
     * @brief Delete the config by config entry
     * @param[in] type Config type
     * @param[in] entityName Entity name
     * @return Operation result
     */
    static bool DeleteConfig(ConfigType const &type, std::string const &entityName) noexcept;

    /**
     * @brief Get the config by config entry
     * @param[in] type Config type
     * @param[in] entityName Entity name
     * @return Config instance
     */
    static std::shared_ptr<ConfigInfoInterface> GetConfig(ConfigType const &type,
                                                          std::string const &entityName) noexcept;

    /**
     * @brief Get a config entry name
     * @param[in] type Config type
     * @param[in] entityName Entity name
     * @return Config instance
     */
    static std::string GetConfigEntry(ConfigType const &type, std::string const &entityName) noexcept;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_CONFIG_H_
