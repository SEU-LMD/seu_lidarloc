/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-21
 */

#ifndef RTF_COM_CONFIG_CONFIG_INFO_INTERFACE_H_
#define RTF_COM_CONFIG_CONFIG_INFO_INTERFACE_H_

#include <cstdint>
#include <string>

namespace rtf {
namespace com {
namespace config {
enum class ConfigType : uint8_t {
    DDS_EVENT_CONFIG,
    DDS_METHOD_CONFIG,
    SOMEIP_SERVICE_CONFIG,
    SOMEIP_EVENT_CONFIG,
    SOMEIP_METHOD_CONFIG
};

class ConfigInfoInterface {
public:
    /**
     * @brief Interface class destructor
     */
    virtual ~ConfigInfoInterface(void)
    {
    }

    /**
     * @brief Return the entity name of the config
     * @note This is an interface
     * @return Entity name of the config
     */
    virtual std::string GetEntityName(void) const noexcept = 0;

    /**
     * @brief Return the type of the config
     * @note This is an interface
     * @return Type of the config
     */
    virtual ConfigType GetConfigType(void) const noexcept = 0;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_CONFIG_INFO_INTERFACE_H_
