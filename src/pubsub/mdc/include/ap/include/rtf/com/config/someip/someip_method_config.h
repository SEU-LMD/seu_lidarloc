/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-17
 */

#ifndef RTF_COM_SOMEIP_METHOD_CONFIG_H_
#define RTF_COM_SOMEIP_METHOD_CONFIG_H_

#include "rtf/com/config/someip/someip_entity_config.h"

namespace rtf {
namespace com {
namespace config {
class SOMEIPMethodConfig : public SOMEIPEntityConfig {
public:
    /**
     * @brief SOMEIPMethodConfig constructor
     *
     * @param[in] methodName Name of the method
     * @param[in] serviceId Service id of the method associated service
     * @param[in] instanceId Instance id of the method associated service
     * @param[in] methodId Method id of the method
     */
    SOMEIPMethodConfig(const std::string&      methodName,
                       const ServiceId&        serviceId,
                       const InstanceId&       instanceId,
                       const someip::MethodId& methodId);

    /**
     * @brief SOMEIPMethodConfig desstructor
     */
    virtual ~SOMEIPMethodConfig(void) = default;

    // ConfigInfoInterface
    /**
     * @brief Return the entity name of the config
     * @note This is an interface implementation
     * @return Entity name of the config
     */
    virtual std::string GetEntityName(void) const noexcept override;

    /**
     * @brief Return the type of the config
     * @note This is an interface implementation
     * @return Type of the config
     */
    virtual ConfigType GetConfigType(void) const noexcept override;

    // SOMEIPMethodConfigInterface
    /**
     * @brief Return the name of the method
     * @return Name of the method
     */
    std::string GetMethodName(void) const noexcept;

    /**
     * @brief Return the id of the method
     * @return Id of the method
     */
    someip::MethodId GetMethodId(void) const noexcept;
private:
    // SOMEIPMethodConfig
    std::string      methodName_;
    someip::MethodId methodId_;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_SOMEIP_METHOD_CONFIG_H_
