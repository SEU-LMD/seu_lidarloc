/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-21
 */

#ifndef RTF_COM_CONFIG_SERVICE_MAINTAIN_INTERFACE_H_
#define RTF_COM_CONFIG_SERVICE_MAINTAIN_INTERFACE_H_

#include <string>

namespace rtf {
namespace com {
namespace config {
class ServiceMaintainInterface {
public:
    /**
     * @brief Interface class destructor
     */
    virtual ~ServiceMaintainInterface(void)
    {
    }

    /**
     * @brief Setter of the instance short name
     * @note This is an interface
     * @note This is a maintain configuration
     * @param[in] instanceShortName The shortName of the current process
     */
    virtual void SetInstanceShortName(const std::string& instanceShortName) noexcept = 0;

    /**
     * @brief Return the instance short name of the current process
     * @note This is an interface
     * @note This is a maintain configuration
     * @return The shortName of the current process
     */
    virtual std::string GetInstanceShortName(void) const noexcept = 0;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_SERVICE_MAINTAIN_CONFIG_INTERFACE_H_
