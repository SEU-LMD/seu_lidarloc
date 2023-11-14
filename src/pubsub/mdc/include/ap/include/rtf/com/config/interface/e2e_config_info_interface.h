/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-11-26
 */

#ifndef RTF_COM_CONFIG_E2E_CONFIG_INFO_INTERFACE_H_
#define RTF_COM_CONFIG_E2E_CONFIG_INFO_INTERFACE_H_

#include <cstdint>
#include <string>
#include "rtf/com/config/e2e/e2e_config.h"

namespace rtf {
namespace com {
namespace config {
class E2EConfigInfoInterface {
public:
    /**
     * @brief Interface class destructor
     */
    virtual ~E2EConfigInfoInterface() = default;

    /**
     * @brief Set a E2EConfig
     *
     * @param[in] e2eConfig   The E2EConfig will be stored
     */
    virtual void SetE2EConfig(const std::shared_ptr<E2EConfig>& e2eConfig) noexcept = 0;

    /**
     * @brief  Get the configuration of E2EConfig
     *
     * @return std::shared_ptr<E2EConfig>
     */
    virtual std::shared_ptr<E2EConfig> GetE2EConfig(void) const noexcept = 0;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_E2E_CONFIG_INFO_INTERFACE_H_
