/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-21
 */

#ifndef RTF_COM_CONFIG_EVENT_MAINTAIN_INTERFACE_H_
#define RTF_COM_CONFIG_EVENT_MAINTAIN_INTERFACE_H_

#include <string>

namespace rtf {
namespace com {
namespace config {
class EventMaintainInterface {
public:
    /**
     * @brief Interface class destructor
     */
    virtual ~EventMaintainInterface(void)
    {
    }

    /**
     * @brief Setter of the event data type
     * @note This is an interface
     * @note This is a maintain configuration
     * @param[in] eventDataType Data type string of the event
     */
    virtual void SetEventDataType(const std::string& eventDataType) noexcept = 0;

    /**
     * @brief Return the data type of the event
     * @note This is an interface
     * @note This is a maintain configuration
     * @return Data type string of the event
     */
    virtual std::string GetEventDataType(void) const noexcept = 0;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_EVENT_CONFIG_INTERFACE_H_
