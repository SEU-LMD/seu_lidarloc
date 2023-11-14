/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-21
 */

#ifndef RTF_COM_CONFIG_EVENT_CONFIG_H_
#define RTF_COM_CONFIG_EVENT_CONFIG_H_

#include <set>
#include <string>

#include "rtf/com/config/someip/someip_entity_config.h"
#include "rtf/com/config/interface/event_maintain_interface.h"

namespace rtf {
namespace com {
namespace config {
class SOMEIPEventConfig : public SOMEIPEntityConfig,
                          public EventMaintainInterface {
public:
    /**
     * @brief SOMEIPEventConfig constructor
     *
     * @param[in] eventName  Name of the event
     * @param[in] serviceId  Service id of the event associated service
     * @param[in] instanceId Instance id of the event associated service
     * @param[in] eventId    Event id of the event
     */
    SOMEIPEventConfig(const std::string&     eventName,
                      const ServiceId&       serviceId,
                      const InstanceId&      instanceId,
                      const someip::EventId& eventId);

    /**
     * @brief SOMEIPEventConfig destructor
     */
    virtual ~SOMEIPEventConfig(void) = default;

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

    // SOMEIPEventConfig
    /**
     * @brief Return the name of the event
     * @return Name of the event
     */
    std::string GetEventName(void) const noexcept;

    /**
     * @brief Return the id of the event
     * @return Name of the event
     */
    someip::EventId GetEventId(void) const noexcept;

    /**
     * @brief Set the event groups
     * @param[in] eventGroups Event groups of the event
     */
    void SetEventGroups(const std::set<someip::EventGroupId>& eventGroups) noexcept;

    /**
     * @brief Return the event groups of the event
     * @return Event groups of the event
     */
    std::set<someip::EventGroupId> GetEventGroups(void) const noexcept;

    // EventMaintainInterface
    /**
     * @brief Return the data type of the event
     * @note This is a maintain configuration
     * @return Data type string of the event
     */
    virtual void SetEventDataType(const std::string& eventDataType) noexcept override;

    /**
     * @brief Return the data type of the event
     * @note This is a maintain configuration
     * @return Data type string of the event
     */
    virtual std::string GetEventDataType(void) const noexcept override;

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
    std::string                    eventName_;
    someip::EventId                eventId_;
    std::set<someip::EventGroupId> eventGroups_;
    // EventMaintainInterface
    std::string eventDataType_;
    someip::ComposePacketInfo composePacketInfo_;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_EVENT_CONFIG_H_
