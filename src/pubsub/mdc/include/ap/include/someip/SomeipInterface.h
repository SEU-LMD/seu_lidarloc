/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 */

#ifndef SOMEIP_INTERFACE_H
#define SOMEIP_INTERFACE_H

#include <chrono>
#include <memory>
#include <set>
#include <map>
#include <vector>
#include <functional>
#include <string>
#include <someip/SomeipEnumTypes.h>
#include <someip/SomeipTypes.h>
#include <someip/SomeipConstants.h>
#include <someip/Payload.h>
#include <someip/Message.h>
#include <someip/SSLCTXInterface.h>

namespace Someip {
using StateCallback = std::function<void(AppState state)>;
using MessageCallback = std::function<void(const std::shared_ptr<Message> &message)>;
using AvailabilityCallback = std::function<void(ServiceID service, InstanceID instance, bool isAvailable)>;
using EpsilonChangeCallback = std::function<bool(const std::shared_ptr<Payload> &previous,
    const std::shared_ptr<Payload> &current)>;
using SubscriptionStatusCallback = std::function<void(const ServiceID service, const InstanceID instance,
    const EventgroupID eventgroup, const EventID event, const uint16_t status)>;
using QueryConfigCallback = std::function<void(const std::string &json, bool isOk)>;
using UpdateConfigCallback = std::function<void(bool isOk)>;
using E2ECallback = std::function<bool(const std::shared_ptr<Message> &message, SessionID newSession)>;
using SSLCTXCallback = std::function<SSLCTXInterface &(const std::string &ip, const uint16_t port)>;
using CheckAuthIDCallback = std::function<bool(const std::string &authID)>;
using StatisticsCallback = std::function<void(const std::vector<uint32_t> &errors)>;
using MethodTimeoutCallback = std::function<void(const MethodTimeoutBody &methodInfo)>;
using DiagnosisCallback = std::function<void(const ServiceID service, const InstanceID instance,
    const DiagnosisType type)>;
using CounterCallback = std::function<void(const DiagnosisCounterType type, const uint16_t counter)>;
using FaultsCallback = std::function<void(const ServiceID service, const InstanceID instance,
    const FaultsReportType type)>;
using EventPktStatisticsCallback = std::function<void(const std::string &statistics)>;

class SomeipInterface {
public:
    /**
     * Creates a someip application object.
     *
     * @req{ AR-nStack-AP-SOMEIP-00088,
     * Support creating a SOMEIP instance,
     * QM,
     * DR-nStack-AP-SOMEIP-00006
     * }
     *
     * @param [in] name the application name
     * @param [in] config the config path
     * @param [in] id the client id of application
     * @param [in] network the network
     * @return someip application object
     *
     * @synchronous{true}
     * @reentrant{true}
     */
    static std::shared_ptr<SomeipInterface> CreateSomeipInterface(
        const std::string &name = "", const std::string &config = "",
        uint16_t id = ILLEGAL_CLIENT_ID, const std::string &network = "",
        bool enableIncrementAddConfig = false);

    /**
     * Create an empty message object.
     *
     * @req{ AR-nStack-AP-SOMEIP-00005,
     * Support creating a SOMEIP message implementation,
     * QM,
     * DR-nStack-AP-SOMEIP-00006
     * }
     *
     * @param [in] reliable determines whether this message shall be sent over a reliable connection or not
     * @return message
     *
     * @synchronous{true}
     * @reentrant{true}
     */
    static std::shared_ptr<Message> CreateMessage(bool reliable = false);

    /**
     * Create an empty request message.
     *
     * @req{ AR-nStack-AP-SOMEIP-00005,
     * Support creating a SOMEIP message implementation,
     * QM,
     * DR-nStack-AP-SOMEIP-00006
     * }
     *
     * @param [in] reliable determines whether this message shall be sent over a reliable connection or not
     * @return request message
     *
     * @synchronous{true}
     * @reentrant{true}
     */
    static std::shared_ptr<Message> CreateRequestMessage(bool reliable = false);

    /**
     * Create an empty response message from a given request message.
     *
     * @req{ AR-nStack-AP-SOMEIP-00005,
     * Support creating a SOMEIP message implementation,
     * QM,
     * DR-nStack-AP-SOMEIP-00006
     * }
     *
     * @param [in] request the request message that shall be answered by the response message
     * @return response message
     *
     * @synchronous{true}
     * @reentrant{true}
     */
    static std::shared_ptr<Message> CreateResponseMessage(const std::shared_ptr<Message> &request);

    /**
     * Create an empty notification message.
     *
     * @req{ AR-nStack-AP-SOMEIP-00005,
     * Support creating a SOMEIP message implementation,
     * QM,
     * DR-nStack-AP-SOMEIP-00006
     * }
     *
     * @param [in] reliable determines whether this message shall be sent over a reliable connection or not
     * @return notification message
     *
     * @synchronous{true}
     * @reentrant{true}
     */
    static std::shared_ptr<Message> CreateNotification(bool reliable = false);

    /**
     * Create an empty payload object.
     *
     * @req{ AR-nStack-AP-SOMEIP-00008,
     * Support creating a payload implementation,
     * QM,
     * DR-nStack-AP-SOMEIP-00006
     * }
     *
     * @return payload object
     *
     * @synchronous{true}
     * @reentrant{true}
     */
    static std::shared_ptr<Payload> CreatePayload();

    /**
     * Create a payload object filled with the given data.
     *
     * @req{ AR-nStack-AP-SOMEIP-00008,
     * Support creating a payload implementation,
     * QM,
     * DR-nStack-AP-SOMEIP-00006
     * }
     *
     * @param [in] data bytes to be copied into the payload object
     * @param [in] size number of bytes to be copied into the payload object
     * @return payload object
     *
     * @synchronous{true}
     * @reentrant{true}
     */
    static std::shared_ptr<Payload> CreatePayload(const uint8_t *data, uint32_t size);

    /**
     * Create a payload object filled with the given data.
     *
     * @req{ AR-nStack-AP-SOMEIP-00008,
     * Support creating a payload implementation,
     * QM,
     * DR-nStack-AP-SOMEIP-00006
     * }
     *
     * @param [in] data bytes to be copied into the payload object
     * @return payload object
     *
     * @synchronous{true}
     * @reentrant{true}
     */
    static std::shared_ptr<Payload> CreatePayload(const std::vector<uint8_t> &data);

    virtual ~SomeipInterface() = default;

    virtual const std::string &GetName() const = 0;
    virtual ClientID GetClientID() const = 0;
    virtual SessionID GetSessionId() = 0;
    virtual ProtocolVersion GetProtocolVersion() const = 0;
    virtual InterfaceVersion GetDefaultInterfaceVersion() const = 0;
    virtual bool Init() = 0;
    virtual void Start() = 0;
    virtual void Stop() = 0;
    virtual bool IsRouting() const = 0;
    virtual void OfferService(ServiceID service, InstanceID instance,
        MajorVersion major = DEFAULT_MAJOR_VERSION, MinorVersion minor = DEFAULT_MINOR_VERSION) = 0;
    virtual void StopOfferService(ServiceID service, InstanceID instance,
        MajorVersion major = DEFAULT_MAJOR_VERSION, MinorVersion minor = DEFAULT_MINOR_VERSION) = 0;
    virtual void OfferEvent(ServiceID service, InstanceID instance, EventID event,
        const std::set<EventgroupID> &eventgroups, bool isField) = 0;
    virtual void OfferCycleEvent(ServiceID service, InstanceID instance, EventID event,
        const std::set<EventgroupID> &eventgroups, bool isField, std::chrono::milliseconds cycle,
        bool changeResetsCycle, const EpsilonChangeCallback &epsilonChangeCallback) = 0;
    virtual void StopOfferEvent(ServiceID service, InstanceID instance, EventID event) = 0;
    virtual void BookService(ServiceID service, InstanceID instance,
        MajorVersion major = ANY_MAJOR_VERSION, MinorVersion minor = ANY_MINOR_VERSION,
        bool useExclusiveProxy = false) = 0;
    virtual void UnbookService(ServiceID service, InstanceID instance) = 0;
    virtual void BookEvent(ServiceID service, InstanceID instance, EventID event,
        const std::set<EventgroupID> &eventgroups, bool isField) = 0;
    virtual void UnbookEvent(ServiceID service, InstanceID instance, EventID event) = 0;
    virtual void Subscribe(ServiceID service, InstanceID instance, EventgroupID eventgroup,
        MajorVersion major = DEFAULT_MAJOR_VERSION,
        SubscriptionType subscriptionType = SubscriptionType::RELIABLE_AND_UNRELIABLE,
        EventID event = ANY_EVENT_ID, uint16_t port = USE_ILLEGAL_PORT) = 0;
    virtual void UnsubscribeEvent(ServiceID service, InstanceID instance,
        EventgroupID eventgroup, EventID event) = 0;
    virtual void UnsubscribeEventGroup(ServiceID service, InstanceID instance, EventgroupID eventgroup) = 0;
    virtual void SendMessage(std::shared_ptr<Message> message, bool flush = true, bool enableE2E = false) = 0;
    virtual bool SendRequestMessage(std::shared_ptr<Message> message, SessionID &session,
        bool flush = true, bool enableE2E = false) = 0;
    virtual void Notify(ServiceID service, InstanceID instance, EventID event, std::shared_ptr<Payload> payload,
        bool force = false, bool flush = true, bool enableE2E = false) = 0;
    virtual void NotifyOne(ServiceID service, InstanceID instance, EventID event,
        std::shared_ptr<Payload> payload, ClientID client,
        bool force = false, bool flush = true, bool enableE2E = false) = 0;
    virtual void RegisterStateCallback(const StateCallback &handler) = 0;
    virtual void UnregisterStateCallback() = 0;
    virtual void RegisterMessageCallback(ServiceID service, InstanceID instance, MethodID method,
        const MessageCallback &handler) = 0;
    virtual void UnregisterMessageCallback(ServiceID service, InstanceID instance, MethodID method) = 0;
    virtual void RegisterNotificationCallback(ServiceID service, InstanceID instance, EventID event,
        const MessageCallback &handler) = 0;
    virtual void UnregisterNotificationCallback(ServiceID service, InstanceID instance, EventID event) = 0;
    virtual void RegisterAvailabilityCallback(ServiceID service, InstanceID instance,
        const AvailabilityCallback &handler, MajorVersion major = DEFAULT_MAJOR_VERSION,
        MinorVersion minor = DEFAULT_MINOR_VERSION) = 0;
    virtual void UnregisterAvailabilityCallback(ServiceID service, InstanceID instance,
        MajorVersion major = DEFAULT_MAJOR_VERSION, MinorVersion minor = DEFAULT_MINOR_VERSION) = 0;
    virtual void RegisterSubscriptionStatusCallback(ServiceID service, InstanceID instance,
        EventgroupID eventgroup, EventID event, const SubscriptionStatusCallback &handler,
        bool isSelective = false) = 0;
    virtual void UnregisterSubscriptionStatusCallback(ServiceID service, InstanceID instance,
        EventgroupID eventgroup, EventID event) = 0;
    virtual void RegisterE2ECallback(const E2ECallback &handler) = 0;
    virtual void UnregisterE2ECallback() = 0;
    virtual void ClearAllCallback() = 0;
    virtual bool IsServiceAvailable(ServiceID service, InstanceID instance,
        MajorVersion major = DEFAULT_MAJOR_VERSION, MinorVersion minor = DEFAULT_MINOR_VERSION) = 0;
    virtual bool AreAvailable(AvailableMap &available,
        ServiceID service = ANY_SERVICE_ID, InstanceID instance = ANY_INSTANCE_ID,
        MajorVersion major = ANY_MAJOR_VERSION, MinorVersion minor = ANY_MINOR_VERSION) = 0;
    virtual void AddConfig(const std::string &section, const std::string &json, const UpdateConfigCallback &cb) = 0;
    virtual void DelConfig(const std::string &section, const std::string &json, const UpdateConfigCallback &cb) = 0;
    virtual void QueryConfig(const std::string &section, const QueryConfigCallback &cb) = 0;
    virtual void RegisterClientSSLCTX(const SSLCTXCallback &handler) = 0;
    virtual void RegisterServerSSLCTX(const SSLCTXCallback &handler) = 0;
    virtual void RegisterClientDTLSCTX(const SSLCTXCallback &handler) = 0;
    virtual void RegisterServerDTLSCTX(const SSLCTXCallback &handler) = 0;
    virtual bool SetAuthID(const std::string &authID) = 0;
    virtual void RegisterCheckAuthIDCallback(const CheckAuthIDCallback &handler) = 0;
    virtual void UnregisterCheckAuthIDCallback() = 0;
    virtual SomeipErrnoCode GetLastErrno() = 0;
    virtual void RegisterStatisticsCallback(const StatisticsCallback &handler) = 0;
    virtual void UnregisterStatisticsCallback() = 0;
    virtual void RequestStatistics() = 0;
    virtual bool SetTcpReconnectTime(uint32_t startMs, uint32_t increaseMs, uint32_t finalMs) = 0;
    virtual void AddServiceFilter(ServiceID service, InstanceID instance,
        const std::string &address, FilterAction action) = 0;
    virtual void RegisterMethodTimeoutCallback(const MethodTimeoutCallback &handler) = 0;
    virtual void UnregisterMethodTimeoutCallback() = 0;
    virtual void ResetDiagnosisCounterReport(ResetDiagnosisCounterType counterType) = 0;
    virtual void SetDiagnosisReportCallback(const DiagnosisCallback &handler) = 0;
    virtual void SetDiagnosisCounterReportCallback(const CounterCallback &handler) = 0;
    virtual void SetFaultsReportCallback(const FaultsCallback &handler) = 0;
    virtual bool GetEventPktStatistics(const EventPktStatisticsCallback &handler,
        ServiceID service, InstanceID instance, EventID event) = 0;
};
}

#endif
