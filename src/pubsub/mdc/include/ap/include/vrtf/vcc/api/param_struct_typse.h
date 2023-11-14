/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: Define Info types in communication mannger
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_API_INTERNAL_INFOTYPE_H
#define VRTF_VCC_API_INTERNAL_INFOTYPE_H
#include <cstdint>
#include <vector>
#include <deque>
#include <functional>
#include <type_traits>
#include <memory>
#include <map>
#include "ara/core/string.h"
#include "ara/core/future.h"
#include "ara/core/promise.h"
#include "ara/core/vector.h"
#include "vrtf/vcc/api/sample_ptr.h"
#include "vrtf/vcc/utils/plog_info.h"
#include "vrtf/vcc/serialize/serialize_config.h"
#ifdef RTFCM_ENABLE
#include "vrtf/vcc/api/raw_data.h"
#endif
namespace vrtf {
namespace core {
template <class T>
using Future = ara::core::Future<T>;
template <class T>
using Promise = ara::core::Promise<T>;
using ErrorCode = ara::core::ErrorCode;
using ErrorDomain = ara::core::ErrorDomain;
using String = ara::core::String;
using Exception = ara::core::Exception;
using FutureException = ara::core::FutureException;
using FutureErrorDomain = ara::core::FutureErrorDomain;
template <class T, class E = vrtf::core::ErrorCode>
using Result = ara::core::Result<T, E>;
template <class T>
using Vector = ara::core::Vector<T>;
}
namespace vcc {
namespace api {
namespace types {
namespace internal {
enum class MethodType {
    GENERAL_METHOD,
    FIELD_SETTER,
    FIELD_GETTER
};

class ErrorDomainInfo {
public:
    ErrorDomainInfo(const std::string& domainName,
                    const vrtf::core::ErrorDomain::IdType& domainId)
        : name_(domainName), id_(domainId)
    {
    }
    ~ErrorDomainInfo() = default;
    vrtf::core::ErrorDomain::IdType Id() const
    {
        return id_;
    }

    std::string Name() const
    {
        return name_;
    }

private:
    std::string name_;
    vrtf::core::ErrorDomain::IdType id_;
};
}

typedef enum {
    PROLOCTYPE,
    DDSTYPE,
    SOMEIPTYPE,
    INVALIDTYPE
} DriverType;

enum class EntityType: uint8_t {
    EVENT = 0,
    METHOD = 1,
    UNKNOW = 255
};
enum class MOVEBIT : std::uint8_t {
    MOVE8BIT = 8,
    MOVE16BIT = 16,
    MOVE32BIT = 32,
    MOVE48BIT = 48,
    MOVE56BIT = 56
};
using SerializationType = vrtf::serialize::SerializationType;
using StructSerializationPolicy = vrtf::serialize::StructSerializationPolicy;
using SerializeType = vrtf::serialize::SerializeType;
using SerializeConfig = vrtf::serialize::SerializeConfig;

static const std::map<DriverType, std::string> DriverTypeMap {
    std::map<DriverType, std::string>::value_type(DriverType::PROLOCTYPE, "proloct protocol"),
    std::map<DriverType, std::string>::value_type(DriverType::DDSTYPE, "dds protocol"),
    std::map<DriverType, std::string>::value_type(DriverType::SOMEIPTYPE, "someip protocol")
};

typedef enum {
    REQUEST,
    REPLY
} MethodType;

enum class ReceiveType: uint8_t {
    OK,
    FULLSLOT,
    EMPTY_CONTAINER,
    E2EFAIL
};

using ServiceId = std::uint16_t;
using ServiceName = std::string;
using InstanceId = std::string;
using NetworkIp = std::pair<std::string, bool>;
using EntityId = std::uint32_t;
using ErrorCode = std::int32_t;
using MajorVersionId = std::uint8_t;
using MinorVersionId = std::uint32_t;
using ClientId = std::uint16_t;
using SessionId = std::uint16_t;
using ShortName = std::string;
using DataTypeName = std::string;
using EventTypeName = std::string;

const size_t MAX_EVENT_SUB_COUNT = 1000;
InstanceId const UNDEFINED_INSTANCEID = "65534";
constexpr ServiceId UNDEFINED_SERVICEID = 0xFFFE;
constexpr std::uint32_t UNDEFINED_UID = 0xFFFFFFFFU;
constexpr EntityId UNDEFINED_ENTITYID = 0xFFFFFFFEU;
InstanceId const ANY_INSTANCEID = "65535";
constexpr EntityId ANY_METHODID = 0xFFFF;
constexpr ServiceId ANY_SERVICEID = 0xFFFF;
constexpr MajorVersionId ANY_MAJOR_VERSIONID = 0xFF;
constexpr MinorVersionId ANY_MINOR_VERSIONID = 0xFFFFFFFFU;
constexpr size_t DEFAULT_EVENT_CACHESIZE = 10;
constexpr std::uint32_t DEFAULT_LOG_LIMIT {2};
const size_t TIME_STAMP_SIZE = sizeof(time_t) + sizeof(long);
std::string const UNDEFINED_SERVICE_NAME = "UNDEFINED_SERVICE_NAME";
std::string const UNDEFINED_QOS_PROFILE = "UNDEFINED_QOS_PROFILE";
NetworkIp const UNDEFINED_NETWORK = {"UNDEFINED_NETWORK", false};
NetworkIp const BUILTIN_APP_CLIENT_NETWORK = {"BUILTIN_APP_CLIENT_FLAG", false};
NetworkIp const BUILTIN_APP_SERVER_NETWORK = {"BUILTIN_APP_SERVER_FLAG", false};
enum class ReturnCode: uint8_t {
    OK = 0,
    ERROR = 1
};

struct Result {
    std::shared_ptr<uint8_t> data;
    size_t length;
};

struct ServiceHandle {
    ServiceId serviceId_;
    InstanceId instanceId_;
    friend bool operator< (const ServiceHandle& a, const ServiceHandle& b)
    {
        bool result = false;
        if (a.serviceId_ < b.serviceId_) {
            result = true;
        } else if (a.serviceId_ == b.serviceId_) {
            result = a.instanceId_ < b.instanceId_;
        } else {
            // do nothing
        }
        return result;
    }
};

enum class EventSubscriptionState : uint8_t {
    kSubscribed,
    kNotSubscribed,
    kSubscriptionPending
};

enum class MethodState : uint8_t {
    kMethodOnline,
    kMethodOffline
};

enum class EventUpdateMode {
    LAST_N,
    NEWEST_N
};

enum class MethodCallProcessingMode: uint8_t {
    kPoll = 0,
    kEvent = 1,
    kEventSingleThread = 2
};

template <typename T>
using SampleContainer = std::deque<T>;
using E2EResultCache = std::deque<ara::com::e2e::Result>;
// 19-11 SWS_CM_00302
class InstanceIdentifier {
public:
    static InstanceId const Any;
    ~InstanceIdentifier(void) = default;
    explicit InstanceIdentifier(const ara::core::StringView& value);
    ara::core::StringView ToString() const;
    bool operator==(const InstanceIdentifier& other) const;
    bool operator<(const InstanceIdentifier& other) const;
    InstanceIdentifier& operator=(const InstanceIdentifier& other);
    const InstanceId GetInstanceIdString() const
    {
        return stringId_;
    }

private:
    InstanceId stringId_ = UNDEFINED_INSTANCEID;
};

class HandleType {
public:
    HandleType(const ServiceId& serviceId, const InstanceId& id, const DriverType& driver)
        : serviceId_(serviceId), id_(id), driver_(driver) {}
    // SWS_CM_00317
    HandleType(const HandleType &other) = default;
    HandleType& operator=(const HandleType& other) = default;

    // SWS_CM_00318
    HandleType(HandleType &&other) = default;
    HandleType& operator=(HandleType &&other) = default;

    ~HandleType() = default;
    DriverType GetDriver() const
    {
        return driver_;
    }
    InstanceIdentifier GetInstanceId() const
    {
        return InstanceIdentifier(ara::core::StringView(id_.c_str()));
    }
    ServiceId GetServiceId() const
    {
        return serviceId_;
    }

    bool operator<(const HandleType &other) const
    {
        if (driver_ < other.driver_) {
            return true;
        }

        if (driver_ == other.driver_) {
            if (serviceId_ < other.serviceId_) {
                return true;
            }
        }

        if ((driver_ == other.driver_) && (serviceId_ == other.serviceId_)) {
            if (id_ < other.id_) {
                return true;
            }
        }
        return false;
    }

    bool operator==(const HandleType &other) const
    {
        if ((id_ == other.id_) && (driver_ == other.driver_) && (serviceId_ == other.serviceId_)) {
            return true;
        }
        return false;
    }
private:
    ServiceId serviceId_ = UNDEFINED_SERVICEID;
    InstanceId id_;
    DriverType driver_;
};

template<typename T>
using ServiceHandleContainer = std::vector<T>;
using SubscriptionStateChangeHandler = std::function<void(EventSubscriptionState)>;
using MethodStateChangeHandler = std::function<void(MethodState)>;
class FindServiceHandle {
public:
    FindServiceHandle() = default;
    ~FindServiceHandle(void) = default;
    bool operator==(const FindServiceHandle& other) const;
    bool operator<(const FindServiceHandle& other) const;
    FindServiceHandle& operator=(const FindServiceHandle& other) =default;

    // Internal class!!! Prohibit to use by Application!!!!
    FindServiceHandle(std::uint32_t uid)
        : uid_(uid){}
    // Internal class!!! Prohibit to use by Application!!!!
    std::uint32_t GetUID() const;

private:
    std::uint32_t uid_ = vrtf::vcc::api::types::UNDEFINED_UID;
};

template<typename T>
using FindServiceHandler = std::function<void(ServiceHandleContainer<T>, FindServiceHandle)>;
class ApplicationName {
public:
    static std::shared_ptr<ApplicationName>& GetInstance()
    {
        static std::shared_ptr<ApplicationName> instance(std::make_shared<ApplicationName>());
        return instance;
    }

    void SetName(const std::string& inputApplicationName);
    std::string GetName()
    {
        return applicationName_;
    }
private:
    std::string applicationName_;
};
class VersionInfo {
public:
    VersionInfo(MajorVersionId major, MinorVersionId serviceMinor)
        : majorVersion_(major), serviceMinorVersion_(serviceMinor) {}
    VersionInfo(){}
    ~VersionInfo(void) = default;
    void SetMajorVersion(const MajorVersionId& major)
    {
        majorVersion_ = major;
    }
    void SetServiceMinorVersion(const MinorVersionId& serviceMinor)
    {
        serviceMinorVersion_ = serviceMinor;
    }
    void SetInstanceMinorVersion(const MinorVersionId& instanceMinor)
    {
        instanceMinorVersion_ = instanceMinor;
    }
    MajorVersionId GetMajorVersion() const
    {
        return majorVersion_;
    }
    MinorVersionId GetServiceMinorVersion() const
    {
        return serviceMinorVersion_;
    }
    MinorVersionId GetInstanceMinorVersion() const
    {
        return instanceMinorVersion_;
    }
    bool operator==(const VersionInfo& other) const
    {
        return (majorVersion_ == other.GetMajorVersion()) && (serviceMinorVersion_ == other.GetServiceMinorVersion());
    }

    bool operator<(const VersionInfo& other) const
    {
        if (majorVersion_ < other.GetMajorVersion()) {
            return true;
        } else if (majorVersion_ == other.GetMajorVersion()) {
            return serviceMinorVersion_ < other.GetServiceMinorVersion();
        } else {
            // do nothing
        }

        return false;
    }

    VersionInfo& operator=(const VersionInfo& other) = default;
private:
    MajorVersionId majorVersion_ = ANY_MAJOR_VERSIONID;
    MinorVersionId serviceMinorVersion_ = ANY_MINOR_VERSIONID;
    MinorVersionId instanceMinorVersion_ = ANY_MINOR_VERSIONID;
};
class SampleTimeInfo {
public:
    void SetReceiveModule(const std::uint8_t& module)
    {
        receiveModule_ = module;
    }

    const std::uint8_t& GetReceiveModule() const
    {
        return receiveModule_;
    }

    void SetPlogId(const std::uint64_t& id)
    {
        plogId_ = id;
    }

    const std::uint64_t& GetPlogId() const
    {
        return plogId_;
    }

    void SetTakeTime(const timespec& time)
    {
        takeTime_ = time;
    }

    const timespec& GetTakeTime() const
    {
        return takeTime_;
    }

    void SetServerSendTime(const timespec& time)
    {
        serverSendTime_ = time;
    }

    const timespec& GetServerSendTime() const
    {
        return serverSendTime_;
    }
private:
    std::uint8_t receiveModule_ = 0;
    std::uint64_t plogId_ = 0;
    timespec takeTime_ {0, 0};
    timespec serverSendTime_ {0, 0};
};
class SampleInfo {
public:
    void SetSampleId(const std::uint64_t& id)
    {
        sampleId_ = id;
    }
    const std::uint64_t& GetSampleId() const
    {
        return sampleId_;
    }
    void SetE2EResult(ara::com::e2e::Result e2eResult)
    {
        e2eResult_ = e2eResult;
    }
    const ara::com::e2e::Result GetE2EResult() const
    {
        return e2eResult_;
    }
private:
    std::uint64_t sampleId_;
    ara::com::e2e::Result e2eResult_ = ara::com::e2e::Result(ara::com::e2e::ProfileCheckStatus::kNotAvailable,
                                                             ara::com::e2e::SMState::kStateMDisabled);
};

namespace internal {
class SampleInfoImpl {
public:
    SampleInfoImpl() = default;
    ~SampleInfoImpl() = default;
    std::shared_ptr<vrtf::vcc::utils::PlogInfo> plogInfo_ = nullptr;
    bool isEvent_ = true;
    timespec sendTime_ {0, 0};
};
}
namespace reserve {
    std::string const AP_NODE_NAME_SPACE = "/Huawei/AP";
    std::string const PHM_NODE_NAME = "PlatformHealthManagement";
    std::string const EM_NODE_NAME = "ExecutionManagement";
    // The current external service domain use the domain ID 0x02 by PHM/EM/Maintaind
    constexpr int16_t EXTERNAL_SERVICE_DOMAIN_ID = 0x02;
    // The current maintaind service domain use the domain ID 0x03 by Rtftools
    constexpr int16_t MAINTAIND_SERVICE_DOMAIN_ID = 0x03;
    // The current internal service domain use the domain ID 0x04 by PHM/EM
    constexpr int16_t INTERNAL_SERVICE_DOMAIN_ID = 0x04;
}

class ServiceDiscoveryInfo {
public:
    ServiceDiscoveryInfo()
        : serviceId_(0), instanceId_("0"), methodMode_(MethodCallProcessingMode::kEvent) {}
    ServiceDiscoveryInfo(const ServiceId& serviceId, const InstanceId& instanceId, const MethodCallProcessingMode& mode)
        : serviceId_(serviceId), instanceId_(instanceId), methodMode_(mode) {}
    virtual ~ServiceDiscoveryInfo(){}

    ServiceId GetServiceId() const noexcept
    {
        return serviceId_;
    }
    void SetServiceId (ServiceId id)
    {
        serviceId_ = id;
    }
    InstanceId GetInstanceId() const noexcept
    {
        return instanceId_;
    }
    void SetInstanceId(const InstanceId& id)
    {
        instanceId_ = id;
    }
    MethodCallProcessingMode GetMethodCallProcessingMode() const
    {
        return methodMode_;
    }
    void SetMethodCallProcessingMode(const MethodCallProcessingMode& mode)
    {
        methodMode_ = mode;
    }

    void SetVersion(const VersionInfo& version)
    {
        version_ = version;
    }

    VersionInfo GetVersion() const
    {
        return version_;
    }

    const NetworkIp& GetNetwork() const
    {
        return network_;
    }

    void SetNetwork(const NetworkIp& network)
    {
        network_ = network;
    }

    void SetConfigInfoByApi(const bool setConfigInfoByApi)
    {
        setConfigInfoByApi_ = setConfigInfoByApi;
    }
    bool IsSetConfigInfoByApi() const
    {
        return setConfigInfoByApi_;
    }
private:
    ServiceId serviceId_;
    InstanceId instanceId_;
    MethodCallProcessingMode methodMode_;
    VersionInfo version_;
    NetworkIp network_ = UNDEFINED_NETWORK;
    bool setConfigInfoByApi_ = false;
};
}
}
}
}
#endif
