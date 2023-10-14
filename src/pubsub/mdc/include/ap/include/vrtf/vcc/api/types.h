/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: Define types in communication mannger
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_API_INTERNAL_DRIVERTYPE_H
#define VRTF_VCC_API_INTERNAL_DRIVERTYPE_H
#include <map>
#include <sstream>
#include "vrtf/vcc/api/param_struct_typse.h"
#include "vrtf/vcc/api/method_error.h"
#include "vrtf/vcc/api/raw_buffer_helper.h"
namespace vrtf {
namespace vcc {
namespace api {
namespace types {
struct FileOwner {
    std::string user;
    std::string group;
};
class ResourceAttr {
public:
    ResourceAttr() = default;
    ResourceAttr(const FileOwner& owner);
    virtual ~ResourceAttr() = default;
    void SetFileOwner(const FileOwner& owner);
    FileOwner GetFileOwner() const;
private:
    FileOwner fileOwner_;
};

enum class ResourceType : uint32_t {
    SHM
};

class ResourcePara {
public:
    explicit ResourcePara(const ResourceType &type);
    virtual ~ResourcePara() = default;
    ResourceType GetType() const noexcept;
private:
    ResourceType type_;
};

class ShmObject : public ResourcePara {
public:
    ShmObject(const ResourceType &type, const std::string &para);
    ~ShmObject() = default;
    std::string GetValue() const noexcept;
private:
    std::string para_;
};

using ResourceCreateHandler = std::function<bool(const std::shared_ptr<ResourcePara>&)>;

class EntityInfo {
public:
    EntityInfo() = default;
    virtual ~EntityInfo() = default;
    void SetEntityId(const EntityId& id) { id_ = id; }
    EntityId GetEntityId() const { return id_; }
    ServiceId GetServiceId() const { return serviceId_; }
    void SetServiceId (ServiceId id) { serviceId_ = id; }
    InstanceId GetInstanceId() const { return instanceId_; }
    uint16_t GetU16InstanceId() const
    {
        return u16InstanceId_;
    }
    void SetInstanceId(const InstanceId& id)
    {
        instanceId_ = id;
        // this id is get from service callback para (handle type) which will ensure is a uint16_t
        u16InstanceId_ = static_cast<uint16_t>(std::stoi(id));
    }
    void SetVersion(const VersionInfo& version) { version_ = version; }
    VersionInfo GetVersion() const { return version_; }
    void SetShortName(const ShortName& shortName) { shortName_ = shortName; }
    ShortName GetShortName() const { return shortName_; }
    void SetInstanceShortName(const std::map<bool, ShortName>& instanceShortName)
    {
        instanceShortName_ = instanceShortName;
    }
    std::map<bool, ShortName> GetInstanceShortName() const { return instanceShortName_; }
    const NetworkIp& GetNetwork() const;
    void SetNetwork(const NetworkIp& network);
    void SetConfigInfoByApi(const bool setConfigInfoByApi) { setConfigInfoByApi_ = setConfigInfoByApi; }
    bool IsSetConfigInfoByApi() const { return setConfigInfoByApi_; }

    void SetSerializationType(const vrtf::serialize::SerializationType& serializationType)
    {
        serializationType_ = serializationType;
    }
    /**
     * @brief  Set E2EInfo which will used in driver
     *
     * @param[in] e2eObject   The pointer points to E2EXf_Object object
     * @param[in] e2eXfCmConfig  The pointer points to E2EXf_CMConfig object
     */
    void SetE2EInfo(const std::shared_ptr<vrtf::com::e2e::E2EXf_Object>& e2eObject,
                    const std::shared_ptr<vrtf::com::e2e::E2EXf_CMConfig>& e2eXfCmConfig);
    /**
     * @brief  Get E2EXf_Object which contains DataID/DataIDList, offset and counter info
     *
     * @return std::shared_ptr<vrtf::com::e2e::E2EXf_Object>   The pointer points to E2EXf_Object object
     */
    std::shared_ptr<vrtf::com::e2e::E2EXf_Object> GetE2EObject() const;

    /**
     * @brief Set a ResourceAttr
     *
     * @param[in] resourceAttr File's user and group
     */
    void SetResourceAttr(const ResourceAttr& resourceAttr) noexcept;

    /**
     * @brief Get a ResourceAttr
     *
     * @return ResourceAttr File's user and group
     */
    ResourceAttr GetResourceAttr() const noexcept;

    void SetResourceCreateHandler(const ResourceCreateHandler &handler);

    ResourceCreateHandler GetResourceCreateHandler() const noexcept;

    /**
     * @brief Get a E2EXfCMConfig
     *
     * @return Collection of all E2E configurations
     */
    std::shared_ptr<vrtf::com::e2e::E2EXf_CMConfig> GetE2EXfCMConfig() const noexcept { return e2eXfCmConfig_; }

    /**
     * @brief If allow to set e2e errorcode in method proxy
     *
     * @return bool
     *      @retval false   not allowed
     *      @retval true    allowed
     */
    bool IsSettingResponseE2EErrc() const { return isSetResponseE2EErrc_; }

    /**
     * @brief  Set if allow to set e2e errorcode in method proxy
     *
     * @param[in] isSetResponseE2EErrc   if allow to set e2e errorcode in method proxy
     */
    void SetIsSettingResponseE2EErrc(bool isSetResponseE2EErrc) { isSetResponseE2EErrc_ = isSetResponseE2EErrc; }
private:
    EntityId id_ = ANY_METHODID;
    ServiceId serviceId_ {0};
    InstanceId instanceId_ = "65535";
    uint16_t u16InstanceId_ = 65535;
    VersionInfo version_;
    ShortName shortName_;
    std::map<bool, ShortName> instanceShortName_;
    NetworkIp network_ = UNDEFINED_NETWORK;
    bool setConfigInfoByApi_ {false};
    vrtf::serialize::SerializationType serializationType_ = vrtf::serialize::SerializationType::CM;
    std::shared_ptr<vrtf::com::e2e::E2EXf_Object> e2eObject_ = nullptr;
    std::shared_ptr<vrtf::com::e2e::E2EXf_CMConfig> e2eXfCmConfig_ = nullptr;
    ResourceAttr resourceAttr_;
    // If setting E2E Error Code of received response checking result in Proxy
    bool isSetResponseE2EErrc_ {false};
    ResourceCreateHandler resHandler_;
};

class EventInfo : public EntityInfo {
public:

    EventInfo() = default;
    virtual ~EventInfo() = default;

    void SetIsField(bool isField) { isField_ = isField; }
    bool GetIsField() const { return isField_; }
    void SetCacheSize(const size_t size) { cacheSize_ = size; }
    size_t GetCacheSize() const { return cacheSize_; }
    void SetDataTypeName(const DataTypeName& dataTypeName) { dataTypeName_ = dataTypeName; }
    DataTypeName GetDataTypeName() const { return dataTypeName_; }
    void SetDpRawDataFlag(bool isDpRawData) { isDpRawData_ = isDpRawData; }
    bool GetDpRawDataFlag() const { return isDpRawData_; }
    void SetSerializeConfig(vrtf::serialize::SerializeConfig const &config) { serializeConfig_ = config; }
    vrtf::serialize::SerializeConfig GetSerializeConfig() const { return serializeConfig_; }
    inline void SetRawBufferHelper(const std::shared_ptr<vrtf::vcc::RawBufferHelper>& rawBufferHelper)
    {
        rawBufferHelper_ = rawBufferHelper;
    }
    inline std::shared_ptr<vrtf::vcc::RawBufferHelper> GetRawBufferHelper() const { return rawBufferHelper_; }

    /**
     * @brief Set true if the public API binding ReceivedHandler and GetNewSample, using in the RTFCOM.
     *        That is the public API of RTFCM is the combinding of ReceivedHandler and GetNewSample.
     *
     * @param[in] isCombined  If it is the combinding API
     */
    void CombindReceivedHdlAndGetSample(const bool isCombined) { isCombindReceivdHdlAndGetSample_ = isCombined; }

    /**
     * @brief Get whether it is used in the combination API, RTFCOM.
     *
     * @retval true  The EventInfo is used in combination mode
     * @retval false The EventInfo is not used in combination  mode
     */
    bool IsCombindReceivdHdlAndGetSample() const { return isCombindReceivdHdlAndGetSample_; }
private:
    bool isField_ {false};
    size_t cacheSize_ {DEFAULT_EVENT_CACHESIZE};
    DataTypeName dataTypeName_;
    bool isDpRawData_ {false};
    vrtf::serialize::SerializeConfig serializeConfig_;
    std::shared_ptr<vrtf::vcc::RawBufferHelper> rawBufferHelper_;
    bool isCombindReceivdHdlAndGetSample_ {false};
};

class MethodInfo : public EntityInfo {
public:
    MethodInfo() = default;
    virtual ~MethodInfo() {}
    void SetFireAndForget(bool isFireAndForget) { isFireAndForget_ = isFireAndForget; }
    void SetMethodCallProcessingMode(const MethodCallProcessingMode methodMode) { methodMode_ = methodMode; }
    bool GetFireAndForget() const { return isFireAndForget_; }
    MethodCallProcessingMode GetMethodCallProcessingMode() const { return methodMode_; }
    void SetIsField(bool isField) { isField_ = isField; }
    bool GetIsField() const { return isField_;}
    internal::MethodType GetMethodType() const { return methodType_; }
    void SetMethodType(const internal::MethodType methodType) { methodType_ = methodType; }
    void SetInputParameterList (const std::vector<std::string>& inputParameterList)
    {
        inputParameterList_ = inputParameterList;
    }
    std::vector<std::string> GetInputParameterList () const { return inputParameterList_; }
    void SetOutputParameterList (const std::vector<std::string>& outputParameterList)
    {
        outputParameterList_ = outputParameterList;
    }
    std::vector<std::string> GetOutputParameterList () const { return outputParameterList_; }
    void SetMethodReplyName(std::string const &replyName) { replyName_ = replyName; }
    std::string GetMethodReplyName() const { return replyName_; }
    void SetRequestSerializeConfig(const vrtf::serialize::SerializeConfig& config)
    {
        serializeRequestConfig_ = config;
    }
    vrtf::serialize::SerializeConfig GetRequestSerializeConfig() const { return serializeRequestConfig_; }
    void SetReplySerializeConfig(const vrtf::serialize::SerializeConfig& config) { serializeReplyConfig_ = config; }
    vrtf::serialize::SerializeConfig GetReplySerializeConfig() const { return serializeReplyConfig_; }
    std::deque<vrtf::vcc::api::types::internal::ErrorDomainInfo> GetMethodErrors() const { return errors_; }
    void SetMethodErrors(const std::deque<vrtf::vcc::api::types::internal::ErrorDomainInfo>& error)
    {
        errors_ = error;
    }

    std::string GetMethodUUIDInfo() const
    {
        // Composition format ServiceId.InstanceId.ShortName.MethodId
        std::stringstream methodUUID;
        methodUUID << GetServiceId() << "." << GetInstanceId() << "." << GetShortName() << "." << GetEntityId();
        return methodUUID.str();
    }
private:
    bool isFireAndForget_ {false};
    MethodCallProcessingMode methodMode_ {vrtf::vcc::api::types::MethodCallProcessingMode::kEvent};
    bool isField_ {false};
    internal::MethodType methodType_ {internal::MethodType::GENERAL_METHOD};
    std::vector<std::string> inputParameterList_;
    std::vector<std::string> outputParameterList_;
    vrtf::serialize::SerializeConfig serializeRequestConfig_;
    vrtf::serialize::SerializeConfig serializeReplyConfig_;
    std::string replyName_;
    std::deque<vrtf::vcc::api::types::internal::ErrorDomainInfo> errors_;
};

class FieldInfo {
public:
    FieldInfo(){}
    ~FieldInfo(void) = default;
    void SetEventInfo(const std::shared_ptr<EventInfo>& event) { eventInfo_ = event; }
    void SetGetterMethodInfo(const std::shared_ptr<MethodInfo>& getter) { getterMethodinfo_ = getter; }
    void SetSetterMethodInfo(const std::shared_ptr<MethodInfo>& setter) { setterMethodinfo_ = setter; }
    EntityId GetEventEntityId() const
    {
        if (eventInfo_ != nullptr) {
            return eventInfo_->GetEntityId();
        }
        return UNDEFINED_ENTITYID;
    }
    EntityId GetGetterMethodEntityId() const
    {
        if (getterMethodinfo_ != nullptr) {
            return getterMethodinfo_->GetEntityId();
        } else {
            return vrtf::vcc::api::types::UNDEFINED_ENTITYID;
        }
    }
    EntityId GetSetterMethodEntityId() const
    {
        if (setterMethodinfo_ != nullptr) {
            return setterMethodinfo_->GetEntityId();
        } else {
            return vrtf::vcc::api::types::UNDEFINED_ENTITYID;
        }
    }
    std::shared_ptr<EventInfo> GetEventInfo() const { return eventInfo_; }
    std::shared_ptr<MethodInfo> GetGetterMethodInfo() const { return getterMethodinfo_; }
    std::shared_ptr<MethodInfo> GetSetterMethodInfo() const { return setterMethodinfo_; }
    void HasSetter(bool hasSetter) { hasSetter_ = hasSetter; }
    void HasGetter(bool hasGetter) { hasGetter_ = hasGetter; }
    void HasNotifier(bool hasNotifier) { hasNotifier_ = hasNotifier; }
    bool IsHasSetter() const { return hasSetter_; }
    bool IsHasGetter() const { return hasGetter_; }
    bool IsHasNotifier() const { return hasNotifier_; }

private:
    bool hasSetter_ {false};
    bool hasGetter_ {false};
    bool hasNotifier_ {false};

    std::shared_ptr<EventInfo> eventInfo_;
    std::shared_ptr<MethodInfo> getterMethodinfo_;
    std::shared_ptr<MethodInfo> setterMethodinfo_;
};
class MethodMsg {
public:
    MethodMsg()
        : entityId_(ANY_METHODID),
          sessionId_(UNDEFINED_SERVICEID),
          payload_(nullptr),
          size_(DEFAULT_EVENT_CACHESIZE),
          type_(INVALIDTYPE),
          instanceId_(UNDEFINED_INSTANCEID),
          e2eReceivedRequestCounter_(0U){}
    virtual ~MethodMsg() { payload_ = nullptr; }
    EntityId GetEntityId() const { return entityId_; }
    DriverType GetDriverType() const { return type_; }
    const uint8_t* GetPayload() const { return payload_; }
    std::size_t GetSize() const { return size_; }
    std::uint16_t GetSessionId() const { return sessionId_; }
    void SetEntityId(EntityId const entityId) { entityId_ = entityId; }
    void SetDriverType(DriverType const driverType) { type_ = driverType; }
    void SetPayload(uint8_t* payload) { payload_ = payload; }
    void SetSize(const size_t size) { size_ = size; }
    void SetSessionId(SessionId const sessionId) { sessionId_ = sessionId; }
    void SetMsgType(bool isErrorMsg) { isErrorMsg_ = isErrorMsg; }
    bool GetMsgType() const { return isErrorMsg_; }
    void SetTrafficCtrlFlag(const bool trafficControl) { trafficControl_ = trafficControl; }
    bool GetTrafficCtrlFlag() const { return trafficControl_; }
    void SetSerializeType(vrtf::serialize::SerializeType type) { serializeType_ = type; }
    vrtf::serialize::SerializeType GetSerializeType() const { return serializeType_; }

    /**
     * @brief Set E2E Checking result that will be used in vcc
     *
     * @param[in] e2eResult  E2E Checking result
     */
    void SetE2EResult(ara::com::e2e::Result const &e2eResult) { e2eResult_ = e2eResult; }

    /**
     * @brief Get E2E checking result
     *
     * @return ara::com::e2e::Result
     */
    ara::com::e2e::Result GetE2EResult() const { return e2eResult_; }

    /**
     * @brief Set E2E counter that will be used by server send reply
     *
     * @param[in] e2eReceivedRequestCounter  E2E counter received by request msg
     */
    void SetE2ECounter(std::uint32_t e2eReceivedRequestCounter)
    {
        e2eReceivedRequestCounter_ = e2eReceivedRequestCounter;
    }

    /**
     * @brief Get E2E counter
     *
     * @return std::uint32_t
     */
    std::uint32_t GetE2ECounter() const { return e2eReceivedRequestCounter_; }
    /**
     * @brief Uniquely identifies the source of a type of message, as one of the identifiers of multithreaded
        scheduling tasks in the thread pool
     * @details Public base of Send request
     *
     * @return Source ID of a type of message, which composed of low and hight.
     */
    std::pair<uint64_t, uint64_t> GetMsgSourceIdToVcc() const { return msgSourceId_; }
    void SetMsgSourceIdToVcc(const std::pair<uint64_t, uint64_t> &msgSourceId) { msgSourceId_ = msgSourceId; }
private:
    EntityId entityId_;
    SessionId sessionId_;
    uint8_t *payload_;
    size_t size_;
    DriverType type_;
    InstanceId instanceId_;
    bool isErrorMsg_ {false};
    bool trafficControl_ {false};
    vrtf::serialize::SerializeType serializeType_ = vrtf::serialize::SerializeType::SHM;
    ara::com::e2e::Result e2eResult_ {ara::com::e2e::Result(ara::com::e2e::ProfileCheckStatus::kCheckDisabled,
        ara::com::e2e::SMState::kStateMDisabled)};
    std::uint32_t e2eReceivedRequestCounter_;
    std::pair<uint64_t, uint64_t> msgSourceId_ {0, 0}; // first is Hight 64 bit, sencond is low 64 bit
};
class EventMsg {
public:
    /**
     * @brief Uniquely identifies the source of a type of message, as one of the identifiers of multithreaded
        scheduling tasks in the thread pool
     * @details Public base of Send request
     *
     * @return Source ID of a type of message, which composed of low and hight.
     */
    std::pair<uint64_t, uint64_t> GetMsgSourceIdToVcc() const { return msgSourceId_; }
    void SetMsgSourceIdToVcc(const std::pair<uint64_t, uint64_t> &msgSourceId) { msgSourceId_ = msgSourceId; }
private:
    std::pair<uint64_t, uint64_t> msgSourceId_ {0, 0}; // first is Hight 64 bit, sencond is low 64 bit
};

using ServiceAvailableHandler = std::function<void(const std::vector<HandleType>&)>;
using EventHandleReceiveHandler = std::function<void(const vrtf::vcc::api::types::EventMsg&)>;
using EventReceiveHandler = std::function<void()>;
using MethodReceiveHandler = std::function<void(const std::shared_ptr<MethodMsg>&)>;
using SubscriberMatchedHandler = std::function<void()>;

namespace errorCode {
constexpr ErrorCode ok {0};
constexpr ErrorCode serviceNotAvailable {0};
constexpr ErrorCode maxSamplesExceeded {0};
constexpr ErrorCode networkBindingFailure {0};
}
}
}
}
}
#endif
