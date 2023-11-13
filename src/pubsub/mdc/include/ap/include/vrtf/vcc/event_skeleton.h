/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: get VCC info and use to transfer driver server event mode
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_EVENTSKELETON_H
#define VRTF_VCC_EVENTSKELETON_H
#include <memory>
#include <map>
#include <type_traits>
#include "vrtf/vcc/utils/thread_pool.h"
#include "vrtf/vcc/api/types.h"
#include "vrtf/vcc/serialize/dds_serialize.h"
#include "vrtf/vcc/serialize/someip_serialize.h"
#include "vrtf/vcc/internal/traffic_crtl_policy.h"
#include "ara/com/com_error_domain.h"
#include "ara/hwcommon/log/log.h"
#include "ara/core/future.h"
#include "ara/core/promise.h"
#include "vrtf/vcc/utils/dp_adapter_handler.h"
#include "vrtf/vcc/api/raw_buffer.h"
#include "vrtf/vcc/driver/event_handler.h"

namespace vrtf {
namespace vcc {
/* EventHolder will be used by skeleton to initiate event operations on event */
class EventSkeleton {
public:
    EventSkeleton(vrtf::vcc::api::types::EntityId id, bool fieldFlag = false);

    virtual ~EventSkeleton() = default;

    virtual void SendInitData(void);

    void AddDriver(vrtf::vcc::api::types::DriverType type,
                   std::shared_ptr<vrtf::vcc::driver::EventHandler>& eventHandler);

    void Send(api::types::RawBuffer && rawBuffer,
              std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info);

    template<class SampleType>
    void Send(const SampleType& data, std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info) noexcept
    {
        using namespace vrtf::vcc::api::types;
        if (trafficCtrlPolicy_ != nullptr) {
            rtf::TrafficCtrlAction action = trafficCtrlPolicy_->GetTrafficCtrlAction();
            if (trafficCtrlPolicy_->UpdateTrafficInfo(action) == false) {
                return;
            }
        }
        for (auto& iter : eventHandlerMap_) {
            if (firstSendFlag_ == true) {
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance_->info()<< (IsField() ? "Field" : "Event") << " start to send data[insatnceId="
                    << GetInstanceId() << ", shortName=" << GetShortNameByDriver(iter.first) << ", protocol="
                    << DriverTypeMap.at(iter.first) << "]";
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
                firstSendFlag_ = false;
            }
            SendOut<SampleType>(iter.second, data, iter.first, info);
        }
    }

    template<class SampleType>
    void Send(std::unique_ptr<SampleType> data,
        std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info) noexcept
    {
        using namespace vrtf::vcc::api::types;
        if (trafficCtrlPolicy_ != nullptr) {
            rtf::TrafficCtrlAction action = trafficCtrlPolicy_->GetTrafficCtrlAction();
            if (trafficCtrlPolicy_->UpdateTrafficInfo(action) == false) {
                return;
            }
        }
        std::shared_ptr<typename std::decay<SampleType>::type> sendptr(data.release());
        for (auto& iter : eventHandlerMap_) {
            if (firstSendFlag_ == true) {
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance_->info()<< (IsField() ? "Field" : "Event") << " start to send data[insatnceId="
                    << GetInstanceId() << ", shortName=" << GetShortNameByDriver(iter.first) << ", protocol="
                    << DriverTypeMap.at(iter.first) << "]";
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
                firstSendFlag_ = false;
            }
            SendOut<SampleType>(iter.second, *sendptr, iter.first, info);
        }
    }

    template <typename SampleType>
    typename std::enable_if<(!vrtf::serialize::dds::is_dp_raw_data<SampleType>::value) &&
        (!vrtf::serialize::ros::IsRosMsg<SampleType>::value), bool>::type
    SendEventByDp(std::shared_ptr<vrtf::vcc::driver::EventHandler> eventHandler, const SampleType &data,
                  vrtf::vcc::api::types::DriverType driverType,
                  std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info) noexcept
    {
        vrtf::serialize::dds::Serializer<typename std::decay<SampleType>::type> sample(
            data, eventInfo_[driverType]->GetSerializeConfig());
        return SendCommonData(eventHandler, sample, info, driverType);
    }

    template <typename SampleType>
    typename std::enable_if<(!vrtf::serialize::dds::is_dp_raw_data<SampleType>::value) &&
        vrtf::serialize::ros::IsRosMsg<SampleType>::value, bool>::type
    SendEventByDp(std::shared_ptr<vrtf::vcc::driver::EventHandler> eventHandler, const SampleType &data,
                  vrtf::vcc::api::types::DriverType driverType,
                  std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info) noexcept
    {
        std::shared_ptr<vrtf::vcc::RawBufferHelper> rawBufferHelper = eventInfo_[driverType]->GetRawBufferHelper();
        if (rawBufferHelper == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Raw buffer helper is null.";
            return false;
        }
        std::string dataType = eventInfo_[driverType]->GetDataTypeName();
        if ((eventHandler->IsEnableDp()) && (!rawBufferHelper->IsBuffMsg(dataType))) {
            vrtf::serialize::dds::Serializer<typename std::decay<SampleType>::type> sample(
                data, eventInfo_[driverType]->GetSerializeConfig());
            return SendCommonData(eventHandler, sample, info, driverType);
        } else if ((eventHandler->IsEnableDp()) && (rawBufferHelper->IsBuffMsg(dataType))) {
            vrtf::serialize::dds::Serializer<typename std::decay<SampleType>::type> sample(
                data, eventInfo_[driverType]->GetSerializeConfig());
            Mbuf *mbuf = rawBufferHelper->GetMbufFromMsg(dataType,
                reinterpret_cast<void *>(const_cast<SampleType *>(&data)));
            return SendDpRawData(eventHandler, sample, mbuf, info);
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "[skeleton]Error config in ros msg.";
        }
        return false;
    }

    template <typename SampleType>
    typename std::enable_if<vrtf::serialize::dds::is_dp_raw_data<SampleType>::value, bool>::type
    SendEventByDp(std::shared_ptr<vrtf::vcc::driver::EventHandler> eventHandler, const SampleType &data,
                  vrtf::vcc::api::types::DriverType driverType,
                  std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info) noexcept
    {
        if (!((eventHandler->IsEnableDp()) && (eventHandler->IsDpRawData()))) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Failed to send raw data by dp.";
            return false;
        }
        vrtf::serialize::dds::Serializer<typename std::decay<SampleType>::type> sample(
            data, eventInfo_[driverType]->GetSerializeConfig());
        Mbuf *mbuf = data.GetMbufPtr();
        return SendDpRawData(eventHandler, sample, mbuf, info);
    }

    template<class SampleType>
    void SendOut(std::shared_ptr<vrtf::vcc::driver::EventHandler> eventHandler, const SampleType &data,
                 vrtf::vcc::api::types::DriverType driverType,
                 std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info)
    {
        if (eventHandler == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Cannot send data: event handler is null with entity id" << id_;
            return;
        }
        if ((!eventHandler->IsEnableDp()) && (eventHandler->IsDpRawData())) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "If send dp raw data, please set data plain transport qos.";
            return;
        }
        if ((eventHandler->IsEnableDp()) &&
            (eventHandler->GetSerializeType() == vrtf::serialize::SerializeType::SOMEIP)) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Someip don not support dp transport.";
            return;
        }

        if (eventHandler->GetSerializeType() == vrtf::serialize::SerializeType::SHM) {
            if (!eventHandler->IsEnableDp()) {
                vrtf::serialize::dds::Serializer<typename std::decay<SampleType>::type> sample(
                    data, eventInfo_[driverType]->GetSerializeConfig());
                (void)SendCommonData(eventHandler, sample, info, driverType);
            } else {
                bool ret = SendEventByDp<SampleType>(eventHandler, data, driverType, info);
                if (ret != true) {
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance_->error() << "Send event by dds failed.";
                }
            }
        } else if (eventHandler->GetSerializeType() == vrtf::serialize::SerializeType::SOMEIP) {
            vrtf::serialize::someip::Serializer<typename std::decay<SampleType>::type> sample(
                data, eventInfo_[driverType]->GetSerializeConfig());
            (void)SendCommonData(eventHandler, sample, info, driverType);
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() << "Wrong serialize type, deserialize error!";
        }
    }

    /**
     * @brief Allocate rawBuffer
     * @details Allocate rawBuffer
     *
     * @param size the size will be allocated
     * @return RawBuffer return one RawBuffer for pub
     *   @retval rawBuffer allocate buffer successful
     *   @retval nullptr allocate buffer failed
     */
    vrtf::vcc::api::types::RawBuffer AllocateRawBuffer(const size_t& size);

    /**
     * @brief pub rawBuffer
     * @details pub rawBuffer
     *
     * @param rawBuffer the rawBuffer class store data pointer to be send
     */
    void DeAllocateRawBuffer(api::types::RawBuffer && rawBuffer);

    /**
     * @brief pub rawBuffer
     * @details pub rawBuffer
     *
     * @param rawBuffer the rawBuffer class store data pointer to be send
     */
    void PubRawBuffer(api::types::RawBuffer && rawBuffer,
                      std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info);

    bool IsField(void) const;

    void SetShortNameByDriver(vrtf::vcc::api::types::DriverType const &drivertype, vrtf::core::String const &shortname);

    vrtf::core::String GetShortNameByDriver(vrtf::vcc::api::types::DriverType const &drivertype);

    vrtf::vcc::api::types::InstanceId GetInstanceId() const;

    void SetInstanceId(vrtf::vcc::api::types::InstanceId const &instanceId);

    void SetTrafficCtrl(const std::shared_ptr<rtf::TrafficCtrlPolicy>& policy);

    void SetEventInfo(const std::map<vrtf::vcc::api::types::DriverType,
                      std::shared_ptr<vrtf::vcc::api::types::EventInfo>>& eventInfo);
    /**
     * @brief Set this proxy latency analysis mode
     * @details Set this proxy latency analysis mode
     * @param[in] mode latency analysis mode
     */
    void SetLatencyAnalysisMode(const utils::LatencyAnalysisMode& mode);
protected:
    vrtf::vcc::api::types::EntityId const id_;
    bool fieldFlag_ = false;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    bool firstSendFlag_ = true;
    std::map<vrtf::vcc::api::types::DriverType, vrtf::core::String> shortName_;
    vrtf::vcc::api::types::InstanceId instanceId_ = vrtf::vcc::api::types::UNDEFINED_INSTANCEID;
private:
    template<class Serializer>
    bool SendCommonData(std::shared_ptr<vrtf::vcc::driver::EventHandler> &eventHandler, Serializer &serializer,
                        std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info,
                        vrtf::vcc::api::types::DriverType const &type) noexcept
    {
        size_t size {0};
        size = serializer.GetSize();
        if (size >= vrtf::serialize::someip::MAX_SOMEIP_SERIALIZE_SIZE) {
            return false;
        }
        std::uint8_t* buffer = nullptr;
        if ((delayMode_ == utils::LatencyAnalysisMode::ENABLE) && (info->isEvent_)) {
            if (size + utils::TLV_TIME_TOTAL_SIZE >= vrtf::serialize::someip::MAX_SOMEIP_SERIALIZE_SIZE) {
                return false;
            }
            buffer = const_cast<std::uint8_t*>(eventHandler->AllocateBuffer(
                static_cast<uint32_t>(size + utils::TLV_TIME_TOTAL_SIZE)));
            if (buffer == nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->warn() << "Allocate Buffer failed";
                return false;
            }
            utils::TlvHelper::AddTlvTimeStamp(
                buffer + size, info->sendTime_);
            size += utils::TLV_TIME_TOTAL_SIZE;
        } else {
            buffer = const_cast<std::uint8_t*>(eventHandler->AllocateBuffer(static_cast<uint32_t>(size)));
        }
        if (buffer == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Allocate Buffer failed";
            return false;
        }

        serializer.Serialize(buffer);
        if (info->plogInfo_ != nullptr) {
            info->plogInfo_->WriteTimeStamp(
                utils::PlogServerTimeStampNode::SERIALIZE_DATA, toPlogDriverTypeMap_[type]);
        }
        eventHandler->SendEvent(buffer, size, info);
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->debug() << "Data size is " << size << " in entity " << id_;
        return true;
    }

    template<class Serializer>
    bool SendDpRawData(std::shared_ptr<vrtf::vcc::driver::EventHandler> &eventHandler,
        Serializer &serializer, Mbuf *mbuf,
        std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info) noexcept
    {
        uint8_t *buffer = nullptr;
        size_t size = serializer.GetSize();
        if (dpHandler_ == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "DpAdapterHandler is null.";
            return false;
        }
        if (size > dpHandler_->GetAvailabeLenth()) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Too many private data for mbuf raw data. Max available size is "
                << dpHandler_->GetAvailabeLenth() << " Now is " << size;
            (void)dpHandler_->MbufFree(mbuf);
            return false;
        }
        std::uint8_t latencySize = 0;
        if ((delayMode_ == utils::LatencyAnalysisMode::ENABLE) && info->isEvent_) {
            latencySize = utils::TLV_TIME_TOTAL_SIZE;
        }
        if ((eventHandler->GetE2EHeaderSize() != vrtf::com::e2e::UNDEFINED_HEADER_SIZE) && (latencySize > 0)) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Using latency and e2e in the same time which is not supported.";
            return false;
        }
        buffer = GetRawDataSerializeBuffer(eventHandler, mbuf, size + latencySize);
        if (buffer == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Failed to get raw data serialize buffer.";
            (void)dpHandler_->MbufFree(mbuf);
            return false;
        } else { // The format in private area is [Private data length][E2E Header][Private data][Latency time stamp]
            if ((latencySize > 0) && (info->isEvent_)) { // Write latency time stamp
                utils::TlvHelper::AddTlvTimeStamp(buffer + size,
                    info->sendTime_);
            }
        }
        serializer.Serialize(buffer);
        if (info->plogInfo_ != nullptr) {
            info->plogInfo_->WriteTimeStamp(
                utils::PlogServerTimeStampNode::SERIALIZE_DATA, utils::PlogDriverType::DDS);
        }
        eventHandler->SendEvent(reinterpret_cast<uint8_t *>(mbuf), size, info);
        /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
        /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
        logInstance_->debug() << "Raw Data private size is " << size << ", and the latency size is " <<
                                 latencySize << " in entity " << id_;
        /* AXIVION enable style AutosarC++19_03-A5.0.1 */
        /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        return true;
    }

    uint8_t* GetRawDataSerializeBuffer(const std::shared_ptr<vrtf::vcc::driver::EventHandler> &eventHandler,
        Mbuf *mbuf, size_t size);

    std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::driver::EventHandler>> eventHandlerMap_;
    std::shared_ptr<vrtf::vcc::utils::DpAdapterHandler> dpHandler_ = nullptr;
    std::shared_ptr<rtf::TrafficCtrlPolicy> trafficCtrlPolicy_;
    std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::EventInfo>> eventInfo_;
    static std::map<vrtf::vcc::api::types::DriverType, vrtf::vcc::utils::PlogDriverType> toPlogDriverTypeMap_;
    utils::LatencyAnalysisMode delayMode_ = utils::LatencyAnalysisMode::DISABLE;
};

template<class T>
class FieldSkelton : public EventSkeleton {
public:
    FieldSkelton(vrtf::vcc::api::types::EntityId id, bool hasNotify)
        : EventSkeleton(id, true), isInitialized_(false), hasNotify_(hasNotify)
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
    }
    ~FieldSkelton() = default;
    /**
     * @brief field send interface
     * @details if use before offerservice, update initdata and store
     *          if use after offerservice, the data will be send by this interface
     *
     * @param[in] data the data which will be sent
     * @param[in] isOfferService if isOfferService is true represent have offerservice
     *            if isOfferService is false represent have not offerservice
     */
    template <typename U = T>
    typename std::enable_if<(!vrtf::serialize::ros::IsRosMsg<U>::value) &&
                            (!vrtf::serialize::ros::IsRosBuiltinMsg<T>::value) &&
                            (!vrtf::serialize::ros::IsShapeShifterMsg<T>::value), void>::type
    Send(const typename std::decay<T>::type& data, bool isOfferService,
         std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info)
    {
        isInitialized_ = true;
        if (isOfferService == false) {
            field_ = data;
            fieldInitValue_ = data;
            return;
        }

        if (hasNotify_ == false) {
            field_ = data;
            return;
        }

        {
            std::lock_guard<std::mutex> guard(firstSendMutex_);
            if (firstSend_ == true) {
                firstSend_ = false;
                field_ = data;
                EventSkeleton::Send<typename std::decay<T>::type>(data, info);
                return;
            }
        }
        if (data == field_) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose()<< "Update the same notifier value";
        } else {
            field_ = data;
            EventSkeleton::Send<typename std::decay<T>::type>(data, info);
        }
    }

    template <typename U = T>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value ||
                            vrtf::serialize::ros::IsRosBuiltinMsg<T>::value ||
                            vrtf::serialize::ros::IsShapeShifterMsg<T>::value, void>::type
    Send(const typename std::decay<T>::type& data, bool isOfferService,
         std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info)
    {}

    template <typename U = T>
    typename std::enable_if<!vrtf::serialize::ros::IsRosMsg<U>::value &&
        !vrtf::serialize::ros::IsRosBuiltinMsg<T>::value, void>::type
    Send(std::unique_ptr<typename std::decay<T>::type> data)
    {
        if (field_ == *data) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose()<< "Update the same notifier value";
            return;
        }
        field_ = *data;
        std::shared_ptr<api::types::internal::SampleInfoImpl> sampleInfo =
            std::make_shared<api::types::internal::SampleInfoImpl>();
        EventSkeleton::Send<typename std::decay<T>::type>(std::move(data), sampleInfo);
    }

    template <typename U = T>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<U>::value ||
        vrtf::serialize::ros::IsRosBuiltinMsg<T>::value, void>::type
    Send(std::unique_ptr<typename std::decay<T>::type> data)
    {}

    virtual void SendInitData(void) override
    {
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose()<< "New subscriber online for field " << id_;
        if (hasNotify_ == true) {
            std::shared_ptr<api::types::internal::SampleInfoImpl> sampleInfo =
                std::make_shared<api::types::internal::SampleInfoImpl>();
            EventSkeleton::Send<T>(fieldInitValue_, sampleInfo);
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose()<< "Send field value to the new subscriber.";
        }
    }

    void RegisterGetMethod(std::function<vrtf::core::Future<T>(void)> getHandler)
    {
        getHandler_ = getHandler;
    }

    void RegisterSetMethod(std::function<vrtf::core::Future<T>(const T& value)> setHandler)
    {
        setHandler_ = setHandler;
    }

    /**
     * @brief query the getHandler
     * @return getHandler or nullptr if not registered.
     */
    const std::function<vrtf::core::Future<T>()>& QueryGetMethod()
    {
        return getHandler_;
    }

    /**
     * @brief query the setHandler
     * @return setHandler or nullptr if not registered.
     */
    const std::function<vrtf::core::Future<T>(const T& value)>& QuerySetMethod()
    {
        return setHandler_;
    }

    /**
     * @brief check if the field initialized.
     * @details the initial data is set by calling Update() before OfferService() by APP.
     * @return the result of check
     *   @retval true the field has initialized
     *   @retval false the field has not initialized
     */
    bool HasInitData()
    {
        return isInitialized_;
    }

    void SetFieldStatus(bool isInit)
    {
        isInitialized_ = isInit;
    }

    vrtf::core::Future<T> UpdateField(const T& data, bool isDeserializefail = false)
    {
        vrtf::core::Promise<T> promise;
        vrtf::core::Future<T> future = promise.get_future();
        if (isDeserializefail == true) {
            if (isInitialized_ == false) {
                promise.SetError(vrtf::core::ErrorCode(vrtf::vcc::api::types::ComErrc::kNetworkBindingFailure));
            } else {
                promise.set_value(fieldInitValue_);
            }
            return future;
        }
        if (setHandler_ != nullptr) {
            auto userFutureData = setHandler_(data).get();
            std::shared_ptr<api::types::internal::SampleInfoImpl> sampleInfo =
                std::make_shared<api::types::internal::SampleInfoImpl>();
            Send(userFutureData, true, sampleInfo);
            promise.set_value(userFutureData);
        } else {
            typename std::decay<T>::type interData;
            // Fix me: error code should be return
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Unregisterd set handler is called with entityId " << id_;
            promise.set_value(interData);
        }
        return future;
    }

    virtual vrtf::core::Future<T> GetField(void)
    {
        vrtf::core::Promise<T> promise;
        vrtf::core::Future<T> future = promise.get_future();
        if (getHandler_ != nullptr) {
            auto userFuture = getHandler_();
            auto userFutureData = userFuture.get();
            promise.set_value(userFutureData);
        } else {
            // Fix me: error code should be return
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Unregisterd get handler is called with entityId " << id_;
            promise.set_value(field_);
        }
        return future;
    }

private:
    // UpdateField should be called when the obj is created
    T field_;
    T fieldInitValue_;
    bool isInitialized_ = false;
    bool hasNotify_ = false;
    bool firstSend_ = true;
    std::mutex firstSendMutex_;
    std::function<vrtf::core::Future<T>(void)> getHandler_ = nullptr;
    std::function<vrtf::core::Future<T>(const T& value)> setHandler_ = nullptr;
};
}
}
#endif
