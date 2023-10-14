/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description: get VCC info and use to transfer driver client event mode
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_EVENTPROXY_H
#define VRTF_VCC_EVENTPROXY_H
#include <deque>
#include <memory>
#include <mutex>
#include <vrtf/vcc/driver/event_handler.h>
#include "vrtf/vcc/utils/thread_pool.h"
#include "vrtf/vcc/utils/lock_free_queue.h"
#include "vrtf/vcc/utils/condition_variable.h"
#include "ara/hwcommon/log/log.h"
#include "vrtf/vcc/api/types.h"
#include "vrtf/vcc/serialize/dds_serialize.h"
#include "vrtf/vcc/serialize/someip_serialize.h"
#include "ara/com/e2e/E2EXf/E2EXf_CM.h"
#include "vrtf/vcc/api/com_error_domain.h"
#include "ara/core/abort.h"
#include "vrtf/vcc/utils/dp_adapter_handler.h"
#include "vrtf/vcc/api/recv_buffer.h"
#include "vrtf/vcc/utils/rtf_spin_lock.h"

namespace vrtf {
namespace vcc {
const size_t EVENTHANDLER_POOL_SIZE = 5;  /* 5 is the default thread pool size for event handler */
static std::uint16_t eventThreadNum {0};
/*
    We need a base class for all EventHandlers, Event !!!
*/
class EventProxy {
public:
    using SMState = ara::com::e2e::SMState;
    using ProfileCheckStatus = ara::com::e2e::ProfileCheckStatus;
    EventProxy() {}
    virtual ~EventProxy() {}
    virtual void SetSubscriptionStateChangeHandler(vrtf::vcc::api::types::SubscriptionStateChangeHandler handler) = 0;
    virtual void SetDriver(std::shared_ptr<vrtf::vcc::driver::EventHandler>& drv) = 0;
    virtual std::shared_ptr<vrtf::vcc::driver::EventHandler> GetDriver() = 0;
    virtual void RemoveDriver() = 0;
    virtual void SetReceiveHandler(vrtf::vcc::api::types::EventReceiveHandler handler) = 0;
    virtual api::types::EventSubscriptionState GetSubscriptionState() const = 0;
    virtual void SetRosThreadPool(std::shared_ptr<utils::ThreadPool> &pool) = 0;
    virtual SMState GetSMState() const = 0;
    virtual const ara::com::e2e::Result GetResult() const = 0;
    virtual size_t GetFreeSampleCount() const = 0;
    virtual void OnDataAvailable(const api::types::EventMsg &eventMsg) = 0;
    virtual void AllocateSamplePtr() = 0;
    virtual void SetEventInfo(std::shared_ptr<vrtf::vcc::api::types::EventInfo>& eventInfo) = 0;
    virtual void SetLatencyAnalysisMode(const utils::LatencyAnalysisMode& mode) = 0;
    virtual utils::LatencyResult GetLatencyResult() = 0;
    vrtf::core::String GetShortName() const
    {
        return shortName_;
    }
    void SetShortName(vrtf::core::String const &shortname)
    {
        shortName_ = shortname;
    }
    vrtf::vcc::api::types::InstanceId GetInstanceId() const
    {
        return instanceId_;
    }
    void SetInstanceId(vrtf::vcc::api::types::InstanceId const &instanceId)
    {
        instanceId_ = instanceId;
    }
private:
    vrtf::core::String shortName_;
    vrtf::vcc::api::types::InstanceId instanceId_;
};

/*
  this handler is designed for receiver side, vcc has a container for it
*/
template<class T>
class EventProxyImpl : public EventProxy, public std::enable_shared_from_this<EventProxyImpl<T>> {
public:
    // use to according the relation of data and plogId
    struct MsgCacheInfo {
        std::shared_ptr<T> data;
        vrtf::vcc::api::types::SampleTimeInfo info;
    };

    EventProxyImpl(vrtf::vcc::api::types::EntityId const id, size_t size, const vrtf::vcc::api::types::DriverType type)
        : id_(id), maxSampleCount_(size), isTrigger_(false), driverType_(type)
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
        eventThreadNum++;
        std::string const threadName {"cm_event_" + std::to_string(eventThreadNum)};
        // max queue size: 1024
        pool_ = std::make_shared<vrtf::vcc::utils::ThreadPool>(1, 1024, threadName);
        readQueue_ = std::make_shared<vrtf::vcc::utils::LockFreeQueue<size_t>>(size);
        samples_.reserve(size);
        e2eResultContainer_.reserve(size);
    }
    virtual ~EventProxyImpl()
    {
        if (pool_ != nullptr) {
            if (pool_.use_count() == 1) {
                pool_->Stop();
            }
            pool_.reset();
        }
        if (driver_ != nullptr) {
            driver_->SetReceiveHandler(nullptr);
            driver_->SetSubscriptionStateChangeHandler(nullptr);
            driver_->UnsubscribeEvent();
            driver_.reset();
        }
    }

    virtual void AllocateSamplePtr() override
    {
        for (size_t i = 0; i < maxSampleCount_; ++i) {
            std::shared_ptr<T> ptr = std::make_shared<T>();
            if (!ptr) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->fatal() << "Allocate of sampleptr failed";
                ara::core::Abort("Allocate Buffer Failed! Check machine status");
            }
            vrtf::vcc::api::types::SampleTimeInfo info;
            MsgCacheInfo msgInfo {ptr, info};
            msgCache_.emplace_back(msgInfo);
            (void)readQueue_->Push(i);
        }
        e2eCache_.resize(maxSampleCount_);
    }
    virtual void SetEventInfo(std::shared_ptr<vrtf::vcc::api::types::EventInfo>& eventInfo) override
    {
        eventInfo_ = eventInfo;
    }
    virtual void SetSubscriptionStateChangeHandler(api::types::SubscriptionStateChangeHandler handler) override;
    virtual api::types::EventSubscriptionState GetSubscriptionState() const override
    {
        return status_;
    }
    virtual void SetDriver(std::shared_ptr<vrtf::vcc::driver::EventHandler>& drv) override
    {
        if (drv != nullptr) {
            driver_ = drv;
            if (drv->IsEnableDp()) {
                dpHandler_ = vrtf::vcc::utils::DpAdapterHandler::GetInstance();
                if (dpHandler_ == nullptr) {
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance_->error() << "DpAdapterHandler GetInstance failed.";
                }
            }
            drv->SetSubscriptionStateChangeHandler(
                std::bind(&EventProxyImpl::StatusChanged, this, std::placeholders::_1));
            std::shared_ptr<EventProxyImpl<T>> self = this->shared_from_this();
            static_cast<void>(drv->EnableEvent());
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Set Driver failed for event " << id_;
        }
    }
    virtual void SetRosThreadPool(std::shared_ptr<utils::ThreadPool> &pool) override
    {
        pool_ = pool;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->debug() << "[RTF] Event thread pool was set to vcc thread pool";
    }
    virtual std::shared_ptr<vrtf::vcc::driver::EventHandler> GetDriver() override
    {
        return driver_;
    }

    virtual void RemoveDriver() override
    {
        driver_ = nullptr;
    }
    virtual void SetReceiveHandler(vrtf::vcc::api::types::EventReceiveHandler handler) override
    {
        spinLock_.Lock();
        handler_ = handler;
        spinLock_.Unlock();
        if (driver_ != nullptr) {
            if (handler_ == nullptr) {
                std::unique_lock<std::mutex> lock(unReceiveMutex_);
                cond_.wait(lock, [this]() {
                    return (this->userTaskNum == 0) ? true : false;
                });
                driver_->SetReceiveHandler(nullptr);
                if (pool_ != nullptr) {
                    if (pool_.use_count() == 1) {
                        pool_->Stop();
                    }
                    pool_.reset();
                }
            } else {
                if (pool_ == nullptr) {
                    eventThreadNum++;
                    std::string const threadName {"cm_event_" + std::to_string(eventThreadNum)};
                    // the thread number: 1. max task queue size: 1024
                    pool_ = std::make_unique<vrtf::vcc::utils::ThreadPool>(1, 1024, threadName);
                }
                std::shared_ptr<EventProxyImpl<T>> self = this->shared_from_this();
                driver_->SetReceiveHandler([self] (const api::types::EventMsg &eventMsg) {
                    self->OnDataAvailable(eventMsg);
                });
            }
        }
    }

    ara::core::Result<size_t> GetNewSamples(std::function<void(vrtf::vcc::api::types::SamplePtr<T const>)> &cb,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        timespec getNewSampleTime = vrtf::vcc::utils::PlogInfo::GetPlogTimeStamp();
        if (firstRead_ == true) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->info()<< "Read event [" << GetInstanceId() << "." << GetShortName() << "] data.";
            firstRead_ = false;
        }
        size_t times = 0;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose() << "Add data in SamplePtr";
        if ((driver_->IsDpRawData()) && (!driver_->IsEnableDp())) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "If receive dp raw data, please set data plain transport qos.";
            return ara::core::Result<size_t>(times);
        }
        auto ret = AddSamptrData(maxNumberOfSamples);
        timespec serializeTime = vrtf::vcc::utils::PlogInfo::GetPlogTimeStamp();
        if (ret == vrtf::vcc::api::types::ReceiveType::FULLSLOT) {
            vrtf::core::ErrorCode errorcode(vrtf::vcc::api::types::ComErrc::kMaxSamplesExceeded);
            return ara::core::Result<size_t>(errorcode);
        } else {
            if (ret == vrtf::vcc::api::types::ReceiveType::EMPTY_CONTAINER) {
                DealWithEmptyReceivedContainer();
            }
            if (ret != vrtf::vcc::api::types::ReceiveType::OK) {
                return ara::core::Result<size_t>(times);
            }
        }
        for (times = 0; times < maxNumberOfSamples; ++times) {
            size_t tmp = 0;
            if (readQueue_->GetFront(tmp) != true) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->debug() << "Now data buffer is nullptr";
                return ara::core::Result<size_t>(times);
            }
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() << "Call user callback, number is " << tmp;
            vrtf::vcc::api::types::SamplePtr<T const> ptr;
            ptr.AddEventPosition(msgCache_.at(tmp).data, readQueue_, e2eCache_.at(tmp));
            WritePlogStamp(msgCache_.at(tmp).info, getNewSampleTime, serializeTime, ptr);
            delayAnalysis_.AddServerSendTime(msgCache_.at(tmp).info.GetServerSendTime());
            e2eResult_ = e2eCache_.at(tmp);
            e2eState_ = e2eCache_.at(tmp).GetSMState();
            cb(std::move(ptr));
            RecoverMsgQueue(tmp);
        }
        return ara::core::Result<size_t>(times);
    }

    void OnDataAvailable(api::types::EventMsg const &eventMsg) override
    {
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->debug() << "Vcc OnDataAvailable enter!";
        /* Fixme: optimize, if handler_ was not set, ignore the Enqueue oper */
        if (handler_ == nullptr) {
            return;
        }
        if (pool_ == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Uninitialize work poll in EventProxy!";
            return;
        }
        if (driver_ == nullptr) {
            return;
        }
        if (!driver_->IsEnableDp()) {
            std::lock_guard<std::mutex> guard(dataReadMutex_);
            std::shared_ptr<EventProxyImpl<T>> self = this->shared_from_this();
            std::function<void()> task = [self]() {
                self->ProcessUserCallback();
            };
            std::pair<uint64_t, uint64_t> msgSourceID = eventMsg.GetMsgSourceIdToVcc();
            utils::TaskSoureceId taskKey {GetTypeId(), msgSourceID.first, msgSourceID.second};
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->debug() << "Put App callback to worker pool[taskTypeId=" << taskKey.typeId << ", uuidHight="
                << taskKey.uuidHight << ", uuidLow=" << taskKey.uuidLow << "]";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            static_cast<void>(pool_->Enqueue(task, taskKey));
        } else {
            ProcessUserCallback();
        }
    }

    void StatusChanged(const vrtf::vcc::api::types::EventSubscriptionState& status)
    {
        status_ = status;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->debug() << "Vcc StatusChanged enter!";
        if (statusChangedHandler_) {
            statusChangedHandler_(status_);
        }
    }

    virtual SMState GetSMState() const override
    {
        return e2eState_;
    }

    const ara::com::e2e::Result GetResult() const override
    {
        return e2eResult_;
    }

    size_t GetFreeSampleCount() const override
    {
        if (readQueue_ != nullptr) {
            return readQueue_->GetSize();
        }
        return 0;
    }
    /**
     * @brief Set this proxy delay analysis mode
     * @details Set this proxy delay analysis mode
     * @param[in] mode delay analysis mode
     */
    void SetLatencyAnalysisMode(const utils::LatencyAnalysisMode& mode) override
    {
        delayMode_ = mode;
    }

    /**
     * @brief Get this proxy delay analysis result
     * @details Get this proxy delay analysis result and clear above delay info
     * @return LatencyResult include avg/max/min delay
     */
    utils::LatencyResult GetLatencyResult() override
    {
        return delayAnalysis_.GetLatencyResult();
    }
private:
    template<typename U = T>
    typename std::enable_if<!api::types::IsRecvBuffer<U>::value>::type RecoverMsgQueue(const size_t& pos) noexcept
    {
        (void)pos;
    }

    template<typename U = T>
    typename std::enable_if<api::types::IsRecvBuffer<U>::value>::type RecoverMsgQueue(const size_t& pos) noexcept
    {
        std::shared_ptr<api::types::RecvBuffer> recvBuffer = std::make_shared<api::types::RecvBuffer>();
        msgCache_.at(pos).data = recvBuffer;
    }

    template<typename U = T>
    typename std::enable_if<(!api::types::IsRecvBuffer<U>::value) && (vrtf::serialize::ros::IsRosMsg<U>::value)>::type
    ReturnLoan() noexcept
    {
        std::shared_ptr<vrtf::vcc::RawBufferHelper> rawBufferHelper = eventInfo_->GetRawBufferHelper();
        if ((rawBufferHelper == nullptr) || (!rawBufferHelper->IsBuffMsg(eventInfo_->GetDataTypeName()))) {
            for (auto iter = samples_.begin(); iter != samples_.end(); ++iter) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->debug() << "VCC ros msg return loan.";
                driver_->ReturnLoan((*iter).GetData());
            }
        }
        samples_.clear();
        e2eResultContainer_.clear();
    }

    template<typename U = T>
    typename std::enable_if<(!api::types::IsRecvBuffer<U>::value) && (!vrtf::serialize::ros::IsRosMsg<U>::value)>::type
    ReturnLoan() noexcept
    {
        if (!driver_->IsDpRawData()) {
            for (auto iter = samples_.begin(); iter != samples_.end(); ++iter) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->debug() << "VCC return loan.";
                driver_->ReturnLoan((*iter).GetData());
            }
        }
        samples_.clear();
        e2eResultContainer_.clear();
    }

    template<typename U = T>
    typename std::enable_if<api::types::IsRecvBuffer<U>::value>::type ReturnLoan() noexcept
    {
        samples_.clear();
        e2eResultContainer_.clear();
    }
    void ReadSampleDataFromDriver(uint32_t freeCount, size_t &maxNumberOfSamples)
    {
        if (freeCount < maxNumberOfSamples) {
            if (maxNumberOfSamples != std::numeric_limits<size_t>::max()) {
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance_->warn() << "User want to update count is : " << maxNumberOfSamples <<
                                        " really can use free count number is : " <<  GetFreeSampleCount();
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            }
            maxNumberOfSamples = freeCount;
        }
        if (!driver_->IsEnableDp()) {
            std::lock_guard<std::mutex> guard(dataReadMutex_);
            driver_->ReadEvent(samples_, e2eResultContainer_, maxNumberOfSamples);
            if (pool_.use_count() == 1) {
                pool_->ClearTask();
            }
        } else {
            driver_->ReadEvent(samples_, e2eResultContainer_, maxNumberOfSamples);
        }
    }

    template <typename SampleType>
    typename std::enable_if<vrtf::serialize::dds::is_dp_raw_data<SampleType>::value, bool>::type
    GetDeserializedValue(vrtf::vcc::driver::EventCacheContainer::iterator const &iter,
                         const std::uint64_t& e2eIter, const size_t& cycletimes) noexcept
    {
        bool result = false;
        std::size_t payloadSize = (*iter).GetSize();
        if (driver_->GetSerializeType() == vrtf::serialize::SerializeType::SHM) {
            vrtf::serialize::dds::Deserializer<T> desr((*iter).GetData(), payloadSize,
                eventInfo_->GetSerializeConfig());
            result = GetDpRawData(desr, iter, e2eIter, cycletimes);
            if (result == false) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->error() << "Failed to get dp raw data in cm msg.";
                (void)dpHandler_->MbufFree(const_cast<Mbuf *>((*iter).GetMbufPtr()));
            }
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() <<  "Wrong serialize type for dp raw data, deserialize error!";
        }
        return result;
    }

    template <typename SampleType>
    typename std::enable_if<
        (!vrtf::serialize::dds::is_dp_raw_data<SampleType>::value) && (!api::types::IsRecvBuffer<SampleType>::value) &&
        (!vrtf::serialize::ros::IsRosMsg<SampleType>::value), bool>::type
    GetDeserializedValue(vrtf::vcc::driver::EventCacheContainer::iterator const &iter,
                         const std::uint64_t& e2eIter, const size_t& cycletimes) noexcept
    {
        bool result = false;
        std::size_t payloadSize = (*iter).GetSize();
        if (driver_->GetSerializeType() == vrtf::serialize::SerializeType::SHM) {
            vrtf::serialize::dds::Deserializer<T> desr((*iter).GetData(), payloadSize,
                eventInfo_->GetSerializeConfig());
            result = GetCommonData(desr, iter, e2eIter, cycletimes);
        } else if (driver_->GetSerializeType() == vrtf::serialize::SerializeType::SOMEIP) {
            vrtf::serialize::someip::Deserializer<T> desr((*iter).GetData(), (*iter).GetSize(),
                eventInfo_->GetSerializeConfig());
            result = GetCommonData(desr, iter, e2eIter, cycletimes);
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() <<  "Wrong serialize type, deserialize error!";
        }
        return result;
    }

    template <typename SampleType>
    typename std::enable_if<
        (!vrtf::serialize::dds::is_dp_raw_data<SampleType>::value) && (!api::types::IsRecvBuffer<SampleType>::value) &&
        (vrtf::serialize::ros::IsRosMsg<SampleType>::value), bool>::type
    GetDeserializedValue(vrtf::vcc::driver::EventCacheContainer::iterator const &iter,
                         const std::uint64_t& e2eIter, const size_t& cycletimes) noexcept
    {
        std::shared_ptr<vrtf::vcc::RawBufferHelper> rawBufferHelper = eventInfo_->GetRawBufferHelper();
        bool result = true;
        std::size_t payloadSize = (*iter).GetSize();
        if (!driver_->IsEnableDp()) {
            if (driver_->GetSerializeType() == vrtf::serialize::SerializeType::SHM) {
                vrtf::serialize::dds::Deserializer<T> desr((*iter).GetData(), payloadSize,
                    eventInfo_->GetSerializeConfig());
                result = GetCommonData(desr, iter, e2eIter, cycletimes);
            } else if (driver_->GetSerializeType() == vrtf::serialize::SerializeType::SOMEIP) {
                vrtf::serialize::someip::Deserializer<T> desr((*iter).GetData(), (*iter).GetSize(),
                    eventInfo_->GetSerializeConfig());
                result = GetCommonData(desr, iter, e2eIter, cycletimes);
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->debug() << "Wrong serialize type, deserialize error!";
                result = false;
            }
        } else if ((driver_->IsEnableDp()) && (rawBufferHelper != nullptr)) {
            if (driver_->GetSerializeType() != vrtf::serialize::SerializeType::SHM) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->debug() << "Wrong serialize type, deserialize error!";
                result = false;
            }
            vrtf::serialize::dds::Deserializer<T> desr((*iter).GetData(), payloadSize,
                eventInfo_->GetSerializeConfig());
            std::string dataType = eventInfo_->GetDataTypeName();
            if ((result) && (!rawBufferHelper->IsBuffMsg(dataType))) {
                result = GetCommonData(desr, iter, e2eIter, cycletimes);
            } else if ((result) && (rawBufferHelper->IsBuffMsg(dataType))) {
                result = GetDpRawData(desr, iter, e2eIter, cycletimes);
                if (result == false) {
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance_->error() << "Failed to get dp raw data in ros msg.";
                    (void)dpHandler_->MbufFree(const_cast<Mbuf *>((*iter).GetMbufPtr()));
                }
            } else {
                // do nothing
            }
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "[proxy]Error config in ros msg.";
            result = false;
        }
        return result;
    }

    template <typename SampleType>
    typename std::enable_if<(!vrtf::serialize::dds::is_dp_raw_data<SampleType>::value) &&
        (api::types::IsRecvBuffer<SampleType>::value), bool>::type
    GetDeserializedValue(vrtf::vcc::driver::EventCacheContainer::iterator const &iter,
                         const std::uint64_t& e2eIter, const size_t& cycletimes) noexcept
    {
        bool result = false;
        if (driver_->GetSerializeType() == vrtf::serialize::SerializeType::SHM) {
            size_t validSlot = 0;
            if (readQueue_->At((uint32_t)cycletimes, validSlot) != true) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->error() << "May be the queue is empty or the pos exceed queue size.";
                return result;
            }
            api::types::RecvBuffer recvBuffer(iter->GetData(), iter->GetSize(), driver_);
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() << "DDS add receive data to Samptr deque, num is " << validSlot;
            *(msgCache_.at(validSlot).data.get()) = std::move(recvBuffer);
            msgCache_.at(validSlot).info = iter->GetSampleTimeInfo();
            e2eCache_.at(validSlot) = e2eResultContainer_.at(e2eIter);
            result = true;
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() <<  "RawBuffer only support DDS protocol";
        }
        return result;
    }

    /**
     * @brief If using E2E, the E2E Check will be called when received no data from driver and GetNewSample is called.
     *
     */
    void DealWithEmptyReceivedContainer()
    {
        auto e2eObject = eventInfo_->GetE2EObject();
        const auto isCombination = eventInfo_->IsCombindReceivdHdlAndGetSample();
        if (isCombination) {
            /* If using combination of ReceivedHandler and GetNewSample,
               the GetNewSample may be called multiple times based on the sending frequency in asynchronous mode.
               Therefore, E2E Check shouldn't be called in this situation */
            if ((handler_ == nullptr) && (e2eObject != nullptr)) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->debug() << "Using E2E with active mode in RTFCOM, and get no data from driver";
                e2eResult_ = vrtf::com::e2e::E2EXf_Protection::Check(*e2eObject, nullptr, 0);
                e2eState_ = e2eResult_.GetSMState();
            }
        } else {
            if (e2eObject != nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->debug() << "Using E2E in ARACOM, and get no data from driver";
                e2eResult_ = vrtf::com::e2e::E2EXf_Protection::Check(*e2eObject, nullptr, 0);
                e2eState_ = e2eResult_.GetSMState();
            }
        }
    }

    vrtf::vcc::api::types::ReceiveType UpdateContainerCheck()
    {
        if (samples_.size() != e2eResultContainer_.size()) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->error() << "The size of EventCacheContainer " << samples_.size()
                << " is not equal to ProfileCheckStatusContainer " << e2eResultContainer_.size();
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return vrtf::vcc::api::types::ReceiveType::E2EFAIL;
        }

        if (samples_.size() == 0) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() << "No data read from driver";
            return vrtf::vcc::api::types::ReceiveType::EMPTY_CONTAINER;
        }
        return vrtf::vcc::api::types::ReceiveType::OK;
    }

    vrtf::vcc::api::types::ReceiveType AddSamptrData(size_t& maxNumberOfSamples)
    {
        auto freeCount = GetFreeSampleCount();
        if (freeCount == 0) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "There is no sampleptr can be used, please release it first";
            return vrtf::vcc::api::types::ReceiveType::FULLSLOT;
        }
        ReadSampleDataFromDriver(freeCount, maxNumberOfSamples);
        const auto ContainerCheckRst = UpdateContainerCheck();
        if (ContainerCheckRst != vrtf::vcc::api::types::ReceiveType::OK) {
            return ContainerCheckRst;
        }
        std::uint64_t e2eContainerIter = 0;
        maxNumberOfSamples = 0;
        auto discardEventNum = 0;
        if (maxSampleCount_ < samples_.size()) {
            discardEventNum = samples_.size() - maxSampleCount_;
        }
        for (auto iter = samples_.begin(); iter != samples_.end(); ++iter, e2eContainerIter++) {
            if (discardEventNum != 0) {
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance_->warn() << "Receive sample more than maxSampleCount : "
                    << discardEventNum << " will ignore";
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
                discardEventNum--;
                continue;
            }
            auto value = GetDeserializedValue<T>(iter, e2eContainerIter, maxNumberOfSamples);
            if (!value) {
                // This log is used for test case monitoring. Before modifying the log, notify the test personnel.
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->warn() << "Deserialize failed, discard invalid data";
                continue;
            }
            ++maxNumberOfSamples;
        }
        ReturnLoan();
        return vrtf::vcc::api::types::ReceiveType::OK;
    }

    void ProcessUserCallback()
    {
        spinLock_.Lock();
        vrtf::vcc::api::types::EventReceiveHandler handlerTmp = handler_;
        if (handlerTmp == nullptr) {
            spinLock_.Unlock();
            return;
        }
        userTaskNum++;
        spinLock_.Unlock();
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose() << "Call user's callback";
        // trigger lastest user callback time
        clock_gettime(CLOCK_REALTIME, &triggerNotifyTime_);
        handlerTmp();
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose() << "Call user's callback end";
        spinLock_.Lock();
        userTaskNum--;
        spinLock_.Unlock();
        if (userTaskNum == 0) {
            if (!driver_->IsEnableDp()) {
                std::unique_lock<std::mutex> lock(unReceiveMutex_);
                cond_.notify_all();
            } else {
                cond_.notify_all();
            }
        }
    }

    template<class Deserializer>
    bool GetCommonData(Deserializer &deserializer, const vrtf::vcc::driver::EventCacheContainer::iterator& iter,
        const int& e2eIter, const size_t& cycletimes) noexcept
    {
        std::size_t payloadSize = (*iter).GetSize();
        std::size_t deserializeSize = deserializer.GetSize();
        if (payloadSize < deserializeSize) {
            return false;
        }

        size_t validSlot = 0;
        if (readQueue_->At((uint32_t)cycletimes, validSlot) != true) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "May be the queue is empty or the pos exceed queue size.";
            return false;
        }
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->debug() <<  "Add receive data to Samptr deque, num is " << validSlot;
        *(msgCache_.at(validSlot).data.get()) = deserializer.GetValue();
        msgCache_.at(validSlot).info = iter->GetSampleTimeInfo();
        e2eCache_.at(validSlot) = e2eResultContainer_.at(e2eIter);
        if (delayMode_ == utils::LatencyAnalysisMode::ENABLE) {
            timespec sendTime {0, 0};
        auto ret {(payloadSize >= (deserializeSize + vrtf::vcc::utils::TLV_TIME_TOTAL_SIZE)) &&
            (utils::TlvHelper::AnalysisTlvTime(iter->GetData() + deserializeSize, sendTime) ==
            utils::TlvAnalysisResult::SUCCESS)};
            if (ret) {
                msgCache_.at(validSlot).info.SetServerSendTime(sendTime);
            } else {
                msgCache_.at(validSlot).info.SetServerSendTime(utils::DEFAULT_TIME_STAMP);
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->debug() << "Server not enable delay mode";
            }
        } else {
            if (payloadSize > deserializeSize) {
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance_->debug() <<
                    "Inconsistent data type, wrong E2E Config or pub already enter delay analysis mode,"
                    <<" please check for " << id_;
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            }
        }
        return true;
    }

    template<class Deserializer>
    bool GetDpRawData(Deserializer &deserializer, const vrtf::vcc::driver::EventCacheContainer::iterator& iter,
                       const int& e2eIter, const size_t& cycletimes) noexcept
    {
        std::size_t payloadSize = (*iter).GetSize();
        std::size_t deserializeSize = deserializer.GetSize();
        if (payloadSize < deserializeSize) {
            return false;
        }
        size_t validSlot = 0;
        if (readQueue_->At((uint32_t)cycletimes, validSlot) != true) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "May be the queue is empty or the pos exceed queue size.";
            return false;
        }
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->debug() <<  "Add receive data to Samptr deque, num is " << validSlot;
        *(msgCache_.at(validSlot).data.get()) = deserializer.GetValue();
        msgCache_.at(validSlot).info = iter->GetSampleTimeInfo();
        e2eCache_.at(validSlot) = e2eResultContainer_.at(e2eIter);
        if (SetMbufToData(iter, validSlot) != true) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Failed to set mbuf to data.";
            return false;
        }
        if (delayMode_ == utils::LatencyAnalysisMode::ENABLE) {
            timespec sendTime {0, 0};
        auto ret {(payloadSize >= (deserializeSize + vrtf::vcc::utils::TLV_TIME_TOTAL_SIZE)) &&
            (utils::TlvHelper::AnalysisTlvTime(iter->GetData() + deserializeSize, sendTime) ==
            utils::TlvAnalysisResult::SUCCESS)};
            if (ret) {
                msgCache_.at(validSlot).info.SetServerSendTime(sendTime);
            } else {
                msgCache_.at(validSlot).info.SetServerSendTime(utils::DEFAULT_TIME_STAMP);
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->debug() << "Server not enable delay mode";
            }
        } else {
            if (payloadSize > deserializeSize) {
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance_->debug() <<
                    "Inconsistent data type or pub already enter delay analysis mode, please check for " << id_;
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            }
        }
        return true;
    }

    template <typename SampleType = T>
    typename std::enable_if<vrtf::serialize::dds::is_dp_raw_data<SampleType>::value, bool>::type
    SetMbufToData(const vrtf::vcc::driver::EventCacheContainer::iterator& iter, size_t validSlot) noexcept
    {
        Mbuf *mbuf = const_cast<Mbuf *>((*iter).GetMbufPtr());
        (*(msgCache_.at(validSlot).data.get())).SetMbufPtr(mbuf);
        return true;
    }

    template <typename SampleType = T>
    typename std::enable_if<vrtf::serialize::ros::IsRosMsg<SampleType>::value, bool>::type
    SetMbufToData(const vrtf::vcc::driver::EventCacheContainer::iterator& iter, size_t validSlot) noexcept
    {
        std::shared_ptr<vrtf::vcc::RawBufferHelper> rawBufferHelper = eventInfo_->GetRawBufferHelper();
        if (rawBufferHelper == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "RawBufferHelper is null.";
            return false;
        }
        std::string dataType = eventInfo_->GetDataTypeName();
        if (!rawBufferHelper->IsBuffMsg(dataType)) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Please set IsBuffMsg to true.";
            return false;
        }
        Mbuf *mbuf = const_cast<Mbuf *>((*iter).GetMbufPtr());
        if (!rawBufferHelper->SetMbufToMsg(dataType, mbuf, reinterpret_cast<void *>(
            msgCache_.at(validSlot).data.get()))) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "RawBufferHelper failed to set mbuf to msg.";
            return false;
        }
        return true;
    }

    void WritePlogStamp(const vrtf::vcc::api::types::SampleTimeInfo& sampleInfo, const timespec& takeSampleTime,
                        const timespec& serializeTimeStamp, vrtf::vcc::api::types::SamplePtr<T const>& ptr)
    {
        if (eventInfo_->GetIsField()) {
            return;
        }
        auto info {utils::PlogInfo::CreatePlogInfo(utils::CM_RECV)};
        if (info == nullptr) {
            return;
        }
        timespec triggerStamp = vrtf::vcc::utils::PlogInfo::GetPlogTimeStamp();
        info->RelatedModule(sampleInfo.GetPlogId(), sampleInfo.GetReceiveModule());
        ptr.SetSampleId(info->GetMsgUid(utils::PlogDriverType::COMMON));
        info->WriteTimeStamp(utils::PlogClientTimeStampNode::RECVIVE_NOTIFY, triggerNotifyTime_);
        info->WriteTimeStamp(utils::PlogClientTimeStampNode::USER_TAKE, takeSampleTime);
        info->WriteTimeStamp(utils::PlogClientTimeStampNode::TAKE_FROM_DRIVER, sampleInfo.GetTakeTime());
        info->WriteTimeStamp(utils::PlogClientTimeStampNode::DESERIALIZE_DATA, serializeTimeStamp);
        info->WriteTimeStamp(utils::PlogClientTimeStampNode::NOTIFY_USER, triggerStamp);
        info->SendPlogStamp(utils::PlogDriverType::COMMON);
    }
    std::uint16_t GetTypeId() const
    {
        const std::uint16_t typeId = (static_cast<uint16_t>(driverType_) <<
            static_cast<std::uint8_t>(api::types::MOVEBIT::MOVE8BIT)) +
            static_cast<std::uint8_t>(api::types::EntityType::EVENT);
        return typeId;
    }
    bool firstRead_ = true;
    size_t writePos = 0;
    vrtf::vcc::driver::EventCacheContainer samples_;
    vrtf::vcc::driver::E2EResultContainer e2eResultContainer_;
    std::shared_ptr<vrtf::vcc::utils::LockFreeQueue<size_t>> readQueue_;
    std::shared_ptr<vrtf::vcc::driver::EventHandler> driver_;
    vrtf::vcc::api::types::EventReceiveHandler handler_;
    vrtf::vcc::api::types::SubscriptionStateChangeHandler statusChangedHandler_ = nullptr;
    std::shared_ptr<utils::ThreadPool> pool_; /* enabled in trigger mode. */
    vrtf::vcc::utils::RtfSpinLock spinLock_;
    std::mutex dataReadMutex_;
    std::mutex queueMutex_;
    vrtf::vcc::api::types::EntityId const id_;
    size_t maxSampleCount_;
    bool isTrigger_;  /* if true, will call handler as soon as event arrived */
    vrtf::vcc::api::types::SampleContainer<MsgCacheInfo> msgCache_;
    vrtf::vcc::api::types::E2EResultCache e2eCache_;
    bool isReceiveHandlerSetted_ = false;
    vrtf::vcc::api::types::EventSubscriptionState status_
        = vrtf::vcc::api::types::EventSubscriptionState::kNotSubscribed;
    std::condition_variable cond_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    size_t userTaskNum = 0;
    std::mutex unReceiveMutex_;
    std::shared_ptr<vrtf::vcc::api::types::EventInfo> eventInfo_;
    std::shared_ptr<vrtf::vcc::utils::DpAdapterHandler> dpHandler_;
    timespec triggerNotifyTime_ {0, 0};
    utils::LatencyAnalysisMode delayMode_ = utils::LatencyAnalysisMode::DISABLE;
    utils::LatencyAnalysis delayAnalysis_;
    ara::com::e2e::SMState e2eState_ = SMState::kStateMDisabled;
    ara::com::e2e::Result e2eResult_ = ara::com::e2e::Result(ProfileCheckStatus::kCheckDisabled,
                                                             SMState::kStateMDisabled);
    vrtf::vcc::api::types::DriverType driverType_ = vrtf::vcc::api::types::DriverType::INVALIDTYPE;
};

template<class T>
void EventProxyImpl<T>::SetSubscriptionStateChangeHandler(
    vrtf::vcc::api::types::SubscriptionStateChangeHandler handler)
{
    statusChangedHandler_ = handler;
}
}
}


#endif /* VRTF_VCC_EVENTHANDLER_H */
