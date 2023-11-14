/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_SKELETON_EVENT_ADAPTER_H_
#define ARA_COM_SKELETON_EVENT_ADAPTER_H_
#include "ara/com/internal/skeleton/skeleton_adapter.h"
#include "vrtf/vcc/internal/traffic_crtl_policy.h"
namespace ara {
namespace com {
namespace internal {
namespace skeleton {
namespace event {
namespace impl {
class EventAdapterImpl {
public:
    EventAdapterImpl(void)
    {
    }

    EventAdapterImpl(const std::shared_ptr<vrtf::vcc::Skeleton> &skeleton, EntityId entityId)
        : skeleton_(skeleton), entityId_(entityId)
    {
    }

    virtual ~EventAdapterImpl(void)
    {
    }

    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetEntityId(void) const
    {
        return entityId_;
    }

protected:
    std::shared_ptr<vrtf::vcc::Skeleton> skeleton_;
    EntityId entityId_ = UNDEFINED_ENTITYID;

    template<class SampleType>
    ara::com::SampleAllocateePtr<SampleType> Allocate(void)
    {
        return skeleton_->Allocate<SampleType>();
    }

    template<class SampleType>
    typename std::enable_if<IsRawMemory<SampleType>::value, RawMemory>::type Allocate(const size_t &size)
    {
        return skeleton_->AllocateRawBuffer(size, entityId_);
    }

    template<class SampleType>
    typename std::enable_if<IsRawMemory<SampleType>::value>::type Send(const SampleType &&data)
    {
        RawMemory rawBuffer = data;
        std::shared_ptr<SampleInfoImpl> sampleInfo = CreateSampleInfo();
        skeleton_->PubRawBuffer(std::move(rawBuffer), entityId_, sampleInfo);
    }

    template<class SampleType>
    typename std::enable_if<!IsRawMemory<SampleType>::value>::type Send(const SampleType &data)
    {
        std::shared_ptr<SampleInfoImpl> sampleInfo = CreateSampleInfo();
        skeleton_->Send<SampleType>(data, entityId_, sampleInfo);
    }

    template<class SampleType>
    typename std::enable_if<!IsRawMemory<SampleType>::value>::type Send(std::unique_ptr<SampleType> data)
    {
        if (data == nullptr) {
            return;
        }
        std::shared_ptr<SampleInfoImpl> sampleInfo = CreateSampleInfo();
        skeleton_->Send<SampleType>(std::move(data), entityId_, sampleInfo);
    }

    bool SetTrafficCtrl(const std::shared_ptr<rtf::TrafficCtrlPolicy> &policy)
    {
        return skeleton_->SetEventTrafficCtrl(policy, entityId_);
    }
private:
    std::shared_ptr<SampleInfoImpl> CreateSampleInfo()
    {
        std::shared_ptr<SampleInfoImpl> sampleInfo = std::make_shared<SampleInfoImpl>();
        sampleInfo->plogInfo_ = PlogInfo::CreatePlogInfo(vrtf::vcc::utils::CM_SEND);
        clock_gettime(CLOCK_REALTIME, &sampleInfo->sendTime_);
        if (sampleInfo->plogInfo_ != nullptr) {
            sampleInfo->plogInfo_->WriteTimeStamp(PlogServerTimeStampNode::USER_SEND_EVENT, PlogDriverType::COMMON);
        }
        return sampleInfo;
    }
};
}

template<class SampleType>
class EventAdapter : public impl::EventAdapterImpl {
public:
    EventAdapter(void)
    {
    }

    EventAdapter(const std::shared_ptr<vrtf::vcc::Skeleton> &skeleton, EntityId entityId)
        : EventAdapterImpl(skeleton, entityId)
    {
    }
    ~EventAdapter() = default;
    /**
     * @brief allocate event data type ptr
     * @details allocate event data type ptr
     *
     * @return Return sample allocate ptr of event type
     */
    ara::com::SampleAllocateePtr<SampleType> Allocate(void)
    {
        return EventAdapterImpl::Allocate<SampleType>();
    }

    /**
     * @brief allocate shm memory for rawmemory type
     * @details allocate shm memory for rawmemory type
     *
     * @param size size of shm memory
     * @return Return raw memory data type
     */
    SampleType Allocate(const size_t size)
    {
        return EventAdapterImpl::Allocate<SampleType>(size);
    }

    /**
     * @brief Send data created by user
     * @details Send data created by user
     *
     * @param data dataType except rawBuffer
     */
    void Send(const SampleType &data)
    {
        EventAdapterImpl::Send<SampleType>(data);
    }

    /**
     * @brief Send rawmemory data type
     * @details Send rawmemory data type
     *
     * @param data raw buffer
     */
    void Send(const SampleType &&data)
    {
        EventAdapterImpl::Send<SampleType>(std::move(data));
    }

    void Send(ara::com::SampleAllocateePtr<SampleType> data)
    {
        EventAdapterImpl::Send<SampleType>(std::move(data));
    }

    bool SetTrafficCtrl(const std::shared_ptr<rtf::TrafficCtrlPolicy> &policy)
    {
        return EventAdapterImpl::SetTrafficCtrl(policy);
    }
};
}
}
}
}
}

#endif
