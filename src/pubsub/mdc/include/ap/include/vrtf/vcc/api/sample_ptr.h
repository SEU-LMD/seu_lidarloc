/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: The definitions of SamplePtr
 * Create: 2019-06-17
 */
#ifndef VRTF_VCC_API_INTERNAL_SAMPLEPTR_H
#define VRTF_VCC_API_INTERNAL_SAMPLEPTR_H
#include <queue>
#include "vrtf/vcc/utils/lock_free_queue.h"
#include "ara/hwcommon/log/log.h"
#include "ara/com/e2e/E2EXf/E2EXf_CM.h"
#include <iostream>
namespace vrtf {
namespace vcc {
namespace api {
namespace types {
static std::mutex readQueueMutex;
template <typename T>
class SamplePtr {
    using ProfileCheckStatus = ara::com::e2e::ProfileCheckStatus;
    using Result = ara::com::e2e::Result;
    using SMState = ara::com::e2e::SMState;
public:
    explicit constexpr SamplePtr() noexcept
        : dataPtr_(nullptr), e2eResult_(Result(ProfileCheckStatus::kCheckDisabled, SMState::kStateMDisabled))
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
    }
    // This is not AutoSAR interface, applications is not allowed use!!!!
    explicit constexpr SamplePtr(ProfileCheckStatus e2eCheckStatus) noexcept
        : dataPtr_(nullptr), e2eResult_(Result(e2eCheckStatus, SMState::kStateMDisabled))
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
    }
    // This is not AutoSAR interface, applications is not allowed use!!!!
    explicit constexpr SamplePtr(std::shared_ptr<T> dataPtr, ProfileCheckStatus e2eCheckStatus) noexcept
        : dataPtr_(dataPtr), e2eResult_(Result(e2eCheckStatus, SMState::kStateMDisabled))
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
    }

    SamplePtr(const SamplePtr<T>& samplePtr) = delete;
    SamplePtr& operator=(const SamplePtr<T>& samplePtr) = delete;

    // Move constructor
    SamplePtr(SamplePtr<T> && samplePtr) noexcept
        : dataPtr_(std::move(samplePtr.dataPtr_)), readQueue_(std::move(samplePtr.readQueue_)),
          e2eResult_(std::move(samplePtr.e2eResult_)),
          samplePtrNum_(std::move(samplePtr.samplePtrNum_)), uid_(std::move(samplePtr.uid_))
    {
    }

    ~SamplePtr()
    {
        if (readQueue_ != nullptr) {
            if (readQueue_->Push(samplePtrNum_) != true) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->error() << "May be queue is full.";
            }
        }
    }
    // to support pre definition of SamplePtr
    // This is not AutoSAR interface, applications is not allowed use!!!!
    SamplePtr& operator=(const std::shared_ptr<T> sharePtr) noexcept
    {
        dataPtr_ = sharePtr;
        return *this;
    }

    // This is not AutoSAR interface, applications is not allowed use!!!!
    T* get() const noexcept
    {
        return dataPtr_.get();
    }

    // Move assignment operator
    SamplePtr& operator=(SamplePtr<T> && samplePtr) noexcept
    {
        if (&samplePtr != this) {
            dataPtr_ = std::move(samplePtr.dataPtr_);
            e2eResult_ = std::move(samplePtr.e2eResult_);
            readQueue_ = std::move(samplePtr.readQueue_);
            samplePtrNum_ = std::move(samplePtr.samplePtrNum_);
            uid_ = std::move(samplePtr.uid_);
        }
        return *this;
    }

    // Dereferences the stored pointer
    T& operator*() const noexcept
    {
        return dataPtr_.operator*();
    }
    T* operator->() const noexcept
    {
        return dataPtr_.operator->();
    }

    // Checks if the stored pointer is null
    explicit operator bool() const noexcept
    {
        if (dataPtr_ == nullptr) {
            return false;
        } else {
            return true;
        }
    }

    // Swaps the managed object
    void Swap(SamplePtr<T>& samplePtr) noexcept
    {
        samplePtr.dataPtr_.swap(dataPtr_);
        auto tmp = e2eResult_;
        e2eResult_ = samplePtr.e2eResult_;
        samplePtr.e2eResult_ = tmp;
    }

    // Replaces the managed object
    void Reset(T* dataPtr)
    {
        dataPtr_ = std::shared_ptr<T>(dataPtr);
    }

    // Returns the stored object
    T* Get() const noexcept
    {
        return dataPtr_.get();
    }

    // Returns the number of SamplePtr objects referring the same managed object
    long UseCount() const noexcept
    {
        return dataPtr_.use_count();
    }

    // Returns the E2E protection check result
    ProfileCheckStatus GetProfileCheckStatus() const noexcept
    {
        return e2eResult_.GetProfileCheckStatus();
    }

    // This is not AutoSAR interface, applications is not allowed use!!!!
    /**
     * @brief Use sampleptr to save data
     * @details use sampleptr to save data and save data queue position
     *
     * @param ptr   save the pointer to data
     * @param queue the position of queue
     * @param e2eStatus the check result of the data
     */
    void AddEventPosition(std::shared_ptr<T> ptr, std::shared_ptr<vrtf::vcc::utils::LockFreeQueue<size_t>> queue,
                          Result e2eResult)
    {
        dataPtr_ = ptr;
        readQueue_ = queue;
        e2eResult_ = e2eResult;
        size_t pos = 0;
        if (readQueue_->Pop(pos) != true) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "May be queue is empty.";
        }
        samplePtrNum_ = pos;
    }

    /**
     * @brief Add sample pointer this msg's uid
     * @details when receive msg, this msg will store one generate generated by plog
     *
     * @param uid msg uid for user to check delay info in plog
     */
    void SetSampleId(const std::uint64_t& uid)
    {
        uid_ = uid;
    }

    /**
     * @brief Get msg's uid
     * @details Get msg uid according to this msg
     *
     * @return std::uint64_t msg uid for user to check delay info in plog
     */
    std::uint64_t GetSampleId() const
    {
        return uid_;
    }

    /**
     * @brief Get msg's e2e check result. This is not the standard API defined by AutoSAR
     *
     * @return Result      msg's e2e check result
     */
    Result GetE2EResult() const noexcept
    {
        return e2eResult_;
    }
private:
    std::shared_ptr<T> dataPtr_;
    std::shared_ptr<vrtf::vcc::utils::LockFreeQueue<size_t>> readQueue_ = nullptr;
    Result e2eResult_;
    size_t samplePtrNum_ = 0;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    std::uint64_t uid_ = UINT64_MAX;
};
}
}
}
}
#endif
