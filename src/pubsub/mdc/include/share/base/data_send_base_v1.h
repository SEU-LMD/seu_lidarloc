
#ifndef ADSF_DATASENDBASE_V1_H
#define ADSF_DATASENDBASE_V1_H

#include <map>
#include <functional>
#include <shared_mutex>
#include <mutex>
#include <condition_variable>

#include "core/status.h"
#include "core/logger.h"
#include "core/thread_safe_stack.h"
#include "ara/com/internal/skeleton/event_adapter.h"
#include "ara/com/internal/skeleton/field_adapter.h"
#include "ara/com/internal/skeleton/skeleton_adapter.h"

namespace Adsfi {

template <typename Skeleton, typename DataType>
class DataSendBaseV1 {
public:
    explicit DataSendBaseV1(const uint32_t idx) : instanceIdx_(idx)
    {
        std::string idStr = std::to_string(instanceIdx_);
        ara::core::StringView id(idStr.c_str());
        using ara::com::InstanceIdentifier;
        using ara::com::MethodCallProcessingMode;
        auto resultToken = Skeleton::Preconstruct(InstanceIdentifier(id), MethodCallProcessingMode::kPoll);
        if (resultToken.HasValue()) {
            dataSkeleton_ = std::make_unique<Skeleton>(std::move(resultToken).Value());
            dataSkeleton_->OfferService();
        }
        else {
            HAF_LOG_ERROR << "Skeleton preconstruct Failed!";
        }
    }
    virtual ~DataSendBaseV1()
    {
        stopFlag_ = true;
        sendCv_.notify_all();
        HAF_LOG_INFO << "Data send base destructor finished. idx:" << instanceIdx_;
    }
    void Stop()
    {
        stopFlag_ = true;
        sendCv_.notify_all();
    }
    bool IsStop() const
    {
        return stopFlag_;
    }
    bool Empty()
    {
        return dataContainer_.Empty();
    }
    uint32_t GetInstanceIdx() const
    {
        return instanceIdx_;
    }
    void Notify()
    {
        std::lock_guard<std::mutex> lk(sendMtx_);
        sendCv_.notify_one();
    }

    HafStatus SendOneData(std::shared_ptr<DataType>& data)
    {
        if (data == nullptr) {
            HAF_LOG_ERROR << "The data ptr for sending is nullptr";
            return HAF_ERROR;
        }
        if (dataSkeleton_ == nullptr) {
            HAF_LOG_ERROR << "DataSkeleton is nullptr, please check network_binding.json. idx:" << instanceIdx_;
            return HAF_ERROR;
        }
        std::lock_guard<std::mutex> lk(sendMtx_);
        dataContainer_.Push(data);
        sendCv_.notify_one();
        return HAF_SUCCESS;
    }

    HafStatus GetOneData(std::shared_ptr<DataType>& data, const uint32_t blockTimeout = UINT32_MAX)
    {
        std::unique_lock<std::mutex> sendLk(sendMtx_);
        if (sendCv_.wait_for(sendLk, std::chrono::milliseconds(blockTimeout)) == std::cv_status::timeout) {
            HAF_LOG_DEBUG << "No new data arrived since this api called. timeout(ms): " << blockTimeout;
            return HAF_NOT_READY;
        }
        if (stopFlag_) {
            HAF_LOG_INFO << "Data send got stopFlag";
            return HAF_PROGRAM_STOPED;
        }
        if (dataContainer_.Empty()) {
            return HAF_NOT_READY;
        }
        auto dataPtr = dataContainer_.GetOneData();
        sendLk.unlock();
        if (dataPtr == nullptr) {
            return HAF_ERROR;
        }
        data = *dataPtr;
        return HAF_SUCCESS;
    }

    std::shared_ptr<DataType> GetOneDataBlocking(const uint32_t blockTimeout = UINT32_MAX)
    {
        std::unique_lock<std::mutex> sendLk(sendMtx_);
        if (sendCv_.wait_for(sendLk, std::chrono::milliseconds(blockTimeout)) == std::cv_status::timeout) {
            HAF_LOG_DEBUG << "No new data arrived since this api called. timeout(ms): " << blockTimeout;
            return nullptr;
        }
        if (stopFlag_) {
            HAF_LOG_INFO << "Data send got stopFlag";
            return nullptr;
        }
        auto dataPtr = dataContainer_.GetOneData();
        sendLk.unlock();
        if (dataPtr == nullptr) {
            return nullptr;
        }
        return *dataPtr;
    }

    HafStatus GetLastOne(std::shared_ptr<DataType>& data, const bool erase = false)
    {
        if (stopFlag_) {
            HAF_LOG_INFO << "Data send got stopFlag signal";
            return HAF_PROGRAM_STOPED;
        }
        if (dataContainer_.Empty()) {
            return HAF_NOT_READY;
        }
        data = *dataContainer_.GetLastOne(erase);
        return HAF_SUCCESS;
    }

    HafStatus GetLastOneBlocking(std::shared_ptr<DataType>& data, const bool erase = false,
                                 const uint32_t blockTimeout = UINT32_MAX)
    {
        std::unique_lock<std::mutex> sendLk(sendMtx_);
        sendCv_.wait_for(sendLk, std::chrono::milliseconds(blockTimeout));
        if (stopFlag_) {
            HAF_LOG_INFO << "Data send got stopFlag";
            return HAF_PROGRAM_STOPED;
        }
        if (dataContainer_.Empty()) {
            return HAF_NOT_READY;
        }
        auto dataPtr = dataContainer_.GetLastOne(erase);
        sendLk.unlock();
        if (dataPtr == nullptr) {
            return HAF_ERROR;
        }
        data = *dataPtr;
        return HAF_SUCCESS;
    }

    void Clear()
    {
        std::lock_guard<std::mutex> lk(sendMtx_);
        dataContainer_.Clear();
    }
    virtual void SendingData() = 0;

protected:
    std::unique_ptr<Skeleton> dataSkeleton_;
    ThreadSafeStack<std::shared_ptr<DataType>> dataContainer_;
    std::mutex sendMtx_;
    std::condition_variable sendCv_;

private:
    uint32_t instanceIdx_;
    bool stopFlag_{ false };
};

}  // namespace Adsfi
#endif
