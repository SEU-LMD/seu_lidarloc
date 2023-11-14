/* *
 * FUNCTION: Define DataSend Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *   */
#ifndef ADSF_DATASENDBASE_H
#define ADSF_DATASENDBASE_H

#include <map>
#include <functional>
#include <shared_mutex>
#include <mutex>
#include <condition_variable>
#ifndef MDC_COMPILER_AOS_SEA_LLVM
#include "core/core.h"
#endif
#include "core/status.h"
#include "core/logger.h"
#include "core/thread_safe_stack.h"
#include "ara/com/internal/skeleton/event_adapter.h"
#include "ara/com/internal/skeleton/field_adapter.h"
#include "ara/com/internal/skeleton/skeleton_adapter.h"

namespace Adsfi {
template <typename Skeleton, typename DataType>
class DataSendBase {
public:
    explicit DataSendBase(const uint32_t idx) : instanceIdx_(idx)
    {
        std::string idStr = std::to_string(instanceIdx_);
        ara::core::StringView id(idStr.c_str());
        using ara::com::InstanceIdentifier;
        using ara::com::MethodCallProcessingMode;
        auto resultToken = Skeleton::Preconstruct(InstanceIdentifier(id), MethodCallProcessingMode::kPoll);
        if (resultToken.HasValue()) {
            dataSkeleton_ = std::make_unique<Skeleton>(std::move(resultToken).Value());
            dataSkeleton_->OfferService();
        } else {
            HAF_LOG_ERROR << "Skeleton preconstruct Failed!";
        }
    }
    virtual ~DataSendBase()
    {
        stopFlag_ = true;
        sendCv_.notify_all();
        HAF_LOG_INFO << "Data send baes destructor finished. idx:" << instanceIdx_;
    }
    void Stop()
    {
        stopFlag_ = true;
    }
    bool IsStop() const
    {
        return stopFlag_;
    }
    bool Empty() const
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
    /**
     * @brief 将需要发送的数据放到缓冲区中。并通知发送线程，该将数据交由AP序列化、发送出去。
     *
     * @param data
     */
    HafStatus SendOneData(std::shared_ptr<DataType>& data)
    {
        if (data == nullptr) {
            HAF_LOG_ERROR << "The data ptr for sending is nullptr";
            return HAF_ERROR;
        }
        if (dataSkeleton_ == nullptr) {
            HAF_LOG_ERROR << "DataSkeleton is nullptr";
            return HAF_ERROR;
        }
        std::lock_guard<std::mutex> lk(sendMtx_);
        dataContainer_.Push(data);
        sendCv_.notify_one();
        return HAF_SUCCESS;
    }
    /**
     * @brief 清空消息容器中的历史数据
     */
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
    bool stopFlag_{false};
};
}  // namespace Adsfi
#endif
