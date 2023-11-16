
#ifndef ADSF_DATARECEIVEBASE_V1_H
#define ADSF_DATARECEIVEBASE_V1_H

#include <shared_mutex>
#include <unistd.h>
#include <memory>
#include <cstdint>

#include "core/thread_safe_stack.h"
#include "ara/com/internal/proxy/event_adapter.h"
#include "ara/com/internal/proxy/proxy_adapter.h"

#include "core/status.h"
#include "core/logger.h"

namespace Adsfi {

template <typename Proxy, typename HandleType, typename DataType>
class DataReceiveBaseV1 {
  public:
    explicit DataReceiveBaseV1(const uint32_t instanceIdxIn, const int64_t time2live = 0, const size_t capLimit = 5U)
      : instanceIdx_(instanceIdxIn), dataContainer_(time2live, capLimit)
    {
    }
    virtual void StartFindService()
    {
        std::string insIdxStr = std::to_string(instanceIdx_);
        ara::core::StringView id(insIdxStr.c_str());
        handle_ = Proxy::StartFindService(
            [this](const ara::com::ServiceHandleContainer<HandleType> handles,
                   const ara::com::FindServiceHandle handler) {
                (void)handler;
                if (!IsHandlesAndProxyValid(handles)) {
                    return;
                }
                for (auto& it : handles) {
                    std::string idStr(it.GetInstanceId().ToString().data());
                    HAF_LOG_INFO << "Got InstanceId : " << idStr;
                    if (std::to_string(instanceIdx_) == idStr) {
                        HAF_LOG_INFO << "Got right instance id!";
                        auto resultToken = Proxy::Preconstruct(it);
                        if (resultToken.HasValue()) {
                            proxyPtr_ = std::make_unique<Proxy>(std::move(resultToken).Value());
                            RegisterHandle();
                            break;
                        }
                        HAF_LOG_WARN << "Proxy preconstruct Failed!";
                    }
                }
            },
            ara::com::InstanceIdentifier(id));
        return;
    }
    virtual ~DataReceiveBaseV1()
    {
        stopFlag_ = true;
        receiveCv_.notify_all();
        Proxy::StopFindService(handle_);
        HAF_LOG_INFO << "Data receive baes destructor finished. idx:" << instanceIdx_;
    }
    void Stop()
    {
        stopFlag_ = true;
        receiveCv_.notify_all();
    }

    HafStatus GetOneData(std::shared_ptr<DataType>& data, const uint32_t blockTimeout = UINT32_MAX)
    {
        std::unique_lock<std::mutex> recvLk(receiveMtx_);
        if (receiveCv_.wait_for(recvLk, std::chrono::milliseconds(blockTimeout)) == std::cv_status::timeout) {
            HAF_LOG_DEBUG << "No new data arrived since this api called. timeout(ms): " << blockTimeout;
            return HAF_NOT_READY;
        }
        if (stopFlag_) {
            HAF_LOG_INFO << "Data receive got stopFlag";
            return HAF_PROGRAM_STOPED;
        }
        if (dataContainer_.Empty()) {
            return HAF_NOT_READY;
        }
        auto dataPtr = dataContainer_.GetOneData();
        recvLk.unlock();
        if (dataPtr == nullptr) {
            return HAF_ERROR;
        }
        data = *dataPtr;
        return HAF_SUCCESS;
    }

    std::shared_ptr<DataType> GetOneDataBlocking(const uint32_t blockTimeout = UINT32_MAX)
    {
        std::unique_lock<std::mutex> recvLk(receiveMtx_);
        if (receiveCv_.wait_for(recvLk, std::chrono::milliseconds(blockTimeout)) == std::cv_status::timeout) {
            HAF_LOG_DEBUG << "No new data arrived since this api called. timeout(ms): " << blockTimeout;
            return nullptr;
        }
        if (stopFlag_) {
            HAF_LOG_INFO << "Data receive got stopFlag";
            return nullptr;
        }
        auto dataPtr = dataContainer_.GetOneData();
        recvLk.unlock();
        if (dataPtr == nullptr) {
            return nullptr;
        }
        return *dataPtr;
    }

    std::shared_ptr<DataType> GetOneData(const uint32_t freshDataTime = UINT32_MAX)
    {
        if (stopFlag_) {
            HAF_LOG_INFO << "Data receive got stopFlag signal";
            return nullptr;
        }
        auto dataPtr = dataContainer_.GetOneData(freshDataTime);
        if (dataPtr == nullptr) {
            return nullptr;
        }
        return *dataPtr;
    }

    HafStatus GetLastOne(std::shared_ptr<DataType>& data, const bool erase = false)
    {
        if (stopFlag_) {
            HAF_LOG_INFO << "Data receive got stopFlag signal";
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
        std::unique_lock<std::mutex> recvLk(receiveMtx_);
        receiveCv_.wait_for(recvLk, std::chrono::milliseconds(blockTimeout));
        if (stopFlag_) {
            HAF_LOG_INFO << "Data receive got stopFlag";
            return HAF_PROGRAM_STOPED;
        }
        if (dataContainer_.Empty()) {
            return HAF_NOT_READY;
        }
        auto dataPtr = dataContainer_.GetLastOne(erase);
        recvLk.unlock();
        if (dataPtr == nullptr) {
            return HAF_ERROR;
        }
        data = *dataPtr;
        return HAF_SUCCESS;
    }

    std::vector<std::shared_ptr<DataType>> GetNdata(const size_t n)
    {
        if (stopFlag_) {
            HAF_LOG_INFO << "Data receive got stopFlag";
            return {};
        }

        auto result = dataContainer_.GetNdata(n);

        std::vector<std::shared_ptr<DataType>> output;
        output.reserve(result.size());

        for (auto& it : result) {
            if (it != nullptr) {
                output.push_back(*it);
            }
        }
        return output;
    }

    void Clear()
    {
        dataContainer_.Clear();
    }
    bool Empty()
    {
        return dataContainer_.Empty();
    }
    virtual void RegisterHandle() = 0;
    void Notify()
    {
        std::lock_guard<std::mutex> recvLk(receiveMtx_);
        receiveCv_.notify_all();
    }
    bool IsStop() const
    {
        return stopFlag_;
    }
    uint32_t GetInstanceId() const
    {
        return instanceIdx_;
    }

  protected:
    size_t GetRecvQSize() const
    {
        return recvQSize_;
    }
    size_t GetMaxNumOfSample() const
    {
        return maxNumberOfSamples_;
    }

    void PushToContainer(DataType&& value)
    {
        std::shared_ptr<DataType> p = std::make_shared<DataType>(std::forward<DataType>(value));
        dataContainer_.Push(std::move(p));
        receiveCv_.notify_all();
    }

    bool IsHandlesAndProxyValid(const ara::com::ServiceHandleContainer<HandleType>& handles) const
    {
        if (handles.empty()) {
            return false;
        }
        if (proxyPtr_ != nullptr) {
            return false;
        }
        return true;
    }

  protected:
    std::unique_ptr<Proxy> proxyPtr_{ nullptr };

  private:
    std::mutex receiveMtx_;
    std::condition_variable receiveCv_;
    uint32_t instanceIdx_{};
    size_t recvQSize_{ 15U };
    size_t maxNumberOfSamples_{ 15U };
    ThreadSafeStack<std::shared_ptr<DataType>> dataContainer_;
    ara::com::FindServiceHandle handle_{};
    bool stopFlag_{ false };
};
}  // namespace Adsfi
#endif
