/**
 * FUNCTION: Define DataReceive Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 **/
#ifndef ADSF_DATARECEIVEBASE_H
#define ADSF_DATARECEIVEBASE_H

#include <shared_mutex>
#include <unistd.h>
#include <memory>
#include <cstdint>

#ifndef MDC_COMPILER_AOS_SEA_LLVM
#include "core/core.h"
#endif

#include "core/thread_safe_stack.h"
#include "ara/com/internal/proxy/event_adapter.h"
#include "ara/com/internal/proxy/proxy_adapter.h"

#include "core/status.h"
#include "core/logger.h"
namespace Adsfi {
template <typename Proxy, typename HandleType, typename DataType>
class DataReceiveBase {
public:
    explicit DataReceiveBase(const uint32_t instanceIdxIn, const int64_t time2live = 0, const size_t capLimit = 5U)
        : instanceIdx_(instanceIdxIn), dataContainer_(time2live, capLimit)
    {}
    virtual void StartFindService()
    {
        HAF_LOG_INFO << "find service....";
        std::string insIdxStr = std::to_string(instanceIdx_);
        ara::core::StringView id(insIdxStr.c_str());
        handle_ = Proxy::StartFindService(
            [this](
                const ara::com::ServiceHandleContainer<HandleType> handles, const ara::com::FindServiceHandle handler) {
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
    virtual ~DataReceiveBase()
    {
        stopFlag_ = true;
        receiveCv_.notify_all();
        Proxy::StopFindService(handle_);
        HAF_LOG_INFO << "Data receive baes destructor finished. idx:" << instanceIdx_;
    }
    void Stop()
    {
        stopFlag_ = true;
    }
    /**
     * @brief 获取一帧最新数据。阻塞式调用。
     *        满足两个条件之一，阻塞会往下走：1.收到数据到来提醒 2.超时时间到
     *        所设置的超时时间，应当比上游数据的发送时间间隔大。
     *        默认超时时间为8年
     */
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
        auto dataPtr = dataContainer_.GetOneData(); // 阻塞式接收，但保留数据在容器中
        recvLk.unlock();
        if (dataPtr == nullptr) {
            return HAF_ERROR;
        }
        data = *dataPtr;
        return HAF_SUCCESS;
    }
    /**
     * @brief 获取一帧最新数据。阻塞式调用。
     *        满足两个条件之一，阻塞会往下走：1.收到数据到来提醒 2.从调用该API开始的最长等待时间
     *        所设置的超时等待时间，应当比上游数据的发送时间间隔大。
     */
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
    /**
     * @brief 非阻塞式调用，立即获取缓存区的1帧数据。适用于本节点处理频率比上游快的场景。
     *        container中的历史数据不直接丢弃。
     *        可设置能够获取到数据的“新鲜”时间。如果没有符合该“新鲜”程度的数据，则返回nullptr
     *
     * @return std::shared_ptr<DataType> return nullptr if container is empty.
     */
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
    // 获取最旧一帧数据。非阻塞式调用。
    // 可选入参： erase ，为 true 则删除缓冲区最旧一帧数据。
    HafStatus GetLastOne(std::shared_ptr<DataType>& data, const bool erase = false)
    {
        if (stopFlag_) {
            HAF_LOG_INFO << "Data receive got stopFlag signal";
            return HAF_PROGRAM_STOPED;
        }
        if (dataContainer_.Empty()) {
            return HAF_NOT_READY;
        }
        data = dataContainer_.GetLastOne(erase);
        return HAF_SUCCESS;
    }
    /**
     * @brief 非阻塞式调用，立即获取缓存区的至多N帧数据。适用于本节点需要历史多帧数据的场景。
     *
     * @param n
     * @return std::vector<std::shared_ptr<DataType>> newer data in the front.
     */
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
    /**
     * @brief 清空消息容器中的历史数据
     */
    void Clear()
    {
        dataContainer_.Clear();
    }
    bool Empty() const
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
    /**
     * @brief 收到新数据后，用于将接收到的数据放到缓存区。并通知所有阻塞式数据获取函数，新数据到了。
     *
     * @param value
     */
    void PushToContainer(DataType&& value)
    {
        std::shared_ptr<DataType> p = std::make_shared<DataType>(std::forward<DataType>(value));

        dataContainer_.Push(std::move(p));
        receiveCv_.notify_all();
    }

    bool IsHandlesAndProxyValid(const ara::com::ServiceHandleContainer<HandleType> &handles) const
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
    std::unique_ptr<Proxy> proxyPtr_{};

private:
    std::mutex receiveMtx_;
    std::condition_variable receiveCv_;
    uint32_t instanceIdx_{};
    size_t recvQSize_{15U};
    size_t maxNumberOfSamples_{15U};
    ThreadSafeStack<std::shared_ptr<DataType>> dataContainer_;
    ara::com::FindServiceHandle handle_{};
    bool stopFlag_{false};
};
}  // namespace Adsfi
#endif
