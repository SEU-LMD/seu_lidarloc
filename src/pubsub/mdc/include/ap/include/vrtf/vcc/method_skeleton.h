/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: get VCC info and use to transfer driver server method mode
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_METHOD_SKELETON_H
#define VRTF_VCC_METHOD_SKELETON_H
#include <functional>
#include <utility>
#include <mutex>
#include "vrtf/vcc/utils/thread_pool.h"
#include "vrtf/vcc/driver/method_handler.h"
#include "vrtf/vcc/internal/traffic_crtl_policy.h"
#include "ara/hwcommon/log/log.h"
#include "vrtf/vcc/serialize/dds_serialize.h"
#include "vrtf/vcc/serialize/someip_serialize.h"
#include "ara/com/com_error_domain.h"
#include "vrtf/vcc/api/types.h"
#include "vrtf/vcc/api/vcc_method_return_type.h"

namespace vrtf {
namespace vcc {
const size_t METHOD_HANDLER_THREAD_NUM {6};
/*
  this class is used by skeleton to support method operation
  subDriver is used to read msg from proxy
  pubDriver is used to write msg to proxy
*/
class MethodSkeleton {
public:
    MethodSkeleton() {}
    virtual ~MethodSkeleton() {}
    virtual vrtf::vcc::api::types::MethodCallProcessingMode GetMethodMode() = 0;
    virtual void SetMethodMode(vrtf::vcc::api::types::MethodCallProcessingMode mode) = 0;
    virtual void SetDriver(
        vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::driver::MethodHandler>& drv) = 0;
    virtual void SetThreadPool(std::shared_ptr<utils::ThreadPool> &pool) = 0;
    virtual void SetMethodVccHandler(
        std::function<void(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg)> handler) = 0;
    virtual void SetFireAndForget(const bool fireAndForget) = 0;
    virtual void UnsetMethodCallback() = 0;
    virtual void ProcessMethodRequest(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg) = 0;
    virtual void SetDriverHandler() = 0;
    virtual void SetTrafficCtrl(const std::shared_ptr<rtf::TrafficCtrlPolicy>& policy) = 0;
    virtual void SetMethodInfo(std::map<vrtf::vcc::api::types::DriverType,
        std::shared_ptr<vrtf::vcc::api::types::MethodInfo>>& methodInfo) = 0;
};
template<class T>
class MethodSkeletonImpl : public MethodSkeleton, public std::enable_shared_from_this<MethodSkeletonImpl<T>> {
public:
    using MethodInfoMap =
        std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::MethodInfo>>;

    MethodSkeletonImpl(vrtf::vcc::api::types::EntityId id, vrtf::vcc::api::types::MethodCallProcessingMode mode,
        std::function<T(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>&, MethodInfoMap&)> cb = nullptr)
        : id_(id), mode_(mode), methodCb_(cb), isFireAndForget_(false),
          logInstance_(ara::godel::common::log::Log::GetLog("CM")), callUserCallBackStatus_(0) {}

    /**
     * @brief Cancel the callback function
     * @details this method must be called before destroy MethodSkeletonImpl, in case
     * @param pool thread pool.
     */
    void UnsetMethodCallback() override
    {
        // After SetReceiveHandler(nullptr), the ongoing  MethodHander callback(OnRequestAvailable) should finish.
        for (auto it = driver_.begin(); it != driver_.end(); it++) {
            it->second->SetReceiveHandler(nullptr);
        }
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            methodCb_ = nullptr;
            pool_.reset();
            vccHandler_ = nullptr;
        }
        {
            std::unique_lock<std::mutex> callBackOverLock(callbackMutex_);
            userCallBackOverCond_.wait(callBackOverLock, [this]() {
                return (callUserCallBackStatus_ == 0);
            });
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "MethodSkeletonImpl is unset method callback with method id:" << id_;
        }
    }

    ~MethodSkeletonImpl()
    {
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose() << "MethodSkeletonImpl deconstructor";
    }

    virtual void SetMethodInfo(MethodInfoMap& methodInfo) override
    {
        methodInfo_ = methodInfo;
    }

    /**
     * @brief before offerservice register method callback
     * @details before offerservice register method callback
     *
     * @param[in] callback register method callback
     */
    void SetMethodSkelCallBack(
        std::function<T(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>&, MethodInfoMap&)> callback)
    {
        methodCb_ = callback;
    }

    void SetTrafficCtrl(const std::shared_ptr<rtf::TrafficCtrlPolicy>& policy) override
    {
        trafficCtrlPolicy_ = policy;
    }

    /* This function is used by protocol driver */
    void OnRequestAvailable(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg)
    {
        if (trafficCtrlPolicy_ != nullptr) {
            rtf::TrafficCtrlAction action = trafficCtrlPolicy_->GetTrafficCtrlAction();
            if (trafficCtrlPolicy_->UpdateTrafficInfo(action) == false) {
                bool trafficControl {true};
                msg->SetTrafficCtrlFlag(trafficControl);
                ProcessMethodRequest(msg);
                return;
            }
        }
        if (mode_ != vrtf::vcc::api::types::MethodCallProcessingMode::kPoll) {
            /* read data from protocal driver */
            if ((pool_ != nullptr) && (methodCb_ != nullptr)) {
                std::shared_ptr<MethodSkeletonImpl<T>> self = this->shared_from_this();
                std::function<void()> task = [self, msg]() {
                    self->ProcessMethodRequest(msg);
                };
                std::pair<uint64_t, uint64_t> msgSourceID = msg->GetMsgSourceIdToVcc();
                utils::TaskSoureceId taskKey {GetTypeId(msg->GetDriverType()), msgSourceID.first, msgSourceID.second};
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance_->verbose() << "Put msg to workpool to process[taskTypeId=" << taskKey.typeId
                    << ", uuidHight=" << taskKey.uuidHight << ", uuidLow=" << taskKey.uuidLow << "]";
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
                if (pool_->Enqueue(task, taskKey) == false) {
                    ReturnLoan(msg);
                }
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                ReturnLoan(msg);
                logInstance_->warn() << "No user handler registerd ...";
            }
        } else {
            if (vccHandler_ != nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->verbose() << "Msg put to vcc to process...";
                vccHandler_(msg);
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->warn() << "No Vcc receiver handler was registered, the msg will be lost...";
            }
        }
    }
    template<typename M, typename std::enable_if<std::is_void<M>::value>::type* = nullptr>
    void TriggerCallbackAndSend(
        std::function<T(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>&, MethodInfoMap&)> methodCb,
        const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg, MethodInfoMap& methodInfo)
    {
        methodCb(msg, methodInfo);
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose() << "Fire and Forget mode, no need to send reply to proxy";
    }

    template<typename M, typename std::enable_if<ara::core::internal::IsFuture<M>::value>::type* = nullptr>
    void TriggerCallbackAndSend(
        std::function<M(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>&, MethodInfoMap&)> methodCb,
        const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg, MethodInfoMap &methodInfo)
    {
        auto res = methodCb(msg, methodInfo);
        if (res.valid() == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "User return future is invalid, please check future whether associate with promise";
            return;
        }
        std::weak_ptr<MethodSkeletonImpl<T>> weakSelf = this->shared_from_this();
        static_cast<void>(res.then([weakSelf, msg](M resultFuture) {
            auto methodSkeletonPtr = weakSelf.lock();
            if (methodSkeletonPtr != nullptr) {
                methodSkeletonPtr->SerializeAndSendReply(std::move(resultFuture), msg);
            }
        }));
    }

    template<typename M,
        typename std::enable_if<vrtf::vcc::api::internal::IsVccMethodReturnType<M>::value>::type* = nullptr>
    void TriggerCallbackAndSend(
        std::function<M(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>&, MethodInfoMap&)> methodCb,
        const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg, MethodInfoMap &methodInfo)
    {
        auto res = methodCb(msg, methodInfo);
        SerializeAndSendReply(std::move(res), msg);
    }

     /**
     * @brief Respond to the request, and trigger the user callback function,
     *        serialize the return value of the callback function and send a
     *        reply to the client
     * @param msg MethodMsg which contain msg info for serialize.
     */
    void ProcessMethodRequest(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg) override
    {
        std::function<T(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>&, MethodInfoMap&)> methodCb;
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            ++callUserCallBackStatus_;
            methodCb = methodCb_;
        }
        if (methodCb != nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Call user method handler " << id_;
            if (msg->GetTrafficCtrlFlag() == true) {
                vrtf::core::Promise<void> promise;
                promise.SetError(vrtf::core::ErrorCode(vrtf::vcc::api::types::ComErrc::kMaxSamplesExceeded));
                vrtf::core::Future<void> res = promise.get_future();
                bool isErrorMsg {true};
                msg->SetMsgType(isErrorMsg);
                SerializeAndSendReply(std::move(res), msg);
            } else {
                TriggerCallbackAndSend<T>(methodCb, msg, methodInfo_);
            }
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Method was not registerd...";
        }
        ReturnLoan(msg);
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            --callUserCallBackStatus_;
        }
        userCallBackOverCond_.notify_all();
    }

    /**
     * @brief After user callback, skeleton should send reply to proxy with vrtf::core::Future<ValueType>.
     * @param result ref from user result, contain data or error code.
     * @param msg MethodMsg which contain msg info for serialize.:w
     */
    template<class ValueType>
    void SerializeAndSendReply(vrtf::core::Future<ValueType> && result,
                               const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg)
    {
        std::lock_guard<std::mutex> lock(replyMutex_);
        if (!isFireAndForget_) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Send method reply msg using using future<ValueType> type, method id is:" << id_;
            auto data = result.GetResult();
            if (data.HasValue()) {
                ReplyValueOrError(data.Value(), msg, false);
            } else {
                auto error = data.Error();
                if (ara::core::GetFutureErrorDomain() == error.Domain()) {
                    ara::core::ThrowOrTerminate<vrtf::core::FutureException>(error);
                }
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->verbose() << "Server starts to send method error code for method id " << id_;
                ReplyValueOrError(error, msg, true);
            }
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Fire and Forget mode, no need to send reply to proxy";
        }
    }

    /**
     * @brief After user callback, skeleton should send reply to proxy with vrtf::core::Future<void>.
     * @param result ref from user result, contain data or error code.
     * @param msg MethodMsg which contain msg info for serialize.
     */
    void SerializeAndSendReply(vrtf::core::Future<void> && result,
                               const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg)
    {
        std::lock_guard<std::mutex> lock(replyMutex_);
        if (!isFireAndForget_) {
            auto data = result.GetResult();
            if (data.HasValue()) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->verbose() << "Send void method reply msg using future<void> type, method id is:" << id_;
                auto drvType = msg->GetDriverType();
                msg->SetMsgType(false);
                // length 1, dds will malloc buffer for CM as map key, 0 will return nullptr
                auto buffer = const_cast<uint8_t*>(driver_[drvType]->AllocateBuffer(1, msg));
                // length 1, dds will malloc buffer for CM as map key, 0 will return nullptr
                driver_[drvType]->Reply(buffer, 1, msg);
            } else {
                auto error = data.Error();
                if (ara::core::GetFutureErrorDomain() == error.Domain()) {
                    ara::core::ThrowOrTerminate<vrtf::core::FutureException>(error);
                }
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->verbose() << "Server starts to send method error code for method id " << id_;
                ReplyValueOrError(error, msg, true);
            }
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Fire and Forget mode, no need to send reply to proxy";
        }
    }

    /**
     * @brief After user callback, skeleton should send reply to proxy with vrtf::core::Future<void>
     *
     * @tparam ValueType            The data type of reply data
     * @param[in] returnValue       Ref from user result, contain data or error code.
     * @param[in] msg               MethodMsg which contain msg info for serialize.
     */
    template<class ValueType>
    void SerializeAndSendReply(vrtf::vcc::api::VccMethodReturnType<ValueType> && returnValue,
                               const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg)
    {
        if (!isFireAndForget_) {
            bool isUsingIncorrectId {returnValue.GetIsUsingIncorrectId()};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Send method reply msg using the type contains E2E info, method id is:" << id_;
            auto replyData = returnValue.GetReplyResult();
            if (replyData.HasValue()) {
                ReplyValueOrError(replyData.Value(), msg, false, isUsingIncorrectId);
            } else {
                auto error = replyData.Error();
                if (ara::core::GetFutureErrorDomain() == error.Domain()) {
                    ara::core::ThrowOrTerminate<vrtf::core::FutureException>(error);
                }
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->verbose() << "Server starts to send method error code for method id " << id_;
                ReplyValueOrError(error, msg, true, isUsingIncorrectId);
            }
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Fire and Forget mode, no need to send reply to proxy";
        }
    }

    virtual vrtf::vcc::api::types::MethodCallProcessingMode GetMethodMode() override
    {
        return mode_;
    }
    virtual void SetMethodMode(vrtf::vcc::api::types::MethodCallProcessingMode mode) override
    {
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->debug() << "Method " << id_ << "s ProcessMode is set";
        mode_ = mode;
    }
    /**
     * @brief When register handler add all driver MethodCallback
     * @details When register handler add all driver MethodCallback
     */
    virtual void SetDriverHandler() override
    {
        std::weak_ptr<MethodSkeletonImpl<T>> weakSelf = this->shared_from_this();
        for (auto iter : driver_) {
            iter.second->SetReceiveHandler([weakSelf] (std::shared_ptr<vrtf::vcc::api::types::MethodMsg> const &msg) {
                std::shared_ptr<MethodSkeletonImpl<T>> self = weakSelf.lock();
                if (self) {
                    self->OnRequestAvailable(msg);
                }
            });
        }
    }

    virtual void SetDriver(vrtf::vcc::api::types::DriverType type,
                           std::shared_ptr<vrtf::vcc::driver::MethodHandler>& drv) override
    {
        using namespace vrtf::vcc::api::types;
        using namespace vrtf::vcc::driver;
        if (driver_.find(type) != driver_.end()) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "driver " << static_cast<int>(type) << "was seted for method :" << id_;
            return;
        }
        driver_.insert(std::pair<DriverType, std::shared_ptr<MethodHandler>>(type, drv));
    }
    /**
     * @brief Set the thread pool in which the method request is processed.
     * @details this method should only be called during Method initialization phase. If you want to destroy method
     *          skeleton , UnsetMethodCallbak should be used to reset thread pool instead of SetThreadPool(nullptr).
     * @param pool thread pool.
     */
    virtual void SetThreadPool(std::shared_ptr<utils::ThreadPool> &pool) override
    {
        pool_ = pool;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->debug() << "Set thread pool into methodSkeleton for " << id_;
    }

    virtual void SetMethodVccHandler(
        std::function<void(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg)> handler) override
    {
        /* This lock is lock vccHandler_. Avoid coredump when set vccHandler_ to null, request onAvailable */
        std::lock_guard<std::mutex> lock(callbackMutex_);
        if (mode_ != vrtf::vcc::api::types::MethodCallProcessingMode::kPoll) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Method mode is not poll , ignore set vcc handler...";
        } else {
            vccHandler_ = handler;
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() << "Vcc handler is setted ...";
        }
    }
    virtual void SetFireAndForget(const bool fireAndForget) override
    {
        if (isFireAndForget_ != fireAndForget) {
            isFireAndForget_ = fireAndForget;
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() << "Method " << id_ << " fireforget mode is set to " << fireAndForget;
        }
    }
private:
    std::mutex callbackMutex_;
    std::mutex replyMutex_;
    vrtf::vcc::api::types::EntityId const id_;
    std::shared_ptr<utils::ThreadPool> pool_; /* handle method requests */
    std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::driver::MethodHandler>> driver_;
    vrtf::vcc::api::types::MethodCallProcessingMode mode_;
    std::function<T(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>&, MethodInfoMap&)> methodCb_;
    std::function<void(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg)> vccHandler_;
    bool isFireAndForget_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    std::shared_ptr<rtf::TrafficCtrlPolicy> trafficCtrlPolicy_;
    MethodInfoMap methodInfo_;
    uint32_t callUserCallBackStatus_;
    std::condition_variable userCallBackOverCond_;
    /**
     * @brief Reply value and error to client
     * @details Serializer the value or error and reply to client
     * @param[in] data The data to serializer and send to client
     * @param[in] msg a point to method msg
     * @param[in] isError Flag of value or error
     */
    template<class Y>
    void ReplyValueOrError(Y& data, const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg, bool isError,
                           const bool isUsingIncorrectId = false)
    {
        if (msg->GetSerializeType() == vrtf::serialize::SerializeType::SHM) {
            vrtf::serialize::dds::Serializer<typename std::decay<Y>::type> sample(
                data, methodInfo_[msg->GetDriverType()]->GetReplySerializeConfig());
            ReplyData(sample, msg, isError, isUsingIncorrectId);
        } else if (msg->GetSerializeType() == vrtf::serialize::SerializeType::SOMEIP) {
            vrtf::serialize::someip::Serializer<typename std::decay<Y>::type> sample(
                data, methodInfo_[msg->GetDriverType()]->GetReplySerializeConfig());
            ReplyData(sample, msg, isError, isUsingIncorrectId);
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Wrong serialization type, SHM serialization will be used!";
            vrtf::serialize::dds::Serializer<typename std::decay<Y>::type> sample(
                data, methodInfo_[msg->GetDriverType()]->GetReplySerializeConfig());
            ReplyData(sample, msg, isError, isUsingIncorrectId);
        }
    }

    template<class Y>
    void ReplyData(Y& sample, const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg, bool isError,
                   const bool isUsingIncorrectId = false)
    {
        size_t size {sample.GetSize()};
        auto drvType = msg->GetDriverType();
        msg->SetMsgType(isError);
        auto buffer = const_cast<uint8_t*>(driver_[drvType]->AllocateBuffer(size, msg));
        if (buffer == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Allocate buffer failed when reply error in method " << id_;
            return;
        }
        sample.Serialize(buffer);
        if (isError) {
            if (buffer == nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->error() << "Allocate buffer failed when reply error in method " << id_;
                return;
            }
            driver_[drvType]->ReplyError(buffer, size, msg, isUsingIncorrectId);
        } else {
            driver_[drvType]->Reply(buffer, size, msg, isUsingIncorrectId);
        }
    }

    void ReturnLoan(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg)
    {
        auto drvType = msg->GetDriverType();
        auto addr = msg->GetPayload();
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose() << "Return resources to driver for method request payload...";
        if (driver_[drvType] == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Driver: " << drvType << " method handler is null.";
        } else {
            driver_[drvType]->ReturnLoan(addr);
        }
    }
    std::uint16_t GetTypeId(api::types::DriverType driverType) const
    {
        const std::uint16_t typeId = (static_cast<uint16_t>(driverType) <<
            static_cast<std::uint8_t>(api::types::MOVEBIT::MOVE8BIT)) +
            static_cast<std::uint8_t>(api::types::EntityType::METHOD);
        return typeId;
    }
};
}
}
#endif /* VRTF_VCC_METHOD_HANDLER_H */
