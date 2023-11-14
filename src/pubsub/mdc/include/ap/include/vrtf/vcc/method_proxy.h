/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: get VCC info and use to transfer driver client method mode
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_METHODPROXY_H
#define VRTF_VCC_METHODPROXY_H
#include <memory>
#include <thread>
#include <map>
#include "ara/core/promise.h"
#include "vrtf/vcc/api/types.h"
#include "vrtf/vcc/api/internal/error_index.h"
#include "vrtf/vcc/api/com_error_domain.h"
#include "vrtf/vcc/serialize/method_param_deserializer.h"
#include "ara/core/error_code.h"
#include "vrtf/vcc/api/recv_buffer.h"
#include "vrtf/vcc/api/dynamic_error_domain.h"
#include "vrtf/vcc/api/e2e_error_domain.h"
namespace vrtf {
namespace vcc {
class MethodProxy {
public:
    MethodProxy() {}
    virtual ~MethodProxy() {}
    virtual void SetDriver(std::shared_ptr<vrtf::vcc::driver::MethodHandler>& drv) = 0;
    virtual std::shared_ptr<vrtf::vcc::driver::MethodHandler> GetDriver() const = 0;
    virtual ara::com::e2e::Result GetE2EResult() const noexcept = 0;
    virtual void SetFireAndForget(const bool fireForget) = 0;
    virtual void RegisterError(vrtf::core::ErrorCode const &error) = 0;
    virtual void DestroyWaitingPromise(vrtf::core::ErrorCode const &errorCode) = 0;
    virtual void SetMethodStateChangeHandler(vrtf::vcc::api::types::MethodStateChangeHandler const &handler) = 0;
    virtual void UnsetMethodStateChangeHandler() = 0;
    virtual void StopReceiveData() noexcept = 0;
    virtual void SetMethodInfo(const std::shared_ptr<vrtf::vcc::api::types::MethodInfo>& methodInfo) = 0;
    /**
     * @brief Used in Client for getting response's E2E SMState
     *
     * @return SMState
     */
    virtual ara::com::e2e::SMState GetSMState() const = 0;
};

/* This class is prepared for proxy to initiate method operation */
template<class Result>
class MethodProxyImpl : public MethodProxy, public std::enable_shared_from_this<MethodProxyImpl<Result>> {
public:
    MethodProxyImpl(vrtf::vcc::api::types::EntityId id)
        : id_(id)
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
        sessionId_ = 0;
        /* Registe CM error to errorList_. */

        RegisterComError(api::types::kMaxSamplesExceeded);
        RegisterComError(api::types::kNetworkBindingFailure);
        RegisterComError(api::types::kServiceNotAvailable);
    }

    ~MethodProxyImpl()
    {
        std::lock_guard<std::mutex> lock(deconMutex_);
        driver_ = nullptr;
    }

    virtual void SetMethodInfo(const std::shared_ptr<vrtf::vcc::api::types::MethodInfo>& methodInfo) override
    {
        methodInfo_ = methodInfo;
    }

    /**
     * @brief Register ComError to errorList_.
     * @details Register ComError to errorList_.
     * @param[in] msg MethodMsg which contain msg info for deserialize.
     */
    void RegisterComError(vrtf::core::ErrorDomain::CodeType type)
    {
        api::types::internal::ErrorIndex index(api::types::ComErrorDomain::ComErrorDomainId, type);
        const vrtf::core::ErrorCode comErrorCode(type, api::types::GetComErrorDomain());
        errorList_.emplace(std::pair<vrtf::vcc::api::types::internal::ErrorIndex,
                                                            vrtf::core::ErrorCode>(index, comErrorCode));
    }

    /**
     * @brief This funtion is proxy receive handler. After receive data from
     *        driver, do CM deserialize and set promise.
     * @param[in] msg MethodMsg which contain msg info for deserialize.
     */
    void OnDataAvailable(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg)
    {
        using namespace vrtf::vcc::api::types;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose() << "Method Proxy data available enter ......";
        std::lock_guard<std::mutex> lock(deconMutex_);
        if (driver_ == nullptr) {
            return;
        }
        if (!isFireAndForget_) {
            vrtf::vcc::api::types::SessionId sessionId = msg->GetSessionId();
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "The method request session id for method " << id_ <<" is "<< sessionId;
            promiseLck.lock();
            auto promisePair = promise_.find(sessionId);
            if (promisePair == promise_.end()) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->warn() << "Invalid method reply for method " << id_ << " received " << sessionId;
            } else {
                e2eResult_ = msg->GetE2EResult();
                e2eState_ = e2eResult_.GetSMState();
                const auto profileCheckStatus = e2eResult_.GetProfileCheckStatus();
                if ((methodInfo_->IsSettingResponseE2EErrc()) &&
                    (profileCheckStatus != ara::com::e2e::ProfileCheckStatus::kOk)) {
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance_->debug() <<  "Setting Response with E2E Error Code for [entityId = " << id_ << "]";
                    promisePair->second->SetError(mapE2EErrCode_.at(profileCheckStatus));
                } else {
                    bool result;
                    if (msg->GetMsgType()) {
                        result = DeserializeErrorCodeAndSetPromise(promisePair->second, msg);
                    } else {
                        result = DeserializeAndSetPromise(promisePair->second, msg);
                    }
                    if (result == false) {
                        /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                        /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                        logInstance_->warn() <<  "Invalid return value for method " << id_
                            << "." << sessionId <<  ". Please check the consistency of the return data type.";
                        /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                        /* AXIVION enable style AutosarC++19_03-A5.1.1 */

                        /* When proxy deserialize failed, we should make com ErrorCode. */
                        vrtf::core::ErrorCode comErrorCode(kNetworkBindingFailure, GetComErrorDomain());
                        promisePair->second->SetError(comErrorCode);
                    }
                }
                promise_.erase(promisePair);
            }
            promiseLck.unlock();
        }
        auto key = msg->GetPayload();
        driver_->ReturnLoan(key);
    }

    /**
     * @brief Destroy all waiting promise
     * @details when server is offline, this function should be used to set all waiting promise to ServiceNotAvailable
     *          error
     */
    virtual void DestroyWaitingPromise(vrtf::core::ErrorCode const &errorCode) override
    {
        promiseLck.lock();
        for (auto p : promise_) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Server is disappeared, promise for session " << p.first << " is destroyed";
            p.second->SetError(errorCode);
        }
        promise_.clear();
        promiseLck.unlock();
    }

    /**
     * @brief register method online status change callback
     * @details In no service-discovery mode, client check server method is online
     *
     * @param handler register callback when server online status changed
     */
    virtual void SetMethodStateChangeHandler(vrtf::vcc::api::types::MethodStateChangeHandler const &handler) override
    {
        if (driver_ != nullptr) {
            driver_->SetMethodStateChangeHandler(handler);
        }
    }

    /**
     * @brief unregister method online status change callback
     * @details unregister method online status change callback
     *
     */
    virtual void UnsetMethodStateChangeHandler() override
    {
        if (driver_ != nullptr) {
            driver_->UnsetMethodStateChangeHandler();
        }
    }

    /**
     * @brief Stop receive data from driver
     * @details According to driver, set revriveHandler to nullptr
     */
    void StopReceiveData() noexcept override
    {
        if (driver_ != nullptr) {
            driver_->SetReceiveHandler(nullptr);
        }
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->info() << "Method proxy unset handler success[entityId=" << id_ << "]";
    }

    template<class ValueType>
    bool DeserializeAndSetPromise(std::shared_ptr<vrtf::core::Promise<ValueType>> promise,
        const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg)
    {
        const auto sizeTmp = msg->GetSize();
        std::size_t deserializeSize = 0;
        if (msg->GetSerializeType() == vrtf::serialize::SerializeType::SHM) {
            vrtf::serialize::dds::Deserializer<ValueType> desr(msg->GetPayload(), sizeTmp,
                methodInfo_->GetReplySerializeConfig());
            deserializeSize = desr.GetSize();
            if (deserializeSize > sizeTmp) {
                return false;
            }
            promise->set_value(desr.GetValue());
        } else if (msg->GetSerializeType() == vrtf::serialize::SerializeType::SOMEIP) {
            vrtf::serialize::someip::Deserializer<ValueType> desr(msg->GetPayload(), sizeTmp,
                methodInfo_->GetReplySerializeConfig());
            deserializeSize = desr.GetSize();
            if (deserializeSize > sizeTmp) {
                return false;
            }
            promise->set_value(desr.GetValue());
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() <<  "Wrong serialize type, SHM deserialize type used!";
            vrtf::serialize::dds::Deserializer<ValueType> desr(msg->GetPayload(), sizeTmp,
                methodInfo_->GetReplySerializeConfig());
            deserializeSize = desr.GetSize();
            if (deserializeSize > sizeTmp) {
                return false;
            }
            promise->set_value(desr.GetValue());
        }
        if (deserializeSize < sizeTmp) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->debug() << "Invalid return value for method " << msg->GetEntityId()
                << "." << msg->GetSessionId() <<  ". Please check the consistency of the return data type.";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        }
        return true;
    }

    bool DeserializeAndSetPromise(std::shared_ptr<vrtf::core::Promise<void>> promise,
        const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg)
    {
        promise->set_value();
        return true;
    }

    /**
     * @brief Get error code from payload
     * @details Deserializer payload and get error code
     * @param[in] msg a shared_ptr point to method message
     * @param[out] errorData error code get from payload
     * @return Whether to get error code successfully
     *   @retval true Get error code success
     *   @retval false Deserializer fail and get error code failed
     */
    bool GetErrorCodeData(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg,
        api::types::MethodError& errorData)
    {
        const auto sizeTmp = msg->GetSize();
        if (msg->GetSerializeType() == vrtf::serialize::SerializeType::SHM) {
            vrtf::serialize::dds::Deserializer<ara::core::ErrorCode> desr(msg->GetPayload(), msg->GetSize(),
                methodInfo_->GetReplySerializeConfig());
            if (desr.GetSize() != sizeTmp) {
                return false;
            }
            errorData = desr.GetValue();
        } else if (msg->GetSerializeType() == vrtf::serialize::SerializeType::SOMEIP) {
            vrtf::serialize::someip::Deserializer<ara::core::ErrorCode> desr(msg->GetPayload(), msg->GetSize(),
                methodInfo_->GetReplySerializeConfig());
            if (desr.GetSize() != sizeTmp) {
                return false;
            }
            errorData = desr.GetValue();
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Wrong serialize type, SHM deserialize type used!";
            vrtf::serialize::dds::Deserializer<ara::core::ErrorCode> desr(msg->GetPayload(), msg->GetSize(),
                methodInfo_->GetReplySerializeConfig());
            if (desr.GetSize() != sizeTmp) {
                return false;
            }
            errorData = desr.GetValue();
        }
        return true;
    }

    /**
     * @brief Deserialize error code and set promise
     * @details Get the error code from payload, and check if errorcode is registered
     * @param[in] promise Promise corresponding to user client future
     * @param[in] msg a point to method msg
     * @return Whether to get error code successfully
     *   @retval true Get and check error code success
     *   @retval false Deserializer fail or check error code failed
     */
    template<class ValueType>
    bool DeserializeErrorCodeAndSetPromise(std::shared_ptr<vrtf::core::Promise<ValueType>> promise,
        const std::shared_ptr<vrtf::vcc::api::types::MethodMsg> &msg)
    {
        api::types::MethodError errorData;

        if (GetErrorCodeData(msg, errorData) == false) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->warn() <<  "Invalid return error code value for method "
                    << msg->GetEntityId() << "." << msg->GetSessionId();
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return false;
        }

        const api::types::internal::ErrorIndex index(errorData.domainValue, errorData.errorCode);
        if (errorList_.find(index) != errorList_.end()) {
            const vrtf::core::ErrorCode userErrorCode(
                errorList_.find(index)->second.Value(),
                errorList_.find(index)->second.Domain());
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->verbose() << "Receive application error errorcode "
                                << errorData.errorCode << " error domain "
                                << errorData.domainValue << " for method " << GetMethodMessage();
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            promise->SetError(userErrorCode);
        } else {
            // Dynamic error codes are used to return all error codes, even if the range of acceptable error codes is
            // not registered to method proxy in advance.
            const vrtf::core::ErrorCode defaultDynamicErrorCode(vrtf::vcc::api::types::DynamicErrc::default_errc);
            const api::types::internal::ErrorIndex defaultDynamicErrorIndex(defaultDynamicErrorCode.Domain().Id(),
                defaultDynamicErrorCode.Value());
            if (errorList_.find(defaultDynamicErrorIndex) != errorList_.end()) {
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance_->verbose() << "Receive dynamic error errorcode " << errorData.errorCode << " error domain "
                    << errorData.domainValue << " for method " << GetMethodMessage();
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
                auto dynamicErrorDomain =
                    std::make_unique<vrtf::vcc::api::types::DynamicErrorDomain>(errorData.domainValue);
                const vrtf::core::ErrorCode dynamicErrorCode(errorData.errorCode, *dynamicErrorDomain);
                promise->SetError(dynamicErrorCode);
                // Because the error code only stores the address of the errodomain, error domain need to be saved in
                // errorCache_ to extend the life cycle of the dynamic error domain. Try to fix the problem by save
                // errordomain value in error code instead of address later
                if (errorCache_.size() == ERROR_CACHE_MAX_SIZE) {
                    errorCache_.pop_front();
                }
                errorCache_.push_back(std::move(dynamicErrorDomain));
                return true;
            } else {
                /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
                /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
                logInstance_->warn() << "Receive unregistered error " << errorData.domainValue << "."
                    << errorData.errorCode << "(error domain.error code) for method " << GetMethodMessage()
                    << " Discard invalid reply msg";
                /* AXIVION enable style AutosarC++19_03-A5.0.1 */
                /* AXIVION enable style AutosarC++19_03-A5.1.1 */
                return false;
            }
        }
        return true;
    }

    virtual void SetDriver(std::shared_ptr<vrtf::vcc::driver::MethodHandler>& drv) override
    {
        driver_ = drv;
        std::weak_ptr<MethodProxyImpl<Result>> weakSelf = this->shared_from_this();
        driver_->SetReceiveHandler([weakSelf](std::shared_ptr<vrtf::vcc::api::types::MethodMsg> const &msg) {
            std::shared_ptr<MethodProxyImpl<Result>> self = weakSelf.lock();
            if (self) {
                self->OnDataAvailable(msg);
            }
        });
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose() << "Method Proxy Set driver success...";
    }

    virtual std::shared_ptr<vrtf::vcc::driver::MethodHandler> GetDriver() const override
    {
        return driver_;
    }
    virtual ara::com::e2e::Result GetE2EResult() const noexcept override
    {
        return e2eResult_;
    }
    /**
     * @brief Save promise and send request by driver.
     * @param ResultType template about Future.
     * @param[in] promise save the promise when receive reply data to set value.
     * @param[in] size    request data size.
     * @param[in] buf     A pointer to the first address of request data.
     * @return A Future which is bind with promise contain a value.
     */
    template<class ResultType>
    vrtf::core::Future<ResultType> SavePromiseAndSendRequest(
        std::shared_ptr<vrtf::core::Promise<ResultType>>& promise, std::size_t size, uint8_t* buf)
    {
        using namespace vrtf::vcc::serialize;
        using namespace vrtf::vcc::api::types;
        auto ret = promise->get_future();
        promiseLck.lock();
        if (promise_.find(sessionId_) != promise_.end()) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->warn() << "Method " << id_ << " request in session " << sessionId_
                            << " has been lost, and the new request will replace the old one!";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        }
        // Save promise_ must before Request
        promise_[sessionId_] = promise;
        promiseLck.unlock();
        /* Check request failed or not, if failed, we should make com ErrorCode. */
        if (driver_->Request(buf, size, sessionId_) == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Method " << id_ << " request failed.";
            const vrtf::core::ErrorCode comErrorCode(kNetworkBindingFailure, GetComErrorDomain());
            promiseLck.lock();
            auto promisePair = promise_.find(sessionId_);
            if (promisePair != promise_.end()) {
                promisePair->second->SetError(comErrorCode);
                promise_.erase(sessionId_);
            }
            promiseLck.unlock();
        }
        /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
        /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
        logInstance_->debug() << "Method proxy send request end[methodUUID=" << methodInfo_->GetMethodUUIDInfo()
            << ", sessionId=" << sessionId_ << "]";
        /* AXIVION enable style AutosarC++19_03-A5.0.1 */
        /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        return ret;
    }

    /**
     * @brief Save promise and send request by driver.
     * @param ResultType template about Future.
     * @param[in] promise save the promise when receive reply data to set value.
     * @param[in] size    request data size.
     * @param[in] buf     A pointer to the first address of request data.
     * @return A Future which is bind with promise contain no value..
     */
    vrtf::core::Future<void> SavePromiseAndSendRequest(
        std::shared_ptr<vrtf::core::Promise<void>>& promise, std::size_t size, uint8_t* buf)
    {
        using namespace vrtf::vcc::serialize;
        using namespace vrtf::vcc::api::types;
        auto ret = promise->get_future();
        // Save promise_ must before Request
        if (isFireAndForget_ == false) {
            promiseLck.lock();
            promise_[sessionId_] = promise;
            promiseLck.unlock();
        }
        /* Check request failed or not, if failed, we should make com ErrorCode. */
        if (driver_->Request(buf, size, sessionId_) == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Method " << id_ << " request failed.";
            const vrtf::core::ErrorCode comErrorCode(kNetworkBindingFailure, GetComErrorDomain());
            promiseLck.lock();
            if (isFireAndForget_ == false) {
                auto promisePair = promise_.find(sessionId_);
                if (promisePair != promise_.end()) {
                    promisePair->second->SetError(comErrorCode);
                    promise_.erase(sessionId_);
                }
            } else {
                promise->SetError(comErrorCode);
            }
            promiseLck.unlock();
        } else {
            if (isFireAndForget_) {
                promise->set_value();
            }
        }
        /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
        /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
        logInstance_->debug() << "Method proxy send request end[type=void, methodUUID="
            << methodInfo_->GetMethodUUIDInfo() << ", sessionId=" << sessionId_ << "]";
        /* AXIVION enable style AutosarC++19_03-A5.0.1 */
        /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        return ret;
    }

    template<class... Args>
    vrtf::core::Future<Result> Request(Args ...args)
    {
        using namespace vrtf::vcc::serialize;
        using namespace vrtf::vcc::api::types;
        uint8_t *buf = nullptr;
        std::shared_ptr<vrtf::core::Promise<Result>> promise = std::make_shared<vrtf::core::Promise<Result>>();
        if (driver_ == nullptr) {
            auto ret = promise->get_future();
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Method id " << id_ <<" driver is nullptr, request failed";
            vrtf::core::ErrorCode comErrorCode(kNetworkBindingFailure, GetComErrorDomain());
            promise->SetError(comErrorCode);
            return ret;
        }
        std::lock_guard<std::mutex> guard(methodProxyMutex_);
        const uint16_t SessionIdMax = 65535;
        if (sessionId_ == SessionIdMax) {
            sessionId_ = 1;
        }
        ++sessionId_;
        ParamsSerializer marshaller(buf, driver_->GetSerializeType(), methodInfo_->GetRequestSerializeConfig());
        const std::size_t size = marshaller.GetSize(args...);
        std::shared_ptr<vrtf::vcc::api::types::MethodMsg> msg = std::make_shared<vrtf::vcc::api::types::MethodMsg>();
        msg->SetSessionId(sessionId_);
        buf = const_cast<uint8_t*>(driver_->AllocateBuffer(size, msg));
        if (buf != nullptr) {
            marshaller.SetBuffer(buf);
            marshaller.Serialize(args...);
        }

        return SavePromiseAndSendRequest(promise, size, buf);
    }

    virtual void SetFireAndForget(const bool fireForget) override
    {
        if (fireForget != isFireAndForget_) {
            isFireAndForget_ = fireForget;
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() << "Method " << id_ << " isFireForget state set to " << fireForget;
        }
    }

    virtual void RegisterError(vrtf::core::ErrorCode const &error) override
    {
        const vrtf::core::ErrorDomain::IdType domainId = error.Domain().Id();
        const vrtf::core::ErrorDomain::CodeType errorCode = error.Value();
        vrtf::vcc::api::types::internal::ErrorIndex index(domainId, errorCode);
        if (errorList_.find(index) != errorList_.end()) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->warn() << "The same error code with domain Id " << domainId << " error code " << errorCode
                << " is already registered. New error code will replace the old one";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
        }
        errorList_.insert(std::pair<vrtf::vcc::api::types::internal::ErrorIndex, vrtf::core::ErrorCode>(index, error));
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->debug() << " Insert errorCode " << errorList_.find(index)->second.Value();
    }

    /**
     * @brief Get method message which displays method information in user-readable string
     * @return method information string
     */
    std::string GetMethodMessage() const
    {
        return std::to_string(methodInfo_->GetServiceId()) + "."
            + methodInfo_->GetInstanceId() + "." + methodInfo_->GetShortName();
    }

    /**
     * @brief Used in Client for getting response's E2E SMState
     *
     * @return SMState
     */
    virtual ara::com::e2e::SMState GetSMState() const override
    {
        return e2eState_;
    }
private:
    vrtf::vcc::api::types::EntityId const id_;
    vrtf::vcc::api::types::SessionId sessionId_;
    std::mutex promiseLck;
    std::map<vrtf::vcc::api::types::SessionId, std::shared_ptr<vrtf::core::Promise<Result>>> promise_;
    std::shared_ptr<vrtf::vcc::driver::MethodHandler> driver_ = nullptr;
    bool isFireAndForget_ = false;
    std::map<api::types::internal::ErrorIndex, vrtf::core::ErrorCode> errorList_;
    std::mutex methodProxyMutex_;
    std::mutex deconMutex_;
    std::mutex promiseSetMutex_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    std::shared_ptr<vrtf::vcc::api::types::MethodInfo> methodInfo_;
    ara::com::e2e::Result e2eResult_;
    ara::com::e2e::SMState e2eState_ = ara::com::e2e::SMState::kStateMDisabled;
    std::deque<std::unique_ptr<vrtf::vcc::api::types::DynamicErrorDomain>> errorCache_;
    const uint32_t ERROR_CACHE_MAX_SIZE = 32;
    const std::map<ara::com::e2e::ProfileCheckStatus, vrtf::vcc::api::types::E2EErrorCode> mapE2EErrCode_ = {
        {ara::com::e2e::ProfileCheckStatus::kRepeated, vrtf::vcc::api::types::E2EErrorCode::repeated},
        {ara::com::e2e::ProfileCheckStatus::kWrongSequence, vrtf::vcc::api::types::E2EErrorCode::wrong_sequence_error},
        {ara::com::e2e::ProfileCheckStatus::kError, vrtf::vcc::api::types::E2EErrorCode::error},
        {ara::com::e2e::ProfileCheckStatus::kNotAvailable, vrtf::vcc::api::types::E2EErrorCode::not_available},
        {ara::com::e2e::ProfileCheckStatus::kNoNewData, vrtf::vcc::api::types::E2EErrorCode::no_new_data},
        {ara::com::e2e::ProfileCheckStatus::kCheckDisabled, vrtf::vcc::api::types::E2EErrorCode::check_disable}
    };
};
}
}
#endif
