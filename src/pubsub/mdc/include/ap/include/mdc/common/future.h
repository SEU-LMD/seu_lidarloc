/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of Future class according to AutoSAR standard core type
 * Create: 2019-07-30
 */
#ifndef MDC_COMMON_FUTURE_H
#define MDC_COMMON_FUTURE_H

#include "mdc/common/future_error_domain.h"
#include "mdc/common/error_code.h"
#include "mdc/common/internal/type_check.h"
#include "mdc/common/result.h"
#include "mdc/common/core_error_domain.h"

#include <chrono>
#include <future>
#include <system_error>

namespace mdc {
namespace common {
namespace internal {
class State {
public:
    using Ptr = std::shared_ptr<State>;

    void FireCallBack()
    {
        threadId_ = std::this_thread::get_id();
        if (callback_ != nullptr) {
            callback_();
        }
    }

    template <typename F>
    void SetCallBack(F&& callback) noexcept
    {
        callback_ = std::forward<F>(callback);
    }

    void LockCallback() noexcept
    {
        mutex_.lock();
    }

    void UnlockCallback() noexcept
    {
        mutex_.unlock();
    }

    std::thread::id GetCallbackThreadId() const noexcept
    {
        return threadId_;
    }

private:
    std::function<void(void)> callback_;
    std::mutex mutex_;
    std::thread::id threadId_; // Fire callback thread id
};
}


template <typename, typename>
class Promise;

enum class future_status : uint8_t {
    ready = 1,
    timeout,
};


/**
 * @brief Provides ara::core specific Future operations to collect the results of an asynchronous call[SWS_CORE_00321].
 *
 * @tparam T the type of values
 * @tparam ErrorCode the type of errors
 */
template <typename T, typename E = ErrorCode>
class Future final {
    using UniLock = std::unique_lock<std::mutex>;

public:
    using ValueType = T;
    using ErrorType = E;
    /// Alias type for the Promise type collaborating with this Future type
    using PromiseType = Promise<T, E>;
    /// Alias type for the future_status type
    using Status = future_status;
    Future() noexcept = default;

    ~Future()
    {
        UniLock lock(mutex_);
        if (state_ && (std::this_thread::get_id() != state_->GetCallbackThreadId())) {
            state_->LockCallback();
            state_->SetCallBack(nullptr);
            state_->UnlockCallback();
        }
    }

    Future(Future const&) = delete;
    Future& operator=(Future const&) = delete;

    Future(Future&& other) noexcept
        : lock_(other.mutex_), stdFuture_(std::move(other.stdFuture_)), state_(std::move(other.state_))
    {
        lock_.unlock();
    }

    Future& operator=(Future&& other) noexcept
    {
        if (&other != this) {
            UniLock lhsLock(mutex_, std::defer_lock);
            UniLock rhsLock(other.mutex_, std::defer_lock);
            std::lock(lhsLock, rhsLock);

            stdFuture_ = std::move(other.stdFuture_);
            state_ = std::move(other.state_);
        }
        return *this;
    }

    Result<T, E> GetResult()
    {
        // Because std::future doesn't support exception check, NoException mode isn't supported in GetResult.
        return stdFuture_.get();
    }

    bool valid() const noexcept
    {
        return stdFuture_.valid();
    }

    void wait() const
    {
        stdFuture_.wait();
    }

    template <typename Rep, typename Period>
    future_status wait_for(std::chrono::duration<Rep, Period> const& timeoutDuration) const
    {
        return check_future_status(stdFuture_.wait_for(timeoutDuration));
    }

    template <typename Clock, typename Duration>
    future_status wait_until(std::chrono::time_point<Clock, Duration> const& deadline) const
    {
        return check_future_status(stdFuture_.wait_until(deadline));
    }

    template<typename F>
    using ResultOfFunc = typename std::result_of<F(Future<T, E>)>::type;

    template <typename F,
        typename std::enable_if<mdc::common::internal::IsResult<ResultOfFunc<F>>::value
        && !mdc::common::internal::IsVoidResult<ResultOfFunc<F>>::value>::type&>
    auto then(F&& func) -> mdc::common::Future<typename ResultOfFunc<F>::value_type,
        typename ResultOfFunc<F>::error_type>
    {
        using ResultType = ResultOfFunc<F>;
        auto promise =
            std::make_shared<mdc::common::Promise<typename ResultType::value_type, typename ResultType::error_type>>();
        auto future = promise->get_future();
        auto stateTemp = state_;
        auto self = std::make_shared<mdc::common::Future<T, E>>(std::move(*this));
        stateTemp->LockCallback();
        stateTemp->SetCallBack([promise, func = std::decay_t<F>(func), self] () mutable  {
            auto promiseTmp = promise;
            auto result = func(std::move(*(self.get())));
            if (result.HasValue()) {
                promiseTmp->set_value(result.Value());
            } else {
                promiseTmp->SetError(result.Error());
            }
        });

        if (self->is_ready()) {
            stateTemp->FireCallBack();
        }
        stateTemp->UnlockCallback();
        return future;
    }

    template <typename F,
        typename std::enable_if<mdc::common::internal::IsResult<ResultOfFunc<F>>::value
        && mdc::common::internal::IsVoidResult<ResultOfFunc<F>>::value>::type&>
    auto then(F&& func) -> mdc::common::Future<typename ResultOfFunc<F>::value_type,
        typename ResultOfFunc<F>::error_type>
    {
        using ResultType = ResultOfFunc<F>;
        auto promise =
            std::make_shared<mdc::common::Promise<typename ResultType::value_type, typename ResultType::error_type>>();
        auto future = promise->get_future();
        auto state = state_;
        auto self = std::make_shared<mdc::common::Future<T, E>>(std::move(*this));
        state->LockCallback();
        state->SetCallBack([promise, func = std::decay_t<F>(func), self] () mutable  {
            auto promiseTmp = promise;
            auto result = func(std::move(*(self.get())));
            if (result.HasValue()) {
                promiseTmp->set_value();
            } else {
                promiseTmp->SetError(result.Error());
            }
        });

        if (self->is_ready()) {
            state->FireCallBack();
        }
        state->UnlockCallback();
        return future;
    }

    template <typename F,
        typename std::enable_if<mdc::common::internal::IsFuture<ResultOfFunc<F>>::value
        && !mdc::common::internal::IsVoidFuture<ResultOfFunc<F>>::value>::type&>
    auto then(F&& func) -> ResultOfFunc<F>
    {
        using ResultType = ResultOfFunc<F>;
        auto promise =
            std::make_shared<mdc::common::Promise<typename ResultType::ValueType, typename ResultType::ErrorType>>();
        auto getFuture = promise->get_future();
        auto stateTmp = state_;
        auto self = std::make_shared<mdc::common::Future<T, E>>(std::move(*this));
        stateTmp->LockCallback();
        stateTmp->SetCallBack([promise, func = std::decay_t<F>(func), self] () mutable {
            auto promiseTmp = promise;
            auto futureTmp = func(std::move(*(self.get())));
            auto result = futureTmp.GetResult();
            if (result.HasValue()) {
                promiseTmp->set_value(result.Value());
            } else {
                promiseTmp->SetError(result.Error());
            }
        });
        if (self->is_ready()) {
            stateTmp->FireCallBack();
        }
        stateTmp->UnlockCallback();
        return getFuture;
    }

    template <typename F,
        typename std::enable_if<mdc::common::internal::IsFuture<ResultOfFunc<F>>::value
        && mdc::common::internal::IsVoidFuture<ResultOfFunc<F>>::value>::type&>
    auto then(F&& func) -> ResultOfFunc<F>
    {
        using ResultType = ResultOfFunc<F>;
        auto promise =
            std::make_shared<mdc::common::Promise<typename ResultType::ValueType, typename ResultType::ErrorType>>();
        auto future = promise->get_future();
        auto stateTmp = state_;
        auto self = std::make_shared<mdc::common::Future<T, E>>(std::move(*this));
        stateTmp->LockCallback();
        stateTmp->SetCallBack([promise, func = std::decay_t<F>(func), self] () mutable {
            auto promiseTmp = promise;
            auto futureTmp = func(std::move(*(self.get())));
            auto result = futureTmp.GetResult();
            if (result.HasValue()) {
                promiseTmp->set_value();
            } else {
                promiseTmp->SetError(result.Error());
            }
        });
        if (self->is_ready()) {
            stateTmp->FireCallBack();
        }
        stateTmp->UnlockCallback();
        return future;
    }

    template <typename F, typename std::enable_if<!mdc::common::internal::IsFuture<ResultOfFunc<F>>::value
        && !mdc::common::internal::IsResult<ResultOfFunc<F>>::value
        && !std::is_same<ResultOfFunc<F>, void>::value>::type&>
    auto then(F&& func) -> mdc::common::Future<ResultOfFunc<F>, E>
    {
        using ResultType = ResultOfFunc<F>;
        auto promisePtr = std::make_shared<mdc::common::Promise<ResultType, E>>();
        auto future = promisePtr->get_future();
        auto stateTmp = state_;
        auto self = std::make_shared<mdc::common::Future<T, E>>(std::move(*this));
        stateTmp->LockCallback();
        stateTmp->SetCallBack([promisePtr, func = std::decay_t<F>(func), self] () mutable {
            auto promiseTmp = promisePtr;
            auto future = std::move(*(self.get()));
            auto value = func(std::move(future));
            promiseTmp->set_value(value);
        });
        if (self->is_ready()) {
            stateTmp->FireCallBack();
        }
        stateTmp->UnlockCallback();
        return future;
    }

    template <typename F, typename std::enable_if<std::is_same<ResultOfFunc<F>, void>::value>::type&>
    auto then(F&& func) -> mdc::common::Future<ResultOfFunc<F>, E>
    {
        using ResultType = ResultOfFunc<F>;
        auto promise = std::make_shared<mdc::common::Promise<ResultType, E>>();
        auto future = promise->get_future();
        auto stateTmp = state_;
        auto self = std::make_shared<mdc::common::Future<T, E>>(std::move(*this));
        stateTmp->LockCallback();
        stateTmp->SetCallBack([promise, func = std::decay_t<F>(func), self] () mutable {
            auto promiseTmp = promise;
            func(std::move(*(self.get())));
            promiseTmp->set_value();
        });
        if (self->is_ready()) {
            stateTmp->FireCallBack();
        }
        stateTmp->UnlockCallback();
        return future;
    }

    bool is_ready() const
    {
        return std::future_status::ready == stdFuture_.wait_for(std::chrono::seconds::zero());
    }

private:
    Future(std::future<Result<T, E>>&& delegate_future, internal::State::Ptr extra_state)
        : stdFuture_(std::move(delegate_future)), state_(extra_state) {}

    future_status check_future_status(std::future_status const& status) const
    {
        switch (status) {
            case std::future_status::ready:
                return future_status::ready;
            case std::future_status::timeout:
                return future_status::timeout;
            default:
                std::cout << "[CORETYPE Future]: Error, Invlid future_status of Future, rerutn default value: timeout"
                    << std::endl;
                return future_status::timeout;
        }
    }

    std::mutex mutex_;
    UniLock lock_;

    std::future<Result<T, E>> stdFuture_;
    internal::State::Ptr state_;

    template <typename, typename>
    friend class Promise;
};
}
}

#endif
