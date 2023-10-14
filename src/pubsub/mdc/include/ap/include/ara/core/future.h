/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of Future class according to AutoSAR standard core type
 * Create: 2019-07-30
 */
#ifndef ARA_CORE_FUTURE_H
#define ARA_CORE_FUTURE_H

#include "ara/core/future_error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/internal/type_check.h"
#include "ara/core/result.h"
#include "ara/core/exception.h"
#include "ara/core/core_error_domain.h"

#include <chrono>
#include <future>
#include <system_error>

namespace ara {
namespace core {
namespace internal {
class State {
public:
    using Ptr = std::shared_ptr<State>;

    void FireCallBack()
    {
        threadId_ = std::this_thread::get_id();
        if (static_cast<bool>(callback_)) {
            callback_();
        }
    }

    template <typename F> void SetCallBack(F &&callback) noexcept
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

    std::thread::id GetPromiseDecontructionThreadId() const noexcept
    {
        return promiseDeconstructionThreadId_;
    }

    void SetPromiseDeconstructionThreadId(const std::thread::id &id) noexcept
    {
        promiseDeconstructionThreadId_ = id;
    }

private:
    std::function<void(void)> callback_;
    std::mutex mutex_;
    std::thread::id threadId_; // Fire callback thread id
    std::thread::id promiseDeconstructionThreadId_;
};
}


template <typename, typename> class Promise;

enum class future_status : uint8_t {
    ready = 1,
    timeout,
};

inline std::ostream &operator << (std::ostream &out, FutureException const & ex)
{
    return (out << "FutureException: " << ex.Error() << " (" << ex.what() << ")");
}

/**
 * @brief Provides ara::core specific Future operations to collect the results of an asynchronous call[SWS_CORE_00321].
 *
 * @tparam T the type of values
 * @tparam ErrorCode the type of errors
 */
template <typename T, typename E = ErrorCode> class Future final {
    using UniLock = std::unique_lock<std::mutex>;

public:
    using ValueType = T;
    using ErrorType = E;
    // / Alias type for the Promise type collaborating with this Future type
    using PromiseType = Promise<T, E>;
    // / Alias type for the future_status type
    using Status = future_status;
    Future() noexcept = default;

    ~Future()
    {
        UniLock lock(mutex_);
        if (state_ && (std::this_thread::get_id() != state_->GetCallbackThreadId()) &&
            (std::this_thread::get_id() != state_->GetPromiseDecontructionThreadId())) {
            state_->LockCallback();
            state_->SetCallBack(nullptr);
            state_->UnlockCallback();
        }
    }

    Future(Future const &) = delete;
    Future &operator = (Future const &) = delete;

    Future(Future &&other) noexcept
        : lock_(other.mutex_), stdFuture_(std::move(other.stdFuture_)), state_(std::move(other.state_))
    {
        lock_.unlock();
    }

    Future &operator = (Future &&other) noexcept
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
#ifndef NOT_SUPPORT_EXCEPTIONS
        try {
            return stdFuture_.get();
        } catch (std::future_error const & futureError) {
            std::error_code const & errorCode = futureError.code();
            future_errc err;
            if (errorCode == std::future_errc::broken_promise) {
                err = future_errc::broken_promise;
            } else if (errorCode == std::future_errc::future_already_retrieved) {
                err = future_errc::future_already_retrieved;
            } else if (errorCode == std::future_errc::promise_already_satisfied) {
                err = future_errc::promise_already_satisfied;
            } else if (errorCode == std::future_errc::no_state) {
                err = future_errc::no_state;
            } else {
                return Result<T, E>::FromError(CoreErrc::kInvalidArgument);
            }
            Result<T, E> r = Result<T, E>::FromError(err);
            return r;
        }
#else
        // Because std::future doesn't support exception check, NoException mode isn't supported in GetResult.
        return stdFuture_.get();
#endif
    }

#ifndef NOT_SUPPORT_EXCEPTIONS
    T get()
    {
        auto result = GetResult();
        if (result.HasValue()) {
            return result.Value();
        } else {
            throw FutureException(result.Error());
        }
    }
#endif

    bool valid() const noexcept
    {
        return stdFuture_.valid();
    }

    void wait() const
    {
        stdFuture_.wait();
    }

    template <typename Rep, typename Period>
    future_status wait_for(std::chrono::duration<Rep, Period> const & timeoutDuration) const
    {
        return check_future_status(stdFuture_.wait_for(timeoutDuration));
    }

    template <typename Clock, typename Duration>
    future_status wait_until(std::chrono::time_point<Clock, Duration> const & deadline) const
    {
        return check_future_status(stdFuture_.wait_until(deadline));
    }

    template <typename F> using ResultOfFunc = typename std::result_of<F(Future<T, E>)>::type;

    template <typename F,
        typename std::enable_if<ara::core::internal::IsResult<ResultOfFunc<F>>::value>::type * = nullptr>
    auto then(F &&func) -> ara::core::Future<typename ResultOfFunc<F>::value_type, typename ResultOfFunc<F>::error_type>
    {
        using ResultType = ResultOfFunc<F>;
        auto promise =
            std::make_shared<ara::core::Promise<typename ResultType::value_type, typename ResultType::error_type>>();
        auto future = promise->get_future();
        auto stateTmp = state_;
        auto self = std::make_shared<ara::core::Future<T, E>>(std::move(*this));
        stateTmp->LockCallback();
        SetCallbackForResult(promise, std::forward<F>(func), stateTmp, self);
        if (self->is_ready()) {
            stateTmp->FireCallBack();
        }
        stateTmp->UnlockCallback();
        return future;
    }

    template <typename F,
        typename std::enable_if<ara::core::internal::IsFuture<ResultOfFunc<F>>::value>::type * = nullptr>
    auto then(F &&func) -> ResultOfFunc<F>
    {
        using ResultType = ResultOfFunc<F>;
        auto promise =
            std::make_shared<ara::core::Promise<typename ResultType::ValueType, typename ResultType::ErrorType>>();
        auto future = promise->get_future();
        auto stateTmp = state_;
        auto self = std::make_shared<ara::core::Future<T, E>>(std::move(*this));
        stateTmp->LockCallback();
        SetCallbackForFuture(promise, std::forward<F>(func), stateTmp, self);
        if (self->is_ready()) {
            stateTmp->FireCallBack();
        }
        stateTmp->UnlockCallback();
        return future;
    }

    template <typename F, typename std::enable_if<!ara::core::internal::IsFuture<ResultOfFunc<F>>::value &&
        !ara::core::internal::IsResult<ResultOfFunc<F>>::value>::type * = nullptr>
    auto then(F &&func) -> ara::core::Future<ResultOfFunc<F>, E>
    {
        using ResultType = ResultOfFunc<F>;
        auto promise = std::make_shared<ara::core::Promise<ResultType, E>>();
        auto future = promise->get_future();
        auto stateTmp = state_;
        auto self = std::make_shared<ara::core::Future<T, E>>(std::move(*this));
        stateTmp->LockCallback();
        SetCallbackForPlainType(promise, std::forward<F>(func), stateTmp, self);
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
    Future(std::future<Result<T, E>> &&delegate_future, internal::State::Ptr extra_state)
        : stdFuture_(std::move(delegate_future)), state_(extra_state)
    {}

    future_status check_future_status(std::future_status const & status) const
    {
        switch (status) {
            case std::future_status::ready:
                return future_status::ready;
            case std::future_status::timeout:
                return future_status::timeout;
            default:
                std::cout <<
                    "[CORETYPE Future]: Error, Invlid future_status of Future, rerutn default value: timeout" <<
                    std::endl;
                return future_status::timeout;
        }
    }

    template <typename F, typename PromiseErrorType>
    void SetCallbackForResult(std::shared_ptr<ara::core::Promise<void, PromiseErrorType>> const & promise, F &&func,
        internal::State::Ptr stateTmp, std::shared_ptr<ara::core::Future<T, E>> self)
    {
        stateTmp->SetCallBack([promise, func = std::decay_t<F>(func), self]() mutable {
            auto promiseTmp = promise;
            auto result = func(std::move(*(self.get())));
            if (result.HasValue()) {
                promiseTmp->set_value();
            } else {
                promiseTmp->SetError(result.Error());
            }
        });
    }

    template <typename F, typename PromiseValueType, typename PromiseErrorType>
    void SetCallbackForResult(std::shared_ptr<ara::core::Promise<PromiseValueType, PromiseErrorType>> const & promise,
        F &&func, internal::State::Ptr const & stateTmp, std::shared_ptr<ara::core::Future<T, E>> const & self)
    {
        stateTmp->SetCallBack([promise, func = std::decay_t<F>(func), self]() mutable {
            auto promiseTmp = promise;
            auto result = func(std::move(*(self.get())));
            if (result.HasValue()) {
                promiseTmp->set_value(result.Value());
            } else {
                promiseTmp->SetError(result.Error());
            }
        });
    }

    template <typename F, typename PromiseErrorType>
    void SetCallbackForFuture(std::shared_ptr<ara::core::Promise<void, PromiseErrorType>> const & promise, F &&func,
        internal::State::Ptr const & stateTmp, std::shared_ptr<ara::core::Future<T, E>> const & self)
    {
        stateTmp->SetCallBack([promise, func = std::decay_t<F>(func), self]() mutable {
            auto promiseTmp = promise;
            auto futureTmp = func(std::move(*(self.get())));
            auto result = futureTmp.GetResult();
            if (result.HasValue()) {
                promiseTmp->set_value();
            } else {
                promiseTmp->SetError(result.Error());
            }
        });
    }

    template <typename F, typename PromiseValueType, typename PromiseErrorType>
    void SetCallbackForFuture(std::shared_ptr<ara::core::Promise<PromiseValueType, PromiseErrorType>> const & promise,
        F &&func, internal::State::Ptr const & stateTmp, std::shared_ptr<ara::core::Future<T, E>> const & self)
    {
        stateTmp->SetCallBack([promise, func = std::decay_t<F>(func), self]() mutable {
            auto promiseTmp = promise;
            auto futureTmp = func(std::move(*(self.get())));
            auto result = futureTmp.GetResult();
            if (result.HasValue()) {
                promiseTmp->set_value(result.Value());
            } else {
                promiseTmp->SetError(result.Error());
            }
        });
    }

    template <typename F, typename PromiseErrorType>
    void SetCallbackForPlainType(std::shared_ptr<ara::core::Promise<void, PromiseErrorType>> const & promise, F &&func,
        internal::State::Ptr stateTmp, std::shared_ptr<ara::core::Future<T, E>> const & self)
    {
        stateTmp->SetCallBack([promise, func = std::decay_t<F>(func), self]() mutable {
            auto promiseTmp = promise;
            func(std::move(*(self.get())));
            promiseTmp->set_value();
        });
    }

    template <typename F, typename PromiseValueType, typename PromiseErrorType>
    void SetCallbackForPlainType(
        std::shared_ptr<ara::core::Promise<PromiseValueType, PromiseErrorType>> const & promise, F &&func,
        internal::State::Ptr stateTmp, std::shared_ptr<ara::core::Future<T, E>> const & self)
    {
        stateTmp->SetCallBack([promise, func = std::decay_t<F>(func), self]() mutable {
            auto promiseTmp = promise;
            auto future = std::move(*(self.get()));
            auto value = func(std::move(future));
            promiseTmp->set_value(value);
        });
    }

    std::mutex mutex_;
    UniLock lock_;

    std::future<Result<T, E>> stdFuture_;
    internal::State::Ptr state_;

    template <typename, typename> friend class Promise;
};
}
}

#endif
