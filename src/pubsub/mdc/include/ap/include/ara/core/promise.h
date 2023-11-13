/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of Promise class according to AutoSAR standard core type
 * Create: 2019-07-30
 */
#ifndef ARA_CORE_PROMISE_H
#define ARA_CORE_PROMISE_H

#include "ara/core/error_code.h"
#include "ara/core/result.h"
#include "ara/core/future.h"

#include <exception>
#include <future>
#include <mutex>
#include <system_error>

namespace ara {
namespace core {
/**
 * @brief ara::core specific variant of std::promise class [SWS_CORE_00340].
 *
 * @tparam T             the type of value
 * @tparam ErrorCode     the type of error
 */
template <typename T, typename E = ErrorCode> class Promise {
    using UniLock = std::unique_lock<std::mutex>;

public:
    /* *
     * @brief Construct a new Promise object [SWS_CORE_00341].
     *
     */
    Promise() : state_(std::make_shared<internal::State>()) {}

    /* *
     * @brief Move constructor [SWS_CORE_00342].
     *
     * @param other  the other instance
     */
    Promise(Promise &&other) noexcept
        : lock_(other.mutex_), stdPromise_(std::move(other.stdPromise_)), state_(std::move(other.state_))
    {
        lock_.unlock();
    }

    /* *
     * @briefCopy constructor shall be disabled [SWS_CORE_00350].
     *
     */
    Promise(Promise const &) = delete;

    /* *
     * @brief Destroy the Promise object [SWS_CORE_00349].
     *
     */
    ~Promise()
    {
        if (state_ != nullptr) {
            state_->SetPromiseDeconstructionThreadId(std::this_thread::get_id());
            state_->LockCallback();
            state_->SetCallBack(nullptr);
            state_->UnlockCallback();
        }
    }

    /* *
     * @brief Copy assignment operator shall be disabled [SWS_CORE_00351].
     *
     */
    Promise &operator = (Promise const &) = delete;

    /* *
     * @brief Move assignment[SWS_CORE_00343].
     *
     * @param other       the other instance
     * @return Promise&    *this
     */
    Promise &operator = (Promise &&other) noexcept
    {
        if (&other != this) {
            UniLock lhsLock(mutex_, std::defer_lock);
            UniLock rhsLock(other.mutex_, std::defer_lock);
            std::lock(lhsLock, rhsLock);

            stdPromise_ = std::move(other.stdPromise_);
            state_ = std::move(other.state_);
        }
        return *this;
    }

    /* *
     * @brief Swap the contents of this instance with another one’s [SWS_CORE_00352].
     *
     * @param other the other instance
     */
    void swap(Promise &other) noexcept
    {
        UniLock lhsLock(mutex_, std::defer_lock);
        UniLock rhsLock(other.mutex_, std::defer_lock);
        std::lock(lhsLock, rhsLock);

        std::swap(stdPromise_, other.stdPromise_);
        std::swap(state_, other.state_);
    }

    /* *
     * @brief Return the associated Future [SWS_CORE_00344].
     *
     * @return Future<T, E> a Future
     */
    Future<T, E> get_future()
    {
        return Future<T, E>(stdPromise_.get_future(), state_);
    }

    /* *
     * @brief Move an error into the shared state and make the state ready [SWS_CORE_00353].
     *
     * @param[in] error the error to store
     */
    void SetError(E &&error)
    {
        UniLock lock(mutex_);

        state_->LockCallback();
        stdPromise_.set_value(Result<T, E>::FromError(std::move(error)));
        state_->FireCallBack();
        state_->UnlockCallback();
    }

    /* *
     * @brief Copy an error into the shared state and make the state ready [SWS_CORE_00354].
     *
     * @param[in] error   the error to store
     */
    void SetError(E const & error)
    {
        UniLock lock(mutex_);

        state_->LockCallback();
        stdPromise_.set_value(Result<T, E>::FromError(error));
        state_->FireCallBack();
        state_->UnlockCallback();
    }

    /* *
     * @brief Move a value into the shared state and make the state ready [SWS_CORE_00346].
     *
     * @param[in] value the value to store
     */
    void set_value(T &&value)
    {
        UniLock lock(mutex_);
        state_->LockCallback();
        stdPromise_.set_value(std::move(value));
        state_->FireCallBack();
        state_->UnlockCallback();
    }

    /* *
     * @brief Copy a value into the shared state and make the state ready [SWS_CORE_00345].
     *
     * @param[in] value the value to store
     */
    void set_value(T const & value)
    {
        UniLock lock(mutex_);
        state_->LockCallback();
        stdPromise_.set_value(value);
        state_->FireCallBack();
        state_->UnlockCallback();
    }

private:
    std::mutex mutex_;
    UniLock lock_;

    std::promise<Result<T, E>> stdPromise_;
    internal::State::Ptr state_;
};

/**
 * @brief Specialization of class Promise for "void" values [SWS_CORE_06340].
 *
 * @tparam E the type of error
 */
template <typename E> class Promise<void, E> final {
    using UniLockSpec = std::unique_lock<std::mutex>;

public:
    /* *
     * @brief Construct a new Promise object with void type [SWS_CORE_06341]
     *
     */
    Promise() : state_(std::make_shared<internal::State>()) {}
    /* *
     * @brief Destructor for Promise objects with void type [SWS_CORE_06349].
     *
     */
    ~Promise()
    {
        if (state_ != nullptr) {
            state_->SetPromiseDeconstructionThreadId(std::this_thread::get_id());
            state_->LockCallback();
            state_->SetCallBack(nullptr);
            state_->UnlockCallback();
        }
    }
    /* *
     * @brief Copy constructor shall be disabled with void type [SWS_CORE_06350]
     *
     */
    Promise(Promise const &) = delete;

    /* *
     * @brief Copy assignment operator shall be disabled [SWS_CORE_06351].
     *
     */
    Promise &operator = (Promise const &) = delete;

    /* *
     * @brief Move constructor [SWS_CORE_06342].
     *
     * @param[in] other the other instance
     */
    Promise(Promise &&other) noexcept
        : lock_(other.mutex_), stdPromiseSpec_(std::move(other.stdPromiseSpec_)), state_(std::move(other.state_))
    {
        lock_.unlock();
    }

    /* *
     * @brief Move assignment [SWS_CORE_06343].
     *
     * @param[in] other        the other instance
     * @return Promise&        *this
     */
    Promise &operator = (Promise &&other) noexcept
    {
        if (&other != this) {
            UniLockSpec lhsLock(mutex_, std::defer_lock);
            UniLockSpec rhsLock(other.mutex_, std::defer_lock);
            std::lock(lhsLock, rhsLock);

            stdPromiseSpec_ = std::move(other.stdPromiseSpec_);
            state_ = std::move(other.state_);
        }
        return *this;
    }

    /* *
     * @brief Swap the contents of this instance with another one’s [SWS_CORE_06352].
     *
     * @param[in]   other    the other instance
     */
    void swap(Promise &other) noexcept
    {
        UniLockSpec lhsLock(mutex_, std::defer_lock);
        UniLockSpec rhsLock(other.mutex_, std::defer_lock);
        std::lock(lhsLock, rhsLock);

        std::swap(stdPromiseSpec_, other.stdPromiseSpec_);
        std::swap(state_, other.state_);
    }

    /* *
     * @brief Get the future object [SWS_CORE_06344]
     *
     * @return Future<void, E> a Future
     */
    Future<void, E> get_future()
    {
        return Future<void, E>(stdPromiseSpec_.get_future(), state_);
    }

    /* *
     * @brief Move an error into the shared state and make the state ready [SWS_CORE_06353].
     *
     * @param[in] err  the error to store
     */
    void SetError(E &&err)
    {
        Result<void, E> r = Result<void, E>::FromError(std::move(err));
        UniLockSpec lock(mutex_);
        state_->LockCallback();
        stdPromiseSpec_.set_value(r);
        state_->FireCallBack();
        state_->UnlockCallback();
    }

    /* *
     * @brief Copy an error into the shared state and make the state ready [SWS_CORE_06354].
     *
     * @param[in] err the error to store
     */
    void SetError(E const & err)
    {
        Result<void, E> r = Result<void, E>::FromError(err);
        UniLockSpec lock(mutex_);
        state_->LockCallback();
        stdPromiseSpec_.set_value(r);
        state_->FireCallBack();
        state_->UnlockCallback();
    }

    /* *
     * @brief Make the shared state ready [SWS_CORE_06345].
     *
     */
    void set_value()
    {
        UniLockSpec lock(mutex_);
        state_->LockCallback();
        stdPromiseSpec_.set_value(Result<void, E>::FromValue());
        state_->FireCallBack();
        state_->UnlockCallback();
    }

private:
    std::mutex mutex_;
    UniLockSpec lock_;

    std::promise<Result<void, E>> stdPromiseSpec_;
    internal::State::Ptr state_;
};
} // namespace core
} // namespace ara

#endif
