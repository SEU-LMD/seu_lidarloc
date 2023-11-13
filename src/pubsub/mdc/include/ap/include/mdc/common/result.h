/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of Result class according to AutoSAR standard core type
 * Create: 2019-07-24
*/
#ifndef MDC_COMMON_RESULT_H
#define MDC_COMMON_RESULT_H
#include <memory>
#include "mdc/common/error_code.h"
#include "mdc/common/core_error_domain.h"

namespace mdc {
namespace common {
namespace internal {
template<typename T, typename... FormatArgs>
class IsFirstType {
    static constexpr bool value = true;
};

template<typename T, typename Head, typename... FormatArgs>
class IsFirstType<T, Head, FormatArgs...> {
public:
    static constexpr bool value = std::is_same<typename std::decay<T>::type, typename std::decay<Head>::type>::value;
};

template<typename T>
class IsFirstType<T> {
public:
    static constexpr bool value = false;
};
}
template <typename T, typename E = ErrorCode>
class Result final {
public:
    using value_type = T;
    using error_type = E;

    Result(T const &t): valueFlag_(true), value_(t), error_(GetDefaultErrorCode()) {}
    Result(T &&t): valueFlag_(true), value_(std::move(t)), error_(GetDefaultErrorCode()) {}
    explicit Result(E const &e): valueFlag_(false), value_(T()), error_(e){}
    explicit Result(E &&e): valueFlag_(false), value_(T()), error_(std::move(e)){}

    Result(Result const &other) = default;
    Result(Result &&other) noexcept(std::is_nothrow_move_constructible<T>::value &&
                                    std::is_nothrow_move_constructible<E>::value) = default;
    ~Result() = default;
    static Result FromValue(T const &t)
    {
        return Result(t);
    }

    static Result FromValue(T &&t)
    {
        return Result(std::move(t));
    }

    template <typename... FormatArgs,
        typename = typename std::enable_if<std::is_constructible<T, FormatArgs&&...>::value
            && !internal::IsFirstType<T, FormatArgs...>::value
            && !internal::IsFirstType<Result, FormatArgs...>::value>::type>
    static Result FromValue(FormatArgs &&...args)
    {
        return Result(T { std::forward<FormatArgs>(args)... });
    }

    static Result FromError(E const &e)
    {
        return Result(e);
    }

    static Result FromError(E &&e)
    {
        return Result(std::move(e));
    }

    template <typename... FormatArgs,
        typename = typename std::enable_if<std::is_constructible<E, FormatArgs...>::value
            && !internal::IsFirstType<E, FormatArgs...>::value
            && !internal::IsFirstType<Result, FormatArgs...>::value>::type>
    static Result FromError(FormatArgs &&...args)
    {
        return Result(E { std::forward<FormatArgs>(args)... });
    }

    Result& operator=(Result const &other) = default;
    Result& operator=(Result &&other) noexcept(std::is_nothrow_move_constructible<T>::value &&
                                               std::is_nothrow_move_assignable<T>::value &&
                                               std::is_nothrow_move_constructible<E>::value &&
                                               std::is_nothrow_move_assignable<E>::value)= default;

    template <typename... FormatArgs>
    void EmplaceValue(FormatArgs &&...args)
    {
        valueFlag_ = true;
        value_ = T(std::forward<FormatArgs>(args)...);
    }

    template <typename... FormatArgs>
    void EmplaceError(FormatArgs &&...args)
    {
        valueFlag_ = false;
        error_ = E(std::forward<FormatArgs>(args)...);
    }

    void Swap(Result &other) noexcept(std::is_nothrow_move_constructible<T>::value &&
                                      std::is_nothrow_move_assignable<T>::value &&
                                      std::is_nothrow_move_constructible<E>::value &&
                                      std::is_nothrow_move_assignable<E>::value)
    {
        std::swap(valueFlag_, other.valueFlag_);
        std::swap(error_, other.error_);
        std::swap(value_, other.value_);
    }

    bool HasValue() const noexcept
    {
        return valueFlag_;
    }

    explicit operator bool() const noexcept
    {
        return HasValue();
    }

    T const& operator*() const &
    {
        return Value();
    }

    T && operator*() &&
    {
        return std::move(*this).Value();
    }

    T const *operator->() const
    {
        return std::addressof(Value());
    }

    T const& Value() const &
    {
        return value_;
    }

    T&& Value() &&
    {
        if (HasValue()) {
            return std::move(value_);
        }

        // if no value return default value_type
        return std::move(value_type());
    }

    E const& Error() const &
    {
        return error_;
    }

    E&& Error() &&
    {
        if (!HasValue()) {
            return std::move(error_);
        }

        // if no error return default error_type
        return std::move(error_type());
    }

    template <typename U>
    T ValueOr(U &&defaultValue) const &
    {
        return HasValue() ? Value() : static_cast<T>(std::forward<U>(defaultValue));
    }

    template <typename U>
    T ValueOr(U &&defaultValue) &&
    {
        return HasValue() ? std::move(Value()) : static_cast<T>(std::forward<U>(defaultValue));
    }

    template <typename G>
    E ErrorOr(G &&defaultError) const
    {
        return HasValue() ? static_cast<E>(std::forward<G>(defaultError)) : Error();
    }

    template <typename G>
    bool CheckError(G &&checkError) const
    {
        return HasValue() ? false : (Error() == static_cast<E>(std::forward<G>(checkError)));
    }

    template <typename F>
    T Resolve(F &&f) const
    {
        return HasValue() ? Value() : std::forward<F>(f)(Error());
    }

    template <bool Condition, typename U = void>
    using enable_if_t = typename std::enable_if<Condition, U>::type;
    template <typename U>
    using result_of_t = typename std::result_of<U>::type;

    template <typename U>
    struct is_result : public std::false_type {
    };
    template <typename U, typename G>
    struct is_result<Result<U, G>> : public std::true_type {
    };
    template <typename F>
    using CallableReturnsResult = enable_if_t<is_result<result_of_t<F(T const&)>>::value>;
    template <typename F>
    using CallableReturnsNoResult = enable_if_t<!is_result<result_of_t<F(T const&)>>::value>;

    template <typename F, typename = CallableReturnsResult<F>>
    auto Bind(F &&f) const -> decltype(f(Value()))
    {
        using U = decltype(f(Value()));
        return HasValue() ? std::forward<F>(f)(Value()) : U(Error());
    }

    template <typename F, typename = CallableReturnsNoResult<F>>
    auto Bind(F &&f) const -> Result<decltype(f(Value())), E>
    {
        using U = decltype(f(Value()));
        using R = Result<U, E>;
        return HasValue() ? std::forward<F>(f)(Value()) : R(Error());
    }
private:
    template<typename EType = E>
    typename std::enable_if<std::is_same<EType, ErrorCode>::value, EType>::type GetDefaultErrorCode()
    {
        return MakeErrorCode(CoreErrc::kInvalidArgument, 0);
    }
    template<typename EType = E>
    typename std::enable_if<!std::is_same<EType, ErrorCode>::value, EType>::type GetDefaultErrorCode()
    {
        return error_type();
    }

    bool valueFlag_ = true;
    value_type value_;
    error_type error_;
};

template <typename E>
class Result<void, E> final {
public:
    using value_type = void; // [SWS_CORE_00811]
    using error_type = E; // [SWS_CORE_00812]

    Result() noexcept : valueFlag_(true), error_(GetDefaultErrorCode()) // [SWS_CORE_00821]
    {}

    explicit Result(E const &e) : valueFlag_(false), error_(e) // [SWS_CORE_00823]
    {}

    explicit Result(E &&e) : valueFlag_(false), error_(std::move(e)) // [SWS_CORE_00824]
    {}

    Result(Result const &other) = default; // [SWS_CORE_00825]

    Result(Result &&other) noexcept(std::is_nothrow_move_constructible<E>::value) = default; // [SWS_CORE_00826]

    ~Result() = default; // [SWS_CORE_00827]

    static Result FromValue() // [SWS_CORE_00831]
    {
        return Result();
    }

    static Result FromError(E const& e) // [SWS_CORE_00834]
    {
        return Result(e);
    }

    static Result FromError(E&& e) // [SWS_CORE_00835]
    {
        return Result(std::move(e));
    }

    template <typename... FormatArgs,
        typename = typename std::enable_if<std::is_constructible<E, FormatArgs&&...>::value
            && !internal::IsFirstType<E, FormatArgs...>::value
            && !internal::IsFirstType<Result, FormatArgs...>::value>::type>
    static Result FromError(FormatArgs &&...args) // [SWS_CORE_00836]
    {
        return Result(E { std::forward<FormatArgs>(args)... });
    }

    Result& operator=(Result const &other) = default; // [SWS_CORE_00841]

    // [SWS_CORE_00842]
    Result& operator=(Result &&other) noexcept(std::is_nothrow_move_constructible<E>::value &&
                                                std::is_nothrow_move_assignable<E>::value) = default;

    template <typename... FormatArgs>
    void EmplaceValue(FormatArgs &&...args) noexcept // [SWS_CORE_00843]
    {
        valueFlag_ = true;
    }

    template <typename... FormatArgs>
    void EmplaceError(FormatArgs &&...args) // [SWS_CORE_00844]
    {
        error_ = E(std::forward<FormatArgs>(args)...);
        valueFlag_ = false;
    }

    void Swap(Result &other) noexcept(std::is_nothrow_move_constructible<E>::value &&
                                       std::is_nothrow_move_assignable<E>::value) // [SWS_CORE_00845]
    {
        std::swap(error_, other.error_);
        std::swap(valueFlag_, other.valueFlag_);
    }

    bool HasValue() const noexcept // [SWS_CORE_00851]
    {
        return valueFlag_;
    }

    void operator*() const // [SWS_CORE_00853]
    {
    }

    explicit operator bool() const noexcept // [SWS_CORE_00852]
    {
        return HasValue();
    }

    void Value() const // [SWS_CORE_00855]
    {
        return;
    }

    E const& Error() const & // [SWS_CORE_00857]
    {
        return error_;
    }

    E&& Error() && // [SWS_CORE_00858]
    {
        if (!HasValue()) {
            return std::move(error_);
        }

        return std::move(error_type());
    }

    template <typename U>
    void ValueOr(U &&defaultValue) const // [SWS_CORE_00861]
    {
        Value();
    }

    template <typename G>
    E ErrorOr(G &&defaultError) const // [SWS_CORE_00863]
    {
        return HasValue() ? static_cast<E>(std::forward<G>(defaultError)) : Error();
    }

    template <typename G>
    bool CheckError(G &&error) const // [SWS_CORE_00865]
    {
        return HasValue() ? false : (Error() == static_cast<E>(std::forward<G>(error)));
    }
    template <typename F>
    void Resolve(F &&f) const
    {
        if (!HasValue()) {
            std::forward<F>(f)(Error());
        }
    }

private:
    template<typename EType = E>
    typename std::enable_if<std::is_same<EType, ErrorCode>::value, EType>::type GetDefaultErrorCode()
    {
        return MakeErrorCode(CoreErrc::kInvalidArgument, 0);
    }
    template<typename EType = E>
    typename std::enable_if<!std::is_same<EType, ErrorCode>::value, EType>::type GetDefaultErrorCode()
    {
        return error_type();
    }

    bool valueFlag_ = true;
    error_type error_;
};

template <typename T, typename E>
bool operator==(Result<T, E> const &lhs, Result<T, E> const& rhs)
{
    bool hasValue = lhs.HasValue() && rhs.HasValue() && (lhs.Value() == rhs.Value());
    bool hasError = (!lhs.HasValue()) && (!rhs.HasValue()) && (lhs.Error() == rhs.Error());
    return hasError || hasValue;
}

template <typename E>
bool operator==(Result<void, E> const &lhs, Result<void, E> const& rhs)
{
    bool hasValue = lhs.HasValue() && rhs.HasValue();
    bool hasError = (!lhs.HasValue()) && (!rhs.HasValue()) && (lhs.Error() == rhs.Error());
    return hasError || hasValue;
}


template <typename T, typename E>
bool operator!=(Result<T, E> const &lhs, Result<T, E> const& rhs)
{
    return !(lhs == rhs);
}

template <typename T, typename E>
bool operator==(Result<T, E> const &lhs, T const &rhs)
{
    return lhs.HasValue() && (lhs.Value() == rhs);
}

template <typename T, typename E>
bool operator==(T const &lhs, Result<T, E> const &rhs)
{
    return rhs == lhs;
}

template <typename T, typename E>
bool operator!=(Result<T, E> const &lhs, T const &rhs)
{
    return !(lhs == rhs);
}

template <typename T, typename E>
bool operator!=(T const &lhs, Result<T, E> const &rhs)
{
    return !(lhs == rhs);
}

template <typename T, typename E>
bool operator==(Result<T, E> const &lhs, E const &rhs)
{
    return (!lhs.HasValue()) && (lhs.Error() == rhs);
}

template <typename T, typename E>
bool operator==(E const &lhs, Result<T, E> const &rhs)
{
    return rhs == lhs;
}

template <typename T, typename E>
bool operator!=(Result<T, E> const &lhs, E const &rhs)
{
    return !(lhs == rhs);
}

template <typename T, typename E>
bool operator!=(E const &lhs, Result<T, E> const &rhs)
{
    return !(lhs == rhs);
}

template <typename T, typename E>
void swap(Result< T, E > &lhs, Result< T, E > &rhs) noexcept(noexcept(lhs.Swap(rhs)))
{
    lhs.Swap(rhs);
}
}
}

#endif
