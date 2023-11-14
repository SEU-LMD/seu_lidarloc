/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: the implementation of Promise class according to AutoSAR standard core type
 * Create: 2021-01-11
 */
#ifndef ARA_CORE_OPTIONAL_H
#define ARA_CORE_OPTIONAL_H
#include <functional>
#include <initializer_list>
#include <memory>
#include <type_traits>

#include "ara/core/utility.h"

namespace ara {
namespace core {
/**
 * @brief ara::core specific variant of std::optional class.
 *
 * @tparam T             the type of value
 */
template <class T>
class Optional;

struct nullopt_t {
    constexpr explicit nullopt_t(int) {}
};
constexpr nullopt_t nullopt {0};

namespace {
template <typename T>
struct is_optional : std::false_type {
};
template <typename T>
struct is_optional<Optional<T>> : std::true_type {
};

template <typename T> using EnableIfCopyConstructible
    = typename std::enable_if<std::is_copy_constructible<T>::value>::type;

template <typename T> using EnableIfMoveConstructible
    = typename std::enable_if<std::is_move_constructible<T>::value>::type;

template <typename T, typename... Args> using EnableIfConstructible
    = typename std::enable_if<std::is_constructible<T, Args...>::value>::type;

template <typename U, typename T> using EnableIfConvertible
    = typename std::enable_if<std::is_convertible<U, T>::value>::type;

template <typename U, typename T> using NotEnableIfConvertible
    = typename std::enable_if<!std::is_convertible<U, T>::value>::type;

template <typename T> using EnableIfNotOptional
    = typename std::enable_if<!is_optional<typename std::decay<T>::type>::value>::type;

#if __cplusplus >= 201703L
template <typename T> using EnableIfSwappable
    = typename std::enable_if<std::is_swappable<T>::value>::type;
#endif

template <bool E, class T = void>
using EnableIfT = typename std::enable_if<E, T>::type;

template <class T, class U, class O>
using ConvertsFromOther = EnableIfT<
    std::is_constructible<T, O>::value &&
    !std::is_constructible<T, Optional<U> &>::value &&
    !std::is_constructible<T, Optional<U> &&>::value &&
    !std::is_constructible<T, Optional<U> const &>::value &&
    !std::is_constructible<T, Optional<U> const &&>::value &&
    !std::is_convertible<Optional<U> &, T>::value &&
    !std::is_convertible<Optional<U> &&, T>::value &&
    !std::is_convertible<Optional<U> const &, T>::value &&
    !std::is_convertible<Optional<U> const &&, T>::value>;

template <class T, class U, class O>
using AssignsFromOther = EnableIfT<
    std::is_constructible<T, O>::value &&
    std::is_assignable<T&, O>::value &&
    !std::is_constructible<T, Optional<U>&>::value &&
    !std::is_constructible<T, Optional<U> &&>::value &&
    !std::is_constructible<T, Optional<U> const &>::value &&
    !std::is_constructible<T, Optional<U> const &&>::value &&
    !std::is_convertible<Optional<U>&, T>::value &&
    !std::is_convertible<Optional<U> &&, T>::value &&
    !std::is_convertible<Optional<U> const &, T>::value &&
    !std::is_convertible<Optional<U> const &&, T>::value &&
    !std::is_assignable<T&, Optional<U>&>::value &&
    !std::is_assignable<T&, Optional<U> &&>::value &&
    !std::is_assignable<T&, Optional<U> const &>::value &&
    !std::is_assignable<T&, Optional<U> const &&>::value>;
}

template <class T>
class Optional {
public:

    using value_type = T;

    Optional() noexcept : opt_(false), val_(nullptr)
    {
    }

    Optional(nullopt_t) noexcept : opt_(false), val_(nullptr)
    {
    }

    Optional(Optional<T> const & rhs) : opt_(rhs.opt_), val_(opt_ ? std::make_unique<T>(*rhs.val_) : nullptr)
    {
    }

    template <typename T_ = T, typename = EnableIfMoveConstructible<T_>>
    Optional(Optional<T> && rhs) noexcept(std::is_nothrow_move_constructible<T_>())
        : opt_(rhs.opt_), val_(std::move(rhs.val_))
    {
    }

    template <typename... Args, typename = EnableIfConstructible<T, Args &&...>>
    explicit Optional(in_place_t, Args &&... args)
        : opt_(true), val_(std::make_unique<T>(std::forward<Args>(args)...))
    {
    }

    template <typename U, typename... Args,
        typename = EnableIfConstructible<T, std::initializer_list<U>, Args &&...>>
    explicit Optional(in_place_t, std::initializer_list<U> il, Args &&... args)
        : opt_(true), val_(std::make_unique<T>(il, std::forward<Args>(args)...))
    {
    }

    template <typename U = T,
        typename = EnableIfConstructible<T, U &&>,
        typename = typename std::enable_if<!std::is_same<typename std::decay<U>::type, in_place_t>::value>::type,
        typename = typename std::enable_if<!std::is_same<Optional<T>, typename std::decay<U>::type>::value>::type>
    Optional(U && v) : opt_(true), val_(std::make_unique<T>(std::forward<U>(v)))
    {
    }

    template <typename U = T,
        typename = EnableIfConstructible<T, U &&>,
        typename = typename std::enable_if<!std::is_same<typename std::decay<U>::type, in_place_t>::value>::type,
        typename = typename std::enable_if<!std::is_same<Optional<T>, typename std::decay<U>::type>::value>::type,
        typename = NotEnableIfConvertible<U &&, T>>
    explicit Optional(U && v) : opt_(true), val_(std::make_unique<T>(std::forward<U>(v)))
    {
    }

    template <class U,
        typename = ConvertsFromOther<T, U, U const &>>
    Optional(Optional<U> const & rhs) : opt_(static_cast<bool>(rhs)), val_(opt_ ? std::make_unique<T>(*rhs) : nullptr)
    {
    }

    template <class U,
        typename = ConvertsFromOther<T, U, U const &>,
        typename = NotEnableIfConvertible<U const &, T>>
    explicit Optional(Optional<U> const & rhs)
        : opt_(static_cast<bool>(rhs)), val_(opt_ ? std::make_unique<T>(*rhs) : nullptr)
    {
    }

    template <class U,
        typename = ConvertsFromOther<T, U, U &&>>
    Optional(Optional<U> && rhs)
    {
        if (rhs) {
            static_cast<void>(emplace(*rhs));
        } else {
            reset();
        }
    }

    template <class U,
        typename = ConvertsFromOther<T, U, U &&>,
        typename = NotEnableIfConvertible<U &&, T>>
    explicit Optional(Optional<U> && rhs)
    {
        if (rhs) {
            static_cast<void>(emplace(*rhs));
        } else {
            reset();
        }
    }

    ~Optional()
    {
        reset();
    }

    Optional<T>& operator=(nullopt_t) noexcept
    {
        reset();
        return *this;
    }

    Optional<T>& operator=(Optional<T> const & rhs)
    {
        reset();
        opt_ = rhs.opt_;
        val_.reset(rhs.opt_ ? new T(*rhs.val_) : nullptr);
        return *this;
    }

    Optional<T>& operator=(Optional<T> && rhs) noexcept(
        std::is_nothrow_move_assignable<T>::value && std::is_nothrow_move_constructible<T>::value)
    {
        reset();
        opt_ = rhs.opt_;
        val_.reset(rhs.opt_ ? new T(std::move(*rhs.val_)) : nullptr);
        return *this;
    }

    template <typename U, typename = EnableIfNotOptional<U>>
    Optional<T>& operator=(U && rhs)
    {
        opt_ = true;
        val_.reset(new T(std::move(rhs)));
        return *this;
    }

    template <class U,
        typename = AssignsFromOther<T, U, U const &>>
    Optional<T>& operator=(Optional<U> const & rhs)
    {
        opt_ = static_cast<bool>(rhs);
        val_.reset(opt_ ? new T(*rhs) : nullptr);
        return *this;
    }

    template <class U,
        typename = AssignsFromOther<T, U, U>>
    Optional<T>& operator=(Optional<U> && rhs)
    {
        if (rhs) {
            static_cast<void>(emplace(*rhs));
        } else {
            reset();
        }
        return *this;
    }

    template <class... Args, typename T_ = T, typename = EnableIfConstructible<T_, Args &&...>>
    T& emplace(Args &&... args)
    {
        reset();
        val_.reset(new T(std::forward<Args>(args)...));
        if (val_ != nullptr) {
            opt_ = true;
        } else {
            opt_ = false;
        }
        return *val_;
    }

    template <class U, class... Args, typename T_ = T,
        typename = EnableIfConstructible<T_, std::initializer_list<U>, Args &&...>>
    T& emplace(std::initializer_list<U> il, Args &&... args)
    {
        reset();
        val_.reset(new T(il, std::forward<Args>(args)...));
        if (val_ != nullptr) {
            opt_ = true;
        } else {
            opt_ = false;
        }
        return *val_;
    }

    template <typename T_ = T, typename = EnableIfMoveConstructible<T_>>
    void swap(Optional<T>& rhs) noexcept (
        std::is_nothrow_move_constructible<T>::value && noexcept(std::swap(std::declval<T&>(), std::declval<T&>())))
    {
        std::swap(this->val_, rhs.val_);
        std::swap(opt_, rhs.opt_);
    }

    constexpr const T* operator->() const
    {
        return val_.get();
    }

    constexpr T* operator->()
    {
        return val_.get();
    }

    constexpr const T& operator*() const&
    {
        return *val_;
    }

    constexpr T& operator*() &
    {
        return *val_;
    }

    constexpr T && operator*() &&
    {
        return std::move(*val_);
    }

    constexpr const T && operator*() const&&
    {
        return std::move(*val_);
    }

    constexpr explicit operator bool() const noexcept
    {
        return opt_;
    }

    constexpr bool has_value() const noexcept
    {
        return opt_;
    }

    template <typename U, typename T_ = T, typename = EnableIfCopyConstructible<T_>,
        typename = EnableIfConvertible<U &&, T_>>
    constexpr T value_or(U && v) const&
    {
        return static_cast<bool>(*this) ? **this : static_cast<T>(std::forward<U>(v));
    }

    template <class U, typename T_ = T, typename = EnableIfMoveConstructible<T_>,
        typename = EnableIfConvertible<U &&, T_>>
    constexpr T value_or(U && v) &&
    {
        return static_cast<bool>(*this) ? std::move(**this) : static_cast<T>(std::forward<U>(v));
    }

    void reset() noexcept
    {
        opt_ = false;
        val_.reset(nullptr);
    }

private:
    bool opt_;
    std::unique_ptr<T> val_;
};

#if __cplusplus >= 201703L
template <class T> Optional(T)->Optional<T>;
#endif

template <typename T, typename U>
constexpr bool operator==(Optional<T> const & x, Optional<U> const & y)
{
    if (static_cast<bool>(x) != static_cast<bool>(y)) {
        return false;
    }
    return static_cast<bool>(x) == false ? true : (*x == *y);
}

template <typename T, typename U>
constexpr bool operator!=(Optional<T> const & x, Optional<U> const & y)
{
    if (static_cast<bool>(x) != static_cast<bool>(y)) {
        return true;
    }
    return static_cast<bool>(x) == false ? false : (*x != *y);
}

template <typename T, typename U>
constexpr bool operator<(Optional<T> const & x, Optional<U> const & y)
{
    if (static_cast<bool>(y) == false) {
        return false;
    }
    return static_cast<bool>(x) == false ? true : (*x < *y);
}

template <typename T, typename U>
constexpr bool operator>(Optional<T> const & x, Optional<U> const & y)
{
    if (static_cast<bool>(x) == false) {
        return false;
    }
    return static_cast<bool>(y) == false ? true : (*x > *y);
}

template <typename T, typename U>
constexpr bool operator<=(Optional<T> const & x, Optional<U> const & y)
{
    if (static_cast<bool>(x) == false) {
        return true;
    }
    return static_cast<bool>(y) == false ? false : (*x <= *y);
}

template <typename T, typename U>
constexpr bool operator>=(Optional<T> const & x, Optional<U> const & y)
{
    if (static_cast<bool>(y) == false) {
        return true;
    }
    return static_cast<bool>(x) == false ? false : (*x >= *y);
}

template <typename T>
constexpr bool operator==(Optional<T> const & x, nullopt_t) noexcept
{
    return static_cast<bool>(x) == false;
}

template <typename T>
constexpr bool operator==(nullopt_t, Optional<T> const & x) noexcept
{
    return static_cast<bool>(x) == false;
}

template <typename T>
constexpr bool operator!=(Optional<T> const & x, nullopt_t) noexcept
{
    return static_cast<bool>(x);
}

template <typename T>
constexpr bool operator!=(nullopt_t, Optional<T> const & x) noexcept
{
    return static_cast<bool>(x);
}

template <typename T>
constexpr bool operator<(Optional<T> const & x, nullopt_t) noexcept
{
    return false;
}

template <typename T>
constexpr bool operator<(nullopt_t, Optional<T> const & x) noexcept
{
    return static_cast<bool>(x);
}

template <typename T>
constexpr bool operator<=(Optional<T> const & x, nullopt_t) noexcept
{
    return static_cast<bool>(x) == false;
}

template <typename T>
constexpr bool operator<=(nullopt_t, Optional<T> const & x) noexcept
{
    return true;
}

template <typename T>
constexpr bool operator>(Optional<T> const & x, nullopt_t) noexcept
{
    return static_cast<bool>(x);
}

template <typename T>
constexpr bool operator>(nullopt_t, Optional<T> const & x) noexcept
{
    return false;
}

template <typename T>
constexpr bool operator>=(Optional<T> const & x, nullopt_t) noexcept
{
    return true;
}

template <typename T>
constexpr bool operator>=(nullopt_t, Optional<T> const & x) noexcept
{
    return static_cast<bool>(x) == false;
}

template <typename T, typename U>
constexpr bool operator==(Optional<T> const & x, U const & v)
{
    return static_cast<bool>(x) ? (*x == v) : false;
}

template <typename T, typename U>
constexpr bool operator==(U const & v, Optional<T> const & x)
{
    return static_cast<bool>(x) ? (v == *x) : false;
}

template <typename T, typename U>
constexpr bool operator!=(Optional<T> const & x, U const & v)
{
    return static_cast<bool>(x) ? (*x != v) : true;
}

template <typename T, typename U>
constexpr bool operator!=(U const & v, Optional<T> const & x)
{
    return static_cast<bool>(x) ? (v != *x) : true;
}

template <typename T, typename U>
constexpr bool operator<(Optional<T> const & x, U const & v)
{
    return static_cast<bool>(x) ? (*x < v) : true;
}

template <typename T, typename U>
constexpr bool operator<(U const & v, Optional<T> const & x)
{
    return static_cast<bool>(x) ? (v < *x) : false;
}

template <typename T, typename U>
constexpr bool operator<=(Optional<T> const & x, U const & v)
{
    return static_cast<bool>(x) ? (*x <= v) : true;
}

template <typename T, typename U>
constexpr bool operator<=(U const & v, Optional<T> const & x)
{
    return static_cast<bool>(x) ? (v <= *x) : false;
}

template <typename T, typename U>
constexpr bool operator>(Optional<T> const & x, U const & v)
{
    return static_cast<bool>(x) ? (*x > v) : false;
}

template <typename T, typename U>
constexpr bool operator>(U const & v, Optional<T> const & x)
{
    return static_cast<bool>(x) ? (v > *x) : true;
}

template <typename T, typename U>
constexpr bool operator>=(Optional<T> const & x, U const & v)
{
    return static_cast<bool>(x) ? (*x >= v) : false;
}

template <typename T, typename U>
constexpr bool operator>=(U const & v, Optional<T> const & x)
{
    return static_cast<bool>(x) ? (v >= *x) : true;
}

#if __cplusplus >= 201703L
template <typename T, typename = EnableIfMoveConstructible<T>, typename = EnableIfSwappable<T>>
void swap(Optional<T>& x, Optional<T>& y) noexcept(noexcept(x.swap(y)))
{
    x.swap(y);
}
#else
template <typename T, typename = EnableIfMoveConstructible<T>>
void swap(Optional<T>& x, Optional<T>& y) noexcept(noexcept(x.swap(y)))
{
    x.swap(y);
}
#endif

template <typename T>
constexpr Optional<typename std::decay<T>::type> make_optional(T && v)
{
    return Optional<typename std::decay<T>::type>(std::forward<T>(v));
}

template <typename T, typename...Args>
constexpr Optional<T> make_optional(Args &&... args)
{
    return Optional<T>(in_place, std::forward<Args>(args)...);
}

template <typename T, typename U, typename... Args>
constexpr Optional<T> make_optional(std::initializer_list<U> il, Args &&... args)
{
    return Optional<T>(in_place, il, std::forward<Args>(args)...);
}
}  // namespace core
}  // namespace ara
namespace std {
template <class T>
struct hash<ara::core::Optional<T>> {
    int UNSPECIFIED_VALUE = 0;
    std::size_t operator()(ara::core::Optional<T> const & o) const
    {
        if (static_cast<bool>(o)) {
            return std::hash<T>()(*o);
        }
        return UNSPECIFIED_VALUE;
    }
};
}
#endif
