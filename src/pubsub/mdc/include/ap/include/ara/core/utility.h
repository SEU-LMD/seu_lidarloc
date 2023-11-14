/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: the implementation according to AutoSAR standard core type
 * Create: 2019-07-19
 */
#ifndef ARA_CORE_UTILITY_H
#define ARA_CORE_UTILITY_H

#include <initializer_list>
#include <iterator>
#include <cstddef>
namespace ara {
namespace core {
template <typename Container>
constexpr auto data(Container &c) -> decltype(c.data())
{
    return c.data();
}

template <typename Container>
constexpr auto data(Container const &c) -> decltype(c.data())
{
    return c.data();
}

template <typename T, std::size_t N>
constexpr T* data(T(&array)[N]) noexcept
{
    return &array[0];
}

template <typename E>
constexpr E const * data(std::initializer_list<E> il) noexcept
{
    return il.begin();
}

template <typename Container>
constexpr auto size(Container const & c) -> decltype(c.size())
{
    return c.size();
}

template <typename T, std::size_t N>
constexpr std::size_t size(T const (&array)[N]) noexcept
{
    return N;
}

template <typename Container>
constexpr auto empty(Container const & c) -> decltype(c.empty())
{
    return c.empty();
}

template <typename T, std::size_t N>
constexpr bool empty(T const (&array)[N]) noexcept
{
    return false;
}

template <typename E>
constexpr bool empty(std::initializer_list<E> il) noexcept
{
    if (il.size() == 0) {
        return true;
    }
    return false;
}

struct in_place_t {
    explicit in_place_t() = default;
};
constexpr in_place_t in_place {};

template <typename T>
struct in_place_type_t {
    explicit in_place_type_t() = default;
};

template <size_t I>
struct in_place_index_t {
    explicit in_place_index_t() = default;
};
}
}
#endif

