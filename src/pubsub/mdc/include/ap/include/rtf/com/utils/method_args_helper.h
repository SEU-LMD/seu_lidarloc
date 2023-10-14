/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The template to check if the function GetResponseArgs() in data definiation is exist
 * Create: 2020-07-31
 */
#ifndef RTF_COM_UTILS_METHOD_ARGS_HELPER_H_
#define RTF_COM_UTILS_METHOD_ARGS_HELPER_H_

#include <string>
#include <vector>
#include <type_traits>
#include <typeinfo>
#include <cxxabi.h>
#include <iostream>

namespace rtf      {
namespace com      {
namespace utils    {
namespace internal {
template <typename T>
struct has_get_response_args {
// Define a helper to detect function
template <typename U, std::vector<std::string>(*)()>
struct Helper;

// Try to match template Helper to detect expected function
// The function is just used for providing return type
// The reason why the function parameter is 'Helper*' is
// we need an expression to satisfy decltype(...)
template <typename U>
static std::true_type HasFunction(Helper<U, &U::GetResponseArgs>*);

// This matches all cases, which is the fallback
// The function is just used for providing return type
template <typename U>
static std::false_type HasFunction(...);

// Calucate the return type of HasFunction by decltype(...)
// and set the type of value
static const decltype(HasFunction<T>(nullptr)) value;
};

template <typename T>
struct has_get_request_args {
// Define a helper to detect function
template <typename U, std::vector<std::string>(*)()>
struct Helper;

// Try to match template Helper to detect expected function
// The function is just used for providing return type
// The reason why the function parameter is 'Helper*' is
// we need an expression to satisfy decltype(...)
template <typename U>
static std::true_type HasFunction(Helper<U, &U::GetRequestArgs>*);

// This matches all cases, which is the fallback
// The function is just used for providing return type
template <typename U>
static std::false_type HasFunction(...);

// Calucate the return type of HasFunction by decltype(...)
// and set the type of value
static const decltype(HasFunction<T>(nullptr)) value;
};
} // namespace internal

using ResponseArgsType = std::vector<std::string>;
using RequestArgsType = std::vector<std::string>;

template <typename T>
constexpr typename std::enable_if<internal::has_get_response_args<T>::value, ResponseArgsType>::type GetResponseArgs()
{
    return T::GetResponseArgs();
}

template <typename T>
constexpr typename std::enable_if<!internal::has_get_response_args<T>::value, ResponseArgsType>::type GetResponseArgs()
{
    return {};
}

template <typename T>
constexpr typename std::enable_if<internal::has_get_request_args<T>::value, RequestArgsType>::type GetRequestArgs()
{
    return T::GetRequestArgs();
}

template <typename T>
constexpr typename std::enable_if<!internal::has_get_request_args<T>::value, RequestArgsType>::type GetRequestArgs()
{
    return {};
}
} // namespace utils
} // namespace com
} // namespace rtf

#endif
