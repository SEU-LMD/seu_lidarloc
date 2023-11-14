/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: the implementation of Array class according to AutoSAR standard core type
 * Create: 2020-05-14
 */

#ifndef ARA_CORE_INTERNAL_TYPE_CHECK_H_
#define ARA_CORE_INTERNAL_TYPE_CHECK_H_

#include "ara/core/error_code.h"
namespace ara {
namespace core {
template<typename T, typename E>
class Result;

template<typename T, typename E>
class Future;

namespace internal {
template<typename T>
struct IsResult : public std::false_type {
};

template<typename T, typename E>
struct IsResult<ara::core::Result<T, E>> : public std::true_type {};


template<typename T>
struct IsVoidResult : public std::false_type {};

template<typename E>
struct IsVoidResult<ara::core::Result<void, E>> : public std::true_type {};

template<typename T>
struct IsFuture: public std::false_type {};

template<typename T, typename E>
struct IsFuture<ara::core::Future<T, E>> : public std::true_type {};

template<typename T>
struct IsVoidFuture : public std::false_type {};

template<typename E>
struct IsVoidFuture<ara::core::Future<void, E>> : public std::true_type {};
}
}
}

#endif

