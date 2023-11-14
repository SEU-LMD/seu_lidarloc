/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: the implementation of Array class according to AutoSAR standard core type
 * Create: 2020-05-14
 */

#ifndef MDC_COMMON_INTERNAL_TYPE_CHECK_H
#define MDC_COMMON_INTERNAL_TYPE_CHECK_H

#include "mdc/common/error_code.h"
namespace mdc {
namespace common {
template<typename T, typename E>
class Result;

template<typename T, typename E>
class Future;

namespace internal {
template<typename T>
struct IsResult: public std::false_type {};

template<typename T, typename E>
struct IsResult<mdc::common::Result<T, E>> : public std::true_type {};


template<typename T>
struct IsVoidResult : public std::false_type {};

template<typename E>
struct IsVoidResult<mdc::common::Result<void, E>> : public std::true_type {};

template<typename T>
struct IsFuture: public std::false_type {};

template<typename T, typename E>
struct IsFuture<mdc::common::Future<T, E>> : public std::true_type {};

template<typename T>
struct IsVoidFuture : public std::false_type {};

template<typename E>
struct IsVoidFuture<mdc::common::Future<void, E>> : public std::true_type {};
}
}
}

#endif

