/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of Vector class according to AutoSAR standard core type
 * Create: 2019-07-24
 */
#ifndef ARA_CORE_VECTOR_H
#define ARA_CORE_VECTOR_H
#include <vector>

namespace ara {
namespace core {
template <typename T>
using vector = std::vector<T>;

template <typename T, typename Allocator = std::allocator<T>>
using Vector = std::vector<T, Allocator>;

template <typename T, typename Allocator>
void swap(Vector<T, Allocator> const & lhs, Vector<T, Allocator> const & rhs)
{
    return std::swap(lhs, rhs);
}
}
}
#endif
