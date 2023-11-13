/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of Array class according to AutoSAR standard core type
 * Create: 2019-07-24
 */
#ifndef ARA_CORE_ARRAY_H
#define ARA_CORE_ARRAY_H
#include <array>
namespace ara {
namespace core {
template<class T, std::size_t N>
using Array = std::array<T, N>;
}
}

#endif
