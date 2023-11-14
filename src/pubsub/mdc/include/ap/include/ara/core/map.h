/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of Map class according to AutoSAR standard core type
 * Create: 2019-07-24
 */
#ifndef ARA_CORE_MAP_H
#define ARA_CORE_MAP_H
#include <map>
namespace ara {
namespace core {
template<class T, class Y>
using Map=std::map<T, Y>;

template<class T, class Y>
using map=std::map<T, Y>;
}
}

#endif