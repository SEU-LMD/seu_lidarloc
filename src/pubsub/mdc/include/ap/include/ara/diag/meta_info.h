/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:meta_info.h is designed for transfer some diagnostic meta infos like conversation id or source address.
 */
#ifndef ARA_DIAG_META_INFO
#define ARA_DIAG_META_INFO
#include "ara/core/map.h"
#include "ara/core/string.h"
namespace ara {
namespace diag {
using MetaInfo = ara::core::Map<ara::core::String, ara::core::String>;
}
}
#endif