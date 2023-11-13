/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: the declaration of the global initialization and shutdown functions that initialize rep.
 * Create: 2020-03-21
 */

#ifndef ARA_CORE_INITIALIZATION_H
#define ARA_CORE_INITIALIZATION_H

#include "ara/core/result.h"

namespace ara {
namespace core {
Result<void> Initialize ();
Result<void> Deinitialize ();
} // End of namespace core
} // End of namespace ara

#endif
