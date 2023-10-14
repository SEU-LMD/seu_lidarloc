/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: the implementation of ErrorCode class according to AutoSAR standard core type
 * Create: 2020-03-20
 */
#ifndef ARA_CORE_ABORT_H_
#define ARA_CORE_ABORT_H_
#include<string>

namespace ara {
namespace core {
/**
 * @brief Terminate the current process abnormally [SWS_CORE_00052].
 *
 * @param[in]   text a custom text to include in the log message being output
 */
void Abort(char const *text) noexcept;
}
}

#endif
