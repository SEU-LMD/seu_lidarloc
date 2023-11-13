/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: function_group header
 * Create: 2020-05-18
 */
#ifndef VRTF_FUNCTION_GROUP_H
#define VRTF_FUNCTION_GROUP_H

#include <cstdint>
#include <string>
#include <ara/core/future.h>
#include "exec_error_domain.h"

namespace ara {
namespace exec {
class FunctionGroup {
public:
    class CtorToken {
    public:
        CtorToken(core::StringView const &functionGroupName) : functionGroupName_(functionGroupName) {};
        ~CtorToken() = default;
        core::StringView functionGroupName_;
    };

    static core::Result<FunctionGroup::CtorToken> Preconstruct(core::StringView metaModelIdentifier) noexcept;
    FunctionGroup(FunctionGroup::CtorToken &&token) noexcept;
    ~FunctionGroup() noexcept = default;

    core::StringView functionGroupName_;
};
}
}

#endif // VRTF_FUNCTION_GROUP_STATE_H

