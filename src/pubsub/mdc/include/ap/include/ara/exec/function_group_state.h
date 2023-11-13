/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: function_group_state header
 * Create: 2020-05-18
 */
#ifndef VRTF_FUNCTION_GROUP_STATE_H
#define VRTF_FUNCTION_GROUP_STATE_H

#include <cstdint>
#include <string>
#include <ara/core/future.h>
#include "exec_error_domain.h"
#include "function_group.h"

namespace ara {
namespace exec {
class FunctionGroupState {
public:
    class CtorToken {
    public:
        CtorToken(core::StringView const &functionGroupName, const core::StringView& targetState)
            : functionGroupName_(functionGroupName), targetState_(targetState) {};
        ~CtorToken() = default;
        core::StringView functionGroupName_;
        core::StringView targetState_;
    };

    static core::Result<FunctionGroupState::CtorToken> Preconstruct(FunctionGroup const &functionGroup,
            core::StringView metaModelIdentifier) noexcept;
    FunctionGroupState(FunctionGroupState::CtorToken &&token) noexcept;
    ~FunctionGroupState() noexcept = default;

    core::StringView functionGroupName_ {};
    core::StringView targetState_ {};
};
}
}

#endif // VRTF_FUNCTION_GROUP_STATE_H

