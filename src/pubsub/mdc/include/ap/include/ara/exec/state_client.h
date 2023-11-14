/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: The StateClient provides the functionality for StateManager.
 *              This class is in draft state and it may be changed in future
 * Create: 2019-07-16
 */

#ifndef ARA_EXEC_STATE_CLIENT_H_
#define ARA_EXEC_STATE_CLIENT_H_

#include <cstdint>
#include <string>
#include <ara/core/future.h>
#include <ara/hwcommon/log/log.h>

#include "exec_error_domain.h"
#include "function_group_state.h"

namespace ara {
namespace exec {
enum class StateReturnType : uint8_t {
    kSuccess      = 0U,
    kBusy         = 1U,
    kTimeout      = 2U,
    kGeneralError = 3U
};

class StateClient {
public:
    // ================================================================================================================
    // Function:    StateClient(void)
    // Description: Class default constructor
    // Params:      void
    // Return:      N/A
    // ================================================================================================================
    StateClient(void);

    // ================================================================================================================
    // Function:    ~StateClient(void)
    // Description: Class default destructor
    // Params:      void
    // Return:      N/A
    // ================================================================================================================
    ~StateClient(void);

    // ================================================================================================================
    // Function:    GetInitialMachineStateTransitionResult
    // Description: Method to retrieve result of Machine State initial transition to Startup state
    // Return:      ara::core::Future<void>   - result
    // ================================================================================================================
    ara::core::Future<void> GetInitialMachineStateTransitionResult() const noexcept;

    // ================================================================================================================
    // Function:    SetState
    // Description: Set the new state to a function group
    // Params:      FunctionGroupState        - functionGroupName & targetState
    // Return:      ara::core::Future<void>   - result
    // ================================================================================================================
    ara::core::Future<void> SetState(FunctionGroupState const &state) const noexcept;
};
} // namespace exec
} // namespace ara

#endif /* ARA_EXEC_STATE_CLIENT_H_ */

