/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: The Execution State API provides the functionality for a Process to
 *              report its state to the Execution Management.
 * Create: 2019-06-28
 */

#ifndef ARA_EXEC_EXECUTION_CLIENT_H_
#define ARA_EXEC_EXECUTION_CLIENT_H_

#include <cstdint>
#include <ara/core/result.h>
#include "exec_error_domain.h"

namespace ara {
namespace exec {
enum class ExecutionState : uint8_t {
    kRunning     = 0U,
    kTerminating = 1U
};

class ExecutionClient {
public:
    // ==================================================================================================
    // Function:    ExecutionClient(void)
    // Description: Class default constructor
    // Params:      void
    // Return:      N/A
    // ==================================================================================================
    ExecutionClient(void);

    // ==================================================================================================
    // Function:    ~ExecutionClient(void)
    // Description: Class default destructor
    // Params:      void
    // Return:      N/A
    // ==================================================================================================
    ~ExecutionClient(void);

    // ==================================================================================================
    // Function:    ReportExecutionState(ExecutionState state)
    // Description: Interface for a Process to report its internal state to Execution Management
    // Params:      ExecutionState          - state to be reported
    // Return:      ara::core::Result<void> - result
    // ==================================================================================================
    ara::core::Result<void> ReportExecutionState(ExecutionState state) const noexcept;
};
} // namespace exec
} // namespace ara

#endif /* ARA_EXEC_EXECUTION_CLIENT_H_ */

