/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: provide to user for include then use Rosadapter function
 * Create: 2019-12-4
 */
#ifndef RTF_INIT_H_
#define RTF_INIT_H_

#include <string>
#include "rtf/com/types/someip_types.h"

#include "rtf/com/entity/thread_group.h"

namespace rtf {
namespace com {
/**
 * @brief Initialize node
 * @param[in] nodeName  The name of the node will be initialized
 * @param[in] namespace The namespace of the node
 * @return Operation result
 */
bool Init(const std::string& nodeName, const std::string& nameSpace = "/") noexcept;

/**
 * @brief Get the initialize state
 * @return return if the node has been initialized
 */
bool IsInitialized(void) noexcept;

/**
 * @brief Get the name of this node
 * @return The name of node
 */
std::string GetNodeName(void) noexcept;

/**
 * @brief Get the namespace of node
 * @return The namespace of node
 */
std::string GetNodeNamespace(void) noexcept;

/**
 * @brief Shut down all NodeHandle
 *
 */
void Shutdown(void) noexcept;

/**
 * @brief Check if call global Shutdown
 *
 * @return true    the global shutdowm isn't called
 * @return false   the global shutdown is called
 */
bool IsOk(void) noexcept;

/**
 * @brief Get event data by spin once synchronously.
 * @param[in] threadGroup  The thread group to get event data
 */
void SpinOnce(ThreadGroup const &threadGroup) noexcept;

/**
 * @brief Register a diagnosis handler which contains some callbacks about diagnosis.
 * @param[in] handler The faults diagnosis callback handler
 */
void RegisterFaultsDiagnosisReportCallback(someip::FaultsDiagnosisHandler const &handler) noexcept;

/**
 * @brief Unregister the diagnosis handler.
 * @param[in] callbackType The callback type to be unregistered.
 */
void UnregisterFaultsDiagnosisReportCallback(someip::FaultsDiagnosisCallbackType const callbackType) noexcept;

/**
 * @brief Reset the counter of diagnosis counter type.
 * @param[in] counterType The diagnosis counter type to be reseted.
 */
void ResetDiagnosisCounterReport(someip::ResetDiagnosisCounterType const counterType) noexcept;
} // namespace com
} // namespace rtf
#endif // RTF_INIT_H_
