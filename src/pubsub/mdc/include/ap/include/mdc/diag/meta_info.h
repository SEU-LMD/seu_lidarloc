/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:meta_info.h is designed for transfer some diagnostic meta infos like conversation id or source address.
 */
#ifndef MDC_DIAG_META_INFO
#define MDC_DIAG_META_INFO
#include <map>
#include <string>
namespace mdc {
namespace diag {
const std::string DEFAULT_SESSION = "default";
const std::string PROGRAMMING_SESSION = "programming";
const std::string EXTENDED_DIAGNOSTIC_SESSION = "extendedDiagnostic";
const std::string SAFETY_SYSTEM_DIAGNOSTIC_SESSION = "safetySystemDiagnostic";
const std::string ISO_SAE_RESERVED = "isoSAEReserved";
using MetaInfo = std::map<std::string, std::string>;
}
}
#endif