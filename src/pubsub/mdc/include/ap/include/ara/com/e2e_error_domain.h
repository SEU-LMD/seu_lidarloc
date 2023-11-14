/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file provides an interface related to communication management using E2E.
 * Create: 2020-03-24
 */
#ifndef ARA_COM_E2E_ERROR_DOMAIN_H_
#define ARA_COM_E2E_ERROR_DOMAIN_H_

#include "vrtf/vcc/api/e2e_error_domain.h"
#include "vrtf/vcc/api/types.h"

#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
#include "ara/core/vector.h"

namespace ara {
namespace com {
namespace e2e {
using E2EErrorCode = vrtf::vcc::api::types::E2EErrorCode;
using DataID = ara::core::Vector<std::uint32_t>;
using MessageCounter = std::uint32_t;
}
}
}
#endif