/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_COM_ERROR_DOMAIN_H_
#define ARA_COM_COM_ERROR_DOMAIN_H_
#include <map>
#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
#include "vrtf/vcc/api/com_error_domain.h"
#include "vrtf/vcc/api/types.h"

namespace ara {
namespace com {
using ComErrc = vrtf::vcc::api::types::ComErrc;
using ComException = vrtf::vcc::api::types::ComException;
using ComErrorDomain = vrtf::vcc::api::types::ComErrorDomain;

constexpr ara::core::ErrorDomain const &GetComErrorDomain() noexcept
{
    return vrtf::vcc::api::types::GetComErrorDomain();
}

constexpr ara::core::ErrorCode MakeErrorCode(ComErrc code,
                                             ara::core::ErrorDomain::SupportDataType data) noexcept
{
    return vrtf::vcc::api::types::MakeErrorCode(code, data);
}
}
}
#endif
