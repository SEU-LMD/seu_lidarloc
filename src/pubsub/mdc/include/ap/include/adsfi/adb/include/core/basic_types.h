/* *
 * FUNCTION: Define Common Types
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * */

#ifndef HAF_CORE_BASIC_TYPES_H
#define HAF_CORE_BASIC_TYPES_H

#include <cstdint>

namespace Adsfi {
/* * Define basic numerical types.
 * MISRA Rule 3-9-2 typedefs that indicate size and signedness should be
 * used in place of the basic numerical types.
 */
using char_t = char;
using int8_t = ::std::int8_t;
using uint8_t = ::std::uint8_t;
using int16_t = ::std::int16_t;
using uint16_t = ::std::uint16_t;
using int32_t = ::std::int32_t;
using uint32_t = ::std::uint32_t;
using int64_t = ::std::int64_t;
using uint64_t = ::std::uint64_t;
using float32_t = float;
using float64_t = double;
using float128_t = long double;
}  // namespace Adsfi
#endif  // HAF_CORE_BASIC_TYPES_H
