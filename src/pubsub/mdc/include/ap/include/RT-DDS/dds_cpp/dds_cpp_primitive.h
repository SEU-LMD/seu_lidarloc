/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_public_primitive.h
 */

#ifndef RT_DDS_DDS_CPP_PRIMITIVE_H
#define RT_DDS_DDS_CPP_PRIMITIVE_H

#include <cstdint>

using DDS_Char = char;

using DDS_Wchar = uint32_t;

using DDS_Octet = uint8_t;

using DDS_Short = int16_t;

using DDS_UnsignedShort = uint16_t;

using DDS_Long = int32_t;

using DDS_UnsignedLong = uint32_t;

using DDS_LongLong = int64_t;

using DDS_UnsignedLongLong = uint64_t;

using DDS_Float = float;

using DDS_Double = double;

using DDS_Boolean = uint8_t;

#define DDS_BOOLEAN_TRUE    ((DDS_Boolean) 1)

#define DDS_BOOLEAN_FALSE   ((DDS_Boolean) 0)

#endif /* RT_DDS_DDS_CPP_PRIMITIVE_H */

