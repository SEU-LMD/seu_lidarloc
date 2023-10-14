/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: FooSeq.h
 */

#ifndef RT_DDS_DDS_CPP_FOOSEQ_H
#define RT_DDS_DDS_CPP_FOOSEQ_H

#include "RT-DDS/dds_cpp/dds_cpp_common.h"
#include "RT-DDS/dds_cpp/dds_cpp_sequence.h"

DDS_SEQUENCE(DDS_CharSeq, DDS_Char);

DDS_SEQUENCE(DDS_WcharSeq, DDS_Wchar);

DDS_SEQUENCE(DDS_OctetSeq, DDS_Octet);

DDS_SEQUENCE(DDS_ShortSeq, DDS_Short);

DDS_SEQUENCE(DDS_UnsignedShortSeq, DDS_UnsignedShort);

DDS_SEQUENCE(DDS_LongSeq, DDS_Long);

DDS_SEQUENCE(DDS_UnsignedLongSeq, DDS_UnsignedLong);

DDS_SEQUENCE(DDS_LongLongSeq, DDS_LongLong);

DDS_SEQUENCE(DDS_UnsignedLongLongSeq, DDS_UnsignedLongLong);

DDS_SEQUENCE(DDS_FloatSeq, DDS_Float);

DDS_SEQUENCE(DDS_DoubleSeq, DDS_Double);

DDS_SEQUENCE(DDS_BooleanSeq, DDS_Boolean);

DDS_SEQUENCE(DDS_StringSeq, char*);

DDS_SEQUENCE(DDS_InstanceHandleSeq, DDS_InstanceHandle);

#endif /* RT_DDS_DDS_CPP_FOOSEQ_H */

