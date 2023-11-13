/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_sequence.h
 */

#ifndef RT_DDS_DDS_CPP_SEQUENCE_H
#define RT_DDS_DDS_CPP_SEQUENCE_H

#include "RT-DDS/dds_cpp/dds_cpp_primitive.h"

#define DDS_SEQUENCE_MEMBERS(FooSeq, Foo)    \
    DDS_UnsignedLong maximum;                \
    DDS_UnsignedLong length;                 \
    Foo *buffer;                             \
    DDS_Boolean owned;                       \

#define DDS_SEQUENCE_METHODS(FooSeq, Foo)                                       \
    explicit FooSeq(DDS_UnsignedLong max = 0);                                  \
    FooSeq(const FooSeq &srcSeq);                                               \
    DDS_UnsignedLong Maximum() const;                                           \
    DDS_Boolean SetMaximum(DDS_UnsignedLong newMax);                            \
    DDS_UnsignedLong Length() const;                                            \
    DDS_Boolean FromArray(const Foo array[], const DDS_UnsignedLong length);    \
    DDS_Boolean ToArray(Foo array[], DDS_UnsignedLong length);                  \
    Foo *GetBuffer();                                                           \
    Foo &At(DDS_UnsignedLong i) const;                                          \
    DDS_Boolean PushBack(Foo value);                                            \
    DDS_Boolean Clear();                                                        \
    void CopyArray(const Foo array[], const DDS_UnsignedLong size);             \
    FooSeq &operator=(const struct FooSeq &srcSeq);                             \
    FooSeq &operator=(struct FooSeq &&srcSeq);                                  \
    ~FooSeq();                                                                  \

#define DDS_SEQUENCE(FooSeq, Foo) \
struct FooSeq{                    \
DDS_SEQUENCE_MEMBERS(FooSeq, Foo) \
DDS_SEQUENCE_METHODS(FooSeq, Foo) \
};                                \

#endif /* RT_DDS_DDS_CPP_SEQUENCE_H */

