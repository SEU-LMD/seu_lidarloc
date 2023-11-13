/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: This file provides an an interface to modify serialization configuration.
 * Create: 2021-02-19
 */
#ifndef VRTF_SERIALIZE_SERIALIZE_CONFIG_H
#define VRTF_SERIALIZE_SERIALIZE_CONFIG_H
#include <cstdint>
namespace vrtf {
namespace serialize {
enum class SerializationType: uint32_t {
    ROS = 0U,
    CM = 1U
};

enum class SerializeType: uint8_t {
    SHM = 0,
    DDS = 1,
    SOMEIP = 2
};

enum class StructSerializationPolicy: uint8_t {
    OUT_LAYER_ALIGNMENT,
    DISABLE_OUT_LAYER_ALIGNMENT
};

enum class WireType: uint8_t {
    STATIC,
    DYNAMIC
};

enum class SomeipSerializeType: uint8_t {
    ENABLETLV,
    DISABLETLV
};

enum class ByteOrder: uint8_t {
    BIGENDIAN,
    LITTLEENDIAN
};
const size_t DEFAULT_DYNAMIC_LENGTH = 4;
class SerializeConfig {
public:
SerializeConfig()
    : type(SerializationType::CM), structPolicy(StructSerializationPolicy::OUT_LAYER_ALIGNMENT),
      someipSerializeType(SomeipSerializeType::DISABLETLV), wireType(WireType::STATIC), byteOrder(ByteOrder::BIGENDIAN),
      staticLengthField(DEFAULT_DYNAMIC_LENGTH), stringLength(DEFAULT_DYNAMIC_LENGTH),
      vectorLength(DEFAULT_DYNAMIC_LENGTH), arrayLength(0), structLength(0),
      implementsLegencyStringSerialization(false), isTopStruct(true), structDeserializeLength(0),
      isIngoreOutLength(false) {}
virtual ~SerializeConfig() = default;
SerializationType type;
StructSerializationPolicy structPolicy;
SomeipSerializeType someipSerializeType =  SomeipSerializeType::DISABLETLV;
WireType wireType = WireType::STATIC;
ByteOrder byteOrder = ByteOrder::BIGENDIAN;
size_t staticLengthField;
size_t stringLength;
size_t vectorLength;
size_t arrayLength;
size_t structLength;
bool implementsLegencyStringSerialization;
bool isTopStruct;
size_t structDeserializeLength;
bool isIngoreOutLength;
void SyncTlvLength()
{
    if (someipSerializeType == SomeipSerializeType::ENABLETLV) {
        if (wireType == WireType::STATIC) {
            stringLength = staticLengthField;
            vectorLength = staticLengthField;
            arrayLength = staticLengthField;
            structLength = staticLengthField;
        } else {
            stringLength = DEFAULT_DYNAMIC_LENGTH;
            vectorLength = DEFAULT_DYNAMIC_LENGTH;
            arrayLength = DEFAULT_DYNAMIC_LENGTH;
            structLength = DEFAULT_DYNAMIC_LENGTH;
        }
    }
}
};
}
}
#endif