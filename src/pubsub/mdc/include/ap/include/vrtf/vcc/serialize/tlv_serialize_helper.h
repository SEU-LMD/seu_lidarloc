/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: Someip tlv serialize util file
 * Create: 2021-05-05
 */
#ifndef VRTF_TLV_SERIALIZE_HELPER_H
#define VRTF_TLV_SERIALIZE_HELPER_H
#include "vrtf/vcc/serialize/serialize_config.h"
#include "vrtf/vcc/serialize/someip_serialize_helper.h"
#include <map>
#include <unordered_map>
#include <arpa/inet.h>
/*
Note: Serialize max size is 16M (equal to 0x100000), cannot be result in integer overflow with type of size_t
*/
namespace tlv {
namespace serialize {
const std::uint16_t WIRETYPE_ZERO {0};
const std::uint16_t WIRETYPE_ONE {1};
const std::uint16_t WIRETYPE_TWO {2};
const std::uint16_t WIRETYPE_THREE {3};
const std::uint16_t WIRETYPE_FOUR {4};
const std::uint16_t WIRETYPE_FIVE {5};
const std::uint16_t WIRETYPE_SIX {6};
const std::uint16_t WIRETYPE_SEVEN {7};
const std::uint16_t DATAID_BIT {0x0FFFU};
const std::uint16_t WIRETYPE_BIT {0x7000U};
const std::uint16_t WIRETYPE_POS {12};
template <typename T>
class is_smart_pointer_helper : public std::false_type {};

template <typename T>
class is_smart_pointer_helper<std::shared_ptr<T> > : public std::true_type {};

struct GetSizeResult {
    bool isSkip;
    size_t size;
    size_t tagPos_;
};

struct dataIdParams {
    size_t totalSize;
    size_t paramsSize;
};

struct RecordDataIdResult {
    std::unordered_map<std::uint16_t, dataIdParams> dataIdMap;
    size_t size;
};
class TlvSerializeHelper {
public:
    static void CopyLengthField(std::uint8_t* data, const size_t dataLength, const size_t lengthField,
        const vrtf::serialize::ByteOrder order)
    {
        if (lengthField == 0) {
            return;
        }
        size_t pos {0};
        errno_t memcpySuccess {1};
        switch (lengthField) {
            case vrtf::serialize::someip::ONE_LENGTH_FIELD: {
                const std::uint8_t length {static_cast<std::uint8_t>(dataLength)};
                memcpySuccess = memcpy_s(data, dataLength + lengthField, &length, lengthField);
                pos += lengthField;
                break;
            }

            case vrtf::serialize::someip::TWO_LENGTH_FIELD: {
                std::uint16_t length {static_cast<std::uint16_t>(dataLength)};
                length  = vrtf::serialize::someip::htonsEx(length, order);
                memcpySuccess = memcpy_s(data, dataLength + lengthField, &length, lengthField);
                pos += lengthField;
                break;
            }

            case vrtf::serialize::someip::FOUR_LENGTH_FIELD: {
                std::uint32_t length {static_cast<std::uint32_t>(dataLength)};
                length  = vrtf::serialize::someip::htonlEx(length, order);
                memcpySuccess = memcpy_s(data, dataLength + lengthField, &length, lengthField);
                pos += lengthField;
                break;
            }
            default: {
                break;
            }
        }
        if (memcpySuccess != 0) {
            std::string const ctxId {"CM"};
            std::shared_ptr<ara::godel::common::log::Log> logInstance {ara::godel::common::log::Log::GetLog(ctxId)};
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->error() << "[Someip Serializer] Memory copy struct static length fail.";
        }
    }

    // template use for future CM support serialize config
    static size_t GetLengthFieldLength(const vrtf::serialize::SerializeConfig& config)
    {
        if ((config.someipSerializeType == vrtf::serialize::SomeipSerializeType::ENABLETLV)
            && (config.wireType == vrtf::serialize::WireType::STATIC)) {
            return config.staticLengthField;
        }
        return sizeof(std::uint32_t); // dynamic and not tlv serialzie length
    }

    static size_t GetDeserializeLength(const vrtf::serialize::SerializeConfig& config, const std::uint8_t* data)
    {
        std::size_t size {vrtf::serialize::someip::MAX_UINT32_T};
        if ((config.someipSerializeType == vrtf::serialize::SomeipSerializeType::ENABLETLV)
            && (config.wireType == vrtf::serialize::WireType::STATIC)) {
            switch (config.staticLengthField) {
                case vrtf::serialize::someip::ONE_LENGTH_FIELD: {
                    size = *reinterpret_cast<const std::uint8_t* >(data);
                    break;
                }

                case vrtf::serialize::someip::TWO_LENGTH_FIELD: {
                    std::uint16_t length {*reinterpret_cast<const std::uint16_t* >(data)};
                    length = vrtf::serialize::someip::ntohsEx(length, config.byteOrder);
                    size = length;
                    break;
                }

                case vrtf::serialize::someip::FOUR_LENGTH_FIELD: {
                    std::uint32_t length {*reinterpret_cast<const std::uint32_t* >(data)};
                    length = vrtf::serialize::someip::ntohlEx(length, config.byteOrder);
                    size = length;
                    break;
                }
                default: {
                    break;
                }
            }
        } else {
            std::uint32_t length {*reinterpret_cast<const std::uint32_t* >(data)};
            length = vrtf::serialize::someip::ntohlEx(length, config.byteOrder);
            size = length;
        }
        return size;
    }

    static size_t GetShiftLength(const std::uint16_t wireType, const vrtf::serialize::SerializeConfig& config,
                                 const std::uint8_t* tlvPtr)
    {
        size_t pos {0};
        switch (wireType) {
            case WIRETYPE_ZERO: {
                pos += sizeof(std::uint8_t);
                break;
            }
            case WIRETYPE_ONE: {
                pos += sizeof(std::uint16_t);
                break;
            }
            case WIRETYPE_TWO: {
                pos += sizeof(std::uint32_t);
                break;
            }
            case WIRETYPE_THREE: {
                pos += sizeof(std::uint64_t);
                break;
            }
            case WIRETYPE_FOUR: {
                pos = tlv::serialize::TlvSerializeHelper::GetDeserializeLength(config, tlvPtr) +
                    config.staticLengthField;
                break;
            }
            case WIRETYPE_FIVE: {
                pos = tlv::serialize::TlvSerializeHelper::GetDeserializeLength(config, tlvPtr) + sizeof(std::uint8_t);
                break;
            }
            case WIRETYPE_SIX: {
                pos = tlv::serialize::TlvSerializeHelper::GetDeserializeLength(config, tlvPtr) + sizeof(std::uint16_t);
                break;
            }
            case WIRETYPE_SEVEN: {
                pos = tlv::serialize::TlvSerializeHelper::GetDeserializeLength(config, tlvPtr) + sizeof(std::uint32_t);
                break;
            }
            default: {
                break;
            }
        }
        return pos;
    }

    static RecordDataIdResult RecordDataId(const vrtf::serialize::SerializeConfig& config,
        const uint8_t* initAddress, const size_t initLength)
    {
        RecordDataIdResult result;
        const uint8_t* tlvPtr {initAddress};
        size_t tlvLength {0};
        while (tlvLength < initLength) {
            if (initLength - tlvLength < sizeof(std::uint16_t)) {
                std::string const ctxId {"CM"};
                std::shared_ptr<ara::godel::common::log::Log> logInstance {ara::godel::common::log::Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "Deserialization[ROS] tlv not find necessary";
                return result;
            }
            std::uint16_t tag {*reinterpret_cast<const std::uint16_t* >(tlvPtr)};
            tag = vrtf::serialize::someip::ntohsEx(tag, config.byteOrder);
            std::uint16_t nowId {static_cast<std::uint16_t>(tag & DATAID_BIT)}; // dataId is low 12 bits
            const std::uint16_t wireType {static_cast<std::uint16_t>((tag & WIRETYPE_BIT) >> WIRETYPE_POS)};
            if (wireType > WIRETYPE_SEVEN) {
                std::string const ctxId {"CM"};
                std::shared_ptr<ara::godel::common::log::Log> logInstance {ara::godel::common::log::Log::GetLog(ctxId)};
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->error() << "Deserialization[ROS] wrong wire type : " << wireType << " dataId is " << nowId;
                return result;
            }
            tlvLength += sizeof(std::uint16_t);
            tlvPtr += sizeof(std::uint16_t); // pos dataId length
            const size_t posLength {GetShiftLength(wireType, config, tlvPtr)};
            dataIdParams params {tlvLength, posLength};
            static_cast<void>(result.dataIdMap.emplace(nowId, params));
            tlvLength += posLength;
            tlvPtr += posLength;
        }
        result.size = tlvLength;
        return result;
    }
};
}
}
#endif
