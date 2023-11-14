/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Ros types definition
 * Create: 2020-04-22
 */
#ifndef RTF_COM_TYPES_ROS_TYPES_H_
#define RTF_COM_TYPES_ROS_TYPES_H_

#include <cstdint>

// Vcc types
#include <vrtf/vcc/api/types.h>
#include <vrtf/vcc/api/proxy.h>
#include <vrtf/vcc/api/skeleton.h>
#include <vrtf/vcc/vcc.h>
#include "vrtf/vcc/internal/traffic_crtl_policy.h"
#include <vrtf/vcc/api/raw_buffer_helper.h>
#include <vrtf/vcc/api/shape_shifter.h>
#include <vrtf/vcc/api/stream.h>
#include "vrtf/vcc/serialize/serialize_config.h"

// Protocol types
#include "rtf/com/types/dds_types.h"
#include "rtf/com/types/someip_types.h"

namespace rtf {
namespace com {
// Primitive types
using ServiceId                 = uint16_t;
using InstanceId                = uint16_t;
using EntityId                  = uint32_t;
using Network                   = std::string;
using RawMemory                 = vrtf::vcc::api::types::RawBuffer;
using RecvMemory                = vrtf::vcc::api::types::RecvBuffer;
using RawBufferHelper           = vrtf::vcc::RawBufferHelper;
using SampleInfo                = vrtf::vcc::api::types::SampleInfo;
using ShapeShifter              = vrtf::vcc::api::types::ShapeShifter;
using Stream                    = vrtf::vcc::api::types::Stream;
namespace Reserve = vrtf::vcc::api::types::reserve;
// Vcc types
using VccProxy                     = vrtf::vcc::Proxy;
using VccSkeleton                  = vrtf::vcc::Skeleton;
using VccThreadPool                = vrtf::vcc::utils::ThreadPool;
using VccDriverType                = vrtf::vcc::api::types::DriverType;
using VccSdInfo                    = vrtf::vcc::api::types::ServiceDiscoveryInfo;
using VccFindServiceHandle         = vrtf::vcc::api::types::FindServiceHandle;
using VccServiceHandle             = vrtf::vcc::api::types::HandleType;
using VccFindServiceHandler        = vrtf::vcc::api::types::FindServiceHandler<VccServiceHandle>;
using VccServiceHandleContainer    = vrtf::vcc::api::types::ServiceHandleContainer<VccServiceHandle>;
using VccEntityInfo                = vrtf::vcc::api::types::EntityInfo;
using VccEventInfo                 = vrtf::vcc::api::types::EventInfo;
using VccMethodInfo                = vrtf::vcc::api::types::MethodInfo;
using VccVersionInfo               = vrtf::vcc::api::types::VersionInfo;
using VccMethodCallProcessingMode  = vrtf::vcc::api::types::MethodCallProcessingMode;
using VccMethodState               = vrtf::vcc::api::types::MethodState;
using VccSerializationType         = vrtf::serialize::SerializationType;
using VccSerializeConfig           = vrtf::serialize::SerializeConfig;
using VccStructSerializationPolicy = vrtf::serialize::StructSerializationPolicy;
using ErrorCodeExDomainType        = vrtf::core::ErrorDomain::IdType;
using ErrorCodeExValueType         = vrtf::core::ErrorDomain::CodeType;
using FileOwner                    = vrtf::vcc::api::types::FileOwner;
using ResourceAttr                 = vrtf::vcc::api::types::ResourceAttr;
template <typename T>
using VccSamplePtr = vrtf::vcc::api::types::SamplePtr<T>;
using ResourceType = vrtf::vcc::api::types::ResourceType;
using ResourcePara = vrtf::vcc::api::types::ResourcePara;
using ShmObject = vrtf::vcc::api::types::ShmObject;
using ResourceCreateHandler = vrtf::vcc::api::types::ResourceCreateHandler;

// Internal types
enum class AdapterType : uint8_t {
    UNKNOWN = 0x00U,
    EVENT   = 0x01U,
    METHOD  = 0x02U
};

enum class ScheduleMode : uint8_t {
    DETERMINATE   = 0x00U,
    INDETERMINATE = 0x01U
};

enum class AdapterProtocol : uint8_t {
    UNKNOWN = 0x00U,
    DDS     = 0x01U,
    SOMEIP  = 0x02U
};

enum class TransportMode : uint8_t {
    TCP      = 0x00U,
    UDP      = 0x01U,
    SHM      = 0x02U,
    DSHM     = 0X03U,
    ICC      = 0x04U,
    UDP2DSHM = 0x05U
};

enum class SerializationType: uint32_t {
    ROS = 0U,
    CM = 1U
};

enum class Role: uint8_t {
    BOTH     = 0x00,
    SERVER   = 0x01,
    CLIENT   = 0x02
};

enum class ThreadMode : uint8_t {
    EVENT  = 0x00U,
    POLL   = 0x01U,
};


struct EntityConfig {
    std::string                    serviceName;
    std::shared_ptr<VccEntityInfo> entityInfo;
    std::shared_ptr<VccSdInfo>     sdInfo;
    std::shared_ptr<rtf::TrafficCtrlPolicy> trafficCrtlPolicy;
};

struct MaintainConfig {
    std::string                    dataType;
    std::string                    requestArgs;
    std::string                    responseArgs;
};

namespace internal {
using PlogServerTimeStampNode   = vrtf::vcc::utils::PlogServerTimeStampNode;
using PlogDriverType            = vrtf::vcc::utils::PlogDriverType;
using PlogInfo                  = vrtf::vcc::utils::PlogInfo;
using SampleInfoImpl            = vrtf::vcc::api::types::internal::SampleInfoImpl;
}
} // namespace com
} // namespace rtf
#endif
