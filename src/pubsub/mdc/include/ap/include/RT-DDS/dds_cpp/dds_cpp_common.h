/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_common.h
 */

#ifndef RT_DDS_DDS_CPP_COMMON_H
#define RT_DDS_DDS_CPP_COMMON_H

#include <securec.h>
#include <vector>
#include <string>

#include "RT-DDS/dds_cpp/dds_cpp_primitive.h"
#include "dp_adapter.h"

const int DDS_HANDLE_NIL = 0;

/** @name Return codes
  @{**/
enum DDS_ReturnCode {
    DDS_RETCODE_OK = 0,
    DDS_RETCODE_ERROR = 1,
    DDS_RETCODE_UNSUPPORTED = 2,
    DDS_RETCODE_BAD_PARAMETER = 3,
    DDS_RETCODE_PRECONDITION_NOT_MET = 4,
    DDS_RETCODE_OUT_OF_RESOURCES = 5,
    DDS_RETCODE_NOT_ENABLED = 6,
    DDS_RETCODE_IMMUTABLE_POLICY = 7,
    DDS_RETCODE_INCONSISTENT_POLICY = 8,
    DDS_RETCODE_ALREADY_DELETED = 9,
    DDS_RETCODE_TIMEOUT = 10,
    DDS_RETCODE_NO_DATA = 11,
    DDS_RETCODE_ILLEGAL_OPERATION = 12,
    DDS_RETCODE_NOT_ALLOWED_BY_SECURITY = 13
};

/** @} */
enum DDS_StatusKind {
    DDS_INCONSISTENT_TOPIC_STATUS = 0x0001,
    DDS_OFFERED_DEADLINE_MISSED_STATUS = 0x0001 << 1,
    DDS_REQUESTED_DEADLINE_MISSED_STATUS = 0x0001 << 2,
    DDS_OFFERED_INCOMPATIBLE_QOS_STATUS = 0x0001 << 3,
    DDS_REQUESTED_INCOMPATIBLE_QOS_STATUS = 0x0001 << 4,
    DDS_SAMPLE_LOST_STATUS = 0x0001 << 5,
    DDS_SAMPLE_REJECTED_STATUS = 0x0001 << 6,
    DDS_DATA_ON_READERS_STATUS = 0x0001 << 7,
    DDS_DATA_AVAILABLE_STATUS = 0x0001 << 8,
    DDS_LIVELINESS_LOST_STATUS = 0x0001 << 9,
    DDS_LIVELINESS_CHANGED_STATUS = 0x0001 << 10,
    DDS_PUBLICATION_MATCHED_STATUS = 0x0001 << 11,
    DDS_SUBSCRIPTION_MATCHED_STATUS = 0x0001 << 12,
};

using DDS_DOMAINID_TYPE_NATIVE  = DDS_Long;
using DDS_HANDLE_TYPE_NATIVE    = DDS_UnsignedLongLong;
using DDS_DomainId              = DDS_DOMAINID_TYPE_NATIVE;
using DDS_InstanceHandle        = DDS_HANDLE_TYPE_NATIVE;
using DDS_StatusMask            = DDS_UnsignedLong;

/*
  Listeners implemented as structs containing callback functions
  that take listener status types as arguments.
*/
struct DDS_OfferedDeadlineMissedStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
    DDS_InstanceHandle lastInstanceHandle;
};

struct DDS_OfferedIncompatibleQosStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
    DDS_UnsignedLong lastPolicyId;
};

struct DDS_PublicationMatchedStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
    DDS_UnsignedLong currentCount;
    DDS_Long currentCountChange;
    DDS_InstanceHandle lastSubscriptionHandle;
};

struct DDS_LivelinessLostStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
};

struct DDS_SubscriptionMatchedStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
    DDS_UnsignedLong currentCount;
    DDS_Long currentCountChange;
    DDS_InstanceHandle lastPublicationHandle;
};

enum DDS_SampleRejectedStatusKind {
    DDS_NOT_REJECTED,
    DDS_REJECTED_BY_INSTANCES_LIMIT,
    DDS_REJECTED_BY_SAMPLES_LIMIT,
    DDS_REJECTED_BY_SAMPLES_PER_INSTANCE_LIMIT
};

struct DDS_SampleRejectedStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
    DDS_SampleRejectedStatusKind lastReason;
    DDS_InstanceHandle lastInstanceHandle;
};

struct DDS_LivelinessChangedStatus {
    DDS_UnsignedLong aliveCount;
    DDS_UnsignedLong notAliveCount;
    DDS_Long aliveCountChange;
    DDS_Long notAliveCountChange;
    DDS_InstanceHandle lastPublicationHandle;
};

struct DDS_RequestedDeadlineMissedStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
    DDS_InstanceHandle last_instance_handle;
};

struct DDS_RequestedIncompatibleQosStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
    DDS_UnsignedLong lastPolicyId;
};

struct DDS_SampleLostStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
};

struct DDS_InconsistentTopicStatus {
    DDS_UnsignedLong totalCount;
    DDS_Long totalCountChange;
};

using DDS_GuidPrefix = uint8_t[12];  /* 12 is the design elements number */

struct DDS_GuidEntityId {
    uint8_t entityKey[3]; /* 3 is the num of entity */
    uint8_t entityKind;
};

struct DDS_Guid {
    DDS_GuidPrefix prefix;
    DDS_GuidEntityId entityId;
};

struct DDS_Time {
    DDS_Long sec;
    DDS_UnsignedLong nanosec;
};

struct DDS_Duration {
    DDS_Long sec;
    DDS_UnsignedLong nanosec;
};

extern const DDS_Octet DDS_DISCOVERY_FILTER_MIN_ID;
extern const DDS_Octet DDS_DISCOVERY_FILTER_MAX_ID;
extern const DDS_UnsignedLong DDS_DISCOVERY_FILTER_MAX_LEN;
extern const DDS_Octet DDS_TRANSPORT_MAX_PRIORITY;
extern const DDS_Long DDS_RESOURCE_LIMIT_INFINITE;
extern const DDS_Long DDS_HISTORY_DEPTH_DEFAULT;

extern const DDS_Long DDS_DURATION_INFINITE_SEC;
extern const DDS_UnsignedLong DDS_DURATION_INFINITE_NSEC;
extern const DDS_Duration DDS_DURATION_INFINITE;

extern const DDS_Long DDS_DURATION_ZERO_SEC;
extern const DDS_UnsignedLong DDS_DURATION_ZERO_NSEC;
extern const DDS_Duration DDS_DURATION_ZERO;

extern const DDS_Duration DDS_MAX_BLOCKING_TIME_DEFAULT;
extern const DDS_Duration DDS_DURATION_BUILTIN_HEARTBEAT_DEFAULT;
extern const DDS_StatusMask DDS_STATUS_MASK_ALL;
extern const DDS_StatusMask DDS_STATUS_MASK_NONE;

extern const DDS_UnsignedLong DDS_BANDWIDTH_DEFAULT;
extern const DDS_UnsignedLong DDS_SENDWINDOW_DEFAULT;

enum DDS_SampleStateKind {
    DDS_READ_SAMPLE_STATE = 0x0001,
    DDS_NOT_READ_SAMPLE_STATE = 0x0001 << 1
};

using DDS_SampleStateMask = DDS_UnsignedLong;

extern const DDS_SampleStateMask DDS_ANY_SAMPLE_STATE;

enum DDS_ViewStateKind {
    DDS_NEW_VIEW_STATE = 0x0001,
    DDS_NOT_NEW_VIEW_STATE = 0x0001 << 1
};

using DDS_ViewStateMask = DDS_UnsignedLong;

extern const DDS_ViewStateMask DDS_ANY_VIEW_STATE;

enum DDS_InstanceStateKind {
    DDS_ALIVE_INSTANCE_STATE = 0x0001,
    DDS_NOT_ALIVE_DISPOSED_INSTANCE_STATE = 0x0001 << 1,
    DDS_NOT_ALIVE_NO_WRITERS_INSTANCE_STATE = 0x0001 << 2
};

using DDS_InstanceStateMask = DDS_UnsignedLong;

extern const DDS_InstanceStateMask DDS_ANY_INSTANCE_STATE;
extern const DDS_InstanceStateMask DDS_NOT_ALIVE_INSTANCE_STATE;

enum DDS_TypeKind {
    DDS_TK_NULL,
    DDS_TK_SHORT,
    DDS_TK_LONG,
    DDS_TK_USHORT,
    DDS_TK_ULONG,
    DDS_TK_FLOAT,
    DDS_TK_DOUBLE,
    DDS_TK_BOOLEAN,
    DDS_TK_CHAR,
    DDS_TK_OCTET,
    DDS_TK_STRUCT,
    DDS_TK_UNION,
    DDS_TK_ENUM,
    DDS_TK_STRING,
    DDS_TK_SEQUENCE,
    DDS_TK_ARRAY,
    DDS_TK_ALIAS,
    DDS_TK_LONGLONG,
    DDS_TK_ULONGLONG,
    DDS_TK_LONGDOUBLE,
    DDS_TK_WCHAR,
    DDS_TK_WSTRING,
    DDS_TK_VALUE,
    DDS_TK_OCTETS
};

struct DDS_TypeCode {
    const DDS_TypeKind kind;
    const char *name;
    const DDS_UnsignedLong offset;
    const DDS_TypeKind arrKind;         /* array kind */
    const DDS_UnsignedShort arrSize;    /* array size */
};

struct DDS_Config {
    std::string name;
    std::string value;
};

using DDS_ConfigSeq = std::vector<DDS_Config>;

#endif /* RT_DDS_DDS_CPP_COMMON_H */

