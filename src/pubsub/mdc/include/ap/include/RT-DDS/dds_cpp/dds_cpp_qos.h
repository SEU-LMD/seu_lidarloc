/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_qos.h
 */

#ifndef RT_DDS_DDS_CPP_QOS_H
#define RT_DDS_DDS_CPP_QOS_H

#include "RT-DDS/dds_cpp/dds_cpp_common.h"
#include "RT-DDS/dds_cpp/dds_cpp_fooseq.h"

using DDS_String = std::string;

struct DDS_UserDataQos {
    DDS_OctetSeq value;
};

struct DDS_GroupDataQos {
    DDS_OctetSeq value;
};

struct DDS_TopicDataQos {
    DDS_OctetSeq value;
};


struct DDS_TransportPriorityQos {
    DDS_Octet value;
};

struct DDS_RtpsReliableReaderProtocol {
    DDS_Duration minHeartbeatResponseDelay;
    DDS_Duration maxHeartbeatResponseDelay;
};

struct DDS_DataReaderProtocolQos {
    DDS_RtpsReliableReaderProtocol rtpsReliableReader;
};

struct DDS_RtpsSPDPProtocol {
    DDS_Octet announcements;
    DDS_UnsignedLong minAnnouncementInterval;
    DDS_UnsignedLong maxAnnouncementInterval;
};

struct DDS_RtpsReliableWriterProtocol {
    DDS_Duration heartbeatPeriod;
    /* private protocol, receive spdp and the delay this duration then response sedp&spdp */
    DDS_Duration spdpResponseDelay;
};

struct DDS_DataWriterProtocolQos {
    DDS_RtpsReliableWriterProtocol rtpsReliableWriter;
};

struct DDS_LifespanQos {
    DDS_Duration duration;
};

enum DDS_DurabilityQosKind {
    DDS_VOLATILE_DURABILITY_QOS,
    DDS_TRANSIENT_LOCAL_DURABILITY_QOS,
};

struct DDS_DurabilityQos {
    DDS_DurabilityQosKind kind;
};

struct DDS_DeadlineQos {
    DDS_Duration period;
};

enum DDS_LivelinessQosKind {
    DDS_AUTOMATIC_LIVELINESS_QOS,
    DDS_MANUAL_BY_PARTICIPANT_LIVELINESS_QOS,
    DDS_MANUAL_BY_TOPIC_LIVELINESS_QOS
};

struct DDS_LivelinessQos {
    DDS_LivelinessQosKind kind;
    DDS_Duration leaseDuration;
};

struct DDS_PartitionQos {
    DDS_StringSeq name;
};

struct DDS_DiscoveryQos {
    DDS_StringSeq initialPeers;
    DDS_StringSeq multicastReceiveAddresses;
};

enum DDS_ReliabilityQosKind {
    DDS_BEST_EFFORT_RELIABILITY_QOS,
    DDS_RELIABLE_RELIABILITY_QOS
};

struct DDS_ReliabilityQos {
    DDS_ReliabilityQosKind kind;
    DDS_Duration maxBlockingTime;
};

enum DDS_DestinationOrderQosKind {
    DDS_BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS,
    DDS_BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS
};

struct DDS_DestinationOrderQos {
    DDS_DestinationOrderQosKind kind;
};

enum DDS_HistoryQosKind {
    DDS_KEEP_LAST_HISTORY_QOS,
    DDS_KEEP_ALL_HISTORY_QOS
};

struct DDS_HistoryQos {
    DDS_HistoryQosKind kind;
    DDS_Long depth;
};

struct DDS_ResourceLimitsQos {
    DDS_Long maxSamples;
    DDS_Long maxInstances;
    DDS_Long maxSamplesPerInstance;
};

struct DDS_WriterDataLifecycleQos {
    DDS_Boolean autodisposeUnregisteredInstances;
};

struct DDS_ReaderDataLifecycleQos {
    DDS_Duration autopurgeNoWriterSamplesDelay;
    DDS_Duration autopurgeDisposedSamplesDelay;
};

struct DDS_DurabilityServiceQos {
    DDS_Duration serviceCleanupDelay;
    DDS_HistoryQosKind historyKind;
    DDS_Long historyDepth;
    DDS_Long maxSamples;
    DDS_Long maxInstances;
    DDS_Long maxSamplesPerInstance;
};

struct DDS_IgnoreLocalParticipantQos {
    DDS_Boolean value;
};

enum DDS_TransportQosKind {
    DDS_TRANSPORT_UDP = 0x0001,
    DDS_TRANSPORT_ICC = 0x0001 << 1,
    DDS_TRANSPORT_DATA_PLAIN = 0x0001 << 2
};

struct DDS_TransportQos {
    DDS_UnsignedLong mask;
};

enum DDS_TransportQosChannel {
    DDS_TRANS_CHANNEL_UDP           = 0x0001,
    DDS_TRANS_CHANNEL_SHM           = 0x0002,
    DDS_TRANS_CHANNEL_DSHM          = 0x0004,
    DDS_TRANS_CHANNEL_ICC           = 0x0008,
    DDS_TRANS_CHANNEL_UDP_SHM       = DDS_TRANS_CHANNEL_UDP | DDS_TRANS_CHANNEL_SHM,
    DDS_TRANS_CHANNEL_UDP_DSHM      = DDS_TRANS_CHANNEL_UDP | DDS_TRANS_CHANNEL_DSHM,
    DDS_TRANS_CHANNEL_UDP_ICC       = DDS_TRANS_CHANNEL_UDP | DDS_TRANS_CHANNEL_ICC,
    DDS_TRANS_CHANNEL_SHM_ICC       = DDS_TRANS_CHANNEL_SHM | DDS_TRANS_CHANNEL_ICC,
    DDS_TRANS_CHANNEL_UDP_SHM_ICC   = DDS_TRANS_CHANNEL_UDP | DDS_TRANS_CHANNEL_SHM | DDS_TRANS_CHANNEL_ICC,
    DDS_TRANS_CHANNEL_DSHM_ICC      = DDS_TRANS_CHANNEL_DSHM | DDS_TRANS_CHANNEL_ICC,
    DDS_TRANS_CHANNEL_UDP_DSHM_ICC  = DDS_TRANS_CHANNEL_UDP | DDS_TRANS_CHANNEL_DSHM | DDS_TRANS_CHANNEL_ICC,
    DDS_TRANS_CHANNEL_UDP2DSHM      = 0x0015,
};

enum DDS_TransportQosShmPer {
    DDS_SHM_FILE_RWRWOO_MODE = 432U,    /* SHM File Permission 660 */
    DDS_SHM_FILE_RWRWRW_MODE = 438U,    /* SHM File Permission 666 */
};

struct DDS_TransChannelQos {
    DDS_UnsignedLong channel;
    DDS_UnsignedLong shmId;
    DDS_UnsignedLong fragSize;
    DDS_UnsignedLong listSize;
    DDS_UnsignedLong shmPer;
    DDS_Boolean weakShmPara;
    DDS_Boolean determinate;
    DDS_Boolean enableUDPMulticast;
    DDS_String multicastAddr;
};

struct DDS_TransportInterfacesQos {
    DDS_StringSeq interfaces;
};

struct DDS_DiscoveryFilterQos {
    DDS_Octet classificationId;
    DDS_String classificationIdFilter;
};

struct DDS_DiscoveryConfigQos {
    DDS_RtpsReliableReaderProtocol publicationReader;
    DDS_RtpsReliableReaderProtocol subscriptionReader;
    DDS_RtpsReliableWriterProtocol publicationWriter;
    DDS_RtpsReliableWriterProtocol subscriptionWriter;
    DDS_RtpsSPDPProtocol announcementConfig;
};

enum DDS_OwnershipQosKind {
    DDS_SHARED_OWNERSHIP_QOS,
    DDS_EXCLUSIVE_OWNERSHIP_QOS
};

struct DDS_OwnershipQos {
    DDS_OwnershipQosKind kind;
};

struct DDS_OwnershipStrengthQos {
    DDS_Long value;
};

struct DDS_EntityFactoryQos {
    DDS_Boolean autoenableCreatedEntities;
};

struct DDS_ConfigQos {
    DDS_ConfigSeq value;
};

struct DDS_DomainParticipantQos {
    DDS_EntityFactoryQos entityFactory;
    DDS_UserDataQos userData;
    DDS_TransportQos transport;
    DDS_DiscoveryConfigQos discoveryConfig;
    DDS_DiscoveryQos discovery;
    DDS_ConfigQos config;
    DDS_TransportInterfacesQos transportInterfaces;
    DDS_DiscoveryFilterQos discoveryFilter;
};

enum DDS_PresentationQosAccessScopeKind {
    DDS_INSTANCE_PRESENTATION_QOS,
    DDS_TOPIC_PRESENTATION_QOS,
    DDS_GROUP_PRESENTATION_QOS
};

struct DDS_PresentationQos {
    DDS_PresentationQosAccessScopeKind accessScope;
    bool coherentAccess;
    bool orderedAccess;
};

struct DDS_LatencyBudgetQos {
    DDS_Duration duration;
};

struct DDS_AuthenticationQos {
    /*
     * It is usually recommended to only use usernames that begin with a lower case letter or an underscore,
     * followed by lower case letters, digits, underscores, or dashes. They can end with a dollar sign. In
     * regular expression terms: [a-z_][a-z0-9_-]*[$]?
     */
    DDS_String shmFileOwner;
};

struct DDS_RelatedEntity {
    DDS_Guid guid;
};

struct DDS_BandwidthControl {
    DDS_UnsignedLong bandwidth;
    DDS_UnsignedLong sendWindow;
};

struct DDS_ExtensionQos {
    DDS_RelatedEntity relatedGuid;
    DDS_BandwidthControl bandwidthControl;
};

struct DDS_SubscriberQos {
    DDS_EntityFactoryQos entityFactory;
    DDS_PartitionQos partition;
};

struct DDS_PublisherQos {
    DDS_EntityFactoryQos entityFactory;
    DDS_PartitionQos partition;
};

struct DDS_TopicQos {
    DDS_DurabilityQos durability;
    DDS_DurabilityServiceQos durabilityService;
    DDS_DeadlineQos deadline;
    DDS_LivelinessQos liveliness;
    DDS_ReliabilityQos reliability;
    DDS_DestinationOrderQos destinationOrder;
    DDS_HistoryQos history;
    DDS_ResourceLimitsQos resourceLimits;
    DDS_TransportPriorityQos transportPriority;
    DDS_LifespanQos lifespan;
    DDS_TopicDataQos topicData;
};

struct DDS_DataWriterQos {
    DDS_DurabilityQos durability;
    DDS_DurabilityServiceQos durabilityService;
    DDS_DeadlineQos deadline;
    DDS_LivelinessQos liveliness;
    DDS_ReliabilityQos reliability;
    DDS_DestinationOrderQos destinationOrder;
    DDS_HistoryQos history;
    DDS_ResourceLimitsQos resourceLimits;
    DDS_TransportPriorityQos transportPriority;
    DDS_LifespanQos lifespan;
    DDS_UserDataQos userData;
    DDS_WriterDataLifecycleQos writerDataLifecycle;
    DDS_TransportQos transport;
    DDS_TransChannelQos transChannel;
    DDS_DataWriterProtocolQos protocol;
    DDS_OwnershipQos ownership;
    DDS_OwnershipStrengthQos ownershipStrength;
    DDS_LatencyBudgetQos latencyBudget;
    DDS_AuthenticationQos authentication;
    DDS_ExtensionQos extension;
};

struct DDS_DataReaderQos {
    DDS_DurabilityQos durability;
    DDS_DeadlineQos deadline;
    DDS_LivelinessQos liveliness;
    DDS_ReliabilityQos reliability;
    DDS_DestinationOrderQos destinationOrder;
    DDS_HistoryQos history;
    DDS_ResourceLimitsQos resourceLimits;
    DDS_UserDataQos userData;
    DDS_ReaderDataLifecycleQos readerDataLifecycle;
    DDS_TransportQos transport;
    DDS_TransChannelQos transChannel;
    DDS_DataReaderProtocolQos protocol;
    DDS_IgnoreLocalParticipantQos ignoreLocalParticipant;
    DDS_TransportPriorityQos transportPriority;
    DDS_ExtensionQos extension;
};

/* Default QoS definitions */
extern const DDS_DomainParticipantQos DDS_PARTICIPANT_QOS_DEFAULT;
extern const DDS_TopicQos DDS_TOPIC_QOS_DEFAULT;
extern const DDS_PublisherQos DDS_PUBLISHER_QOS_DEFAULT;
extern const DDS_SubscriberQos DDS_SUBSCRIBER_QOS_DEFAULT;
extern const DDS_DataReaderQos DDS_DATAREADER_QOS_DEFAULT;
extern const DDS_DataWriterQos DDS_DATAWRITER_QOS_DEFAULT;

#endif /* RT_DDS_DDS_CPP_QOS_H */

