/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: rtdds_cpp_namespace.h
 */

#ifndef RTDDS_CPP_NAMESPACE_H
#define RTDDS_CPP_NAMESPACE_H

#include "RT-DDS/rtdds_cpp.h"

namespace DDS {
    using Char = DDS_Char;
    using Octet = DDS_Octet;
    using Wchar = DDS_Wchar;
    using Short = DDS_Short;
    using UnsignedShort = DDS_UnsignedShort;
    using Long = DDS_Long;
    using UnsignedLong = DDS_UnsignedLong;
    using LongLong = DDS_LongLong;
    using UnsignedLongLong = DDS_UnsignedLongLong;
    using Float = DDS_Float;
    using Double = DDS_Double;
    using Boolean = DDS_Boolean;

    extern const Boolean BOOLEAN_TRUE;
    extern const Boolean BOOLEAN_FALSE;

    using ReturnCode = DDS_ReturnCode;
    using InstanceHandle = DDS_InstanceHandle;
    using DomainId = DDS_DomainId;
    using DataSeq = DDS_DataSeq;

    using BuiltinTopicKey = DDS_BuiltinTopicKey;

    using InstanceHandleSeq = DDS_InstanceHandleSeq;
    using StringSeq = DDS_StringSeq;

    using Duration = DDS_Duration;
    using Time = DDS_Time;

    using StatusMask = DDS_StatusMask;
    using StatusKind = DDS_StatusKind;
    extern const StatusMask STATUS_MASK_NONE;
    extern const StatusMask STATUS_MASK_ALL;

    extern const StatusKind INCONSISTENT_TOPIC_STATUS;
    extern const StatusKind OFFERED_DEADLINE_MISSED_STATUS;
    extern const StatusKind REQUESTED_DEADLINE_MISSED_STATUS;
    extern const StatusKind OFFERED_INCOMPATIBLE_QOS_STATUS;
    extern const StatusKind REQUESTED_INCOMPATIBLE_QOS_STATUS;
    extern const StatusKind SAMPLE_LOST_STATUS;
    extern const StatusKind SAMPLE_REJECTED_STATUS;
    extern const StatusKind DATA_ON_READERS_STATUS;
    extern const StatusKind DATA_AVAILABLE_STATUS;
    extern const StatusKind LIVELINESS_LOST_STATUS;
    extern const StatusKind LIVELINESS_CHANGED_STATUS;
    extern const StatusKind PUBLICATION_MATCHED_STATUS;
    extern const StatusKind SUBSCRIPTION_MATCHED_STATUS;
    extern const StatusKind SHM_CREATED_STATUS;

    using InconsistentTopicStatus = DDS_InconsistentTopicStatus;
    using SampleLostStatus = DDS_SampleLostStatus;
    using SampleRejectedStatusKind = DDS_SampleRejectedStatusKind;
    using SampleRejectedStatus = DDS_SampleRejectedStatus;
    using RequestedDeadlineMissedStatus = DDS_RequestedDeadlineMissedStatus;
    using LivelinessChangedStatus = DDS_LivelinessChangedStatus;
    using LivelinessLostStatus = DDS_LivelinessLostStatus;
    using OfferedDeadlineMissedStatus = DDS_OfferedDeadlineMissedStatus;
    using OfferedIncompatibleQosStatus = DDS_OfferedIncompatibleQosStatus;
    using RequestedIncompatibleQosStatus = DDS_RequestedIncompatibleQosStatus;
    using PublicationMatchedStatus = DDS_PublicationMatchedStatus;
    using SubscriptionMatchedStatus = DDS_SubscriptionMatchedStatus;

    /********************
     * QOS POLICY KIND
     ********************/
    using DurabilityQosKind = DDS_DurabilityQosKind;
    extern const DurabilityQosKind VOLATILE_DURABILITY_QOS;
    extern const DurabilityQosKind TRANSIENT_LOCAL_DURABILITY_QOS;
    extern const DurabilityQosKind TRANSIENT_DURABILITY_QOS;
    extern const DurabilityQosKind PERSISTENT_DURABILITY_QOS;

    using HistoryQosKind = DDS_HistoryQosKind;
    extern const DDS_HistoryQosKind KEEP_LAST_HISTORY_QOS;
    extern const DDS_HistoryQosKind KEEP_ALL_HISTORY_QOS;

    using LivelinessQosKind = DDS_LivelinessQosKind;
    extern const LivelinessQosKind AUTOMATIC_LIVELINESS_QOS;
    extern const LivelinessQosKind MANUAL_BY_PARTICIPANT_LIVELINESS_QOS;
    extern const LivelinessQosKind MANUAL_BY_TOPIC_LIVELINESS_QOS;

    using ReliabilityQosKind = DDS_ReliabilityQosKind;
    extern const ReliabilityQosKind BEST_EFFORT_RELIABILITY_QOS;
    extern const ReliabilityQosKind RELIABLE_RELIABILITY_QOS;

    using DestinationOrderQosKind = DDS_DestinationOrderQosKind;
    extern const DestinationOrderQosKind BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS;
    extern const DestinationOrderQosKind BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS;

    /********************
    * QOS POLICY STRUCT
    ********************/
    using DeadlineQos = DDS_DeadlineQos;
    using IgnoreLocalParticipantQos = DDS_IgnoreLocalParticipantQos;
    using DestinationOrderQos = DDS_DestinationOrderQos;
    using HistoryQos = DDS_HistoryQos;
    using ResourceLimitsQos = DDS_ResourceLimitsQos;
    using TransportPriorityQos = DDS_TransportPriorityQos;
    using LifespanQos = DDS_LifespanQos;
    using DurabilityQos = DDS_DurabilityQos;
    using LivelinessQos = DDS_LivelinessQos;
    using PartitionQos = DDS_PartitionQos;
    using ReliabilityQos = DDS_ReliabilityQos;
    using DurabilityServiceQos = DDS_DurabilityServiceQos;
    using WriterDataLifecycleQos = DDS_WriterDataLifecycleQos;
    using ReaderDataLifecycleQos = DDS_ReaderDataLifecycleQos;
    using UserDataQos = DDS_UserDataQos;
    using DiscoveryFilterQos = DDS_DiscoveryFilterQos;

    using DomainParticipantQos = DDS_DomainParticipantQos;
    using TopicQos = DDS_TopicQos;
    using DataWriterQos = DDS_DataWriterQos;
    using PublisherQos = DDS_PublisherQos;
    using DataReaderQos = DDS_DataReaderQos;
    using SubscriberQos = DDS_SubscriberQos;

    using SampleStateKind = DDS_SampleStateKind;
    using SampleStateMask = DDS_SampleStateMask;
    extern const SampleStateMask ANY_SAMPLE_STATE;
    extern const SampleStateKind READ_SAMPLE_STATE;
    extern const SampleStateKind NOT_READ_SAMPLE_STATE;

    using ViewStateKind = DDS_ViewStateKind;
    using ViewStateMask = DDS_ViewStateMask;
    extern const ViewStateMask ANY_VIEW_STATE;
    extern const ViewStateKind NEW_VIEW_STATE;
    extern const ViewStateKind NOT_NEW_VIEW_STATE;

    using InstanceStateKind = DDS_InstanceStateKind;
    using InstanceStateMask = DDS_InstanceStateMask;
    extern const InstanceStateMask ANY_INSTANCE_STATE;
    extern const InstanceStateMask NOT_ALIVE_INSTANCE_STATE;
    extern const InstanceStateKind ALIVE_INSTANCE_STATE;
    extern const InstanceStateKind NOT_ALIVE_DISPOSED_INSTANCE_STATE;
    extern const InstanceStateKind NOT_ALIVE_NO_WRITERS_INSTANCE_STATE;

    using Entity = DDS_Entity;
    using WaitSet = DDS_WaitSet;
    using Condition = DDS_Condition;
    using StatusCondition = DDS_StatusCondition;
    using Listener = DDS_Listener;

    using DomainParticipantFactory = DDS_DomainParticipantFactory;
    using DomainParticipant = DDS_DomainParticipant;
    using DomainParticipantListener = DDS_DomainParticipantListener;

    using TopicDescription = DDS_TopicDescription;
    using Topic = DDS_Topic;
    using TopicListener = DDS_TopicListener;

    using Subscriber = DDS_Subscriber;
    using SubscriberListener = DDS_SubscriberListener;
    using DataReader = DDS_DataReader;
    using DataReaderListener = DDS_DataReaderListener;
    using ReadCondition = DDS_ReadCondition;
    using SampleInfo = DDS_SampleInfo;

    using Publisher = DDS_Publisher;
    using PublisherListener = DDS_PublisherListener;
    using DataWriter = DDS_DataWriter;
    using DataWriterListener = DDS_DataWriterListener;

    using ConditionSeq = DDS_ConditionSeq;
    using SampleInfoSeq = DDS_SampleInfoSeq;

    using QosProviderFactory = DDS_QosProviderFactory;
    using QosProvider = DDS_QosProvider;

    extern const DomainParticipantQos PARTICIPANT_QOS_DEFAULT;
    extern const TopicQos TOPIC_QOS_DEFAULT;
    extern const PublisherQos PUBLISHER_QOS_DEFAULT;
    extern const SubscriberQos SUBSCRIBER_QOS_DEFAULT;
    extern const DataReaderQos DATAREADER_QOS_DEFAULT;
    extern const DataWriterQos DATAWRITER_QOS_DEFAULT;

    using GuidPrefix = DDS_GuidPrefix;
    using GuidEntityId = DDS_GuidEntityId;
    using Guid = DDS_Guid;

    /********************
     * Builtin Support
     ********************/
    using ParticipantBuiltinTopicDataDataReader = DDS_ParticipantBuiltinTopicDataDataReader;
    using PublicationBuiltinTopicDataDataReader = DDS_PublicationBuiltinTopicDataDataReader;
    using SubscriptionBuiltinTopicDataDataReader = DDS_SubscriptionBuiltinTopicDataDataReader;
    using ParticipantBuiltinTopicData = DDS_ParticipantBuiltinTopicData;
    using PublicationBuiltinTopicData = DDS_PublicationBuiltinTopicData;
    using SubscriptionBuiltinTopicData = DDS_SubscriptionBuiltinTopicData;
    using ParticipantBuiltinTopicDataSeq = DDS_ParticipantBuiltinTopicDataSeq;
    using PublicationBuiltinTopicDataSeq = DDS_PublicationBuiltinTopicDataSeq;
    using SubscriptionBuiltinTopicDataSeq = DDS_SubscriptionBuiltinTopicDataSeq;

    extern const char *PARTICIPANT_TOPIC_NAME;
    extern const char *PUBLICATION_TOPIC_NAME;
    extern const char *SUBSCRIPTION_TOPIC_NAME;

    /********************
     * Octets Support
    ********************/
    using Octets = DDS_Octets;
}

#endif /* RTDDS_CPP_NAMESPACE_H */

