/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_builtin.h
 */

#ifndef RT_DDS_DDS_CPP_BUILTIN_H
#define RT_DDS_DDS_CPP_BUILTIN_H

#include "RT-DDS/dds_cpp/dds_cpp_infrastruture.h"
#include "RT-DDS/dds_cpp/dds_cpp_usertype.h"

#define DDS_BUILTIN_TOPIC_KEY_TYPE_NATIVE DDS_Long

const DDS_Octet MAX_TOPIC_KEY_CNT = 4;

struct DDS_BuiltinTopicKey {
    DDS_BUILTIN_TOPIC_KEY_TYPE_NATIVE value[MAX_TOPIC_KEY_CNT];
};

struct DDS_ParticipantBuiltinTopicData {
    DDS_BuiltinTopicKey key;
    DDS_UserDataQos userData;
};

struct DDS_PublicationBuiltinTopicData {
    DDS_BuiltinTopicKey key;
    DDS_BuiltinTopicKey participantKey;
    char *topicName;
    char *typeName;
    DDS_ReliabilityQos reliability;
    DDS_DurabilityQos durability;
    DDS_DeadlineQos deadline;
    DDS_HistoryQos history;
};

struct DDS_SubscriptionBuiltinTopicData {
    DDS_BuiltinTopicKey key;
    DDS_BuiltinTopicKey participantKey;
    char *topicName;
    char *typeName;
    DDS_ReliabilityQos reliability;
    DDS_DurabilityQos durability;
    DDS_DeadlineQos deadline;
    DDS_HistoryQos history;
};

class DDS_ParticipantBuiltinTopicDataDataReader;

DDS_SEQUENCE(DDS_ParticipantBuiltinTopicDataSeq, DDS_ParticipantBuiltinTopicData *);

DDS_DATAREADER_CPP(DDS_ParticipantBuiltinTopicDataDataReader,
                   DDS_ParticipantBuiltinTopicDataSeq,
                   DDS_ParticipantBuiltinTopicData);

extern const char *DDS_PARTICIPANT_TOPIC_NAME;

class DDS_PublicationBuiltinTopicDataDataReader;

DDS_SEQUENCE(DDS_PublicationBuiltinTopicDataSeq, DDS_PublicationBuiltinTopicData *);

DDS_DATAREADER_CPP(DDS_PublicationBuiltinTopicDataDataReader,
                   DDS_PublicationBuiltinTopicDataSeq,
                   DDS_PublicationBuiltinTopicData);

extern const char *DDS_PUBLICATION_TOPIC_NAME;

class DDS_SubscriptionBuiltinTopicDataDataReader;

DDS_SEQUENCE(DDS_SubscriptionBuiltinTopicDataSeq, DDS_SubscriptionBuiltinTopicData *);

DDS_DATAREADER_CPP(DDS_SubscriptionBuiltinTopicDataDataReader,
                   DDS_SubscriptionBuiltinTopicDataSeq,
                   DDS_SubscriptionBuiltinTopicData);

extern const char *DDS_SUBSCRIPTION_TOPIC_NAME;

#endif /* RT_DDS_DDS_CPP_BUILTIN_H */

