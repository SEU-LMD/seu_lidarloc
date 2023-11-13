/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_usertype.h
 */

#ifndef RT_DDS_DDS_CPP_USERTYPE_H
#define RT_DDS_DDS_CPP_USERTYPE_H

#include "RT-DDS/dds_cpp/dds_cpp_publication.h"
#include "RT-DDS/dds_cpp/dds_cpp_subscription.h"

class DDSTypeSupport {
protected:
    virtual ~DDSTypeSupport()
    {};
};

#define TYPESUPPORT_CPP(TTypeSupport, TData) DDS_TYPESUPPORT_CPP(TTypeSupport, TData);

#define DDS_TYPESUPPORT_CPP(TTypeSupport, TData) \
class TTypeSupport : public DDSTypeSupport { \
public: \
    static DDS_ReturnCode RegisterType( \
        DDS_DomainParticipant *participant, \
        const char *typeName); \
    static TData *CreateData(); \
    static void DeleteData(TData *data); \
    static const char *GetTypeName(); \
private: \
    TTypeSupport(); \
};

#define DDS_DATAWRITER_CPP_PUBLIC_UNTYPED_METHODS() \
    virtual DDS_ReturnCode GetPublicationMatchedStatus( \
        DDS_PublicationMatchedStatus &status); \
    virtual DDS_Topic* GetTopic(); \
    virtual DDS_Publisher* GetPublisher(); \
    virtual DDS_ReturnCode SetQos(const DDS_DataWriterQos &qos); \
    virtual DDS_ReturnCode GetQos(DDS_DataWriterQos &qos); \
    virtual DDS_ReturnCode SetListener( \
        DDS_DataWriterListener* listener, \
        DDS_StatusMask mask = DDS_STATUS_MASK_ALL); \
    virtual DDS_DataWriterListener* GetListener(); \
    virtual DDS_StatusCondition* GetStatusCondition(); \
    virtual DDS_StatusMask GetStatusChanges(); \
    virtual DDS_InstanceHandle GetInstanceHandle(); \
    virtual DDS_ReturnCode GetGuid(DDS_Guid &guid)

#define DDS_DATAWRITER_CPP_PUBLIC_METHODS(TDataWriter, TData) \
    static TDataWriter* CreateI(DDS_DataWriter *writer); \
    static DDS_ReturnCode DestroyI(TDataWriter *writer); \
    virtual DDS_ReturnCode Write( \
        const TData &data, \
        const DDS_InstanceHandle &handle); \
    virtual DDS_ReturnCode WriteRawData( \
        Mbuf *data); \
    virtual DDS_ReturnCode WriteRawData( \
        Mbuf *data, const DDS_WriteParams &writeParams); \
    virtual DDS_ReturnCode AllocateOctets( \
        const TData &data, DDS_InstanceHandle& handleOfAllocated);  \
    /* The old API for CI DURATION */ \
    DDS_ReturnCode AllocateOctets( \
        const TData &data) {return DDS_RETCODE_ERROR;};  \
    virtual void DeallocateOctets(const DDS_InstanceHandle& theHandle2Deallocate); \
    virtual DDS_ReturnCode Write( \
        const DDS_InstanceHandle &handle); \
    virtual DDS_ReturnCode Write( \
        const DDS_WriteParams &writeParams); \
    virtual DDS_ReturnCode Write( \
        const TData &data, \
        const DDS_WriteParams &writeParams); \

#define DDS_DATAWRITER_CPP(TDataWriter, TData) \
class TDataWriter : public DDS_DataWriter { \
public: \
DDS_DATAWRITER_CPP_PUBLIC_UNTYPED_METHODS(); \
DDS_DATAWRITER_CPP_PUBLIC_METHODS(TDataWriter, TData); \
protected: \
    TDataWriter(DDS_DataWriter* impl); \
    virtual ~TDataWriter(); \
};

#define DATAWRITER_CPP(TDataWriter, TData) DDS_DATAWRITER_CPP(TDataWriter, TData)

#define DDS_DATAREADER_CPP_PUBLIC_UNTYPED_METHODS() \
    virtual DDS_ReadCondition* CreateReadCondition( \
        DDS_SampleStateMask sampleStates, \
        DDS_ViewStateMask viewStates, \
        DDS_InstanceStateMask instanceStates) override; \
    virtual DDS_ReturnCode DeleteReadCondition( \
        DDS_ReadCondition* condition) override; \
    virtual DDS_TopicDescription* GetTopicDescription() override; \
    virtual DDS_Subscriber* GetSubscriber() override; \
    virtual DDS_ReturnCode SetQos(const DDS_DataReaderQos &qos) override; \
    virtual DDS_ReturnCode GetQos(DDS_DataReaderQos &qos) override; \
    virtual DDS_ReturnCode SetListener( \
        DDS_DataReaderListener* listener, \
        DDS_StatusMask mask = DDS_STATUS_MASK_ALL) override; \
    virtual DDS_DataReaderListener* GetListener() override; \
    virtual DDS_ReturnCode GetSampleRejectedStatus( \
        DDS_SampleRejectedStatus &status) override; \
    virtual DDS_ReturnCode GetRequestedDeadlineMissedStatus( \
        DDS_RequestedDeadlineMissedStatus &status) override; \
    virtual DDS_ReturnCode GetRequestedIncompatibleQosStatus( \
        DDS_RequestedIncompatibleQosStatus &status) override; \
    virtual DDS_ReturnCode GetSampleLostStatus( \
        DDS_SampleLostStatus &status) override; \
    virtual DDS_ReturnCode GetSubscriptionMatchedStatus( \
        DDS_SubscriptionMatchedStatus &status) override; \
    virtual DDS_StatusCondition* GetStatusCondition() override; \
    virtual DDS_StatusMask GetStatusChanges() override; \
    virtual DDS_InstanceHandle GetInstanceHandle() override

#define DDS_DATAREADER_CPP(TDataReader, TDataSeq, TData) \
class TDataReader : public DDS_DataReader { \
public: \
    DDS_DATAREADER_CPP_PUBLIC_UNTYPED_METHODS(); \
    DDS_ReturnCode Read(TDataSeq &receivedData, \
        DDS_SampleInfoSeq &infoSeq, \
        DDS_Long maxSamples, \
        DDS_SampleStateMask sampleStates = DDS_ANY_SAMPLE_STATE, \
        DDS_ViewStateMask viewStates = DDS_ANY_VIEW_STATE, \
        DDS_InstanceStateMask instanceStates = DDS_ANY_INSTANCE_STATE); \
    DDS_ReturnCode Take(TDataSeq &receivedData, \
        DDS_SampleInfoSeq &infoSeq, \
        DDS_Long maxSamples, \
        DDS_SampleStateMask sampleStates = DDS_ANY_SAMPLE_STATE, \
        DDS_ViewStateMask viewStates = DDS_ANY_VIEW_STATE, \
        DDS_InstanceStateMask instanceStates = DDS_ANY_INSTANCE_STATE); \
    DDS_ReturnCode TakeRawData(TDataSeq &receivedData, \
        DDS_SampleInfoSeq &infoSeq, \
        DDS_Long maxSamples); \
    DDS_ReturnCode ReadWithCondition(TDataSeq &receivedData, \
        DDS_SampleInfoSeq &infoSeq, \
        DDS_Long maxSamples, \
        DDS_ReadCondition* condition); \
    DDS_ReturnCode TakeWithCondition(TDataSeq &receivedData, \
        DDS_SampleInfoSeq &infoSeq, \
        DDS_Long maxSamples, \
        DDS_ReadCondition* condition); \
    DDS_ReturnCode ReturnLoan(TDataSeq &receivedData, \
        DDS_SampleInfoSeq &infoSeq); \
        DDS_ReturnCode ReturnLoan(const DDS_InstanceHandle& handleOfData2Return); \
public: \
    static TDataReader* CreateI(DDS_DataReader* reader); \
    static DDS_ReturnCode DestroyI(TDataReader* reader) noexcept; \
protected: \
    explicit TDataReader(DDS_DataReader* impl); \
    virtual ~TDataReader(); \
}

#define DATAREADER_CPP(TDataReader, TData, TDataSeq) DDS_DATAREADER_CPP(TDataReader, TDataSeq, TData);

#define DDS_INITIALIZE_TYPECODE(TData, Name, Type)  \
{                                                   \
    Type,                                           \
    #Name,                                          \
    offsetof (TData, Name),                         \
    DDS_TK_NULL,                                    \
    0                                               \
}

#define DDS_INITIALIZE_ARRAY_TYPECODE(TData, Name, SubType, Size)   \
{                                                                   \
    DDS_TK_ARRAY,                                                   \
    #Name,                                                          \
    offsetof (TData, Name),                                         \
    SubType,                                                        \
    Size                                                            \
}

#define DDS_INITIALIZE_SEQUENCE_TYPECODE(TData, Name, SubType)  \
{                                                               \
    DDS_TK_SEQUENCE,                                            \
    #Name,                                                      \
    offsetof (TData, Name),                                     \
    SubType,                                                    \
    0                                                           \
}

#endif /* RT_DDS_DDS_CPP_USERTYPE_H */

