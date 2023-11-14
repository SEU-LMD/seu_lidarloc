/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_qosprovider.h
 */

#ifndef RT_DDS_DDS_CPP_QOSPROVIDER_H
#define RT_DDS_DDS_CPP_QOSPROVIDER_H

#include "RT-DDS/dds_cpp/dds_cpp_qos.h"

class DDS_QosProvider {
public:
    virtual ~DDS_QosProvider()
    {}

    virtual DDS_ReturnCode GetParticipantQos(DDS_DomainParticipantQos &qos, const char *name) = 0;

    virtual DDS_ReturnCode GetTopicQos(DDS_TopicQos &qos, const char *name) = 0;

    virtual DDS_ReturnCode GetPublisherQos(DDS_PublisherQos &qos, const char *name) = 0;

    virtual DDS_ReturnCode GetSubscriberQos(DDS_SubscriberQos &qos, const char *name) = 0;

    virtual DDS_ReturnCode GetDataWriterQos(DDS_DataWriterQos &qos, const char *name) = 0;

    virtual DDS_ReturnCode GetDataReaderQos(DDS_DataReaderQos &qos, const char *name) = 0;
};

class DDS_QosProviderFactory {
public:
    virtual ~DDS_QosProviderFactory()
    {}

    static DDS_QosProviderFactory *GetInstance();

    virtual DDS_QosProvider *CreateQosProvider(const char *uri, const char *profile) = 0;

    virtual DDS_ReturnCode DeleteQosProvider(DDS_QosProvider *qosProvider) = 0;
};

#endif /* RT_DDS_DDS_CPP_QOSPROVIDER_H */

