/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_topic.h
 */

#ifndef RT_DDS_DDS_CPP_TOPIC_H
#define RT_DDS_DDS_CPP_TOPIC_H

#include "RT-DDS/dds_cpp/dds_cpp_infrastruture.h"
#include "RT-DDS/dds_cpp/dds_cpp_qos.h"

class DDS_DomainParticipant;

class DDS_TopicListener;

class DDS_TopicDescriptionImpl;

class DDS_TopicDescription {
public:
    /**
     * @brief Get the underlying implementation of TopicDescription.
     * @return pointer to DDS_TopicDescriptionImpl
     */
    virtual DDS_TopicDescriptionImpl *GetTopicDescriptionImpl() = 0;

    /**
     * @brief Get topic name.
     * @return pointer to topic name
     */
    virtual const char *GetName() = 0;

    /**
     * @brief Get type name of topic.
     * @return pointer to type name
     */
    virtual const char *GetTypeName() = 0;

    /**
     * @brief Get participant of topic.
     * @return pointer to DDS_DomainParticipant
     */
    virtual const DDS_DomainParticipant *GetParticipant() = 0;

protected:
    /**
     * @brief Destructor of DDS_TopicDescription.
     */
    virtual ~DDS_TopicDescription()
    {}
};

class DDS_Topic : public DDS_Entity, public DDS_TopicDescription {
public:
    /**
     * @brief Set topic qos.
     * @param[out] qos DDS_TopicQos
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetQos(const DDS_TopicQos &qos) = 0;

    /**
     * @brief Get topic qos.
     * @param[out] qos DDS_TopicQos
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetQos(DDS_TopicQos &qos) = 0;

    /**
     * @brief Set topic listener.
     * @param[in]  mask    DDS_StatusMask
     * @param[in] listener DDS_TopicListener
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetListener(DDS_TopicListener *listener, DDS_StatusMask mask) = 0;

    /**
     * @brief Get topic listener.
     * @return pointer to DDS_TopicListener
     */
    virtual DDS_TopicListener *GetListener() = 0;

    /**
     * @brief Get inconsistent topic status.
     * @param[out] status DDS_InconsistentTopicStatus
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetInconsistentTopicStatus(DDS_InconsistentTopicStatus *status) = 0;

protected:
    /**
     * @brief Destructor of DDS_Topic.
     */
    ~DDS_Topic() override = default;
};

struct DDS_TopicType {
    const DDS_UnsignedShort fieldNum;
    const DDS_TypeCode *pTypeCode;
    const DDS_UnsignedLong size;
};

class DDS_TopicListener : public virtual DDS_Listener {
public:
    /**
     * @brief Listener for on inconsistent topic status.
     * @param[in] topic  DDS_Topic
     * @param[in] status DDS_InconsistentTopicStatus
     */
    virtual void OnInconsistentTopic(DDS_Topic *topic, const DDS_InconsistentTopicStatus &status)
    {
        static_cast<void>(topic);
        static_cast<void>(status);
    }

    /**
     * @brief Destructor of DDS_TopicListener.
     */
    ~DDS_TopicListener() override = default;
};

#endif /* RT_DDS_DDS_CPP_TOPIC_H */

