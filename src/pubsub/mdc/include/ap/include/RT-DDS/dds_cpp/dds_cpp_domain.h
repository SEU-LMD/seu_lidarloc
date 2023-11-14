/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_domain.h
 */

#ifndef RT_DDS_DDS_CPP_DOMAIN_H
#define RT_DDS_DDS_CPP_DOMAIN_H

#include <memory>

#include "RT-DDS/dds_cpp/dds_cpp_infrastruture.h"
#include "RT-DDS/dds_cpp/dds_cpp_publication.h"
#include "RT-DDS/dds_cpp/dds_cpp_subscription.h"
#include "RT-DDS/dds_cpp/dds_cpp_topic.h"

class DDS_DomainParticipantListener : public virtual DDS_PublisherListener, public virtual DDS_SubscriberListener {
public:
    ~DDS_DomainParticipantListener() override = default;
};

class DDS_DomainParticipant : public DDS_Entity {
public:
    /**
     * @brief This operation creates a DDS_Publisher with the desired QoS
     *        policies and attaches to it the specified DDS_PublisherListener.
     * @param[in] qos QoS to be used for creating the new DDS_Publisher.
     * @param[in] listener Listener to be attached to the newly created
     *                     DDS_Publisher.
     * @param[in] mask Changes of communication status.
     * @return newly created publisher object or nullptr on failure.
     */
    virtual DDS_Publisher *CreatePublisher(const DDS_PublisherQos &qos = DDS_PUBLISHER_QOS_DEFAULT,
                                           DDS_PublisherListener *listener = nullptr,
                                           DDS_StatusMask mask = DDS_STATUS_MASK_NONE) = 0;

    /**
     * @brief Deletes an existing DDS_Publisher.
     * @param publisher DDS_Publisher to be deleted.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_BAD_PARAMETER
     * @retval DDS_RETCODE_PRECONDITION_NOT_MET
     */
    virtual DDS_ReturnCode DeletePublisher(DDS_Publisher *publisher) = 0;

    /**
     * @brief This operation creates a DDS_Subscriber with the desired QoS
     *        policies and attaches to it the specified DDS_SubscriberListener.
     * @param[in] qos QoS to be used for creating the new DDS_Subscriber.
     * @param[in] listener Listener to be attached.
     * @param[in] mask Changes of communication status to be invoked on the
     *                 listener.
     * @return newly created subscriber object or nullptr on failure.
     */
    virtual DDS_Subscriber *CreateSubscriber(const DDS_SubscriberQos &qos = DDS_SUBSCRIBER_QOS_DEFAULT,
                                             DDS_SubscriberListener *listener = nullptr,
                                             DDS_StatusMask mask = DDS_STATUS_MASK_NONE) = 0;

    /**
     * @brief Deletes an existing DDS_Subscriber.
     * @param[in] publisher DDS_Subscriber to be deleted.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_BAD_PARAMETER
     * @retval DDS_RETCODE_PRECONDITION_NOT_MET
     */
    virtual DDS_ReturnCode DeleteSubscriber(DDS_Subscriber *subscriber) = 0;

    /**
     * @brief This operation creates a DDS_Topic with the desired QoS policies
     *        and attaches to it the specified DDS_TopicListener.
     * @param[in] topicName Name for the new topic.
     * @param[in] typeName The type to which the new DDS_Topic will be bound.
     * @param[in] qos QoS to be used for creating the new DDS_Topic.
     * @param[in] listener Listener to be attached.
     * @param[in] mask Changes of communication status to be invoked on the
     *                 listener.
     * @return newly created topic, or nullptr on failure
     */
    virtual DDS_Topic *CreateTopic(const char *topicName,
                                   const char *typeName,
                                   const DDS_TopicQos &qos = DDS_TOPIC_QOS_DEFAULT,
                                   DDS_TopicListener *listener = nullptr,
                                   DDS_StatusMask mask = DDS_STATUS_MASK_NONE) = 0;

    /**
     * @brief Deletes a DDS_Topic.
     * @param[in] topic DDS_Topic to be deleted.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_BAD_PARAMETER
     * @retval DDS_RETCODE_ERROR
     */
    virtual DDS_ReturnCode DeleteTopic(DDS_Topic *topic) = 0;

    /**
     * @brief This operation allows access to the built-in DDS_Subscriber.
     * @details Each DDS_DomainParticipant contains several built-in Topic
     *          objects as well as corresponding DDS_DataReader objects to
     *          access them. All these DDS_DataReader objects belong to a single
     *          built-in DDS_Subscriber.
     * @return The built-in DDS_Subscriber singleton.
     */
    virtual DDS_Subscriber *GetBuiltinSubscriber() = 0;

    /**
     * @brief Instructs the service to locally ignore a remote
     *        DDS_DomainParticipant.
     * @param[in] instanceHandle DDS_InstanceHandle of the DDS_DomainParticipant
     *                           to be ignored.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_ERROR
     */
    virtual DDS_ReturnCode IgnoreParticipant(DDS_InstanceHandle instanceHandle) = 0;

    /**
     * @brief Finds an existing DDS_Topic based on its name.
     * @param[in] topicName Name of the DDS_Topic to search for.
     * @param[in] timeout The time to wait if the DDS_Topic does not exist
     *                    already.
     * @return The topic, if it exists, or nullptr.
     */
    virtual DDS_Topic *FindTopic(const char *topicName, const DDS_Duration &timeout) = 0;

    /**
     * @brief Change the QoS of this DomainParticipant.
     * @param[in] qos Set of policies to be applied to DDS_DomainParticipant.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_ERROR
     */
    virtual DDS_ReturnCode SetQos(const DDS_DomainParticipantQos &qos) = 0;

    /**
     * @brief Get the participant QoS.
     * @param[in,out] qos QoS to be filled up.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetQos(DDS_DomainParticipantQos &qos) = 0;

    /**
     * @brief Sets the participant listener.
     * @param[in] listener Listener to be installed on the entity.
     * @param[in] mask Changes of communication status to be invoked on the
     *                 listener.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetListener(DDS_DomainParticipantListener *listener, DDS_StatusMask mask) = 0;

    /**
     * @brief Get the participant listener.
     * @return Existing listener attached to the DDS_DomainParticipant.
     */
    virtual DDS_DomainParticipantListener *GetListener() = 0;

    /**
     * @brief Get the unique domain identifier.
     * @return The unique domainId that was used to create the domain.
     */
    virtual DDS_DomainId GetDomainId() = 0;

    /**
     * @brief Set the default DDS_PublisherQos values for this DomainParticipant.
     * @param[in] qos Default qos to be set.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetDefaultPublisherQos(const DDS_PublisherQos &qos) = 0;

    /**
     * @brief Copy the default DDS_PublisherQos values into the provided
     *        DDS_PublisherQos instance.
     * @param[in, out] qos Qos to be filled up.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetDefaultPublisherQos(DDS_PublisherQos &qos) = 0;

    /**
     * @brief Set the default DDS_SubscriberQos values for this
     *        DomainParticipant.
     * @param[in] qos Default qos to be set.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetDefaultSubscriberQos(const DDS_SubscriberQos &qos) = 0;

    /**
     * @brief Copy the default DDS_SubscriberQos values into the provided
     *        DDS_SubscriberQos instance.
     * @param[in, out] qos Qos to be filled up.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetDefaultSubscriberQos(DDS_SubscriberQos &qos) = 0;

    /**
     * @brief Set the default DDS_TopicQos values for this DomainParticipant.
     * @param[in] qos Default qos to be set.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetDefaultTopicQos(const DDS_TopicQos &qos) = 0;

    /**
     * @brief Copy the default DDS_SubscriberQos values into the provided
     *        DDS_SubscriberQos instance.
     * @param[in, out] qos Qos to be filled up.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetDefaultTopicQos(DDS_TopicQos &qos) = 0;

    /**
     * @brief Allows an application to communicate the existence of a data type.
     * @param[in] typeName the type name under with the data type.
     * @param topicType the topic type info.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_BAD_PARAMETER
     */
    virtual DDS_ReturnCode RegisterType(const char *typeName, const DDS_TopicType *topicType) = 0;

    /**
     * @brief Delete all the entities that were created by means of the "create"
     *        operations on the DDS_DomainParticipant.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode DeleteContainedEntities() = 0;

    /**
     * @deprecated
     */
    virtual DDS_WaitSet *CreateWaitSet() = 0;

    /**
     * @deprecated
     */
    virtual DDS_ReturnCode DeleteWaitSet(DDS_WaitSet *waitSet) noexcept = 0;

protected:
    ~DDS_DomainParticipant() override = default;
};

class DDS_DomainParticipantFactory : public std::enable_shared_from_this<DDS_DomainParticipantFactory> {
public:
    /**
     * @brief Gets the singleton instance of this class.
     * @return The singleton DDS_DomainParticipantFactory instance.
     * @note the user can get the shared_ptr by res->shared_from_this(),
     * to control the life cycle.
     */
    static DDS_DomainParticipantFactory *GetInstance();

    /**
     * @brief Creates a new DDS_DomainParticipant object.
     * @param[in] domainId ID of the domain that the application intends to
     *                     join.
     * @param[in] qos the DomainParticipant's QoS.
     * @param[in] listener the domain participant's listener.
     * @param[in] mask Changes of communication status to be invoked on the
     *                 listener.
     * @return domain participant or nullptr on failure
     */
    virtual DDS_DomainParticipant *CreateParticipant(
        DDS_DomainId domainId,
        const DDS_DomainParticipantQos &qos = DDS_PARTICIPANT_QOS_DEFAULT,
        DDS_DomainParticipantListener *listener = nullptr,
        DDS_StatusMask mask = DDS_STATUS_MASK_NONE) = 0;

    /**
     * @brief Deletes an existing DDS_DomainParticipant.
     * @param participant DDSDomainParticipant to be deleted.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_BAD_PARAMETER
     * @retval DDS_RETCODE_PRECONDITION_NOT_MET
     */
    virtual DDS_ReturnCode DeleteParticipant(DDS_DomainParticipant *participant) = 0;

    /**
     * @deprecated
     */
    virtual DDS_ReturnCode DeleteParticipantAll(DDS_DomainParticipant *participant) = 0;

    /**
     * @brief Sets the default DDS_DomainParticipantQos values for this domain
     *        participant factory.
     * @param[in] qos QoS to be filled up.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetDefaultParticipantQos(const DDS_DomainParticipantQos &qos) = 0;

    /**
     * @brief Initializes the DDS_DomainParticipantQos instance with default
     *        values.
     * @param[in,out] qos the domain participant's QoS.
     * @return Standard DDS ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetDefaultParticipantQos(DDS_DomainParticipantQos &qos) = 0;

protected:
    DDS_DomainParticipantFactory() = default;

    /**
     * @brief Destructor
     */
    virtual ~DDS_DomainParticipantFactory()
    {}
};

#endif /* RT_DDS_DDS_CPP_DOMAIN_H */

