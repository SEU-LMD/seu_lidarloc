/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_subscription.h
 */

#ifndef RT_DDS_DDS_CPP_SUBSCRIPTION_H
#define RT_DDS_DDS_CPP_SUBSCRIPTION_H

#include "RT-DDS/dds_cpp/dds_cpp_infrastruture.h"
#include "RT-DDS/dds_cpp/dds_cpp_qos.h"

#include "plog/PLogDefsAndLimits.hpp"

class DDS_DataReader;

class DDS_ReadCondition : virtual public DDS_Condition {
public:
    /**
     * @brief Get mask of sample state.
     * @return DDS_SampleStateMask(uint32_t)
     */
    virtual DDS_SampleStateMask GetSampleStateMask() = 0;

    /**
     * @brief Get mask of view state.
     * @return DDS_ViewStateMask(uint32_t)
     */
    virtual DDS_ViewStateMask GetViewStateMask() = 0;

    /**
     * @brief Get mask of instance state.
     * @return DDS_InstanceStateMask(uint32_t)
     */
    virtual DDS_InstanceStateMask GetInstanceStateMask() = 0;

    /**
     * @brief Get data reader of the condition.
     * @return pointer to DDS_DataReader
     */
    virtual DDS_DataReader *GetDataReader() = 0;
};

class DDS_DataReaderListener : public virtual DDS_Listener {
public:
    /**
     * @brief Function to be called when requested deadline missed.
     * @param[in] reader
     * @param[in] status status of requested deadline missed
     */
    virtual void OnRequestedDeadlineMissed(DDS_DataReader *reader, const DDS_RequestedDeadlineMissedStatus &status)
    {
        static_cast<void>(reader);
        static_cast<void>(status);
    }

    /**
     * @brief Function to be called when liveliness changed.
     * @param[in] reader
     * @param[in] status status of liveliness changed
     */
    virtual void OnLivelinessChanged(DDS_DataReader *reader, const DDS_LivelinessChangedStatus &status)
    {
        static_cast<void>(reader);
        static_cast<void>(status);
    }

    /**
     * @brief Function to be called when requested incompatible qos.
     * @param[in] reader
     * @param[in] status status of requested incompatible qos
     */
    virtual void OnRequestedIncompatibleQos(DDS_DataReader *reader, const DDS_RequestedIncompatibleQosStatus &status)
    {
        static_cast<void>(reader);
        static_cast<void>(status);
    }

    /**
     * @brief Function to be called when sample rejected.
     * @param[in] reader
     * @param[in] status status of sample rejected
     */
    virtual void OnSampleRejected(DDS_DataReader *reader, const DDS_SampleRejectedStatus &status)
    {
        static_cast<void>(reader);
        static_cast<void>(status);
    }

    /**
     * @brief Function to be called when data available
     * @param[in] reader
     */
    virtual void OnDataAvailable(DDS_DataReader *reader)
    {
        static_cast<void>(reader);
    }

    /**
     * @brief Function to be called when detected matched writer
     * @param[in] reader
     * @param[in] status status of subscription matched
     */
    virtual void OnSubscriptionMatched(DDS_DataReader *reader, const DDS_SubscriptionMatchedStatus &status)
    {
        static_cast<void>(reader);
        static_cast<void>(status);
    }

    /**
     * @brief Function to be called when sample lost
     * @param[in] reader
     * @param[in] status status of sample lost
     */
    virtual void OnSampleLost(DDS_DataReader *reader, const DDS_SampleLostStatus &status)
    {
        static_cast<void>(status);
    }

    /**
     * @brief Destructor of DDS_DataReaderListener
     */
    ~DDS_DataReaderListener() override = default;
};

struct DDS_SampleInfo {
    DDS_SampleStateKind sampleState;
    DDS_ViewStateKind viewState;
    DDS_InstanceStateKind instanceState;
    DDS_Time sourceTimestamp;
    DDS_InstanceHandle instanceHandle;
    DDS_InstanceHandle publicationHandle;
    DDS_Long disposedGenerationCount;
    DDS_Long noWritersGenerationCount;
    DDS_Long sampleRank;
    DDS_Long generationRank;
    DDS_Long absoluteGenerationRank;
    rbs::plog::PlogUid plogUid = rbs::plog::PLOG_UID_MAX;
    DDS_Boolean validData;
};

DDS_SEQUENCE(DDS_SampleInfoSeq, DDS_SampleInfo);

DDS_SEQUENCE(DDS_DataSeq, void*);

class DDS_TopicDescription;
class DDS_Subscriber;

class DDS_DataReader : public DDS_Entity {
public:
    /**
     * @brief Create a read condition to the data reader.
     * @details Create a read condition implement and insert this to reader condition set.
     * @param[in] sampleStates
     * @param[in] viewStates
     * @param[in] instanceStates
     * @return pointer to DDS_ReadCondition
     */
    virtual DDS_ReadCondition *CreateReadCondition(
        DDS_SampleStateMask sampleStates, DDS_ViewStateMask viewStates, DDS_InstanceStateMask instanceStates)
    {
        return impl_->CreateReadCondition(sampleStates, viewStates, instanceStates);
    }

    /**
     * @brief Delete the condition in reader condition set.
     * @param[in] condition
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_BAD_PARAMETER Invalid pointer or condition has been deleted
     * @retval DDS_RETCODE_OK            Delete success
     */
    virtual DDS_ReturnCode DeleteReadCondition(DDS_ReadCondition *condition)
    {
        return impl_->DeleteReadCondition(condition);
    }

    /**
     * @brief Set data reader qos.
     * @param[in] qos data reader qos
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_ERROR update RTPS qos fail
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetQos(const DDS_DataReaderQos &qos)
    {
        return impl_->SetQos(qos);
    }

    /**
     * @brief Get data reader qos.
     * @param[out] qos
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetQos(DDS_DataReaderQos &qos)
    {
        return impl_->GetQos(qos);
    }

    /**
     * @brief Set data reader listener.
     * @details Init notification, set listener to reader and set mask to entity.
     * @param[in] listener data reader listener
     * @param[in] mask     status mask in entity
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetListener(DDS_DataReaderListener *listener, DDS_StatusMask mask)
    {
        return impl_->SetListener(listener, mask);
    }

    /**
     * @brief Get data reader listener.
     * @return pointer to DDS_DataReaderListener
     */
    virtual DDS_DataReaderListener *GetListener()
    {
        return impl_->GetListener();
    }

    /**
     * @brief Get topic description
     * @return pointer to DDS_TopicDescription
     */
    virtual DDS_TopicDescription *GetTopicDescription()
    {
        return impl_->GetTopicDescription();
    }

    /**
     * @brief Get subscriber to this reader
     * @return pointer to DDS_Subscriber
     */
    virtual DDS_Subscriber *GetSubscriber()
    {
        return impl_->GetSubscriber();
    }

    /**
     * @brief Get sample rejected status.
     * @param[out] status status of sample rejected
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetSampleRejectedStatus(DDS_SampleRejectedStatus &status)
    {
        return impl_->GetSampleRejectedStatus(status);
    }

    /**
     * @brief Get liveliness changed status
     * @param[out] status status of liveliness changed
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetLivelinessChangedStatus(DDS_LivelinessChangedStatus &status)
    {
        return impl_->GetLivelinessChangedStatus(status);
    }

    /**
     * @brief Get requested deadline missed status.
     * @param[out] status status of requested deadline missed
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetRequestedDeadlineMissedStatus(DDS_RequestedDeadlineMissedStatus &status)
    {
        return impl_->GetRequestedDeadlineMissedStatus(status);
    }

    /**
     * @brief Get requested imcompatible qos status.
     * @param[out] status status of requested imcompatible qos
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetRequestedIncompatibleQosStatus(DDS_RequestedIncompatibleQosStatus &status)
    {
        return impl_->GetRequestedIncompatibleQosStatus(status);
    }

    /**
     * @brief Get subscription matched status.
     * @param[out] status status of subscription matched
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetSubscriptionMatchedStatus(DDS_SubscriptionMatchedStatus &status)
    {
        return impl_->GetSubscriptionMatchedStatus(status);
    }

    /**
     * @brief Get sample lost status.
     * @param[out] status status of sample lost
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetSampleLostStatus(DDS_SampleLostStatus &status)
    {
        return impl_->GetSampleLostStatus(status);
    }

    /**
     * @brief Get status condition from entity.
     * @return pointer to DDS_StatusCondition
     */
    DDS_StatusCondition *GetStatusCondition() override
    {
        return impl_->GetStatusCondition();
    }

    /**
     * @brief Get status changed from entity.
     * @return DDS_StatusMask which is changed
     */
    DDS_StatusMask GetStatusChanges() override
    {
        return impl_->GetStatusChanges();
    }

    /**
     * @brief Get RTPS reader instance handle
     * @return DDS_InstanceHandle(uint64_t)
     */
    DDS_InstanceHandle GetInstanceHandle() override
    {
        return impl_->GetInstanceHandle();
    }

    /**
     * @brief Enable the DDS_DataReader.
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_ERROR
     */
    virtual DDS_ReturnCode Enable()
    {
        return impl_->Enable();
    }

    /**
     * @brief Get guid of data reader.
     * @param[in,out] guid output of reader guid.
     */
    virtual void GetGuid(DDS_Guid &guid)
    {
        return impl_->GetGuid(guid);
    }

    /**
     * @brief Set the related entity of the datawriter
     * @param[in,out] guid reference of DDS_Guid
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetRelatedEntity(const DDS_Guid &guid)
    {
        return impl_->SetRelatedEntity(guid);
    }

protected:
    /**
     * @brief Read or take a collection of data samples.
     * @param[in,out] dataValues  data sequence
     * @param[in,out] sampleInfos sample info of data sequence
     * @param[in] maxSamples      length of data sequence (0,1000]
     * @param[in] states          status of reader(unused now)
     * @param[in] take            if true, take value. if false, read value.
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_BAD_PARAMETER maxSamples is not equal to dataSeq size or nullptr in sequence
     * @retval DDS_RETCODE_ERROR         when loan is used, allocate data fail
     * @retval DDS_RETCODE_NO_DATA       no data found
     * @retval DDS_RETCODE_OK
     * @note if the dataValues.Maximum() == 0, API will treat the data as zero Cpy.
     * Then user needs to returnLoaned after the Take, user only need to return
     * the data whose sampleInfo.validData == true
     */
    virtual DDS_ReturnCode ReadOrTakeUntypedI(
        DDS_DataSeq &dataValues, DDS_SampleInfoSeq &sampleInfos, DDS_Long maxSamples, DDS_UnsignedLong states,
        bool take)
    {
        return impl_->ReadOrTakeUntypedI(dataValues, sampleInfos, maxSamples, states, take);
    }

    /* raw data */
    virtual DDS_ReturnCode TakeRawDataImpl(
        DDS_DataSeq &dataValues, DDS_SampleInfoSeq &sampleInfos, DDS_Long maxSamples)
    {
        return impl_->TakeRawDataImpl(dataValues, sampleInfos, maxSamples);
    }

    /**
     * @brief Read or take samples that match the specified condition
     * @param[in,out] dataValues
     * @param[in,out] sampleInfos
     * @param[in] maxSamples
     * @param[in] condition
     * @param[in] take
     * @return DDS_ReturnCode
     */
    virtual DDS_ReturnCode ReadOrTakeWConditionUntypedI(
        DDS_DataSeq &dataValues, DDS_SampleInfoSeq &sampleInfos, DDS_Long maxSamples, DDS_ReadCondition *condition,
        bool take)
    {
        return impl_->ReadOrTakeWConditionUntypedI(dataValues, sampleInfos, maxSamples, condition, take);
    }

    /**
     * @brief Read or take the next not_proviously_accessed data value
     * @param[in,out] receivedData
     * @param[in,out] sampleInfo
     * @param[in] take
     * @return DDS_ReturnCode
     */
    virtual DDS_ReturnCode ReadOrTakeNextSampleUntypedI(
        void *receivedData, DDS_SampleInfo &sampleInfo, bool take)
    {
        return impl_->ReadOrTakeNextSampleUntypedI(receivedData, sampleInfo, take);
    }

    /**
     * @brief Accessing the collection of data values and info sequence obtained by some earlier
     * invocation of read or take.
     * @param[in] dataValues
     * @param[in] infoSeq
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode ReturnLoanUntypedI(
        DDS_DataSeq &dataValues, DDS_SampleInfoSeq &infoSeq)
    {
        return impl_->ReturnLoanUntypedI(dataValues, infoSeq);
    }

    virtual DDS_ReturnCode ReturnLoanUntypedI(const DDS_InstanceHandle& handleOfData2Return)
    {
        return impl_->ReturnLoanUntypedI(handleOfData2Return);
    }

    /**
     * @brief Constructor of DDS_DataReader, set implement.
     * @param[in] impl pointer of DDS_DataReaderImpl
     */
    explicit DDS_DataReader(DDS_DataReader *impl) : impl_(impl)
    {}

    /**
     * @brief Destructor of DDS_DataReader, this will set implement to nullptr.
     */
    ~DDS_DataReader() override
    {
        impl_ = nullptr;
    }

private:
    DDS_DataReader *impl_;
};

class DDS_SubscriberListener : public virtual DDS_DataReaderListener {
public:
    /**
     * @brief Function to be called when new data is available in reader.
     * @param[in] subscriber
     */
    virtual void OnDataOnReaders(DDS_Subscriber *subscriber)
    {
        static_cast<void>(subscriber);
    }

    /**
     * @brief Destructor of DDS_SubscriberListener
     */
    ~DDS_SubscriberListener() override = default;
};

class DDS_DomainParticipant;

class DDS_Subscriber : public DDS_Entity {
public:
    /**
     * @brief Set subscriber qos.
     * @param[in] qos subscriber qos
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetQos(DDS_SubscriberQos &qos) = 0;

    /**
     * @brief Get subscriber qos.
     * @param[in,out] qos subscriber qos
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetQos(DDS_SubscriberQos &qos) = 0;

    /**
     * @brief Set the subscriber listener.
     * @param[in] listener
     * @param[in] mask associated with subscriber listener
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetListener(DDS_SubscriberListener *listener, DDS_StatusMask mask) = 0;

    /**
     * @brief Get subscriber listener.
     * @return pointer to subscriber listener
     */
    virtual DDS_SubscriberListener *GetListener() = 0;

    /**
     * @brief Create a data reader and attach to this subscriber.
     * @param[in] topic    DDS_TopicDescription
     * @param[in] qos      DDS_DataReaderQos
     * @param[in] listener DDS_DataReaderListener
     * @param[in] mask     DDS_StatusMask
     * @return pointer to a new DDS_DataReader
     */
    virtual DDS_DataReader *CreateDataReader(
        DDS_TopicDescription *topic,
        const DDS_DataReaderQos &qos = DDS_DATAREADER_QOS_DEFAULT,
        DDS_DataReaderListener *listener = nullptr,
        DDS_StatusMask mask = DDS_STATUS_MASK_NONE) = 0;

    /**
     * @brief Delete a reader belongs to the subscriber.
     * @param[in] reader data reader to be deleted
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_BAD_PARAMETER        invalid pointer
     * @retval DDS_RETCODE_PRECONDITION_NOT_MET the reader not belongs to the subscriber or has been deleted
     */
    virtual DDS_ReturnCode DeleteDataReader(DDS_DataReader *reader) = 0;

    /**
     * @brief return which the subscriber belongs.
     * @return pointer to DDS_DomainParticipant
     */
    virtual DDS_DomainParticipant *GetParticipant() = 0;

    /**
     * @brief Set the default qos for this subscriber
     * @param[in] qos the default qos to be set the default qos
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetDefaultDataReaderQos(const DDS_DataReaderQos &qos) = 0;

    /**
     * @brief Copy the default qos.
     * @param[in,out] qos to be filled up
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetDefaultDataReaderQos(DDS_DataReaderQos &qos) = 0;

    /**
     * @brief Retrieves reader contians within the subscriber by name.
     * @param[in] topicName entity name of the reader
     * @return pointer to the found reader
     */
    virtual DDS_DataReader *LookupDataReader(const char *topicName) = 0;

protected:
    /**
     * @brief Destructor of DDS_Subscriber
     */
    ~DDS_Subscriber() override = default;
};

#endif /* RT_DDS_DDS_CPP_SUBSCRIPTION_H */

