/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_publication.h
 */

#ifndef RT_DDS_DDS_CPP_PUBLICATION_H
#define RT_DDS_DDS_CPP_PUBLICATION_H

#include "RT-DDS/dds_cpp/dds_cpp_infrastruture.h"
#include "RT-DDS/dds_cpp/dds_cpp_qos.h"

#include "plog/PLogDefsAndLimits.hpp"

struct DDS_WriteParams {
    DDS_InstanceHandle handle;
    DDS_Guid relatedSourceGuid;
    rbs::plog::PlogUid plogUid = rbs::plog::PLOG_UID_MAX;
};

struct DDS_Octets {
    uint32_t length;
    uint8_t *value;
};

class DDS_DataWriter;

class DDS_DataWriterListener : public virtual DDS_Listener {
public:
    /**
     * @brief Function to be called when deadline was not respected for a specific instance.
     * @param[in] writer pointer to DDS_DataWriter
     * @param[in] status current deadline missed status
     */
    virtual void OnOfferedDeadlineMissed(DDS_DataWriter *writer, const DDS_OfferedDeadlineMissedStatus &status)
    {
        static_cast<void>(writer);
        static_cast<void>(status);
    }

    /**
     * @brief Function to be called when writer was incompatible with what was requested by reader.
     * @param[in] writer pointer to DDS_DataWriter
     * @param[in] status current incompatible qos status
     */
    virtual void OnOfferedIncompatibleQos(DDS_DataWriter *writer, const DDS_OfferedIncompatibleQosStatus &status)
    {
        static_cast<void>(writer);
        static_cast<void>(status);
    }

    /**
     * @brief Function to be called when writer has committed through its liveliness qos was not respected.
     * @param[in] writer pointer to DDS_DataWriter
     * @param[in] status current liveliness qos status
     */
    virtual void OnLivelinessLost(DDS_DataWriter *writer, const DDS_LivelinessLostStatus &status)
    {
        static_cast<void>(writer);
        static_cast<void>(status);
    }

    /**
     * @brief Function to be called when writer found a reader that matched the topic.
     * @param[in] writer pointer to DDS_DataWriter
     * @param[in] status current publication matched qos status
     */
    virtual void OnPublicationMatched(DDS_DataWriter *writer, const DDS_PublicationMatchedStatus &status)
    {
        static_cast<void>(writer);
        static_cast<void>(status);
    }

    virtual bool OnShmCreated(DDS_DataWriter *writer, const std::string &shmName)
    {
        static_cast<void>(writer);
        static_cast<void>(shmName);
        return true;
    }

    /**
     * @brief Destructor of DDS_DataWriterListener.
     */
    ~DDS_DataWriterListener() override = default;
};

class DDS_Topic;
class DDS_Publisher;

class DDS_DataWriter : public DDS_Entity {
public:
    /**
     * @brief Set the writer Qos.
     * @param[in] qos reference of DDS_DataWriterQos
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK    set success
     * @retval DDS_RETCODE_ERROR set RTPS Qos fail
     */
    virtual DDS_ReturnCode SetQos(const DDS_DataWriterQos &qos)
    {
        return impl_->SetQos(qos);
    }

    /**
     * @brief Get the writer qos.
     * @param[in,out] qos reference of DDS_DataWriterQos
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetQos(DDS_DataWriterQos &qos)
    {
        return impl_->GetQos(qos);
    }

    /**
     * @brief Set the writer listener.
     * @param[in,out] listener pointer to DDS_DataWriterListener
     * @param[in] mask         DDS_StatusMask
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetListener(DDS_DataWriterListener *listener, DDS_StatusMask mask)
    {
        return impl_->SetListener(listener, mask);
    }

    /**
     * @brief Get the writer listener.
     * @return pointer to DDS_DataWriterListener
     */
    virtual DDS_DataWriterListener *GetListener()
    {
        return impl_->GetListener();
    }

    /**
     * @brief Get the writer topic.
     * @return pointer to DDS_Topic
     */
    virtual DDS_Topic *GetTopic()
    {
        return impl_->GetTopic();
    }

    /**
     * @brief Get the publisher that the writer belongs to.
     * @return pointer to DDS_Publisher
     */
    virtual DDS_Publisher *GetPublisher()
    {
        return impl_->GetPublisher();
    }

    /**
     * @brief Get the DDS_PublicationMatchedStatus
     * @param[in,out] status reference of DDS_PublicationMatchedStatus
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetPublicationMatchedStatus(DDS_PublicationMatchedStatus &status)
    {
        return impl_->GetPublicationMatchedStatus(status);
    }

    /**
     * @brief Enable the DDS_DataWriter.
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_ERROR
     */
    virtual DDS_ReturnCode Enable(void)
    {
        return impl_->Enable();
    }

    /**
     * @brief Get the writer GUID.
     * @param[in,out] guid reference of DDS_Guid
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetGuid(DDS_Guid &guid)
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

    /**
     * @brief Get status condition associated with entity.
     * @return pointer to DDS_StatusCondition
     */
    DDS_StatusCondition *GetStatusCondition() override
    {
        return impl_->GetStatusCondition();
    }

    /**
     * @brief Get the list of status whose value changed since using the get_status() method.
     * @return DDS_StatusMask
     */
    DDS_StatusMask GetStatusChanges() override
    {
        return impl_->GetStatusChanges();
    }

    /**
     * @brief Get instance handle of this writer.
     * @return DDS_InstanceHandle in RTPS
     */
    DDS_InstanceHandle GetInstanceHandle() override
    {
        return impl_->GetInstanceHandle();
    }

protected:
    /**
     * @brief Constructor of DDS_DataWriter.
     * @param[in] impl implement of writer.
     */
    explicit DDS_DataWriter(DDS_DataWriter *impl) : impl_(impl)
    {}

    /**
     * @brief Constructor of DDS_DataWriter, set writer implement to nullptr.
     */
    ~DDS_DataWriter(void) override
    {
        impl_ = nullptr;
    }

    /**
     * @brief These are implemented in the "type-specific" FooDataWriter generated classes.
     * @param[in] instanceData data to be wrote
     * @param[in] handle       [unused[
     * @return DDS_ReturnCode
     * @retval RTPS_RETURN_OK               write success
     * @retval DDS_RETCODE_BAD_PARAMETER    instanceData is null
     * @retval DDS_RETCODE_OUT_OF_RESOURCES size is larger than DEFAULT_MAX_MSG_PAYLOAD_SIZE
     * @retval DDS_RETCODE_ERROR            miss deadline or write fail
     */
    virtual DDS_ReturnCode WriteUntypedI(const void *instanceData, const DDS_InstanceHandle &handle)
    {
        return impl_->WriteUntypedI(instanceData, handle);
    }

    /**
     * @brief Allocate zero-copy octets to payload.
     * @param[in]  instanceData      zero-copy data
     * @param[out] handleOfAllocated the handle of the allocated data
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK               write success
     * @retval DDS_RETCODE_BAD_PARAMETER    input is nullptr
     * @retval DDS_RETCODE_OUT_OF_RESOURCES the size is way too large (16MB) or user has allocated more than 256 changes
     * @retval DDS_RETCODE_ERROR            The allocation or pre-serialization failed
     * @note You should write through WriteUntypedI(const DDS_WriteParams &writeParams) or
     * WriteUntypedI(const DDS_InstanceHandle &handle)
     * @see DDS_ReturnCode WriteUntypedI(const DDS_InstanceHandle &handle)
     * @see DDS_ReturnCode WriteUntypedI(const void *instanceData, const DDS_WriteParams &writeParams)
     */
    virtual DDS_ReturnCode AllocateOctetsUntypedI(const void* instanceData, DDS_InstanceHandle& handleOfAllocated)
    {
        return impl_->AllocateOctetsUntypedI(instanceData, handleOfAllocated);
    }

    /**
     * @brief Deallocate the mem preallocated
     * @param[in] theHandle2Deallocate The handle given when allocateing the mem
     */
    virtual void DeallocateOctetsUntypedI(const DDS_InstanceHandle& theHandle2Deallocate)
    {
        return impl_->DeallocateOctetsUntypedI(theHandle2Deallocate);
    }

    /**
     * @brief Write data after allocate payload.
     * @param[in] handle the handle of the allocated data, which is given at AllocateOctetsUntypedI
     * @see DDS_ReturnCode AllocateOctetsUntypedI(const void* instanceData, DDS_InstanceHandle& handleOfAllocated)
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK    write success
     * @retval DDS_RETCODE_ERROR an illegal handle or it has been sent, fail to add history cache
     */
    virtual DDS_ReturnCode WriteUntypedI(const DDS_InstanceHandle &handle)
    {
        return impl_->WriteUntypedI(handle);
    }

    virtual DDS_ReturnCode WriteRawData(Mbuf *mbuf)
    {
        return impl_->WriteRawData(mbuf, {});
    }

    virtual DDS_ReturnCode WriteRawData(Mbuf *mbuf, const DDS_WriteParams &writeParams)
    {
        return impl_->WriteRawData(mbuf, writeParams);
    }

    /**
     * @brief Write data after allocate payload.
     * @param writeParams the write params, handle and the target guid
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK    write success
     * @retval DDS_RETCODE_ERROR an illegal handle or it has been sent, fail to add history cache
     * @see DDS_ReturnCode AllocateOctetsUntypedI(const void* instanceData, DDS_InstanceHandle& handleOfAllocated)
     */
    virtual DDS_ReturnCode WriteUntypedI(const DDS_WriteParams &writeParams)
    {
        return impl_->WriteUntypedI(writeParams);
    }

    /**
     * @brief OverLoad write function, write data to specific reader.
     * @param[in] instanceData data to be wrote
     * @param[in] writeParams  guid of reader
     * @return DDS_ReturnCode
     * @retval RTPS_RETURN_OK               write success
     * @retval DDS_RETCODE_BAD_PARAMETER    instanceData is null
     * @retval DDS_RETCODE_OUT_OF_RESOURCES size is larger than DEFAULT_MAX_MSG_PAYLOAD_SIZE
     * @retval DDS_RETCODE_ERROR            miss deadline or write fail
     */
    virtual DDS_ReturnCode WriteUntypedI(const void *instanceData, const DDS_WriteParams &writeParams)
    {
        return impl_->WriteUntypedI(instanceData, writeParams);
    }
private:
    DDS_DataWriter *impl_;
};

class DDS_PublisherListener : public virtual DDS_DataWriterListener {
public:
    /**
     * @brief Destructor of DDS_PublisherListener.
     */
    ~DDS_PublisherListener() override = default;
};

class DDS_DomainParticipant;

class DDS_Publisher : public DDS_Entity {
public:

    /**
     * @brief Create a writer which belongs to this publisher.
     * @param[in] topic    pointer to DDS_Topic, cannot be null
     * @param[in] qos      reference of DDS_DataWriterQos
     * @param[in] listener pointer to DDS_DataWriterListener
     * @param[in] mask     DDS_StatusMask
     * @return pointer to the new DDS_DataWriter or null if an error occurred
     */
    virtual DDS_DataWriter *CreateDataWriter(
        DDS_Topic *topic,
        const DDS_DataWriterQos &qos = DDS_DATAWRITER_QOS_DEFAULT,
        DDS_DataWriterListener *listener = nullptr,
        DDS_StatusMask mask = DDS_STATUS_MASK_NONE) = 0;

    /**
     * @brief Delete a writer that belongs to this publisher.
     * @param[in,out] writer to be deleted
     * @return DDS_ReturnCode
     * @retval DDS_RETCODE_OK                   delete success
     * @retval DDS_RETCODE_BAD_PARAMETER        invalid writer
     * @retval DDS_RETCODE_PRECONDITION_NOT_MET publisher not contain this writer or it is still using
     */
    virtual DDS_ReturnCode DeleteDataWriter(DDS_DataWriter *writer) = 0;

    /**
     * @brief Set publisher Qos.
     * @param[in] qos reference of DDS_PublisherQos
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetQos(const DDS_PublisherQos &qos) = 0;

    /**
     * @brief Get publisher Qos.
     * @param[in,out] qos reference of DDS_PublisherQos
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetQos(DDS_PublisherQos &qos) = 0;

    /**
     * @brief Set the publisher listener.
     * @param[in] listener pointer to DDS_PublisherListener
     * @param[in] mask     DDS_StatusMask
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetListener(DDS_PublisherListener *listener, DDS_StatusMask mask) = 0;

    /**
     * @brief Get the publisher listener.
     * @return pointer to DDS_PublisherListener
     */
    virtual DDS_PublisherListener *GetListener() = 0;

    /**
     * @brief Get participant that the publisher belongs to.
     * @return pointer to DDS_DomainParticipant
     */
    virtual DDS_DomainParticipant *GetParticipant() = 0;

    /**
     * @brief Set publisher default DDS_DataWriterQos.
     * @param[in] qos reference of DDS_DataWriterQos
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetDefaultDataWriterQos(const DDS_DataWriterQos &qos) = 0;

    /**
     * @brief Get default DDS_DataWriterQos.
     * @param[in,out] qos reference of DDS_DataWriterQos to be filled in
     * @return DDS_ReturnCode DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode GetDefaultDataWriterQos(DDS_DataWriterQos &qos) = 0;

protected:
    /**
     * @brief Destructor of DDS_Publisher
     */
    ~DDS_Publisher() override = default;
};

#endif /* RT_DDS_DDS_CPP_PUBLICATION_H */

