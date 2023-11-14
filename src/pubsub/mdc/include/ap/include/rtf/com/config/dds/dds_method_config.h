/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-20
 */

#ifndef RTF_COM_CONFIG_DDS_METHOD_CONFIG_H_
#define RTF_COM_CONFIG_DDS_METHOD_CONFIG_H_

#include <set>
#include <string>

#include "rtf/com/config/dds/dds_entity_config.h"

namespace rtf {
namespace com {
namespace config {
class DDSMethodConfig : public DDSEntityConfig {
public:
    /**
     * @brief DDSMethodConfig constructor
     *
     * @param[in] methodName      Name of the method
     * @param[in] requestFragSize Request fragment size of the method
     * @param[in] requestListSize Request list size of the method
     * @param[in] replyFragSize   Reply fragment size of the method
     * @param[in] replyListSize   Reply list size of the method
     */
    DDSMethodConfig(const std::string&   methodName,
                    const dds::FragSize& requestFragSize,
                    const dds::ListSize& requestListSize,
                    const dds::FragSize& replyFragSize,
                    const dds::FragSize& replyListSize);

    /**
     * @brief DDSMethodConfig constructor
     *
     * @param[in] methodName     Name of the method
     * @param[in] domainId       Domain id of the method
     * @param[in] transportModes Transport modes that the method uses
     */
    DDSMethodConfig(const std::string&             methodName,
                    const dds::DomainId&           domainId,
                    const std::set<TransportMode>& transportModes = { TransportMode::SHM,
                                                                      TransportMode::UDP
                                                                    });

    /**
     * @brief DDSMethodConfig constructor
     *
     * @param[in] methodName Name of the method
     * @param[in] transports Transport modes that the method uses
     */
    DDSMethodConfig(const std::string& methodName, const std::set<TransportMode>& transportModes);

    /**
     * @brief DDSMethodConfig constructor
     *
     * @param methodName[in] Name of the method
     */
    explicit DDSMethodConfig(const std::string& methodName);

    /**
     * @brief DDSMethodConfig destructor
     */
    virtual ~DDSMethodConfig(void) = default;

    // ConfigInfoInterface
    /**
     * @brief Return the name of the config
     * @return Name of the config
     */
    virtual std::string GetEntityName(void) const noexcept override;

    /**
     * @brief Return the type of the config
     * @return Type of the config
     */
    virtual ConfigType GetConfigType(void) const noexcept override;

    // DDSMethodConfig
    /**
     * @brief Set the method name
     * @param[in] methodName Name of the method
     */
    void SetMethodName(const std::string& methodName) noexcept;

    /**
     * @brief Return the name of the method
     * @return Name of the method
     */
    std::string GetMethodName(void) const noexcept;

    /**
     * @brief Set the method request topic name
     * @param[in] topicName Request topic name of the method
     */
    void SetRequestTopicName(const std::string& requestTopicName) noexcept;

    /**
     * @brief Return the request topic name of the method
     * @return Request topic name of the method
     */
    std::string GetRequestTopicName(void) const noexcept;

    /**
     * @brief Set the request fragment size
     * @param[in] fragSize Request fragment size of the method
     */
    void SetRequestFragSize(const dds::FragSize& requestFragSize) noexcept;

    /**
     * @brief Return the request fragment size of the method
     * @return Request fragment size of the method
     */
    dds::FragSize GetRequestFragSize(void) const noexcept;

    /**
     * @brief Set the request list size
     * @param[in] listSize Request list size of the method
     */
    void SetRequestListSize(const dds::ListSize& requestListSize) noexcept;

    /**
     * @brief Return the request list size of the method
     * @return Request list size of the method
     */
    dds::ListSize GetRequestListSize(void) const noexcept;

    /**
     * @brief Set the method reply topic name
     * @param[in] topicName Reply topic name of the method
     */
    void SetReplyTopicName(const std::string& requestTopicName) noexcept;

    /**
     * @brief Return the reply topic name of the method
     * @return Reply topic name of the method
     */
    std::string GetReplyTopicName(void) const noexcept;

    /**
     * @brief Set the reply fragment size
     * @param[in] fragSize Reply fragment size of the method
     */
    void SetReplyFragSize(const dds::FragSize& replyFragSize) noexcept;

    /**
     * @brief Return the reply fragment size of the method
     * @return Reply fragment size of the method
     */
    dds::FragSize GetReplyFragSize(void) const noexcept;

    /**
     * @brief Set the reply list size
     * @param[in] listSize Replyt list size of the method
     */
    void SetReplyListSize(const dds::ListSize& replyListSize) noexcept;

    /**
     * @brief Return the reply list size of the method
     * @return Reply list size of the method
     */
    dds::ListSize GetReplyListSize(void) const noexcept;

    /**
     * @brief Set the request WriterQos
     * @param[in] writerQos request WriterQos of the method
     */
    void SetRequestWriterQos(const dds::WriterQos& writerQos) noexcept;

    /**
     * @brief Return the request WriterQos
     * @return Request WriterQos of the method
     */
    dds::WriterQos GetRequestWriterQos() const noexcept;

    /**
     * @brief Set the request ReaderQos
     * @param[in] readerQos request ReaderQos of the method
     */
    void SetRequestReaderQos(const dds::ReaderQos& readerQos) noexcept;

    /**
     * @brief Return the request ReaderQos
     * @return Request ReaderQos of the method
     */
    dds::ReaderQos GetRequestReaderQos() const noexcept;

    /**
     * @brief Set the reply WriterQos
     * @param[in] writerQos reply WriterQos of the method
     */
    void SetReplyWriterQos(const dds::WriterQos& writerQos) noexcept;

    /**
     * @brief Return the reply WriterQos
     * @return Reply WriterQos of the method
     */
    dds::WriterQos GetReplyWriterQos() const noexcept;

    /**
     * @brief Set the reply ReaderQos
     * @param[in] ReaderQos reply ReaderQos of the method
     */
    void SetReplyReaderQos(const dds::ReaderQos& readerQos) noexcept;

    /**
     * @brief Return the reply ReaderQos
     * @return Reply ReaderQos of the method
     */
    dds::ReaderQos GetReplyReaderQos() const noexcept;

private:
    std::string   methodName_;
    std::string   requestTopicName_;
    dds::FragSize requestFragSize_;
    dds::ListSize requestListSize_;
    std::string   replyTopicName_;
    dds::FragSize replyFragSize_;
    dds::ListSize replyListSize_;
    dds::WriterQos requestWriterQos_;
    dds::ReaderQos requestReaderQos_;
    dds::WriterQos replyWriterQos_;
    dds::ReaderQos replyReaderQos_;

    static dds::FragSize const DEFAULT_REQUEST_FRAG_SIZE;
    static dds::FragSize const DEFAULT_REPLY_FRAG_SIZE;
    static dds::ListSize const DEFAULT_REQUEST_LIST_SIZE;
    static dds::ListSize const DEFAULT_REPLY_LIST_SIZE;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_DDS_METHOD_CONFIG_H_
