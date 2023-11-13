/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This provide the config to support that adapt to ros platform.
 * Create: 2020-04-20
 */

#ifndef RTF_COM_CONFIG_DDS_EVENT_CONFIG_H_
#define RTF_COM_CONFIG_DDS_EVENT_CONFIG_H_

#include <set>
#include <string>
#include <mutex>

#include "rtf/com/config/dds/dds_entity_config.h"
#include "rtf/com/config/interface/event_maintain_interface.h"

namespace rtf {
namespace com {
namespace config {
class DDSEventConfig : public DDSEntityConfig,
                       public EventMaintainInterface {
public:
    /**
     * @brief DDSEventConfig constructor
     *
     * @param[in] eventName Name of the event
     * @param[in] fragSize  Fragment size of the event
     * @param[in] listSize  List size of the event
     */
    DDSEventConfig(std::string const &eventName, const dds::FragSize &fragSize, const dds::ListSize &listSize);

    /**
     * @brief DDSEventConfig constructor
     *
     * @param[in] eventName      Name of the event
     * @param[in] transportModes Transport modes of the event
     * @param[in] scheduleMode   Schedule mode of dshm and defualt is determinate
     */
    DDSEventConfig(const std::string& eventName,
                   const std::set<TransportMode>& transportModes,
                   const rtf::com::ScheduleMode& scheduleMode = rtf::com::ScheduleMode::DETERMINATE);

    /**
     * @brief DDSEventConfig constructor
     *
     * @param eventName[in] Name of the event
     */
    explicit DDSEventConfig(const std::string& eventName);

    /**
     * @brief DDSEventConfig destructor
     */
    virtual ~DDSEventConfig(void) = default;

    // ConfigInfoInterface
    /**
     * @brief Return the entity name of the config
     * @note This is an interface implementation
     * @return Entity name of the config
     */
    virtual std::string GetEntityName(void) const noexcept override;

    /**
     * @brief Return the type of the config
     * @note This is an interface implementation
     * @return Type of the config
     */
    virtual ConfigType GetConfigType(void) const noexcept override;

    // DDSEventConfig
    /**
     * @brief Set the event name
     * @param[in] eventName Name of the event
     */
    void SetEventName(std::string const &eventName) noexcept;

    /**
     * @brief Return the name of the event
     * @return Name of the event
     */
    std::string GetEventName(void) const noexcept;

    /**
     * @brief Set the event topic name
     * @param[in] topicName Topic name of the event
     */
    void SetEventTopicName(std::string const &topicName) noexcept;

    /**
     * @brief Return the topic name of the event
     * @return Topic name of the event
     */
    std::string GetEventTopicName(void) const noexcept;

    /**
     * @brief Set the fragment size of the event
     * @param[in] fragSize Fragment size (transport buffer size)
     */
    void SetFragSize(const dds::FragSize& fragSize) noexcept;

    /**
     * @brief Return the fragment size of the event
     * @return Fragment size (transport buffer size)
     */
    dds::FragSize GetFragSize(void) const noexcept;

    /**
     * @brief Set the list size of the event
     * @param[in] listSize List size (SHM buffer size)
     */
    void SetListSize(const dds::ListSize& listSize) noexcept;

    /**
     * @brief Return the list size of the event
     * @return List size (SHM buffer size)
     */
    dds::ListSize GetListSize(void) const noexcept;

    /**
     * @brief Set the cache size of the event
     * @param[in] cacheSize Cache size
     */
    void SetCacheSize(const dds::CacheSize& cacheSize) noexcept;

    /**
     * @brief Return the cache size of the event
     * @return Cache size
     */
    dds::CacheSize GetCacheSize(void) const noexcept;

    // EventMaintainInterface
    /**
     * @brief Set the event data type
     * @note This is a maintain configuration
     * @param[in] eventDataType Data type string of the event
     */
    virtual void SetEventDataType(const std::string& eventDataType) noexcept override;

    /**
     * @brief Return the data type of the event
     * @note This is a maintain configuration
     * @return Data type string of the event
     */
    virtual std::string GetEventDataType(void) const noexcept override;

    /**
     * @brief Set the flag if use raw data
     *
     * @param[in] isUseRawData
     */
    void SetDPRawDataFlag (const bool& isUsedRawData) noexcept;

    /**
     * @brief Get the info that if use raw data
     *
     * @return true   use raw data
     * @return false  do not use raw data
     */
    bool GetDPRawDataFlag () const noexcept;

    /**
     * @brief Set the info of schedule mode
     *
     * @param[in] scheduleMode enum of schedule mode
     */
    void SetScheduleMode (const rtf::com::ScheduleMode& scheduleMode) noexcept;

    /**
     * @brief Get the info of schedule mode
     *
     * @return DETERMINATE    dshm transport mode is determinate
     * @return INDETERMINATE  dshm transport mode is undeterminate
     */
    rtf::com::ScheduleMode GetScheduleMode () const noexcept;

    /**
     * @brief Set the info of schedule mode
     *
     * @param[in] rawBufferHelper raw buffer helper that implement by user.
     */
    void SetRawBufferHelper(const std::shared_ptr<rtf::com::RawBufferHelper>& rawBufferHelper) noexcept;

    /**
     * @brief Get the raw buffer helper
     *
     * @return Raw buffer helper
     */
    std::shared_ptr<rtf::com::RawBufferHelper> GetRawBufferHelper() const noexcept;

    /**
     * @brief For support connect to ros1 by rtftools, to set attribute type.
     * @param[in] attributeType  DDS attribute type.
     * @param[in] attributeValue  DDS attribute value.
     */
    void SetAttribute(const std::string& attributeType, const std::string& attributeValue);
    /**
     * @brief Get the attribute list.
     * @return The attribute list.
     */
    std::map<std::string, std::string> GetAttribute() const noexcept;

    /**
     * @brief Support to ignore SHM check in writer, when writer fragsize and listsize settings are different from
     *  already existed reader, and reader's fragsize and listsize setting will be used.
     * @param[in] true, ignore SHM check; false(default), do SHM check.
     */
    void IgnoreSHMCheck(const bool& isCheck);
    /**
     * @brief Check if SHM check is ignored.
     * @return true, ignore SHM check; false(default), do SHM check
     */
    bool IsSHMCheckIgnored() const noexcept;

        /**
     * @brief Set the reply WriterQos
     * @param[in] writerQos reply WriterQos of the event
     */
    void SetWriterQos(const dds::WriterQos& writerQos) noexcept;

    /**
     * @brief Return the reply WriterQos
     * @return Reply WriterQos of the event
     */
    dds::WriterQos GetWriterQos() const noexcept;

    /**
     * @brief Set the reply ReaderQos
     * @param[in] ReaderQos reply ReaderQos of the method
     */
    void SetReaderQos(const dds::ReaderQos& readerQos) noexcept;

    /**
     * @brief Return the reply ReaderQos
     * @return Reply ReaderQos of the method
     */
    dds::ReaderQos GetReaderQos() const noexcept;

private:
    std::string    eventName_;
    std::string    topicName_;
    dds::FragSize  fragSize_;
    dds::ListSize  listSize_;
    dds::CacheSize cacheSize_;
    dds::ReaderQos readerQos_;
    dds::WriterQos writerQos_;
    // EventMaintainInterfaceInterface
    std::string    eventDataType_;
    bool           isUsedRawData_;
    rtf::com::ScheduleMode scheduleMode_;
    std::mutex mapMutex_;
    std::map<std::string, std::string> attributeList_;
    std::shared_ptr<vrtf::vcc::RawBufferHelper> rawBufferHelper_;
    bool isSHMCheckIgnored_ = false;
    std::shared_ptr<rtf::com::utils::Logger> logger_;
    static dds::FragSize const DEFAULT_FRAG_SIZE;
    static dds::ListSize const DEFAULT_LIST_SIZE;
    static dds::CacheSize const DEFAULT_CACHE_SIZE;
    static const rtf::com::ScheduleMode DEFAULT_SCHEDULE_MODE;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_DDS_EVENT_CONFIG_H_
