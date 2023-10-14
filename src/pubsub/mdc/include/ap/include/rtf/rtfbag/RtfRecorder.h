/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 *      This file is the implement of class RtfRecorder.
 *      rtf recorder will record event info.
 * Author: danjinguo 00299008
 * Create: 2019-12-03
 * Notes: NA
 * History: 2019-12-03 danjinguo 00299008 revised this file.
 */
#ifndef RTF_RECORDER_H
#define RTF_RECORDER_H

#include "ara/core/string.h"
#include "ara/core/vector.h"
#include "ara/core/map.h"
#include <chrono>
#include <queue>
#include <memory>
#include <mutex>
#include <thread>
#include <functional>
#include <unistd.h>
#include <fstream>
#include <list>
#include <sstream>
#include <iomanip>
#include <sys/vfs.h>
#include <sys/statvfs.h>
#include <condition_variable>
#include <set>
#include "rtf/internal/RtfBagFile.h"
#include "rtf/internal/tools_common_client_manager.h"
#include "rtf/rtfevent/RtfEvent.h"
#include "rtf/internal/RtfEventCommand.h"
#include "rtf/com/rtf_com.h"
#include "vrtf/vcc/api/raw_buffer_for_raw_data.h"

namespace rtf {
namespace rtfbag {
struct RecorderOptions {
    RecorderOptions();

    bool                recordAll;
    bool                appendDate;
    ara::core::String   prefix;
    ara::core::String   name;
    uint32_t            bufferSize;             // the unit of bufferSize is Byte
    uint32_t            chunkSize;
    uint32_t            limit;
    uint32_t            skipFrame;              // 跳帧个数
    bool                split;                  // 分隔的个数
    uint64_t            maxSize;                // 最大文件大小
    uint32_t            maxSplits;              // 最大分割个数
    uint32_t            maxDuration;
    ara::core::Vector<ara::core::String> events;
    uint64_t            minSpace;
    ara::core::String   path;
    ara::core::String   someipNet;
    ara::core::String   ddsNet;
};

enum class RecorderType : uint8_t {
    INFO_SUBSCRIBE_TO_EVENT   = 0,    // 订阅消息
    INFO_RECORDER_TO_FILE     = 1,    // 记录消息到bag file
    INFO_CLOSE_TO_FILE        = 2,    // 关闭当前bag file
    WARN_LESS_SPACE           = 3,    // 磁盘空间告警提示
    WARN_RECORD_SAME_FILE     = 4,    // 录制相同的文件名称
    WARN_UNSUBSCRIBE_TO_EVENT = 5,    // 没有订阅消息
    WARN_OPEN_INVALID_PATH    = 6,    // 打开非法的文件路径
    WARN_NO_SOMEIP_NET        = 7,    // 没有输入SOME/IP网卡network名称
    EROR_CREATE_SOMEIP        = 8,    // SOMEIP sub创建失败
    EROR_CREATE_DDS           = 9,    // DDS sub创建失败
    EROR_PARAMETER            = 10,   // 输入参数存在错误
};
enum class SubscribeType : uint8_t {
    SUBSCRIBE_SUCCESS       = 0,    // 订阅成功
    WAITFOR_UNSUBSCRIBE     = 1,    // 等待订阅取消
    UNSUBSCRIBE_SUCCESS     = 2,    // 取消订阅成功
};
struct OutgoingMessage {
    uint64_t                            time;
    ara::core::String                   eventName;
    ara::core::String                   type;
    rtf::maintaind::DriverType          driverType;
    // buff for nomal msg or the msg in format[PrivateLength][PrivateValue][MbufData]
    uint8_t*                            msgBuff;
    uint32_t                            buffLen;
};

struct OutgoingQueue {
    OutgoingQueue(const ara::core::String &fileName, const std::queue<OutgoingMessage> &que, const uint64_t &tm);

    ara::core::String               fileName;
    std::queue<OutgoingMessage>     queue;
    uint64_t                        time;
};

class RtfRecorder {
public:
    using RecorderCallback = std::function<void(RecorderType, ara::core::String)>;
    RtfRecorder(const RecorderOptions& options);
    ~RtfRecorder() = default;

    int32_t Run();
    void Stop();
    void RecorderRegisterCallback(const RecorderCallback &callback);
private:
    struct DDSEventIndexInfo {
        rtf::stdtype::uint16_t serviceId;
        rtf::stdtype::uint16_t instanceId;
        ara::core::String topic;
    };

    struct SOMEIPEventIndexInfo {
        rtf::stdtype::uint16_t serviceId;
        rtf::stdtype::uint16_t instanceId;
        rtf::stdtype::uint16_t eventId;
    };

    bool IsRepeatedEvent(rtf::maintaind::EventInfoWithPubSub& eventInfoWithPubSub,
                         const ara::core::String& event) const;
    void DealWithSubscribedEvent(const ara::core::String& event,
        rtf::maintaind::EventInfoWithPubSub& eventInfoWithPubSub,
        const bool& ddsExist,
        const bool& someipExist);
    bool CheckTheRecorderOptions();
    bool CheckStop();
    void DoRecord();
    void DoRecordHandle();
    void StartWriting();
    void StopWriting();
    void CheckNumSplits();
    void UpdateFileNames();
    bool CheckSize();
    bool CheckDisk();
    void DoSplit(uint64_t time, uint64_t maxDuration);
    bool CheckDuration(uint64_t time);
    void CheckLimit(const ara::core::String& event);
    bool CheckSkipFrame(const ara::core::String& event);
    OutgoingMessage PrepareOutOutgoingMessage(const ara::core::String& event,
                                              const vrtf::core::RawBufferForRawData& msg);
    OutgoingMessage PrepareOutOutgoingMessage(const ara::core::String& event, const vrtf::core::RawBuffer& msg);
    void DoQueue(const ara::core::String& event, OutgoingMessage& outMsg);
    void SubscribeEvent(const ara::core::String& event);
    bool ShouldSubscribe(const ara::core::String& event);
    void Subscribe();
    void DoSubscribe(const ara::core::String& event,
        ara::core::Vector<rtf::maintaind::EventInfoWithPubSub>& eventInfoWithPubSubList);
    bool RegistConfig(const ara::core::String& event,
        ara::core::Vector<rtf::maintaind::EventInfoWithPubSub>& eventInfoWithPubSubList,
        bool& ddsExist,
        bool& someipExist);
    void GetPubSub(const QueryEventInfo::Output,
        ara::core::Vector<rtf::maintaind::EventInfoWithPubSub>& eventInfoWithPubSubList);
    void HandleSub(const ara::core::String& event, rtf::com::Subscriber& sub);
    bool IsSubscribed(const ara::core::String& event) const;
    void UnSubscribeEvent(const ara::core::String& event);
    void UnSubscribeCheck();
    ara::core::String TimeToStr();
    bool ScheduledCheckDisk();
    void SubscribeCallback(const ara::core::String& event, const vrtf::core::RawBuffer& msg);
    /**
     * @brief The callback of subscriber for recording the msg with mbuf
     *
     * @param[in] event    The event name using mbuf
     * @param[in] msg      The msg with mbuf will be recorded
     */
    void SubscribeCallback(const ara::core::String& event, const vrtf::core::RawBufferForRawData& msg);
    void GetEventList();
    uint64_t GetCurrentRealTime() const;
    uint64_t GetCurrentMonotonicTime() const;
    uint64_t ChangeMonotonicToRealTime(const uint64_t& monotonicTime) const;
    bool RecordReady() const;
    void Start();
    void PrintInfo(rtf::maintaind::DriverType driverType, const ara::core::String& event);
    void PrintSomeipInfo(const ara::core::String& event, const rtf::maintaind::SomeipEventInfo& someipInfo);
    void DDSConfig(const ara::core::String& event, rtf::maintaind::DDSEventInfo& ddsInfo);
    void SOMEIPConfig(const ara::core::String& event, rtf::maintaind::SomeipEventInfo& someipInfo);
    ara::core::String GetInstanceName(ara::core::String eventName);
    void UnSubscribeAll();
    void GetAllDataType();
    void GetDataTypeRefList();
    bool WriteEventMsg(const OutgoingMessage& out);
    bool InsertMsgDef(const ara::core::String& eventName, const ara::core::String& dataTypeName);
    void SetStartTime();
    void SetStopTime();
    void InitStartTime();
    void PrintE2ECheckResult(const ara::core::String& event, const rtf::com::SampleInfo& sampleInfo);
    rtf::com::Subscriber SubscribeDdsRawDataEvent(const ara::core::String& event, const bool isEnableE2E);
    rtf::com::Subscriber SubscribeCommonEvent(const ara::core::String& event, const bool isEnableE2E);
    rtf::maintaind::EventInfo tempEventInfo_;
    ara::core::String event_;
    RecorderOptions options_;
    ara::core::Vector<ara::core::String> events_;
    std::set<ara::core::String> subCreateFailed_;
    std::set<ara::core::String> allEvents_;
    bool ddsCreateError_;
    bool someipCreateError_;
    ara::core::String pureFileName_;
    ara::core::String targetFileName_;
    ara::core::String writeFileName_;
    ara::core::String fieldName_;
    uint64_t splitCount_;
    uint64_t queueSize_;
    std::queue<OutgoingMessage>* queue_;
    uint64_t startTime_;
    uint64_t startRecordRealTime_;
    uint64_t startRecordMonotonicTime_;
    uint64_t checkDiskNext_;
    bool writeEnabled_;
    std::mutex mutex_;
    std::mutex dataTypeMutex_;
    std::mutex stopMutex_;
    std::mutex eventsMutex_;
    std::thread thread_;
    std::condition_variable_any queueCondition_;
    RecorderCallback recorderCallback_;     // 回调函数保存
    std::list<ara::core::String> currentFiles_;         // 当前文件保存
    std::shared_ptr<rtf::rtfbag::RtfBagFile> bag_;
    uint64_t lastBufferWarnTime_;
    std::set<ara::core::String> currentlyRecording_;
    std::shared_ptr<rtf::com::NodeHandle> nh_;
    bool doRecordEnabled;
    bool isBagCreated_;
    ara::core::Map<ara::core::String, uint32_t> limit_;
    ara::core::Map<ara::core::String, uint32_t> skipFrame_;
    ara::core::Map<ara::core::String, bool> recordErrorEvent_;
    ara::core::Map<ara::core::String, DDSEventIndexInfo> ddsSubs_;
    ara::core::Map<ara::core::String, SOMEIPEventIndexInfo> someipSubs_;
    ara::core::Vector<rtf::rtfevent::RtfEventInfo> eventInfoList_;
    ara::core::Map<ara::core::String, rtf::maintaind::EventInfo> eventInfos_;
    ara::core::Map<ara::core::String, ara::core::String> types_;
    ara::core::Map<ara::core::String, rtf::maintaind::DriverType> driverTypes_;
    ara::core::Map<ara::core::String, rtf::com::Subscriber> subs_;
    ara::core::Map<ara::core::String, SubscribeType> subscribeStatus_;
    ara::core::Map<ara::core::String, bool> unsubscribeStatus_;
    // key: eventName对应的DataTypeName, Value: 对应msg文件的全量信息
    ara::core::Map<ara::core::String, ara::core::String> eventToMsgDef_;
    ara::core::Map<ara::core::String, ara::core::String> dataTypeToDef_;
    // key:eventName对应的DataTypeName, value:对应的所有数据类型描述信息
    ara::core::Map<ara::core::String, ara::core::Vector<ara::core::String>> dataTypeToRefDataType_;
    std::shared_ptr<rtf::rtftools::common::ToolsCommonClientManager> toolsCommonClientManager_ = nullptr;
    const uint32_t bufferSize_ = 20; // DDS depth DTS202008200NK171P0J00
};
}  // namespace rtfbag
}  // namespace rtf
#endif // RTF_RECORDER_H
