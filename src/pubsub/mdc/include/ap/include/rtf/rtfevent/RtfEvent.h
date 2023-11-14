/*

 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.

 * Description: rtf event API

 * Author: 沈祖定 s00396010

 * Create: 2019-11-25

 * Notes: 无

 * History: 2020-01-13 刘伟

 */

#ifndef RTFTOOLS_RTFEVENT_H
#define RTFTOOLS_RTFEVENT_H

#include "ara/core/vector.h"
#include "ara/core/string.h"
#include "ara/core/map.h"
#include <deque>
#include <chrono>
#include <numeric>
#include <mutex>
#include <set>
#include <functional>
#include <cmath>
#include "rtf/internal/tools_common_client_manager.h"
#include "json_parser/document.h"
#include "json_parser/global.h"
#include "rtf/com/rtf_com.h"
#include "rtf/internal/rtf_tools_type.h"

using Document = ara::godel::common::jsonParser::Document;
using JsonParseValue = ara::godel::common::jsonParser::JsonParseValue;
using rtf::maintaind::proxy::methods::QueryEventInfo;
using rtf::maintaind::proxy::methods::QueryEventShow;
using rtf::maintaind::proxy::methods::QueryDataType;
using rtf::maintaind::EventInfoWithPubSub;

namespace rtf {
namespace rtfevent {
const ara::core::String DATA_TYPE_VALUE_UINT8 = "uint8_t";
const ara::core::String DATA_TYPE_VALUE_UINT16 = "uint16_t";
const ara::core::String DATA_TYPE_VALUE_UINT32 = "uint32_t";
const ara::core::String DATA_TYPE_VALUE_UINT64 = "uint64_t";
const ara::core::String DATA_TYPE_VALUE_INT8 = "int8_t";
const ara::core::String DATA_TYPE_VALUE_INT16 = "int16_t";
const ara::core::String DATA_TYPE_VALUE_INT32 = "int32_t";
const ara::core::String DATA_TYPE_VALUE_INT64 = "int64_t";
const ara::core::String DATA_TYPE_VALUE_STRING = "string";
const ara::core::String DATA_TYPE_VALUE_FLOAT = "float";
const ara::core::String DATA_TYPE_VALUE_DOUBLE = "double";
const ara::core::String DATA_TYPE_STRUCT = "structure";
const ara::core::String DATA_TYPE_VECTOR = "vector";
const ara::core::String DATA_TYPE_ARRAY = "array";
const ara::core::String DATA_TYPE_MAP = "associative_map";
const ara::core::String DATA_TYPE_ENUMERATION = "enumeration";

struct DataTypeReturnValue {
    ara::core::String name;
    ara::core::String type;
    ara::core::String dataType;
    ara::core::String value;
    ara::core::String isConstant;
};

struct EventShowReturnValue {
    ara::core::String dataTypeName;
    ara::core::String dataType;
};

class RtfEvent {
public:
    RtfEvent();
    ~RtfEvent() = default;
    int Query(const ara::core::String eventName, RtfEventInfo &rtfEventInfo);

    /*******************************************************************************
      函 数 名		:  QueryAll
      功能描述		:  查询环境上存在的event信息，支持参数过滤功能
      输入参数		:  filter，过滤参数
      输出参数		:  rtfEventInfoList，环境上存在的event信息
      返 回 值		:  int, 0表示查询成功，-1表示查询出错
    *******************************************************************************/
    int QueryAll(ara::core::Vector<RtfEventInfo> &rtfEventInfoList,
                 const std::shared_ptr<EventFilter> &filter = nullptr);

    /*******************************************************************************
      函 数 名		:  QueryAllWithNamespace
      功能描述		:  查询环境上指定nameSpace下存在的event信息，支持参数过滤功能
      输入参数		:  filter，过滤参数
                      nameSpace, 指定命名空间
      输出参数		:  rtfEventInfoList，环境上存在的event信息
      返 回 值		:  int, 0表示查询成功，-1表示查询出错
    *******************************************************************************/
    int QueryAllWithNamespace(const ara::core::String nameSpace,
                              ara::core::Vector<RtfEventInfo> &rtfEventInfoList,
                              const std::shared_ptr<EventFilter> &filter = nullptr);
    int QueryEventShowInfo(const ara::core::String eventName, EventShowReturnValue &rtfEventDataType);
    int QueryDataTypeInfo(const ara::core::String dataTypeName,
                          ara::core::Vector<DataTypeReturnValue> &rtfDataTypeMsg);
    int Init();

private:
    bool GetShowResult(const QueryEventShow::Output outPut, EventShowReturnValue &rtfEventType);
    void GetInfoResult(const QueryEventInfo::Output outPut,
                       ara::core::Vector<EventInfoWithPubSub> &eventInfoWithPubSubListTmp);
    rtf::maintaind::QuerySubEventInfoIndex GetSubNodeIndex(
        const rtf::maintaind::EventInfoWithPubSub &eventInfoWithPubSub);
    int QuerySubNodeInfo(ara::core::Vector<rtf::maintaind::EventInfoWithPubSub> &eventInfoWithPubSubList);
    bool isInit_  = false;
    std::shared_ptr<rtf::rtftools::common::ToolsCommonClientManager> toolsCommonClientManager_ = nullptr;
};

// hz
struct EventHzInfo {
    double min_delta;   // 最小的时间间隔差。
    double max_delta;   // 最大的时间间隔差。
    double rate;        // 统计的平均频率，rate=num/sum。
    double std_dev;     // 所有时间标准差，标准差计算公式如下所示。
    uint32_t window;    // 记录消息窗口个数，有最大缓存上限，需要配置初始值。
    bool publish;       // 是否发布对应的消息
    bool isSubed;       // sub是否建立成功
};
struct DDSTopicConfig {
    ara::core::String topic;
};

struct HzOptions {
    ara::core::String ddsNet;
    ara::core::String someipNet;
};

enum class RtfeventHzType : uint8_t {
    ALL_RIGHT                 = 0,    // 订阅消息
    WARN_NO_SOMEIP_NET        = 1,    // 没有输入SOME/IP网卡network名称
    EROR_CREATE_SOMEIP        = 2,    // SOMEIP sub创建失败
    EROR_CREATE_DDS           = 3,    // DDS sub创建失败
    EROR_CREATE_ALL           = 4,    // 所有event sub创建失败
};

class RtfEventHz {
public:
    typedef std::function<void(ara::core::Vector<std::pair<ara::core::String, EventHzInfo>>, RtfeventHzType)>
        HzInfoCallback;

    /** @brief the callback will be called with the event's name as arg input */
    using ReceiveMsgCallback = std::function<void(const ara::core::String&)>;

    RtfEventHz();
    ~RtfEventHz() = default;
    bool Init(const ara::core::Vector<ara::core::String>& eventList, const uint32_t window,
              HzOptions hzOptions, HzInfoCallback callback);
    /**
     * @brief register a callback, which will be called everytime when we
     * receive msg from any interested events
     * @param eventList the interested event names
     * @param callback the callback will be called when receive msg
     * @note You shall not do any time consuming operations in the callback
     */
    bool Init(const ara::core::Vector<ara::core::String>& eventList,
              const HzOptions& hzOptions, ReceiveMsgCallback callback);
    void RtfEventHzStop();
    void RtfEventHzStart();
private:
    enum class InitState {
        NOT_INITED,
        RAW_CALLBACK,
        ONE_SECOND_CALLBACK
    };

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
    bool IsUsingRawData(const rtf::maintaind::VectorTransportQos& transportQos);
    bool InitCommon(const ara::core::Vector<ara::core::String>& eventList, const HzOptions& hzOptions);
    void RtfEventPrintHz();
    bool RtfEventHzReady() const;
    void RtfEventGetHz();
    double GetMeanTimes(const std::deque<uint64_t>& times);
    double GetStdDevTimes(double mean, const std::deque<uint64_t>& times) const;
    void ReceivedMessageCallback(const ara::core::String& eventName);
    void SubscribeEvent(const ara::core::String& event);
    void DDSConfig(const ara::core::String& event, rtf::maintaind::DDSEventInfo& ddsInfo);
    void SOMEIPConfig(const ara::core::String& event, rtf::maintaind::SomeipEventInfo& someipInfo);
    void UnSubscribe(const ara::core::String& event);
    bool ShouldSubscribe(const ara::core::String& event);
    void PrintInfo(rtf::maintaind::DriverType driverType, const ara::core::String& event);
    void Subscribe();
    bool IsSubCreateFailed(const ara::core::String& event) const;
    bool IsSubscribed(const ara::core::String& event) const;
    ara::core::String GetInstanceName(ara::core::String eventName);
    void DoSubscribe(const ara::core::String& event,
        ara::core::Vector<rtf::maintaind::EventInfoWithPubSub>& eventInfoWithPubSubList);
    void RegistConfig(const ara::core::String& event,
        ara::core::Vector<rtf::maintaind::EventInfoWithPubSub>& eventInfoWithPubSubList,
        bool& ddsExist,
        bool& someipExist);
    void GetPubSub(const QueryEventInfo::Output outPut,
        ara::core::Vector<rtf::maintaind::EventInfoWithPubSub>& eventInfoWithPubSubList);
    void CheckSub(const ara::core::String& event, bool ddsExist, bool someipExist);
    void HandleSub(const ara::core::String& event, rtf::com::Subscriber& sub);

    HzOptions hzOption_;
    std::set<ara::core::String> eventInfo_;     // 用来保存event的信息，可能存在同时查找多个event的场景
    ara::core::Vector<ara::core::String> foundEvent_;
    ara::core::Map<ara::core::String, DDSTopicConfig> subEventInfo_;
    std::set<ara::core::String> currentlyRecording_;
    std::set<ara::core::String> subCreateFailed_;
    ara::core::Vector<rtf::rtfevent::RtfEventInfo> eventInfoList_;
    ara::core::Map<ara::core::String, ara::core::String> types_;
    ara::core::Map<ara::core::String, rtf::com::Subscriber> subs_;
    ara::core::Map<ara::core::String, DDSEventIndexInfo> ddsSubs_;
    ara::core::Map<ara::core::String, SOMEIPEventIndexInfo> someipSubs_;
    ara::core::String fieldName_;
    uint32_t window_;
    ara::core::Map<ara::core::String, EventHzInfo> eventHz_;  // hz信息保存表格，用来输出当前的信息
    bool eventHzEnable_;         // rtf event hz start stop控制
    bool ddsCreateError_;
    bool someipCreateError_;
    InitState initState_;
    std::mutex mutex_;
    ara::core::Map<ara::core::String, std::chrono::steady_clock::time_point> lastPrintTn_;
    ara::core::Map<ara::core::String, std::chrono::steady_clock::time_point> lastTimes_;
    ara::core::Map<ara::core::String, std::deque<uint64_t>> times_;   // event 消息间隔
    std::shared_ptr<rtf::com::NodeHandle> nh_;
    HzInfoCallback eventHzCallback_;     // 回调函数保存
    ReceiveMsgCallback receiveMsgCallback_;
    ara::core::Map<ara::core::String, uint32_t> filterEventMsg_;
    std::shared_ptr<rtf::rtftools::common::ToolsCommonClientManager> toolsCommonClientManager_ = nullptr;
};
}
}
#endif // RTFTOOLS_RTFEVENT_H
