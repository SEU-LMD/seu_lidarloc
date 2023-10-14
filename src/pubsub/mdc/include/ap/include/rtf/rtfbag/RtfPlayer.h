/* Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 *      This file is the defination of class RtfPlayer.
 * Create: 2019-12-12
 * Notes: NA
 * History: 2019-12-12 Create.
 */
#ifndef RTF_PLAYER_H
#define RTF_PLAYER_H

#include <functional>
#ifndef RTFBAG_PLAY_CFLOAT
#define RTFBAG_PLAY_CFLOAT
#include <cfloat>
#endif
#include <memory>
#include <chrono>

#include "rtf/internal/RtfMsgEntity.h"
#include "rtf/internal/RtfBagFile.h"
#include "rtf/internal/tools_common_client_manager.h"
#include "rtf/com/rtf_com.h"
#include "rtf/internal/RtfTimeTranslator.h"
#include "rtf/internal/RtfPubClock.h"
#include "rtf/internal/RtfView.h"
#include "ara/core/string.h"
#include "ara/core/vector.h"
#include "ara/core/map.h"
#include "vrtf/vcc/api/raw_buffer_for_raw_data.h"

namespace rtf {
namespace rtfbag {
struct PlayerOptions {
    PlayerOptions();

    // 浮点数默认值设置最好是2的倍数或者1/2的倍数，否则可能会导致判断错误
    const std::uint32_t MAX_QUEUE_SIZE = 1000;
    const std::uint32_t DEFAULT_QUEUE_SIZE = 100;
    const std::uint32_t MIN_QUEUE_SIZE = 0;

    const double MAX_FREQUENCY = 1000.0;
    const double DEFAULT_FREQUENCY = 0.0;
    const double MIN_FREQUENCY = 0.0;

    const double MAX_RATE = 50.0;
    const double DEFAULT_RATE = 1.0;
    const double MIN_RATE = 0.0625;

    const double MAX_DELAYTIME = 20.0;
    const double DEFAULT_DELAYTIME = 0.2;
    const double MIN_DELAYTIME = 0.0;

    const double MAX_STARTTIME = 10000.0;
    const double DEFAULT_STARTTIME = 0.0;
    const double MIN_STARTTIME = 0.0;

    const double MAX_DURATION = 10000000.0;
    const double DEFAULT_DURATION = 0.0;
    const double MIN_DURATION = 0.0;

    const double MAX_SKIPEMPTY = DBL_MAX;
    const double DEFAULT_SKIPEMPTY = DBL_MAX;
    const double MIN_SKIPEMPTY = 0.0;

    bool           quiet;       // 默认false
    bool           immediate;   // 默认false
    bool           pause;       // 默认false
    std::uint32_t  queueSize;   // 默认100，取值范围[0,1000]
    double         frequency;   // 默认0.0，取值范围[0.0,1000.0]
    double         rate;        // 默认1.0，取值范围[0.0625,50.0]
    double         delayTime;   // 默认0.2s，取值范围[0.0,20.0]
    double         startTime;   // 默认0.0，取值范围[0.0,10000.0]
    double         duration;    // 默认0.0，取值范围[0.0,10000000.0]
    double         skipEmpty;   // 默认DBL_MAX，取值范围[0.0,DBL_MAX]
    bool           loop;        // 默认为false
    ara::core::String    port;        // 输入可选port区间，闭区间[num1,num2]
    ara::core::String    someipNet;   // 输入SOMEIP network名称
    ara::core::String    ddsNet;      // 输入DDS network ip地址

    ara::core::Vector<ara::core::String> events;
    ara::core::Vector<ara::core::String> pauseEvents;
    ara::core::Vector<ara::core::String> skipEvents;
    ara::core::Vector<ara::core::String> bagFiles;
    ara::core::Map<ara::core::String, ara::core::String> forcePlayEvents; // key: eventName, value: ip
};

class RtfPlayer {
using RtfBagLog = ara::godel::common::log::Log;

public:
    enum class PlayerPrintType : uint8_t {
        OPEN_FILE  = 0,     // 打开文件
        OPEN_FAIL,          // 文件打开失败
        NO_MSG,             // 文件中无可播放的消息
        WAITING,            // 延时等待提示
        WAIT_DONE,          // 延时等待结束
        HIT_PAUSE_STEP,     // 提示暂停或单步方法
        PLAYING,            // 播放过程提示，反复刷新同一行
        PLAY_DONE,          // 播放结束
        PORT_EROR,          // 可用端口不够或者错误
        NO_PORT,            // 没有输入端口区间
        NO_SOMEIPNET,       // 没有输入SOME/IP网卡
        SOMEIP_PUB_EROR,    // SOMEIP pub创建失败
        DDS_PUB_EROR,       // DDS pub创建失败
        E2E_INFO,           // 显示使用了E2E的events
        PARAMETER_EROR,     // 输入参数存在错误
        MAX_TYPE
    };

    struct PlayerEchoInfo {
        PlayerEchoInfo();
        ara::core::String fileName;
        ara::core::String portSec;
        bool   isRunning;
        bool   isTracing;
        double delayTime;
        double bagTime;
        std::atomic<double> duration;
        std::atomic<double> totalTime;
        std::set<std::string> e2eEventNameList;
    };

    struct PortUse {
        rtf::stdtype::uint16_t port;
        bool inUse;
    };

    using PrintCallBack = std::function<void(PlayerPrintType, const PlayerEchoInfo&)>;

    RtfPlayer(const PlayerOptions& options);
    ~RtfPlayer();
    bool Publish();
    void Pause();
    void Step();
    void Trace();
    void Stop();
    void RegPrintCallback(const PrintCallBack& callback);
    double GetPlayProgress() const;

protected:
    void ShowThePlayerOptions();
    bool OpenRead();
    void CalculateTime(uint64_t& initTime, uint64_t& finishTime);
    bool Run(const uint64_t& initTime, const uint64_t& finishTime);
    bool DoRun(std::unique_ptr<RtfView>& viewPtr);
    bool IsUsingRawData(const Connection& connection);
    bool CreatePublisher(const std::string& eventName, const Connection& connection);
    void Advertise(ara::core::Vector<Connection const *> const& connections);
    bool DoPublish(RtfMsgEntity const& msgEntity, RtfBuffer& buffer);
    bool PreparePubMsg(const RtfBuffer& buffer, const bool isUsingRawData);
    bool PubMsg(uint64_t const &time, const bool isUsingRawData);
    bool IsSkipEvent() const;
    void PauseStart();
    void PauseEvent();
    bool IsPauseEvent() const;
    bool PubImmediate(const bool isUsingRawData);
    bool PubSkipEmpty(const bool isUsingRawData);
    bool PubFrequency(const bool isUsingRawData);
    void ProcessPause();
    void PubStep(const bool isUsingRawData);
    void SetMsgPubTime(uint64_t const& time);
    bool DoPubMsg(const bool isUsingRawData);

private:
    void DDSConfig(const ara::core::String& event, const ara::core::String& dataType,
                    rtf::maintaind::DDSEventInfo& ddsInfo, const bool isUsingRawData);
    void SOMEIPEventConfig(const ara::core::String& eventName, const ara::core::String& dataType,
                    rtf::maintaind::SomeipEventInfo& someipInfo);
    void SOMEIPServiceConfig(const ara::core::String& event, const ara::core::String& dataType,
                    rtf::maintaind::SomeipEventInfo& someipInfo);
    ara::core::String GetInstanceName(ara::core::String eventName);
    void PrintInfo(rtf::maintaind::DriverType driverType, const ara::core::String &eventName);
    void DevideConnections(
                        ara::core::Vector<Connection const *> & connections,
                        ara::core::Vector<Connection const *> & connectionsDDS,
                        ara::core::Vector<Connection const *> & connectionsSOMEIP);
    bool AdvertiseEvents(const ara::core::Vector<Connection const *> &connectionsDDS,
                         const ara::core::Vector<Connection const *> &connectionsSOMEIP);
    bool InitSOMEIPPubPort(ara::core::Vector<Connection const *> const& connections);
    bool InitSOMEIPNetwork(ara::core::Vector<Connection const *> const& connections);
    void PrintSomeipInfo(const ara::core::String& eventName,
                         const ara::core::String& dataType,
                         const rtf::maintaind::SomeipEventInfo& someipInfo);
    void InitNodeHandle();
    void CheckPortStatusInService(rtf::stdtype::uint16_t serviceId,
                                  ara::core::Map<rtf::stdtype::uint16_t, PortUse> portInfoWithIid);
    bool CheckPortStatus();
    bool CheckPublishers();
    bool IsPortInUse(const rtf::stdtype::uint16_t& port);
    bool GetInputPorts(const std::string& port);
    bool CheckThePlayerOptions();
    bool CheckInputPortInfo(const std::string& port) noexcept;
    bool CheckPortValue(const ara::core::String& portVal);
    bool GetInputPubPort();
    void UpdatePortMapInService(rtf::stdtype::uint16_t serviceId,
            ara::core::Map<rtf::stdtype::uint16_t, PortUse> portInfoWithIid);
    void UpdatePortMap();
    ara::core::Vector<ara::core::String> CollectEvents(const PlayerOptions& options);

    std::shared_ptr<rtf::com::config::E2EConfig> QueryDDSE2EConfigInfo(
        const ara::core::String& eventName, const rtf::maintaind::DDSEventInfo& ddsInfo);
    std::shared_ptr<rtf::com::config::E2EConfig> QuerySomeipE2EConfigInfo(
        const ara::core::String& eventName, const rtf::maintaind::SomeipEventInfo& someipInfo);
    rtf::maintaind::E2EConfigInfo QueryE2EConfigInfoForAll(const rtf::maintaind::QuerySubEventInfoIndex& index);
    bool QueryE2EConfigInfoForOne(const std::shared_ptr<rtftools::common::RTFMaintaindToolsServiceProxy>& proxy,
    const rtf::maintaind::QuerySubEventInfoIndex& index, ::rtf::maintaind::E2EConfigInfo& e2eConfig);

    PlayerOptions options_;
    ara::core::Vector<std::shared_ptr<RtfBagFile>> bags_;
    uint64_t      startTime_;
    bool          isPaused_;
    bool          isPauseStart_;
    bool          isDoStep_;
    bool          isStop_;
    bool          isPauseEventEnable_;      // is pause event enable
    bool          isPauseEvents_;        // is has --pause-events parameters

    std::shared_ptr<rtf::com::NodeHandle> node_;
    using PublisherMap = ara::core::Map<ara::core::String, rtf::com::Publisher<vrtf::core::RawBuffer>>;
    using RawDataPublisherMap = ara::core::Map<ara::core::String, rtf::com::Publisher<vrtf::core::RawBufferForRawData>>;
    using PublisherPortMap = ara::core::Map<rtf::stdtype::uint16_t, ara::core::Map<rtf::stdtype::uint16_t, PortUse>>;
    PublisherMap publishers_;
    RawDataPublisherMap rawDataPublisher_;
    PublisherPortMap pubPorts_;
    rtf::stdtype::uint16_t minPort_;
    rtf::stdtype::uint16_t maxPort_;
    std::set<rtf::stdtype::uint16_t> portSet_;
    std::queue<rtf::stdtype::uint16_t> availablePortQue_;
    rtf::stdtype::uint16_t currentPort_;
    int requiredPorts_;
    ara::core::String pubEventName_;
    vrtf::core::RawBufferForRawData pubMsg_;
    bool ddsCreateFailed_;
    bool someipCreateFailed_;
    std::atomic<bool> isPausedStart_;

    MilliTimePoint  pausedTime_;
    MilliTimePoint  sysHorizon_;
    MilliTimePoint  frePubStartTime_;
    PrintCallBack   printCallback_;
    PlayerEchoInfo  printInfo_;
    rtf::rtfbag::RtfPubClock       pubClock_;
    rtf::rtfbag::RtfTimeTranslator timeTranlator_;
    std::atomic<double> duration_;
    std::atomic<double> totalTime_;
    std::shared_ptr<rtf::rtftools::common::ToolsCommonClientManager> toolsCommonClientManager_ = nullptr;
    std::set<std::string> e2eEventNameList_;
};
}  // namespace rtfbag
}  // namespace rtf
#endif // RTF_PLAYER_H
