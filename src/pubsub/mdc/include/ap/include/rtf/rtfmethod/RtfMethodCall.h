/* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: rtf method API
 * Author: wangweiyang 00517688
 * Create: 2020-12-03
 * Notes: NA
 * History: 2020-12-03 wangweiyang
 */

#ifndef RTFTOOLS_RTFMETHOD_CALL_H
#define RTFTOOLS_RTFMETHOD_CALL_H

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
#include "ara/core/vector.h"
#include "ara/core/string.h"
#include "ara/core/map.h"

using Document = ara::godel::common::jsonParser::Document;
using JsonParseValue = ara::godel::common::jsonParser::JsonParseValue;
using rtf::maintaind::proxy::methods::QueryMethodInfo;
using rtf::maintaind::proxy::methods::QueryMethodType;
using rtf::maintaind::proxy::methods::QueryDataType;
using rtf::maintaind::MethodRegisterInfo;

namespace rtf {
namespace rtfmethodcall {
struct MethodCallInfo {
    ara::core::String e2eCheckResult;
    ara::core::String replyResponse;
};

struct CallOptions {
    ara::core::Vector<ara::core::String> reqArgsStringList;
    ara::core::String inputParamJsonPath;
    ara::core::String ddsNet;
    ara::core::String someipNet;
};

enum class RtfmethodCallType : uint8_t {
    ALL_RIGHT                      = 0,    // 订阅消息
    WARN_NO_SOMEIP_NET             = 1,    // 没有输入SOME/IP网卡network名称
    ERR_SERVER_STATUS              = 2,    // method server不存在或者存在多个相同server
    ERR_REQ_MSG                    = 3,    // 缺少入参的msg文件或序列化错误
    ERR_REP_MSG                    = 4,    // 缺少出参的msg文件或反序列化错误
    ERR_INPUT_VALUE_NUM            = 5,    // 输入参数个数存在错误
    ERR_INPUT_VALUE_TYPE           = 6,    // 输入参数值存在错误
    ERR_REQUEST_TIME_OUT           = 7,    // method请求超时
    ERR_REPLY_VALUE                = 8,    // method返回值错误，触发流控或者异常导致
    ERR_CREATE_SOMEIP              = 9,    // SOMEIP sub创建失败
    ERR_CREATE_DDS                 = 10,   // DDS sub创建失败
    GET_RESPONSE                   = 11,   // 获取到返回值
    NO_RESPONSE                    = 12,   // 没有返回值，fireandforget场景或者返回值注册信息不全
    ERR_INPUT_FILE_PATH            = 13,   // 输入json文件不存在
    ERR_CHECK_E2E                  = 14    // E2E保护结果存在错误时，将使用该错误码
};

class RtfMethodCall {
public:
    typedef std::function<void(MethodCallInfo, RtfmethodCallType)> CallResultCallback;
    RtfMethodCall();
    ~RtfMethodCall() = default;
    bool Init(const ara::core::String &method, CallOptions callOptions, CallResultCallback callback);
    int Call();
    void Stop();
private:
    class MethodType {
    public:
        struct Request {
            vrtf::core::RawBuffer data;
            static bool IsPlane()
            {
                return false;
            }
            using IsDpRawDataTag = void;
            using IsEnumerableTag = void;
            template<typename F>
            void enumerate(F& fun)
            {
                fun(data);
            }
            template<typename F>
            void enumerate(F& fun) const
            {
                fun(data);
            }
            bool operator == (const Request& t) const
            {
                return (data == t.data);
            }
        };
        struct Response {
            vrtf::core::RawBuffer data;
            static bool IsPlane()
            {
                return false;
            }
            using IsDpRawDataTag = void;
            using IsEnumerableTag = void;
            template<typename F>
            void enumerate(F& fun)
            {
                fun(data);
            }
            template<typename F>
            void enumerate(F& fun) const
            {
                fun(data);
            }
            bool operator == (const Response& t) const
            {
                return (data == t.data);
            }
        };
        Request  req;
        Response res;
    };
    bool InitInputParams();
    bool SerializeInputParams(MethodType& data);
    bool RtfMethodCallReady() const;
    void CreateMethod(const ara::core::String& method);
    void FilterMethodList(const ara::core::String pubName,
                          ara::core::Vector<MethodRegisterInfo> &methodRegisterInfoList);
    void PrintDDSConfig(rtf::maintaind::MethodRegisterInfo& ddsInfo);
    void DDSConfig(const ara::core::String& method, rtf::maintaind::MethodRegisterInfo& ddsInfo);
    void SOMEIPConfig(const ara::core::String& method, rtf::maintaind::MethodRegisterInfo& someipInfo);
    void PrintInfo(rtf::maintaind::DriverType driverType, const ara::core::String& method);
    ara::core::String GetInstanceName(ara::core::String methodName);
    void CreateMethodClient(const ara::core::String& method,
        ara::core::Vector<rtf::maintaind::MethodRegisterInfo>& methodInfoWithPubSubList);
    void CheckMethodClient(const ara::core::String& method, bool ddsExist, bool someipExist);
    void GetPubSub(const QueryMethodInfo::Output outPut,
        ara::core::Vector<rtf::maintaind::MethodRegisterInfo>& methodInfoWithPubSubList);
    ara::core::String QueryTypeDefinitionFromMaintaind(const ara::core::String& dataType) const noexcept;
    bool QueryAllTypeFromMaintaind();
    void PrintUserInfo(const MethodCallInfo& info, const RtfmethodCallType type) const;
    int DealWithCallResult(const rtf::com::MethodClientResult& callResult,
                           const vrtf::core::RawBuffer& response,
                           const std::size_t& outputTypeSize);
    int CheckIsValidAndDoCall();
    RtfmethodCallType CheckAndSetE2EResult(const rtf::com::MethodClientResult& callResult, MethodCallInfo& info);
    std::string DoDeserialize(const rtf::com::MethodClientResult& callResult, const rtf::com::ErrorCode& err,
        const vrtf::core::RawBuffer& response);
    bool SerializeCommandLineInput(ara::core::Vector<rtf::rtfmethodcall::BitBuffer> &bitBufferList,
        std::size_t& dataLength);
    bool SerializeJsonFileInput(ara::core::Vector<rtf::rtfmethodcall::BitBuffer> &bitBufferList,
        std::size_t& dataLength);
    ara::core::String method_;
    CallOptions callOption_;
    rtf::com::MethodClient methodClient_;
    bool methodCallEnable_;
    bool ddsCreateError_;
    bool someipCreateError_;
    ara::core::Vector<Document> documentList_;
    ara::core::Vector<ara::core::String> requestArgsTypeString_;
    ara::core::String replyArgsTypeString_;
    ara::core::String replyOutputName_;
    ara::core::String fieldName_;
    rtf::maintaind::DriverType protocol_;
    rtf::maintaind::SerializationType serializeTypeRequest_;
    rtf::maintaind::SerializationType serializeTypeReply_;
    rtf::maintaind::StructSerializationPolicy structPolicyRequest_;
    rtf::maintaind::StructSerializationPolicy structPolicyReply_;
    ara::core::Map<ara::core::String, ara::core::String> dataTypeJsonList_;
    std::mutex mutex_;
    std::shared_ptr<rtf::com::NodeHandle> nh_;
    CallResultCallback callResultCallback_;     // 回调函数保存
    bool isInit_ = false;
    bool isFireAndForget_ = false;
    bool isEnableE2E_ = false;
    const std::map<rtf::com::e2e::ProfileCheckStatus, std::string> ProfileStatusMappingToString = {
        {rtf::com::e2e::ProfileCheckStatus::kOk, "kOk"},
        {rtf::com::e2e::ProfileCheckStatus::kRepeated, "kRepeated"},
        {rtf::com::e2e::ProfileCheckStatus::kWrongSequence, "kWrongSequence"},
        {rtf::com::e2e::ProfileCheckStatus::kError, "kError"},
        {rtf::com::e2e::ProfileCheckStatus::kNotAvailable, "kNotAvailable"},
        {rtf::com::e2e::ProfileCheckStatus::kNoNewData, "kNoNewData"},
        {rtf::com::e2e::ProfileCheckStatus::kCheckDisabled, "kCheckDisabled"}
    };
    std::shared_ptr<rtf::rtftools::common::ToolsCommonClientManager> toolsCommonClientManager_ = nullptr;
};
}
}
#endif // RTFTOOLS_RTFMETHOD_CALL_H
