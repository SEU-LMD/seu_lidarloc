/* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: rtf node API .h file
 * Author: 刘伟 l00500320
 * Create: 2020-02-07
 * Notes:
 * History:
 */

#ifndef RTFTOOLS_RTFNODE_H
#define RTFTOOLS_RTFNODE_H

#include "ara/core/vector.h"
#include "ara/core/string.h"
#include "rtf/stdtype/impl_type_int32_t.h"
#include <chrono>
#include <numeric>
#include <mutex>
#include <set>
#include <functional>
#include <cmath>
#include "rtf/internal/tools_common_client_manager.h"

using QueryNodeInfo = rtf::maintaind::proxy::methods::QueryNodeInfo;

namespace rtf {
namespace rtfnode {
/**
 * the class to store basic node info
 * @example Name Example <br>
 * ApplicationName = ${NodeName}[${HostName}] <br>
 * @note HostName should never be empty, unless you input an error value
 */
class RtfNodeInfo {
public:
    RtfNodeInfo() = default;
    ~RtfNodeInfo() = default;
    ara::core::Vector<ara::core::String> GetServicInstance() const;
    void SetServiceInstances(ara::core::Vector<ara::core::String> serviceInstances);

    ara::core::Vector<ara::core::String> GetEventPubList() const;
    void SetPubs(ara::core::Vector<ara::core::String> pubs);

    ara::core::Vector<ara::core::String> GetEventSubList() const;
    void SetSubs(ara::core::Vector<ara::core::String> subs);

    ara::core::Vector<ara::core::String> GetMethodList() const;
    void SetMethods(ara::core::Vector<ara::core::String> methods);

    ara::core::Vector<ara::core::String> GetFieldList() const;
    void SetFields(ara::core::Vector<ara::core::String> fields);

    int32_t GetPidValue() const;
    void SetPidValue(const int32_t value);

    ara::core::String GetApplicationName() const;
    void SetApplicationName(const ara::core::String applicationName);

    const ara::core::String& GetNodeName() const;
    void SetNodeName(const ara::core::String& nodeName);

    const ara::core::String& GetHostName() const;
    void SetHostName(const ara::core::String& hostName);

private:
    ara::core::Vector<ara::core::String> serviceInstanceList_;
    ara::core::Vector<ara::core::String> eventPubList_;
    ara::core::Vector<ara::core::String> eventSubList_;
    ara::core::Vector<ara::core::String> methodList_;
    ara::core::Vector<ara::core::String> fieldList_;
    int32_t pid_ = 0;
    ara::core::String nodeName_;
    ara::core::String hostName_;
};

class RtfNode {
public:
    RtfNode();
    ~RtfNode() = default;
    int Query(const ara::core::String nodeName, RtfNodeInfo &rtfNodeInfo);
    int QueryAll(ara::core::Vector<RtfNodeInfo> &rtfNodeInfoList);
    int QueryAllWithNamespace(const ara::core::String &nodeNamespace, ara::core::Vector<RtfNodeInfo> &rtfNodeInfoList);
    int Init();

private:
    bool GetQueryInfoResult(const QueryNodeInfo::Output outPut, RtfNodeInfo &rtfNodeInfo);
    void GetQueryListResult(QueryNodeInfo::Output outPut, QueryNodeInfo::Output &nodeInfoListTmp);
    void SetQueryListResultWithFilter(QueryNodeInfo::Output outPut,
                                      const ara::core::String nodeNamespace,
                                      ara::core::Vector<RtfNodeInfo> &rtfNodeInfoList);
    void SetQueryListResult(QueryNodeInfo::Output outPut,
                            ara::core::Vector<RtfNodeInfo> &rtfNodeInfoList);
    bool isInit_ = false;
    std::shared_ptr<rtf::rtftools::common::ToolsCommonClientManager> toolsCommonClientManager_ = nullptr;
};
}
}
#endif
