/* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: rtf method API
 * Author: wangweiyang 00517688
 * Create: 2020-10-27
 * Notes: NA
 * History: 2020-10-27 wangweiyang
 */

#ifndef RTFTOOLS_RTFMETHOD_H
#define RTFTOOLS_RTFMETHOD_H

#include "ara/core/string.h"
#include "ara/core/vector.h"
#include "json_parser/document.h"
#include "json_parser/global.h"

#include "rtf/internal/tools_common_client_manager.h"

using Document = ara::godel::common::jsonParser::Document;
using JsonParseValue = ara::godel::common::jsonParser::JsonParseValue;
using rtf::maintaind::proxy::methods::QueryMethodInfo;
using rtf::maintaind::proxy::methods::QueryMethodType;
using rtf::maintaind::proxy::methods::QueryDataType;
using rtf::maintaind::MethodRegisterInfo;

namespace rtf {
namespace rtfmethod {
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

enum class RtfMethodRetType : int {
    SUCCESS = 0,
    FAILURE = -1,
    PARTIAL_FAILURE = -2
};

struct DataTypeReturnValue {
    ara::core::String name;
    ara::core::String type;
    ara::core::String dataType;
    ara::core::String value;
    ara::core::String isConstant;
};

struct MethodShowReturnValue {
    ara::core::String dataTypeName;
    ara::core::String dataType;
};

/**
 * the class to store basic method info
 * @example Name Example <br>
 * MethodName = ${MethodType}[${instanceShortname}] <br>
 * ServerName is ${AppName} also is ${Node}
 * @note instanceShortname could be empty
 */
class RtfMethodInfo {
public:
    RtfMethodInfo() = default;
    ~RtfMethodInfo() = default;

    const ara::core::String GetMethodName() const;
    void SetMethodName(const ara::core::String& type2set);

    const ara::core::String& GetMethodType() const;
    void SetMethodType(const ara::core::String& type2set);

    const ara::core::Vector<ara::core::String> & GetRequestArgs() const;
    void SetRequestArgs(const ara::core::Vector<ara::core::String>& args);

    const ara::core::Vector<ara::core::String>& GetResponseArgs () const;
    void SetResponseArgs (const ara::core::Vector<ara::core::String>& args);

    const ara::core::String& GetServer() const;
    void SetServer(const ara::core::String& server);

    const ara::core::String& GetInstanceShortName() const;
    void SetInstanceShortName(const ara::core::String& instanceShortName);

private:
    /** methodName without instanceShortName */
    ara::core::String methodType_;
    /** equal to APP name */
    ara::core::String server_;
    /** instance name is surrounded by '[' ']' */
    ara::core::String instanceShortName_;
    ara::core::Vector<ara::core::String> reqArgs_;
    ara::core::Vector<ara::core::String> respArgs_;
};

class RtfMethod {
public:
    RtfMethod();
    ~RtfMethod() = default;
    int Query(const ara::core::String methodName, RtfMethodInfo &rtfMethodInfo);
    int QueryAll(ara::core::Vector<RtfMethodInfo> &rtfMethodInfoList);
    int QueryAllWithNamespace(const ara::core::String nameSpace, ara::core::Vector<RtfMethodInfo> &rtfMethodInfoList);
    int QueryMethodShowInfo(const ara::core::String methodName,
                            ara::core::Vector<MethodShowReturnValue> &rtfMethodReqDataTypeList,
                            ara::core::Vector<MethodShowReturnValue> &rtfMethodRepDataTypeList);
    int QueryDataTypeInfo(const ara::core::String dataTypeName,
                          ara::core::Vector<DataTypeReturnValue> &rtfDataTypeMsg);
    int Init();

private:
    bool GetShowResult(const QueryMethodType::Output outPut,
                       ara::core::Vector<MethodShowReturnValue> &rtfMethodReqDataTypeList,
                       ara::core::Vector<MethodShowReturnValue> &rtfMethodRepDataTypeList);
    void GetInfoResult(const QueryMethodInfo::Output outPut,
                       ara::core::Vector<MethodRegisterInfo> &methodInfoWithPubSubListTmp);
    void GetListResult(ara::core::Vector<MethodRegisterInfo> &methodRegisterInfoList,
        ara::core::Vector<RtfMethodInfo> &rtfMethodInfoList);
    void FilterMethodList(const ara::core::String pubName,
                          const ara::core::Vector<MethodRegisterInfo> methodRegisterInfoList,
                          RtfMethodInfo &rtfMethodInfo);
    bool isInit_ = false;
    std::shared_ptr<rtf::rtftools::common::ToolsCommonClientManager> toolsCommonClientManager_ = nullptr;
};
}
}
#endif // RTFTOOLS_RTFMETHOD_H
