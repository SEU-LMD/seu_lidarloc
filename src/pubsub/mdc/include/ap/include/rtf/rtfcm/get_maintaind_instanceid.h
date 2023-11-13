/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: Support to get maintaind use insatnceId
 * Create: 2021-06-19
 */
#ifndef GET_MAINTAIND_INSTANCEID_ID_H
#define GET_MAINTAIND_INSTANCEID_ID_H
#include <string>

namespace rtf {
namespace rtfcm {
namespace rtfmaintaind {
class GetMaintaindInstanceId {
public:
    static std::string GetInstanceId();
};
}
}
}
#endif