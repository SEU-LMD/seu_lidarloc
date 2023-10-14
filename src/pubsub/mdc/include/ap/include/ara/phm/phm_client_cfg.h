/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: Phm client config header
 * Create: 2021-01-28
 */
#ifndef ARA_PHM_CLIENT_CFG_H
#define ARA_PHM_CLIENT_CFG_H

#include <string>

namespace ara {
namespace phm {
class PhmClientCfg {
public:
    PhmClientCfg() = default;
    ~PhmClientCfg() = default;
    static void SetUser(std::string const &user);
    static std::string GetUser();
};
} // namespace phm
} // namespace ara
#endif

