/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:  dag.h是samm中用于描述存放场景对应DAG的结构
 */

#ifndef SAMM_SAMM_DAG_H
#define SAMM_SAMM_DAG_H

#include <vector>
#include <string>
namespace Adsfi {
struct HafTaskDescription {
    std::string taskName;   // Dag对应的Task名称
    std::vector<std::string> jobList;   // Job的名称列表
};

struct HafSammDag {
    std::string sceneName;              // Dag对应的场景名称
    uint32_t sceneId;                   // Dag对应的场景Id
    std::vector<HafTaskDescription> tasks; // Dag对应的Task名称
    HafSammDag(): sceneName(""), sceneId(0U), tasks({}){}
};
} // namespace Adsfi
#endif // SAMM_SAMM_DAG_H
