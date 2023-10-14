/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:  samm_scene.h Samm对外发出的场景信息
 * Create: 2020-10-07
 */

#ifndef SAMM_SAMM_SCENE_H
#define SAMM_SAMM_SCENE_H

#include <vector>
#include <string>
#include "samm_dag.h"
#include "core/types.h"
namespace Adsfi {
    struct HafScene {
        uint32_t id;
        HafSammDag sceneDag;
    };

    struct HafSammScene {
        HafHeader header;
        HafScene scene;
    };
}
#endif // SAMM_SAMM_SCENE_H
