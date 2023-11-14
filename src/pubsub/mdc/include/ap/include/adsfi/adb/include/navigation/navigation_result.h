/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  navigationResult.h 导航结果,用于表征轨迹
 */

#ifndef HAF_ROUTING_NAVIGATION_RESULT_H
#define HAF_ROUTING_NAVIGATION_RESULT_H

#include "core/types.h"
namespace Adsfi {
    /* ******************************************************************************
        结构 名		:  HafRoadType
        功能描述		:  道路类型
    ****************************************************************************** */
    enum class HafRoadType {
        REAL = 0,   // 真实道路
        VIRTUAL = 1 // 虚拟道路
    };

    /* ******************************************************************************
        结构 名		:  HafDirectionType
        功能描述		:  Lane方向属性
    ****************************************************************************** */
    enum class HafDirectionType {
        FORWARD = 0, // 直行
        LEFT = 1,    // 左
        RIGHT = 2    // 右
    };

    /* ******************************************************************************
        结构 名		:  HafLanePiece
        功能描述		:  LanePiece Lane片段结构体
    ****************************************************************************** */
    struct HafLanePiece {
        int32_t id;     // laneId
        float64_t startS; // Lane的开始路点累加值
        float64_t endS;   // Lane的终点s累加值
    };

    /* ******************************************************************************
        结构 名		:  HafLaneSeries
        功能描述		:  纵向多条车道集合结构体
    ****************************************************************************** */
    struct HafLaneSeries {
        bool isDrivable;                          // 当前LaneSeries是否断头
        HafDirectionType directionType;           // 方向
        std::vector<HafLanePiece> laneSeries;     // 车道集合
    };

    /* ******************************************************************************
        结构 名		:  HafRoadPiece
        功能描述		:  RoadPiece 道路片段结构体
    ****************************************************************************** */
    struct HafRoadPiece {
        int32_t id;                           // RoadId
        HafRoadType roadType;                 // Road类型
        std::vector<HafLaneSeries> roadPiece; // LaneSeries集合
    };

    /* ******************************************************************************
        结构 名		:  HafNaviRoadPoint
        功能描述		:
    ****************************************************************************** */
    struct HafNaviRoadPoint {
        int32_t id;
        float64_t s;
        Point3d pose;
    };

    /* ******************************************************************************
        结构 名		:  HafNavigationResult
        功能描述		:  Navigation全局导航结构体
    ****************************************************************************** */
    struct HafNavigationResult {
        bool isNewRouting = false;                             // 是否重新导航
        HafHeader naviResultHeader;                            // 时间戳
        std::string mapVersion;                                // 地图版本信息
        std::vector<HafRoadPiece> roadPieces;                  // roadPiece信息
        std::vector<HafNaviRoadPoint> selectPoints;            // 导航起始点，途经点，终点
        std::vector<std::string> blacklistRoads;               // 黑名单road标识
        std::vector<HafLanePiece> blacklistLaneSegments;       // 黑名单lane标识
    };
}
#endif // HAF_ROUTING_NAVIGATION_RESULT_H