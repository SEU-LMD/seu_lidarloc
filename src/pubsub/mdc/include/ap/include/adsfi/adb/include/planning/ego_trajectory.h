/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  ego_trajectory.h Planning中所使用的结构体,用于表征轨迹
 */

#ifndef HAF_PLANNING_EGO_TRAJECTORY_H__
#define HAF_PLANNING_EGO_TRAJECTORY_H__

#include "core/types.h"
namespace Adsfi {
    /* ******************************************************************************
        结构 名		:  HafWayPoint
        功能描述		:  生成的Path点信息
    ****************************************************************************** */
    struct HafWayPoint {
        HafHeader header;
        float64_t x;                        // utm坐标系下x值
        float64_t y;                        // utm坐标系下y值
        float64_t z;                        // utm坐标系下z值
        float64_t theta;                    // 该点处path切线方向与正东方向夹角
        float64_t curvature;                // path在该点处的曲率
        float64_t s;                        // sl坐标系下的s值
        float64_t deltaCurvature;             // 该点处曲率的导数
        std::string laneId;              // 该点所在的lane的ID
        float64_t deltaX;                     // path在该点处x方向的导数
        float64_t deltaY;                     // path在该点处y方向的导数
    };
    /* ******************************************************************************
        结构 名		:  HafEstop
        功能描述		:  紧急停车信息
    ****************************************************************************** */
    struct HafEstop {
        HafHeader header;
        uint8_t isEstop;                  // 是否紧急停车
        std::string description;          // 紧急停车原因
    };

    /* ******************************************************************************
        结构 名		:  HafTrajectoryPoint
        功能描述		:  轨迹点信息
    ****************************************************************************** */
    struct HafTrajectoryPoint {
        HafHeader header;
        HafWayPoint wayPoint;            // 轨迹上的path信息
        float64_t speed;                     // 轨迹点的速度值
        float64_t acc;                       // 轨迹点的加速度值
        float64_t timeRelative;              // 轨迹点的相对规划起始点的时间
        float64_t deltaAcc;                  // 轨迹点的加加速度
        float64_t steerAngle;                // 当前时刻前轮转向角
    };

    /* ******************************************************************************
        结构 名		:  HafEgoTrajectory
        功能描述		:  轨迹信息
    ****************************************************************************** */
    struct HafEgoTrajectory {
        HafHeader egoTrajectoryHeader;
        float64_t trajectoryLength;                                   // path总长度
        float64_t trajectoryPeriod;                                   // path总时长
        std::vector<HafTrajectoryPoint> trajectoryPoints;          // 轨迹点集
        HafEstop estop;                                            // 紧急停车
        std::vector<HafWayPoint> wayPoints;                        // path点集
        bool isReplanning;                                // 该轨迹的规划起始点是否是重新选择的
        uint32_t gear;                                        // 前进档/倒档
        HafHeader routingHeader;                                   // 导航起始时间
        std::string selfLaneId;                                    // 自车所在的lane ID
        uint32_t trajectoryType;                              // 轨迹类型，unknown, normal等
        std::string targetLaneId;                                // lane change时的目标lane ID
        uint8_t turnLight;                                      // 转向灯信息
        bool isHold;                                            // 是否保持0速停车
    };
}
#endif // HAF_PLANNING_EGO_TRAJECTORY_H__
