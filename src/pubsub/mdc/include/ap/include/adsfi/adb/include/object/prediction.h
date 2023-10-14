/* *
 * FUNCTION: Define Prediction Struct
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 *        */
#ifndef HAF_OBJECT_PREDICTION_H
#define HAF_OBJECT_PREDICTION_H

#include <vector>
#include "core/types.h"
#include "core/timer.h"
namespace Adsfi {
const uint32_t COVARIANCE_SIZE_IN_PREDICTION = 9U;
struct HafPathPoint {
    Point3d point;
    float64_t s;       // 轨迹的累计行进s距离
    float64_t theta;   // 轨迹点朝向角
    float64_t kappa;   // 曲率
    float64_t dkappa;  // 曲率一次导数
    float64_t heading; // 轨迹点朝向
    float64_t ddkappa; // 曲率二次导数
};

struct HafTrajectoryPointInPrediction {
    HafPathPoint pathPoint;
    float64_t velocity;
    float64_t acceleration;
    float64_t relativeTime; // 相对时间
};

struct HafTrajectoryInPrediction {
    float64_t probability; // 置信度
    std::vector<HafTrajectoryPointInPrediction> trajectoryPoints;
};

struct HafPredictionTrajectoryPoint {
    Point3d point; // 预测的路径点，复用Point类型，将velocity填入z字段
    HafTime timestamp;
};
struct HafObstacleFeature {
    uint32_t id;                       // 障碍物id
    std::vector<Point3d> polygonPoint; // 多边形顶点
    Point3d position;                  // 中心位置
    Point3d frontPosition;             // 车头位置
    Point3d velocity;                  // 障碍物速度
    Point3d rawVelocity;               // 感知传入的速度
    Point3d acceleration;              // 加速度
    float64_t velocityHeading;         // 速度朝向
    float64_t speed;                   // 速度
    float64_t acc;                     // 加速度
    float64_t theta;                   // 车头朝向角
    float64_t length;                  //
    float64_t width;
    float64_t height;
    float64_t trackingTime;
    HafTime timestamp;
    Point3d tPosition;              // 被追踪到的位置
    Point3d tVelocity;              // 被追踪到的速度
    float64_t tVelocityHeading;     // 被追踪到的速度朝向
    float64_t tSpeed;               // 被追踪到的速率
    Point3d tAcceleration;          // 被追踪到的加速度
    float64_t tAcc;                 // 被追踪到的加速率
    bool isStill = false;           // 是否是静止，默认值为false
    uint8_t type;                   // 感知到的障碍物类型
    float64_t labelUpdateTimeDelta; // 时间更新标记位
    uint8_t priority;               // 障碍物优先级  1-CAUTION,    2-NORMAL,  3-IGNORE
    bool isNearJunction = false;    // 是否靠近路口
    std::vector<HafPredictionTrajectoryPoint> futureTrajectoryPoints;          // 未来时刻轨迹点
    std::vector<HafTrajectoryPointInPrediction> shortTermPredictedTrajectoryPoints; // 短距离预测的轨迹点
    std::vector<HafTrajectoryInPrediction> predictedTrajectory;                // 预测的轨迹
    std::vector<HafTrajectoryPointInPrediction> adcTrajectoryPoint;                 // 自动驾驶车辆的轨迹点
};

struct HafPerceptionObstacle {
    uint32_t id;
    Point3d position;         // 地图坐标系下障碍物的Point位置
    float64_t theta;          // 地图坐标系下障碍物姿态角，长边与x轴的夹角，0-360，单位角度
    Point3d velocity;         // 速度，障碍物在地图坐标系下的速度分量
    bool hasVelocity = false; // 是否有速度，默认值为false
    Point3d size;             // 障碍物长、宽、高
    std::vector<Point3d> polygonPoint; // 地图坐标系下多边形点集合
    float64_t trackingTime;            // 追踪时间，保留位
    uint8_t type; // 0-Unknown,1-Unknown_movable,2-Unknown_unmovable,3-Pedestrian,4-Bicycle,5-Vehicle
    float64_t confidence;
    HafTime gpsTime;                  // 时间戳，记录GPS时间
    uint8_t confidenceType;           // 置信度类型 0-CONFIDENCE_UNKNOWN, 1-CONFIDENCE_CN, 2-CONFIDENCE_RAD
    std::vector<Point3d> drops;       // 障碍物轨迹
    Point3d acceleration;             // 障碍物加速度
    Point3d anchorPoint;              // 障碍物的锚点
    std::vector<Point3d> boundingBox; // 障碍物bounding_box　按顺时针顺序排列
    uint8_t subType;                  // 障碍物子类别
    float64_t heightAboveGround;      // 障碍物最低点离地垂直高度
    std::array<float64_t, COVARIANCE_SIZE_IN_PREDICTION> positionCovariance;     // 位置协方差
    std::array<float64_t, COVARIANCE_SIZE_IN_PREDICTION> velocityCovariance;     // 速度协方差
    std::array<float64_t, COVARIANCE_SIZE_IN_PREDICTION> accelerationCovariance; // 加速度协方差
    // 障碍物车辆信号灯状态：0-OFF,1-LEFT_TURN_VISIBLE,2-LEFT_TURN_ON,
    // 3-RIGHT_TURN_VISIBLE,4-RIGHT_TURN_ON,5-BRAKE_VISIBLE,6-BRAKE_ON,7-UNKNOWN
    uint8_t lightStatus;
};

struct HafPredictionObstacle {
    HafPerceptionObstacle object;
    HafTime gpsTime;
    float64_t predictedPeriod;                            // 预测时长
    std::vector<HafTrajectoryInPrediction> trajectories; // 障碍物的多条轨迹
    uint8_t intent;                                    // 障碍物意图
    uint8_t priority;                                  // 障碍物优先级
    bool isStatic;
    HafObstacleFeature feature; // 障碍物详细特征
};

struct HafObjPredictionOutArray {
    HafHeader header;
    std::vector<HafPredictionObstacle> predictionObstacle;
    HafTime startTime;
    HafTime endTime;
    uint8_t changeOriginFlag; // 0： 坐标系切换成功，1：坐标系切换中，2：坐标系切换故障
    uint8_t selfIntent;       // 自动驾驶车辆意图 0-UNKNOWN, 1-STOP, 2-CRUISE, 3-CHANGE_LANE
    uint16_t scenario;        // 场景 0-UNKNOWN, 1000-CRUISE, 1001-CRUISE_URBAN, 1002-CRUISE_HIGHWAY, 2000-JUNCTION,
                              // 2001-JUNCTION_TRAFFIC_LIGHT, 2002-JUNCTION_STOP_SIGN
};
}
#endif