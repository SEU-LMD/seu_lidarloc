/* *
 * FUNCTION: Define Object Struct
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *   */
#ifndef HAF_OBJECT_OBJECT_H
#define HAF_OBJECT_OBJECT_H

#include "core/types.h"
#include "core/timer.h"

#include <vector>
#include <list>
#include <iostream>
namespace Adsfi {
    template<typename T> struct Haf2dRectArray {
        uint32_t count;
        std::vector<HafRect2D<T>> rect;
    };

    /* ******************************************************************************
        结构名      :  Haf2dDetectionOut
        功能描述    :  障碍物2d检测结果
    ****************************************************************************** */
    template <typename T>
    struct Haf2dDetectionOut {
        int32_t objectID{};                    // 障碍物id
        int32_t cls{};                         // 障碍物类别
        float32_t confidence{};                // 障碍物类别置信度
        float32_t existenceProbability{};      // 障碍物存在的概率
        enum Coordinate coordinate { IMAGE };  // 坐标系类别
        HafRect2D<T> rect{};                   // 障碍物边框
        Point2f velocity{};                    // 障碍物在当前坐标系下的速度
        HafTime timeCreation{};                // 障碍物被识别的时间戳
        Point3d acceleration{};                // 障碍物相对主车的加速度
        uint8_t blinkerStatus{};               // 障碍物信号灯状态
        uint8_t cipvFlag{};                    // 障碍物标志状态  0-CIPV 1-CIPS 2-LPIHP 3-RPIHP 4-NONE
        uint8_t cameraStatus{};                // 相机状态
    };

    /* ******************************************************************************
        结构名      :  Haf3dDetectionOut
        功能描述    :  障碍物3d检测结果
    ****************************************************************************** */
    template <typename T>
    struct Haf3dDetectionOut {
        int32_t objectID{};                      // 障碍物id
        int32_t cls{};                           // 障碍物类别
        int32_t types{};                         // 障碍物类别（红绿灯用）
        float32_t confidence{};                  // 障碍物类别置信度
        float32_t existenceProbability{};        // 障碍物存在的概率
        enum Coordinate coordinate { VEHICLE };  // 坐标系类别
        HafRect3D<T> rect{};                     // 障碍物边框
        Point3f velocity{};                      // 障碍物在当前坐标系下的速度
        std::list<Point3D<T>> contourPoints;     // 障碍物在指定坐标系下的所有点云
        HafTime timeCreation{};                  // 障碍物被识别的时间戳
        Point3d acceleration{};                  // 障碍物相对主车的加速度
        uint8_t blinkerStatus{};                 // 障碍物信号灯状态
        uint8_t cipvFlag{};                      // 障碍物标志状态 0-CIPV 1-CIPS 2-LPIHP 3-RPIHP 4-NONE
        uint8_t cameraStatus{};                  // 相机状态
    };

    /* ******************************************************************************
        结构 名		:  Haf3dDetectionOutArray
        功能描述		:  障碍物列表3d检测结果
    ****************************************************************************** */
    template<typename T> struct Haf3dDetectionOutArray {
        HafTime timestamp{};
        int32_t count{};
        std::list<Haf3dDetectionOut<T>> detectionOut3d;
        std::list<Haf2dDetectionOut<T>> detectionOut2d;
        std::string frameID;          // 该帧数据的传感器源
        uint32_t seq{};
    };

    /* ******************************************************************************
        结构 名		:  HafFusionOut
        功能描述		:  融合后障碍物结果
    ****************************************************************************** */
    template<typename T> struct HafFusionOut {
        int32_t objectID{};                    // 障碍物id
        int32_t cls{};                         // 障碍物类别
        float32_t confidence{};                // 障碍物类别置信度
        float32_t existenceProbability{};      // 障碍物存在的概率
        HafRect3D<T> rect{};               // 障碍物边框（自车坐标系）
        HafRect3D<T> absRect{};            // 障碍物边框(世界坐标系)
        Point3f velocity{};                // 障碍物在当前坐标系下的速度
        Point3f absVelocity{};             // 障碍物在世界坐标系下的速度
        HafMatrix3d velocityCovariance{};        // 车身坐标系下障碍物速度协方差矩阵
        HafMatrix3d absVelocityCovariance{};     // 世界坐标系下障碍物速度协方差矩阵
        Point3d acceleration{};                  // 车身坐标系下障碍物的相对加速度
        Point3d absAcceleration{};               // 世界坐标系下障碍物加速度
        HafMatrix3d accelerationCovariance{};    // 车身坐标系下障碍物加速度协方差矩阵
        HafMatrix3d absAccelerationCovariance{}; // 世界坐标系下障碍物加速度协方差矩阵
        HafTime timeCreation{};            // 障碍物被识别的时间戳
        HafTime lastUpdatedTime{};         // 障碍物最近更新时间
        uint8_t cipvFlag{};                // 障碍物状态标志: 0-CIPV 1-CIPS 2-LPIHP 3-RPIHP 4-NONE
        uint8_t blinkerFlag{};             // 障碍物车辆信号灯状态：
                                        // 0-OFF 1-LEFT_TURN_VISIBLE 2-LEFT_TURN_ON 3-RIGHT_TURN_VISIBLE
                                        // 4-RIGHT_TURN_ON 5-BRAKE_VISIBLE 6-BRAKE_ON 7-UNKNOWN
        uint8_t fusionType{};              // 融合障碍物类型: 0-CAMERA 1-RADAR 2-LIDAR 3-FUSED 4-UNKNOWN
        Point3d anchorPoint{};             // 车身坐标系障碍物锚点
        Point3d absAnchorPoint{};          // 世界坐标系障碍物锚点
        std::vector<Point3f> pathPoints; // 轨迹点
    };
    using HafFusionOutI = HafFusionOut<int32_t>;
    using HafFusionOutF = HafFusionOut<float32_t>;
    using HafFusionOutD = HafFusionOut<float64_t>;
    /* ******************************************************************************
        结构 名		:  HafFusionOut
        功能描述		:  融合后障碍物队列结果
    ****************************************************************************** */
    template<typename T> struct HafFusionOutArray {
        HafTime stamp{};
        int32_t count{};
        std::list<HafFusionOut<T>> fusionOut;
        std::string frameID;
        uint32_t seq{};
    };
    using HafFusionOutArrayI = HafFusionOutArray<int32_t>;
    using HafFusionOutArrayF = HafFusionOutArray<float32_t>;
    using HafFusionOutArrayD = HafFusionOutArray<float64_t>;

    /* ******************************************************************************
        结构 名             :  HaftlDetectionOut
        功能描述            :  红绿灯检测结果
    ****************************************************************************** */
    template<typename T>
    struct HafTlDetectionOut {
        int32_t objectId{};               // 红绿灯id
        int32_t color{};                    // 红绿灯类别
        int32_t type{};                  // 红绿灯类别
        float64_t confidence{};           // 红绿灯置信度
        HafRect3D<T> rect{};             // 红绿灯边框
        Point3d position{};              // 红绿灯位置
        float64_t distance{};            // 红绿灯停止线距离
        std::list<int32_t> tlLanes;    // 红绿灯对应的车道
        bool flash{false};                    // 红绿灯是否闪烁
        float64_t timeTrack{};           // 红绿灯已经跟踪的时间
        float64_t timeFlash{};           // 红绿灯已经跟踪的闪烁的时间
        float64_t timeV2X{};             // 红绿灯V2X时间
        HafTime timeCreation{};          // 红绿灯检测结果创建时间
    };

    /* ******************************************************************************
        结构 名             :  HaftlDetectionOutF
        功能描述            :  红绿灯检测结果列表
    ****************************************************************************** */
    template<typename T> struct HafTlDetectionOutArray {
        HafTime stamp{};
        int32_t count{};
        std::list<HafTlDetectionOut<T>> tlDetectionOut;
        std::string frameID;
        uint32_t seq{};
    };
}
#endif
