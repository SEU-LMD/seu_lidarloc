/**
 * FUNCTION: Define Lane Struct
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 **/
#ifndef HAF_OBJECT_LANES_H
#define HAF_OBJECT_LANES_H
#include <vector>
#include <list>
#include <iostream>
#include "core/types.h"
#include "core/timer.h"
namespace Adsfi {
    /* *
    *  三次函数系数: x = a * y^3 + b * y^2 + c * y + d;
    * */
    struct HafLaneParam {
        float64_t a = 0.0;
        float64_t b = 0.0;
        float64_t c = 0.0;
        float64_t d = 0.0;
    };


    /*******************************************************************************
        结构 名		:  HafLaneDetectionOut
        功能描述		:  车道线检测结果
    *******************************************************************************/
    struct HafLaneDetectionOut {
        int32_t laneID;                   // 车道线id
        int32_t cls;                      // 车道线类别
        float32_t confidence;             // 车道线置信度
        enum Coordinate laneCoordinate;   // 坐标系类别
        HafLaneParam laneFit;         // 车道线拟合函数系数
        HafMatrix3d homographyMat;    // 透视变换矩阵
        HafMatrix3d homographyMatInv; // 透视变换逆矩阵
        Point2i startPoint;           // 车道线上顶点
        Point2i endPoint;             // 车道线下顶点
        std::list<Point2i> keyPoint;  // 车道线关键点
        HafTime timeCreation;         // 车道线被识别的时间戳
    };


    /*******************************************************************************
        结构 名		:  HafLaneDetectionOutF
        功能描述		:  车道线列表检测结果
    *******************************************************************************/
    struct HafLaneDetectionOutArray {
        HafTime lanesTimeStamp;
        int32_t count;
        std::list<HafLaneDetectionOut> laneDetectionOut;
        std::string frameID;                               // 该帧数据的传感器源
        uint32_t seq;
    };
}
#endif  //  HAF_OBJECT_LANES_H__
