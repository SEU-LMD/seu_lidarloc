//
// Created by xiang on 2022/1/4.
//

#ifndef SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H
#define SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H

#include "gnss.h"
#include "utm.h"





namespace sad {
  
constexpr double PI = 3.14159265358979323846;   //π

constexpr double kDEG2RAD = PI / 180.0;
constexpr double kRAD2DEG = 180.0 / PI;

/**
 * 计算本书的GNSS读数对应的UTM pose和六自由度Pose
 * @param gnss_reading  输入gnss读数
 * @param antenna_pos   安装位置
 * @param antenna_angle 安装偏角
 * @param map_origin    地图原点，指定时，将从UTM位置中减掉坐标原点
 * @return
 */

 ///经纬度转UTM
bool LatLon2UTM(const Vec2d& latlon, sad::UTMCoordinate& utm_coor) {
    long zone = 0;
    char char_north = 0;
    long ret = Convert_Geodetic_To_UTM(latlon[0] * kDEG2RAD, latlon[1] * kDEG2RAD, &zone, &char_north,
                                       &utm_coor.xy_[0], &utm_coor.xy_[1]);//是否能用还不知道
    utm_coor.zone_ = (int)zone;
    utm_coor.north_ = char_north == 'N';
    std::cout<<"ret="<<ret<<std::endl;

    return ret == 0;
}
///UTM转换到经纬度。
bool UTM2LatLon(const sad::UTMCoordinate& utm_coor, Vec2d& latlon) {
    bool ret = Convert_UTM_To_Geodetic((long)utm_coor.zone_, utm_coor.north_ ? 'N' : 'S', utm_coor.xy_[0],
                                       utm_coor.xy_[1], &latlon[0], &latlon[1]);
    latlon *= kRAD2DEG;
    return ret == 0;
}


}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H
