/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Publisher
 */


#ifndef VIZ_LIB_PUBLISHER_H
#define VIZ_LIB_PUBLISHER_H

#include <typeinfo>
#include <cstdint>
#include "ara/core/string.h"
#include "display_types/viz_point_cloud2.h"
#include "display_types/viz_image.h"
#include "display_types/viz_marker.h"
#include "display_types/viz_marker_array.h"
#include "display_types/viz_path.h"
#include "display_types/viz_point.h"
#include "display_types/viz_pose.h"
#include "display_types/viz_tf.h"
#include "display_types/viz_polygon.h"
#include "display_types/viz_map.h"
#include "display_types/viz_navigation_status.h"
#include "display_types/viz_vehicle_status.h"
#include "display_types/viz_traffic_info.h"
#include "display_types/viz_object_array.h"
#include "display_types/viz_grid_cells.h"
#include "display_types/viz_range.h"
#include "display_types/viz_rectangle.h"
#include "display_types/viz_image_rectangle_array.h"
#include "display_types/viz_image_line_array.h"
#include "display_types/viz_key_object.h"
#include "display_types/viz_location_status.h"
#include "display_types/viz_road_info.h"
#include "display_types/viz_coords_offset.h"
#include "display_types/viz_plot_data.h"
#include "display_types/viz_st_space.h"
#include "display_types/viz_slt_space.h"
#include "display_types/viz_radar_detect_array.h"
#include "display_types/viz_radar_track_array.h"

namespace mdc {
namespace visual {
/*
 * 功能：开启viz库功能，尝试连接客户端
 * 返回：true：操作成功 false:操作失败
 * 注意：会读取viz_address.conf中的客户端地址，仅需调用一次,当链接无法建立时，会持续尝试连接3s
 */
bool Connect();

/*
 * 功能：关闭viz-lib库功能，断开跟客户端的连接
 * 返回：true：操作成功 false:操作失败
 * 注意：如果成功会释放内存占用的内存
 */
bool Close();

class Publisher {
public:
    /*
     * 功能：消息发布构造函数
     * 入参：需要发布的topic
     * 返回：发布消息的对象
     */
    Publisher() : m_messageCode(0U), m_topic() {}
    ~Publisher() = default;
    template<typename T> static Publisher Advertise(const ara::core::String topic)
    {
        Publisher pub(typeid(T).hash_code(), topic);
        return pub;
    }

    /*
     * 功能：发布结构体消息
     * 入参：需要发布消息的结构体
     * 返回：true：操作成功 false:操作失败
     * 注意：特定结构体模板化的Advertise只能发送特定结构体
     */
    bool Publish(const PointCloud2 &vizPointCloud2) const;
    bool Publish(const Marker &vizMarker) const;
    bool Publish(const MarkerArray &vizMarkerArray) const;
    bool Publish(const Image &iImage) const;
    bool Publish(const Path &vizPath) const;

    bool Publish(const PoseStamped &vizPoseStamped) const;
    bool Publish(const PolygonStamped &vizPolygonStamped) const;
    bool Publish(const PointStamped &vizPointStamped) const;
    bool Publish(const Tf &vizTf) const;

    bool Publish(const OccupancyGrid &vizOccupancyGrid) const;
    bool Publish(const VehicleStatus &status) const;
    bool Publish(const NavigationStatus &status) const;
    bool Publish(const TrafficInfo &info) const;
    bool Publish(const ObjectArray &vizObjects) const;

    bool Publish(const GridCells &cells) const;
    bool Publish(const Range &vizRange) const;

    bool Publish(const ImageLineArray &images) const;
    bool Publish(const ImageRectangleArray &images) const;

    bool Publish(const KeyObject &vizObject) const;
    bool Publish(const LocationStatus &status) const;
    bool Publish(const RoadInfo &info) const;

    bool Publish(const CoordsOffset &offset) const;
    bool Publish(const PlotData &vizPlotData) const;
    bool Publish(const StSpace &vizStSpace) const;
    bool Publish(const SltSpace &vizSltSpace) const;

    bool Publish(const RadarDetectArray &vizDetects) const;
    bool Publish(const RadarTrackArray &vizTracks) const;

    /*
     * 功能：发布PointCloud<PointT> 对象, 类似于ROS PCL 发布接口功能
     * 入参：PointT 发布的点云类型
     * 返回：true：发送成功 false:操作失败
     * 注意：支持 PointXYZ, PointXYZI , PointXYZIR, PointXYZRGB, 详细数据结构参考文件定义： viz_point_cloud2.h
     */
    template<typename PointT> bool Publish(const PointCloud<PointT> &vizPointCloud) const
    {
        PointCloud2 stdPointCloud;
        const auto ret = ConvertToStdPointCloud(vizPointCloud, stdPointCloud);
        if (ret != RetCode::VIZ_OK) {
            return false;
        }
        return Publish(stdPointCloud);
    }
    /*
     * 功能：发布自定义消息
     * 入参：需要发布消息地址，及其长度
     * 返回：true：操作成功 false:操作失败
     * 注意：特定结构体模板化的Advertise只能发送特定结构体
     */
    bool Publish(uint8_t const pubData[], const uint32_t len) const;

private:
    Publisher(const std::size_t hash_code, const ara::core::String topic) : m_messageCode(hash_code), m_topic(topic) {}
    bool IsExistActiveLink() const;
    std::size_t m_messageCode;
    /*
     * 对应消息数据类型，对topic名约束如下：
     * 1.KeyObjects
     *    TopicKeyObjects
     * 2.LocationStatus
     *    TopicLocationStatus
     * 3.NavigationStatus
     *    TopicNavigationStatus
     * 4.ObjectArray
     *    TopicObjectArray
     * 5.RoadInfo
     *    TopicRoadInfo
     * 6.RoadLineImage
     *    TopicRoadLine2Ds
     * 7.TrafficInfo
     *    TopicTrafficInfo
     * 8.TrafficLightImage
     *    TopicTraficLight2Ds
     * 9.VehicleStatus （两个不同数据，数据结构相同）
     *    底盘状态：TopicVehicleInfo
     *    控制信息：TopicVehicleControl
     * 10.终点下发：
     *    TopicEndPoint
     *    途经点下发：
     *    TopicWayPoint
     *    地图刷新
     *    TopicRefreshMap
     */
    ara::core::String m_topic;
    bool trueValid_ = true;
};
template bool Publisher::Publish(const PointCloud<PointXYZ> &vizPointCloud) const;
template bool Publisher::Publish(const PointCloud<PointXYZI> &vizPointCloud) const;
template bool Publisher::Publish(const PointCloud<PointXYZIR> &vizPointCloud) const;
template bool Publisher::Publish(const PointCloud<PointXYZRGB> &vizPointCloud) const;
} // namespace visual
} // namespace ara

#endif // VIZ_LIB_PUBLISHER_H
