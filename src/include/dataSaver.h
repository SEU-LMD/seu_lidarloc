
#ifndef FAST_LIO_SRC_PGO_SRC_DATASAVER_H_
#define FAST_LIO_SRC_PGO_SRC_DATASAVER_H_

#include <geometry_msgs/TransformStamped.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/dataset.h>
//#include <libxml/parser.h>
//#include <libxml/tree.h>
//#include <libxml/xmlmemory.h>
//#include <libxml/xmlstring.h>
//#include <libxml/xpath.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#include <fstream>
#include <iostream>
#include <pcl/search/impl/search.hpp>
#include "config_helper.h"
//#include <glog/logging.h>

using namespace std;
using namespace gtsam;

using PointT = pcl::PointXYZI;

class DataSaver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DataSaver(){}

  ~DataSaver(){}


  void saveOriginGPS(Eigen::Vector3d gps_point){
      std::fstream originStream( MappingConfig::save_map_path +"origin.txt", std::fstream::out);
      originStream.precision(15);
      originStream << gps_point[0] << " " << gps_point[1] << " " << gps_point[2]
                   << std::endl;
      originStream.close();
  }


  void saveOptimizedVerticesTUM(gtsam::Values _estimates) {
      std::fstream stream(MappingConfig::save_map_path + "Pose_TUM.txt",
                          std::fstream::out);
      stream.precision(15);
      for (int i = 0; i < _estimates.size(); i++) {
          auto &pose = _estimates.at(i).cast<gtsam::Pose3>();
          gtsam::Point3 p = pose.translation();
          gtsam::Quaternion q = pose.rotation().toQuaternion();
          stream << i << " " << p.x() << " " << p.y() << " "
                 << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
                 << q.w() << std::endl;
      }
  }
};

#endif  // FAST_LIO_SRC_PGO_SRC_DATASAVER_H_
