
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
//#include "config_helper.h"
//#include <glog/logging.h>

using namespace std;
using namespace gtsam;

using PointT = pcl::PointXYZI;

class DataSaver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DataSaver(){}

  ~DataSaver(){}

  DataSaver(string _base_dir, string _sequence_name){
      this->base_dir = _base_dir;
      this->sequence_name = _sequence_name;

      if (_base_dir.back() != '/') {
          _base_dir.append("/");
      }
      save_directory = _base_dir + sequence_name + '/';
      std::cout << "SAVE DIR:" << save_directory << std::endl;

      auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
      unused = system((std::string("mkdir -p ") + save_directory).c_str());
  }

  void setDir(string _base_dir, string _sequence_name){
      this->base_dir = _base_dir;
      this->sequence_name = _sequence_name;

      if (_base_dir.back() != '/') {
          _base_dir.append("/");
      }
      save_directory = _base_dir + sequence_name + '/';

      auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
      unused = system((std::string("mkdir -p ") + save_directory).c_str());
  }

  void setConfigDir(string _config_dir){
      if (_config_dir.back() != '/') {
          _config_dir.append("/");
      }
      this->config_directory = _config_dir;
  }

  void setExtrinc(bool _use_imu, Eigen::Vector3d _t_body_sensor,
                  Eigen::Quaterniond _q_body_sensor){
      this->use_imu_frame = _use_imu;
      this->t_body_sensor = _t_body_sensor;
      this->q_body_sensor = _q_body_sensor;
  }

  //void saveOptimizedVerticesKITTI(gtsam::Values _estimates);

  //void saveOdometryVerticesKITTI(std::string _filename);

  void saveOriginGPS(Eigen::Vector3d gps_point){
      std::fstream originStream( save_directory+"origin.txt", std::fstream::out);
      originStream.precision(15);
      originStream << gps_point[0] << " " << gps_point[1] << " " << gps_point[2]
                   << std::endl;
      originStream.close();
  }

//  void saveTimes(vector<double> keyframeTimes){
//      if (_keyframeTimes.empty()) {
//          //    LOG(ERROR) << "EMPTY KEYFRAME TIMES!";
//          return;
//      }
//      this->keyframeTimes = _keyframeTimes;
//      std::fstream pgTimeSaveStream(save_directory + "times.txt",
//                                    std::fstream::out);
//      pgTimeSaveStream.precision(15);
//      // save timestamp
//      for (auto const timestamp : keyframeTimes) {
//          pgTimeSaveStream << timestamp << std::endl;
//      }
//      pgTimeSaveStream.close();
//  }

  void saveOptimizedVerticesTUM(gtsam::Values _estimates){
      std::fstream stream(save_directory + "optimized_odom_tum.txt",
                          std::fstream::out);
      stream.precision(15);
      for (int i = 0; i < _estimates.size(); i++) {
          auto &pose = _estimates.at(i).cast<gtsam::Pose3>();
          gtsam::Point3 p = pose.translation();
          gtsam::Quaternion q = pose.rotation().toQuaternion();
          stream << keyframeTimes.at(i) << " " << p.x() << " " << p.y() << " "
                 << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
                 << q.w() << std::endl;
      }

  }

  void saveOdometryVerticesTUM(std::vector<nav_msgs::Odometry> keyframePosesOdom)
  {
      std::fstream stream(save_directory + "odom_tum.txt", std::fstream::out);
      stream.precision(15);
      for (int i = 0; i < keyframePosesOdom.size(); i++) {
          nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
          double time = odometry.header.stamp.toSec();
          // check the size of keyframeTimes
          stream << time << " " << odometry.pose.pose.position.x << " "
                 << odometry.pose.pose.position.y << " "
                 << odometry.pose.pose.position.z << " "
                 << odometry.pose.pose.orientation.x << " "
                 << odometry.pose.pose.orientation.y << " "
                 << odometry.pose.pose.orientation.z << " "
                 << odometry.pose.pose.orientation.w << std::endl;
      }
  }

// // void saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph,
//                      gtsam::ISAM2 *isam, gtsam::Values isamCurrentEstimate);

  //void saveGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom);

//  void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
//                     std::vector<sensor_msgs::PointCloud2> allResVec);

//  void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
//                     std::vector<sensor_msgs::PointCloud2> allResVec,
//                     std::vector<geometry_msgs::TransformStamped> trans_vec);

//  void saveLoopandImagePair(
//      std::map<int, int> loopIndexCheckedMap,
//      std::vector<std::vector<int>> all_camera_corre_match_pair);

//  void savePointCloudMap(std::vector<nav_msgs::Odometry> allOdometryVec,
//                         std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

  //void savePointCloudMap(pcl::PointCloud<PointT> &allResVec);

//  int readParameter(){
//
//  }

//  int saveKMLTrajectory(const std::vector<Eigen::Vector3d> lla_vec){
//      if (1 == readParameter()) {
//          return 1;
//  }

 private:
  string base_dir, sequence_name;
  string save_directory, config_directory;

  vector<string> configParameter;

  bool use_imu_frame = false;
  Eigen::Quaterniond q_body_sensor;
  Eigen::Vector3d t_body_sensor;

  vector<double> keyframeTimes;
};

#endif  // FAST_LIO_SRC_PGO_SRC_DATASAVER_H_
