#include "cloud_info.h"
#include "utility.h"
#include "timer.h"

#include "config_helper.h"
#include "easylogging++.h"

#include <iostream>
#include <pcl/io/pcd_io.h>       //PCD读写类相关的头文件
#include <pcl/io/ply_io.h>
#include "MapSaver.h"

INITIALIZE_EASYLOGGINGPP
struct smoothness_t {
  float value;
  size_t ind;
};

struct by_value {
  bool operator()(smoothness_t const &left, smoothness_t const &right) {
    return left.value < right.value;
  }
};

class FeatureExtraction {

 public:
  MapSaver map_saver;
  int frame_id = 0;
  ros::NodeHandle nh;
  ros::Subscriber subLaserCloudInfo;

  ros::Publisher pubLaserCloudInfo;
  ros::Publisher pubCornerPoints;
  ros::Publisher pubSurfacePoints;

  pcl::PointCloud<PointType>::Ptr extractedCloud;
  pcl::PointCloud<PointType>::Ptr cornerCloud;
  pcl::PointCloud<PointType>::Ptr surfaceCloud;

  pcl::VoxelGrid<PointType> downSizeFilter;

  lio_sam_6axis::cloud_info cloudInfo;
  std_msgs::Header cloudHeader;

  std::vector<smoothness_t> cloudSmoothness;
  float *cloudCurvature;    // for calculate curvature
  int *cloudNeighborPicked; // 1:after process; 0:before process
  int *cloudLabel;          //1:corner; 0:surface

  FeatureExtraction() {
    subLaserCloudInfo = nh.subscribe<lio_sam_6axis::cloud_info>(
        "lio_sam_6axis/deskew/cloud_info", 1,
        &FeatureExtraction::laserCloudInfoHandler, this,
        ros::TransportHints().tcpNoDelay());

    pubLaserCloudInfo = nh.advertise<lio_sam_6axis::cloud_info>(
        "lio_sam_6axis/feature/cloud_info", 1);
    pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>(
        "lio_sam_6axis/feature/cloud_corner", 1);
    pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>(
        "lio_sam_6axis/feature/cloud_surface", 1);

    initializationValue();

  }

  void initializationValue() {
    cloudSmoothness.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);

    downSizeFilter.setLeafSize(MappingConfig::odometrySurfLeafSize, MappingConfig::odometrySurfLeafSize,
                               MappingConfig::odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    cloudCurvature = new float[SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN];
    cloudNeighborPicked = new int[SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN];
    cloudLabel = new int[SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN];

  }

  void laserCloudInfoHandler(const lio_sam_6axis::cloud_infoConstPtr &msgIn) {

      TicToc timer;

    EZLOG(INFO)<<"***************laserCloudInfoHandler "<<std::endl;

    cloudInfo = *msgIn;           // new cloud info
    cloudHeader = msgIn->header;  // new cloud header
    pcl::fromROSMsg(msgIn->cloud_deskewed,
                    *extractedCloud);  // new cloud for extraction

      //1.calculate point curvature and store in cloudSmoothness
      calculateSmoothness();

      //2.mark point which does not extract features
      markOccludedPoints();

      //3.extract corner and surface point
      extractFeatures();
//      double time_featureextraction = timer.toc();
//      EZLOG(INFO)<<"***************time_featureextraction "<<time_featureextraction<<std::endl;
      //4.publish cloud
      publishFeatureCloud();

      double time_featureextraction = timer.toc();
      EZLOG(INFO)<<"***************time_featureextraction "<<time_featureextraction<<std::endl;

      //save
      CloudInfo cloudinfo;
      cloudinfo.frame_id = frame_id;
      frame_id++;
      cloudinfo.corner_cloud = cornerCloud;
      cloudinfo.surf_cloud = surfaceCloud;
      map_saver.AddCloudToSave(cloudinfo);
  }

  void calculateSmoothness() {

    int cloudSize = extractedCloud->points.size();

    for (int i = 5; i < cloudSize - 5; i++) {
      //use front 5points and rear 5points to calculate curvature
      float diffRange =
          cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4] +
          cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2] +
          cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10 +
          cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2] +
          cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4] +
          cloudInfo.pointRange[i + 5];

      //calculate curvature
      cloudCurvature[i] = diffRange * diffRange;  // diffX * diffX + diffY * diffY + diffZ * diffZ;

      //0:before feature extraction; 1 after feature extraction or obscured or parallel
      cloudNeighborPicked[i] = 0;
      //-1: surface point; 1:corner point
      cloudLabel[i] = 0;

      // cloudSmoothness for
      cloudSmoothness[i].value = cloudCurvature[i];
      cloudSmoothness[i].ind = i;
    }

  }

  void markOccludedPoints() {

    int cloudSize = extractedCloud->points.size();

    EZLOG(INFO)<<"***************cloudSize = "<<cloudSize<<std::endl;

//    //use for debug
//    EZLOG(INFO)<<"cloudInfo.T_w_l_curlidar.pose.pose.position.x = "<<cloudInfo.T_w_l_curlidar.pose.pose.position.x<<std::endl;
//    EZLOG(INFO)<<"cloudInfo.T_w_l_curlidar.pose.pose.position.y = "<<cloudInfo.T_w_l_curlidar.pose.pose.position.y<<std::endl;
//    EZLOG(INFO)<<"cloudInfo.T_w_l_curlidar.pose.pose.position.z = "<<cloudInfo.T_w_l_curlidar.pose.pose.position.z<<std::endl;
//    EZLOG(INFO)<<"cloudInfo.T_w_l_curlidar.pose.pose.orientation.x = "<<cloudInfo.T_w_l_curlidar.pose.pose.orientation.x<<std::endl;
//    EZLOG(INFO)<<"cloudInfo.T_w_l_curlidar.pose.pose.orientation.y = "<<cloudInfo.T_w_l_curlidar.pose.pose.orientation.y<<std::endl;
//    EZLOG(INFO)<<"cloudInfo.T_w_l_curlidar.pose.pose.orientation.z = "<<cloudInfo.T_w_l_curlidar.pose.pose.orientation.z<<std::endl;
//    EZLOG(INFO)<<"cloudInfo.T_w_l_curlidar.pose.pose.orientation.w = "<<cloudInfo.T_w_l_curlidar.pose.pose.orientation.w<<std::endl;
//    EZLOG(INFO)<<"cloudInfo.T_w_l_curlidar.header.stamp.sec = "<<cloudInfo.T_w_l_curlidar.header.stamp.sec<<std::endl;
//    EZLOG(INFO)<<"cloudInfo.T_w_l_curlidar.header.seq = "<<cloudInfo.T_w_l_curlidar.header.seq<<std::endl;


    // mark occluded points and parallel beam points
    //find current and next point's range value
    for (int i = 5; i < cloudSize - 6; ++i) {
      // occluded points
      float depth1 = cloudInfo.pointRange[i];
      float depth2 = cloudInfo.pointRange[i + 1];

      int columnDiff = std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

      if (columnDiff < 10) {
        // 10 pixel diff in range image
        //if distance > 0.3 ,occluded point and mark neighbor point 1(don't extract feature)
        if (depth1 - depth2 > 0.3) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        } else if (depth2 - depth1 > 0.3) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
      // parallel beam
      float diff1 = std::abs(
          float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
      float diff2 = std::abs(
          float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));

      //if far from adjacent point ,bad point
      if (diff1 > 0.02 * cloudInfo.pointRange[i] &&
          diff2 > 0.02 * cloudInfo.pointRange[i])
        cloudNeighborPicked[i] = 1;
    }
  }

  void extractFeatures() {
    cornerCloud->clear();
    surfaceCloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < SensorConfig::N_SCAN; i++) {
      surfaceCloudScan->clear();

      for (int j = 0; j < 6; j++) {

        //divide into 6 parts
        int sp = (cloudInfo.startRingIndex[i] * (6 - j) +
                  cloudInfo.endRingIndex[i] * j) /6;
        int ep = (cloudInfo.startRingIndex[i] * (5 - j) +
                  cloudInfo.endRingIndex[i] * (j + 1)) /6 - 1;

        if (sp >= ep) continue;

        //sort point by comparing curvature(small - large)
        std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep,
                  by_value());
        //extract corner point
        int largestPickedNum = 0;

        //traverse by curvature, from small to large
        for (int k = ep; k >= sp; k--) {
          //point index
          int ind = cloudSmoothness[k].ind;
          //if point
          if (cloudNeighborPicked[ind] == 0 &&
              cloudCurvature[ind] > MappingConfig::edgeThreshold) {
            largestPickedNum++;
            if (largestPickedNum <= 20) {
              cloudLabel[ind] = 1;
              cornerCloud->push_back(extractedCloud->points[ind]);
            } else {
              break;
            }

            cloudNeighborPicked[ind] = 1;

            for (int l = 1; l <= 5; l++) {
              int columnDiff =
                  std::abs(int(cloudInfo.pointColInd[ind + l] -
                               cloudInfo.pointColInd[ind + l - 1]));
              if (columnDiff > 10) break;
              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--) {
              int columnDiff =
                  std::abs(int(cloudInfo.pointColInd[ind + l] -
                               cloudInfo.pointColInd[ind + l + 1]));
              if (columnDiff > 10) break;
              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }// end! traverse by curvature, from small to large
        //extract surface point
        for (int k = sp; k <= ep; k++) {
          int ind = cloudSmoothness[k].ind;
          if (cloudNeighborPicked[ind] == 0 &&
              cloudCurvature[ind] < MappingConfig::surfThreshold) {
            cloudLabel[ind] = -1;
            cloudNeighborPicked[ind] = 1;

            for (int l = 1; l <= 5; l++) {
              int columnDiff =
                  std::abs(int(cloudInfo.pointColInd[ind + l] -
                               cloudInfo.pointColInd[ind + l - 1]));
              if (columnDiff > 10) break;

              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--) {
              int columnDiff =
                  std::abs(int(cloudInfo.pointColInd[ind + l] -
                               cloudInfo.pointColInd[ind + l + 1]));
              if (columnDiff > 10) break;

              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        // surface point and point doesn't be processed ,regard as surface
        for (int k = sp; k <= ep; k++) {
          if (cloudLabel[k] <= 0) {
            surfaceCloudScan->push_back(extractedCloud->points[k]);
          }
        }
      }

      surfaceCloudScanDS->clear();
      downSizeFilter.setInputCloud(surfaceCloudScan);
      downSizeFilter.filter(*surfaceCloudScanDS);

      *surfaceCloud += *surfaceCloudScanDS;
    }
  }

  void freeCloudInfoMemory() {
    cloudInfo.startRingIndex.clear();
    cloudInfo.endRingIndex.clear();
    cloudInfo.pointColInd.clear();
    cloudInfo.pointRange.clear();
  }

  void publishFeatureCloud() {

    EZLOG(INFO)<<"***************publishFeatureCloud "<<std::endl;


      // free cloud info memory
    freeCloudInfoMemory();
    // save newly extracted features
//    cloudInfo.cloud_corner = publishCloud(pubCornerPoints, cornerCloud,
//                                          cloudHeader.stamp, SensorConfig::lidarFrame);
      cloudInfo.cloud_corner = publishCloud(pubCornerPoints, cornerCloud,
                                            cloudHeader.stamp, "map");
      cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud,
                                             cloudHeader.stamp, "map");
      EZLOG(INFO)<<"***************cornerCloud->size() "<<cornerCloud->size()<<std::endl;
      EZLOG(INFO)<<"***************surfaceCloud->size() "<<surfaceCloud->size()<<std::endl;
    // publish to mapOptimization
    pubLaserCloudInfo.publish(cloudInfo);

//    static long long int idx = 0;
//      //for deubug use (output pcd/ply)
//        {
//            pcl::PointCloud<pcl::PointXYZ> temp ;
//            for(int i = 0; i < cornerCloud -> points.size(); ++i){
//                pcl::PointXYZ p;
//                p.x = cornerCloud -> points[i].x;
//                p.y = cornerCloud -> points[i].y;
//                p.z = cornerCloud -> points[i].z;
//                temp.push_back(p);
//            }
//            std::string filename = "/home/lsy/point/cornerCloud"+ to_string( idx )+".ply";
//            pcl::io::savePLYFile(filename, temp);
////            ++idx;
//        }
//
//      {
//          pcl::PointCloud<pcl::PointXYZ> temp ;
//          for(int i = 0; i < surfaceCloud -> points.size(); ++i){
//              pcl::PointXYZ p;
//              p.x = surfaceCloud -> points[i].x;
//              p.y = surfaceCloud -> points[i].y;
//              p.z = surfaceCloud -> points[i].z;
//              temp.push_back(p);
//          }
//          std::string filename2 = "/home/lsy/point/surfaceCloud"+ to_string( idx )+".ply";
//          pcl::io::savePLYFile(filename2, temp);
//          ++idx;
//      }
//      //end debug use


  }
};

int main(int argc, char **argv) {
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%level %file %line : %msg");
#ifdef ELPP_THREAD_SAFE
    EZLOG(INFO) << "easylogging++ thread safe!";
#else
    EZLOG(INFO) << "easylogging++ thread unsafe";
#endif
    Load_Sensor_YAML("./config/sensor.yaml");
    Load_Mapping_YAML("./config/mapping.yaml");

  ros::init(argc, argv, "ft_ext");

  FeatureExtraction FE;
  std::thread saveMapThread(&MapSaver::do_work, &(FE.map_saver));//comment fyy
  ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

  ros::spin();

  return 0;
}