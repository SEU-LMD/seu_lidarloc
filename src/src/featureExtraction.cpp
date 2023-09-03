#include "cloud_info.h"
#include "utility.h"
#include "timer.h"

#include "config_helper.h"
#include "easylogging++.h"

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
  float *cloudCurvature;
  int *cloudNeighborPicked;
  int *cloudLabel;

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
    cloudInfo = *msgIn;           // new cloud info
    cloudHeader = msgIn->header;  // new cloud header
    pcl::fromROSMsg(msgIn->cloud_deskewed,
                    *extractedCloud);  // new cloud for extraction

    calculateSmoothness();

    markOccludedPoints();

    extractFeatures();

    publishFeatureCloud();
  }

  void calculateSmoothness() {
      int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++) {
      float diffRange =
          cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4] +
          cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2] +
          cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10 +
          cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2] +
          cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4] +
          cloudInfo.pointRange[i + 5];

      cloudCurvature[i] =  diffRange * diffRange;  // diffX * diffX + diffY * diffY + diffZ * diffZ;


      cloudNeighborPicked[i] = 0;//初始化所有的点都没有被标记过
      cloudLabel[i] = 0;
      // cloudSmoothness for sorting
      cloudSmoothness[i].value = cloudCurvature[i];
      cloudSmoothness[i].ind = i;
    }
  }

  void markOccludedPoints() {
    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i) {
      // occluded points
      float depth1 = cloudInfo.pointRange[i];
      float depth2 = cloudInfo.pointRange[i + 1];
      int columnDiff = std::abs(
          int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

      if (columnDiff < 10) {
        // 10 pixel diff in range image
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

      if (diff1 > 0.02 * cloudInfo.pointRange[i] &&
          diff2 > 0.02 * cloudInfo.pointRange[i])
        cloudNeighborPicked[i] = 1;
    }
  }

  void extractFeatures() {


    cornerCloud->clear();
    surfaceCloud->clear();


    for (int i = 0; i < SensorConfig::N_SCAN; i++) {

      pcl::PointCloud<PointType>::Ptr surfaceCloudScan(  new pcl::PointCloud<PointType>());

      //给一个scan分成6个不同的扇区
      for (int j = 0; j < 6; j++) {
            int sp = ( cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j ) / 6;
            int ep = ( cloudInfo.startRingIndex[i] * (5 - j) +  cloudInfo.endRingIndex[i] * (j + 1) ) / 6 - 1;

            if (sp >= ep) continue;

            std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());
            //提取角点×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
            int largestPickedNum = 0;//只在提取角点时使用
            for (int k = ep; k >= sp; k--) {
                  int ind = cloudSmoothness[k].ind;
                  if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > MappingConfig::edgeThreshold) {
                        largestPickedNum++;
                        //每个扇区最多只能提取出20个角点
                        if (largestPickedNum <= 20) {
                          cloudLabel[ind] = 1;//标记这个点不是面点 是角点
                          cornerCloud->push_back(extractedCloud->points[ind]);//very important movement!!!!!!!!!!!!!11
                        } else {
                          break;
                        }
                        cloudNeighborPicked[ind] = 1;
                        //周围的几个点如果距离不远，也被标记为被选中，即不再参与角点的计算
                        for (int l = 1; l <= 5; l++) {
                          int columnDiff =  std::abs(int(cloudInfo.pointColInd[ind + l] -  cloudInfo.pointColInd[ind + l - 1]));
                          if (columnDiff > 10) break;
                          cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                          int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] -  cloudInfo.pointColInd[ind + l + 1]));
                          if (columnDiff > 10) break;
                          cloudNeighborPicked[ind + l] = 1;
                        }
                  }
            }
            //提取面点×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
            for (int k = sp; k <= ep; k++) {
                  int ind = cloudSmoothness[k].ind;
                  if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < MappingConfig::surfThreshold) {
                        cloudLabel[ind] = -1;//标记这个点是面点
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {
                          int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                          if (columnDiff > 10) break;
                          cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                          int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                          if (columnDiff > 10) break;
                          cloudNeighborPicked[ind + l] = 1;
                        }
                  }
            }

            for (int k = sp; k <= ep; k++) {
              if (cloudLabel[k] <= 0) {
                surfaceCloudScan->push_back(extractedCloud->points[k]);
              }
            }
      }//for (int j = 0; j < 6; j++)

      //对属于同一个scan的面点进行降采样
      pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS( new pcl::PointCloud<PointType>());
      downSizeFilter.setInputCloud(surfaceCloudScan);
      downSizeFilter.filter(*surfaceCloudScanDS);

      *surfaceCloud += *surfaceCloudScanDS;//very important function!!!!!!!
    }//end function     for (int i = 0; i < SensorConfig::N_SCAN; i++) {

  }//end fucntion   void extractFeatures() {


    void freeCloudInfoMemory() {
    cloudInfo.startRingIndex.clear();
    cloudInfo.endRingIndex.clear();
    cloudInfo.pointColInd.clear();
    cloudInfo.pointRange.clear();
  }

  void publishFeatureCloud() {
    // free cloud info memory
    freeCloudInfoMemory();
    // save newly extracted features
    cloudInfo.cloud_corner = publishCloud(pubCornerPoints, cornerCloud,
                                          cloudHeader.stamp, SensorConfig::lidarFrame);
    cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud,
                                           cloudHeader.stamp, SensorConfig::lidarFrame);
    // publish to mapOptimization
    pubLaserCloudInfo.publish(cloudInfo);
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

  ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

  ros::spin();

  return 0;
}