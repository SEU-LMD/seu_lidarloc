#include <mutex>
#ifndef SEU_LIDARLOC_FEATUREEXTRACTION_H
#define SEU_LIDARLOC_FEATUREEXTRACTION_H
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "utils/timer.h"
#include "opt_mapping.h"
#include "opt_loc.h"
#include "data_preprocess.h"
#include "utils/MapSaver.h"

#include "pcl/common/pca.h"


struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

class FeatureExtraction  {
public:
    PubSubInterface* pubsub;
    std::mutex cloud_mutex;
    std::deque<CloudInfo> deque_cloud;
    std::thread* do_work_thread;
    std::thread* save_Map_thread;

    std::function<void(const CloudFeature&)> Function_AddCloudFeatureToLOCMapping;
    std::function<void(const CloudFeature&)> Function_AddCloudFeatureToOPTMapping;

    MapSaver map_saver;
    int frame_id = -1;

    std::string topic_corner_world= "/cloud_corner";
    std::string topic_surf_world = "/cloud_surface";

    std::vector<float> cloudCurvature;
    std::vector<int> cloudNeighborPicked;// 1:after process; 0:before process
    std::vector<int> cloudLabel;//1:corner; -1:surface
    std::vector<smoothness_t> cloudSmoothness;
    pcl::VoxelGrid<PointType> downSizeFilter;

    void AllocateMemeory(){
        cloudCurvature.assign(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN,0);
        cloudNeighborPicked.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);
        cloudLabel.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);
        cloudSmoothness.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);
    }

    void CalculateSmoothness(CloudInfo& cur_scan) {

        int cloudSize = cur_scan.cloud_ptr->points.size();

        for (int i = 5; i < cloudSize - 5; i++) {
            //use front 5points and rear 5points to calculate curvature
            float diffRange =
                    cur_scan.cloud_ptr->points[i- 5].range+ cur_scan.cloud_ptr->points[i - 4].range +
                    cur_scan.cloud_ptr->points[i - 3].range + cur_scan.cloud_ptr->points[i - 2].range +
                    cur_scan.cloud_ptr->points[i - 1].range - cur_scan.cloud_ptr->points[i].range * 10 +
                    cur_scan.cloud_ptr->points[i + 1].range + cur_scan.cloud_ptr->points[i + 2].range +
                    cur_scan.cloud_ptr->points[i + 3].range + cur_scan.cloud_ptr->points[i + 4].range +
                    cur_scan.cloud_ptr->points[i + 5].range;

            //calculate curvature
            cloudCurvature[i] = diffRange * diffRange;  // diffX * diffX + diffY * diffY + diffZ * diffZ;

            //0:before feature extraction; 1 after feature extraction or obscured or parallel
            cloudNeighborPicked[i] = 0;
            //-1: surface point; 1:corner point
            cloudLabel[i] = 0;
            cur_scan.cloud_ptr->points[i].label = 0;

            // cloudSmoothness for
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }    //end fucntion calculateSmoothness

    void MarkOccludedPoints(CloudInfo& cur_scan) {

        int cloudSize = cur_scan.cloud_ptr->points.size();

        // mark occluded points and parallel beam points
        //find current and next point's range value
        for (int i = 5; i < cloudSize - 6; ++i) {
            // occluded points
            float depth1 = cur_scan.cloud_ptr->points[i].range;
            float depth2 = cur_scan.cloud_ptr->points[i + 1].range;

            int columnDiff = std::abs(int(cur_scan.cloud_ptr->points[i + 1].col - cur_scan.cloud_ptr->points[i].col));

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
                    float(cur_scan.cloud_ptr->points[i - 1].range - cur_scan.cloud_ptr->points[i].range));
            float diff2 = std::abs(
                    float(cur_scan.cloud_ptr->points[i + 1].range - cur_scan.cloud_ptr->points[i].range));

            //if far from adjacent point ,bad point
            if (diff1 > 0.02 * cur_scan.cloud_ptr->points[i].range &&
                diff2 > 0.02 * cur_scan.cloud_ptr->points[i].range)
                cloudNeighborPicked[i] = 1;
        }
    }//end function markOccludedPoints


    void ExtractFeatures(CloudInfo& cur_scan,
                         pcl::PointCloud<PointType>::Ptr &cornerCloud,
                         pcl::PointCloud<PointType>::Ptr &surfaceCloud,
                         pcl::PointCloud<PointType>::Ptr &rawCloud) {

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());
      //  EZLOG(INFO)<<" ExtractFeatures:: cur_scan: "<<cur_scan.cloud_ptr->points.size();

        for (int i = 0; i < SensorConfig::N_SCAN; i++) {
            surfaceCloudScan->clear();

            for (int j = 0; j < 6; j++) {

                //divide into 6 parts
                int sp = (cur_scan.startRingIndex[i] * (6 - j) +
                        cur_scan.endRingIndex[i] * j) /6;
                int ep = (cur_scan.startRingIndex[i] * (5 - j) +
                        cur_scan.endRingIndex[i] * (j + 1)) /6 - 1;
//
//                EZLOG(INFO) << "sp = " << sp << std::endl;
//                EZLOG(INFO) << "ep = " << ep << std::endl;


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
                    float edgeThreshold;
                    if(MappingConfig::slam_mode_switch == 1){
                        edgeThreshold = MappingConfig::edgeThreshold;
                    }
                    if(LocConfig::slam_mode_on == 1){
                        edgeThreshold = LocConfig::edgeThreshold;
                    }

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > edgeThreshold) {
                        largestPickedNum++;
                        if (largestPickedNum <= 20) {
                            cloudLabel[ind] = 1;
                            cur_scan.cloud_ptr->points[ind].label = 1;
                            PointType pt_tmp;
                            auto& pt_origin = cur_scan.cloud_ptr->points[ind];
                            pt_tmp.x = pt_origin.x;
                            pt_tmp.y = pt_origin.y;
                            pt_tmp.z = pt_origin.z;
//                            pt_tmp.intensity = pt_origin.intensity;
                            pt_tmp.intensity = 1;
                            cornerCloud->push_back(pt_tmp);
                            rawCloud->push_back(pt_tmp);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {
                            int columnDiff =
                                    std::abs(int(cur_scan.cloud_ptr->points[ind + l].col -
                                                 cur_scan.cloud_ptr->points[ind + l - 1].col));
                            if (columnDiff > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff =
                                    std::abs(int(cur_scan.cloud_ptr->points[ind + l].col -
                                                     cur_scan.cloud_ptr->points[ind + l + 1].col));
                            if (columnDiff > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }// end! traverse by curvature, from small to large
                float surfThreshold;
                if(MappingConfig::slam_mode_switch == 1){
                    surfThreshold = MappingConfig::surfThreshold;
                }
                if(LocConfig::slam_mode_on == 1){
                    surfThreshold = LocConfig::surfThreshold;
                }
                //extract surface point
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < surfThreshold) {
                        cloudLabel[ind] = -1;
                        cur_scan.cloud_ptr->points[ind].label = -1;
                        cloudNeighborPicked[ind] = 1;

                        //add by lsy
                        {
                            PointType pt_tmp;
                            auto &pt_origin = cur_scan.cloud_ptr->points[ind];
                            pt_tmp.x = pt_origin.x;
                            pt_tmp.y = pt_origin.y;
                            pt_tmp.z = pt_origin.z;
//                            pt_tmp.intensity = pt_origin.intensity;
                            pt_tmp.intensity = -1;
                            surfaceCloud->push_back(pt_tmp);
                            rawCloud->push_back(pt_tmp);
                        }

                        for (int l = 1; l <= 5; l++) {
                            int columnDiff =
                                    std::abs(int(cur_scan.cloud_ptr->points[ind + l].col -
                                                 cur_scan.cloud_ptr->points[ind + l - 1].col));
                            if (columnDiff > 10) break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff =
                                    std::abs(int(cur_scan.cloud_ptr->points[ind + l].col -
                                                 cur_scan.cloud_ptr->points[ind + l + 1].col));
                            if (columnDiff > 10) break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

            }

        }
    }//end fucntion ExtractFeatures

    void DoWork(){
        while(1){

            {
                std::lock_guard<std::mutex> lock(cloud_mutex);
                if(deque_cloud.empty()){
                    sleep(0.01);
                    continue;
                }
            }
//            if(!isempty){
            {
                CloudInfo cur_cloud;
                cloud_mutex.lock();
                cur_cloud = deque_cloud.front();
                deque_cloud.pop_front();
                cloud_mutex.unlock();

                //do some work
                TicToc timer;
                CalculateSmoothness(cur_cloud);
                MarkOccludedPoints(cur_cloud);
//                EZLOG(INFO)<<"before feature extraction cost time(ms) = "<<timer.toc()<<std::endl;
                TicToc timer2;
                pcl::PointCloud<PointType>::Ptr cornerCloud(new  pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr surfaceCloud(new  pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr rawCloud(new  pcl::PointCloud<PointType>());
                ExtractFeatures(cur_cloud, cornerCloud, surfaceCloud,rawCloud);
                TicToc timer1;

                CloudFeature cloud_feature;
                cloud_feature.timestamp = cur_cloud.timestamp;
                cloud_feature.pose = cur_cloud.pose;
                cloud_feature.pose_reliable = cur_cloud.pose_reliable;
                cloud_feature.cov = cur_cloud.cov;
                cloud_feature.DRPose = cur_cloud.DRPose;
                cloud_feature.frame_id = cur_cloud.frame_id;
                cloud_feature.cornerCloud = cornerCloud;
                cloud_feature.surfaceCloud = surfaceCloud;

                if(MappingConfig::slam_mode_switch == 1 ){
                    Function_AddCloudFeatureToOPTMapping(cloud_feature);
                  //  EZLOG(INFO)<<"3 feature_extraction send to Mapping!And current lidar pointCloud surfaceCloud size is: "<<cloud_feature.surfaceCloud->points.size()<<", cornerCloud is: "<<cloud_feature.cornerCloud->points.size();
                }
                if(LocConfig::slam_mode_on == 1){
                    Function_AddCloudFeatureToLOCMapping(cloud_feature);
                  //  EZLOG(INFO)<<"3 feature_extraction send to Loc! And current lidar pointCloud surfaceCloud size is: "<<cloud_feature.surfaceCloud->points.size()<<", cornerCloud is: "<<cloud_feature.cornerCloud->points.size();
                }

                //for debug use
                if(MappingConfig::if_debug)
                {
                    CloudTypeXYZI corner_pub,surf_pub;
                    corner_pub.frame = "map";
                    corner_pub.timestamp = cur_cloud.timestamp;
                    corner_pub.cloud = *cornerCloud;
                    surf_pub.frame = "map";
                    surf_pub.timestamp = cur_cloud.timestamp;
                    surf_pub.cloud = *surfaceCloud;
                    pubsub->PublishCloud(topic_corner_world, corner_pub);
                    pubsub->PublishCloud(topic_surf_world, surf_pub);
                }

            }

        }
    }

    void AddCloudData(const CloudInfo& data){
      //  EZLOG(INFO)<<"featureext_AddCloudData  "<<std::endl;
        cloud_mutex.lock();
        deque_cloud.push_back(data);
        cloud_mutex.unlock();
    }

    void Init(PubSubInterface* pubsub_){
        AllocateMemeory();
        downSizeFilter.setLeafSize(MappingConfig::odometrySurfLeafSize, MappingConfig::odometrySurfLeafSize,
                                   MappingConfig::odometrySurfLeafSize);

        pubsub = pubsub_;
        pubsub->addPublisher(topic_corner_world, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_surf_world, DataType::LIDAR, 10);
        do_work_thread = new std::thread(&FeatureExtraction::DoWork, this);

    }

};
#endif
