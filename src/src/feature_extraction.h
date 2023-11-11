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
    int slam_mode_switch = 0;
    PubSubInterface* pubsub;
    std::mutex cloud_mutex;
    // std::mutex work_mutex;//TODO 1029 delete this mutex
    std::deque<CloudInfo> deque_cloud;
    std::thread* do_work_thread;
    std::thread* save_Map_thread;

    std::function<void(const CloudFeature&)> Function_AddCloudFeatureToLOCMapping;
    std::function<void(const CloudFeature&)> Function_AddCloudFeatureToOPTMapping;

    MapSaver map_saver;
    int frame_id = -1;

    std::string topic_corner_world= "/cloud_corner";
    std::string topic_surf_world = "/cloud_surface";

//    std::string topic_cloud_pillar_world = "/cloud_pillar_world";
//    std::string topic_cloud_beam_world = "/cloud_beam_world";
//    std::string topic_cloud_facade_world = "/cloud_facade_world";
//    std::string topic_cloud_roof_world = "/cloud_roof_world";

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
                         pcl::PointCloud<PointType>::Ptr cornerCloud,
                         pcl::PointCloud<PointType>::Ptr surfaceCloud,
                         pcl::PointCloud<PointType>::Ptr rawCloud) {

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
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > MappingConfig::edgeThreshold) {
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

                //extract surface point
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < MappingConfig::surfThreshold) {
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

                //for debug use
                {
//                    int n_cor = 0;
//                    int n_sur = 0;
//                    int n_pt = 0;
//                    for (int n_label = 0; n_label <= cur_scan.label.size(); ++n_label) {
//                        if (cur_scan.label[n_label] == 1) {
//                            ++n_cor;
//                        } else if (cur_scan.label[n_label] == -1) {
//                            ++n_sur;
//                        } else if (cur_scan.label[n_label] == 0) {
//                            ++n_pt;
//                        }
//                    }
//                    EZLOG(INFO) << "n_cor = " << n_cor << std::endl;
//                    EZLOG(INFO) << "n_sur = " << n_sur << std::endl;
//                    EZLOG(INFO) << "n_pt = " << n_pt << std::endl;
//                    EZLOG(INFO) << "cornerCloud->size() = " << cornerCloud->size() << std::endl;
                }

                  // surface point and point doesn't be processed ,regard as surface
                  ///take too much time
//                for (int k = sp; k <= ep; k++) {
//                    if (cloudLabel[k] <= 0) {
//                        PointType pt_tmp;
//                        auto& pt_origin = cur_scan.cloud_ptr->points[k];
//                        pt_tmp.x = pt_origin.x;
//                        pt_tmp.y = pt_origin.y;
//                        pt_tmp.z = pt_origin.z;
//                        pt_tmp.intensity = pt_origin.intensity;
//                        surfaceCloudScan->push_back(pt_tmp);
//                    }
//                }
            }

//            surfaceCloudScanDS->clear();
//            downSizeFilter.setInputCloud(surfaceCloudScan);
//            downSizeFilter.filter(*surfaceCloudScanDS);
//
//            *surfaceCloud += *surfaceCloudScanDS;
        }
    }//end fucntion ExtractFeatures

    void DoWork(){
        while(1){
           // EZLOG(INFO)<<"featureext_DoWork while "<<std::endl;
//            int isempty = false;
//
//            CloudInfo cur_cloud;
//            cloud_mutex.lock();
//            if(deque_cloud.size()==0){
//                isempty = true;
//            }
//            else{
////                EZLOG(INFO)<<"featureext_DoWork  "<<endl;
//                isempty = false;
//                cur_cloud = deque_cloud.front();
////                EZLOG(INFO)<<"cur_cloud =  "<<endl;
//                deque_cloud.pop_front();
//            }
//            cloud_mutex.unlock();
            CloudInfo cur_cloud;
            {
                std::lock_guard<std::mutex> lock(cloud_mutex);
                if(deque_cloud.empty()){
                    sleep(0.01);
                    continue;
                }
                cur_cloud = deque_cloud.front();
                deque_cloud.pop_front();
            }


            {
              //  EZLOG(INFO)<<"featureext_DoWork  "<<std::endl;
                // CloudInfo cur_cloud;
                // cloud_mutex.lock();
                // cur_cloud = deque_cloud.front();
                // deque_cloud.pop_front();
                // cloud_mutex.unlock();

              //  EZLOG(INFO)<<"cur_cloud.cloud_ptr->size() = "<<cur_cloud.cloud_ptr->size()<<std::endl;
          //      EZLOG(INFO)<<"cur_cloud.frame_id = "<<cur_cloud.frame_id<<std::endl;
//                EZLOG(INFO)<<"cur_cloud.cloud_ptr->size() = "<<cur_cloud.cloud_ptr->size()<<std::endl;
//                EZLOG(INFO)<<"cur_cloud.frame_id = "<<cur_cloud.frame_id<<std::endl;

                CloudFeature cloud_feature;

//                if(FrontEndConfig::use_unground_pts_classify){
//
//
//                    ///classify_nground_pts
//
////                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar (new pcl::PointCloud<PointXYZICOLRANGE>);
////                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam (new pcl::PointCloud<PointXYZICOLRANGE>);
////                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade (new pcl::PointCloud<PointXYZICOLRANGE>);
////                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof (new pcl::PointCloud<PointXYZICOLRANGE>);
////                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar_down (new pcl::PointCloud<PointXYZICOLRANGE>);
////                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam_down (new pcl::PointCloud<PointXYZICOLRANGE>);
////                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade_down (new pcl::PointCloud<PointXYZICOLRANGE>);
////                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof_down (new pcl::PointCloud<PointXYZICOLRANGE>);
////                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_vertex (new pcl::PointCloud<PointXYZICOLRANGE>);
//
////                    float pca_neighbor_radius = 1.0;int pca_neighbor_k = 30 ;int pca_neighbor_k_min = 8;int pca_down_rate = 1;
////                    float edge_thre = 0.65 ;float planar_thre = 0.65 ; float edge_thre_down = 0.75 ; float planar_thre_down = 0.75;
////                    int extract_vertex_points_method = 2;float curvature_thre = 0.12;float vertex_curvature_non_max_r = 1.5 * pca_neighbor_radius;
////                    float linear_vertical_sin_high_thre = 0.94;float linear_vertical_sin_low_thre = 0.17;
////                    float planar_vertical_sin_high_thre = 0.98; float planar_vertical_sin_low_thre = 0.34;
//
//                    EZLOG(INFO)<<"cloud_unground->points.size() =  "<<cur_cloud.cloud_unground->points.size()<<endl;
//
//                    TicToc time_classify_nground_pts;
//
//                    classify_nground_pts(cur_cloud.cloud_unground,cloud_feature.cloud_pillar,cloud_feature.cloud_beam,cloud_feature.cloud_facade,cloud_feature.cloud_roof,
//                                         cloud_feature.cloud_pillar_down,cloud_feature.cloud_beam_down,cloud_feature.cloud_facade_down,cloud_feature.cloud_roof_down
////                                         cloud_vertex,pca_neighbor_radius, pca_neighbor_k, pca_neighbor_k_min, pca_down_rate,
////                                         edge_thre, planar_thre, edge_thre_down, planar_thre_down,
////                                         extract_vertex_points_method, curvature_thre, vertex_curvature_non_max_r,
////                                         linear_vertical_sin_high_thre, linear_vertical_sin_low_thre,
////                                         planar_vertical_sin_high_thre, planar_vertical_sin_low_thre
////                                         fixed_num_downsampling, pillar_down_fixed_num, facade_down_fixed_num,
////                                         beam_down_fixed_num, roof_down_fixed_num, unground_down_fixed_num,
////                                         beam_height_max, roof_height_min, feature_pts_ratio_guess,
////                                         sharpen_with_nms_on, use_distance_adaptive_pca
//                    );
//                    EZLOG(INFO)<<"time_classify_nground_pts.toc() =  "<<time_classify_nground_pts.toc()<<endl;
//
//                    EZLOG(INFO)<<"cloud_pillar->points.size() =  "<<cloud_feature.cloud_pillar->points.size()<<endl;
//                    EZLOG(INFO)<<"cloud_beam->points.size() =  "<<cloud_feature.cloud_beam->points.size()<<endl;
//                    EZLOG(INFO)<<"cloud_facade->points.size() =  "<<cloud_feature.cloud_facade->points.size()<<endl;
//                    EZLOG(INFO)<<"cloud_roof->points.size() =  "<<cloud_feature.cloud_roof->points.size()<<endl;
//
//                    if(MappingConfig::if_debug)
//                    {
//
//                        CloudTypeXYZICOLRANGE cloud_pillar_pub,cloud_beam_pub,cloud_facade_pub,cloud_roof_pub;
//                        cloud_pillar_pub.timestamp = cur_cloud.timestamp;
//                        cloud_beam_pub.timestamp = cur_cloud.timestamp;
//                        cloud_facade_pub.timestamp = cur_cloud.timestamp;
//                        cloud_roof_pub.timestamp = cur_cloud.timestamp;
//                        cloud_pillar_pub.frame = "map";
//                        cloud_beam_pub.frame = "map";
//                        cloud_facade_pub.frame = "map";
//                        cloud_roof_pub.frame = "map";
//                        pcl::transformPointCloud(*cloud_feature.cloud_pillar, cloud_pillar_pub.cloud, cur_cloud.pose.pose.cast<float>());
//                        pcl::transformPointCloud(*cloud_feature.cloud_beam, cloud_beam_pub.cloud, cur_cloud.pose.pose.cast<float>());
//                        pcl::transformPointCloud(*cloud_feature.cloud_facade, cloud_facade_pub.cloud, cur_cloud.pose.pose.cast<float>());
//                        pcl::transformPointCloud(*cloud_feature.cloud_roof, cloud_roof_pub.cloud, cur_cloud.pose.pose.cast<float>());
//                        pubsub->PublishCloud(topic_cloud_pillar_world, cloud_pillar_pub);
//                        pubsub->PublishCloud(topic_cloud_beam_world, cloud_beam_pub);
//                        pubsub->PublishCloud(topic_cloud_facade_world, cloud_facade_pub);
//                        pubsub->PublishCloud(topic_cloud_roof_world, cloud_roof_pub);
//
//                    }
//
//                } // end if(FrontEndConfig::use_unground_pts_classify)

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
//                EZLOG(INFO)<<"feature extraction cost time(ms) = "<<timer2.toc()<<std::endl;
                TicToc timer1;

//                int n_cor =0;
//                int n_sur =0;
//                for(int k_rawCloud = 0;k_rawCloud <= rawCloud->size();++k_rawCloud){
//                    if(rawCloud->points[k_rawCloud].intensity == 1){
//                        ++n_cor;
//                    }else if(rawCloud->points[k_rawCloud].intensity == -1){
//                        ++n_sur;
//                    }
//                }
//                EZLOG(INFO)<<"n_cor = "<<n_cor<<std::endl;
          //      EZLOG(INFO)<<"cornerCloud->size() = "<<cornerCloud->size();
//                EZLOG(INFO)<<"n_sur = "<<n_sur<<std::endl;
           //     EZLOG(INFO)<<"surfaceCloud->size() = "<<surfaceCloud->size();
//                int n_cor =0;
//                int n_sur =0;
//                for(int k_rawCloud = 0;k_rawCloud <= rawCloud->size();++k_rawCloud){
//                    if(rawCloud->points[k_rawCloud].intensity == 1){
//                        ++n_cor;
//                    }else if(rawCloud->points[k_rawCloud].intensity == -1){
//                        ++n_sur;
//                    }
//                }
//                EZLOG(INFO)<<"n_cor = "<<n_cor<<std::endl;
//                EZLOG(INFO)<<"cornerCloud->size() = "<<cornerCloud->size()<<std::endl;
//                EZLOG(INFO)<<"n_sur = "<<n_sur<<std::endl;
//                EZLOG(INFO)<<"surfaceCloud->size() = "<<surfaceCloud->size()<<std::endl;

//                ///save cloud with label
//                if(MappingConfig::slam_mode_switch == 0)
//                {
//                    CloudInfoFt raw_cloud;
//                    raw_cloud.raw_Cloud = rawCloud;
//                    raw_cloud.frame_id = ++frame_id;
//                    //map_saver.AddCloudToSave(raw_cloud);
//                    EZLOG(INFO)<<"save cloud with label!"<<std::endl;
//                }
//                CloudInfoFt raw_cloud;
//                raw_cloud.raw_cloud = cur_cloud.cloud_ptr;
//                raw_cloud.frame_id = ++frame_id;
//                map_saver.AddCloudToSave(raw_cloud);
//                EZLOG(INFO)<<"save cloud with label!"<<std::endl;


                cloud_feature.timestamp = cur_cloud.timestamp;
                cloud_feature.pose = cur_cloud.pose;
                cloud_feature.DRPose = cur_cloud.DRPose;
                cloud_feature.frame_id = cur_cloud.frame_id;
                cloud_feature.cornerCloud = cornerCloud;
                cloud_feature.surfaceCloud = surfaceCloud;

                EZLOG(INFO)<<"cloud_feature.cornerCloud->size() = "<<cloud_feature.cornerCloud->size()<<std::endl;
                EZLOG(INFO)<<"cloud_feature.surfaceCloud->size() = "<<cloud_feature.surfaceCloud->size()<<std::endl;

//                switch it when you test your code
//                opt_mapping_ptr->AddCloudData(cloud_feature);
//                loc_mapping_ptr->AddCloudData(cloud_feature);

                if(slam_mode_switch == 0 ){
                    Function_AddCloudFeatureToOPTMapping(cloud_feature);
                }
                else{
                    Function_AddCloudFeatureToLOCMapping(cloud_feature);
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
//                EZLOG(INFO)<<"send feature extraction to next = "<<timer1.toc()<<std::endl;

            }
//
//            else{
//                sleep(0.01);
//            }
        }
    }

    void AddCloudData(const CloudInfo& data){
      //  EZLOG(INFO)<<"featureext_AddCloudData  "<<std::endl;
        cloud_mutex.lock();
        deque_cloud.push_back(data);
        cloud_mutex.unlock();
    }

    void Init(PubSubInterface* pubsub_, int _slam_mode_switch){
        AllocateMemeory();
        slam_mode_switch = _slam_mode_switch;
        EZLOG(INFO)<<"feature Extraction init! slam_mode_switch:"<<slam_mode_switch;
        downSizeFilter.setLeafSize(MappingConfig::odometrySurfLeafSize, MappingConfig::odometrySurfLeafSize,
                                   MappingConfig::odometrySurfLeafSize);

        pubsub = pubsub_;
        pubsub->addPublisher(topic_corner_world, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_surf_world, DataType::LIDAR, 10);
        do_work_thread = new std::thread(&FeatureExtraction::DoWork, this);

    }

};
#endif
