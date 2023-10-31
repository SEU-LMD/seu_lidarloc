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

    std::string topic_cloud_pillar_world = "/cloud_pillar_world";
    std::string topic_cloud_beam_world = "/cloud_beam_world";
    std::string topic_cloud_facade_world = "/cloud_facade_world";
    std::string topic_cloud_roof_world = "/cloud_roof_world";

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
            int isempty = false;
             
            CloudInfo cur_cloud;
            cloud_mutex.lock();
            if(deque_cloud.size()==0){
                isempty = true;
            }
            else{
                isempty = false;
                cur_cloud = deque_cloud.front();
                deque_cloud.pop_front();
            }
            cloud_mutex.unlock();

        
            if(!isempty){
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

                if(FrontEndConfig::use_unground_pts_classify){


                    ///classify_nground_pts

//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar_down (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam_down (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade_down (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof_down (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_vertex (new pcl::PointCloud<PointXYZICOLRANGE>);

//                    float pca_neighbor_radius = 1.0;int pca_neighbor_k = 30 ;int pca_neighbor_k_min = 8;int pca_down_rate = 1;
//                    float edge_thre = 0.65 ;float planar_thre = 0.65 ; float edge_thre_down = 0.75 ; float planar_thre_down = 0.75;
//                    int extract_vertex_points_method = 2;float curvature_thre = 0.12;float vertex_curvature_non_max_r = 1.5 * pca_neighbor_radius;
//                    float linear_vertical_sin_high_thre = 0.94;float linear_vertical_sin_low_thre = 0.17;
//                    float planar_vertical_sin_high_thre = 0.98; float planar_vertical_sin_low_thre = 0.34;

                    EZLOG(INFO)<<"cloud_unground->points.size() =  "<<cur_cloud.cloud_unground->points.size()<<endl;

                    TicToc time_classify_nground_pts;

                    classify_nground_pts(cur_cloud.cloud_unground,cloud_feature.cloud_pillar,cloud_feature.cloud_beam,cloud_feature.cloud_facade,cloud_feature.cloud_roof,
                                         cloud_feature.cloud_pillar_down,cloud_feature.cloud_beam_down,cloud_feature.cloud_facade_down,cloud_feature.cloud_roof_down
//                                         cloud_vertex,pca_neighbor_radius, pca_neighbor_k, pca_neighbor_k_min, pca_down_rate,
//                                         edge_thre, planar_thre, edge_thre_down, planar_thre_down,
//                                         extract_vertex_points_method, curvature_thre, vertex_curvature_non_max_r,
//                                         linear_vertical_sin_high_thre, linear_vertical_sin_low_thre,
//                                         planar_vertical_sin_high_thre, planar_vertical_sin_low_thre
//                                         fixed_num_downsampling, pillar_down_fixed_num, facade_down_fixed_num,
//                                         beam_down_fixed_num, roof_down_fixed_num, unground_down_fixed_num,
//                                         beam_height_max, roof_height_min, feature_pts_ratio_guess,
//                                         sharpen_with_nms_on, use_distance_adaptive_pca
                    );
                    EZLOG(INFO)<<"time_classify_nground_pts.toc() =  "<<time_classify_nground_pts.toc()<<endl;

                    EZLOG(INFO)<<"cloud_pillar->points.size() =  "<<cloud_feature.cloud_pillar->points.size()<<endl;
                    EZLOG(INFO)<<"cloud_beam->points.size() =  "<<cloud_feature.cloud_beam->points.size()<<endl;
                    EZLOG(INFO)<<"cloud_facade->points.size() =  "<<cloud_feature.cloud_facade->points.size()<<endl;
                    EZLOG(INFO)<<"cloud_roof->points.size() =  "<<cloud_feature.cloud_roof->points.size()<<endl;

                    if(MappingConfig::if_debug)
                    {

                        CloudTypeXYZICOLRANGE cloud_pillar_pub,cloud_beam_pub,cloud_facade_pub,cloud_roof_pub;
                        cloud_pillar_pub.timestamp = cur_cloud.timestamp;
                        cloud_beam_pub.timestamp = cur_cloud.timestamp;
                        cloud_facade_pub.timestamp = cur_cloud.timestamp;
                        cloud_roof_pub.timestamp = cur_cloud.timestamp;
                        cloud_pillar_pub.frame = "map";
                        cloud_beam_pub.frame = "map";
                        cloud_facade_pub.frame = "map";
                        cloud_roof_pub.frame = "map";
                        pcl::transformPointCloud(*cloud_feature.cloud_pillar, cloud_pillar_pub.cloud, cur_cloud.pose.pose.cast<float>());
                        pcl::transformPointCloud(*cloud_feature.cloud_beam, cloud_beam_pub.cloud, cur_cloud.pose.pose.cast<float>());
                        pcl::transformPointCloud(*cloud_feature.cloud_facade, cloud_facade_pub.cloud, cur_cloud.pose.pose.cast<float>());
                        pcl::transformPointCloud(*cloud_feature.cloud_roof, cloud_roof_pub.cloud, cur_cloud.pose.pose.cast<float>());
                        pubsub->PublishCloud(topic_cloud_pillar_world, cloud_pillar_pub);
                        pubsub->PublishCloud(topic_cloud_beam_world, cloud_beam_pub);
                        pubsub->PublishCloud(topic_cloud_facade_world, cloud_facade_pub);
                        pubsub->PublishCloud(topic_cloud_roof_world, cloud_roof_pub);

                    }

                } // end if(FrontEndConfig::use_unground_pts_classify)

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

            else{
                sleep(0.01);
            }
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
        downSizeFilter.setLeafSize(MappingConfig::odometrySurfLeafSize, MappingConfig::odometrySurfLeafSize,
                                   MappingConfig::odometrySurfLeafSize);

        pubsub = pubsub_;
        pubsub->addPublisher(topic_corner_world, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_surf_world, DataType::LIDAR, 10);

        pubsub->addPublisher(topic_cloud_pillar_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_cloud_beam_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_cloud_facade_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_cloud_roof_world,DataType::LIDAR,1);

        do_work_thread = new std::thread(&FeatureExtraction::DoWork, this);
        slam_mode_switch = _slam_mode_switch;
        if(slam_mode_switch == 0){
            //save_Map_thread = new std::thread(&MapSaver::do_work, &(FeatureExtraction::map_saver));
        }


    }

    struct eigenvalue_t // Eigen Value ,lamada1 > lamada2 > lamada3;
    {
        double lamada1;
        double lamada2;
        double lamada3;
    };

    struct eigenvector_t //the eigen vector corresponding to the eigen value
    {
        Eigen::Vector3f principalDirection;
        Eigen::Vector3f middleDirection;
        Eigen::Vector3f normalDirection;
    };

    struct pca_feature_t //PCA
    {
        eigenvalue_t values;
        eigenvector_t vectors;
        double curvature;
        double linear;
        double planar;
        double spherical;
        double linear_2;
        double planar_2;
        double spherical_2;
        double normal_diff_ang_deg;
        pcl::PointNormal pt;
        int ptId;
        int pt_num = 0;
        std::vector<int> neighbor_indices;
        std::vector<bool> close_to_query_point;
    };

    bool classify_nground_pts(const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud_in,//input 非地面点
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar_down,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam_down,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade_down,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof_down,
//                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_vertex,
//                              float neighbor_searching_radius, int neighbor_k, int neigh_k_min, int pca_down_rate, // one in ${pca_down_rate} unground points would be select as the query points for calculating pca, the else would only be used as neighborhood points
//                              float edge_thre, float planar_thre, float edge_thre_down, float planar_thre_down,
//                              int extract_vertex_points_method, float curvature_thre, float vertex_curvature_non_max_radius,
//                              float linear_vertical_sin_high_thre, float linear_vertical_sin_low_thre,
//                              float planar_vertical_sin_high_thre, float planar_vertical_sin_low_thre,
//                              bool fixed_num_downsampling = false, int pillar_down_fixed_num = 200, int facade_down_fixed_num = 800, int beam_down_fixed_num = 200,
//                              int roof_down_fixed_num = 100, int unground_down_fixed_num = 20000,
                              float beam_height_max = FLT_MAX, float roof_height_min = -FLT_MAX
//                              float feature_pts_ratio_guess = 0.3, bool sharpen_with_nms = true,
//                              bool use_distance_adaptive_pca = false
                                      )
    {

//        if (fixed_num_downsampling) //false
//            random_downsample_pcl(cloud_in, unground_down_fixed_num);

        //Do PCA
//        PrincipleComponentAnalysis<PointT> pca_estimator;
        std::vector<pca_feature_t> cloud_features;

        typename pcl::KdTreeFLANN<PointXYZICOLRANGE>::Ptr tree(new pcl::KdTreeFLANN<PointXYZICOLRANGE>);
        tree->setInputCloud(cloud_in);

        float unit_distance = 30.0;
        ///1.计算每个非地面点的pca参数
        //output  param = cloud_features
        get_pc_pca_feature(cloud_in, cloud_features, tree, FrontEndConfig::neighbor_searching_radius, FrontEndConfig::neighbor_k, 1, FrontEndConfig::pca_down_rate, FrontEndConfig::use_distance_adaptive_pca, unit_distance);
        //LOG(WARNING)<< "PCA done";

        std::chrono::steady_clock::time_point toc_pca = std::chrono::steady_clock::now();

        //the radius should be larger for far away points
        //1.2.遍历每个非地面点，并根据pca参数为每个点进行归类
        std::vector<int> index_with_feature(cloud_in->points.size(), 0); // 0 - not special points, 1 - pillar, 2 - beam, 3 - facade, 4 - roof
        for (int i = 0; i < cloud_in->points.size(); i++)
        {
            if (cloud_features[i].pt_num > FrontEndConfig::neigh_k_min)//后面所有的代码都在这个if下，离群点不对他归类
            {

                if (cloud_features[i].linear_2 > FrontEndConfig::edge_thre)
                {
                    if (std::abs(cloud_features[i].vectors.principalDirection.z()) > FrontEndConfig::linear_vertical_sin_high_thre)
                    {
                        assign_normal(cloud_in->points[i], cloud_features[i], false);
                        cloud_pillar->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 1;
                    }
                    else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < FrontEndConfig::linear_vertical_sin_low_thre &&
                             cloud_in->points[i].z < beam_height_max)
                    {
                        assign_normal(cloud_in->points[i], cloud_features[i], false);
                        cloud_beam->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 2;
                    }
                    else
                    {
                        ;
                    }

                    if (!FrontEndConfig::sharpen_with_nms && cloud_features[i].linear_2 > FrontEndConfig::edge_thre_down)
                    {
                        if (std::abs(cloud_features[i].vectors.principalDirection.z()) > FrontEndConfig::linear_vertical_sin_high_thre)
                            cloud_pillar_down->points.push_back(cloud_in->points[i]);
                        else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < FrontEndConfig::linear_vertical_sin_low_thre && cloud_in->points[i].z < beam_height_max)
                            cloud_beam_down->points.push_back(cloud_in->points[i]);
                        else
                        {
                            ;
                        }
                    }
                }//end if (cloud_features[i].linear_2 > edge_thre)

                else if (cloud_features[i].planar_2 > FrontEndConfig::planar_thre)
                {
                    if (std::abs(cloud_features[i].vectors.normalDirection.z()) > FrontEndConfig::planar_vertical_sin_high_thre && cloud_in->points[i].z > roof_height_min)
                    {
                        assign_normal(cloud_in->points[i], cloud_features[i], true);
                        cloud_roof->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 4;
                    }
                    else if (std::abs(cloud_features[i].vectors.normalDirection.z()) < FrontEndConfig::planar_vertical_sin_low_thre)
                    {
                        assign_normal(cloud_in->points[i], cloud_features[i], true);
                        cloud_facade->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 3;
                    }
                    else
                    {
                        ;
                    }
                    if (!FrontEndConfig::sharpen_with_nms && cloud_features[i].planar_2 > FrontEndConfig::planar_thre_down)
                    {
                        if (std::abs(cloud_features[i].vectors.normalDirection.z()) > FrontEndConfig::planar_vertical_sin_high_thre && cloud_in->points[i].z > roof_height_min)
                            cloud_roof_down->points.push_back(cloud_in->points[i]);
                        else if (std::abs(cloud_features[i].vectors.normalDirection.z()) < FrontEndConfig::planar_vertical_sin_low_thre)
                            cloud_facade_down->points.push_back(cloud_in->points[i]);
                        else
                        {
                            ;
                        }
                    }
                }
            }
        } // end for (int i = 0; i < cloud_in->points.size(); i++)

        //According to the parameter 'extract_vertex_points_method' (0,1,2...)
        if (FrontEndConfig::curvature_thre < 1e-8) // set stablilty_thre as 0 to disable the vertex extraction
            FrontEndConfig::extract_vertex_points_method = 0;

        //Find Edge points by picking high curvature points among the neighborhood of unground geometric feature points (2)
        //1.3.从非特殊点中再根据这个点的周围点的信息，将其归类为beam或者pillar点
        if (FrontEndConfig::extract_vertex_points_method == 2)
        {
            float vertex_feature_ratio_thre = FrontEndConfig::feature_pts_ratio_guess / FrontEndConfig::pca_down_rate;
            for (int i = 0; i < cloud_in->points.size(); i++)
            {
                // if (index_with_feature[i] == 0)
                // 	cloud_vertex->points.push_back(cloud_in->points[i]);
                //这个点是非特殊点，且这个点周围有足够多的点，且这个点的曲率非常大
                if (index_with_feature[i] == 0 &&
                    cloud_features[i].pt_num > FrontEndConfig::neigh_k_min &&
                    cloud_features[i].curvature > FrontEndConfig::curvature_thre) //curvature_thre means curvature_thre here
                {
                    int geo_feature_point_count = 0;
                    for (int j = 0; j < cloud_features[i].neighbor_indices.size(); j++)
                    {
                        if (index_with_feature[cloud_features[i].neighbor_indices[j]])
                            geo_feature_point_count++;
                    }
                    //LOG(INFO)<< "facade neighbor num: " <<geo_feature_point_count;
                    //这个非特殊点周围点特殊点也要足够多
                    if (1.0 * geo_feature_point_count / cloud_features[i].pt_num > vertex_feature_ratio_thre) //most of the neighbors are feature points
                    {
                        //cloud_vertex->points.push_back(cloud_in->points[i]);

                        assign_normal(cloud_in->points[i], cloud_features[i], false);
                        cloud_in->points[i].normal[3] = 5.0 * cloud_features[i].curvature; //save in the un-used normal[3]  (PointNormal4D)
                        if (std::abs(cloud_features[i].vectors.principalDirection.z()) > FrontEndConfig::linear_vertical_sin_high_thre)
                        {
                            cloud_pillar->points.push_back(cloud_in->points[i]);
                            //cloud_pillar_down->points.push_back(cloud_in->points[i]);
                            index_with_feature[i] = 1;
                        }
                        else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < FrontEndConfig::linear_vertical_sin_low_thre && cloud_in->points[i].z < beam_height_max)
                        {
                            cloud_beam->points.push_back(cloud_in->points[i]);
                            //cloud_beam_down->points.push_back(cloud_in->points[i]);
                            index_with_feature[i] = 2;
                        }
                    }
                }
            }
        }

        //if extract_vertex_points_method == 0 ---> do not extract vertex points (0)
//        std::chrono::steady_clock::time_point toc_1 = std::chrono::steady_clock::now();
        //extract neighborhood feature descriptor for pillar points
        //Find Vertex (Edge) points by picking points with maximum local curvature (1)
        //if (extract_vertex_points_method == 1) //Deprecated
        //detect_key_pts(cloud_in, cloud_features, index_with_feature,cloud_vertex, 4.0 * curvature_thre, vertex_curvature_non_max_radius, 0.5 * curvature_thre);
        int min_neighbor_feature_pts = (int)(FrontEndConfig::feature_pts_ratio_guess / FrontEndConfig::pca_down_rate * FrontEndConfig::neighbor_k) - 1;

        //get the vertex keypoints and encode its neighborhood in a simple descriptor
        ///2.为某个点生成描述子
//        encode_stable_points(cloud_in, cloud_vertex, cloud_features, index_with_feature,
//                             0.3 * curvature_thre, min_neighbor_feature_pts, neigh_k_min); //encode the keypoints, we will get a simple descriptor of the putable keypoints

        //LOG(WARNING)<< "encode ncc feature descriptor done";


        //Non_max_suppression of the feature points //TODO: add already built-kd tree here
        ///3.1对pillar cloud_facade beam cloud_roof 特征点进行非最大值抑制
//        if (sharpen_with_nms)
//        {
//            float nms_radius = 0.25 * neighbor_searching_radius;
//#pragma omp parallel sections
//            {
//#pragma omp section
//                {
//                    if (pillar_down_fixed_num > 0)
//                        non_max_suppress(cloud_pillar, cloud_pillar_down, nms_radius);
//                }
//#pragma omp section
//                {
//                    if (facade_down_fixed_num > 0)
//                        non_max_suppress(cloud_facade, cloud_facade_down, nms_radius);
//                }
//#pragma omp section
//                {
//                    if (beam_down_fixed_num > 0)
//                        non_max_suppress(cloud_beam, cloud_beam_down, nms_radius);
//
//                    if (roof_down_fixed_num > 0)
//                        non_max_suppress(cloud_roof, cloud_roof_down, nms_radius);
//                }
//            }
//        }

        ///3.2.对cloud_facade cloud_beam_down cloud_roof_down 将点云分成不同sector，然后在sector中进行随机采样
//        if (fixed_num_downsampling)
//        {
//            random_downsample_pcl(cloud_pillar_down, pillar_down_fixed_num);
//            int sector_num = 4;
//            xy_normal_balanced_downsample(cloud_facade_down, (int)(facade_down_fixed_num / sector_num), sector_num);
//
//            xy_normal_balanced_downsample(cloud_beam_down, (int)(beam_down_fixed_num / sector_num), sector_num); // here the normal is the primary vector
//            //random_downsample_pcl(cloud_roof_down, 100);
//            random_downsample_pcl(cloud_roof_down, roof_down_fixed_num);
//        }

        //Free the memory
        std::vector<pca_feature_t>().swap(cloud_features);
        std::vector<int>().swap(index_with_feature);



        return 1;
    } //end classify_nground_pts

    // R - K neighborhood (with already built-kd tree)
    //within the radius, we would select the nearest K points for calculating PCA
    bool get_pc_pca_feature(typename pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                            std::vector<pca_feature_t> &features,
                            typename pcl::KdTreeFLANN<PointXYZICOLRANGE>::Ptr &tree,
                            float radius, int nearest_k, int min_k = 1, int pca_down_rate = 1,
                            bool distance_adaptive_on = false, float unit_dist = 35.0)
    {
        //LOG(INFO) << "[" << in_cloud->points.size() << "] points used for PCA, pca down rate is [" << pca_down_rate << "]";
        features.resize(in_cloud->points.size());

        for (int i = 0; i < in_cloud->points.size(); i += pca_down_rate) //faster way
        {
            // if (i % pca_down_rate == 0) {//this way is much slower
            std::vector<int> search_indices_used; //points would be stored in sequence (from the closest point to the farthest point within the neighborhood)
            std::vector<int> search_indices;	  //point index vector
            std::vector<float> squared_distances; //distance vector

            float neighborhood_r = radius;
            int neighborhood_k = nearest_k;

            if (distance_adaptive_on)
            {
                double dist = std::sqrt(in_cloud->points[i].x * in_cloud->points[i].x +
                                        in_cloud->points[i].y * in_cloud->points[i].y +
                                        in_cloud->points[i].z * in_cloud->points[i].z);
                if (dist > unit_dist)
                {
                    neighborhood_r = std::sqrt(dist / unit_dist) * radius;
                    //neighborhood_k = (int)(unit_dist / dist * nearest_k));
                }
            }
            //nearest_k=0 --> the knn is disabled, only the rnn is used
            tree->radiusSearch(i, neighborhood_r, search_indices, squared_distances, neighborhood_k);

            features[i].pt.x = in_cloud->points[i].x;
            features[i].pt.y = in_cloud->points[i].y;
            features[i].pt.z = in_cloud->points[i].z;
            features[i].ptId = i;
            features[i].pt_num = search_indices.size();

            //deprecated
            features[i].close_to_query_point.resize(search_indices.size());
            for (int j = 0; j < search_indices.size(); j++)
            {
                if (squared_distances[j] < 0.64 * radius * radius) // 0.5^(2/3)
                    features[i].close_to_query_point[j] = true;
                else
                    features[i].close_to_query_point[j] = false;
            }

            get_pca_feature(in_cloud, search_indices, features[i]);

            if (features[i].pt_num > min_k)
                assign_normal(in_cloud->points[i], features[i]);
            EZLOG(INFO) << 1111 << std::endl;
            std::vector<int>().swap(search_indices);
            std::vector<int>().swap(search_indices_used);
            std::vector<float>().swap(squared_distances);
        }
        //}
        return true;
    }

    /**
		* \brief Use PCL to accomplish the Principle Component Analysis (PCA)
		* of one point and its neighborhood
		* \param[in] in_cloud is the input Point Cloud Pointer
		* \param[in] search_indices is the neighborhood points' indices of the search point.
		* \param[out]feature is the pca_feature_t of the search point.
		*/
    bool get_pca_feature(typename pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                         std::vector<int> &search_indices,
                         pca_feature_t &feature)
    {
        int pt_num = search_indices.size();

        if (pt_num <= 3)
            return false;

        pcl::PointCloud<PointXYZICOLRANGE>::Ptr selected_cloud(new pcl::PointCloud<PointXYZICOLRANGE>());
        EZLOG(INFO) << 1 << std::endl;
        const int maxIdx = in_cloud->points.size();
        for (int i = 0; i < pt_num; ++i) {
            int idx = search_indices[i];
            if(idx < maxIdx){
                selected_cloud->points.push_back(in_cloud->points[search_indices[i]]);
            }else{
                EZLOG(INFO) << "index out of range!!!" << std::endl;
            }
        }
        EZLOG(INFO) << 11 << std::endl;
        pcl::PCA<PointXYZICOLRANGE> pca_operator;
        pca_operator.setInputCloud(selected_cloud);

        // Compute eigen values and eigen vectors
        Eigen::Matrix3f eigen_vectors = pca_operator.getEigenVectors();
        Eigen::Vector3f eigen_values = pca_operator.getEigenValues();

        feature.vectors.principalDirection = eigen_vectors.col(0);
        feature.vectors.normalDirection = eigen_vectors.col(2);

        feature.vectors.principalDirection.normalize();
        feature.vectors.normalDirection.normalize();

        feature.values.lamada1 = eigen_values(0);
        feature.values.lamada2 = eigen_values(1);
        feature.values.lamada3 = eigen_values(2);

        if ((feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3) == 0)
            feature.curvature = 0;
        else
            feature.curvature = feature.values.lamada3 / (feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3);

        // feature.linear_2 = (sqrt(feature.values.lamada1) - sqrt(feature.values.lamada2)) / sqrt(feature.values.lamada1);
        // feature.planar_2 = (sqrt(feature.values.lamada2) - sqrt(feature.values.lamada3)) / sqrt(feature.values.lamada1);
        // feature.spherical_2 = sqrt(feature.values.lamada3) / sqrt(feature.values.lamada1);
        feature.linear_2 = ((feature.values.lamada1) - (feature.values.lamada2)) / (feature.values.lamada1);
        feature.planar_2 = ((feature.values.lamada2) - (feature.values.lamada3)) / (feature.values.lamada1);
        feature.spherical_2 = (feature.values.lamada3) / (feature.values.lamada1);

        search_indices.swap(feature.neighbor_indices);
        EZLOG(INFO) << 111 << std::endl;
        return true;
    }

    //is_palne_feature (true: assign point normal as pca normal vector, false: assign point normal as pca primary direction vector)
    bool assign_normal(PointXYZICOLRANGE &pt, pca_feature_t &pca_feature, bool is_plane_feature = true)
    {
        if (is_plane_feature)
        {
            pt.normal_x = pca_feature.vectors.normalDirection.x();
            pt.normal_y = pca_feature.vectors.normalDirection.y();
            pt.normal_z = pca_feature.vectors.normalDirection.z();
            pt.normal[3] = pca_feature.planar_2; //planrity
        }
        else
        {
            pt.normal_x = pca_feature.vectors.principalDirection.x();
            pt.normal_y = pca_feature.vectors.principalDirection.y();
            pt.normal_z = pca_feature.vectors.principalDirection.z();
            pt.normal[3] = pca_feature.linear_2; //linarity
        }
        return true;
    }

};
#endif
