#include <iostream>
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include "pcl/point_types.h"
#include "pcl/registration/ndt.h"
#include "pcl/filters/approximate_voxel_grid.h"
#include "pcl/visualization/cloud_viewer.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>
#include <Eigen/Core>           //eigen
#include <Eigen/Geometry>       //Isometry3d
#include "ros/ros.h"
#include <chrono>
#include "MapSaver.h"
#include "yaml-cpp/yaml.h"
#include "utility.h"
#include  <memory>

#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)(float, time, time)
)

using PointXYZIRT = VelodynePointXYZIRT;
typedef pcl::PointXYZ pclPointType;
//typedef pcl::PointXYZ pclPointType;

class priorMapLoc {
public:
    //ros
    ros::NodeHandle nh;
    ros::Subscriber subCurrentScan;
    ros::Publisher pubPriorSurfMap;
    ros::Publisher pubNDTAlignedScan;
    ros::Publisher pubCurrentScan;
    sensor_msgs::PointCloud2 currentScanMsg;
    sensor_msgs::PointCloud2 pubMsg_Surf_proirMap;
    sensor_msgs::PointCloud2 pubMsg_NDT_AlignedScan;
    sensor_msgs::PointCloud2 pubMsg_currentScan;
    std::deque<sensor_msgs::PointCloud2> ScanMsgQueue;


    //prior map and pose init
    bool flag_read_map,flag_read_pose;
    int scanNum;
    std::vector<Eigen::Matrix3f> rotation_matrix_prior;
    std::vector<Eigen::Vector3f> translation_vector_prior;
    pcl::PointCloud<pclPointType>::Ptr prior_map_ptr;
    pcl::PointCloud<pclPointType>::Ptr current_scan_temp_ptr;
    pcl::PointCloud<pclPointType>::Ptr current_scan_ptr;
    pcl::PointCloud<PointXYZIRT>::Ptr current_scanIn;
    pcl::PointCloud<pclPointType>::Ptr ds_current_scan_ptr;
    pcl::PointCloud<pclPointType>::Ptr NDT_aligned_scan_ptr;
    pcl::PointCloud<pclPointType>::Ptr NDT_aligned_scan_test_ptr;
    Eigen::Matrix4f initial_transform;
    double sequence_num;

    priorMapLoc():flag_read_map(false),flag_read_pose(false),scanNum(0)
    {
//        read raw lidar message without deskew
        EZLOG(INFO)<<"0.1 Init------------------NOW we are in priorMapLoc()";
        prior_map_ptr.reset(new pcl::PointCloud<pclPointType>);
//        current_scan_ptr.reset(new pcl::PointCloud<pclPointType>);
//        ds_current_scan_ptr.reset(new pcl::PointCloud<pclPointType>);
//        NDT_aligned_scan_ptr.reset(new pcl::PointCloud<pclPointType>);
//        NDT_aligned_scan_test_ptr.reset(new pcl::PointCloud<pclPointType>);
//        current_scan_temp_ptr.reset(new pcl::PointCloud<pclPointType>);

        subCurrentScan = nh.subscribe<sensor_msgs::PointCloud2>(SensorConfig::pointCloudTopic,
                                                               5,
                                                               &priorMapLoc::currentScanHandler,
                                                               this,
                                                               ros::TransportHints().tcpNoDelay());
        pubPriorSurfMap = nh.advertise<sensor_msgs::PointCloud2>("prior_surf_Map", 5);
        pubNDTAlignedScan = nh.advertise<sensor_msgs::PointCloud2>("NDT_aligned_scan", 5);
        pubCurrentScan = nh.advertise<sensor_msgs::PointCloud2>("Current_Scan", 5);

        EZLOG(INFO)<<"Publish ros topic!"<<std::endl;
        // 读取先验地图
        if(!flag_read_map) {
            EZLOG(INFO)<<"reading prior map file!"<<std::endl;
            loadPriorMap(prior_map_ptr);
            flag_read_map = true;
        }
        EZLOG(INFO)<<"Publish Ros Viewer"<<std::endl;
        pubPriorMap(prior_map_ptr);
    }

    void loadPriorMap(pcl::PointCloud<pclPointType>::Ptr prior_map_ptr){
        EZLOG(INFO)<<"0.2 Init------------------NOW we are in loadPriorMap()";
        EZLOG(INFO)<<"read prior map file from : "<<SerializeConfig::prior_map_path<<std::endl;
        if (pcl::io::loadPCDFile<pclPointType>(SerializeConfig::prior_map_path, *prior_map_ptr) == -1)
        {
            EZLOG(INFO)<<"Couldn't read prior map file"<<std::endl;
            return ;
        }
        EZLOG(INFO)<<"Read prior map!"<<std::endl;
    }

    void currentScanHandler(const sensor_msgs::PointCloud2ConstPtr &currentScanMsg){

        current_scan_ptr.reset(new pcl::PointCloud<pclPointType>);
        ds_current_scan_ptr.reset(new pcl::PointCloud<pclPointType>);
        NDT_aligned_scan_ptr.reset(new pcl::PointCloud<pclPointType>);
        NDT_aligned_scan_test_ptr.reset(new pcl::PointCloud<pclPointType>);
        current_scan_temp_ptr.reset(new pcl::PointCloud<pclPointType>);

        EZLOG(INFO)<<"1.1 Handler------------------NOW we are in currentScanHandler()";
        EZLOG(INFO)<<"I hear from " << SensorConfig::pointCloudTopic;

        loadCurrentScan(current_scan_temp_ptr,current_scan_ptr,currentScanMsg);
        pubCurrentLidarScan(current_scan_ptr);

        downSampleCurrentScan(current_scan_ptr,ds_current_scan_ptr);

        loadPriorPose(initial_transform);
        //            auto start_time = std::chrono::high_resolution_clock::now();

        NDTSequence(prior_map_ptr,ds_current_scan_ptr,initial_transform,NDT_aligned_scan_ptr);
        pubNDTScan(NDT_aligned_scan_ptr);

    }
    void loadCurrentScan(pcl::PointCloud<pclPointType>::Ptr _current_scan_temp_ptr,
                         pcl::PointCloud<pclPointType>::Ptr _current_scan_ptr,
                         const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        EZLOG(INFO)<<"1.2 Handler------------------NOW we are in loadCurrentScan()";
        ScanMsgQueue.push_back(*laserCloudMsg);
//        if (ScanMsgQueue.size() <= 2)
//        {
//            return;
//        }
        currentScanMsg = std::move(ScanMsgQueue.front());
//        ScanMsgQueue.pop_front();

        pcl::moveFromROSMsg(currentScanMsg, *_current_scan_temp_ptr);
        int cloudSize = _current_scan_temp_ptr->points.size();
        EZLOG(INFO) << "cur_scan_cloud_body pts size = " << cloudSize << std::endl;
        // remove NaN points with 2 method
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*_current_scan_temp_ptr,*_current_scan_temp_ptr,indices);

        pcl::PointCloud<pclPointType>::iterator it = _current_scan_temp_ptr->points.begin();
        while (it != _current_scan_temp_ptr->points.end())
        {
            float x, y, z, rgb;
            x = it->x;
            y = it->y;
            z = it->z;
            if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
            {
                it = _current_scan_temp_ptr->points.erase(it);
            }
            else
                ++it;
        }
        cloudSize = _current_scan_temp_ptr->points.size();
        EZLOG(INFO) << "After earse NaN points cur_scan_cloud_body pts NOW size = " << cloudSize << std::endl;
        if (_current_scan_temp_ptr->is_dense == false) {
            EZLOG(INFO)<<"Point cloud is not in dense format, please remove NaN points first!";
        }
        // move current_scan_temp_ptr -> current_scan_ptr
//        pcl::copyPointCloud(*_current_scan_temp_ptr, *_current_scan_ptr);

        // 逐点拷贝点云
        _current_scan_ptr->width = _current_scan_temp_ptr->width;
        _current_scan_ptr->height = _current_scan_temp_ptr->height;
        _current_scan_ptr->points.resize(_current_scan_ptr->width * _current_scan_ptr->height);
        for (size_t i = 0; i < _current_scan_temp_ptr->points.size(); ++i) {
            _current_scan_ptr->points[i] = _current_scan_temp_ptr->points[i];
        }
    }
    void downSampleCurrentScan(const pcl::PointCloud<pclPointType>::Ptr _current_scan,
                               pcl::PointCloud<pclPointType>::Ptr _ds_current_scan){
            EZLOG(INFO)<<"1.3 Handler------------------NOW we are in downSampleCurrentScan()";
            pcl::ApproximateVoxelGrid<pclPointType> approximate_voxel_filter;
            approximate_voxel_filter.setLeafSize(SerializeConfig::setLeafSize, SerializeConfig::setLeafSize,SerializeConfig::setLeafSize);
            EZLOG(INFO)<<"setLeafSize  is : "<<SerializeConfig::setLeafSize;
            approximate_voxel_filter.setInputCloud(_current_scan);
            approximate_voxel_filter.filter(*_ds_current_scan);
            EZLOG(INFO)<<"current_scan size is : "<<_current_scan->size()<<" , filter_current_scan: "<< _ds_current_scan->size();

    }
    void ReadTumPose(const std::string path,
                     std::vector<Eigen::Matrix3f> &_rotation_matrix_prior,
                     std::vector<Eigen::Vector3f> &_translation_vector_prior){
        EZLOG(INFO)<<"1.3 Handler------------------NOW we are in ReadTumPose()";
        Eigen::Vector3f tanslation;
        Eigen::Quaternionf rotation;

        Eigen::Isometry3f T=Eigen::Isometry3f::Identity();
        std::ifstream prior_init_pose_file(path);
        std::string line;
        std::stringstream ss;
        std::vector<float> array_temp;
        int i = 0;
        while(getline(prior_init_pose_file, line)) {
            ss.str(line);
            std::string single;
            array_temp.clear();
            // 按照空格分隔
            while (getline(ss, single, ' ')) {
                array_temp.push_back(atof(single.c_str()));
            }
            ss.clear(); //必须加，不然写不到string里。
            tanslation.x() = array_temp[1];
            tanslation.y() = array_temp[2];
            tanslation.z() = array_temp[3];

            rotation.x() = array_temp[4];
            rotation.y() = array_temp[5];
            rotation.z() = array_temp[6];
            rotation.w() = array_temp[7];

            rotation.normalize();

            _rotation_matrix_prior.emplace_back(rotation.toRotationMatrix());
            _translation_vector_prior.emplace_back(tanslation);
        }
    }
    void loadPriorPose(Eigen::Matrix4f &_initial_transform){
        EZLOG(INFO)<<"1.3 Handler------------------NOW we are in loadPriorPose()";
        if(!flag_read_pose)
        {
            ReadTumPose(SerializeConfig::prior_pose_path,rotation_matrix_prior,translation_vector_prior);
            flag_read_pose = true;
        }
        _initial_transform.setIdentity();
        _initial_transform.block<3,3>(0,0) = rotation_matrix_prior[scanNum];
        _initial_transform.block<3,1>(0,3) = translation_vector_prior[scanNum++];

        EZLOG(INFO) << " initial_transform matrix:" << std::endl << _initial_transform << std::endl;
    }
    void NDTSequence( pcl::PointCloud<pclPointType>::Ptr _prior_map,
                      pcl::PointCloud<pclPointType>::Ptr _ds_current_scan,
                     const Eigen::Matrix4f &_initial_transform,
                     pcl::PointCloud<pclPointType>::Ptr _NDT_aligned_scan)
    {
        // 初始化NDT配准对象
        EZLOG(INFO)<<"1.3 Handler------------------NOW we are in NDTSequence()";
        pcl::NormalDistributionsTransform<pclPointType, pclPointType> ndt;

        ndt.setInputSource(_ds_current_scan);
        ndt.setInputTarget(_prior_map);
        EZLOG(INFO)<<"  filter_current_scan size: "<< _ds_current_scan->points.size()
                   <<"  prior_map size: "<< _prior_map->points.size()<<std::endl;


        // 设置NDT参数
        EZLOG(INFO) <<"  Tepsilion: "<< SerializeConfig::Tepsilion
                    <<"  step_size: "<< SerializeConfig::step_size
                    <<"  size_resolution: "<< SerializeConfig::size_resolution
                    <<"  max_inter_num: "<< SerializeConfig::max_inter_num;

        ndt.setTransformationEpsilon(SerializeConfig::Tepsilion);
        ndt.setStepSize(SerializeConfig::step_size);
        ndt.setResolution(SerializeConfig::size_resolution);
        ndt.setMaximumIterations(SerializeConfig::max_inter_num);



        EZLOG(INFO)<<"NDT Aligning!!!!";
        EZLOG(INFO)<<"NDT _initial_transform:"<<std::endl<<_initial_transform;
        pcl::PointCloud<pclPointType>::Ptr output_cloud (new pcl::PointCloud<pclPointType>);


        ndt.align(*output_cloud, _initial_transform);
        if (ndt.hasConverged())
        {
            EZLOG(INFO) << "NDT has converged, score: " << ndt.getFitnessScore() << std::endl;
            Eigen::Matrix4f final_transform = ndt.getFinalTransformation();
            EZLOG(INFO) << "Final transformation matrix:" << std::endl << final_transform << std::endl;
            EZLOG(INFO) << "NDT iteration number: "<<ndt.getFinalNumIteration() << std::endl;

            pcl::transformPointCloud(*_ds_current_scan, *output_cloud, ndt.getFinalTransformation());

        }
        else
        {
            EZLOG(INFO) << "NDT has not converged." << std::endl;
            return ;
        }


    }

    void pubPriorMap(pcl::PointCloud<pclPointType>::Ptr _prior_map_ptr){

//        pub prior map
        EZLOG(INFO)<<"0.3 Init------------------NOW we are in pubPriorMap()";
        if (pubPriorSurfMap.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*_prior_map_ptr, pubMsg_Surf_proirMap);
            pubMsg_Surf_proirMap.header.frame_id = "map";
            pubMsg_Surf_proirMap.header.stamp = ros::Time::now();
            pubPriorSurfMap.publish(pubMsg_Surf_proirMap);
            EZLOG(INFO)<<"Pub surf Prior Map!"<<std::endl;
        }
    }
    void pubCurrentLidarScan(pcl::PointCloud<pclPointType>::Ptr _current_scan_ptr){
        EZLOG(INFO)<<"1.3 Handler------------------NOW we are in pubCurrentLidarScan()";
//        because we changed currrent scan size, so the --- point->size != point->width * point->height
//        pub ndt aligned map
        if (pubCurrentScan.getNumSubscribers() != 0)
        {
            EZLOG(INFO)<<"_current_scan_ptr size is:"<<_current_scan_ptr->size();
            pcl::toROSMsg(*_current_scan_ptr, pubMsg_currentScan);
            pubMsg_currentScan.header.frame_id = "map";
            pubMsg_currentScan.header.stamp = ros::Time::now();
            pubCurrentScan.publish(pubMsg_currentScan);
            EZLOG(INFO)<<"Pub Current Scan!"<<std::endl;
        }

    }
    void pubNDTScan(pcl::PointCloud<pclPointType>::Ptr _NDT_aligned_scan_ptr){
        EZLOG(INFO)<<"1.3 Handler------------------NOW we are in pubNDTScan()";
//        pub ndt aligned map
        if (pubNDTAlignedScan.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*NDT_aligned_scan_test_ptr, pubMsg_NDT_AlignedScan);
            pubMsg_NDT_AlignedScan.header.frame_id = "map";
            pubMsg_NDT_AlignedScan.header.stamp = ros::Time::now();
            pubNDTAlignedScan.publish(pubMsg_NDT_AlignedScan);
            EZLOG(INFO)<<"Pub NDT Aligned Scan!"<<std::endl;
        }

    }
};


int main(int argc, char** argv)
{
    // 读取先验地图点云
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%level %file %line : %msg");
#ifdef ELPP_THREAD_SAFE
    EZLOG(INFO) << "easylogging++ thread safe!";
#else
    EZLOG(INFO) << "easylogging++ thread unsafe";
#endif
    Load_Sensor_YAML("./config/sensor.yaml");
    Load_offline_YAML("./config/offline_mapping.yaml");

    ros::init(argc, argv, "priorMapLoc_node");

    priorMapLoc PM;
    EZLOG(INFO)<<"Prior Map Localization Started!"<<std::endl;

    ros::spin();

    return 0;
}
