
//created by rongxuan && xiaoqiang at 2023.0819 peaceful night
#ifndef SEU_CONFIG_HELPER
#define SEU_CONFIG_HELPER

#define FLT_MAX		__FLT_MAX__
#define DBL_MAX		__DBL_MAX__

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <string>

#include "yaml-cpp/yaml.h"

namespace LidarType{
    std::string HESAI = "hesai";
    std::string VELODYNE = "velodyne";
    std::string OUSTER = "ouster";
    std::string LIVOX = "livox";
}


class SensorConfig{
    public:
    //Topics
    static std::string pointCloudTopic;
    static  std::string  gpsTopic;
    //Frame
    static std::string  mapFrame;
    //GPS Setting
    static bool  useGpsElevation;
    static float  gpsCovThreshold;
    //Lidar settings
    static std::string sensor; //bug*********sensortype
    static int N_SCAN;
    static int  Horizon_SCAN;

    static float  lidarMinRange;
    static float lidarMaxRange;
    static int lidarMinRing;
    static int lidarMaxRing;
    static int LMOpt_Cnt;

    //Extrinsics (lidar -> IMU)
    static Eigen::Vector3d  extrinsicTrans;//
    static Eigen::Matrix3d extrinsicRot;//
    static Eigen::Vector3d  extrinsicTrans_DR;//
    static Eigen::Matrix3d extrinsicRot_DR;//
    static Eigen::Matrix4d T_L_B;
    static Eigen::Matrix4d T_L_DR;
    static Eigen::Matrix3d extrinsicRPY;//  **meiyong
    static Eigen::Quaterniond extrinsicQRPY;//not
    static Eigen::Vector3d t_body_sensor;
    static Eigen::Quaterniond q_body_sensor;

    static bool use_drodom_deskew;
    static bool use_unground_classify;
    static bool use_gnss_deskew;
    static int gtsamGNSSBetweenFactorDistance;
    static int lidarScanDownSample;
};

class MappingConfig{
    public:
        static int slam_mode_switch;

        static Eigen::Vector3d origin_gnss;
        static bool use_deskew;
        // save map
        static std::string save_map_path;

        // LOAM feature threshold
        static float  edgeThreshold;
        static float surfThreshold;
        static int  edgeFeatureMinValidNum;
        static int  surfFeatureMinValidNum;

        //voxel filter paprams
        static float odometrySurfLeafSize;
        static float mappingCornerLeafSize;
        static float  mappingSurfLeafSize;
        static int DownSampleModeSwitch;

        static float odometrySurfRadiusSize_US;
        static float mappingCornerRadiusSize_US;
        static float mappingSurfRadiusSize_US;
        static float surroundingKeyframeDensity_US;

    // robot motion constraint (in case you are using a 2D robot)
        static float z_tollerance;
        static float rotation_tollerance;

        //CPU Params
        static float mappingProcessInterval;

        //Surrounding map
        static double  surroundingkeyframeAddingDistThreshold;
        static double  surroundingkeyframeAddingAngleThreshold;
        static double surroundingKeyframeDensity;
        static double surroundingKeyframeSearchRadius;

        //Loop closure
        static bool  loopClosureEnableFlag;
        static float  loopClosureFrequency;
        static int surroundingKeyframeSize;
        static float historyKeyframeSearchRadius;
        static float historyKeyframeSearchTimeDiff;
        static int historyKeyframeSearchNum;
        static float  historyKeyframeFitnessScore;//

        static double GNSS_noise_roll;
        static double GNSS_noise_pitch;
        static double GNSS_noise_yaw;
        static double GNSS_noise_x;
        static double GNSS_noise_y;
        static double GNSS_noise_z;
        static double LidarOdom_noise_roll;
        static double LidarOdom_noise_pitch;
        static double LidarOdom_noise_yaw;
        static double LidarOdom_noise_x;
        static double LidarOdom_noise_y;
        static double LidarOdom_noise_z;
        static double DR_noise_roll;
        static double DR_noise_pitch;
        static double DR_noise_yaw;
        static double DR_noise_x;
        static double DR_noise_y;
        static double DR_noise_z;
        static int GNSSupdateDR;
        static double loop_noise_roll;
        static double loop_noise_pitch;
        static double loop_noise_yaw;
        static double loop_noise_x;
        static double loop_noise_y;
        static double loop_noise_z;

};

class LocConfig{
public:
    static int slam_mode_on;
    static Eigen::Vector3d origin_gnss;
    // save map
    static std::string save_map_path;

    // LOAM feature threshold
    static int  edgeFeatureMinValidNum;
    static int  surfFeatureMinValidNum;
    static float odometrySurfRadiusSize_US;
    static float mappingCornerRadiusSize_US;
    static float mappingSurfRadiusSize_US;
    static float edgeThreshold;
    static float surfThreshold;

    // robot motion constraint (in case you are using a 2D robot)
    static float z_tollerance;
    static float rotation_tollerance;
    //CPU Params
    static float mappingProcessInterval;
    static int use_DR_or_fuse_in_loc;
    static int maxIters;
    static double surroundingkeyframeAddingDistThreshold;
    static double surroundingkeyframeAddingAngleThreshold;

    // GTSAM noise
    static double GNSS_noise_roll;
    static double GNSS_noise_pitch;
    static double GNSS_noise_yaw;
    static double GNSS_noise_x;
    static double GNSS_noise_y;
    static double GNSS_noise_z;
    static double LidarOdom_noise_roll;
    static double LidarOdom_noise_pitch;
    static double LidarOdom_noise_yaw;
    static double LidarOdom_noise_x;
    static double LidarOdom_noise_y;
    static double LidarOdom_noise_z;
    static double DR_noise_roll;
    static double DR_noise_pitch;
    static double DR_noise_yaw;
    static double DR_noise_x;
    static double DR_noise_y;
    static double DR_noise_z;
    static int GNSSupdateDR;

};
class SerializeConfig{
   public:
      static std::string map_in_path;
      static std::string map_out_path;
      static int up2down_num;
      static double lidar_range;
      static int frame_sum;

      static double Tepsilion;
      static double step_size;
      static float size_resolution;
      static double max_inter_num;
      static double setLeafSize;
      static double sequence_num;

      static int up_grid_size;
      static int lasercloud_width;
      static int lasercloud_height;
      static int lasercloud_num;

   };

class FrontEndConfig{
    public:
    static int slam_mode_switch;
    static int if_debug;
    static bool use_ground_filter;
    static int min_grid_pt_num;
    static float max_ground_height;
    static float grid_resolution;
    static int distance_weight_downsampling_method;
    static float standard_distance;
    static int nonground_random_down_rate;
    static float intensity_thre ;
    static bool apply_grid_wise_outlier_filter ;
    static float outlier_std_scale;
    static int reliable_neighbor_grid_num_thre;
    static int ground_random_down_rate ;
    static float neighbor_height_diff;
    static float max_height_difference;
    static int estimate_ground_normal_method;
    static float normal_estimation_radius;
    static bool fixed_num_downsampling;
    static int ground_random_down_down_rate;

    static bool use_unground_pts_classify;
    static float neighbor_searching_radius;
    static int neighbor_k;
    static int neigh_k_min;
    static int pca_down_rate;
    static float edge_thre;
    static float planar_thre;
    static float edge_thre_down;
    static float planar_thre_down;
    static int extract_vertex_points_method;
    static float curvature_thre;
    static float vertex_curvature_non_max_radius;
    static float linear_vertical_sin_high_thre;
    static float linear_vertical_sin_low_thre;
    static float planar_vertical_sin_high_thre;
    static float planar_vertical_sin_low_thre;
    static int pillar_down_fixed_num;
    static int facade_down_fixed_num;
    static int beam_down_fixed_num;
    static int roof_down_fixed_num;
    static int unground_down_fixed_num;
    static float beam_height_max;
    static float roof_height_min;
    static  float feature_pts_ratio_guess;
    static bool sharpen_with_nms;
    static bool use_distance_adaptive_pca;
    static float gnss_pop_threshold;
    static float gnss_align_threshold;

}; // end FrontEndConfig

class UdpConfig{
    public:
    static std::string cleint_ip;
    static int clinet_port;
    static int server_port;
};//end UdpConfig

std::string SensorConfig::pointCloudTopic = "";
std::string  SensorConfig::gpsTopic = "";

//Frame

std::string  SensorConfig::mapFrame="";

//GPS Setting
Eigen::Matrix4d SensorConfig::T_L_B = Eigen::Matrix4d::Identity();
Eigen::Matrix4d SensorConfig::T_L_DR = Eigen::Matrix4d::Identity();
bool SensorConfig::useGpsElevation=false;
float  SensorConfig::gpsCovThreshold=-1;

//Export settings
std::string SensorConfig::sensor="";
int SensorConfig::N_SCAN=-1;
int  SensorConfig::Horizon_SCAN=-1;
float  SensorConfig::lidarMinRange=-1;
float SensorConfig::lidarMaxRange=-1;
int SensorConfig::lidarMinRing=-1;
int SensorConfig::lidarMaxRing=-1;
int SensorConfig::LMOpt_Cnt = 10;

//IMU Settings

Eigen::Vector3d SensorConfig::extrinsicTrans;
Eigen::Matrix3d SensorConfig::extrinsicRot;
Eigen::Vector3d SensorConfig::extrinsicTrans_DR;
Eigen::Matrix3d SensorConfig::extrinsicRot_DR;
Eigen::Matrix3d SensorConfig::extrinsicRPY;
Eigen::Quaterniond SensorConfig::extrinsicQRPY;
Eigen::Vector3d SensorConfig::t_body_sensor;
Eigen::Quaterniond SensorConfig::q_body_sensor;
int SensorConfig::gtsamGNSSBetweenFactorDistance = 10;

bool SensorConfig::use_drodom_deskew =false;
bool SensorConfig::use_gnss_deskew = false;
bool SensorConfig::use_unground_classify = false;

int SensorConfig::lidarScanDownSample = 2;

int MappingConfig::slam_mode_switch = 0;
std::string MappingConfig::save_map_path = "";

Eigen::Vector3d MappingConfig::origin_gnss = Eigen::Vector3d(0,0,0);
// LOAM feature threshold
float MappingConfig::edgeThreshold=-1;
float MappingConfig::surfThreshold=-1;

int  MappingConfig::edgeFeatureMinValidNum=-1;
int  MappingConfig::surfFeatureMinValidNum=-1;

//voxel filter paprams
float MappingConfig::odometrySurfLeafSize=-1;
float MappingConfig::mappingCornerLeafSize=-1;
float MappingConfig::mappingSurfLeafSize=-1;
int MappingConfig::DownSampleModeSwitch = 0;

float MappingConfig::odometrySurfRadiusSize_US = 0.6;
float MappingConfig::mappingCornerRadiusSize_US = 0.4;
float MappingConfig::mappingSurfRadiusSize_US = 0.6;
float MappingConfig::surroundingKeyframeDensity_US = 2.0;

// robot motion constraint (in case you are using a 2D robot)
float MappingConfig::z_tollerance=-1;
float MappingConfig::rotation_tollerance=-1;

//CPU Params
float MappingConfig::mappingProcessInterval=-1;

//Surrounding map
double  MappingConfig::surroundingkeyframeAddingDistThreshold=-1;
double  MappingConfig::surroundingkeyframeAddingAngleThreshold=-1;
double MappingConfig::surroundingKeyframeDensity=-1;
double MappingConfig::surroundingKeyframeSearchRadius=-1;

//Loop closure
bool  MappingConfig::loopClosureEnableFlag=false;
float  MappingConfig::loopClosureFrequency=-1;
int MappingConfig::surroundingKeyframeSize=-1;
float MappingConfig::historyKeyframeSearchRadius=-1;
float MappingConfig::historyKeyframeSearchTimeDiff=-1;
int MappingConfig::historyKeyframeSearchNum=-1;
float MappingConfig::historyKeyframeFitnessScore=-1;

//GTSAM
double MappingConfig::GNSS_noise_roll=1e-2;
double MappingConfig::GNSS_noise_pitch =1e-2;
double MappingConfig::GNSS_noise_yaw = 1e-2;
double MappingConfig::GNSS_noise_x = 1e-6;
double MappingConfig::GNSS_noise_y = 1e-6;
double MappingConfig::GNSS_noise_z =  1e-6;
double MappingConfig::LidarOdom_noise_roll = 1e-2;
double MappingConfig::LidarOdom_noise_pitch = 1e-2;
double MappingConfig::LidarOdom_noise_yaw = 1e-2;
double MappingConfig::LidarOdom_noise_x = 1e-2;
double MappingConfig::LidarOdom_noise_y = 1e-2;
double MappingConfig::LidarOdom_noise_z = 1e-2;
double MappingConfig::DR_noise_roll = 1e-1;
double MappingConfig::DR_noise_pitch = 1e-1;
double MappingConfig::DR_noise_yaw = 1e-1;
double MappingConfig::DR_noise_x = 1e-4;
double MappingConfig::DR_noise_y = 1e-4;
double MappingConfig::DR_noise_z = 1e-4;
int MappingConfig::GNSSupdateDR = 0;
double MappingConfig::loop_noise_roll = 1e-1;
double MappingConfig::loop_noise_pitch = 1e-1;
double MappingConfig::loop_noise_yaw = 1e-1;
double MappingConfig::loop_noise_x = 1e-4;
double MappingConfig::loop_noise_y = 1e-4;
double MappingConfig::loop_noise_z = 1e-4;

int LocConfig::maxIters = 30;
int LocConfig::slam_mode_on = 0;
std::string LocConfig::save_map_path = "";
int  LocConfig::edgeFeatureMinValidNum=-1;
int  LocConfig::surfFeatureMinValidNum=-1;
float LocConfig::edgeThreshold = 1.0;
float LocConfig::surfThreshold = 1.0;
float LocConfig::odometrySurfRadiusSize_US = 0.6;
float LocConfig::mappingCornerRadiusSize_US = 0.4;
float LocConfig::mappingSurfRadiusSize_US = 0.6;
float LocConfig::z_tollerance=-1;
float LocConfig::rotation_tollerance=-1;
float LocConfig::mappingProcessInterval=-1;
double  LocConfig::surroundingkeyframeAddingDistThreshold = 1.0;
double  LocConfig::surroundingkeyframeAddingAngleThreshold = 3.0;
int LocConfig::use_DR_or_fuse_in_loc = 1;
double LocConfig::GNSS_noise_roll = 1e-2;
double LocConfig::GNSS_noise_pitch = 1e-2;
double LocConfig::GNSS_noise_yaw = 1e-2;
double LocConfig::GNSS_noise_x = 1e-2;
double LocConfig::GNSS_noise_y = 1e-2;
double LocConfig::GNSS_noise_z = 1e-2;
double LocConfig::LidarOdom_noise_roll = 1e-2;
double LocConfig::LidarOdom_noise_pitch = 1e-2;
double LocConfig::LidarOdom_noise_yaw = 1e-2;
double LocConfig::LidarOdom_noise_x = 1e-2;
double LocConfig::LidarOdom_noise_y = 1e-2;
double LocConfig::LidarOdom_noise_z = 1e-2;
double LocConfig::DR_noise_x = 1e-2;
double LocConfig::DR_noise_y = 1e-2;
double LocConfig::DR_noise_z = 1e-2;
double LocConfig::DR_noise_roll = 1e-2;
double LocConfig::DR_noise_pitch = 1e-2;
double LocConfig::DR_noise_yaw = 1e-2;
int LocConfig::GNSSupdateDR = 0;

// offline mapping
std::string SerializeConfig::map_in_path = "";
std::string SerializeConfig::map_out_path = "";
int SerializeConfig::up2down_num = 4;
double SerializeConfig::lidar_range = 50;

int SerializeConfig::frame_sum = 185;
int SerializeConfig::up_grid_size = 200;
int SerializeConfig::lasercloud_width=0;
int SerializeConfig::lasercloud_height=0;
int SerializeConfig::lasercloud_num=0;
//PriorMap localization

double SerializeConfig::Tepsilion = 0.01;
double SerializeConfig::step_size = 1.0;
float SerializeConfig::size_resolution = 10.0;
double SerializeConfig::max_inter_num = 35;
double SerializeConfig::setLeafSize = 0.6;
double SerializeConfig::sequence_num = 271;

// ground filter
int FrontEndConfig::slam_mode_switch = 0;
int FrontEndConfig::if_debug = 1;
bool FrontEndConfig::use_ground_filter = true;
int FrontEndConfig::min_grid_pt_num = 8;
//float FrontEndConfig::max_ground_height = 50.0;
float FrontEndConfig::grid_resolution = 1.5;
int FrontEndConfig::distance_weight_downsampling_method =0;
float FrontEndConfig::standard_distance =15.0;
int FrontEndConfig::nonground_random_down_rate =1;
float FrontEndConfig::intensity_thre = -1;
bool FrontEndConfig::apply_grid_wise_outlier_filter =false;
float FrontEndConfig::outlier_std_scale =3.0;
int FrontEndConfig::reliable_neighbor_grid_num_thre =0;
int FrontEndConfig::ground_random_down_rate =1;
float FrontEndConfig::neighbor_height_diff =25.0;
float FrontEndConfig::max_height_difference =0.3;
int FrontEndConfig::estimate_ground_normal_method =3;
float FrontEndConfig::normal_estimation_radius =2.0;
bool FrontEndConfig::fixed_num_downsampling =false;
int FrontEndConfig::ground_random_down_down_rate =2;

//unground pts classify
bool FrontEndConfig::use_unground_pts_classify = true;
float FrontEndConfig::neighbor_searching_radius = 1.0;
int FrontEndConfig::neighbor_k = 30;
int FrontEndConfig::neigh_k_min= 8;
int FrontEndConfig::pca_down_rate=1;
float FrontEndConfig::edge_thre=0.65;
float FrontEndConfig::planar_thre=0.65;
float FrontEndConfig::edge_thre_down=0.75;
float FrontEndConfig::planar_thre_down=0.75;
int FrontEndConfig::extract_vertex_points_method=2;
float FrontEndConfig::curvature_thre =0.12;
float FrontEndConfig::vertex_curvature_non_max_radius = 1.5;
float FrontEndConfig::linear_vertical_sin_high_thre =0.94;
float FrontEndConfig::linear_vertical_sin_low_thre=0.17;
float FrontEndConfig::planar_vertical_sin_high_thre =0.98;
float FrontEndConfig::planar_vertical_sin_low_thre=0.34;
int FrontEndConfig::pillar_down_fixed_num = 200;
int FrontEndConfig::facade_down_fixed_num = 800 ;
int FrontEndConfig::beam_down_fixed_num = 200;
int FrontEndConfig::roof_down_fixed_num = 100;
int FrontEndConfig::unground_down_fixed_num = 20000;
//float FrontEndConfig::beam_height_max = 500.0;
//float FrontEndConfig::roof_height_min = 500.0;
float FrontEndConfig::feature_pts_ratio_guess = 0.3 ;
bool FrontEndConfig::sharpen_with_nms = true;
bool FrontEndConfig::use_distance_adaptive_pca = false;
float FrontEndConfig::gnss_pop_threshold = 0.01f ;
float FrontEndConfig::gnss_align_threshold = 0.01f;

//udp
std::string UdpConfig::cleint_ip = " ";
int UdpConfig::clinet_port = -1;
int UdpConfig::server_port = -1;

void SetOptMappingMode(){
    FrontEndConfig::slam_mode_switch = 1;
}

void SetOptLocationMode(){
    FrontEndConfig::slam_mode_switch = 0;
}

void Load_Sensor_YAML(std::string sensorpath)
{
    YAML::Node sensorconfig;
    try{
        sensorconfig = YAML::LoadFile(sensorpath);
    } catch(YAML::BadFile &e) {
        std::cout<<"sensorconfig yaml read error!"<<sensorpath<<std::endl;
        exit(1);
    }

    SensorConfig::pointCloudTopic = sensorconfig["pointCloudTopic"].as<std::string>();
    SensorConfig::gpsTopic = sensorconfig["gpsTopic"].as<std::string>();
    SensorConfig::mapFrame=sensorconfig["mapFrame"].as<std::string>();

//     //GPS Setting
    SensorConfig::useGpsElevation=sensorconfig["useGpsElevation"].as<bool>();
    SensorConfig::gpsCovThreshold=sensorconfig["gpsCovThreshold"].as<float >();

    SensorConfig::sensor=sensorconfig["sensor"].as<std::string >();
    SensorConfig::N_SCAN=sensorconfig["N_SCAN"].as<int >();
    SensorConfig::Horizon_SCAN=sensorconfig["Horizon_SCAN"].as<int >();
    SensorConfig::lidarMinRange=sensorconfig["lidarMinRange"].as<float >();
    SensorConfig::lidarMaxRange=sensorconfig["lidarMaxRange"].as<float >();
    SensorConfig::lidarMinRing=sensorconfig["lidarMinRing"].as<int>();
    SensorConfig::lidarMaxRing=sensorconfig["lidarMaxRing"].as<int>();
    SensorConfig::LMOpt_Cnt=sensorconfig["LMOpt_Cnt"].as<int>();
    SensorConfig::use_gnss_deskew = sensorconfig["use_drodom_deskew"].as<bool>();
    SensorConfig::use_unground_classify = sensorconfig["use_drodom_deskew"].as<bool>();


//    std::cout<<SensorConfig::lidarMaxRange<<std::endl;
    //IMU Settings

    SensorConfig::use_drodom_deskew=sensorconfig["use_drodom_deskew"].as<bool>();

    //Extrinsics (lidar -> IMU)
    SensorConfig::extrinsicTrans<<sensorconfig["extrinsicTrans"][0].as<double >(),sensorconfig["extrinsicTrans"][1].as<double >(),sensorconfig["extrinsicTrans"][2].as<double >();
    //construct(Config::extrinsicTrans,config["extrinsicTrans"].as<double >());
    //std::cout<<Config::extrinsicTrans<<std::endl;
    // std::cout << "key type " <<config["extrinsicTrans"].size() << std::endl;
    SensorConfig::extrinsicRot<<sensorconfig["extrinsicRot"][0].as<double >(),sensorconfig["extrinsicRot"][1].as<double >(),sensorconfig["extrinsicRot"][2].as<double >(),
            sensorconfig["extrinsicRot"][3].as<double >(),sensorconfig["extrinsicRot"][4].as<double >(),sensorconfig["extrinsicRot"][5].as<double >(),
            sensorconfig["extrinsicRot"][6].as<double >(),sensorconfig["extrinsicRot"][7].as<double >(),sensorconfig["extrinsicRot"][8].as<double >();


    SensorConfig::extrinsicRPY<<sensorconfig["extrinsicRPY"][0].as<double >(),sensorconfig["extrinsicRPY"][1].as<double >(),sensorconfig["extrinsicRPY"][2].as<double >(),
            sensorconfig["extrinsicRPY"][3].as<double >(),sensorconfig["extrinsicRPY"][4].as<double >(),sensorconfig["extrinsicRPY"][5].as<double >(),
            sensorconfig["extrinsicRPY"][6].as<double >(),sensorconfig["extrinsicRPY"][7].as<double >(),sensorconfig["extrinsicRPY"][8].as<double >();

    SensorConfig::extrinsicRot_DR<<sensorconfig["extrinsicRot_DR"][0].as<double >(),sensorconfig["extrinsicRot_DR"][1].as<double >(),sensorconfig["extrinsicRot_DR"][2].as<double >(),
            sensorconfig["extrinsicRot_DR"][3].as<double >(),sensorconfig["extrinsicRot_DR"][4].as<double >(),sensorconfig["extrinsicRot_DR"][5].as<double >(),
            sensorconfig["extrinsicRot_DR"][6].as<double >(),sensorconfig["extrinsicRot_DR"][7].as<double >(),sensorconfig["extrinsicRot_DR"][8].as<double >();


    SensorConfig::extrinsicTrans_DR<< sensorconfig["extrinsicTrans_DR"][0].as<double >(),
                                    sensorconfig["extrinsicTrans_DR"][1].as<double >(),
                                    sensorconfig["extrinsicTrans_DR"][2].as<double >();


    Eigen::Matrix4d T_L_B_tmp = Eigen::Matrix4d::Identity();
    T_L_B_tmp.block<3,3>(0,0) = SensorConfig::extrinsicRot;
    T_L_B_tmp.block<3,1>(0,3) = SensorConfig::extrinsicTrans;
    SensorConfig::T_L_B = T_L_B_tmp;

    Eigen::Matrix4d T_L_DR_tmp = Eigen::Matrix4d::Identity();
    T_L_DR_tmp.block<3,3>(0,0) = SensorConfig::extrinsicRot_DR;
    T_L_DR_tmp.block<3,1>(0,3) = SensorConfig::extrinsicTrans_DR;
    SensorConfig::T_L_DR = T_L_DR_tmp;

    SensorConfig::extrinsicQRPY = Eigen::Quaterniond(SensorConfig::extrinsicRPY);
    Eigen::Quaterniond q_sensor_body(SensorConfig::extrinsicRPY);
    Eigen::Vector3d t_sensor_body = SensorConfig::extrinsicTrans;
    SensorConfig::q_body_sensor = q_sensor_body.inverse();
    SensorConfig::t_body_sensor = -(q_sensor_body.inverse() * t_sensor_body);
    SensorConfig::gtsamGNSSBetweenFactorDistance = sensorconfig["gtsamGNSSBetweenFactorDistance"].as<int>();
    SensorConfig::lidarScanDownSample = sensorconfig["lidarScanDownSample"].as<int>();
    std::cout<<"SensorConfig::lidarScanDownSample: "<<SensorConfig::lidarScanDownSample<<std::endl;
    std::cout<<"SensorConfig::lidarMaxRing: "<<SensorConfig::lidarMaxRing<<std::endl;
    std::cout<<"SensorConfig::lidarMinRing: "<<SensorConfig::lidarMinRing<<std::endl;
    std::cout<<"SensorConfig::use_gnss_deskew: "<<SensorConfig::use_drodom_deskew<<std::endl;
    std::cout<<"SensorConfig::T_L_DR : "<<std::endl;
    std::cout<< SensorConfig::T_L_DR << std::endl;
    std::cout<<"SensorConfig::sensorconfig yaml success load"<<std::endl;

}

void Load_Mapping_YAML(std::string mappingpath)
{
        YAML::Node mappingconfig;
        try{
            mappingconfig = YAML::LoadFile(mappingpath);
        } catch(YAML::BadFile &e) {
            std::cout << "mapping yaml read error!" << mappingpath << std::endl;
            exit(1);
        }

        MappingConfig::slam_mode_switch = mappingconfig["slam_mode_switch"].as<int>();

        MappingConfig::save_map_path = mappingconfig["save_map_path"].as<std::string>();

        // LOAM feature threshold
        MappingConfig::edgeThreshold=mappingconfig["edgeThreshold"].as<float >();
        MappingConfig::surfThreshold=mappingconfig["surfThreshold"].as<float >();
        MappingConfig::edgeFeatureMinValidNum=mappingconfig["edgeFeatureMinValidNum"].as<int >();
        MappingConfig::surfFeatureMinValidNum=mappingconfig["surfFeatureMinValidNum"].as<int >();

        //voxel filter paprams
        MappingConfig::DownSampleModeSwitch = mappingconfig["DownSampleModeSwitch"].as<int>();
        MappingConfig::odometrySurfLeafSize=mappingconfig["odometrySurfLeafSize"].as<float >();
        MappingConfig::mappingCornerLeafSize=mappingconfig["mappingCornerLeafSize"].as<float >();
        MappingConfig::mappingSurfLeafSize=mappingconfig["mappingSurfLeafSize"].as<float >();

        MappingConfig::odometrySurfRadiusSize_US = mappingconfig["odometrySurfRadiusSize_US"].as<float >();
        MappingConfig::mappingCornerRadiusSize_US = mappingconfig["mappingCornerRadiusSize_US"].as<float >();
        MappingConfig::mappingSurfRadiusSize_US = mappingconfig["mappingSurfRadiusSize_US"].as<float >();
        MappingConfig::surroundingKeyframeDensity_US = mappingconfig["surroundingKeyframeDensity_US"].as<float >();
//        std::cout<<MappingConfig::mappingSurfLeafSize<<std::endl;

        // robot motion constraint (in case you are using a 2D robot)
        MappingConfig::z_tollerance=mappingconfig["z_tollerance"].as<float >();
        MappingConfig::rotation_tollerance=mappingconfig["rotation_tollerance"].as<float >();
//        std::cout<<MappingConfig::rotation_tollerance<<std::endl;

        //CPU Params
        MappingConfig::mappingProcessInterval=mappingconfig["mappingProcessInterval"].as<float >();

        //Surrounding map
        MappingConfig::surroundingkeyframeAddingDistThreshold=mappingconfig["surroundingkeyframeAddingDistThreshold"].as<double >();
        MappingConfig::surroundingkeyframeAddingAngleThreshold=mappingconfig["surroundingkeyframeAddingAngleThreshold"].as<double >();
        MappingConfig::surroundingKeyframeDensity=mappingconfig["surroundingKeyframeDensity"].as<double >();
        MappingConfig::surroundingKeyframeSearchRadius=mappingconfig["surroundingKeyframeSearchRadius"].as<double >();
//        std::cout<<MappingConfig::surroundingKeyframeSearchRadius<<std::endl;

        //Loop closure
        MappingConfig::loopClosureEnableFlag=mappingconfig["loopClosureEnableFlag"].as<bool >();
        MappingConfig::loopClosureFrequency=mappingconfig["loopClosureFrequency"].as<float >();
        MappingConfig::surroundingKeyframeSize=mappingconfig["surroundingKeyframeSize"].as<int >();
        MappingConfig::historyKeyframeSearchRadius=mappingconfig["historyKeyframeSearchRadius"].as<float >();
        MappingConfig::historyKeyframeSearchTimeDiff=mappingconfig["historyKeyframeSearchTimeDiff"].as<float >();
        MappingConfig::historyKeyframeSearchNum=mappingconfig["historyKeyframeSearchNum"].as<int >();
        MappingConfig::historyKeyframeFitnessScore=mappingconfig["historyKeyframeFitnessScore"].as<float >();

        MappingConfig::GNSS_noise_roll = mappingconfig["GNSS_noise_roll"].as<double>();
        MappingConfig::GNSS_noise_pitch = mappingconfig["GNSS_noise_pitch"].as<double>();
        MappingConfig::GNSS_noise_yaw = mappingconfig["GNSS_noise_yaw"].as<double>();
        MappingConfig::GNSS_noise_x = mappingconfig["GNSS_noise_x"].as<double>();
        MappingConfig::GNSS_noise_y = mappingconfig["GNSS_noise_y"].as<double>();
        MappingConfig::GNSS_noise_z = mappingconfig["GNSS_noise_z"].as<double>();

        MappingConfig::LidarOdom_noise_roll = mappingconfig["LidarOdom_noise_roll"].as<double>();
        MappingConfig::LidarOdom_noise_pitch = mappingconfig["LidarOdom_noise_pitch"].as<double>();
        MappingConfig::LidarOdom_noise_yaw = mappingconfig["LidarOdom_noise_yaw"].as<double>();
        MappingConfig::LidarOdom_noise_x = mappingconfig["LidarOdom_noise_x"].as<double>();
        MappingConfig::LidarOdom_noise_y = mappingconfig["LidarOdom_noise_y"].as<double>();
        MappingConfig::LidarOdom_noise_z = mappingconfig["LidarOdom_noise_z"].as<double>();

        MappingConfig::DR_noise_roll = mappingconfig["DR_noise_roll"].as<double>();
        MappingConfig::DR_noise_pitch = mappingconfig["DR_noise_pitch"].as<double>();
        MappingConfig::DR_noise_yaw = mappingconfig["DR_noise_yaw"].as<double>();
        MappingConfig::DR_noise_x = mappingconfig["DR_noise_x"].as<double>();
        MappingConfig::DR_noise_y = mappingconfig["DR_noise_y"].as<double>();
        MappingConfig::DR_noise_z = mappingconfig["DR_noise_z"].as<double>();

        MappingConfig::GNSSupdateDR = mappingconfig["GNSSupdateDR"].as<int>();

        MappingConfig::loop_noise_roll = mappingconfig["loop_noise_roll"].as<double>();
        MappingConfig::loop_noise_pitch = mappingconfig["loop_noise_pitch"].as<double>();
        MappingConfig::loop_noise_yaw = mappingconfig["loop_noise_yaw"].as<double>();
        MappingConfig::loop_noise_x = mappingconfig["loop_noise_x"].as<double>();
        MappingConfig::loop_noise_y = mappingconfig["loop_noise_y"].as<double>();
        MappingConfig::loop_noise_z = mappingconfig["loop_noise_z"].as<double>();


        std::cout<<"MappingConfig::mappingProcessInterval"<<MappingConfig::mappingProcessInterval<<std::endl;
        std::cout<<"MappingConfig::DownSampleModeSwitch: "<< MappingConfig::DownSampleModeSwitch<<std::endl;

        std::cout<<"MappingConfig::odometrySurfRadiusSize_US: "<<MappingConfig::odometrySurfRadiusSize_US<<std::endl;
        std::cout<<"MappingConfig::mappingCornerRadiusSize_US: "<<MappingConfig::mappingCornerRadiusSize_US<<std::endl;
        std::cout<<"MappingConfig::mappingSurfRadiusSize_US: "<< MappingConfig::mappingSurfRadiusSize_US<<std::endl;
        std::cout<<"MappingConfig::surroundingKeyframeDensity_US: "<< MappingConfig::surroundingKeyframeDensity_US<<std::endl;
       // std::cout<<"MappingConfig::use_DR_or_fuse_in_loc: "<<MappingConfig::use_DR_or_fuse_in_loc<<std::endl;
        std::cout<<"mapping yaml success load"<<std::endl;

}//end function Load_Mapping_YAML
void Load_Loc_YAML(std::string locPath){
    YAML::Node LocConfig;
    try{
        LocConfig = YAML::LoadFile(locPath);
    } catch(YAML::BadFile &e) {
        std::cout << "loc yaml read error!" << locPath << std::endl;
        exit(1);
    }
    LocConfig::slam_mode_on = LocConfig["slam_mode_on"].as<int>();
    LocConfig::save_map_path = LocConfig["save_map_path"].as<std::string>();
    LocConfig::edgeFeatureMinValidNum=LocConfig["edgeFeatureMinValidNum"].as<int >();
    LocConfig::surfFeatureMinValidNum=LocConfig["surfFeatureMinValidNum"].as<int >();
    LocConfig::mappingCornerRadiusSize_US = LocConfig["mappingCornerRadiusSize_US"].as<float >();
    LocConfig::mappingSurfRadiusSize_US = LocConfig["mappingSurfRadiusSize_US"].as<float >();
    LocConfig::edgeThreshold = LocConfig["edgeThreshold"].as<float>();
    LocConfig::surfThreshold = LocConfig["surfThreshold"].as<float>();
    LocConfig::z_tollerance=LocConfig["z_tollerance"].as<float >();
    LocConfig::rotation_tollerance=LocConfig["rotation_tollerance"].as<float >();
    LocConfig::mappingProcessInterval=LocConfig["mappingProcessInterval"].as<float >();
    LocConfig::surroundingkeyframeAddingDistThreshold=LocConfig["surroundingkeyframeAddingDistThreshold"].as<double >();
    LocConfig::surroundingkeyframeAddingAngleThreshold=LocConfig["surroundingkeyframeAddingAngleThreshold"].as<double >();
    LocConfig::use_DR_or_fuse_in_loc = LocConfig["use_DR_or_fuse_in_loc"].as<int>();
    LocConfig::maxIters =  LocConfig["maxIters"].as<int>();
    LocConfig::GNSS_noise_roll = LocConfig["GNSS_noise_roll"].as<double>();
    LocConfig::GNSS_noise_pitch = LocConfig["GNSS_noise_pitch"].as<double>();
    LocConfig::GNSS_noise_yaw = LocConfig["GNSS_noise_yaw"].as<double>();
    LocConfig::GNSS_noise_x = LocConfig["GNSS_noise_x"].as<double>();
    LocConfig::GNSS_noise_y = LocConfig["GNSS_noise_y"].as<double>();
    LocConfig::GNSS_noise_z = LocConfig["GNSS_noise_z"].as<double>();
    LocConfig::LidarOdom_noise_x = LocConfig["LidarOdom_noise_x"].as<double>();
    LocConfig::LidarOdom_noise_y = LocConfig["LidarOdom_noise_y"].as<double>();
    LocConfig::LidarOdom_noise_z = LocConfig["LidarOdom_noise_z"].as<double>();
    LocConfig::LidarOdom_noise_roll = LocConfig["LidarOdom_noise_roll"].as<double>();
    LocConfig::LidarOdom_noise_pitch = LocConfig["LidarOdom_noise_pitch"].as<double>();
    LocConfig::LidarOdom_noise_yaw = LocConfig["LidarOdom_noise_yaw"].as<double>();
    LocConfig::DR_noise_x = LocConfig["DR_noise_x"].as<double>();
    LocConfig::DR_noise_y = LocConfig["DR_noise_y"].as<double>();
    LocConfig::DR_noise_z = LocConfig["DR_noise_z"].as<double>();
    LocConfig::DR_noise_roll = LocConfig["DR_noise_roll"].as<double>();
    LocConfig::DR_noise_pitch = LocConfig["DR_noise_pitch"].as<double>();
    LocConfig::DR_noise_yaw = LocConfig["DR_noise_yaw"].as<double>();
    LocConfig::GNSSupdateDR = LocConfig["GNSSupdateDR"].as<int>();

    std::cout<<"LocConfig::mappingProcessInterval"<<LocConfig::mappingProcessInterval<<std::endl;
    std::cout<<"LocConfig::odometrySurfRadiusSize_US: "<<LocConfig::odometrySurfRadiusSize_US<<std::endl;
    std::cout<<"LocConfig::mappingCornerRadiusSize_US: "<<LocConfig::mappingCornerRadiusSize_US<<std::endl;
    std::cout<<"LocConfig::mappingSurfRadiusSize_US: "<< LocConfig::mappingSurfRadiusSize_US<<std::endl;
    std::cout<<"LocConfig::use_DR_or_fuse_in_loc: "<<LocConfig::use_DR_or_fuse_in_loc<<std::endl;
    std::cout<<"LocConfig::surroundingkeyframeAddingAngleThreshold: "<<LocConfig::surroundingkeyframeAddingAngleThreshold<<std::endl;
    std::cout<<"LocConfig::surroundingkeyframeAddingDistThreshold: "<<LocConfig::surroundingkeyframeAddingDistThreshold<<std::endl;
    std::cout<<"LocConfig::maxIters: "<<LocConfig::maxIters<<std::endl;
    std::cout<<"LocConfig::DR_noise_roll: "<<LocConfig::DR_noise_roll<<std::endl;
    std::cout<<"LocConfig::DR_noise_pitch: "<<LocConfig::DR_noise_pitch<<std::endl;
    std::cout<<"LocConfig::DR_noise_yaw: "<<LocConfig::DR_noise_yaw<<std::endl;
    std::cout<<"LocConfig::DR_noise_x: "<<LocConfig::DR_noise_x<<std::endl;
    std::cout<<"LocConfig::DR_noise_y: "<<LocConfig::DR_noise_y<<std::endl;
    std::cout<<"LocConfig::DR_noise_z: "<<LocConfig::DR_noise_z<<std::endl;
    std::cout<<"LocConfig::LidarOdom_noise_roll: "<<LocConfig::LidarOdom_noise_roll<<std::endl;
    std::cout<<"LocConfig::LidarOdom_noise_pitch: "<<LocConfig::LidarOdom_noise_pitch<<std::endl;
    std::cout<<"LocConfig::LidarOdom_noise_yaw: "<<LocConfig::LidarOdom_noise_yaw<<std::endl;
    std::cout<<"LocConfig::LidarOdom_noise_x: "<<LocConfig::LidarOdom_noise_x<<std::endl;
    std::cout<<"LocConfig::LidarOdom_noise_y: "<<LocConfig::LidarOdom_noise_y<<std::endl;
    std::cout<<"LocConfig::LidarOdom_noise_z: "<<LocConfig::LidarOdom_noise_z<<std::endl;
    std::cout<<"LocConfig::GNSS_noise_roll: "<<LocConfig::GNSS_noise_roll<<std::endl;
    std::cout<<"LocConfig::GNSS_noise_pitch: "<<LocConfig::GNSS_noise_pitch<<std::endl;
    std::cout<<"LocConfig::GNSS_noise_yaw: "<<LocConfig::GNSS_noise_yaw<<std::endl;
    std::cout<<"LocConfig::GNSS_noise_x: "<<LocConfig::GNSS_noise_x<<std::endl;
    std::cout<<"LocConfig::GNSS_noise_y: "<<LocConfig::GNSS_noise_y<<std::endl;
    std::cout<<"LocConfig::GNSS_noise_z: "<<LocConfig::GNSS_noise_z<<std::endl;
    std::cout<<"LocConfig::GNSSupdateDR: "<<LocConfig::GNSSupdateDR<<std::endl;


    std::cout<<"Loc yaml success load"<<std::endl;
}
void Load_offline_YAML(std::string offlinepath)
    {
        YAML::Node offlineconfig;
        try{
            offlineconfig = YAML::LoadFile(offlinepath);
        } catch(YAML::BadFile &e) {
            std::cout<<"offline yaml read error!"<<offlinepath<<std::endl;
            exit(1);
        }

        SerializeConfig::map_in_path = offlineconfig["map_in_path"].as<std::string>();
        SerializeConfig::map_out_path = offlineconfig["map_out_path"].as<std::string>();
        SerializeConfig::up2down_num = offlineconfig["up2down_num"].as<int>();
        SerializeConfig::lidar_range = offlineconfig["lidar_range"].as<double>();
        SerializeConfig::frame_sum = offlineconfig["frame_sum"].as<int>();

        SerializeConfig::up_grid_size= offlineconfig["up_grid_size"].as<int>();
        SerializeConfig::lasercloud_width= offlineconfig["lasercloud_width"].as<int>();
        SerializeConfig::lasercloud_height= offlineconfig["lasercloud_height"].as<int>();
        SerializeConfig::lasercloud_num= offlineconfig["lasercloud_num"].as<int>();

        std::cout<<"offline yaml success load"<<std::endl;

    }

void Load_FrontEnd_YAML(std::string frontendpath)
{
    YAML::Node frontendconfig;
    try{
        frontendconfig = YAML::LoadFile(frontendpath);
    } catch(YAML::BadFile &e) {
        std::cout<<"front_end yaml read error!"<<frontendpath<<std::endl;
        exit(1);
    }
    FrontEndConfig::slam_mode_switch = frontendconfig["slam_mode_switch"].as<int>();
    FrontEndConfig::if_debug = frontendconfig["if_debug"].as<int>();
// ground filter
    FrontEndConfig::use_ground_filter = frontendconfig["use_ground_filter"].as<bool>();
    FrontEndConfig::min_grid_pt_num = frontendconfig["min_grid_pt_num"].as<int>();
//    FrontEndConfig::max_ground_height = frontendconfig["max_ground_height"].as<float>();
    FrontEndConfig::grid_resolution  = frontendconfig["grid_resolution"].as<float>();
    FrontEndConfig::distance_weight_downsampling_method  = frontendconfig["distance_weight_downsampling_method"].as<int>();
    FrontEndConfig::standard_distance = frontendconfig["standard_distance"].as<float>();
    FrontEndConfig::nonground_random_down_rate = frontendconfig["nonground_random_down_rate"].as<int>();
    FrontEndConfig::intensity_thre = frontendconfig["intensity_thre"].as<float>();
    FrontEndConfig::apply_grid_wise_outlier_filter = frontendconfig["apply_grid_wise_outlier_filter"].as<bool>();
    FrontEndConfig::outlier_std_scale = frontendconfig["outlier_std_scale"].as<float>();
    FrontEndConfig::reliable_neighbor_grid_num_thre  = frontendconfig["reliable_neighbor_grid_num_thre"].as<int>();
    FrontEndConfig::ground_random_down_rate  = frontendconfig["ground_random_down_rate"].as<int>();
    FrontEndConfig::neighbor_height_diff  = frontendconfig["neighbor_height_diff"].as<float>();
    FrontEndConfig::max_height_difference  = frontendconfig["max_height_difference"].as<float>();
    FrontEndConfig::estimate_ground_normal_method  = frontendconfig["estimate_ground_normal_method"].as<int>();
    FrontEndConfig::normal_estimation_radius  = frontendconfig["normal_estimation_radius"].as<float>();
    FrontEndConfig::fixed_num_downsampling  = frontendconfig["fixed_num_downsampling"].as<bool>();
    FrontEndConfig::ground_random_down_down_rate = frontendconfig["ground_random_down_down_rate"].as<int>();

//unground pts classify
    FrontEndConfig::use_unground_pts_classify = frontendconfig["use_unground_pts_classify"].as<bool>();
    FrontEndConfig::neighbor_searching_radius = frontendconfig["neighbor_searching_radius"].as<float>();
    FrontEndConfig::neighbor_k = frontendconfig["neighbor_k"].as<int>();
    FrontEndConfig::neigh_k_min= frontendconfig["neigh_k_min"].as<int>();
    FrontEndConfig::pca_down_rate= frontendconfig["pca_down_rate"].as<int>();
    FrontEndConfig::edge_thre= frontendconfig["edge_thre"].as<float>();
    FrontEndConfig::planar_thre= frontendconfig["planar_thre"].as<float>();
    FrontEndConfig::edge_thre_down= frontendconfig["edge_thre_down"].as<float>();
    FrontEndConfig::planar_thre_down= frontendconfig["planar_thre_down"].as<float>();
    FrontEndConfig::extract_vertex_points_method= frontendconfig["extract_vertex_points_method"].as<int>();
    FrontEndConfig::curvature_thre = frontendconfig["curvature_thre"].as<float>();
    FrontEndConfig::vertex_curvature_non_max_radius = frontendconfig["vertex_curvature_non_max_radius"].as<float>();
    FrontEndConfig::linear_vertical_sin_high_thre = frontendconfig["linear_vertical_sin_high_thre"].as<float>();
    FrontEndConfig::linear_vertical_sin_low_thre= frontendconfig["linear_vertical_sin_low_thre"].as<float>();
    FrontEndConfig::planar_vertical_sin_high_thre = frontendconfig["planar_vertical_sin_high_thre"].as<float>();
    FrontEndConfig::planar_vertical_sin_low_thre= frontendconfig["planar_vertical_sin_low_thre"].as<float>();
    FrontEndConfig::pillar_down_fixed_num = frontendconfig["pillar_down_fixed_num"].as<int>();
    FrontEndConfig::facade_down_fixed_num = frontendconfig["facade_down_fixed_num"].as<int>();
    FrontEndConfig::beam_down_fixed_num = frontendconfig["beam_down_fixed_num"].as<int>();
    FrontEndConfig::roof_down_fixed_num = frontendconfig["roof_down_fixed_num"].as<int>();
    FrontEndConfig::unground_down_fixed_num = frontendconfig["unground_down_fixed_num"].as<int>();
//    FrontEndConfig::beam_height_max = frontendconfig["beam_height_max"].as<float>();
//    FrontEndConfig::roof_height_min = frontendconfig["roof_height_min"].as<float>();
    FrontEndConfig::feature_pts_ratio_guess = frontendconfig["feature_pts_ratio_guess"].as<float>();
    FrontEndConfig::sharpen_with_nms = frontendconfig["sharpen_with_nms"].as<bool>();
    FrontEndConfig::use_distance_adaptive_pca = frontendconfig["use_distance_adaptive_pca"].as<bool>();
    FrontEndConfig:: gnss_pop_threshold=frontendconfig["gnss_pop_threshold"].as<float >();
    FrontEndConfig:: gnss_align_threshold=frontendconfig["gnss_align_threshold"].as<float >();

    std::cout<<"offline yaml success load"<<std::endl;

} // end Load_FrontEnd_YAML

void Load_Udp_YAML(std::string udppath)
{
    YAML::Node udpconfig;
    try{
        udpconfig = YAML::LoadFile(udppath);
    } catch(YAML::BadFile &e) {
        std::cout<<"offline yaml read error!"<<udppath<<std::endl;
        exit(1);
    }
    UdpConfig::cleint_ip = udpconfig["serverIP"].as<std::string>();
    UdpConfig::clinet_port = udpconfig["clinetPort"].as<int>();
    UdpConfig::server_port = udpconfig["serverPort"].as<int>();
    std::cout<<"UdpConfig::server_port: "<<UdpConfig::server_port;
    std::cout<<"UdpConfig::clinet_port: "<<UdpConfig::clinet_port;
    std::cout<<"UdpConfig::cleint_ip: "<<UdpConfig::cleint_ip;
    std::cout<<"Load_Udp_YAML success! ";

}

#endif