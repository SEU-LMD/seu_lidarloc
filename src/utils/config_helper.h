
//created by rongxuan && xiaoqiang at 2023.0819 peaceful night
#ifndef SEU_CONFIG_HELPER
#define SEU_CONFIG_HELPER


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
    static std::string imuTopic;
    static  std::string  odomTopic;
    static  std::string  gpsTopic;
    static int imuHZ;
    //Frame
    static std::string  lidarFrame;
    static std::string  baselinkFrame;
    static std::string  odometryFrame;
    static std::string  mapFrame;

    //GPS Setting
    static bool  useGPS;
    static bool   updateOrigin;//没有使用
    static int  gpsFrequence;//notuse
    static bool  useImuHeadingInitialization;
    static bool useGpsElevation;
    static float  gpsCovThreshold;
    static float   poseCovThreshold;
    static float  gpsDistance;//not use

    //debu setting
    static bool debugLidarTimestamp;
    static bool debugImu;
    static bool  debugGps;//not use

    //Lidar settings
    static std::string sensor; //bug*********sensortype
    static int N_SCAN;
    static int  Horizon_SCAN;
    static int  downsampleRate;
    static float  lidarMinRange;
    static float lidarMaxRange;
    static int lidarMinRing;
    static int lidarMaxRing;
    static int LMOpt_Cnt;

    //IMU Settings
    static double imuAccNoise;
    static double imuGyrNoise;
    static double  imuAccBiasN;
    static double  imuGyrBiasN;
    static double  imuGravity;
    static double imuRPYWeight;
    static int imuGTSAMReset;

    //Extrinsics (lidar -> IMU)
    static int imu_type;//meiyong buyiyang********
    static Eigen::Vector3d  extrinsicTrans;//
    static Eigen::Matrix3d extrinsicRot;//
    static Eigen::Matrix4d T_L_B;
    static Eigen::Matrix3d extrinsicRPY;//  **meiyong
    static Eigen::Quaterniond extrinsicQRPY;//not
    static Eigen::Vector3d t_body_sensor;
    static Eigen::Quaterniond q_body_sensor;

    static bool use_gnss_deskew;
    static int gtsamGNSSBetweenFactorDistance;
    static int lidarScanDownSample;
};

class MappingConfig{
    public:
        static int slam_mode_switch;
        static int if_debug;
        static Eigen::Vector3d origin_gnss;
        static bool use_deskew;
        // save map
        static std::string save_map_path;

        //Export settings
        static bool  savePCD;
        static std::string savePCDDirectory;

        // LOAM feature threshold
        static float  edgeThreshold;
        static float surfThreshold;
        static int  edgeFeatureMinValidNum;
        static int  surfFeatureMinValidNum;

        //voxel filter paprams
        static float odometrySurfLeafSize;
        static float mappingCornerLeafSize;
        static float  mappingSurfLeafSize;

        // robot motion constraint (in case you are using a 2D robot)
        static float z_tollerance;
        static float rotation_tollerance;

        //CPU Params
        static int numberOfCores;
        static float mappingProcessInterval;

        //Surrounding map
        static double  surroundingkeyframeAddingDistThreshold;
        static double  surroundingkeyframeAddingAngleThreshold;
        static double surroundingKeyframeDensity;
        static double surroundingKeyframeSearchRadius;
        static double localMap_searchRadius;
        static double localMap_searchRadius_surf;
        static double localMap_searchRadius_corner;
        static int LocalMap_updata_perframe;
        static int scan_2_prior_map;
        static std::string prior_map_surf;
        static std::string prior_map_corner;

        //Loop closure
        static bool  loopClosureEnableFlag;
        static float  loopClosureFrequency;
        static int surroundingKeyframeSize;
        static float historyKeyframeSearchRadius;
        static float historyKeyframeSearchTimeDiff;
        static int historyKeyframeSearchNum;
        static float  historyKeyframeFitnessScore;//

        // Visualization
        static float globalMapVisualizationSearchRadius;
        static float  globalMapVisualizationPoseDensity;
        static float globalMapVisualizationLeafSize;

        //mapping
        static float globalMapLeafSize;//meiyong
        static int scan_2_scan_num_corner;
        static int scan_2_scan_num_surf;
    };


class SerializeConfig{
   public:
      static std::string map_in_path;
      static std::string map_out_path;
      static int up2down_num;
      static double lidar_range;
      static int frame_sum;

      static std::string current_lidar_path;
      static std::string prior_map_path;
      static std::string prior_pose_path;
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

std::string SensorConfig::pointCloudTopic = "";
std::string SensorConfig::imuTopic = "";
std::string  SensorConfig::odomTopic = "";
std::string  SensorConfig::gpsTopic = "";


//Frame
std::string  SensorConfig::lidarFrame="";
std::string SensorConfig::baselinkFrame="";
std::string  SensorConfig::odometryFrame="";
std::string  SensorConfig::mapFrame="";

//GPS Setting
Eigen::Matrix4d SensorConfig::T_L_B = Eigen::Matrix4d::Identity();
bool  SensorConfig::useGPS=false;
bool   SensorConfig::updateOrigin=false;
int  SensorConfig::gpsFrequence=-1;
bool  SensorConfig::useImuHeadingInitialization=false;
bool SensorConfig::useGpsElevation=false;
float  SensorConfig::gpsCovThreshold=-1;
float  SensorConfig::poseCovThreshold=-1;
float  SensorConfig::gpsDistance=-1;

//debu setting
bool SensorConfig::debugLidarTimestamp=false;
bool SensorConfig::debugImu=false;
bool  SensorConfig::debugGps=false;

//Export settings
std::string SensorConfig::sensor="";
int SensorConfig::N_SCAN=-1;
int  SensorConfig::Horizon_SCAN=-1;
int  SensorConfig::downsampleRate=-1;
float  SensorConfig::lidarMinRange=-1;
float SensorConfig::lidarMaxRange=-1;
int SensorConfig::lidarMinRing=-1;
int SensorConfig::lidarMaxRing=-1;
int SensorConfig::LMOpt_Cnt = 10;


    //IMU Settings
double SensorConfig::imuAccNoise=-1;
double SensorConfig::imuGyrNoise=-1;
double  SensorConfig::imuAccBiasN=-1;
double  SensorConfig::imuGyrBiasN=-1;
double SensorConfig:: imuGravity=-1;
double SensorConfig::imuRPYWeight=-1;
int  SensorConfig::imuHZ = -1;
int SensorConfig::imuGTSAMReset = 25;


int SensorConfig::imu_type=-1;
Eigen::Vector3d SensorConfig::extrinsicTrans;
Eigen::Matrix3d SensorConfig::extrinsicRot;
Eigen::Matrix3d SensorConfig::extrinsicRPY;
Eigen::Quaterniond SensorConfig::extrinsicQRPY;
Eigen::Vector3d SensorConfig::t_body_sensor;
Eigen::Quaterniond SensorConfig::q_body_sensor;
int SensorConfig::gtsamGNSSBetweenFactorDistance = 10;

bool SensorConfig::use_gnss_deskew=false;
int SensorConfig::lidarScanDownSample = 2;

int MappingConfig::slam_mode_switch = 1;
int MappingConfig::if_debug = 1;
std::string MappingConfig::save_map_path = "";

Eigen::Vector3d MappingConfig::origin_gnss = Eigen::Vector3d(0,0,0);
bool  MappingConfig::savePCD=false;
std::string  MappingConfig::savePCDDirectory="";
std::string MappingConfig::prior_map_surf = " ";
std::string MappingConfig::prior_map_corner = " ";

// LOAM feature threshold
float MappingConfig::edgeThreshold=-1;
bool MappingConfig::use_deskew = false;
float MappingConfig::surfThreshold=-1;
int  MappingConfig::edgeFeatureMinValidNum=-1;
int  MappingConfig::surfFeatureMinValidNum=-1;

//voxel filter paprams
float MappingConfig::odometrySurfLeafSize=-1;
float MappingConfig::mappingCornerLeafSize=-1;
float MappingConfig::mappingSurfLeafSize=-1;

// robot motion constraint (in case you are using a 2D robot)
float MappingConfig::z_tollerance=-1;
float MappingConfig::rotation_tollerance=-1;

//CPU Params
int MappingConfig::numberOfCores=-1;
float MappingConfig::mappingProcessInterval=-1;

//Surrounding map
double  MappingConfig::surroundingkeyframeAddingDistThreshold=-1;
double  MappingConfig::surroundingkeyframeAddingAngleThreshold=-1;
double MappingConfig::surroundingKeyframeDensity=-1;
double MappingConfig::surroundingKeyframeSearchRadius=-1;
int MappingConfig::scan_2_prior_map = 1;
double MappingConfig::localMap_searchRadius=-1;
double MappingConfig::localMap_searchRadius_surf=-1;
double MappingConfig::localMap_searchRadius_corner=-1;
int MappingConfig::LocalMap_updata_perframe=-1;

//Loop closure
bool  MappingConfig::loopClosureEnableFlag=false;
float  MappingConfig::loopClosureFrequency=-1;
int MappingConfig::surroundingKeyframeSize=-1;
float MappingConfig::historyKeyframeSearchRadius=-1;
float MappingConfig::historyKeyframeSearchTimeDiff=-1;
int MappingConfig::historyKeyframeSearchNum=-1;
float MappingConfig::historyKeyframeFitnessScore=-1;

// Visualization
float MappingConfig::globalMapVisualizationSearchRadius=-1;
float  MappingConfig::globalMapVisualizationPoseDensity=-1;
float MappingConfig::globalMapVisualizationLeafSize=-1;

//mapping
float MappingConfig::globalMapLeafSize=-1;
int  MappingConfig::scan_2_scan_num_surf = 1;
int  MappingConfig::scan_2_scan_num_corner = 1;

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
std::string SerializeConfig::current_lidar_path = "";
std::string SerializeConfig::prior_map_path = "";
std::string SerializeConfig::prior_pose_path = "";

double SerializeConfig::Tepsilion = 0.01;
double SerializeConfig::step_size = 1.0;
float SerializeConfig::size_resolution = 10.0;
double SerializeConfig::max_inter_num = 35;
double SerializeConfig::setLeafSize = 0.6;
double SerializeConfig::sequence_num = 271;



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
    SensorConfig::imuTopic = sensorconfig["imuTopic"].as<std::string>();
    SensorConfig::odomTopic = sensorconfig["odomTopic"].as<std::string>();
    SensorConfig::gpsTopic = sensorconfig["gpsTopic"].as<std::string>();

//     //Frame
    SensorConfig::lidarFrame=sensorconfig["lidarFrame"].as<std::string>();
    SensorConfig::baselinkFrame=sensorconfig["baselinkFrame"].as<std::string>();
    SensorConfig::odometryFrame=sensorconfig["odometryFrame"].as<std::string>();
    SensorConfig::mapFrame=sensorconfig["mapFrame"].as<std::string>();
//    std::cout<<SensorConfig:: mapFrame<<std::endl;

//     //GPS Setting
    SensorConfig::useGPS=sensorconfig["useGPS"].as<bool>();
    SensorConfig::updateOrigin=sensorconfig["updateOrigin"].as<bool>();
    SensorConfig::gpsFrequence=sensorconfig["gpsFrequence"].as<int >();
    SensorConfig::useImuHeadingInitialization=sensorconfig["useImuHeadingInitialization"].as<bool>();
    SensorConfig::useGpsElevation=sensorconfig["useGpsElevation"].as<bool>();
    SensorConfig::gpsCovThreshold=sensorconfig["gpsCovThreshold"].as<float >();
    SensorConfig::poseCovThreshold=sensorconfig["poseCovThreshold"].as<float >();
    SensorConfig::gpsDistance=sensorconfig["gpsDistance"].as<float >();
//    std::cout<<SensorConfig::gpsDistance<<std::endl;

    // //debu setting
    SensorConfig::debugLidarTimestamp=sensorconfig["debugLidarTimestamp"].as<bool >();
    SensorConfig::debugImu=sensorconfig["debugImu"].as<bool >();
    SensorConfig::debugGps=sensorconfig["debugGps"].as<bool >();
//    std::cout<<SensorConfig::debugGps<<std::endl;

    SensorConfig::sensor=sensorconfig["sensor"].as<std::string >();
    SensorConfig::N_SCAN=sensorconfig["N_SCAN"].as<int >();
    SensorConfig::Horizon_SCAN=sensorconfig["Horizon_SCAN"].as<int >();
    SensorConfig::downsampleRate=sensorconfig["downsampleRate"].as<int >();
    SensorConfig::lidarMinRange=sensorconfig["lidarMinRange"].as<float >();
    SensorConfig::lidarMaxRange=sensorconfig["lidarMaxRange"].as<float >();
    SensorConfig::lidarMinRing=sensorconfig["lidarMinRing"].as<int>();
    SensorConfig::lidarMaxRing=sensorconfig["lidarMaxRing"].as<int>();
    SensorConfig::LMOpt_Cnt=sensorconfig["LMOpt_Cnt"].as<int>();
//    std::cout<<SensorConfig::lidarMaxRange<<std::endl;


    //IMU Settings
    SensorConfig:: imuAccNoise=sensorconfig["imuAccNoise"].as<double >();
    SensorConfig:: imuGyrNoise=sensorconfig["imuGyrNoise"].as<double >();
    SensorConfig::  imuAccBiasN=sensorconfig["imuAccBiasN"].as<double >();
    SensorConfig::  imuGyrBiasN=sensorconfig["imuGyrBiasN"].as<double >();
    SensorConfig::  imuGravity=sensorconfig["imuGravity"].as<double >();
    SensorConfig::  imuRPYWeight=sensorconfig["imuRPYWeight"].as<double >();
    SensorConfig::imuHZ = sensorconfig["imuHZ"].as<int >();
    SensorConfig::use_gnss_deskew=sensorconfig["use_gnss_deskew"].as<bool >();
    SensorConfig::imuGTSAMReset=sensorconfig["imuGTSAMReset"].as<int>();

//    std::cout<<SensorConfig::imuRPYWeight<<std::endl;



    //Extrinsics (lidar -> IMU)
    SensorConfig::imu_type=sensorconfig["imu_type"].as<int >();
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

    Eigen::Matrix4d T_L_B_tmp = Eigen::Matrix4d::Identity();
    T_L_B_tmp.block<3,3>(0,0) = SensorConfig::extrinsicRot;
    T_L_B_tmp.block<3,1>(0,3) = SensorConfig::extrinsicTrans;
    SensorConfig::T_L_B = T_L_B_tmp;

    SensorConfig::extrinsicQRPY = Eigen::Quaterniond(SensorConfig::extrinsicRPY);
    Eigen::Quaterniond q_sensor_body(SensorConfig::extrinsicRPY);
    Eigen::Vector3d t_sensor_body = SensorConfig::extrinsicTrans;
    SensorConfig::q_body_sensor = q_sensor_body.inverse();
    SensorConfig::t_body_sensor = -(q_sensor_body.inverse() * t_sensor_body);
    SensorConfig::gtsamGNSSBetweenFactorDistance = sensorconfig["gtsamGNSSBetweenFactorDistance"].as<int>();
    SensorConfig::lidarScanDownSample = sensorconfig["lidarScanDownSample"].as<int>();
    std::cout<<"lidarScanDownSample: "<<SensorConfig::lidarScanDownSample<<std::endl;
    std::cout<<"lidarMaxRing: "<<SensorConfig::lidarMaxRing<<std::endl;
    std::cout<<"lidarMinRing: "<<SensorConfig::lidarMinRing<<std::endl;
    std::cout<<"sensorconfig yaml success load"<<std::endl;
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
        MappingConfig::if_debug = mappingconfig["if_debug"].as<int>();
        MappingConfig::use_deskew=mappingconfig["use_deskew"].as<bool >();

        MappingConfig::save_map_path = mappingconfig["save_map_path"].as<std::string>();
        // //Export settings
        MappingConfig::savePCD=mappingconfig["savePCD"].as<bool >();
        MappingConfig::savePCDDirectory=mappingconfig["savePCDDirectory"].as<std::string >();

        // LOAM feature threshold
        MappingConfig::edgeThreshold=mappingconfig["edgeThreshold"].as<float >();
        MappingConfig::surfThreshold=mappingconfig["surfThreshold"].as<float >();
        MappingConfig::edgeFeatureMinValidNum=mappingconfig["edgeFeatureMinValidNum"].as<int >();
        MappingConfig::surfFeatureMinValidNum=mappingconfig["surfFeatureMinValidNum"].as<int >();
//        std::cout<<MappingConfig::surfFeatureMinValidNum<<std::endl;

        //voxel filter paprams
        MappingConfig::odometrySurfLeafSize=mappingconfig["odometrySurfLeafSize"].as<float >();
        MappingConfig::mappingCornerLeafSize=mappingconfig["mappingCornerLeafSize"].as<float >();
        MappingConfig::mappingSurfLeafSize=mappingconfig["mappingSurfLeafSize"].as<float >();
//        std::cout<<MappingConfig::mappingSurfLeafSize<<std::endl;

        // robot motion constraint (in case you are using a 2D robot)
        MappingConfig::z_tollerance=mappingconfig["z_tollerance"].as<float >();
        MappingConfig::rotation_tollerance=mappingconfig["rotation_tollerance"].as<float >();
//        std::cout<<MappingConfig::rotation_tollerance<<std::endl;

        //CPU Params
        MappingConfig::numberOfCores=mappingconfig["numberOfCores"].as<int >();
        MappingConfig::mappingProcessInterval=mappingconfig["mappingProcessInterval"].as<float >();
//        std::cout<<MappingConfig::mappingProcessInterval<<std::endl;

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
        MappingConfig::localMap_searchRadius_surf = mappingconfig["localMap_searchRadius_surf"].as<double >();
        MappingConfig::localMap_searchRadius_corner = mappingconfig["localMap_searchRadius_corner"].as<double >();
        MappingConfig::LocalMap_updata_perframe = mappingconfig["LocalMap_updata_perframe"].as<int>();
        MappingConfig::scan_2_prior_map = mappingconfig["scan_2_prior_map"].as<int>();
        MappingConfig::localMap_searchRadius = mappingconfig["localMap_searchRadius"].as<double >();
        MappingConfig::prior_map_corner = mappingconfig["prior_corner_map_path"].as<std::string>();
        MappingConfig::prior_map_surf = mappingconfig["prior_surf_map_path"].as<std::string>();

        // Visualization
        MappingConfig::globalMapVisualizationSearchRadius=mappingconfig["globalMapVisualizationSearchRadius"].as<float >();
        MappingConfig::globalMapVisualizationPoseDensity=mappingconfig["globalMapVisualizationPoseDensity"].as<float >();
        MappingConfig::globalMapVisualizationLeafSize=mappingconfig["globalMapVisualizationLeafSize"].as<float >();
//        std::cout<<MappingConfig::globalMapVisualizationLeafSize<<std::endl;


        //mapping
        MappingConfig:: globalMapLeafSize=mappingconfig["globalMapLeafSize"].as<float >();
//        std::cout<<MappingConfig::globalMapLeafSize<<std::endl;
        MappingConfig:: scan_2_scan_num_surf=mappingconfig["scan_2_scan_num_surf"].as<float >();
        MappingConfig:: scan_2_scan_num_corner=mappingconfig["scan_2_scan_num_corner"].as<float >();
//        std::cout<<MappingConfig::globalMapLeafSize<<std::endl;

        std::cout<<"prior_corner_map_path: "<<MappingConfig::prior_map_corner<<std::endl;
        std::cout<<"prior_surf_map_path: "<<MappingConfig::prior_map_surf<<std::endl;

        std::cout<<"mapping yaml success load"<<std::endl;

}//end function Load_Mapping_YAML

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

        //Prior Map localization
        SerializeConfig::current_lidar_path = offlineconfig["CURRENT_LIDAR_PATH"].as<std::string>();
        SerializeConfig::prior_map_path = offlineconfig["PRIOR_MAP_PATH"].as<std::string>();
        SerializeConfig::prior_pose_path = offlineconfig["PRIOR_POSE_PATH"].as<std::string>();

        std::cout<<"CURRENT_LIDAR_PATH: "<<SerializeConfig::current_lidar_path<<std::endl;
        std::cout<<"PRIOR_MAP_PATH: "<<SerializeConfig::prior_map_path<<std::endl;
        std::cout<<"PRIOR_POSE_PATH: "<<SerializeConfig::prior_pose_path<<std::endl;

//    ndt param
        SerializeConfig::Tepsilion = offlineconfig["T_EPSILION"].as<double>();
        SerializeConfig::step_size = offlineconfig["STEP_SIZE"].as<double>();
        SerializeConfig::size_resolution = offlineconfig["SIZE_RESOLUTION"].as<float>();
        SerializeConfig::max_inter_num = offlineconfig["MAXMUN_INTERNUM"].as<double>();
        SerializeConfig::setLeafSize = offlineconfig["DOWNSIZE_LEAF"].as<double>();
        SerializeConfig::sequence_num = offlineconfig["CURRENT_LIDAR_NUM"].as<double>();

        std::cout<<"T_EPSILION: "<<SerializeConfig::Tepsilion<<std::endl;
        std::cout<<"STEP_SIZE: "<<SerializeConfig::step_size<<std::endl;
        std::cout<<"SIZE_RESOLUTION: "<<SerializeConfig::size_resolution<<std::endl;
        std::cout<<"DOWNSIZE_LEAF: "<<SerializeConfig::setLeafSize<<std::endl;
        std::cout<<"MAXMUN_INTERNUM: "<<SerializeConfig::max_inter_num<<std::endl;
        std::cout<<"CURRENT_LIDAR_NUM: "<<SerializeConfig::sequence_num<<std::endl;
        SerializeConfig::map_out_path =offlineconfig["map_out_path"].as<std::string>();
        SerializeConfig::up2down_num = offlineconfig["up2down_num"].as<int>();
        SerializeConfig::lidar_range = offlineconfig["lidar_range"].as<double>();
        SerializeConfig::frame_sum = offlineconfig["frame_sum"].as<int>();
        SerializeConfig::up_grid_size= offlineconfig["up_grid_size"].as<int>();
        SerializeConfig::lasercloud_width= offlineconfig["lasercloud_width"].as<int>();
        SerializeConfig::lasercloud_height= offlineconfig["lasercloud_height"].as<int>();
        SerializeConfig::lasercloud_num= offlineconfig["lasercloud_num"].as<int>();

        std::cout<<"offline yaml success load"<<std::endl;

    }

#endif