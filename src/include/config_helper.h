
//created by rongxuan && xiaoqiang at 2023.0819 peaceful night
#ifndef CONFIG_HELPER
#define CONFIG_HELPER


#include <iostream>
#include <fstream>
#include<vector>
#include <Eigen/Dense>
#include <string>


#include "yaml-cpp/yaml.h"


namespace LidarType{
    std::string HESAI = "hesai";
    std::string VELODYNE = "velodyne";
    std::string OUSTER = "ouster";
    std::string LIVOX = "livox";
}

class Config{
    public:
    //Topics
    static std::string pointCloudTopic;
    static std::string imuTopic;
    static  std::string  odomTopic;
    static  std::string  gpsTopic;

    //Frame
    static std::string  lidarFrame;
    static std::string  baselinkFrame;
    static std::string  odometryFrame;
    static std::string  mapFrame;   

    //GPS Setting
    static bool  useGPS; //没有使用
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

    //Export settings
    static bool  savePCD;
    static std::string savePCDDirectory;

    static std::string sensor; //bug*********sensortype
    static int N_SCAN;
    static int  Horizon_SCAN;
    static int  downsampleRate;
    static float  lidarMinRange;
    static float lidarMaxRange;

    //IMU Settings
    static double imuAccNoise;
    static double imuGyrNoise;
    static double  imuAccBiasN;
    static double  imuGyrBiasN;
    static double  imuGravity;
    static double imuRPYWeight;

    //Extrinsics (lidar -> IMU)
    static int imu_type;//meiyong buyiyang********
    static Eigen::Vector3d  extrinsicTrans;
    static Eigen::Matrix3d extrinsicRot;
    static Eigen::Matrix3d extrinsicRPY;//  **meiyong 

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


        
};

std::string Config::pointCloudTopic = "";
std::string Config::imuTopic = "";
std::string  Config::odomTopic = "";
std::string  Config::gpsTopic = "";


//Frame
std::string  Config::lidarFrame="";
std::string  Config::baselinkFrame="";
std::string  Config::odometryFrame="";
std::string  Config::mapFrame="";   

    //GPS Setting
bool  Config::useGPS=false;
bool   Config::updateOrigin=false;
int  Config::gpsFrequence=-1;
bool  Config::useImuHeadingInitialization=false;   
bool Config::useGpsElevation=false;
float  Config::gpsCovThreshold=-1;
float   Config::poseCovThreshold=-1;
float  Config::gpsDistance=-1;     

    //debu setting
bool Config::debugLidarTimestamp=false;
bool Config::debugImu=false;
bool  Config::debugGps=false;

    //Export settings
bool  Config::savePCD=false;
std::string  Config::savePCDDirectory="";
std::string Config::sensor="";
int Config::N_SCAN=-1;
int  Config::Horizon_SCAN=-1;
int  Config::downsampleRate=-1;
float  Config::lidarMinRange=-1;
float Config::lidarMaxRange=-1;


    //IMU Settings
double Config::imuAccNoise=-1;
double Config::imuGyrNoise=-1;
double  Config::imuAccBiasN=-1;
double  Config::imuGyrBiasN=-1;
double Config:: imuGravity=-1;
double Config::imuRPYWeight=-1;


int Config::imu_type=-1;
Eigen::Vector3d Config::extrinsicTrans;
Eigen::Matrix3d Config::extrinsicRot;
Eigen::Matrix3d Config::extrinsicRPY;

// LOAM feature threshold
float  Config::edgeThreshold=-1;
float Config::surfThreshold=-1;
int  Config::edgeFeatureMinValidNum=-1;
int  Config::surfFeatureMinValidNum=-1;

//voxel filter paprams
float Config::odometrySurfLeafSize=-1;
float Config::mappingCornerLeafSize=-1;
float  Config::mappingSurfLeafSize=-1;

// robot motion constraint (in case you are using a 2D robot)
float Config::z_tollerance=-1;
float Config::rotation_tollerance=-1;

//CPU Params
int Config::numberOfCores=-1;
float Config::mappingProcessInterval=-1;

//Surrounding map
double  Config::surroundingkeyframeAddingDistThreshold=-1;
double  Config::surroundingkeyframeAddingAngleThreshold=-1;
double Config::surroundingKeyframeDensity=-1;
double Config::surroundingKeyframeSearchRadius=-1;

//Loop closure
bool  Config::loopClosureEnableFlag=false;
float  Config::loopClosureFrequency=-1;
int Config::surroundingKeyframeSize=-1;
float Config::historyKeyframeSearchRadius=-1;
float Config::historyKeyframeSearchTimeDiff=-1;
int Config::historyKeyframeSearchNum=-1;
float  Config::historyKeyframeFitnessScore=-1;

// Visualization
float Config::globalMapVisualizationSearchRadius=-1;
float  Config::globalMapVisualizationPoseDensity=-1;
float Config::globalMapVisualizationLeafSize=-1;

//mapping
float Config::globalMapLeafSize=-1;


void Load_YAML(std::string path)
{
    YAML::Node config;
    try{
         config = YAML::LoadFile(path);
    } catch(YAML::BadFile &e) {
        std::cout<<"read error!"<<path<<std::endl;
        exit(1);
    }
    std::cout<<"success load"<<std::endl;
    Config::pointCloudTopic = config["pointCloudTopic"].as<std::string>();
    std::cout<<"1111111 load"<<std::endl;
    Config::imuTopic = config["imuTopic"].as<std::string>();
    Config::odomTopic = config["odomTopic"].as<std::string>();
    Config::gpsTopic = config["gpsTopic"].as<std::string>();

//     //Frame
    Config::lidarFrame=config["lidarFrame"].as<std::string>();
    Config::baselinkFrame=config["baselinkFrame"].as<std::string>();
    Config::odometryFrame=config["odometryFrame"].as<std::string>();
    Config::mapFrame=config["mapFrame"].as<std::string>();   
    std::cout<<Config:: mapFrame<<std::endl; 

//     //GPS Setting
    Config::useGPS=config["useGPS"].as<bool>();
    Config::updateOrigin=config["updateOrigin"].as<bool>();
    Config::gpsFrequence=config["gpsFrequence"].as<int >();
    Config::useImuHeadingInitialization=config["useImuHeadingInitialization"].as<bool>();   
    Config::useGpsElevation=config["useGpsElevation"].as<bool>();
    Config::gpsCovThreshold=config["gpsCovThreshold"].as<float >();
    Config::poseCovThreshold=config["poseCovThreshold"].as<float >();
    Config::gpsDistance=config["gpsDistance"].as<float >();     
    std::cout<<Config::gpsDistance<<std::endl;

    // //debu setting
    Config::debugLidarTimestamp=config["debugLidarTimestamp"].as<bool >(); 
    Config::debugImu=config["debugImu"].as<bool >(); 
    Config::debugGps=config["debugGps"].as<bool >(); 
    std::cout<<Config::debugGps<<std::endl;

    // //Export settings
    Config::savePCD=config["savePCD"].as<bool >(); 
    Config::savePCDDirectory=config["savePCDDirectory"].as<std::string >(); 
    

    Config::sensor=config["sensor"].as<std::string >(); 
    Config::N_SCAN=config["N_SCAN"].as<int >();
    Config::Horizon_SCAN=config["Horizon_SCAN"].as<int >();
    Config::downsampleRate=config["downsampleRate"].as<int >();
    Config::lidarMinRange=config["lidarMinRange"].as<float >();
    Config::lidarMaxRange=config["lidarMaxRange"].as<float >();
   std::cout<<Config::lidarMaxRange<<std::endl;


    //IMU Settings
    Config:: imuAccNoise=config["imuAccNoise"].as<double >();
    Config:: imuGyrNoise=config["imuGyrNoise"].as<double >();
    Config::  imuAccBiasN=config["imuAccBiasN"].as<double >();
    Config::  imuGyrBiasN=config["imuGyrBiasN"].as<double >();
    Config::  imuGravity=config["imuGravity"].as<double >();
    Config::  imuRPYWeight=config["imuRPYWeight"].as<double >();
    std::cout<<Config::imuRPYWeight<<std::endl;



    //Extrinsics (lidar -> IMU)
    Config::imu_type=config["imu_type"].as<int >();
    Config::extrinsicTrans<<config["extrinsicTrans"][0].as<double >(),config["extrinsicTrans"][1].as<double >(),config["extrinsicTrans"][2].as<double >();
    //construct(Config::extrinsicTrans,config["extrinsicTrans"].as<double >());
    //std::cout<<Config::extrinsicTrans<<std::endl;
   // std::cout << "key type " <<config["extrinsicTrans"].size() << std::endl;
    Config::extrinsicRot<<config["extrinsicRot"][0].as<double >(),config["extrinsicRot"][1].as<double >(),config["extrinsicRot"][2].as<double >(),
                                                config["extrinsicRot"][3].as<double >(),config["extrinsicRot"][4].as<double >(),config["extrinsicRot"][5].as<double >(),
                                                config["extrinsicRot"][6].as<double >(),config["extrinsicRot"][7].as<double >(),config["extrinsicRot"][8].as<double >();

                                                                                                                                
    Config::extrinsicRPY<<config["extrinsicRPY"][0].as<double >(),config["extrinsicRPY"][1].as<double >(),config["extrinsicRPY"][2].as<double >(),
                                                config["extrinsicRPY"][3].as<double >(),config["extrinsicRPY"][4].as<double >(),config["extrinsicRPY"][5].as<double >(),
                                                config["extrinsicRPY"][6].as<double >(),config["extrinsicRPY"][7].as<double >(),config["extrinsicRPY"][8].as<double >();

    std::cout<<Config::extrinsicRot(4)<<std::endl;


    // LOAM feature threshold
    Config::edgeThreshold=config["edgeThreshold"].as<float >();
    Config::surfThreshold=config["surfThreshold"].as<float >();
    Config::edgeFeatureMinValidNum=config["edgeFeatureMinValidNum"].as<int >();
    Config::surfFeatureMinValidNum=config["surfFeatureMinValidNum"].as<int >();
    std::cout<<Config::surfFeatureMinValidNum<<std::endl;

    //voxel filter paprams
    Config::odometrySurfLeafSize=config["odometrySurfLeafSize"].as<float >();
    Config::mappingCornerLeafSize=config["mappingCornerLeafSize"].as<float >();
    Config::mappingSurfLeafSize=config["mappingSurfLeafSize"].as<float >();
    std::cout<<Config::mappingSurfLeafSize<<std::endl;

    // robot motion constraint (in case you are using a 2D robot)
    Config::z_tollerance=config["z_tollerance"].as<float >();
    Config::rotation_tollerance=config["rotation_tollerance"].as<float >();
    std::cout<<Config::rotation_tollerance<<std::endl;  

    //CPU Params
   Config::numberOfCores=config["numberOfCores"].as<int >();
   Config::mappingProcessInterval=config["mappingProcessInterval"].as<float >();
   std::cout<<Config::mappingProcessInterval<<std::endl;  

    //Surrounding map
    Config::surroundingkeyframeAddingDistThreshold=config["surroundingkeyframeAddingDistThreshold"].as<double >();
    Config::surroundingkeyframeAddingAngleThreshold=config["surroundingkeyframeAddingAngleThreshold"].as<double >();
    Config::surroundingKeyframeDensity=config["surroundingKeyframeDensity"].as<double >();
    Config::surroundingKeyframeSearchRadius=config["surroundingKeyframeSearchRadius"].as<double >();
    std::cout<<Config::surroundingKeyframeSearchRadius<<std::endl;  

    //Loop closure
    Config::loopClosureEnableFlag=config["loopClosureEnableFlag"].as<bool >();
    Config::loopClosureFrequency=config["loopClosureFrequency"].as<float >();
    Config::surroundingKeyframeSize=config["surroundingKeyframeSize"].as<int >();
    Config::historyKeyframeSearchRadius=config["historyKeyframeSearchRadius"].as<float >();
    Config::historyKeyframeSearchTimeDiff=config["historyKeyframeSearchTimeDiff"].as<float >();
    Config::historyKeyframeSearchNum=config["historyKeyframeSearchNum"].as<int >();
    Config::historyKeyframeFitnessScore=config["historyKeyframeFitnessScore"].as<float >();
     std::cout<<Config::historyKeyframeSearchNum<<std::endl;

    // Visualization
    Config::globalMapVisualizationSearchRadius=config["globalMapVisualizationSearchRadius"].as<float >();
    Config::globalMapVisualizationPoseDensity=config["globalMapVisualizationPoseDensity"].as<float >();
    Config::globalMapVisualizationLeafSize=config["globalMapVisualizationLeafSize"].as<float >();
     std::cout<<Config::globalMapVisualizationLeafSize<<std::endl;


    //mapping
    Config:: globalMapLeafSize=config["globalMapLeafSize"].as<float >();
    std::cout<<Config::globalMapLeafSize<<std::endl;

}
#endif