//
// Created by wxy on 23-9-20.
//

#ifndef ERASOR_CONFIG_HELPER_H
#define ERASOR_CONFIG_HELPER_H

#endif //ERASOR_CONFIG_HELPER_H

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

class ErasorConfig{
      public:
           static float max_range;
           static int  num_rings;
           static int  num_sectors;
           static double  min_h; //[m] Note that it depends on the distance between body frame and ground contact point of mobile robots
           static double max_h; // [m] Note that it depends on the distance between body frame and ground contact point of mobile robots
           static double th_bin_max_h;
           static float scan_ratio_threshold;
           static int minimum_num_pts;
           static int rejection_ratio;
           static float gf_dist_thr;
           static int gf_iter;
           static int gf_num_lpr;
           static float gf_th_seeds_height;
           static int version; // # 2: R-GPF / 3: R-GPF w/ blocking (not in paper)


           static std::string data_name;
           static std::string initial_map_path;
           static std::string env;
           static std::string save_path;
           static float query_voxel_size;
           static float map_voxel_size;
           static int voxelization_interval;
           static int removal_interval;

           static std::string data_dir;
           static float voxel_size;
           static int init_idx;
           static int interval;

           static Eigen::VectorXd lidar2body;
           //static Eigen::Quaterniond q_lidar_body;
          // static Vector lidar2body[7];//lidar2body: [0.0, 0.0, 0.0, 0, 0.0, 0.0, 1.0] # xyz q_x, q_y, q_z, q_w in order
           static bool verbose;
           static double surroundingKeyframeSearchRadius;
};

           float ErasorConfig::max_range = 9.5;
           int ErasorConfig::num_rings = 8;
           int ErasorConfig::num_sectors = 60;
           double ErasorConfig::min_h = -1.6;
           double ErasorConfig::max_h = 13;
           double ErasorConfig::th_bin_max_h = -1.0;
           float ErasorConfig::scan_ratio_threshold = 0.2;
           int ErasorConfig::minimum_num_pts = 5;
           int ErasorConfig::rejection_ratio = 0;
           float ErasorConfig::gf_dist_thr = 0.075;
           int ErasorConfig::gf_iter = 3;
           int ErasorConfig::gf_num_lpr = 12;
           float ErasorConfig::gf_th_seeds_height = 0.5;
           int ErasorConfig::version = 3;

           std::string ErasorConfig::data_name = " ";
           std::string ErasorConfig::initial_map_path = " ";
           std::string ErasorConfig::env = " ";
           std::string ErasorConfig::save_path = " ";
           float ErasorConfig::query_voxel_size = 0.2;
           float ErasorConfig::map_voxel_size = 0.2;
           int ErasorConfig::voxelization_interval = 2;
           int ErasorConfig::removal_interval = 4;

           std::string  ErasorConfig::data_dir =" ";
           float ErasorConfig::voxel_size = 0.075;
           int ErasorConfig::init_idx = 130;
           int ErasorConfig::interval = 2;

           Eigen::VectorXd ErasorConfig::lidar2body;
          // Eigen::Quaterniond ErasorConfig::q_lidar_body;
//  Vector lidar2body[7];//lidar2body: [0.0, 0.0, 0.0, 0, 0.0, 0.0, 1.0] # xyz q_x, q_y, q_z, q_w in order
           bool ErasorConfig::verbose = true;
           double ErasorConfig::surroundingKeyframeSearchRadius = 30;

  void Load_Erasor_Yaml(std::string path){
      YAML::Node config;
      try{
          config = YAML::LoadFile(path);
      } catch(YAML::BadFile &e) {
          std::cout<<"sensorconfig yaml read error!"<<path<<std::endl;
          exit(1);
      }

      ErasorConfig::max_range = config["max_range"].as<float>();
      ErasorConfig::num_rings = config["num_rings"].as<int>();
      ErasorConfig::num_sectors = config["num_sectors"].as<int>();
      ErasorConfig::min_h = config["min_h"].as<double>();
      ErasorConfig::max_h = config["max_h"].as<double>();
      ErasorConfig::th_bin_max_h = config["th_bin_max_h"].as<double>();
      ErasorConfig::scan_ratio_threshold = config["scan_ratio_threshold"].as<float>();
      ErasorConfig::minimum_num_pts = config["minimum_num_pts"].as<int>();
      ErasorConfig::rejection_ratio = config["rejection_ratio"].as<int>();
      ErasorConfig::gf_dist_thr = config["gf_dist_thr"].as<float>();
      ErasorConfig::gf_iter = config["gf_iter"].as<int>();
      ErasorConfig::gf_num_lpr = config["gf_num_lpr"].as<int>();
      ErasorConfig::gf_th_seeds_height = config["gf_th_seeds_height"].as<float>();
      ErasorConfig::version = config["version"].as<int>();

      ErasorConfig::data_name = config["data_name"].as<std::string>();
      ErasorConfig::initial_map_path = config["initial_map_path"].as<std::string>();
      ErasorConfig::env = config["env"].as<std::string>();
      ErasorConfig::save_path = config["env"].as<std::string>();
      ErasorConfig::query_voxel_size = config["query_voxel_size"].as<float>();
      ErasorConfig::map_voxel_size = config["map_voxel_size"].as<float>();
      ErasorConfig::voxelization_interval = config["voxelization_interval"].as<float>();
      ErasorConfig::removal_interval = config["removal_interval"].as<int>();

      ErasorConfig::data_dir = config["data_dir"].as<std::string>();
      ErasorConfig::voxel_size = config["voxel_size"].as<float>();
      ErasorConfig::init_idx = config["init_idx"].as<int>();
      ErasorConfig::interval = config["interval"].as<int>();
      ErasorConfig::lidar2body << config["lidar2body"][0].as<double>(),config["lidar2body"][1].as<double>(),config["lidar2body"][2].as<double>(),
                                  config["lidar2body"][3].as<double>(),config["lidar2body"][4].as<double>(),config["lidar2body"][5].as<double>(),
                                  config["lidar2body"][6].as<double>();

      ErasorConfig::surroundingKeyframeSearchRadius = config["surroundingKeyframeSearchRadius"].as<double>();

      std::cout<<"config yaml success load"<<std::endl;

  }


