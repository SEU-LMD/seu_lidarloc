//
// Created by fyy on 23-8-23.
//

#ifndef SEU_LIDARLOC_SAVE_MAP_H
#define SEU_LIDARLOC_SAVE_MAP_H
#include <string>
#include <thread>
#include <mutex>
#include <deque>
#include <vector>
#include "Eigen/Dense"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "./utility.h"
#include "timer.h"
//TODO11111 chang name to map_saver
struct CloudInfoFt{
    pcl::PointCloud<PointType>::Ptr corner_cloud;
    pcl::PointCloud<PointType>::Ptr surf_cloud;
    int frame_id;
};

class MapSaver{
public:
    std::string sav_root_path;
    std::mutex data_mutex;
    std::deque<CloudInfoFt> pts_deque;


    void SaveCloud(const CloudInfoFt& cloud_info){
       // pcl::io::savePCDFileBinary(MappingConfig::save_map_path+std::to_string(cloud_info.frame_id)+"_raw.pcd", *cloud_info.raw_Cloud);
        pcl::io::savePCDFileBinary(MappingConfig::save_map_path+std::to_string(cloud_info.frame_id)+"_surf.pcd", *cloud_info.surf_cloud);
        pcl::io::savePCDFileBinary(MappingConfig::save_map_path+std::to_string(cloud_info.frame_id)+"_corner.pcd", *cloud_info.corner_cloud);

    }

    static void SaveOriginLLA(const Eigen::Vector3d& gps_point){
        std::fstream originStream( MappingConfig::save_map_path +"Origin.txt", std::fstream::out);
        EZLOG(INFO)<<"save GNSS init point!"<<std::endl;
        originStream.precision(9);
        originStream << gps_point[0] << " " << gps_point[1] << " " << gps_point[2]
                     << std::endl;
        originStream.close();
    }

    static void SavePoses(const std::vector<PoseT>& opt_poses) {
        std::fstream stream(MappingConfig::save_map_path + "opt_poses.txt",std::fstream::out);
        stream.precision(6);
        for (int i = 0; i < opt_poses.size(); i++) {
            Eigen::Vector3d p = opt_poses[i].GetXYZ();
            Eigen::Quaterniond  q = opt_poses[i].GetQ();
            stream << i << " " << p.x() << " " << p.y() << " "
                   << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
                   << q.w() << std::endl;
        }

    }

    static void SaveCorrectPoses(const vector<pair<PointType,PointType>>& beforeLoopOpt, const  vector<pair<PointType,PointType>>& afterLoopOpt){
        std::fstream stream1(MappingConfig::save_map_path + "correct_opt_poses.txt",std::fstream::out);
        std::fstream stream2(MappingConfig::save_map_path + "before_correct_opt_poses.txt",std::fstream::out);
        stream1.precision(6);
        stream2.precision(6);
        assert(beforeLoopOpt.size() == afterLoopOpt.size());
        for (int i = 0; i < beforeLoopOpt.size(); i++) {
            PointType  before = beforeLoopOpt[i].first;
            PointType  after = afterLoopOpt[i].first;
            PointType  pre = beforeLoopOpt[i].second;
            stream2 << before.x << " " <<before.y << " "<< before.z << std::endl;
            stream1 << after.x << " " <<after.y << " "<< after.z << std::endl;
            stream1 << pre.x << " " <<pre.y << " "<< pre.z << std::endl;
        }
        stream1.close();
        stream2.close();
    }



    void do_work(){
        while(1){
            if(pts_deque.size()!=0){
              //  EZLOG(INFO)<<"pts_deque.size() = "<<pts_deque.size()<<std::endl;
                data_mutex.lock();
                auto cloud_info = pts_deque.front();
                pts_deque.pop_front();
                data_mutex.unlock();

                SaveCloud(cloud_info);
                //EZLOG(INFO)<<"SaveCloud !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            }
            sleep(0.05);
        }
    }

    void AddCloudToSave(const CloudInfoFt& cloud_info){
        data_mutex.lock();
        pts_deque.push_back(cloud_info);
        data_mutex.unlock();
        //EZLOG(INFO)<<"receive Cloud !"<<std::endl;
        //EZLOG(INFO)<<"pts_deque.size() = "<<pts_deque.size()<<std::endl;
    }

};

#endif //SEU_LIDARLOC_SAVE_MAP_H
