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

struct CloudInfo{
    pcl::PointCloud<PointType> corner_cloud;
    pcl::PointCloud<PointType> surf_cloud;
    int frame_id;
};

class SaveMap{
public:
    std::string sav_root_path;
    std::mutex data_mutex;
    std::deque<CloudInfo> pts_deque;

    void SaveToDisk(const CloudInfo& cloud_info){
        pcl::io::savePCDFileASCII(sav_root_path+std::to_string(cloud_info.frame_id)+"_corner.pcd", cloud_info.corner_cloud);
        pcl::io::savePCDFileASCII(sav_root_path+std::to_string(cloud_info.frame_id)+"_surf.pcd", cloud_info.surf_cloud);
    }

    void do_work(){
        while(1){
            if(pts_deque.size()!=0){
                data_mutex.lock();
                auto cloud_info = pts_deque.front();
                pts_deque.pop_front();
                data_mutex.unlock();
                SaveToDisk(cloud_info);
            }
            sleep(10);
        }
    }

    void addToSave(const CloudInfo& cloud_info){
        data_mutex.lock();
        pts_deque.push_back(cloud_info);
        data_mutex.unlock();
    }

    void Init(const std::string& path){
        sav_root_path = path;
    }
};

#endif //SEU_LIDARLOC_SAVE_MAP_H
