
#ifndef SEU_LIDARLOC_DATA_CHECK_MANAGER_H
#define SEU_LIDARLOC_DATA_CHECK_MANAGER_H
#include <mutex>
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "utils/timer.h"
#include "opt_mapping.h"
#include "opt_loc.h"
#include "data_preprocess.h"
#include "utils/MapSaver.h"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"


class DataCheck  {
public:
    PubSubInterface* pubsub;

    std::mutex gnss_mutex;
    std::mutex cloud_mutex;

    std::deque<CloudTypeXYZIRTPtr> deque_cloud;
    std::deque<GNSSINSType> gnssQueue;
    std::deque<WheelType> drQueue;

    std::thread* do_work_thread;

    TicToc timer_dr;

    bool init = false;
    GeographicLib::LocalCartesian geoConverter;

    std::string topic_gnss_odom_world = "/gnss_odom_world";
    std::string topic_dr_odom_world = "/dr_odom_world";
    std::string save_map_path = "/home/lsy/map/";

    //data from gnss
    Eigen::Vector3d lla;
    Eigen::Vector3d lla_sigma;

    double roll,pitch,yaw;
    Eigen::Vector3d rpy_sigma;

    Eigen::Vector3d imu_angular_v_raw;
    Eigen::Vector3d imu_linear_acc_raw;

    Eigen::Vector3d imu_angular_v_body;
    Eigen::Vector3d imu_linear_acc_body;

    double velocity;
    double velocity_sigma;
    Eigen::Vector4d wheel_speed;//RR RL FR FL

    int gps_status;


    void DoWork(){
        while(1){

            if(timer_dr.toc()>5000){
                
                for(int i = 0;i < gnssQueue.size();++i){

                    lla = gnssQueue[i].lla;




                    std::fstream originStream( save_map_path +"Origin.txt", std::fstream::out);
//                    EZLOG(INFO)<<"save GNSS init point!"<<std::endl;
                    originStream.precision(9);
                    originStream << gps_point[0] << " " << gps_point[1] << " " << gps_point[2]
                                 << std::endl;
                    originStream.close();

                }






            }
            else{
                sleep(0.01);
            }
        }//end fucntion while(1)
    }//end fucntion do work



    void AddCloudData(const CloudTypeXYZIRT& data){

        CloudTypeXYZIRTPtr cloud_ptr(new CloudTypeXYZIRT());

        *cloud_ptr = data;//深拷贝
        cloud_mutex.lock();
        deque_cloud.push_back(cloud_ptr);
        cloud_mutex.unlock();

    }

    void AddGNSSINSData(GNSSINSType& data){

//        WheelType wheel_tmp;
//        wheel_tmp.ESCWhlRRSpd = data.wheel_speed[0];
//        wheel_tmp.ESCWhlRLSpd = data.wheel_speed[1];
//        wheel_tmp.ESCWhlFRSpd = data.wheel_speed[2];
//        wheel_tmp.ESCWhlFLSpd = data.wheel_speed[3];
//
//        gnss_mutex.lock();
//        drQueue.push_back(wheel_tmp);
//        gnss_mutex.unlock();

        if(!init)
        {
            geoConverter.Reset(data.lla[0], data.lla[1], data.lla[2]);
            init = true;
            MapSaver::SaveOriginLLA(data.lla);
            return;
        }

        double t_enu[3];
        geoConverter.Forward(data.lla[0], data.lla[1], data.lla[2],
                             t_enu[0], t_enu[1], t_enu[2]);//t_enu = enu coordiate

        Eigen::Matrix3d z_matrix;//calculate Quaternion
        Eigen::Matrix3d x_matrix;
        Eigen::Matrix3d y_matrix;
        double heading_Z = -data.yaw * 3.1415926535 / 180.0;
        double pitch_Y = data.pitch * 3.1415926535 / 180.0;
        double roll_X = data.roll * 3.1415926535 / 180.0;
        z_matrix << cos(heading_Z), -sin(heading_Z), 0,
                sin(heading_Z), cos(heading_Z),  0,
                0,                 0,            1;

        x_matrix << 1,                 0,              0,
                0,            cos(pitch_Y),     -sin(pitch_Y),
                0,            sin(pitch_Y),    cos(pitch_Y);

        y_matrix << cos(roll_X) ,     0,        sin(roll_X),
                0,                 1,               0 ,
                -sin(roll_X),      0,         cos(roll_X);

        Eigen::Matrix3d R_w_b =  (z_matrix*x_matrix*y_matrix);   // Pw = Twb * Pb
        Eigen::Vector3d t_w_b(t_enu[0], t_enu[1], t_enu[2]);
        Eigen::Quaterniond q_w_b(R_w_b);//获得局部坐标系的四元数
        PoseT T_w_b(t_w_b, R_w_b);
//        world is GNSS ,base is car, T_w_l =
        PoseT T_w_l = PoseT(T_w_b.pose*(SensorConfig::T_L_B.inverse())); // Pw = Twb * Tbl * Pl

        OdometryType T_w_l_pub;
        T_w_l_pub.frame = "map";
        T_w_l_pub.timestamp = data.timestamp;
        T_w_l_pub.pose = T_w_l;

        data.lla[0] = t_enu[0];
        data.lla[1] = t_enu[1];
        data.lla[2] = t_enu[2];
       
        gnss_mutex.lock();
        gnssQueue.push_back(data);
        gnss_mutex.unlock();

    }

    void Init(PubSubInterface* pubsub_){

        pubsub = pubsub_;
        pubsub->addPublisher(topic_gnss_odom_world, DataType::ODOMETRY,2000);
        pubsub->addPublisher(topic_dr_odom_world, DataType::ODOMETRY,2000);
        do_work_thread = new std::thread(&DataCheck::DoWork, this);

    }

};
#endif
