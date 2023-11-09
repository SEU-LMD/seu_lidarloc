
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
    std::deque<GNSSINSType> gnssQueue ;
    std::deque<WheelType> drQueue;

    std::thread* do_work_thread;

    TicToc timer_gnss;

    bool init = false;
    bool gnsstime_init = false;
    bool lidartime_init = false;
    GeographicLib::LocalCartesian geoConverter;

    std::string topic_gnss_odom_world = "/gnss_odom_world";
    std::string topic_dr_odom_world = "/dr_odom_world";
    std::string save_map_path = "/home/lsy/map/";

    double lidar_start_time;
    double gnss_start_time;

    double time_init_lidar;
    double time_init_gnss;


    void DoWork(){
        while(1){

//            if(timer_gnss.toc()>5000){
            if(!deque_cloud.empty()){

//                EZLOG(INFO)<<"dealing with data!"<<endl;

                gnss_mutex.lock();
                //deal with gnss
                for(int i = 0 ;i < gnssQueue.size();++i){

                    if(!gnsstime_init){
                        time_init_gnss = gnssQueue[i].timestamp;
                        gnsstime_init = true;
                        auto timestamp = gnssQueue[i].timestamp;
                        auto timestamp_rel = 0;
                        auto lla = gnssQueue[i].lla;
                        auto lla_sigma = gnssQueue[i].lla_sigma;
                        auto  roll= gnssQueue[i]. roll;
                        auto  pitch= gnssQueue[i]. pitch;
                        auto  yaw= gnssQueue[i]. yaw;
                        auto  rpy_sigma= gnssQueue[i]. rpy_sigma;
                        auto  imu_angular_v_raw= gnssQueue[i]. imu_angular_v_raw;
                        auto  imu_linear_acc_raw= gnssQueue[i]. imu_linear_acc_raw;
                        auto  imu_angular_v_body= gnssQueue[i]. imu_angular_v_body;
                        auto  imu_linear_acc_body= gnssQueue[i]. imu_linear_acc_body;
                        auto  velocity= gnssQueue[i]. velocity;
                        auto  velocity_sigma= gnssQueue[i]. velocity_sigma;
                        auto  wheel_speed= gnssQueue[i]. wheel_speed;
                        auto  gps_status= gnssQueue[i]. gps_status;

                        std::ofstream originStream( save_map_path +"gnss_data.txt", std::ios_base::app);
                        originStream.precision(11);
                        originStream << setprecision(11)<<timestamp_rel << " " << lla[0] << " " << lla[1] << " " << lla[2] << " " //1-4col time lla
                                     << lla_sigma[0] << " " << lla_sigma[1] << " " << lla_sigma[2] << " " //5-7 col

                                     << roll << " " << pitch << " " << yaw << " "  //8-10 col
                                     << rpy_sigma[0] << " " << rpy_sigma[1] << " " << rpy_sigma[2] << " " //11-13 col

                                     << imu_angular_v_raw[0] << " " << imu_angular_v_raw[1] << " " << imu_angular_v_raw[2] << " " //14-16 col
                                     << imu_linear_acc_raw[0] << " " << imu_linear_acc_raw[1] << " " << imu_linear_acc_raw[2] << " " //17-19 col

                                     << imu_angular_v_body[0] << " " << imu_angular_v_body[1] << " " << imu_angular_v_body[2] << " " //20-22 col
                                     << imu_linear_acc_body[0] << " " << imu_linear_acc_body[1] << " " << imu_linear_acc_body[2] << " " //23-25 col

                                     << velocity << " " << velocity_sigma << " " << gps_status << " " //26-28 col
                                     << wheel_speed[0] << " " << wheel_speed[1] << " " << wheel_speed[2] << " " << wheel_speed[3] << " " << i << " " << timestamp //29-32 col

                                     << std::endl;
                        originStream.close();
                        continue;
                    }
                    auto timestamp = gnssQueue[i].timestamp;
                    auto timestamp_rel = gnssQueue[i].timestamp - time_init_gnss;
                    auto lla = gnssQueue[i].lla;
                    auto lla_sigma = gnssQueue[i].lla_sigma;
                    auto  roll= gnssQueue[i]. roll;
                    auto  pitch= gnssQueue[i]. pitch;
                    auto  yaw= gnssQueue[i]. yaw;
                    auto  rpy_sigma= gnssQueue[i]. rpy_sigma;
                    auto  imu_angular_v_raw= gnssQueue[i]. imu_angular_v_raw;
                    auto  imu_linear_acc_raw= gnssQueue[i]. imu_linear_acc_raw;
                    auto  imu_angular_v_body= gnssQueue[i]. imu_angular_v_body;
                    auto  imu_linear_acc_body= gnssQueue[i]. imu_linear_acc_body;
                    auto  velocity= gnssQueue[i]. velocity;
                    auto  velocity_sigma= gnssQueue[i]. velocity_sigma;
                    auto  wheel_speed= gnssQueue[i]. wheel_speed;
                    auto  gps_status= gnssQueue[i]. gps_status;

                    std::ofstream originStream( save_map_path +"gnss_data.txt", std::ios_base::app);
                    originStream.precision(11);
                    originStream << setprecision(11)<<timestamp_rel << " " << lla[0] << " " << lla[1] << " " << lla[2] << " " //1-4col time lla
                                 << lla_sigma[0] << " " << lla_sigma[1] << " " << lla_sigma[2] << " " //5-7 col

                                 << roll << " " << pitch << " " << yaw << " "  //8-10 col
                                 << rpy_sigma[0] << " " << rpy_sigma[1] << " " << rpy_sigma[2] << " " //11-13 col

                                 << imu_angular_v_raw[0] << " " << imu_angular_v_raw[1] << " " << imu_angular_v_raw[2] << " " //14-16 col
                                 << imu_linear_acc_raw[0] << " " << imu_linear_acc_raw[1] << " " << imu_linear_acc_raw[2] << " " //17-19 col

                                 << imu_angular_v_body[0] << " " << imu_angular_v_body[1] << " " << imu_angular_v_body[2] << " " //20-22 col
                                 << imu_linear_acc_body[0] << " " << imu_linear_acc_body[1] << " " << imu_linear_acc_body[2] << " " //23-25 col

                                 << velocity << " " << velocity_sigma << " " << gps_status << " " //26-28 col
                                 << wheel_speed[0] << " " << wheel_speed[1] << " " << wheel_speed[2] << " " << wheel_speed[3] << " " << i << " " << timestamp //29-32 col

                                 << std::endl;
                    originStream.close();

                }
                while(!gnssQueue.empty()){
                    gnssQueue.pop_front();
                }
                gnss_mutex.unlock();

                cloud_mutex.lock();
                //deal with lidar
                for(int i = 0; i < deque_cloud.size(); ++i){

                    if(!lidartime_init){
                        time_init_lidar = deque_cloud[i]->timestamp;
                        lidartime_init = true;
                        auto lidar_time = deque_cloud[i]->timestamp;
                        auto lidar_time_rel = 0;
                        auto lidar_size = deque_cloud[i]->cloud.size();

                        std::ofstream originStream( save_map_path +"lidar_data.txt", std::ios_base::app);
                        originStream.precision(11);
                        originStream << setprecision(11)<<lidar_time_rel << " " << lidar_size <<  " " << i <<  " " << lidar_time <<std::endl; //1-4col time lla
                        continue;
                    }

                    auto lidar_time = deque_cloud[i]->timestamp;
                    auto lidar_time_rel = deque_cloud[i]->timestamp - time_init_lidar;
                    auto lidar_size = deque_cloud[i]->cloud.size();

                    std::ofstream originStream( save_map_path +"lidar_data.txt", std::ios_base::app);
                    originStream.precision(11);
                    originStream << setprecision(11)<<lidar_time_rel << " " << lidar_size <<  " " << i <<  " " << lidar_time << std::endl; //1-4col time lla

                }
                while(!deque_cloud.empty()){
                    deque_cloud.pop_front();
                }
                cloud_mutex.unlock();

                ///perform once
//                {
//
//                    EZLOG(INFO)<<"gnssQueue.size() = "<<gnssQueue.size()<<endl;
//
//                    for(int i = 0 ;i < gnssQueue.size();++i){
//
//                        if(!gnsstime_init){
//                            time_init_gnss = gnssQueue[i].timestamp;
//                            gnsstime_init = true;
//                            auto timestamp = gnssQueue[i].timestamp;
//                            auto timestamp_rel = 0;
//                            auto lla = gnssQueue[i].lla;
//                            auto lla_sigma = gnssQueue[i].lla_sigma;
//                            auto  roll= gnssQueue[i]. roll;
//                            auto  pitch= gnssQueue[i]. pitch;
//                            auto  yaw= gnssQueue[i]. yaw;
//                            auto  rpy_sigma= gnssQueue[i]. rpy_sigma;
//                            auto  imu_angular_v_raw= gnssQueue[i]. imu_angular_v_raw;
//                            auto  imu_linear_acc_raw= gnssQueue[i]. imu_linear_acc_raw;
//                            auto  imu_angular_v_body= gnssQueue[i]. imu_angular_v_body;
//                            auto  imu_linear_acc_body= gnssQueue[i]. imu_linear_acc_body;
//                            auto  velocity= gnssQueue[i]. velocity;
//                            auto  velocity_sigma= gnssQueue[i]. velocity_sigma;
//                            auto  wheel_speed= gnssQueue[i]. wheel_speed;
//                            auto  gps_status= gnssQueue[i]. gps_status;
//
//                            std::ofstream originStream( save_map_path +"gnss_data.txt", std::ios_base::app);
//                            originStream.precision(11);
//                            originStream << setprecision(11)<<timestamp_rel << " " << lla[0] << " " << lla[1] << " " << lla[2] << " " //1-4col time lla
//                                         << lla_sigma[0] << " " << lla_sigma[1] << " " << lla_sigma[2] << " " //5-7 col
//
//                                         << roll << " " << pitch << " " << yaw << " "  //8-10 col
//                                         << rpy_sigma[0] << " " << rpy_sigma[1] << " " << rpy_sigma[2] << " " //11-13 col
//
//                                         << imu_angular_v_raw[0] << " " << imu_angular_v_raw[1] << " " << imu_angular_v_raw[2] << " " //14-16 col
//                                         << imu_linear_acc_raw[0] << " " << imu_linear_acc_raw[1] << " " << imu_linear_acc_raw[2] << " " //17-19 col
//
//                                         << imu_angular_v_body[0] << " " << imu_angular_v_body[1] << " " << imu_angular_v_body[2] << " " //20-22 col
//                                         << imu_linear_acc_body[0] << " " << imu_linear_acc_body[1] << " " << imu_linear_acc_body[2] << " " //23-25 col
//
//                                         << velocity << " " << velocity_sigma << " " << gps_status << " " //26-28 col
//                                         << wheel_speed[0] << " " << wheel_speed[1] << " " << wheel_speed[2] << " " << wheel_speed[3] << " " << i << " " << timestamp //29-32 col
//
//                                         << std::endl;
//                            originStream.close();
//                            continue;
//                        }
//                        auto timestamp = gnssQueue[i].timestamp;
//                        auto timestamp_rel = gnssQueue[i].timestamp - time_init_gnss;
//                        auto lla = gnssQueue[i].lla;
//                        auto lla_sigma = gnssQueue[i].lla_sigma;
//                        auto  roll= gnssQueue[i]. roll;
//                        auto  pitch= gnssQueue[i]. pitch;
//                        auto  yaw= gnssQueue[i]. yaw;
//                        auto  rpy_sigma= gnssQueue[i]. rpy_sigma;
//                        auto  imu_angular_v_raw= gnssQueue[i]. imu_angular_v_raw;
//                        auto  imu_linear_acc_raw= gnssQueue[i]. imu_linear_acc_raw;
//                        auto  imu_angular_v_body= gnssQueue[i]. imu_angular_v_body;
//                        auto  imu_linear_acc_body= gnssQueue[i]. imu_linear_acc_body;
//                        auto  velocity= gnssQueue[i]. velocity;
//                        auto  velocity_sigma= gnssQueue[i]. velocity_sigma;
//                        auto  wheel_speed= gnssQueue[i]. wheel_speed;
//                        auto  gps_status= gnssQueue[i]. gps_status;
//
//                        std::ofstream originStream( save_map_path +"gnss_data.txt", std::ios_base::app);
//                        originStream.precision(11);
//                        originStream << setprecision(11)<<timestamp_rel << " " << lla[0] << " " << lla[1] << " " << lla[2] << " " //1-4col time lla
//                                     << lla_sigma[0] << " " << lla_sigma[1] << " " << lla_sigma[2] << " " //5-7 col
//
//                                     << roll << " " << pitch << " " << yaw << " "  //8-10 col
//                                     << rpy_sigma[0] << " " << rpy_sigma[1] << " " << rpy_sigma[2] << " " //11-13 col
//
//                                     << imu_angular_v_raw[0] << " " << imu_angular_v_raw[1] << " " << imu_angular_v_raw[2] << " " //14-16 col
//                                     << imu_linear_acc_raw[0] << " " << imu_linear_acc_raw[1] << " " << imu_linear_acc_raw[2] << " " //17-19 col
//
//                                     << imu_angular_v_body[0] << " " << imu_angular_v_body[1] << " " << imu_angular_v_body[2] << " " //20-22 col
//                                     << imu_linear_acc_body[0] << " " << imu_linear_acc_body[1] << " " << imu_linear_acc_body[2] << " " //23-25 col
//
//                                     << velocity << " " << velocity_sigma << " " << gps_status << " " //26-28 col
//                                     << wheel_speed[0] << " " << wheel_speed[1] << " " << wheel_speed[2] << " " << wheel_speed[3] << " " << i << " " << timestamp //29-32 col
//
//                                     << std::endl;
//                        originStream.close();
//
//                    }
//
//                    EZLOG(INFO)<<"deque_cloud.size() = "<<deque_cloud.size()<<endl;
//
//                    for(int i = 0; i < deque_cloud.size(); ++i){
//
//                        if(!lidartime_init){
//                            time_init_lidar = gnssQueue[i].timestamp;
//                            lidartime_init = true;
//                            auto lidar_time = gnssQueue[i].timestamp;
//                            auto lidar_time_rel = 0;
//                            auto lidar_size = deque_cloud[i]->cloud.size();
//
//                            std::ofstream originStream( save_map_path +"lidar_data.txt", std::ios_base::app);
//                            originStream.precision(11);
//                            originStream << setprecision(11)<<lidar_time_rel << " " << lidar_size <<  " " << i <<  " " << lidar_time <<std::endl; //1-4col time lla
//                            continue;
//                        }
//
//                        auto lidar_time = gnssQueue[i].timestamp;
//                        auto lidar_time_rel = deque_cloud[i]->timestamp - time_init_lidar;
//                        auto lidar_size = deque_cloud[i]->cloud.size();
//
//                        std::ofstream originStream( save_map_path +"lidar_data.txt", std::ios_base::app);
//                        originStream.precision(11);
//                        originStream << setprecision(11)<<lidar_time_rel << " " << lidar_size <<  " " << i <<  " " << lidar_time << std::endl; //1-4col time lla
//
//                    }
//
////                    lidar_start_time = deque_cloud[0]->timestamp;
////                    gnss_start_time = gnssQueue[0].timestamp;
////                    std::ofstream originStream( save_map_path +"time_diff.txt", std::ios_base::app);
////                    originStream.precision(11);
////                    originStream << "lidar_start_time - gnss_start_time = " << lidar_start_time - gnss_start_time << " " << std::endl;
//
//
//                }

//                EZLOG(INFO)<<"finish saving data!! "<<endl;

//                exit(-1);

            }else{
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

//        if(!init)
//        {
//            geoConverter.Reset(data.lla[0], data.lla[1], data.lla[2]);
//            init = true;
//            MapSaver::SaveOriginLLA(data.lla);
//            return;
//        }
//
//        double t_enu[3];
//        geoConverter.Forward(data.lla[0], data.lla[1], data.lla[2],
//                             t_enu[0], t_enu[1], t_enu[2]);//t_enu = enu coordiate
//
//        Eigen::Matrix3d z_matrix;//calculate Quaternion
//        Eigen::Matrix3d x_matrix;
//        Eigen::Matrix3d y_matrix;
//        double heading_Z = -data.yaw * 3.1415926535 / 180.0;
//        double pitch_Y = data.pitch * 3.1415926535 / 180.0;
//        double roll_X = data.roll * 3.1415926535 / 180.0;
//        z_matrix << cos(heading_Z), -sin(heading_Z), 0,
//                sin(heading_Z), cos(heading_Z),  0,
//                0,                 0,            1;
//
//        x_matrix << 1,                 0,              0,
//                0,            cos(pitch_Y),     -sin(pitch_Y),
//                0,            sin(pitch_Y),    cos(pitch_Y);
//
//        y_matrix << cos(roll_X) ,     0,        sin(roll_X),
//                0,                 1,               0 ,
//                -sin(roll_X),      0,         cos(roll_X);
//
//        Eigen::Matrix3d R_w_b =  (z_matrix*x_matrix*y_matrix);   // Pw = Twb * Pb
//        Eigen::Vector3d t_w_b(t_enu[0], t_enu[1], t_enu[2]);
//        Eigen::Quaterniond q_w_b(R_w_b);//获得局部坐标系的四元数
//        PoseT T_w_b(t_w_b, R_w_b);
////        world is GNSS ,base is car, T_w_l =
//        PoseT T_w_l = PoseT(T_w_b.pose*(SensorConfig::T_L_B.inverse())); // Pw = Twb * Tbl * Pl
//
//        OdometryType T_w_l_pub;
//        T_w_l_pub.frame = "map";
//        T_w_l_pub.timestamp = data.timestamp;
//        T_w_l_pub.pose = T_w_l;
//
//        data.lla[0] = t_enu[0];
//        data.lla[1] = t_enu[1];
//        data.lla[2] = t_enu[2];
//
        gnss_mutex.lock();
        gnssQueue.push_back(data);
//        timer_gnss.tic();
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
