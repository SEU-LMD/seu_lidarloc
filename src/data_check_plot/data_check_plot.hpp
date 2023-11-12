#ifndef DATA_CHECK_PLOT_H
#define DATA_CHECK_PLOT_H

#include <fstream>              //文件输入输出的文件相关的
#include "Eigen/Core"           //eigen
#include "Eigen/Geometry"       //Isometry3d
#include <string>
#include <vector>
#include <iostream>

class Plot{

    public:

    //data from gnss
    double timestamp;
    std::vector<double>  v_timestamp;

    Eigen::Vector3d lla;
    std::vector<Eigen::Vector3d> v_lla;

    Eigen::Vector3d lla_sigma;
    std::vector<Eigen::Vector3d> v_lla_sigma;

    double roll,pitch,yaw;

    std::vector<double> v_roll;
    std::vector<double> v_pitch;
    std::vector<double> v_yaw;

    Eigen::Vector3d rpy_sigma;
    std::vector<Eigen::Vector3d> v_rpy_sigma;

    Eigen::Vector3d imu_angular_v_raw;
    std::vector<Eigen::Vector3d> v_imu_angular_v_raw;

    Eigen::Vector3d imu_linear_acc_raw;
    std::vector<Eigen::Vector3d> v_imu_linear_acc_raw;

    Eigen::Vector3d imu_angular_v_body;
    std::vector<Eigen::Vector3d> v_imu_angular_v_body;

    Eigen::Vector3d imu_linear_acc_body;
    std::vector<Eigen::Vector3d> v_imu_linear_acc_body;

    double velocity;
    std::vector<double> v_velocity;

    double  velocity_sigma;

    Eigen::Vector4d wheel_speed;//RR RL FR FL
    std::vector<Eigen::Vector4d> v_wheel_speed;

    int gps_status;

    std::string sav_path;

    //gnss check para
    std::vector<double> diff_gnss_timestamp;
    double average_gnss_time;
    double gnss_variance;
    double gnss_std_deviation;
    double  over3std_deviation_p;
    int gnss_time_run_count;


    //lidar para
    double  lidar_timestamp;
    std::vector<double> v_lidar_timestamp;

    int lidar_num;
    std::vector<int> v_lidar_num;

    //lidar check para
    std::vector<double> diff_lidar_timestamp;
    double average_lidar_time;
    double lidar_variance;
    double lidar_std_deviation;
    double  lidar_over3std_deviation_p;
    int lidar_time_run_count;


    constexpr int point_num = 144000;
    int count_iseuql;


    Plot(std::string sav_path_){
        sav_path=sav_path_;
    }

    //ReadGNSS()
    void readGNss(){
        std::ifstream gnss_file(sav_path+"/gnss_data.txt");
        std::string line;
        while (std::getline(gnss_file, line)) {
            if (line.empty()) {
                // 遇到空行，结束循环
                break;
            }
            // 处理非空行的逻辑
            std::istringstream isss(line);
            isss >> timestamp >> lla[0] >> lla[1] >> lla[2] >> lla_sigma[0] >> lla_sigma[1] >> roll
            >> pitch >> yaw >> rpy_sigma[0] >> rpy_sigma[1] >> rpy_sigma[2] >> imu_angular_v_raw[0]
            >> imu_angular_v_raw[1] >> imu_angular_v_raw[2] >> imu_linear_acc_raw[0]
            >> imu_linear_acc_raw[1] >> imu_linear_acc_raw[2] >> imu_angular_v_body[0]
            >> imu_angular_v_body[1] >> imu_angular_v_body[2] >> imu_linear_acc_body[0]
            >> imu_linear_acc_body[1] >> imu_linear_acc_body[2] >> velocity >> velocity_sigma
            >> gps_status >> wheel_speed[0] >> wheel_speed[1] >> wheel_speed[2] >> wheel_speed[3];

            v_timestamp.push_back(timestamp);
            v_lla.push_back(lla);
            v_lla_sigma.push_back(lla_sigma);
            v_roll.push_back(roll);
            v_pitch.push_back(pitch);
            v_yaw.push_back(yaw);
            v_rpy_sigma.push_back(rpy_sigma);
            v_imu_angular_v_raw.push_back(imu_angular_v_raw);
            v_imu_linear_acc_raw.push_back(imu_linear_acc_raw);
            v_imu_angular_v_body.push_back(imu_angular_v_body);
            v_imu_linear_acc_body.push_back(imu_linear_acc_body);
            v_imu_linear_acc_body.push_back(imu_linear_acc_body);
            v_velocity.push_back(velocity);
            v_wheel_speed.push_back(wheel_speed);
        }

        gnss_file.close();  // 关闭文件
    }

    void readLidar(){
        std::ifstream lidar_file(sav_path+"/lidar_data.txt");
        std::string line;
        while (std::getline(lidar_file, line)) {
            if (line.empty()) {
                // 遇到空行，结束循环
                break;
            }

            std::istringstream isss(line);
            isss >> lidar_timestamp >> lidar_num;
            v_lidar_timestamp.push_back(lidar_timestamp);
            v_lidar_num.push_back(lidar_num);
        }
        lidar_file.close();
    }

     void checkLidar(){
        std::ofstream check_lidar(sav_path+"/check_data.txt",std::ios::app);

         checkLidarTimestamp();
         check_lidar << "lidar:" <<std::endl<<"fre:"<< (1/average_lidar_time)<<std::endl
         <<"lidar_wrong_count:" << lidar_time_run_count << std::endl
           <<"std_deviation:"<< lidar_std_deviation <<std::endl
           <<"over3std_P:" << lidar_over3std_deviation_p*100 << "%" << std::endl;

         checkLidarPoint();
         check_lidar <<"count is not normal:"<< count_iseuql<<std::endl;
         check_lidar.close();
    }

    //public
    void checkGnss(){
        std::ofstream check_gnss(sav_path+"/check_data.txt",std::ios::app);
        checkGnssTimestamp();
        check_gnss << "gnss:" <<std::endl<<"fre:"<< (1/average_gnss_time)<<std::endl
        <<"gnss_wrong_count:" << gnss_time_run_count << std::endl
        <<"std_deviation:"<< gnss_std_deviation <<std::endl
        <<"over3std_P:" << over3std_deviation_p*100 << "%" << std::endl;

        check_gnss.close();
    }

    //private TODO 1111
    void checkLidarTimestamp(){
        double diff;
        lidar_time_run_count=0;
        for (size_t i = 1; i < v_lidar_timestamp.size(); i++) {
            diff = v_lidar_timestamp[i] - v_lidar_timestamp[i-1];  // 计算相邻元素的差值
            if(diff < 0) {lidar_time_run_count++;}
            diff_lidar_timestamp.push_back(diff);  // 将差值压入新vector
        }
        double sum = 0 ;
        for (const auto& diff_ :diff_lidar_timestamp ){
            sum += diff_ ;
        }
        average_lidar_time = sum/diff_lidar_timestamp.size();

        lidar_variance = 0.0;
        for (const auto& diff_ : diff_lidar_timestamp) {
            double diff__ = diff_ - average_lidar_time;
            lidar_variance += diff__ * diff__;
        }
        lidar_variance /= diff_lidar_timestamp.size();
        lidar_std_deviation = std::sqrt(lidar_variance);

        int over_cont_lidar_std_deviation = 0;
        for (const auto& diff_ : diff_lidar_timestamp) {
            if(std::abs(diff_ - average_lidar_time)>3*lidar_std_deviation){ over_cont_lidar_std_deviation++;}
        }
        lidar_over3std_deviation_p = over_cont_lidar_std_deviation / static_cast<double>(diff_lidar_timestamp.size());
    }

    void checkLidarPoint(){
        count_iseuql = 0;
        for(const auto& liadr_num_:v_lidar_num){
            if(liadr_num_ != point_num){
                count_iseuql++;
            }
        }
    }

    void checkGnssTimestamp(){
        double diff;
        gnss_time_run_count=0;
        for (size_t i = 1; i < v_timestamp.size(); i++) {
            diff = v_timestamp[i] - v_timestamp[i-1];  // 计算相邻元素的差值
            if(diff<0) { gnss_time_run_count++; }
            diff_gnss_timestamp.push_back(diff);  // 将差值压入新vector
        }
        double sum = 0 ;
        for (const auto& diff_ :diff_gnss_timestamp ){
            sum += diff_ ;
        }
        average_gnss_time = sum/diff_gnss_timestamp.size();

        gnss_variance = 0.0;
        for (const auto& diff_ : diff_gnss_timestamp) {
            double diff__ = diff_ - average_gnss_time;
            gnss_variance += diff__ * diff__;
        }
        gnss_variance /= diff_gnss_timestamp.size();
        gnss_std_deviation = std::sqrt(gnss_variance);

        int over_cont_gnss_std_deviation = 0;
        for (const auto& diff_ : diff_gnss_timestamp) {
            if(std::abs(diff_ - average_gnss_time)>3*gnss_std_deviation){ over_cont_gnss_std_deviation++;}
        }
        over3std_deviation_p = over_cont_gnss_std_deviation / static_cast<double>(diff_gnss_timestamp.size());
    }






};











#endif







