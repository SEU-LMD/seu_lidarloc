
// Use the Velodyne point format as a common representation
#ifndef SEU_LIDARLOC_IMUPREINTEGRATION_H
#define SEU_LIDARLOC_IMUPREINTEGRATION_H
// system
#include <mutex>
#include <thread>
#include <iostream>
// 3rdPatry
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
// homeMade
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "utils/MapSaver.h"
#include "utils/timer.h"
#include "udp_seralize.h"
#include "utils/udp_thread.h"
#include "utils/filesys.h"
#include "config/abs_current_path.h"

#define average_kmh2Ms 0.138888888

//TODO remove
//TODO change name to IMUWHEELDR
class IMUWHEELDR  {
public:
    PubSubInterface* pubsub;
    std::mutex odom_mutex;
    std::mutex gnssins_mutex;
    std::mutex lidarodom_mutex;
    std::function<void(const OdometryType&)> Function_AddDROdometryTypeToDataPreprocess;
    std::function<void(const DROdometryType&)> Function_AddDROdometryTypeToFuse;
    std::ofstream outfile;

    std::thread* do_lidar_thread;
    GeographicLib::LocalCartesian geoConverter;
    std::deque<GNSSINSType> deque_gnssins;

    std::string topic_imu_raw_odom = "/imu_odom_raw";
    std::string topic_imu_raw_odom_origin = "/imu_odom_raw_origin";

    bool systemInitialized = false;
    // 噪声协方差

    std::deque<IMURawDataPtr> imuQueOpt;
    std::deque<IMURawDataPtr> imuQueImu;

    Eigen::Vector3d prior_acc_bias;
    Eigen::Vector3d prior_gyro_bias;

    double lastImuT_imu = -1;
    int key = 1;
    OdometryType first_lidar_odom;
    StateData state,last_state;
    Eigen::Matrix3d R_w_b;
    double current_V;
    IMURawDataPtr cur_imu;

    std::shared_ptr<UDP_THREAD> udp_thread;
    bool flag_firstGNSSPoint = 0;
    PoseT First_Gnss_poseT;

    Eigen::Matrix3d deltaRotMat(const Eigen::Vector3d &delta_rot_vec) {
        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
        Eigen::Quaterniond delta_q;
        delta_q.w() = 1;
        delta_q.vec() = 0.5 * delta_rot_vec;
        delta_R = delta_q.toRotationMatrix();

        return delta_R;
    }

    Eigen::Matrix3d rotationUpdate(const Eigen::Matrix3d &Rwi,
                                   const Eigen::Matrix3d &delta_rot_mat) {
//        need debug
        Eigen::Matrix3d updatedR = Eigen::Matrix3d::Identity();
        updatedR = Rwi * delta_rot_mat;
        //updatedR = delta_rot_mat * Rwi;
        return updatedR;
    }

    void Wheel_predict(IMURawWheelDataPtr curr_imu){
        double imuTime = curr_imu->timestamp;
        double dt = (lastImuT_imu < 0) ? (1.0 / 100) : (imuTime - lastImuT_imu);
//        double dt = (lastImuT_imu < 0) ? (1.0 / SensorConfig::imuHZ) : (imuTime - lastImuT_imu);//TODO???? 1111----Done
        lastImuT_imu = imuTime;
//        state_.timestamp = curr_imu->timestamp;
//        const Eigen::Vector3d gyr_unbias =  curr_imu->imu_angular_v - prior_gyro_bias;  // gyro not unbias for now
        const Eigen::Vector3d gyr_unbias =  curr_imu->imu_angular_v_body;  // no bias
        const auto &dR = deltaRotMat(gyr_unbias * dt);
        Eigen::Vector3d state_v_b = Eigen::Vector3d (0, curr_imu->velocity,0);
//        state.v_wb_ = SensorConfig::T  state_v_b

        //pure DR
        state.Rwb_ = last_state.Rwb_;
        state.Rwb_ = rotationUpdate(state.Rwb_ , dR);
        Eigen::Vector3d state_v_w = state.Rwb_ * state_v_b;
        state.v_w_ = state_v_w;
        state.p_wb_ = last_state.p_wb_ + (last_state.v_w_ + state.v_w_)/2 *dt;//position
       // EZLOG(INFO)<<state.p_wb_.x()<< " "<< state.p_wb_.y() <<" "<<state.p_wb_.z()<<endl;

        last_state = state;
    }

    void Udp_OdomPub(const PoseT& data){
        Vis_Odometry odom_out;
        std::string fu_str;
        odom_out.type = "dr";
        odom_out.t[0]= data.GetXYZ().x();
        odom_out.t[1]= data.GetXYZ().y();
        odom_out.t[2]= data.GetXYZ().z();

        odom_out.q.x() = data.GetQ().x();
        odom_out.q.y() = data.GetQ().y();
        odom_out.q.z() = data.GetQ().z();
        odom_out.q.w() = data.GetQ().w();

        fu_str = odom_out.ToString();
        udp_thread -> SendUdpMSg(fu_str);
    }

    void AddGNSSINSData(const GNSSINSType& gnss_ins_data){

        if(IsFileDirExist(ABS_CURRENT_SOURCE_PATH+"/flag_rmAlldata")) {
            return;
        }

        TicToc time1;
        double currentTime = gnss_ins_data.timestamp;
        static bool init = false;

        if(!init)
        {
            double x,y,z;
            if(LocConfig::slam_mode_on == 1){ // loc
                std::ifstream downfile(LocConfig::save_map_path+"Origin.txt");  //打开文件
                std::string line; //字符串
                std::getline(downfile, line);//
                std::istringstream iss(line);
                iss >> x >> y >> z;
                EZLOG(INFO)<<"First GNSS position: "<<x<<", "<<y<<", "<<z;
                downfile.close(); // 关闭文件
                geoConverter.Reset(x, y, z);
            }
            else{ // mapping
                geoConverter.Reset(gnss_ins_data.lla[0], gnss_ins_data.lla[1], gnss_ins_data.lla[2]);
            }
            //TODO 1111 move below gnss code to here---Done
//            if(load_first_gnss_point == true){ return;}
            //gnss bad
            if(gnss_ins_data.lla[0] < 0 || gnss_ins_data.lla[1] < 0 || gnss_ins_data.lla[2] < 0){
//                EZLOG(INFO)<<"DR: GNSS value is bad: "<<gnss_ins_data.lla[0]<<","
//                                                <<gnss_ins_data.lla[1]<<","
//                                                <<gnss_ins_data.lla[2];
                return;
            }
            if(gnss_ins_data.gps_status != 42 && gnss_ins_data.gps_status != 52 && gnss_ins_data.gps_status != 33){
//                EZLOG(INFO)<<"DR: GNSS status is bad: "<<gnss_ins_data.gps_status;
                return;
            }
            // gnss good
            Eigen::Matrix3d z_matrix;
            Eigen::Matrix3d x_matrix;
            Eigen::Matrix3d y_matrix;
            double heading_Z = -gnss_ins_data.yaw * 0.017453293;
            double pitch_Y = gnss_ins_data.pitch * 0.017453293;
            double roll_X = gnss_ins_data.roll * 0.017453293;
            z_matrix << cos(heading_Z), -sin(heading_Z), 0,
                    sin(heading_Z), cos(heading_Z),  0,
                    0,                 0,            1;

            x_matrix << 1,                 0,              0,
                    0,            cos(pitch_Y),     -sin(pitch_Y),
                    0,            sin(pitch_Y),    cos(pitch_Y);

            y_matrix << cos(roll_X) ,     0,        sin(roll_X),
                    0,                 1,               0 ,
                    -sin(roll_X),      0,         cos(roll_X);
            double t_enu[3];
            geoConverter.Forward(gnss_ins_data.lla[0], gnss_ins_data.lla[1], gnss_ins_data.lla[2],
                                 t_enu[0], t_enu[1], t_enu[2]);//t_enu = enu coordiate
            Eigen::Matrix3d R_w_b =  (z_matrix*x_matrix*y_matrix);   // Pw = Twb * Pb zxy右前上，zyx前左上
            Eigen::Vector3d t_w_b(t_enu[0], t_enu[1], t_enu[2]);

            state.p_wb_ = t_w_b;
            state.Rwb_ = R_w_b;
                    state.v_w_[1] = gnss_ins_data.velocity; // gnss velocity
//            state.v_w_[1] = (gnss_ins_data.wheel_speed[0] + gnss_ins_data.wheel_speed[1]) * average_kmh2Ms; // gnss velocity, /2 /3.6
            last_state.p_wb_ = state.p_wb_;
            last_state.Rwb_ = state.Rwb_;
            last_state.v_w_ = state.v_w_;

            init = 1;

//            EZLOG(INFO)<<"RESET DR with GNSS: t_w_b"<<t_w_b.transpose();
            return;
        }

//        // update wheel
        static int gnss_cnt = 0;
//        // first init, or gnss_cnt>500,but gnss must 42 or 52
        if(IsFileDirExist(ABS_CURRENT_SOURCE_PATH+"/flag_gnss")){
            if(gnss_cnt > 500){
                if(gnss_ins_data.lla[0] > 0 && gnss_ins_data.lla[1] > 0 && gnss_ins_data.lla[2] > 0){
                    if(gnss_ins_data.gps_status ==42){
                        // gnss good
                        Eigen::Matrix3d z_matrix;
                        Eigen::Matrix3d x_matrix;
                        Eigen::Matrix3d y_matrix;
                        double heading_Z = -gnss_ins_data.yaw * 0.017453293;
                        double pitch_Y = gnss_ins_data.pitch * 0.017453293;
                        double roll_X = gnss_ins_data.roll * 0.017453293;
                        z_matrix << cos(heading_Z), -sin(heading_Z), 0,
                                sin(heading_Z), cos(heading_Z),  0,
                                0,                 0,            1;

                        x_matrix << 1,                 0,              0,
                                0,            cos(pitch_Y),     -sin(pitch_Y),
                                0,            sin(pitch_Y),    cos(pitch_Y);

                        y_matrix << cos(roll_X) ,     0,        sin(roll_X),
                                0,                 1,               0 ,
                                -sin(roll_X),      0,         cos(roll_X);
                        double t_enu[3];
                        geoConverter.Forward(gnss_ins_data.lla[0], gnss_ins_data.lla[1], gnss_ins_data.lla[2],
                                             t_enu[0], t_enu[1], t_enu[2]);//t_enu = enu coordiate
                        Eigen::Matrix3d R_w_b =  (z_matrix*x_matrix*y_matrix);   // Pw = Twb * Pb zxy右前上，zyx前左上
                        Eigen::Vector3d t_w_b(t_enu[0], t_enu[1], t_enu[2]);

                        state.p_wb_ = t_w_b;
                        state.Rwb_ = R_w_b;
                        state.v_w_[1] = gnss_ins_data.velocity; // gnss velocity
//                        state.v_w_[1] = (gnss_ins_data.wheel_speed[2] + gnss_ins_data.wheel_speed[3]) * average_kmh2Ms; // gnss velocity, /2 /3.6
                        last_state.p_wb_ = state.p_wb_;
                        last_state.Rwb_ = state.Rwb_;
                        last_state.v_w_ = state.v_w_;
                        gnss_cnt = 0;
                    }
                }
            }
            if(LocConfig::GNSSupdateDR == 1){
                gnss_cnt++;
            }
        }

        // DR----->>>>>>TO ENU
        IMURawWheelDataPtr imuWheel_raw (new IMURawWheelData);
        imuWheel_raw->imu_angular_v_body = gnss_ins_data.imu_angular_v_body * 0.017453293; //转弧度值
        imuWheel_raw->imu_linear_acc_body = gnss_ins_data.imu_linear_acc_body;
        imuWheel_raw->timestamp = gnss_ins_data.timestamp;
        imuWheel_raw->velocity = gnss_ins_data.velocity;
//        imuWheel_raw->velocity = (gnss_ins_data.wheel_speed[0] + gnss_ins_data.wheel_speed[1]) * average_kmh2Ms;
        Wheel_predict(imuWheel_raw);

        OdometryType Odometry_imuPredict_pub;
        OdometryType Odometry_imuPredict_pub_origin;
        DROdometryType DR_pose;

        Eigen::Matrix3d R_b_l;
        Eigen::Vector3d lidar_preditct_pose_p_wl_;
        Eigen::Matrix3d lidar_preditct_pose_Rwl_;
        PoseT lidar_preditct_pose_Twb_;
        Eigen::Matrix4d current_state_Rwb;
        R_b_l = SensorConfig::T_L_DR.block<3,3>(0, 0);
        lidar_preditct_pose_Rwl_ = state.Rwb_ * R_b_l;
        lidar_preditct_pose_p_wl_ = state.Rwb_ * (SensorConfig::T_L_B.inverse().block<3,1>(0, 3)) + state.p_wb_; // 地面存在高程误差——外参？ 建图？
//        lidar_preditct_pose_p_wl_ =  lidar_preditct_pose_p_wl_ + SensorConfig::T_L_DR.block<3,1>(0,3);
//        EZLOG(INFO) << lidar_preditct_pose_p_wl_.x() << " " << lidar_preditct_pose_p_wl_.y() << " " << lidar_preditct_pose_p_wl_.z() << endl;

        PoseT lidar_preditct_pose(lidar_preditct_pose_p_wl_, lidar_preditct_pose_Rwl_);
        Odometry_imuPredict_pub.pose = lidar_preditct_pose;
        DR_pose.pose = lidar_preditct_pose;

        //DR draw data
        if(FrontEndConfig::if_debug){
            Odometry_imuPredict_pub_origin.pose = PoseT(state.p_wb_,state.Rwb_);
            Odometry_imuPredict_pub_origin.timestamp = currentTime;
            Odometry_imuPredict_pub_origin.frame = "map";
            pubsub->PublishOdometry(topic_imu_raw_odom_origin, Odometry_imuPredict_pub_origin);
        }


        Odometry_imuPredict_pub.timestamp = currentTime;
        Odometry_imuPredict_pub.frame = "map";
        if(LocConfig::use_DR_or_fuse_in_loc == 1){
            Function_AddDROdometryTypeToDataPreprocess(Odometry_imuPredict_pub);
//            EZLOG(INFO)<<"1 of 2, DR send to data_preprocess. And Send Pose begin: ";
//            EZLOG(INFO)<<Odometry_imuPredict_pub.pose.pose;
//            EZLOG(INFO)<<"1 of 2, DR send to data_preprocess. And Send Pose end!";
        }

        //for debug use
        DR_pose.timestamp = currentTime;
        DR_pose.frame = "map";
        if(LocConfig::slam_mode_on == 1){
            Function_AddDROdometryTypeToFuse(DR_pose);
//            EZLOG(INFO)<<"2 of 2, DR send to fuse. And Send Pose begin: ";
//            EZLOG(INFO)<<DR_pose.pose.pose;
//            EZLOG(INFO)<<"2 of 2, DR send to fuse. And Send Pose end!";
            Udp_OdomPub(Odometry_imuPredict_pub.pose);
        }
       // Udp_OdomPub(Odometry_imuPredict_pub.pose);
       if(FrontEndConfig::if_debug){
           pubsub->PublishOdometry(topic_imu_raw_odom, Odometry_imuPredict_pub);
       }

    }

    void SetIMUPreParamter(){

        lastImuT_imu = -1;

        state.Rwb_ <<1,0,0,
                    0,1,0,
                    0,0,1;
        state.p_wb_ << 0,0,0;
        state.v_w_ << 0,0,0;

        last_state = state;
    }

    //TODO adjust seq to satisify default udp_thread intput!!!!!!----Done
    void Init(PubSubInterface* pubsub_,std::shared_ptr<UDP_THREAD> udp_thread_ = nullptr){
        //设置imu的参数
        SetIMUPreParamter();
        pubsub = pubsub_;
        udp_thread = udp_thread_;
        pubsub->addPublisher(topic_imu_raw_odom, DataType::ODOMETRY, 10);
        pubsub->addPublisher(topic_imu_raw_odom_origin, DataType::ODOMETRY, 10);
    }

};

#endif //SEU_LIDARLOC_IMUPREINTEGRATION_H
