
#ifndef SEU_LIDARLOC_DRCALIBRATION_H
#define SEU_LIDARLOC_DRCALIBRATION_H
#include <mutex>
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "utils/timer.h"
#include "opt_mapping.h"
#include "opt_lopc.h"
#include "imageProjection.h"
#include "utils/MapSaver.h"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"

class DRCalibration  {
public:
    PubSubInterface* pubsub;
    std::mutex gnss_mutex;
    std::mutex dr_mutex;
    std::mutex work_mutex;
    std::deque<OdometryType> gnssQueue;
    std::deque<DROdometryType> drQueue;
    std::thread* do_work_thread;

    bool init = false;
    GeographicLib::LocalCartesian geoConverter;

    std::string topic_gnss_odom_world = "/gnss_odom_world";
    std::string topic_dr_odom_world = "/dr_odom_world";

    void DoWork(){
        while(1){
           // EZLOG(INFO)<<"featureext_DoWork while "<<std::endl;
            bool isempty = false;
            {
                std::lock_guard<std::mutex> lock(work_mutex);
//                isempty = deque_cloud.empty();
            }

            if(!isempty){




            }

            else{
                sleep(0.01);
            }
        }
    }



    void AddDrData(const DROdometryType& data){
        EZLOG(INFO)<<"featureext_AddCloudData  "<<std::endl;
        dr_mutex.lock();
        drQueue.push_back(data);
        dr_mutex.unlock();
    }

    void AddGNSSINSData(const GNSSINSType& data){

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
//        T_w_l:
//        -0.973379  -0.229171 0.00382091   -128.353
//        0.2292  -0.973157  0.0207947 0.00482607
//        -0.0010472  0.0211169   0.999776  0.0740949
//        0          0          0          1
//        T_w_b:
//        0.229171   -0.973379  0.00382091    -128.355
//        0.973157      0.2292   0.0207947 -0.00585202
//        -0.0211169  -0.0010472    0.999776    -0.43929
//        0           0           0           1

        OdometryType T_w_l_pub;
        T_w_l_pub.frame = "map";
        T_w_l_pub.timestamp = data.timestamp;
        T_w_l_pub.pose = T_w_l;

        GNSSINSType T_w_l_to_mapopt;
        T_w_l_to_mapopt.lla[0] = T_w_l.GetXYZ()[0];
        T_w_l_to_mapopt.lla[1] = T_w_l.GetXYZ()[1];
        T_w_l_to_mapopt.lla[2] = T_w_l.GetXYZ()[2];
//        opt_mapping_ptr->AddGNSSINSData(T_w_l_to_mapopt);

        gnss_mutex.lock();
        gnssQueue.push_back(T_w_l_pub);
        gnss_mutex.unlock();

        GNSSOdometryType T_w_l_gnss;
        T_w_l_gnss.frame = "map";
        T_w_l_gnss.timestamp = data.timestamp;
        T_w_l_gnss.pose = T_w_l;
        pubsub->PublishOdometry(topic_gnss_odom_world, T_w_l_pub);

    }

    void Init(PubSubInterface* pubsub_){

        pubsub = pubsub_;
        pubsub->addPublisher(topic_gnss_odom_world, DataType::ODOMETRY,2000);
        pubsub->addPublisher(topic_dr_odom_world, DataType::ODOMETRY,2000);
        do_work_thread = new std::thread(&DRCalibration::DoWork, this);
        if(MappingConfig::slam_mode_switch == 0){
            //save_Map_thread = new std::thread(&MapSaver::do_work, &(FeatureExtraction::map_saver));
        }


    }

};
#endif
