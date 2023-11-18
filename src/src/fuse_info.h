//
// Created by fyy on 23-10-12.
//

#ifndef SEU_LIDARLOC_FUSE_INFO_H
#define SEU_LIDARLOC_FUSE_INFO_H
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "utils/timer.h"

#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/ISAM2Params.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"//TODO 1111
#include "gtsam/slam/dataset.h"  // gtsam
#include "math.h"

#include "udp_seralize.h"
#include "utils/udp_thread.h"
#include "utils/filesys.h"
#include "config/abs_current_path.h"


class Fuse{
public:
    PubSubInterface* pubsub;
    std::thread* HighFrequencyLoc_thread;
    std::deque<std::shared_ptr<BaseType>> data_deque;//TODO 1111 what is the meaning of, all type of data---Done
    std::deque<std::shared_ptr<DROdometryType>> DR_data_deque;
    std::deque<std::shared_ptr<GNSSOdometryType>> Gnss_data_deque;
    std::function<void(const OdometryType&)> Function_AddLidarOdometryTypeToDR;
    std::function<void(const OdometryType&)> Function_AddLidarOdometryTypeToMapManager;
    std::function<void(const OdometryType&)> Function_AddOdometryTypeTodataPreprocess;
    std::mutex mutex_data;//TODO 1111 what is the meaning of data, all type of data---Done

    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::ISAM2 *isam;
    gtsam::Values initialEstimate;
    gtsam::Pose3 Lidar_T_w_Li_1_factor;
    gtsam::Pose3 current_DR_pose_forDRFactor;
    gtsam::Pose3 DR_T_w_bi_factor;
    PoseT DR_T_w_bi_1_factor;

    gtsam::Values isamCurrentEstimate; // 所有关键帧位姿的优化结果
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr GNSSNoise;
    gtsam::noiseModel::Diagonal::shared_ptr DRNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;

    PoseT delta_pose_DR;
    PoseT current_T_i_0;

    int key = 1;
    int lidar_keyFrame_cnt = 0;
    int lidar_keyFrame_frameid = 0;
    OdometryType loc_result;
    OdometryType current_lidar_world;
    PoseT last_pose_fromRollBack;
    PoseT last_DR_pose;


    bool if_asb_loc_arrived = 0;//TODO :if abs loc arrived

    std::string topic_highHz_pose = "/loc_result";
    std::string topic_testforRollBack_pose = "/loc_result_roll_back";
    std::string topic_current_lidar_pose = "/loc_lidar_result";

    std::shared_ptr<UDP_THREAD> udp_thread;


    //this fucntion needs to be binded by lidar loc node
    void AddLidarLocToFuse(const OdometryType &lidar_loc_res){
        mutex_data.lock();
        std::shared_ptr<BaseType> odometryPtr = std::make_shared<OdometryType>(lidar_loc_res);
        data_deque.push_back(odometryPtr);
        mutex_data.unlock();
    }

    void AddIMUToFuse(const DROdometryType &imu_odom){
        mutex_data.lock();
        std::shared_ptr<BaseType> odometryPtr = std::make_shared<DROdometryType>(imu_odom);
        data_deque.push_back(odometryPtr);
        mutex_data.unlock();
    }

    void fuseInitialized(){
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(parameters);

        priorNoise = gtsam::noiseModel::Diagonal::Variances(
                        (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI,
                                                         1e8, 1e8, 1e8).finished());  // rad*rad, meter*meter
        odometryNoise = gtsam::noiseModel::Diagonal::Variances(
                        (gtsam::Vector(6) << LocConfig::LidarOdom_noise_roll,
                                                         LocConfig::LidarOdom_noise_pitch,
                                                         LocConfig::LidarOdom_noise_yaw,
                                                         LocConfig::LidarOdom_noise_x,
                                                         LocConfig::LidarOdom_noise_y,
                                                         LocConfig::LidarOdom_noise_z).finished());
        GNSSNoise = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << LocConfig::GNSS_noise_roll,
                                                 LocConfig::GNSS_noise_pitch,
                                                 LocConfig::GNSS_noise_yaw,
                                                LocConfig::GNSS_noise_x,
                                                LocConfig::GNSS_noise_y,
                                                LocConfig::GNSS_noise_z).finished());
        DRNoise = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << LocConfig::DR_noise_roll,
                                                LocConfig::DR_noise_pitch,
                                                LocConfig::DR_noise_yaw,
                                                 LocConfig::DR_noise_x,
                                                 LocConfig::DR_noise_y,
                                                 LocConfig::GNSS_noise_z).finished());
        last_pose_fromRollBack = PoseT(Eigen::Matrix4d::Identity());

        EZLOG(INFO)<<" Fuse Init Successful!";
    }

    void RollBack(const OdometryType &_current_pose,
                  const PoseT &poseFrom,
                  const PoseT &poseTo,
                  const double &roll_back_timestamp,
                  OdometryType &_current_pose_align,
                  PoseT &_last_pose_fromRollBack){

                { // for debug use
                    _current_pose_align.frame = "map";
                    _current_pose_align.timestamp = roll_back_timestamp;
                    _current_pose_align.pose = PoseT(_current_pose.pose * poseFrom.between(poseTo));
                    pubsub->PublishOdometry(topic_testforRollBack_pose, _current_pose_align);
                }
                if_asb_loc_arrived = 1;
                _last_pose_fromRollBack = PoseT(_current_pose.pose * poseFrom.between(poseTo));

    }

    void Udp_OdomPub(const PoseT& odom_in){
        Vis_Odometry odom_out;
        std::string fu_str;
        odom_out.type = "fu";
        odom_out.t[0]= odom_in.GetXYZ().x();
        odom_out.t[1]= odom_in.GetXYZ().y();
        odom_out.t[2]= odom_in.GetXYZ().z();

        odom_out.q.x() = odom_in.GetQ().x();
        odom_out.q.y() = odom_in.GetQ().y();
        odom_out.q.z() = odom_in.GetQ().z();
        odom_out.q.w() = odom_in.GetQ().w();

        fu_str = odom_out.ToString();
        udp_thread -> SendUdpMSg(fu_str);
    }


    //bool TODO 1118
    void DRAlignWithLidarAndClear(std::deque<std::shared_ptr<DROdometryType>> &_DR_data_deque,//TODO 1118 add const
                                  double &current_lidar_time,//TODO 1118 add const
                                  PoseT &current_DR_pose,
                                  PoseT &last_DR_pose
                                  ){
        while(!_DR_data_deque.empty()){
        
            if(current_lidar_time - _DR_data_deque.front()->timestamp < 0.01f){
                current_DR_pose = _DR_data_deque.front()->pose;
                last_DR_pose = _DR_data_deque.back()->pose;
                //return true;
                break;
            }
            else{
                _DR_data_deque.pop_front();
            }
        }
        //return false;
    }

    void Init(PubSubInterface* pubsub_,std::shared_ptr<UDP_THREAD> udp_thread_ = nullptr){
        pubsub = pubsub_;
        udp_thread = udp_thread_;
        fuseInitialized();

        //TODO 1111 MDC???
        pubsub->addPublisher(topic_highHz_pose, DataType::ODOMETRY, 10);
        pubsub->addPublisher(topic_current_lidar_pose, DataType::ODOMETRY, 10);
        pubsub->addPublisher(topic_testforRollBack_pose, DataType::ODOMETRY, 10);

        HighFrequencyLoc_thread = new std::thread(&Fuse::DoWork, this);
    }

    void DoWork(){
        while(1){
            if(data_deque.size()!=0){

                mutex_data.lock();
                auto front_data = data_deque.front();
                data_deque.pop_front();
                mutex_data.unlock();
                double time_Li = front_data->timestamp;
                TicToc t_fuse;
                switch (front_data->getType()) {
                    //from lidar
                    case DataType::ODOMETRY:
                    {
                        PoseT DR_T_w_bi_rollback;
                        PoseT DR_T_w_bc_rollback;

                        OdometryTypePtr Odom_Li_temp;
                        Odom_Li_temp = std::static_pointer_cast<OdometryType>(std::move(front_data));
                        lidar_keyFrame_frameid = Odom_Li_temp->frame_cnt;

                        gtsam::Pose3 Lidar_T_w_Li_factor(gtsam::Rot3(Odom_Li_temp->pose.GetR()),
                                                         gtsam::Point3(Odom_Li_temp->pose.GetXYZ()));

                        if(lidar_keyFrame_cnt == 0){

                            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, Lidar_T_w_Li_factor,
                                                                            priorNoise));
                            initialEstimate.insert(0, Lidar_T_w_Li_factor);

                            Lidar_T_w_Li_1_factor = Lidar_T_w_Li_factor;
                            DR_T_w_bi_1_factor = Lidar_T_w_Li_factor.matrix();

                            lidar_keyFrame_cnt++;
                            EZLOG(INFO)<<"GTSAM Optimization Init Successful!";
                            break;
                        }
                        else{
//                            add DR between factor
//                          ------Li-DRi------DRc-
                            DRAlignWithLidarAndClear(DR_data_deque, time_Li, DR_T_w_bi_rollback, DR_T_w_bc_rollback);
//                          ADD DR factor
//                          ------Li_1-DRi_1 --------Li-Di--------
                            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                                    lidar_keyFrame_cnt - 1, lidar_keyFrame_cnt,
                                    gtsam::Pose3(DR_T_w_bi_1_factor.between(DR_T_w_bi_rollback).pose), DRNoise));

                            DR_T_w_bi_1_factor = DR_T_w_bi_rollback;

//                          ADD Lidar factor
//                          ------Li_1-DRi_1 --------Li-Di--------
                            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                                    lidar_keyFrame_cnt - 1, lidar_keyFrame_cnt,
                                    Lidar_T_w_Li_1_factor.between(Lidar_T_w_Li_factor), odometryNoise));
                            initialEstimate.insert(lidar_keyFrame_cnt, Lidar_T_w_Li_factor);


//                          ADD GNSS factor
//                          ------Gi_1 Li_1-DRi_1 --------Gi-Li-Di--------
                            if(Odom_Li_temp->GTpose_reliability == true
                            && IsFileDirExist(ABS_CURRENT_SOURCE_PATH+"/flag_gnss")){
                                gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(
                                lidar_keyFrame_cnt,
                                gtsam::Pose3(gtsam::Rot3(Odom_Li_temp->GTpose.GetR()) ,
                                             gtsam::Point3(Odom_Li_temp->GTpose.GetXYZ())),
                                GNSSNoise));
                            }
                            else{
                                EZLOG(INFO)<<"current gnss is not used! reason are here: "<<Odom_Li_temp->GTpose_reliability;
                                EZLOG(INFO)<<"IsFileDirExist(flag_gnss): "
                                           <<IsFileDirExist(ABS_CURRENT_SOURCE_PATH+"/flag_gnss");
                                EZLOG(INFO)<<"Odom_Li_temp->GTpose: "<<Odom_Li_temp->GTpose.GetXYZ().transpose();
                            }
                        }

                        EZLOG(INFO) << "****************************************************";
                        gtSAMgraph.print("Fuse GTSAM Graph:\n");
                        // update iSAM
                        isam->update(gtSAMgraph, initialEstimate);
                        isam->update();
                        gtSAMgraph.resize(0);
                        initialEstimate.clear();

                        gtsam::Pose3 latestEstimate;
                        isamCurrentEstimate = isam->calculateEstimate();
                        latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
                        EZLOG(INFO) << "****************************************************";
//                        isamCurrentEstimate.print("Fuse Current estimate: ");

                        PoseT current_pose(latestEstimate.translation(),latestEstimate.rotation().matrix());
                        current_lidar_world.frame = "map";
                        current_lidar_world.timestamp = time_Li;
                        current_lidar_world.pose.pose = current_pose.pose;
                        pubsub->PublishOdometry(topic_current_lidar_pose, current_lidar_world);

                        RollBack(current_lidar_world,
                                 DR_T_w_bi_rollback,
                                 DR_T_w_bc_rollback,
                                 DR_data_deque.back()->timestamp,
                                 loc_result,
                                 last_pose_fromRollBack);

                        Lidar_T_w_Li_1_factor = Lidar_T_w_Li_factor;
                        lidar_keyFrame_cnt++;

                        break;
                    }

                    //TODO change name
                    //TODO 1111 add beteween factor when lidar and gnss arrives!!!!!!!!
                    case DataType::DR:
                    {
                        // 1. wait for first gnss pose
                        DROdometryTypePtr  cur_Wheel_odom;
                        cur_Wheel_odom = std::static_pointer_cast<DROdometryType>(std::move(front_data));
                        DR_data_deque.push_back(cur_Wheel_odom);
                        static PoseT last_DR_pose_for_predict = cur_Wheel_odom->pose;

                        // 2. cal dR of DR
                        gtsam::Pose3 current_Wheel_pose(gtsam::Rot3(cur_Wheel_odom->pose.GetR()),
                                                        gtsam::Point3(cur_Wheel_odom->pose.GetXYZ()));

                        PoseT current_DR_pose(current_Wheel_pose.translation(),current_Wheel_pose.rotation().matrix());
                        delta_pose_DR = last_DR_pose_for_predict.between(current_DR_pose);

                        // 3.iteration of DR: update iteration init point when rollback
                        loc_result.frame = "map";
                        loc_result.timestamp = time_Li;
                        if(if_asb_loc_arrived){// predict from last rollback
                            loc_result.pose.pose = last_pose_fromRollBack.pose * delta_pose_DR.pose;
                            last_pose_fromRollBack = loc_result.pose.pose;
                        }else{ // normal predict
                            loc_result.pose.pose = last_DR_pose_for_predict.pose * delta_pose_DR.pose;
                        }
                        // 4.iteration settings and pub the high frequency loc result
                        pubsub->PublishOdometry(topic_highHz_pose, loc_result);
                        Function_AddLidarOdometryTypeToMapManager(loc_result);//TODO 1111 not used anymore
                        EZLOG(INFO) << "5 Fuse to map_loder, and the loc_result pose :"<<loc_result.pose.GetXYZ().transpose();
                        if(LocConfig::use_DR_or_fuse_in_loc == 0){
                            Function_AddOdometryTypeTodataPreprocess(loc_result);
                        }
                        // TODO loc_result->>>>>>> is high frequency loc result need UDP
                        Udp_OdomPub(current_DR_pose);
                        last_DR_pose_for_predict = current_DR_pose;
                        break;
                    }
                }
                EZLOG(INFO)<<"Fuse cost in ms: "<<t_fuse.toc();
            }
            else{
                sleep(0.001);
            }
        }
    }//end fucntion do work


};
#endif //SEU_LIDARLOC_FUSE_INFO_H
