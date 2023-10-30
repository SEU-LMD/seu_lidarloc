
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

struct datapoint{
    double timestamp;
    double velocity;
    double gnssx;
    double gnssy;
};

class DRCalibration  {
public:
    PubSubInterface* pubsub;
    std::mutex gnss_mutex;
    std::mutex dr_mutex;
    // std::mutex work_mutex;//TODO 1029 delete this mutex
    std::deque<OdometryType> gnssQueue;
    std::deque<WheelType> drQueue;
    std::thread* do_work_thread;

    TicToc timer_dr;

    bool init = false;
    GeographicLib::LocalCartesian geoConverter;

    std::string topic_gnss_odom_world = "/gnss_odom_world";
    std::string topic_dr_odom_world = "/dr_odom_world";

    int pieces=1000;  //矩阵大小
    std::vector<double> gnssdistances;

    std::vector<double> speedIntegration(const std::vector<double>& timestamps, const std::vector<double>& speeds){  //轮速计速度积分
        int n = timestamps.size();
        std::vector<double> distances(n,0.0);

        for (int i = 1; i<n; i++){
            double timeInterval = timestamps[i] - timestamps[i-1];
            distances[i] = distances[i-1] + speeds[i]*timeInterval;
        }
        return distances;
    }

    std::deque<datapoint> extractDataInRange(const std::deque<datapoint>& inputQueue, double startTime, double endTime){  //该函数提取指定时间段的队列
        std::deque<datapoint> extractData;
        int n= inputQueue.size();
        std::vector<double> distances(n,0.0);
        for(const auto &point : inputQueue){
            if(point.timestamp >=startTime && point.timestamp<=endTime){
                extractData.push_back(point);
            }
        }
        return extractData;
    }


    void DoWork(){
        while(1){


            if(timer_dr.toc()>5000){
                
                dr_mutex.lock();//hack operation just for use
                gnss_mutex.lock();
                
                std::vector<double> distances;

                std::vector<double> canbusdistances;
                std::vector<double> shorttimestamps;
                std::vector<double> shortvelcoities;

                //待输入数据
                std::deque<datapoint> canbusData;
                std::deque<datapoint> gnssData;

                std::deque<datapoint> shortcanbusData;
                std::deque<datapoint> shortgnssData;

                for(int i=0;i<drQueue.size();i++){
                    canbusData[i].timestamp=drQueue[i].timestamp;
                    canbusData[i].velocity=(drQueue[i].ESCWhlFLSpd+drQueue[i].ESCWhlFRSpd+drQueue[i].ESCWhlRLSpd+drQueue[i].ESCWhlRRSpd)/4;
                }

                for(int i=0;i<gnssQueue.size();i++){
                    gnssData[i].timestamp=gnssQueue[i].timestamp;
                    gnssData[i].gnssx=gnssQueue[i].pose.GetXYZ().x();
                    gnssData[i].gnssy=gnssQueue[i].pose.GetXYZ().y();
                }


                int divisor=0;
                int sum=0;
                int length = canbusData.size();
                divisor = length/pieces;

                shortcanbusData[0]=canbusData[0];
                shorttimestamps[0]=shortcanbusData[0].timestamp;
                shortvelcoities[0]=shortcanbusData[0].velocity;
                for(int i=1;i<pieces;i++){
                    shortcanbusData[i]=canbusData[i*divisor-1];
                    shorttimestamps[i]=shortcanbusData[i].timestamp;
                    shortvelcoities[i]=shortcanbusData[i].velocity;
                }
                canbusdistances = speedIntegration(shorttimestamps, shortvelcoities);  //得到canbus的距离

                std::deque<datapoint>temData;
                //std::deque<datapoint>temData=extractDataInRange(gnssData,0,shortcanbusData[0].timestamp);
                gnssdistances[0]=0;
                for(int i=1;i<pieces;i++){
                    temData=extractDataInRange(gnssData,shortcanbusData[i-1].timestamp,shortcanbusData[i].timestamp);
                    distances[0]=0;
                    for(int j=1;j<temData.size();i++){//?????TODO???
                        //TODO 1029
                        // Eigen::Vector3d pre();
                        // Eigen::Vector3 cur();
                        // double dist =  (cur-pre).norm();
                        // sum + = dist;
                        distances[j]=sqrt((temData[j].gnssx-temData[j-1].gnssx)*(temData[j].gnssx-temData[j-1].gnssx)+(temData[j].gnssy-temData[j-1].gnssy)*(temData[j].gnssy-temData[j-1].gnssy));
                        sum=sum+distances[j];
                    }
                    gnssdistances[i]=sum;  //得到gnss的距离
                    sum=0;
                    std::deque<datapoint>().swap(temData);//TODO delete!!!
                }

                //TODO 1029 not neccessary to use eigen to calc k and bias
                //just use distance to calc k directly.
                double k,bias;
                Eigen::MatrixXd A(pieces,2);
                Eigen::VectorXd b(pieces);

                for(int i=0;i<pieces;i++){
                    b(i)=gnssdistances[i];
                    A(i,1)=shorttimestamps[i];
                    A(i,0)=canbusdistances[i];
                }
                Eigen::VectorXd x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

                EZLOG(INFO)<<"k:"<<x(0,0);
                EZLOG(INFO)<<"bias:"<<x(1,0);//TODO 1029 erase bias

                exit(-1);//add by fyy
            }
            else{
                sleep(0.01);
            }
        }//end fucntion while(1)
    }//end fucntion do work



    void AddDrData(const WheelType& data){
        EZLOG(INFO)<<"featureext_AddCloudData  "<<std::endl;
        dr_mutex.lock();
        drQueue.push_back(data);
        timer_dr.tic();
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

    }

};
#endif
