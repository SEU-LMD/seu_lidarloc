#ifndef POSITION_H_
#define POSITION_H_


#include "bitwise.h"
#include "position_base.h"
#include "pubsub/data_types.h"
#include "timestamp.h"
#include "utils/utility.h"
#include <mutex>

// INITIALIZE_EASYLOGGINGPP


namespace shineauto {
namespace position {

class Position {
  public:
    CallBackT LidarFunction;
    CallBackT GNSSINSFunction;

    explicit Position(std::string configFile) : node_(configFile){};
 ~Position(){
      for (auto& it : threadPool_) {
        if (it.joinable()) {
            it.join();
        }
    }
 };

    Adsfi::HafStatus Init()
    {
        return node_.Init();
    };

    bool IsStop() const
    {
        return node_.IsStop();
    };

    void Stop()
    {
        node_.Stop();
    };
    void Process(){
          threadPool_.push_back(std::thread(&Position::SubPosition, this));
          threadPool_.push_back(std::thread(&Position::SubLidar, this));
          threadPool_.push_back(std::thread(&Position::SubCanData, this));
    };



private:
    PositionBase node_;
    std::vector<std::thread> threadPool_;
    std::mutex  wheel_mutx;
    double wheel_speed[4]={0};




    //GNSSINS thread to receive data
    void SubPosition(){
        double lat = -1.0;              // 纬度
        double lng = -1.0;              // 经度
        double alt = -1.0;              // 高度计
        std::shared_ptr<GNSSINSType> data_out = make_shared<GNSSINSType>();
        //EZLOG(INFO)<<"beign subpostiotin"<<std::endl;
        while (!node_.IsStop()){
            //EZLOG(INFO)<<"beign subpostiotin loop"<<std::endl;

            std::shared_ptr<HafCanRxFrame> recv_data;
            //EZLOG(INFO)<<" subpostiotin getdata"<<std::endl;
                    HafStatus ret = node_.GetPosition(recv_data);
            //EZLOG(INFO)<<"subpostiotin getdata end"<<std::endl;

            //自己的数据类型
                    if (ret == HAF_PROGRAM_STOPED)
                    {
                        EZLOG(INFO)<<"imu wrong"<<std::endl;
                        return;
                    }
                    if (ret == HAF_NOT_READY) {
                    HAF_LOG_ERROR << "Get Position Data Timeout";
                        //EZLOG(INFO)<<"Get Position Data Timeout"<<std::endl;
                        continue;
                    }
                    //EZLOG(INFO)<<"subpostion begin send"<<std::endl;
                    bool send = false;
                    for (size_t i = 0; i < recv_data->elements.size(); ++i) {

                        //EZLOG(INFO)<<"imu coming"<<std::endl;
                        HafCanRxBaseFrame can_frame = recv_data->elements[i];
                        uint32_t canId = can_frame.canId;
                        auto& data = can_frame.data;
                        if (canId == 801) {
                            int32_t angRawX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                angRawX -= (1 << 20);
                            }
                            data_out->imu_angular_v_raw[0] = static_cast<double>(angRawX) * 0.01;

                            int32_t angRawY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {
                                angRawY -= (1 << 20);
                            }
                            data_out->imu_angular_v_raw[1] = static_cast<double>(angRawY) * 0.01;

                            int32_t angRawZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                angRawZ -= (1 << 20);
                            }
                            data_out->imu_angular_v_raw[2] = static_cast<double>(angRawZ) * 0.01;
//                            HAF_LOG_INFO << "canId = 801, angRawX: " << data_out->imu_angular_v_raw[0] << ", angRawY: " << data_out->imu_angular_v_raw[1] << ", angRawZ:" << data_out->imu_angular_v_raw[2];
                        }
                        else if (canId == 802) {
                            int32_t accRawX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                accRawX -= (1 << 20);
                            }
                            data_out-> imu_linear_acc_raw[0] = static_cast<double>(accRawX) * 0.0001;

                            int32_t accRawY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {
                                accRawY -= (1 << 20);
                            }
                            data_out-> imu_linear_acc_raw[1] = static_cast<double>(accRawY) * 0.0001;

                            int32_t accRawZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                accRawZ -= (1 << 20);
                            }
                            data_out-> imu_linear_acc_raw[2] = static_cast<double>(accRawZ) * 0.0001;
//                            HAF_LOG_INFO << "canId = 802, accRawX: " << data_out-> imu_linear_acc_raw[0] << ", accRawY: " << data_out-> imu_linear_acc_raw[1] << ", accRawZ:" << data_out-> imu_linear_acc_raw[2];
                        }
                        else if (canId == 803) {  // 定位状态
                            data_out->gps_status = data[2] * 10 + data[0];
//                            HAF_LOG_INFO << "canId = 803, position_type: " << data_out->gps_status;
                        }
                        else if (canId == 805) {  // 大地高度
                            int32_t posAlt = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
                            alt = static_cast<double>(posAlt) * 0.001;
//                            HAF_LOG_INFO << ara::log::Setprecision(16) << "canId = 805, alt: " << data_out->lla[2];
                        }
                        else if (canId == 806) {
                            uint32_t posESigmaX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                posESigmaX -= (1 << 20);
                            }
                            data_out->lla_sigma[0] = static_cast<double>(posESigmaX) * 0.0001;

                            uint32_t posESigmaY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {
                                posESigmaY -= (1 << 20);
                            }
                            data_out->lla_sigma[1] = static_cast<double>(posESigmaY) * 0.0001;

                            uint32_t posESigmaZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                posESigmaZ -= (1 << 20);
                            }
                            data_out->lla_sigma[2] = static_cast<double>(posESigmaZ) * 0.0001;
//                            HAF_LOG_INFO << "canId = 806, posESigmaX: " << data_out->lla_sigma[0] << ", posESigmaY: " << data_out->lla_sigma[1]<< ", posESigmaZ:" << data_out->lla_sigma[2];
                        }
                        else if (canId == 807) {                        // 速度
                            int16_t carVel = data[6] + (data[7] << 8);  // 车辆速度，单位m/s
                            data_out->velocity = static_cast<double>(carVel) * 0.01;
//                            HAF_LOG_INFO << "canId = 807, vel: " << data_out->velocity;
                        }
                        else if (canId == 808) {                        // 速度标准差
                            uint16_t carVelSigma = data[6] + (data[7] << 8);  // 车辆速度，单位m/s
                            data_out->velocity_sigma = static_cast<double>(carVelSigma) * 0.001;
//                            HAF_LOG_INFO << "canId = 808, velSigma: " << data_out->velocity_sigma;
                        }
                        else if (canId == 809) {  // 车辆坐标系加速度
                            int32_t accelX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                accelX -= (1 << 20);
                            }
                            data_out->imu_linear_acc_body[0] = static_cast<double>(accelX) * 0.0001;

                            int32_t accelY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {
                                accelY -= (1 << 20);
                            }
                            data_out->imu_linear_acc_body[1] = static_cast<double>(accelY) * 0.0001;

                            int32_t accelZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                accelZ -= (1 << 20);
                            }
                            data_out->imu_linear_acc_body[2] = static_cast<double>(accelZ) * 0.0001;
//                            HAF_LOG_INFO << "canId = 809, accelX: " << data_out->imu_linear_acc_body[0] << ", accelY: " << data_out->imu_linear_acc_body[1] << ", accelZ:" << data_out->imu_linear_acc_body[2] ;
                        }
                        else if (canId == 810) {  // 姿态角
                            uint16_t angleHeading = data[0] + (data[1] << 8);
                            data_out->yaw = static_cast<double>(angleHeading) * 0.01;

                            int16_t anglePitch = data[2] + (data[3] << 8);
                            data_out->pitch = static_cast<double>(anglePitch) * 0.01;

                            int16_t angleRoll = data[4] + (data[5] << 8);
                            data_out->roll = static_cast<double>(angleRoll) * 0.01;

//                            HAF_LOG_INFO << ara::log::Setprecision(16) << "canId = 810, angleHeading: " << data_out->yaw
//                                         << ", anglePitch: " << data_out->pitch << ", angleRoll: " << data_out->roll;
                        }
                        else if (canId == 811) {
                            uint32_t angRPYSigmaX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                angRPYSigmaX -= (1 << 20);
                            }
                            data_out->rpy_sigma[0] = static_cast<float>(angRPYSigmaX) * 0.0001;

                            uint32_t angRPYSigmaY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {
                                angRPYSigmaY -= (1 << 20);
                            }
                            data_out->rpy_sigma[1] = static_cast<float>(angRPYSigmaY) * 0.0001;

                            uint32_t angRPYSigmaZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                angRPYSigmaZ -= (1 << 20);
                            }
                            data_out->rpy_sigma[2] = static_cast<float>(angRPYSigmaZ) * 0.0001;
//                            HAF_LOG_INFO << "canId = 811, angRPYSigmaX: " << data_out->rpy_sigma[0] << ", angRPYSigmaY: " << data_out->rpy_sigma[1]<< ", angRPYSigmaZ:" << data_out->rpy_sigma[2];
                        }
                        else if (canId == 812) {  // 车辆坐标系角速度
                            int32_t angRateX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                angRateX -= (1 << 20);
                            }
                            data_out->imu_angular_v_body[0] = static_cast<double>(angRateX) * 0.01;

                            int32_t angRateY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {
                                angRateY -= (1 << 20);
                            }
                            data_out->imu_angular_v_body[1] = static_cast<double>(angRateY) * 0.01;

                            int32_t angRateZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                angRateZ -= (1 << 20);
                            }
                            data_out->imu_angular_v_body[2] = static_cast<double>(angRateZ) * 0.01;

                            //HAF_LOG_INFO << "canId = 812, angRateX: " << data_out->imu_angular_v_body[0] << ", angRateY: " << data_out->imu_angular_v_body[1]
//                                         << ", angRateZ: " << data_out->imu_angular_v_body[2];
                            send = true;
                        }
                        else if (canId == 813) {  // 经度
                            uint32_t posLon = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);

                             lng= (static_cast<double>(posLon) + data[4] * 4294967296) * 0.00000001;

                            //HAF_LOG_INFO << "canId = 813, lng: " << ara::log::Setprecision(16) << data_out->lla[1];
                        }
                        else if (canId == 814) {  // 纬度
                            uint32_t posLat = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
                            lat = static_cast<double>(posLat) * 0.00000001;

                            //HAF_LOG_INFO << "canId = 814, lat: " << ara::log::Setprecision(16) << data_out->lla[0];
                        }
                    }
                    
                    if (!send ) {
                           continue;
                    }
                    else{
                        //std::cout << "begin fasong"<<std::endl;
                        data_out->timestamp = Adsfi::ToSecond(GetHafTimestamp());
                        //std::cout << "end fasong"<<std::endl;

                        data_out->frame="map";
                        data_out->lla[1] = lng;
                        data_out->lla[0] = lat;
                        data_out->lla[2] = alt;
                        {
                            std::lock_guard<std::mutex> lock(wheel_mutx);
                            data_out->wheel_speed[0] = wheel_speed[0];
                            data_out->wheel_speed[1] = wheel_speed[1];
                            data_out->wheel_speed[2] = wheel_speed[2];
                            data_out->wheel_speed[3] = wheel_speed[3];
                        }
                        //std::cout<<"begin callback" <<std::endl;
                        GNSSINSFunction(*data_out);
                        lat = -1.0;              // 纬度
                        lng = -1.0;              // 经度
                        alt = -1.0;              // 高度计
                    	//EZLOG(INFO)<<"imu_gnss_out"<<std::endl;
		    }
        }  //end while()
    };//end function  SubPosition

  //lidar thread to receive data
    void SubLidar(){
                size_t point_bytes = sizeof(Adsfi::PointXYZIRTV2);
                int lidarNum = 0;
                
                while (!node_.IsStop()) {
                    CloudTypeXYZIRTPtr  lidar_out =  make_shared<CloudTypeXYZIRT>(); 
                    std::shared_ptr<Adsfi::LidarFrameV2> data;
                    Adsfi::HafStatus ret = node_.GetLidar(data,UINT32_MAX);
                    if (ret == Adsfi::HAF_PROGRAM_STOPED)
                        break;
                    if (ret == Adsfi::HAF_SUCCESS) {
                        ++lidarNum;
                        //HAF_LOG_INFO << "Get Lidar Data Succeed, Seq = " << data->seq;
                        //EZLOG(INFO)<<"lidar_is_here"<<std::endl;
                        const uint8_t* buf = data->rawData.data();
                        size_t points_size = data->rawData.size() / point_bytes;
                        double min_time = 1.0 * 1e20;
                        double max_time = -1.0;
                        for (size_t i = 0U; i < points_size; ++i) {
                            PointXYZIRTSEU point;
                            Adsfi::PointXYZIRTV2* p = (Adsfi::PointXYZIRTV2*)(buf + point_bytes * i);
                            point.x=p->x;
                            point.y=p->y;
                            point.z=p->z;
                            point.ring=p->ring;
                            point.intensity=p->intensity;
                            point.latency=p->timestamp;
                            min_time = std::min(min_time,p->timestamp);
                            max_time = std::max(max_time,p->timestamp);
                            lidar_out->cloud.points.push_back(point);//fix by fyy
                        }
                            lidar_out->frame = "map";
                            lidar_out->timestamp =  Adsfi::ToSecond(data->timestamp);
                            LidarFunction(*lidar_out);
                            if(lidarNum>50){
                                EZLOG(INFO)<<"lidar per scan time : "<< max_time - min_time <<std::endl;
                                lidarNum = 0;
                            }

                      }
                }
    }//end function SubLidar

    void SubCanData(){
        while (!node_.IsStop()) {
            std::shared_ptr<HafCanRxFrame> frame;
            auto ret = node_.GetCanEPSData(frame);
            if (HAF_PROGRAM_STOPED == ret)
                break;
            for (auto el : frame->elements) {
                uint32_t canId = el.canId;
                if (canId == 0x23A) {
//                    EZLOG(INFO)<<"wheel_in"<<std::endl;
                    std::lock_guard<std::mutex> lock(wheel_mutx);
                    wheel_speed[0] = (((el.data[1] & 0x0F) << 9) + (el.data[2] << 1) + ((el.data[3] & 0x80) >> 7)) * 0.0563; // 右后轮速
                    wheel_speed[1] = (((el.data[3] & 0x7F) << 6) + ((el.data[4] & 0xFC) >> 2)) * 0.0563; // 左后轮速
                    wheel_speed[2]= (((el.data[4] & 0x03) << 11) + (el.data[5] << 3) + ((el.data[6] & 0xE0) >> 5)) * 0.0563; // 右前轮速
                    wheel_speed[3] = (((el.data[6] & 0x1F) << 8) + el.data[7]) * 0.0563; // 左前轮速
//                    EZLOG(INFO)<<"wheel_out"<<std::endl;
                }
            }
        }
        //HAF_LOG_INFO << "SubCanData exit";
    }//end SubCanData


}; //class Position
} // namespace position
}  // namespace shineauto

#endif
