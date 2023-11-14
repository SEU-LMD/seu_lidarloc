#ifndef POSITION_H_
#define POSITION_H_


#include "bitwise.h"
#include "position_base.h"
#include "pubsub/data_types.h"
#include "timestamp.h"
#include "utils/utility.h"

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


    void pubilsh(const std::string &topic_name, const OdometryType &data){
        HafTime timestamp = GetHafTimestamp();
        auto out = std::make_shared<HafLocation>();
        if(topic_name=="/lidar_odometry")
        out->header.seq = 1;
        else if(topic_name=="/imu_odom_raw")
        out->header.seq = 2;
        else if(topic_name=="/gnss_odom_world")
        out->header.seq = 3;
        out->header.timestamp = timestamp;
        out->header.frameID = "location";
        // out->pose.pose.position.x = lat;  // 纬度信息,latdata.pose.GetXYZ[0]
        // out->pose.pose.position.y = lng;  // 经度信息,lon
        // out->pose.pose.position.z = alt;  // 海拔信息,height
        Eigen::Vector3d t_data = data.pose.GetXYZ();
        out->pose.pose.position.x = t_data[0];  // 纬度信息,latdata.pose.GetXYZ[0]
        out->pose.pose.position.y = t_data[1];  // 经度信息,lon
        out->pose.pose.position.z = t_data[2];  // 海拔信息,height

        Eigen::Quaterniond Q_data = data.pose.GetQ();
        out->pose.pose.orientation.x = Q_data.x();
        out->pose.pose.orientation.y = Q_data.y();
        out->pose.pose.orientation.z = Q_data.z();
        out->pose.pose.orientation.w = Q_data.w();

        //check order;

        out->odomType = 0;

        if (node_.SendLocation(out) != Adsfi::HAF_SUCCESS) {
            HAF_LOG_ERROR << "Send Location Failed!";
        }
    };


private:
    PositionBase node_;
    std::vector<std::thread> threadPool_;

    //GNSSINS thread to receive data
    void SubPosition(){
                      while (!node_.IsStop()){
                    std::shared_ptr<HafCanRxFrame> recv_data;
                    HafStatus ret = node_.GetPosition(recv_data);
                    //自己的数据类型
                    std::shared_ptr<GNSSINSType> data_out = make_shared<GNSSINSType>();
                    if (ret == HAF_PROGRAM_STOPED)
                    return;
                    if (ret == HAF_NOT_READY) {
                    HAF_LOG_ERROR << "Get Position Data Timeout";
                    continue;
                    }
                    bool send = false;
                    for (size_t i = 0; i < recv_data->elements.size(); ++i) {
                        HafCanRxBaseFrame can_frame = recv_data->elements[i];
                        uint32_t canId = can_frame.canId;
                        auto& data = can_frame.data;
                          if (canId == 812) {  //原始角速度
                            int32_t angRawX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                angRawX -= (1 << 20);
                            }
                            data_out->imu_angular_v[0] = static_cast<double>(angRawX) * 0.01;

                            int32_t angRawY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {  
                                angRawY -= (1 << 20);
                            }
                            data_out->imu_angular_v[1] = static_cast<double>(angRawY) * 0.01;

                            int32_t angRawZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                angRawZ -= (1 << 20);
                            }
                            data_out->imu_angular_v[2] = static_cast<double>(angRawZ) * 0.01;
                        }
                        else if (canId == 809) { //原始加速度
                            int32_t accRawX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                accRawX -= (1 << 20);
                            }
                            data_out->imu_linear_acc[0] = static_cast<double>(accRawX) * 0.0001;

                            int32_t accRawY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {
                                accRawY -= (1 << 20);
                            }
                            data_out->imu_linear_acc[1]  = static_cast<double>(accRawY) * 0.0001;

                            int32_t accRawZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                accRawZ -= (1 << 20);
                            }
                            data_out->imu_linear_acc[2]  = static_cast<double>(accRawZ) * 0.0001;
                        }
                        else if (canId == 803) {  // 定位状态
                            int system_state = data[0];
                            int satenum = data[1];          // 卫星数量
                            int satellite_state = data[2];  // 卫星状态
                            if (system_state == 2 && satellite_state == 4) {
                                data_out->gps_status = "42";
                            }
                            else {
                                data_out->gps_status = "0";
                            }
                        }
                        else if (canId == 805) {  // 大地高度
                            int32_t posAlt = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
                            data_out->lla[2] = static_cast<double>(posAlt) * 0.001;
                        }
                        else if (canId == 806) {  //位置标准差  东北天
                            uint32_t posESigmaX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                posESigmaX -= (1 << 20);
                            }
                            data_out->cov[0] = static_cast<double>(posESigmaX) * 0.0001;

                            uint32_t posESigmaY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {
                                posESigmaY -= (1 << 20);
                            }
                            data_out->cov[1] = static_cast<double>(posESigmaY) * 0.0001;

                            uint32_t posESigmaZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                posESigmaZ -= (1 << 20);
                            }
                            data_out->cov[2] = static_cast<double>(posESigmaZ) * 0.0001;

                        }
                        else if (canId == 810) {  // 姿态角
                            uint16_t angleHeading = data[0] + (data[1] << 8);
                            data_out->yaw = static_cast<double>(angleHeading) * 0.01;

                            int16_t anglePitch = data[2] + (data[3] << 8);
                            data_out->pitch = static_cast<double>(anglePitch) * 0.01;

                            int16_t angleRoll = data[4] + (data[5] << 8);
                            data_out->roll= static_cast<double>(angleRoll) * 0.01;
                        }
                        else if (canId == 811) {  //姿态角度  yaw pitch roll
                            uint32_t angRPYSigmaX = data[0] + (data[1] << 8) + ((data[2] & 0x0F) << 16);
                            if (CHECK_BIT(data[2], 3)) {
                                angRPYSigmaX -= (1 << 20);
                            }
                            data_out->cov[3] = static_cast<double>(angRPYSigmaX) * 0.0001;
                            uint32_t angRPYSigmaY = ((data[2] & 0xf0) >> 4) + (data[3] << 4) + ((data[4] & 0xFF) << 12);
                            if (CHECK_BIT(data[4], 7)) {
                                angRPYSigmaY -= (1 << 20);
                            }
                            data_out->cov[4] = static_cast<double>(angRPYSigmaY) * 0.0001;

                            uint32_t angRPYSigmaZ = data[5] + (data[6] << 8) + ((data[7] & 0x0F) << 16);
                            if (CHECK_BIT(data[7], 3)) {
                                angRPYSigmaZ -= (1 << 20);
                            }
                            data_out->cov[5] = static_cast<double>(angRPYSigmaZ) * 0.0001;
                        }
                        else if (canId == 813) {  // 经度
                            uint32_t posLon = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
                            data_out->lla[0] = (static_cast<double>(posLon) + data[4] * 4294967296) * 0.00000001;
                        }
                        else if (canId == 814) {  // 纬度
                        uint32_t posLat = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
                       data_out->lla[1]= static_cast<double>(posLat) * 0.00000001;
                        send = true;
                    }
                    }
                    
                    if (!send || std::fabs(data_out->lla[0] ) < 0.1) {
                           continue;
                    }
                    else{
                        data_out->timestamp = Adsfi::ToSecond(GetHafTimestamp());
                        data_out->frame="map";
                        GNSSINSFunction(*data_out);
                    	EZLOG(INFO)<<"imu_gnss"<<std::endl;
		    }
                }  //end while()         
  };//end function  SubPosition

  //lidar thread to receive data
    void SubLidar(){
                size_t point_bytes = sizeof(Adsfi::PointXYZIRTV2);
                
                while (!node_.IsStop()) {
                    CloudTypeXYZIRTPtr  lidar_out =  make_shared<CloudTypeXYZIRT>(); 
                    std::shared_ptr<Adsfi::LidarFrameV2> data;
                    Adsfi::HafStatus ret = node_.GetLidar(data);
                    if (ret == Adsfi::HAF_PROGRAM_STOPED)
                        break;
                    if (ret == Adsfi::HAF_SUCCESS) { 
                        HAF_LOG_INFO << "Get Lidar Data Succeed, Seq = " << data->seq;
                        std::cout<<"lidar_is_here"<<std::endl;
                        const uint8_t* buf = data->rawData.data();
                        size_t points_size = data->rawData.size() / point_bytes;
                        for (size_t i = 0U; i < points_size; ++i) {
                            PointXYZIRTSEU point;
                            Adsfi::PointXYZIRTV2* p = (Adsfi::PointXYZIRTV2*)(buf + point_bytes * i);
                            point.x=p->x;
                            point.y=p->y;
                            point.z=p->z;
                            point.ring=p->ring;
                            point.intensity=p->intensity;
                            point.latency=p->timestamp;
                            lidar_out->cloud.points.push_back(point);//fix by fyy
                        }
                            lidar_out->frame = "map";
                            lidar_out->timestamp =  Adsfi::ToSecond(data->timestamp);
                            LidarFunction(*lidar_out);
			               EZLOG(INFO)<<"lidar_out"<<std::endl;
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
                    whl_spd_.ESCWhlRRSpd = (((el.data[1] & 0x0F) << 9) + (el.data[2] << 1) + ((el.data[3] & 0x80) >> 7)) * 0.0563; // 右后轮速
                    whl_spd_.ESCWhlRLSpd = (((el.data[3] & 0x7F) << 6) + ((el.data[4] & 0xFC) >> 2)) * 0.0563; // 左后轮速
                    whl_spd_.ESCWhlFRSpd = (((el.data[4] & 0x03) << 11) + (el.data[5] << 3) + ((el.data[6] & 0xE0) >> 5)) * 0.0563; // 右前轮速
                    whl_spd_.ESCWhlFLSpd = (((el.data[6] & 0x1F) << 8) + el.data[7]) * 0.0563; // 左前轮速
                    HAF_LOG_INFO << "recv can eps data, ESCWhlRRSpd = " << whl_spd_.ESCWhlRRSpd << ", ESCWhlRLSpd = " << whl_spd_.ESCWhlRLSpd
                                 << ", ESCWhlFRSpd = " << whl_spd_.ESCWhlFRSpd << ", ESCWhlFLSpd = " << whl_spd_.ESCWhlFLSpd;
                }
            }
        }
        HAF_LOG_INFO << "SubCanData exit";
    }//end SubCanData


}; //class Position
} // namespace position
}  // namespace shineauto

#endif
