//
// Created by fyy on 23-9-9.
//

#include "utils/config_helper.h"
#include "easylogging++.h"
#include "./loc_manager.h"
#include "utils/filesys.h"
#include "utils/udp_thread.h"

//选择中间件
#define X86
#ifdef X86
#include "pubsub/ros/ros_pubsub.h"
#else
#include "pubsub/mdc/mdc_pubsub.h"
#endif
#include "config/abs_current_path.h"


INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {

    CreateDirWithDelete(ABS_CURRENT_SOURCE_PATH+"/log");//TODO 1111 change to absolute path

    //1.初始化 log
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%level %file %line : %msg");
#ifdef ELPP_THREAD_SAFE
    EZLOG(INFO) << "easylogging++ thread safe!";
#else
    EZLOG(INFO) << "easylogging++ thread unsafe";
#endif

    //2.初始化中间件
    PubSubInterface* pubsub;
    #ifdef X86
    pubsub = new ROSPubSub();
    #else
    pubsub = new MDCPubSub();
    #endif

    pubsub->initPubSub(argc, argv, "mapping");

    //3.初始化配置参数
    Load_Udp_YAML(ABS_CURRENT_SOURCE_PATH+"/config/udp.yaml");
    Load_Sensor_YAML(ABS_CURRENT_SOURCE_PATH+"/config/sensor.yaml");
    Load_Loc_YAML(ABS_CURRENT_SOURCE_PATH+"/config/loc.yaml");//TODO 1111 loc.yaml
    Load_offline_YAML(ABS_CURRENT_SOURCE_PATH+"/config/offline_mapping.yaml");
    Load_FrontEnd_YAML(ABS_CURRENT_SOURCE_PATH +"/config/front_end.yaml");
    SetOptLocationMode();

    //3.5 udp
    std::shared_ptr<UDP_THREAD> udp_thread = make_shared<UDP_THREAD>();
    udp_thread->init(UdpConfig::cleint_ip,UdpConfig::clinet_port,UdpConfig::server_port);

    //4.启动多个线程
    LocManager loc_manager;
    loc_manager.Init(pubsub,udp_thread);

    //5.设置mapping manager的回调函数
    auto cloud_callback = std::bind(&LocManager::CloudCallback, &loc_manager,std::placeholders::_1);
    auto gnssins_callback = std::bind(&LocManager::GNSSINSCallback, &loc_manager,std::placeholders::_1);

    pubsub->addSubscriber(SensorConfig::pointCloudTopic, DataType::LIDAR, cloud_callback);
    pubsub->addSubscriber(SensorConfig::gpsTopic, DataType::GNSS_INS, gnssins_callback);

    //开始运行程序
    pubsub->run();
}