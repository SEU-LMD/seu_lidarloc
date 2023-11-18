//
// Created by fyy on 23-9-9.
//

#include "utils/config_helper.h"
#include "easylogging++.h"
#include "./mapping_manager.h"
#include "utils/filesys.h"
#include "utils/udp_thread.h"
//选择中间件
#define X86
#ifdef X86
#include "pubsub/ros/ros_pubsub.h"
#else
#include "pubsub/mdc/mdc_pubsub.h"
#endif

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {

    CreateDirWithDelete("./log");

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
    Load_Sensor_YAML("./config/sensor.yaml");
    Load_Mapping_YAML("./config/mapping.yaml");
    Load_FrontEnd_YAML("./config/front_end.yaml");
    SetOptMappingMode();

    //TODO 1111 remove !!!!
    //4.启动多个线程
    MappingManager mapping_manager;
    mapping_manager.Init(pubsub);

    //5.设置mapping manager的回调函数
    auto cloud_callback = std::bind(&MappingManager::CloudCallback, &mapping_manager,std::placeholders::_1);
    auto gnssins_callback = std::bind(&MappingManager::GNSSINSCallback, &mapping_manager,std::placeholders::_1);


    pubsub->addSubscriber(SensorConfig::pointCloudTopic, DataType::LIDAR, cloud_callback);
    pubsub->addSubscriber(SensorConfig::gpsTopic, DataType::GNSS_INS, gnssins_callback);


    //开始运行程序
    pubsub->run();
}