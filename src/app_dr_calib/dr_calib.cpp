//
// Created by fyy on 23-9-9.
//

#include "utils/config_helper.h"
#include "easylogging++.h"
#include "./dr_calib_manager.h"
#include "utils/filesys.h"
//选择中间件
//#ifdef X86
#include "pubsub/ros/ros_pubsub.h"
//#endif

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
    //#ifdef X86
    pubsub = new ROSPubSub();
    //#endif

    pubsub->initPubSub(argc, argv, "dr_calib");

    //3.初始化配置参数

    //4.启动多个线程
    DRCalibManager dr_calib_manager;
    dr_calib_manager.Init(pubsub);

    //5.设置mapping manager的回调函数
    auto gnssins_callback = std::bind(&DRCalibManager::GNSSINSCallback, &dr_calib_manager,std::placeholders::_1);
    auto dr_callback = std::bind(&DRCalibManager::DrCallback, &dr_calib_manager,std::placeholders::_1);

    pubsub->addSubscriber(SensorConfig::gpsTopic, DataType::GNSS_INS, gnssins_callback);
    pubsub->addSubscriber(SensorConfig::gpsTopic, DataType::WHEEL, dr_callback);//TODO


    //开始运行程序
    pubsub->run();
}