//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_PUBUSB_H
#define SEU_LIDARLOC_PUBUSB_H
#include <functional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "data_types.h"

//所有中间件的基类
typedef std::function<void(const BaseType&)> CallBackT;

class PubSubInterface {
public:
    virtual void initPubSub(int argc, char **argv, const std::string &node_name) = 0;

    virtual void addSubscriber(const std::string &topic_name, const DataType &type, CallBackT callback) = 0;

    virtual void addPublisher(const std::string &topic_name, const DataType &type, int queue_size) = 0;

    virtual void run() = 0;

    virtual void PublishCloud(const std::string &topic_name, const CloudTypeXYZIRT &data) = 0;
    virtual void PublishCloud(const std::string &topic_name, const CloudTypeXYZI &data) = 0;
    virtual void PublishCloud(const std::string &topic_name, const CloudTypeXYZICOLRANGE &data) = 0;


    virtual void PublishOdometry(const std::string &topic_name, const OdometryType &data) = 0;
};
#endif //SEU_LIDARLOC_PUBUSB_H
