#include <mutex>
#ifndef SEU_LIDARLOC_FEATUREEXTRACTION_H
#define SEU_LIDARLOC_FEATUREEXTRACTION_H
#include "pubsub/pubusb.h"
#include "pubsub//data_types.h"
class FeatureExtraction  {
public:
    PubSubInterface* pubsub;
    std::mutex cloud_mutex;
    std::deque<CloudInfo> deque_cloud;

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
    }

    void DoWork(){
        while(1){

        }
    }

    void AddCloudData(const CloudInfo& data){
        cloud_mutex.lock();
        deque_cloud.push_back(data);
        cloud_mutex.unlock();
    }

};
#endif
