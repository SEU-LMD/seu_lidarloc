#include <mutex>
#ifndef SEU_LIDARLOC_FEATUREEXTRACTION_H
#define SEU_LIDARLOC_FEATUREEXTRACTION_H
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"


class FeatureExtraction  {
public:
    PubSubInterface* pubsub;
    std::mutex cloud_mutex;
    std::deque<CloudInfo> deque_cloud;
    std::thread* do_work_thread;


    void DoWork(){
        while(1){
            if(deque_cloud.size()!=0){
                CloudInfo cur_cloud;
                cloud_mutex.lock();
                cur_cloud = deque_cloud.front();
                cloud_mutex.unlock();

                //do some work

            }
        }
    }




    void AddCloudData(const CloudInfo& data){
        cloud_mutex.lock();
        deque_cloud.push_back(data);
        cloud_mutex.unlock();
    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
        do_work_thread = new std::thread(&FeatureExtraction::DoWork, this);

    }

};
#endif
