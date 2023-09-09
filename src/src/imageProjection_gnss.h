
// Use the Velodyne point format as a common representation

#include <mutex>
#include <thread>

#include "pubsub/pubusb.h"
#include "featureExtraction.h"

class ImageProjection  {
public:
    PubSubInterface* pubsub;
    std::mutex cloud_mutex;
    std::thread* do_work_thread;
    std::deque<CloudType> deque_cloud;
    FeatureExtraction* ft_extr_ptr;

    void DoWork(){
        while(1){
            if(deque_cloud.size()!=0){
                CloudType cur_scan;
                cloud_mutex.lock();
                cur_scan = deque_cloud.front();
                deque_cloud.pop_front();
                cloud_mutex.unlock();
                //do something


            }
        }
    }

    void addCloudData(const CloudType& data){
        cloud_mutex.lock();
        deque_cloud.push_back(data);
        cloud_mutex.unlock();
    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
        do_work_thread = new std::thread(&ImageProjection::DoWork, this);
    }


};


