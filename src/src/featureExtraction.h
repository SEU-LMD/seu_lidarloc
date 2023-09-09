#include <mutex>

#include "pubsub/pubusb.h"
class FeatureExtraction  {
public:
    PubSubInterface* pubsub;
    std::mutex
    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
    }

    void DoWork(){
        while(1){

        }
    }

};
