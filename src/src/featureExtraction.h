#include <mutex>
#ifndef SEU_LIDARLOC_FEATUREEXTRACTION_H
#define SEU_LIDARLOC_FEATUREEXTRACTION_H
#include "pubsub/pubusb.h"
class FeatureExtraction  {
public:
    PubSubInterface* pubsub;
    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
    }

    void DoWork(){
        while(1){

        }
    }

};
#endif
