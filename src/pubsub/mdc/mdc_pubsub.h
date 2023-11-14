#include <csignal>
#include "singleton.h"
#include "position.h"


#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"




namespace {
        sig_atomic_t g_stopFlag = 0;
        void INTSigHandler(int32_t num)
        {
            g_stopFlag = 1;
            std::cout << "  Signal Interactive attention" << num << "received." << std::endl;
        }
 }  // namespace


class MDCPubSub:public PubSubInterface{

        public:
            // std::vector<ros::Subscriber> subscribers;
           unique_ptr<shineauto::position::Position> position_ptr;

            void initPubSub(int argc, char** argv, const std::string& node_name){
                        signal(SIGINT, INTSigHandler);
                        std::cout << "start mdc" << std::endl;
                        if (!IsSingleton()) {
                            std::cout << "position already exists!" << std::endl;
                            exit(0);
                        }
                       position_ptr = std::make_unique<shineauto::position::Position>("Config.yaml");
                        HafStatus ret = position_ptr->Init();
                        if (ret != HAF_SUCCESS) {
                            HAF_LOG_ERROR << "position init failed!";
                            exit(-1);                          
                             }
                        position_ptr->Process();
            }



            void addSubscriber(const std::string &topic_name, const DataType &type, CallBackT callback) {
                    if(type==DataType::LIDAR){
                                position_ptr->LidarFunction = callback ;
                    }
                    else if(type==DataType::GNSS_INS){
                            position_ptr->GNSSINSFunction = callback ;
                    }
            }


            void run(){
                while ((!g_stopFlag) && !position_ptr->IsStop()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                position_ptr->Stop();
            }
    void addPublisher(const std::string &topic_name, const DataType &type, int queue_size){}
    void PublishCloud(const std::string &topic_name, const CloudTypeXYZIRT &data){}
    void PublishCloud(const std::string &topic_name, const CloudTypeXYZI &data){}
    void PublishCloud(const std::string &topic_name, const CloudTypeXYZICOLRANGE &data) {}
    void PublishOdometry(const std::string &topic_name, const OdometryType &data){
              position_ptr->pubilsh(topic_name,data);
     }
};//end class MDCPubSub 