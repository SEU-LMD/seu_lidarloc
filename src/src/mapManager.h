//
// Created by fyy on 23-10-12.
//

#ifndef SEU_LIDARLOC_MAPMANAGER_H
#define SEU_LIDARLOC_MAPMANAGER_H
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "utils/timer.h"
#include "utils/MapSaver.h"
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"

struct MyPointType{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    int down_grid_index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(int, down_grid_index, down_grid_index)
)

typedef pcl::PointXYZI PointType;

class MapManager{
public:
    typedef struct{
        std::string down_grid_index;
        int down_grid_num;
    }Downgrid;


    typedef struct {
        double x;
        double y;
        double z;
    }Gnsspostion;

    PubSubInterface* pubsub;
    std::thread* mapManager_thread;

    std::mutex mutex_data;
    std::mutex mutex_DR_data;
    std::mutex mtxGraph;

    std::string topic_priorMap_corner_mapManger = "/Prior_map_corner_MM";
    std::string topic_priorMap_surf_mapManger = "/Prior_map_surf_MM";
    std::deque<std::shared_ptr<OdometryType>> data_deque;

    //是否更新标志位
    bool top_cache_update=false;
    //是否是第一帧标志位
    bool is_initailed= false;

    int laserCloudWidth=0;
    int laserCloudHeight=0;
    bool if_need_load_map = 0;
    double cur_time;

//从txt中获得
    double x_min_t;
    double y_min_t;
//建立的地图数据种类
    std::vector<std::string>   pcd_feature;
//有多少个小grid就有多少个指针
    pcl::PointCloud<PointType>::Ptr laser_cloud_corner_array[112];
    pcl::PointCloud<PointType>::Ptr laser_cloud_surf_array[112];
//需要建图的索引 3*3  jianshu
    int laser_cloud_valid_ind[9];
//需要加载的地图的索引  5*5 jiazai
    int laser_cloud_load_ind[25];

//有一部分是空地图，可能造成指针没有fenpei
    int laser_cloud_corner_load_ind[25];
    int laser_cloud_surf_load_ind[25];

//建图的点云指针
// surround points in map to build tree
    pcl::PointCloud<PointType>::Ptr laser_cloud_corner_from_map;
    pcl::PointCloud<PointType>::Ptr laser_cloud_surf_from_map;
//build tree
//shu chu
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_from_map;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_from_map;
//remeber empty
    std::vector<std::string>  corner_empty;
    std::vector<std::string>  surf_empty;


//记住上次的参数
    int last_center_cubeI;
    int last_center_cubeJ;
    int map_center_cubeI;
    int map_center_cubeJ;
    int last_laser_cloud_load_ind[25];

    std::vector<int> lasercloud_loaded_map;
    std::function<void(const PriorMap&)> Function_AddPriorMapToLoc;


//获得初始变量，对标志位赋值
    Gnsspostion now_position;


    void MapmanagerInitialized(const std::string& map_read_path){
        std::ifstream file(map_read_path+"index.txt",std::ios_base::in);
        std::string line;
        std::getline(file, line); // 读取第一行并存储到line中
        std::istringstream iss(line);
        iss >> x_min_t >> y_min_t; //
        EZLOG(INFO)<<x_min_t<<"   "<<y_min_t<<std::endl;
        while (std::getline(file, line)) {
            std::string string_feature;
            std::string string_index;
            std::istringstream isss(line);
            isss >> string_feature >> string_index;
            EZLOG(INFO)<<string_feature<<"   "<<string_index<<std::endl;
            if(string_feature=="corner")  corner_empty.push_back(string_index);
            else if(string_feature=="surf") surf_empty.push_back(string_index);
        }

        pcd_feature.push_back("corner");
        pcd_feature.push_back("surf");
    }

//小grid索引映射
    int down_grid_index2int(const std::string& down_grid_index){

        std::istringstream ss(down_grid_index);
        int i, j;

        ss >> i;
        ss.ignore(1, '_'); // 忽略并跳过下划线字符
        ss >> j;

        return i+j*SerializeConfig::lasercloud_width;
    }

//大格子字符串映射
//输入小grid整体编号
//输出需要加载的大grid编号  小格子到大格子的映射
    std::string int2up_grid_index(const int& lasercloud_index){

        int i,j,I,J;
        i=lasercloud_index%SerializeConfig::lasercloud_width;
        j=lasercloud_index/SerializeConfig::lasercloud_width;

        I=i/SerializeConfig::up2down_num;
        J=j/SerializeConfig::up2down_num;

        std::string down_grid_string=std::to_string(i)+"_"+std::to_string(j);
        std::string up_grid_string=std::to_string(I)+"_"+std::to_string(J);
        return up_grid_string;
    }

    void MallocMeomery(const std::string& up_grid_string,const std::string& feature_string ){  //有待改进成完全由超参数定义数据大小
        int lasercloud_index[4];
        std::istringstream ss(up_grid_string);

        int i, j;

        ss >> i;
        ss.ignore(1, '_'); // 忽略并跳过下划线字符
        ss >> j;

        lasercloud_index[0]=(i*SerializeConfig::up2down_num)+(j*SerializeConfig::up2down_num*SerializeConfig::lasercloud_width);
        lasercloud_index[1]=lasercloud_index[0]+1;
        lasercloud_index[2]=lasercloud_index[0]+SerializeConfig::lasercloud_width;
        lasercloud_index[3]=lasercloud_index[2]+1;
        EZLOG(INFO)<<"lasercloud_index[0]:"<<lasercloud_index[0]<<std::endl;
        EZLOG(INFO)<<"lasercloud_index[2]:"<<lasercloud_index[2]<<std::endl;

        for(int k=0; k<SerializeConfig::up2down_num*SerializeConfig::up2down_num; ++k){
            if(feature_string=="corner"){
                laser_cloud_corner_array[lasercloud_index[k]].reset(new pcl::PointCloud<PointType>());
                lasercloud_loaded_map.push_back(lasercloud_index[k]);
            }
            if(feature_string=="surf"){
                laser_cloud_surf_array[lasercloud_index[k]].reset(new pcl::PointCloud<PointType>());
                lasercloud_loaded_map.push_back(lasercloud_index[k]);
            }
        }
    }


//加载大格子到地图中
//输入为大grid
    void UpdateTopGrid(const std::string& map_read_path,const std::string& index,const std::string& feature){

        pcl::PointCloud<MyPointType>::Ptr loadcloud(new pcl::PointCloud<MyPointType>);

        std::string lidar_filename_path=map_read_path+index+feature+".pcd";
        if (pcl::io::loadPCDFile<MyPointType>(lidar_filename_path, *loadcloud) == -1) {
            EZLOG(INFO) << "ERROR: Cannot open file " << lidar_filename_path
                        << "! Aborting..." << std::endl;
            return;
        }
        MallocMeomery(index,feature);
        EZLOG(INFO)<<"begin_loop"<<std::endl;
        for (const MyPointType& point : *loadcloud){
            PointType outpoint;
            outpoint.x=point.x;
            outpoint.y=point.y;
            outpoint.z=point.z;
            outpoint.intensity=point.intensity;
            if (feature=="corner")
                laser_cloud_corner_array[point.down_grid_index]->push_back(outpoint);
            else if(feature=="surf")
                laser_cloud_surf_array[point.down_grid_index]->push_back(outpoint);
        }
        EZLOG(INFO)<<"end_loop"<<std::endl;
    }
    void LoadMap(const int& center_cubeI,const int& center_cubeJ){

        int begin_load_cubeI;
        int begin_load_cubeJ;

        //当前map的中心
        map_center_cubeI=center_cubeI;
        map_center_cubeJ=center_cubeJ;

        /*****************************loadmap*************/
        if(center_cubeI == 0 || center_cubeI == SerializeConfig::lasercloud_width-1){
            begin_load_cubeI = center_cubeI == 0 ? 0 : SerializeConfig::lasercloud_width - 5;
        }
        else if(center_cubeI == 1 || center_cubeI == SerializeConfig::lasercloud_width-2){
            begin_load_cubeI = center_cubeI == 1 ? 0 : SerializeConfig::lasercloud_width - 5;
        }
        else begin_load_cubeI=center_cubeI-2;
        //y方向
        if(center_cubeJ == 0 || center_cubeJ == SerializeConfig::lasercloud_height-1){
            begin_load_cubeJ = center_cubeJ == 0 ? 0 : SerializeConfig::lasercloud_height - 5;
        }
        else if(center_cubeJ == 1 || center_cubeJ == SerializeConfig::lasercloud_height-2){
            begin_load_cubeJ = center_cubeJ == 1 ? 0 : SerializeConfig::lasercloud_height - 5;
        }
        else begin_load_cubeJ=center_cubeJ-2;
        EZLOG(INFO)<<"loadbegin："<<begin_load_cubeI<<" "<<" "<<begin_load_cubeJ<<std::endl;
        //需要加载的索引以及寻找是否已经加载
        int k=0;
        for (int i=begin_load_cubeJ; i<begin_load_cubeJ+5; ++i) {
            for(int j=begin_load_cubeI; j<begin_load_cubeI+5; ++j){
                bool is_in_memeroy= false;
                laser_cloud_load_ind[k]=j+i*SerializeConfig::lasercloud_width;
                //寻找地图中有没有当前的点
                for (const auto& pair : lasercloud_loaded_map) {
                    if(pair==laser_cloud_load_ind[k]) {
                        is_in_memeroy= true;
                        break;
                    }
                }
                //如果没有就加载
                if(!is_in_memeroy){
                    bool isconer_empty= false;
                    bool issurf_empty= false;
                    std::string laod_up_grid_string=int2up_grid_index(laser_cloud_load_ind[k]);
                    EZLOG(INFO)<<"laod_up_grid_string"<<laod_up_grid_string<<std::endl;
                    for(const auto& pair : corner_empty){
                        if(pair==laod_up_grid_string){
                            isconer_empty=true;
                        }
                    }
                    for(const auto& pair : surf_empty){
                        if(pair==laod_up_grid_string){
                            issurf_empty=true;
                        }
                    }
                    if(!isconer_empty)  UpdateTopGrid(SerializeConfig::map_out_path,laod_up_grid_string,"corner");

                    if(!issurf_empty) UpdateTopGrid(SerializeConfig::map_out_path,laod_up_grid_string,"surf");
                }
                ++k;
            }
        }
    }


    void BuildTree(const int& center_cubeI,const int& center_cubeJ){
        int begin_bulid_cubeI;
        int begin_bulid_cubeJ;
        //std::cout<<"hello"<<std::endl;
        /***************build tree*********************************/
        //调整需要加载的中心位置 x方向
        if(center_cubeI == 0 || center_cubeI == SerializeConfig::lasercloud_width-1){
            begin_bulid_cubeI = center_cubeI == 0 ? 0 : SerializeConfig::lasercloud_width - 3;
        }
        else begin_bulid_cubeI=center_cubeI-1;
        //y方向
        if(center_cubeJ == 0 || center_cubeJ == SerializeConfig::lasercloud_height-1){
            begin_bulid_cubeJ = center_cubeJ == 0 ? 0 : SerializeConfig::lasercloud_height - 3;
        }
        else begin_bulid_cubeJ=center_cubeJ-1;

        int k=0;
        for (int i=begin_bulid_cubeJ; i<begin_bulid_cubeJ+3; ++i) {
            for(int j=begin_bulid_cubeI; j<begin_bulid_cubeI+3; ++j){
                laser_cloud_valid_ind[k]=j+i*SerializeConfig::lasercloud_width;
                EZLOG(INFO)<<laser_cloud_valid_ind[k]<<std::endl;
                k++;
            }
        }

        for(int i=0; i<9; ++i){
            if(laser_cloud_corner_array[laser_cloud_valid_ind[i]]!= nullptr)
                *laser_cloud_corner_from_map+=*laser_cloud_corner_array[laser_cloud_valid_ind[i]];
            if(laser_cloud_surf_array[laser_cloud_valid_ind[i]]!= nullptr)
                *laser_cloud_surf_from_map+=*laser_cloud_surf_array[laser_cloud_valid_ind[i]];
        }


        EZLOG(INFO)<<laser_cloud_corner_from_map->size()<<std::endl;

        kdtree_corner_from_map->setInputCloud(laser_cloud_corner_from_map);
        kdtree_surf_from_map->setInputCloud(laser_cloud_surf_from_map);
        { // for debug
            CloudTypeXYZI PriorMap_surf_pub;
            PriorMap_surf_pub.frame = "map";
            PriorMap_surf_pub.timestamp = cur_time;
            PriorMap_surf_pub.cloud = *laser_cloud_surf_from_map;
            pubsub->PublishCloud(topic_priorMap_surf_mapManger, PriorMap_surf_pub);
            std::cout << "Pub Surf Map to loc!" << std::endl;

            CloudTypeXYZI PriorMap_corner_pub;
            PriorMap_corner_pub.frame = "map";
            PriorMap_corner_pub.timestamp = cur_time;
            PriorMap_corner_pub.cloud = *laser_cloud_corner_from_map;
            pubsub->PublishCloud(topic_priorMap_corner_mapManger, PriorMap_corner_pub);
            std::cout << "Pub Corner Map to loc!" << std::endl;
        }
        // pub to Loc
        PriorMap priormap;
        priormap.PriorSurfMapKDTree = kdtree_surf_from_map;
        priormap.PriorCornerMapKDTree = kdtree_corner_from_map;
        priormap.PriorSurfMap = laser_cloud_surf_from_map;
        priormap.PriorCornerMap = laser_cloud_corner_from_map;
        priormap.timestamp = cur_time;
        Function_AddPriorMapToLoc(priormap);

    }

    void ErasElement(){
        for(int i=0; i<25; ++i){
            bool is_equl= false;
            for(int j=0; j<25; ++j){
                if(last_laser_cloud_load_ind[i]==laser_cloud_valid_ind[j]){
                    is_equl= true;
                    break;
                }
            }
            if(!is_equl){
                laser_cloud_surf_array[last_laser_cloud_load_ind[i]]->clear();
                // 遍历map并删除包含目标值的vector<int>
                for (auto it = lasercloud_loaded_map.begin(); it != lasercloud_loaded_map.end();) {
                    if (*it == last_laser_cloud_load_ind[i]) {
                        it = lasercloud_loaded_map.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
        }
    }


    void process(const Gnsspostion& gnsspostion){


        int center_cubeI=static_cast<int>(std::floor(gnsspostion.x-x_min_t)/(SerializeConfig::up_grid_size/SerializeConfig::up2down_num));
        int center_cubeJ=static_cast<int>(std::floor(gnsspostion.y-y_min_t)/(SerializeConfig::up_grid_size/SerializeConfig::up2down_num));
        EZLOG(INFO)<<"center_cubeI："<<center_cubeI<<" "<<"center_cubeJ："<<center_cubeJ<<std::endl;
        //初始化过程
        if(!is_initailed){
            LoadMap(center_cubeI,center_cubeJ);
            BuildTree(center_cubeI,center_cubeJ);
            //记录本次数据
            last_center_cubeI=center_cubeI;
            last_center_cubeJ=center_cubeJ;
            for(int i=0;i<25;i++){
                last_laser_cloud_load_ind[i]=laser_cloud_load_ind[i];
            }
            laser_cloud_corner_from_map->clear();
            laser_cloud_surf_from_map->clear();
            is_initailed= true;
            return;
        }
        //如果完全没动直接跳出
        if(center_cubeI==last_center_cubeI&&center_cubeJ==last_center_cubeJ) {
            return;
        }
        //判断是否需要加载地图    简洁版本，后续需要继续优化(本生就在大地图的边缘)
        //到大地图的边缘就需要加载了
        if(abs(center_cubeI-map_center_cubeI)>1||abs(center_cubeJ-map_center_cubeJ)>1) {
            top_cache_update = true;
        }
        else {
            top_cache_update = false;
            return;
        }
        //需要加载就加载
        if(top_cache_update) {
            LoadMap(center_cubeI, center_cubeJ);
        }
        //建树
        BuildTree(center_cubeI,center_cubeJ);
        //清除多于的部分
        ErasElement();

        //记录本次相关相关参数
        last_center_cubeI=center_cubeI;
        last_center_cubeJ=center_cubeJ;
        for(int i=0;i<25;i++){
            last_laser_cloud_load_ind[i]=laser_cloud_load_ind[i];
        }
        laser_cloud_corner_from_map->clear();
        laser_cloud_surf_from_map->clear();
    }

    //this fucntion needs to be binded by lidar loc node
    void AddLoctoMapManager(const OdometryType &loc_res){
        mutex_data.lock();
        std::shared_ptr<OdometryType> odometryPtr = std::make_shared<OdometryType>(loc_res);
        //TODO if need to load map
        data_deque.push_back(odometryPtr);
        mutex_data.unlock();
    }

    void MapManagerInitialized(){
        MapmanagerInitialized(SerializeConfig::map_out_path);
        laser_cloud_corner_from_map.reset(new pcl::PointCloud<PointType>());
        laser_cloud_surf_from_map.reset(new pcl::PointCloud<PointType>());
        kdtree_surf_from_map.reset(new pcl::KdTreeFLANN<PointType>());
        kdtree_corner_from_map.reset(new pcl::KdTreeFLANN<PointType>());

        EZLOG(INFO)<<" MapManager Init Successful!";
    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
        MapManagerInitialized();

        pubsub->addPublisher(topic_priorMap_corner_mapManger, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_priorMap_surf_mapManger, DataType::LIDAR, 10);

        mapManager_thread = new std::thread(&MapManager::DoWork, this);
    }

    void DoWork(){
        while(1){
            if(data_deque.size()!=0){
                OdometryTypePtr current_loc_res;

                mutex_data.lock();
                current_loc_res = data_deque.back();
                mutex_data.unlock();

                now_position.x = current_loc_res->pose.GetXYZ().x();
                now_position.y = current_loc_res->pose.GetXYZ().y();
                now_position.z = current_loc_res->pose.GetXYZ().z();
                cur_time = current_loc_res->timestamp;
                process(now_position);

                mutex_data.lock();
                data_deque.clear();
                mutex_data.unlock();

            }
            else{
                sleep(0.1);
            }
        }
    }//end fucntion do work


};
#endif //SEU_LIDARLOC_MAPMANAGER_H
