//
// Created by slam on 23-8-28.
//

#include "mapmanager.h"


//是否更新标志位
bool top_cache_update=false;
//是否是第一帧标志位
bool is_initailed= false;

//从txt中获得
double x_min_t;
double y_min_t;
//建立的地图数据种类
std::vector<std::string>   pcd_feature;
//有多少个小grid就有多少个指针
pcl::PointCloud<PointType>::Ptr laser_cloud_corner_array[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laser_cloud_surf_array[laserCloudNum];
//需要建图的索引 3*3  jianshu
int laser_cloud_valid_ind[9];
//需要加载的地图的索引  5*5 jiazai
int laser_cloud_load_ind[25];
//建图的点云指针
// surround points in map to build tree
pcl::PointCloud<PointType>::Ptr laser_cloud_corner_from_map(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laser_cloud_surf_from_map(new pcl::PointCloud<PointType>());
//build tree
pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_from_map(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_from_map(new pcl::KdTreeFLANN<PointType>());

//记住上次的参数
int last_center_cubeI;
int last_center_cubeJ;
int map_center_cubeI;
int map_center_cubeJ;
int last_laser_cloud_load_ind[25];


std::map<std::string, std::vector<int>> laser_cloud_load_map;




//获得初始变量，对标志位赋值
void
MapmanagerInitialized(){
    std::ifstream file(map_read_path+"index.txt",std::ios_base::in);
    std::string line;
    std::getline(file, line); // 读取第一行并存储到line中
    std::istringstream iss(line);
    iss >> x_min_t >> y_min_t; //
    std::cout<<"x_min"<<x_min_t<<std::endl;
    std::cout<<"y_min"<<y_min_t<<std::endl;

    pcd_feature.push_back("corner");
    pcd_feature.push_back("surf");
}



//小grid索引映射
int
down_grid_index2int(std::string down_grid_index){

    std::istringstream ss(down_grid_index);
    int i, j;

    ss >> i;
    ss.ignore(1, '_'); // 忽略并跳过下划线字符
    ss >> j;

    return i+j*laserCloudWidth;
}

//大格子字符串映射
//输入小grid整体编号
//输出需要加载的大grid编号  小格子到大格子的映射
std::string
int2up_grid_index(int lasercloud_index){

    int i,j,I,J;
    i=lasercloud_index%laserCloudWidth;
    j=lasercloud_index/laserCloudWidth;

    I=i/up2down_num;
    J=j/up2down_num;

    std::string down_grid_string=std::to_string(i)+"_"+std::to_string(j);
    std::string up_grid_string=std::to_string(I)+"_"+std::to_string(J);
    return up_grid_string;
}



//加载大格子到地图中
//输入为大grid
void
UpdateTopGrid(std::string index,std::string feature){
    Downgrid downgrid;
    std::ifstream downfile(map_read_path+index+feature+".txt");
    for(int j=0;j<up2down_num*up2down_num;j++)
    {
        if (downfile.is_open()) {
            std::string line;
            std::getline(downfile, line);
            std::istringstream iss(line);

            iss >> downgrid.down_grid_index >> downgrid.down_grid_num;
            int lasercloud_index=down_grid_index2int(downgrid.down_grid_index);
            std::cout<<lasercloud_index<<std::endl;
            laser_cloud_load_map[index].push_back(lasercloud_index);
            if (feature=="corner")

                laser_cloud_corner_array[lasercloud_index].reset(new pcl::PointCloud<PointType>());

            else if(feature=="surf")
                laser_cloud_surf_array[lasercloud_index].reset(new pcl::PointCloud<PointType>());

            for (int i = 0; i < downgrid.down_grid_num; i++) {

                std::getline(downfile, line);
                std::istringstream iss(line);

                PointType point;
                iss >> point.x >> point.y >> point.z;
                if (feature=="corner")
                    laser_cloud_corner_array[lasercloud_index]->push_back(point);
                else if(feature=="surf")
                    laser_cloud_surf_array[lasercloud_index]->push_back(point);
            }
                if (feature=="corner")
                    std::cout << "读取到点个数" << laser_cloud_corner_array[lasercloud_index] ->size()<< std::endl;
                else if(feature=="surf")
                    std::cout << "读取到点个数" << laser_cloud_surf_array[lasercloud_index] ->size()<< std::endl;

                downgrid.down_grid_index.clear();
        }
    }
    downfile.close(); // 关闭文件
}

void
LoadMap(const int& center_cubeI,const int& center_cubeJ){

    int center_load_cubeI;
    int center_load_cubeJ;

    //当前map的中心
    map_center_cubeI=center_cubeI;
    map_center_cubeJ=center_cubeJ;

    /*****************************loadmap*************/
    if(center_cubeI == 0 || center_cubeI == laserCloudWidth-1){
        center_load_cubeI = center_cubeI == 0 ? 0 : laserCloudWidth - 5;
    }
    else if(center_cubeI == 1 || center_cubeI == laserCloudWidth-2){
        center_load_cubeI = center_cubeI == 1 ? 0 : laserCloudWidth - 5;
    }
    else center_load_cubeI=center_cubeI-2;
    //y方向
    if(center_cubeJ == 0 || center_cubeJ == laserCloudHeight-1){
        center_load_cubeJ = center_cubeJ == 0 ? 0 : laserCloudHeight - 5;
    }
    else if(center_cubeJ == 1 || center_cubeJ == laserCloudHeight-2){
        center_load_cubeJ = center_cubeJ == 1 ? 0 : laserCloudHeight - 5;
    }
    else center_load_cubeJ=center_cubeJ-2;
    std::cout<<"load"<<center_load_cubeI<<" "<<" "<<center_load_cubeJ<<std::endl;
    //需要加载的索引以及寻找是否已经加载
    int k=0;
    for (int i=center_load_cubeJ; i<center_load_cubeJ+5; ++i) {
        for(int j=center_load_cubeI; j<center_load_cubeJ+5; ++j){
            bool is_in_memeroy= false;
            laser_cloud_load_ind[k]=j+i*laserCloudWidth;
            //寻找地图中有没有当前的点
            for (const auto& pair : laser_cloud_load_map) {
                const std::vector<int>& vec = pair.second;
                for (int value : vec) {
                    if(value==laser_cloud_load_ind[k]) {
                        is_in_memeroy= true;
                        break;
                    }
                }
                if(is_in_memeroy)
                    break;
            }
            //如果没有就加载
            if(!is_in_memeroy){
                std::string laod_up_grid_string=int2up_grid_index(laser_cloud_load_ind[k]);
                UpdateTopGrid(laod_up_grid_string,"corner");
                UpdateTopGrid(laod_up_grid_string,"surf");
                std::cout<<"hello"<<std::endl;
            }
            ++k;
            std::cout<<"k:"<<k<<std::endl;
        }
    }
}


void
BuildTree(const int& center_cubeI,const int& center_cubeJ){
    int center_bulid_cubeI;
    int center_bulid_cubeJ;
    //std::cout<<"hello"<<std::endl;
    /***************build tree*********************************/
    //调整需要加载的中心位置 x方向
    if(center_cubeI == 0 || center_cubeI == laserCloudWidth-1){
        center_bulid_cubeI = center_cubeI == 0 ? 0 : laserCloudWidth - 3;
    }
    else center_bulid_cubeI=center_cubeI-1;
    //y方向
    if(center_cubeJ == 0 || center_cubeJ == laserCloudHeight-1){
        center_bulid_cubeJ = center_cubeJ == 0 ? 0 : laserCloudHeight - 3;
    }
    else center_bulid_cubeJ=center_cubeJ-1;

    int k=0;
    for (int i=center_bulid_cubeJ; i<center_bulid_cubeJ+3; ++i) {
        for(int j=center_bulid_cubeI; j<center_bulid_cubeJ+3; ++j){
            laser_cloud_valid_ind[k]=j+i*laserCloudWidth;
            std::cout<<laser_cloud_valid_ind[k]<<std::endl;
            k++;
        }
    }

    for(int i=0; i<9; ++i){
        *laser_cloud_corner_from_map+=*laser_cloud_corner_array[laser_cloud_valid_ind[i]];
        *laser_cloud_surf_from_map+=*laser_cloud_surf_array[laser_cloud_valid_ind[i]];
    }
    std::cout<<laser_cloud_corner_from_map->size()<<std::endl;
    kdtree_corner_from_map->setInputCloud(laser_cloud_corner_from_map);
    kdtree_surf_from_map->setInputCloud(laser_cloud_surf_from_map);
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
            for (auto& item : laser_cloud_load_map) {
                auto& vec = item.second;
                auto it = std::find(vec.begin(), vec.end(), last_laser_cloud_load_ind[i]);
                if (it != vec.end()) {
                    vec.erase(it);
                    break; // 只删除第一个匹配的值
                }
            }
        }
    }
}




void process(const Gnsspostion& gnsspostion){

        int center_cubeI=static_cast<int>(std::floor(gnsspostion.x-x_min_t)/(up_grid_size/up2down_num));
        int center_cubeJ=static_cast<int>(std::floor(gnsspostion.y-y_min_t)/(up_grid_size/up2down_num));
        std::cout<<"center_cubeI"<<center_cubeI<<" "<<"center_cubeJ"<<center_cubeJ<<std::endl;
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
        if(center_cubeI==last_center_cubeI&&center_cubeJ==last_center_cubeJ) return;

        //判断是否需要加载地图    简洁版本，后续需要继续优化(本生就在大地图的边缘)
        //到大地图的边缘就需要加载了
        if(abs(center_cubeI-map_center_cubeI)>1||abs(center_cubeJ-map_center_cubeJ)>1)
            top_cache_update= true;
        else top_cache_update= false;
        //需要加载就加载
        if(top_cache_update)  LoadMap(center_cubeI,center_cubeJ);
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



int
main (int argc, char** argv)
{
    TicToc time;
    MapmanagerInitialized();
    Gnsspostion gnsspostion;
    for(int i=0;i<8;++i){
        gnsspostion.x=-49.00+i*20;
        gnsspostion.y=-110.00+i*20;
        process(gnsspostion);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout<<"time"<<time.toc()<<std::endl;
}