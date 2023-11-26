
// PCL specific includes
#define PCL_NO_PRECOMPILE //TODO 
#include <iostream>              //标准C++库中的输入输出的头文件
#include <pcl/io/pcd_io.h>       //PCD读写类相关的头文件
#include <pcl/point_types.h>     //PCL中支持的点类型的头文件
#include <pcl/point_cloud.h>
#include <algorithm>            //找最大值和最小值的头文件
#include <cmath>                //向上取整数相关的
#include <fstream>              //文件输入输出的文件相关的
#include <pcl/common/transforms.h>  //点的变换相关的头文件
#include "Eigen/Core"           //eigen
#include "Eigen/Geometry"       //Isometry3d
#include "config_helper.h"
#include <pcl/filters/uniform_sampling.h>
#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP

struct Tum{
    double timestamp; // 时间戳
    double tx, ty, tz; // 平移向量
    double qx, qy, qz, qw; // 四元数表示的旋转
};

struct MyPointType{
    PCL_ADD_POINT4D;

    PCL_ADD_INTENSITY;

    int down_grid_index;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,
(float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(int, down_grid_index, down_grid_index)
)

typedef MyPointType PointType;




std::vector<std::string>  pcd_index;
std::vector<Eigen::Isometry3d>  pcd_tans;
std::vector<std::string>   pcd_feature;
std::vector<Tum> tum_sum;
double x_min_t;
double x_max_t;
double y_min_t;
double y_max_t;



int x_up_num=0;
int y_up_num=0;
int up_bloc_count=0;
int write_sum=0;




void
LoadTxt(const std::string& map_in_path,const int& frame_sum)
{
    std::string pcdindex;
    Eigen::Vector3d tanslation;
    Eigen::Quaterniond rotation;
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    double origin[3];
    std::ifstream originfile(map_in_path+"Origin.txt");
    std::string originlie;
    std::getline(originfile, originlie);
    std::istringstream isss(originlie);
    isss>>origin[0]>>origin[1]>>origin[2];
    originfile.close();

    std::ifstream downfile(map_in_path+"opt_poses.txt");
    Tum tum;
    for(int j=0;j<frame_sum;j++){
        std::string line;
        std::getline(downfile, line);
        std::istringstream iss(line);
        iss >> pcdindex >> tum.tx >> tum.ty >> tum.tz >> tum.qx >> tum.qy >> tum.qz >> tum.qw;
//        tum.tx += origin[0];
//        tum.ty += origin[1];
//        tum.tz += origin[2];
        tanslation<<tum.tx,tum.ty,tum.tz;
        rotation.x()=tum.qx;
        rotation.y()=tum.qy;
        rotation.z()=tum.qz;
        rotation.w()=tum.qw;
        rotation.normalize();
        T.rotate(rotation);
        T.pretranslate(tanslation);
        std::cout<<pcdindex<<std::endl;
        tum_sum.push_back(tum);
        pcd_index.push_back(pcdindex);
        pcd_tans.push_back(T);
        T=Eigen::Isometry3d::Identity();
    }
    downfile.close(); // 关闭文件
    for(int i=0;i<frame_sum;i++){
        EZLOG(INFO)<< pcd_index[i]<<std::endl;
        EZLOG(INFO)<< pcd_tans[i].matrix()<<std::endl;
        EZLOG(INFO)<< tum_sum[i].tx<<std::endl;
    }
    pcd_feature.push_back("corner");
    pcd_feature.push_back("surf");
}

//通过位姿求地图四个角
void
FindMinMaxUsePose(const std::string& map_out_path,const int& lidar_range){
    //x排序
    std::sort(tum_sum.begin(), tum_sum.end(), [](const Tum& a, const Tum& b) {
        return a.tx < b.tx;
    });
    //求地图四个角
    x_min_t=tum_sum.begin()->tx-lidar_range;
    x_max_t=std::prev(tum_sum.end())->tx+lidar_range;
    //y排序
    std::sort(tum_sum.begin(), tum_sum.end(), [](const Tum& a, const Tum& b) {
        return a.ty < b.ty;
    });
    //求地图四个角
    y_min_t=tum_sum.begin()->ty-lidar_range;
    y_max_t=std::prev(tum_sum.end())->ty+lidar_range;
    //输出
    //输出到文本文件
    std::ofstream file(map_out_path+"index.txt");
    file<<x_min_t<<" "<<y_min_t<<" "<<std::endl;
    file.close();
    EZLOG(INFO)<<"xmin:"<<x_min_t<<std::endl;
    EZLOG(INFO)<<"xmax:"<<x_max_t<<std::endl;
    EZLOG(INFO)<<"ymin:"<<y_min_t<<std::endl;
    EZLOG(INFO)<<"ymax:"<<y_max_t<<std::endl;
}


void
CutDownMapCaulate(){
    x_up_num=static_cast<int>(std::ceil((x_max_t-x_min_t)/SerializeConfig::up_grid_size));
    y_up_num=static_cast<int>(std::ceil((y_max_t-y_min_t)/SerializeConfig::up_grid_size));
    up_bloc_count=x_up_num*y_up_num;
    std::ofstream file(SerializeConfig::map_out_path+"index.txt",std::ios_base::app);
    file<<x_up_num*SerializeConfig::up2down_num <<" "<<y_up_num*SerializeConfig::up2down_num<<std::endl;
    file.close();
    EZLOG(INFO)<<  "x分区: " << x_up_num << std::endl;
    EZLOG(INFO)<< "y分区: " << y_up_num << std::endl;
    EZLOG(INFO)<< "分区总数" <<up_bloc_count<<std::endl;
}

PointType
PointPoseTrance(const pcl::PointXYZI& piont_in,const Eigen::Isometry3d& trans ){
    Eigen::Vector3d piont_trans(piont_in.x,piont_in.y,piont_in.z);
    Eigen::Vector3d piont_transformed=trans*piont_trans;
    PointType piont_out;
    piont_out.x=piont_transformed.x();
    piont_out.y=piont_transformed.y();
    piont_out.z=piont_transformed.z();
    piont_out.intensity=piont_in.intensity;
    return piont_out;
}

pcl::PointXYZI
PointPoseTranceGlobal(const pcl::PointXYZI& piont_in,const Eigen::Isometry3d& trans ){
    Eigen::Vector3d piont_trans(piont_in.x,piont_in.y,piont_in.z);
    Eigen::Vector3d piont_transformed=trans*piont_trans;
    pcl::PointXYZI piont_out;
    piont_out.x=piont_transformed.x();
    piont_out.y=piont_transformed.y();
    piont_out.z=piont_transformed.z();
    piont_out.intensity=piont_in.intensity;
    return piont_out;
}


void
CutMap(const std::string& feature,const std::string& map_out_path,const std::string& map_in_path,const int frame_sum){
    //down size
    pcl::UniformSampling<PointType> downSizeFilterCorner_US;
    pcl::UniformSampling<PointType> downSizeFilterSurf_US;

    downSizeFilterCorner_US.setRadiusSearch(MappingConfig::mappingCornerRadiusSize_US);
    downSizeFilterSurf_US.setRadiusSearch(MappingConfig::mappingSurfRadiusSize_US);


    //大grid
        //内层为x加数据即 x先y后加
    for(int i=0;i<y_up_num;i++){        //外层为y
        for(int j=0;j<x_up_num;j++){   //内层x
//            std::string filename=map_out_path+std::to_string(j)+"_"+std::to_string(i)+feature+".txt";
//            std::ofstream file(filename);
                   //小grid编写
            pcl::PointCloud<PointType>::Ptr out_cloud(new pcl::PointCloud<PointType>);
            for(int n=0;n<SerializeConfig::up2down_num;n++){           //外城为y
                for (int m=0; m<SerializeConfig::up2down_num;m++ ) {   //内层为x
                    int laser_cloud_width=x_up_num*SerializeConfig::up2down_num;
                    int index=(m+j*SerializeConfig::up2down_num)+(n+i*SerializeConfig::up2down_num)*laser_cloud_width;

                    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZI>);

                    for(int k=0;k<frame_sum;k++){                        //循环每一个小pcd
                        if (pcl::io::loadPCDFile<pcl::PointXYZI> (map_in_path+pcd_index[k]+"_"+feature+".pcd", *in_cloud) == -1)
                        {
//                            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
//                            continue;
                        }
                        for (const pcl::PointXYZI& point : *in_cloud){//  循环每一个点
                            PointType transformed_point;
                            transformed_point=PointPoseTrance(point,pcd_tans[k]);
                            if ( transformed_point.x>=(x_min_t+(m+j*SerializeConfig::up2down_num)*(SerializeConfig::up_grid_size/SerializeConfig::up2down_num))&&transformed_point.y >=(y_min_t + (n+i*SerializeConfig::up2down_num)*(SerializeConfig::up_grid_size/SerializeConfig::up2down_num))&&
                                    transformed_point.x < (x_min_t + (m+j*SerializeConfig::up2down_num+1) * (SerializeConfig::up_grid_size/SerializeConfig::up2down_num)) && transformed_point.y < (y_min_t + (n+i*SerializeConfig::up2down_num + 1) * (SerializeConfig::up_grid_size/SerializeConfig::up2down_num))) {
                                transformed_point.down_grid_index=index;
                                out_cloud->push_back(transformed_point);
                            }
                        }
                    }
                }
            }
            std::string filename=map_out_path+std::to_string(j)+"_"+std::to_string(i)+feature+".pcd";
            if(out_cloud->size()!=0){
                if(feature=="corner"){
                    pcl::PointCloud<PointType>::Ptr localMap_corner_ds(new pcl::PointCloud<PointType>);
                    downSizeFilterCorner_US.setInputCloud(out_cloud);
                    downSizeFilterCorner_US.filter(*localMap_corner_ds);
                    pcl::io::savePCDFileBinary (filename, *localMap_corner_ds);
                }
                if(feature=="surf"){
                    pcl::PointCloud<PointType>::Ptr localMap_surf_ds(new pcl::PointCloud<PointType>);
                    downSizeFilterSurf_US.setInputCloud(out_cloud);
                    downSizeFilterSurf_US.filter(*localMap_surf_ds);
                    pcl::io::savePCDFileBinary (filename, *localMap_surf_ds);
                }
            }
            else{
                std::ofstream file(map_out_path+"index.txt",std::ios_base::app);
                file<<feature<<" "<<std::to_string(j)+"_"+std::to_string(i)<<std::endl;
                file.close();
            }
        }
    }
};

void
GolbalMap(const std::string& feature,const std::string& map_out_path,const std::string& map_in_path,const int frame_sum){
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(int k=0;k<frame_sum;k++){                        //循环每一个小pcd
        if (pcl::io::loadPCDFile<pcl::PointXYZI> (map_in_path+pcd_index[k]+"_"+feature+".pcd", *in_cloud) == -1)
        {
//                            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
//                            continue;
        }
        for (const pcl::PointXYZI& point : *in_cloud){//  循环每一个点
            pcl::PointXYZI transformed_point;
            transformed_point=PointPoseTranceGlobal(point,pcd_tans[k]);
            out_cloud->push_back(transformed_point);
        }
    }
    std::string filename=map_out_path+"global"+feature+".pcd";
    pcl::io::savePCDFileBinary (filename, *out_cloud);
}


int
main (int argc, char** argv)
{
    Load_offline_YAML("./config/offline_mapping.yaml");


    LoadTxt(SerializeConfig::map_in_path,SerializeConfig::frame_sum);
    FindMinMaxUsePose(SerializeConfig::map_out_path,SerializeConfig::lidar_range);
    CutDownMapCaulate();
//    CutMap(pcd_feature[0],SerializeConfig::map_out_path,SerializeConfig::map_in_path,SerializeConfig::frame_sum);
//    CutMap(pcd_feature[1],SerializeConfig::map_out_path,SerializeConfig::map_in_path,SerializeConfig::frame_sum);
    GolbalMap(pcd_feature[0],SerializeConfig::map_out_path,SerializeConfig::map_in_path,SerializeConfig::frame_sum);
    GolbalMap(pcd_feature[1],SerializeConfig::map_out_path,SerializeConfig::map_in_path,SerializeConfig::frame_sum);
}