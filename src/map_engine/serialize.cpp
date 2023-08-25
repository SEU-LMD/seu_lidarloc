
// PCL specific includes


#include "serialize.h"


int count=0;
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
LoadTxt()
{
    std::string pcdindex;
    Eigen::Vector3d tanslation;
    Eigen::Quaterniond rotation;
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    std::ifstream downfile(Map_Index_Path);
    Tum tum;
    for(int j=0;j<frame_sum;j++){
        std::string line;
        std::getline(downfile, line);
        std::istringstream iss(line);
        iss >> pcdindex >> tum.tx >> tum.ty >> tum.tz >> tum.qx >> tum.qy >> tum.qz >> tum.qw;
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
        std::cout<< pcd_index[i]<<std::endl;
        std::cout<< pcd_tans[i].matrix()<<std::endl;
        std::cout<< tum_sum[i].tx<<std::endl;
    }
    pcd_feature.push_back("corner");
    pcd_feature.push_back("surf");
}

//通过位姿求地图四个角
void
FindMinMaxUsePose(){
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
    std::cout<<"xmin"<<x_min_t<<std::endl;
    std::cout<<"xmax"<<x_max_t<<std::endl;
    std::cout<<"ymin"<<y_min_t<<std::endl;
    std::cout<<"ymax"<<y_max_t<<std::endl;
}


void
CutDownMapCaulate(){
    x_up_num=static_cast<int>(std::ceil((x_max_t-x_min_t)/up_grid_size));
    y_up_num=static_cast<int>(std::ceil((y_max_t-y_min_t)/up_grid_size));
    up_bloc_count=x_up_num*y_up_num;
    std::cout << "x分区: " << x_up_num << std::endl;
    std::cout << "y分区: " << y_up_num << std::endl;
    std::cout << "分区总数" <<up_bloc_count<<std::endl;
}

pcl::PointXYZ
PointPoseTrance(const pcl::PointXYZ& piont_in,const Eigen::Isometry3d& trans ){
    Eigen::Vector3d piont_trans(piont_in.x,piont_in.y,piont_in.z);
    Eigen::Vector3d piont_transformed=trans*piont_trans;
    pcl::PointXYZ piont_out;
    piont_out.x=piont_transformed.x();
    piont_out.y=piont_transformed.y();
    piont_out.z=piont_transformed.z();
    return piont_out;
}



void
CutMap(std::string feature){
    //大grid
        //内层为x加数据即 x先y后加
    for(int i=0;i<y_up_num;i++){        //外层为y
        for(int j=0;j<x_up_num;j++){   //内层x
            std::string filename=Map_Out_path+std::to_string(j)+"_"+std::to_string(i)+feature+".txt";
            std::ofstream file(filename);
                   //小grid编写
            for(int n=0;n<up2down_num;n++){           //外城为y
                for (int m=0; m<up2down_num;m++ ) {   //内层为x
                    pcl::PointCloud<PointType>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PointCloud<PointType>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    for(int k=0;k<frame_sum;k++){                        //循环每一个小pcd
                        if (pcl::io::loadPCDFile<pcl::PointXYZ> (Map_In_path+pcd_index[k]+"_"+feature+".pcd", *in_cloud) == -1)
                        {
//                            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
//                            continue;
                        }
                        for (const pcl::PointXYZ& point : *in_cloud){//  循环每一个点
                            pcl::PointXYZ transformed_point;
                            transformed_point=PointPoseTrance(point,pcd_tans[k]);
                            if ( transformed_point.x>=(x_min_t+(m+j*up2down_num)*(up_grid_size/up2down_num))&&transformed_point.y >=(y_min_t + (n+i*up2down_num)*(up_grid_size/up2down_num))&&
                                    transformed_point.x < (x_min_t + (m+j*up2down_num+1) * (up_grid_size/up2down_num)) && transformed_point.y < (y_min_t + (n+i*up2down_num + 1) * (up_grid_size/up2down_num))) {
                                out_cloud->push_back(transformed_point);
                            }
                        }
                    }
                    file<<(m+j*up2down_num)<<"_"<<(n+i*up2down_num)<<" "<<out_cloud->size()<<std::endl;
                    for (const auto& point : *out_cloud) {
                        file<< point.x << " " << point.y << " " << point.z << std::endl; // 将点的坐标写入文件
                        write_sum++;
                    }
                    if(write_sum!=out_cloud->size()){
                        std::cout<<"wrong in"<<(m+j*up2down_num)<<" "<<(n+i*up2down_num);
                        write_sum=0;
                        exit(1);
                    }
                    else{
                        write_sum=0;
                    }
                    in_cloud->clear();
                    out_cloud->clear();
                }

            }
            file.close();
        }
    }
};




int
main (int argc, char** argv)
{
    LoadTxt();
    FindMinMaxUsePose();
    CutDownMapCaulate();
    CutMap(pcd_feature[0]);
    CutMap(pcd_feature[1]);
}