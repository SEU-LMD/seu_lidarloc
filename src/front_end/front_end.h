//
// Created by lsy on 23-11-15.
//

#ifndef SEU_LIDARLOC_FRONT_END_H
#define SEU_LIDARLOC_FRONT_END_H

#include "utils/utility.h"
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "pcl/features/normal_3d_omp.h" //by wxq
#include "pcl/segmentation/sac_segmentation.h"

#include <mutex>
#include <thread>

#include "feature_extraction.h"
#include "utils/timer.h"
#include "opencv2/opencv.hpp"   // for opencv4
//#include <opencv/cv.h>
//#include "pcl/features/normal_3d_omp.h" //by wxq
//#include "pcl/segmentation/sac_segmentation.h"
#include "utils/filesys.h"

#include <fstream>

struct grid_t
{
    std::vector<int> point_id;
    float min_z;
    float max_z;
    float delta_z;
    float min_z_x; //X of Lowest Point in the Voxel;
    float min_z_y; //Y of Lowest Point in the Voxel;
    float min_z_outlier_thre;
    float neighbor_min_z;
    int pts_count;
    int reliable_neighbor_grid_num;
    float mean_z;
    float dist2station;//这个grid距离激光的距离

    grid_t()
    {
        min_z = min_z_x = min_z_y = neighbor_min_z = mean_z = 0.f;
        pts_count = 0;
        reliable_neighbor_grid_num = 0;
        delta_z = 0.0;
        dist2station = 0.001;
        min_z_outlier_thre = -FLT_MAX;
    }
};

struct bounds_t
{
    double min_x;
    double min_y;
    double min_z;
    double max_x;
    double max_y;
    double max_z;
    int type;

    bounds_t()
    {
        min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
    }
    void inf_x()
    {
        min_x = -DBL_MAX;
        max_x = DBL_MAX;
    }
    void inf_y()
    {
        min_y = -DBL_MAX;
        max_y = DBL_MAX;
    }
    void inf_z()
    {
        min_z = -DBL_MAX;
        max_z = DBL_MAX;
    }
    void inf_xyz()
    {
        inf_x();
        inf_y();
        inf_z();
    }
};

struct centerpoint_t
{
    double x;
    double y;
    double z;
    centerpoint_t(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

void get_cloud_bbx(const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud, bounds_t &bound)
{
    double min_x = DBL_MAX;
    double min_y = DBL_MAX;
    double min_z = DBL_MAX;
    double max_x = -DBL_MAX;
    double max_y = -DBL_MAX;
    double max_z = -DBL_MAX;
    //
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (min_x > cloud->points[i].x)
            min_x = cloud->points[i].x;
        if (min_y > cloud->points[i].y)
            min_y = cloud->points[i].y;
        if (min_z > cloud->points[i].z)
            min_z = cloud->points[i].z;
        if (max_x < cloud->points[i].x)
            max_x = cloud->points[i].x;
        if (max_y < cloud->points[i].y)
            max_y = cloud->points[i].y;
        if (max_z < cloud->points[i].z)
            max_z = cloud->points[i].z;
    }
    bound.min_x = min_x;
    bound.max_x = max_x;
    bound.min_y = min_y;
    bound.max_y = max_y;
    bound.min_z = min_z;
    bound.max_z = max_z;
}

void get_cloud_bbx_cpt(const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud, bounds_t &bound, centerpoint_t &cp)
{
    get_cloud_bbx(cloud, bound);
    cp.x = 0.5 * (bound.min_x + bound.max_x);
    cp.y = 0.5 * (bound.min_y + bound.max_y);
    cp.z = 0.5 * (bound.min_z + bound.max_z);
}



struct eigenvalue_t // Eigen Value ,lamada1 > lamada2 > lamada3;
{
    double lamada1;
    double lamada2;
    double lamada3;
};

struct eigenvector_t //the eigen vector corresponding to the eigen value
{
    Eigen::Vector3f principalDirection;
    Eigen::Vector3f middleDirection;
    Eigen::Vector3f normalDirection;
};

struct pca_feature_t //PCA
{
    eigenvalue_t values;
    eigenvector_t vectors;
    double curvature;
    double linear;
    double planar;
    double spherical;
    double linear_2;
    double planar_2;
    double spherical_2;
    double normal_diff_ang_deg;
    pcl::PointNormal pt;
    int ptId;
    int pt_num = 0;
    std::vector<int> neighbor_indices;
    std::vector<bool> close_to_query_point;
};

//is_palne_feature (true: assign point normal as pca normal vector, false: assign point normal as pca primary direction vector)
bool assign_normal(PointXYZICOLRANGE &pt, pca_feature_t &pca_feature, bool is_plane_feature = true)
{
    if (is_plane_feature)
    {
//            EZLOG(INFO)<< "select points step 1" << std::endl;
        pt.normal_x = pca_feature.vectors.normalDirection.x();
//            EZLOG(INFO)<< "select points step 2" << std::endl;
        pt.normal_y = pca_feature.vectors.normalDirection.y();
//            EZLOG(INFO)<< "select points step 3" << std::endl;
        pt.normal_z = pca_feature.vectors.normalDirection.z();
//            EZLOG(INFO)<< "select points step 4" << std::endl;
        pt.normal[3] = pca_feature.planar_2; //planrity
    }
    else
    {
        pt.normal_x = pca_feature.vectors.principalDirection.x();
        pt.normal_y = pca_feature.vectors.principalDirection.y();
        pt.normal_z = pca_feature.vectors.principalDirection.z();
        pt.normal[3] = pca_feature.linear_2; //linarity
    }
    return true;
}

/**
    * \brief Use PCL to accomplish the Principle Component Analysis (PCA)
    * of one point and its neighborhood
    * \param[in] in_cloud is the input Point Cloud Pointer
    * \param[in] search_indices is the neighborhood points' indices of the search point.
    * \param[out]feature is the pca_feature_t of the search point.
    */
bool get_pca_feature(typename pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                     std::vector<int> &search_indices,
                     pca_feature_t &feature)
{
    int pt_num = search_indices.size();

    if (pt_num <= 3)
        return false;

    pcl::PointCloud<PointXYZICOLRANGE>::Ptr selected_cloud(new pcl::PointCloud<PointXYZICOLRANGE>());
    const int maxIdx = in_cloud->points.size();
//        EZLOG(INFO)<< "select points" << std::endl;
    for (int i = 0; i < pt_num; ++i)
        selected_cloud->points.push_back(in_cloud->points[search_indices[i]]);
//        EZLOG(INFO)<< "select points finish" << std::endl;
    pcl::PCA<PointXYZICOLRANGE> pca_operator;
    pca_operator.setInputCloud(selected_cloud);

    // Compute eigen values and eigen vectors
    Eigen::Matrix3f eigen_vectors = pca_operator.getEigenVectors();
    Eigen::Vector3f eigen_values = pca_operator.getEigenValues();

    feature.vectors.principalDirection = eigen_vectors.col(0);
    feature.vectors.normalDirection = eigen_vectors.col(2);

    feature.vectors.principalDirection.normalize();
    feature.vectors.normalDirection.normalize();

    feature.values.lamada1 = eigen_values(0);
    feature.values.lamada2 = eigen_values(1);
    feature.values.lamada3 = eigen_values(2);

    if ((feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3) == 0)
        feature.curvature = 0;
    else
        feature.curvature = feature.values.lamada3 / (feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3);

    // feature.linear_2 = (sqrt(feature.values.lamada1) - sqrt(feature.values.lamada2)) / sqrt(feature.values.lamada1);
    // feature.planar_2 = (sqrt(feature.values.lamada2) - sqrt(feature.values.lamada3)) / sqrt(feature.values.lamada1);
    // feature.spherical_2 = sqrt(feature.values.lamada3) / sqrt(feature.values.lamada1);
    feature.linear_2 = ((feature.values.lamada1) - (feature.values.lamada2)) / (feature.values.lamada1);
    feature.planar_2 = ((feature.values.lamada2) - (feature.values.lamada3)) / (feature.values.lamada1);
    feature.spherical_2 = (feature.values.lamada3) / (feature.values.lamada1);

    search_indices.swap(feature.neighbor_indices);
    return true;
}

// R - K neighborhood (with already built-kd tree)
//within the radius, we would select the nearest K points for calculating PCA
bool get_pc_pca_feature(pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                        std::vector<pca_feature_t> &features,
                        pcl::KdTreeFLANN<PointXYZICOLRANGE>::Ptr &tree,
                        float radius, int nearest_k, int min_k = 1, int pca_down_rate = 1,
                        bool distance_adaptive_on = false, float unit_dist = 35.0)
{
    //LOG(INFO) << "[" << in_cloud->points.size() << "] points used for PCA, pca down rate is [" << pca_down_rate << "]";
    features.resize(in_cloud->points.size());
//        EZLOG(INFO) << "start get pca feature "<< std::endl;
    for (int i = 0; i < in_cloud->points.size(); i += pca_down_rate) //faster way
    {
        // if (i % pca_down_rate == 0) {//this way is much slower
        std::vector<int> search_indices_used; //points would be stored in sequence (from the closest point to the farthest point within the neighborhood)
        std::vector<int> search_indices;	  //point index vector
        std::vector<float> squared_distances; //distance vector

        float neighborhood_r = radius;
        int neighborhood_k = nearest_k;

        if (distance_adaptive_on)
        {
            double dist = std::sqrt(in_cloud->points[i].x * in_cloud->points[i].x +
                                    in_cloud->points[i].y * in_cloud->points[i].y +
                                    in_cloud->points[i].z * in_cloud->points[i].z);
            if (dist > unit_dist)
            {
                neighborhood_r = std::sqrt(dist / unit_dist) * radius;
                //neighborhood_k = (int)(unit_dist / dist * nearest_k));
            }
        }
        //nearest_k=0 --> the knn is disabled, only the rnn is used
        tree->radiusSearch(i, neighborhood_r, search_indices, squared_distances, neighborhood_k);

        features[i].pt.x = in_cloud->points[i].x;
        features[i].pt.y = in_cloud->points[i].y;
        features[i].pt.z = in_cloud->points[i].z;
        features[i].ptId = i;
        features[i].pt_num = search_indices.size();

        //deprecated
        features[i].close_to_query_point.resize(search_indices.size());
        for (int j = 0; j < search_indices.size(); j++)
        {
            if (squared_distances[j] < 0.64 * radius * radius) // 0.5^(2/3)
                features[i].close_to_query_point[j] = true;
            else
                features[i].close_to_query_point[j] = false;
        }
//            EZLOG(INFO) << "get current point feature"<< std::endl;
        get_pca_feature(in_cloud, search_indices, features[i]);
//            EZLOG(INFO) << "after current point feature, start assign normal"<< std::endl;
        if (features[i].pt_num > min_k)
            assign_normal(in_cloud->points[i], features[i]);
//            EZLOG(INFO) << "after assign normal" << std::endl;
        std::vector<int>().swap(search_indices);
//            EZLOG(INFO) << "after swap 1" << std::endl;
        std::vector<int>().swap(search_indices_used);
//            EZLOG(INFO) << "after swap 2" << std::endl;
        std::vector<float>().swap(squared_distances);
//            EZLOG(INFO) << "after swap 3" << std::endl;
    }
//        EZLOG(INFO) << "pca over! "<< std::endl;
    //}
    return true;
}

bool classify_nground_pts(pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud_in,//input 非地面点
                          pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar,
                          pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam,
                          pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade,
                          pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof,
                          pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar_down,
                          pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam_down,
                          pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade_down,
                          pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof_down,
//                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_vertex,
//                              float neighbor_searching_radius, int neighbor_k, int neigh_k_min, int pca_down_rate, // one in ${pca_down_rate} unground points would be select as the query points for calculating pca, the else would only be used as neighborhood points
//                              float edge_thre, float planar_thre, float edge_thre_down, float planar_thre_down,
//                              int extract_vertex_points_method, float curvature_thre, float vertex_curvature_non_max_radius,
//                              float linear_vertical_sin_high_thre, float linear_vertical_sin_low_thre,
//                              float planar_vertical_sin_high_thre, float planar_vertical_sin_low_thre,
//                              bool fixed_num_downsampling = false, int pillar_down_fixed_num = 200, int facade_down_fixed_num = 800, int beam_down_fixed_num = 200,
//                              int roof_down_fixed_num = 100, int unground_down_fixed_num = 20000,
                          float beam_height_max = FLT_MAX, float roof_height_min = -FLT_MAX
//                              float feature_pts_ratio_guess = 0.3, bool sharpen_with_nms = true,
//                              bool use_distance_adaptive_pca = false
)
{

//        if (fixed_num_downsampling) //false
//            random_downsample_pcl(cloud_in, unground_down_fixed_num);

    //Do PCA
//        PrincipleComponentAnalysis<PointT> pca_estimator;
    std::vector<pca_feature_t> cloud_features;

    typename pcl::KdTreeFLANN<PointXYZICOLRANGE>::Ptr tree(new pcl::KdTreeFLANN<PointXYZICOLRANGE>);
    tree->setInputCloud(cloud_in);

    float unit_distance = 30.0;
    ///1.计算每个非地面点的pca参数
    //output  param = cloud_features
    get_pc_pca_feature(cloud_in, cloud_features, tree, FrontEndConfig::neighbor_searching_radius, FrontEndConfig::neighbor_k, 1, FrontEndConfig::pca_down_rate, FrontEndConfig::use_distance_adaptive_pca, unit_distance);
    //LOG(WARNING)<< "PCA done";

    std::chrono::steady_clock::time_point toc_pca = std::chrono::steady_clock::now();

    //the radius should be larger for far away points
    //1.2.遍历每个非地面点，并根据pca参数为每个点进行归类
    std::vector<int> index_with_feature(cloud_in->points.size(), 0); // 0 - not special points, 1 - pillar, 2 - beam, 3 - facade, 4 - roof
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        if (cloud_features[i].pt_num > FrontEndConfig::neigh_k_min)//后面所有的代码都在这个if下，离群点不对他归类
        {

            if (cloud_features[i].linear_2 > FrontEndConfig::edge_thre)
            {
                if (std::abs(cloud_features[i].vectors.principalDirection.z()) > FrontEndConfig::linear_vertical_sin_high_thre)
                {
                    assign_normal(cloud_in->points[i], cloud_features[i], false);
                    cloud_pillar->points.push_back(cloud_in->points[i]);
                    index_with_feature[i] = 1;
                }
                else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < FrontEndConfig::linear_vertical_sin_low_thre &&
                         cloud_in->points[i].z < beam_height_max)
                {
                    assign_normal(cloud_in->points[i], cloud_features[i], false);
                    cloud_beam->points.push_back(cloud_in->points[i]);
                    index_with_feature[i] = 2;
                }
                else
                {
                    ;
                }

                if (!FrontEndConfig::sharpen_with_nms && cloud_features[i].linear_2 > FrontEndConfig::edge_thre_down)
                {
                    if (std::abs(cloud_features[i].vectors.principalDirection.z()) > FrontEndConfig::linear_vertical_sin_high_thre)
                        cloud_pillar_down->points.push_back(cloud_in->points[i]);
                    else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < FrontEndConfig::linear_vertical_sin_low_thre && cloud_in->points[i].z < beam_height_max)
                        cloud_beam_down->points.push_back(cloud_in->points[i]);
                    else
                    {
                        ;
                    }
                }
            }//end if (cloud_features[i].linear_2 > edge_thre)

            else if (cloud_features[i].planar_2 > FrontEndConfig::planar_thre)
            {
                if (std::abs(cloud_features[i].vectors.normalDirection.z()) > FrontEndConfig::planar_vertical_sin_high_thre && cloud_in->points[i].z > roof_height_min)
                {
                    assign_normal(cloud_in->points[i], cloud_features[i], true);
                    cloud_roof->points.push_back(cloud_in->points[i]);
                    index_with_feature[i] = 4;
                }
                else if (std::abs(cloud_features[i].vectors.normalDirection.z()) < FrontEndConfig::planar_vertical_sin_low_thre)
                {
                    assign_normal(cloud_in->points[i], cloud_features[i], true);
                    cloud_facade->points.push_back(cloud_in->points[i]);
                    index_with_feature[i] = 3;
                }
                else
                {
                    ;
                }
                if (!FrontEndConfig::sharpen_with_nms && cloud_features[i].planar_2 > FrontEndConfig::planar_thre_down)
                {
                    if (std::abs(cloud_features[i].vectors.normalDirection.z()) > FrontEndConfig::planar_vertical_sin_high_thre && cloud_in->points[i].z > roof_height_min)
                        cloud_roof_down->points.push_back(cloud_in->points[i]);
                    else if (std::abs(cloud_features[i].vectors.normalDirection.z()) < FrontEndConfig::planar_vertical_sin_low_thre)
                        cloud_facade_down->points.push_back(cloud_in->points[i]);
                    else
                    {
                        ;
                    }
                }
            }
        }
    } // end for (int i = 0; i < cloud_in->points.size(); i++)

    //According to the parameter 'extract_vertex_points_method' (0,1,2...)
    if (FrontEndConfig::curvature_thre < 1e-8) // set stablilty_thre as 0 to disable the vertex extraction
        FrontEndConfig::extract_vertex_points_method = 0;

    //Find Edge points by picking high curvature points among the neighborhood of unground geometric feature points (2)
    //1.3.从非特殊点中再根据这个点的周围点的信息，将其归类为beam或者pillar点
    if (FrontEndConfig::extract_vertex_points_method == 2)
    {
        float vertex_feature_ratio_thre = FrontEndConfig::feature_pts_ratio_guess / FrontEndConfig::pca_down_rate;
        for (int i = 0; i < cloud_in->points.size(); i++)
        {
            // if (index_with_feature[i] == 0)
            // 	cloud_vertex->points.push_back(cloud_in->points[i]);
            //这个点是非特殊点，且这个点周围有足够多的点，且这个点的曲率非常大
            if (index_with_feature[i] == 0 &&
                cloud_features[i].pt_num > FrontEndConfig::neigh_k_min &&
                cloud_features[i].curvature > FrontEndConfig::curvature_thre) //curvature_thre means curvature_thre here
            {
                int geo_feature_point_count = 0;
                for (int j = 0; j < cloud_features[i].neighbor_indices.size(); j++)
                {
                    if (index_with_feature[cloud_features[i].neighbor_indices[j]])
                        geo_feature_point_count++;
                }
                //LOG(INFO)<< "facade neighbor num: " <<geo_feature_point_count;
                //这个非特殊点周围点特殊点也要足够多
                if (1.0 * geo_feature_point_count / cloud_features[i].pt_num > vertex_feature_ratio_thre) //most of the neighbors are feature points
                {
                    //cloud_vertex->points.push_back(cloud_in->points[i]);

                    assign_normal(cloud_in->points[i], cloud_features[i], false);
                    cloud_in->points[i].normal[3] = 5.0 * cloud_features[i].curvature; //save in the un-used normal[3]  (PointNormal4D)
                    if (std::abs(cloud_features[i].vectors.principalDirection.z()) > FrontEndConfig::linear_vertical_sin_high_thre)
                    {
                        cloud_pillar->points.push_back(cloud_in->points[i]);
                        //cloud_pillar_down->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 1;
                    }
                    else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < FrontEndConfig::linear_vertical_sin_low_thre && cloud_in->points[i].z < beam_height_max)
                    {
                        cloud_beam->points.push_back(cloud_in->points[i]);
                        //cloud_beam_down->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 2;
                    }
                }
            }
        }
    }

    //if extract_vertex_points_method == 0 ---> do not extract vertex points (0)
//        std::chrono::steady_clock::time_point toc_1 = std::chrono::steady_clock::now();
    //extract neighborhood feature descriptor for pillar points
    //Find Vertex (Edge) points by picking points with maximum local curvature (1)
    //if (extract_vertex_points_method == 1) //Deprecated
    //detect_key_pts(cloud_in, cloud_features, index_with_feature,cloud_vertex, 4.0 * curvature_thre, vertex_curvature_non_max_radius, 0.5 * curvature_thre);
    int min_neighbor_feature_pts = (int)(FrontEndConfig::feature_pts_ratio_guess / FrontEndConfig::pca_down_rate * FrontEndConfig::neighbor_k) - 1;

    //get the vertex keypoints and encode its neighborhood in a simple descriptor
    ///2.为某个点生成描述子
//        encode_stable_points(cloud_in, cloud_vertex, cloud_features, index_with_feature,
//                             0.3 * curvature_thre, min_neighbor_feature_pts, neigh_k_min); //encode the keypoints, we will get a simple descriptor of the putable keypoints

    //LOG(WARNING)<< "encode ncc feature descriptor done";


    //Non_max_suppression of the feature points //TODO: add already built-kd tree here
    ///3.1对pillar cloud_facade beam cloud_roof 特征点进行非最大值抑制
//        if (sharpen_with_nms)
//        {
//            float nms_radius = 0.25 * neighbor_searching_radius;
//#pragma omp parallel sections
//            {
//#pragma omp section
//                {
//                    if (pillar_down_fixed_num > 0)
//                        non_max_suppress(cloud_pillar, cloud_pillar_down, nms_radius);
//                }
//#pragma omp section
//                {
//                    if (facade_down_fixed_num > 0)
//                        non_max_suppress(cloud_facade, cloud_facade_down, nms_radius);
//                }
//#pragma omp section
//                {
//                    if (beam_down_fixed_num > 0)
//                        non_max_suppress(cloud_beam, cloud_beam_down, nms_radius);
//
//                    if (roof_down_fixed_num > 0)
//                        non_max_suppress(cloud_roof, cloud_roof_down, nms_radius);
//                }
//            }
//        }

    ///3.2.对cloud_facade cloud_beam_down cloud_roof_down 将点云分成不同sector，然后在sector中进行随机采样
//        if (fixed_num_downsampling)
//        {
//            random_downsample_pcl(cloud_pillar_down, pillar_down_fixed_num);
//            int sector_num = 4;
//            xy_normal_balanced_downsample(cloud_facade_down, (int)(facade_down_fixed_num / sector_num), sector_num);
//
//            xy_normal_balanced_downsample(cloud_beam_down, (int)(beam_down_fixed_num / sector_num), sector_num); // here the normal is the primary vector
//            //random_downsample_pcl(cloud_roof_down, 100);
//            random_downsample_pcl(cloud_roof_down, roof_down_fixed_num);
//        }

    //Free the memory
    std::vector<pca_feature_t>().swap(cloud_features);
    std::vector<int>().swap(index_with_feature);



    return 1;
} //end classify_nground_pts








void fast_ground_filter(
        const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud_in,
        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_ground,//地面点
        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_ground_down,//降才采样之后的地面点
        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_unground,//非地面点
//                            int min_grid_pt_num,
        float max_ground_height = FLT_MAX,
//                            float grid_resolution,
//                            int distance_weight_downsampling_method,float standard_distance,int nonground_random_down_rate,
        float intensity_thre = FLT_MAX
//                            bool apply_grid_wise_outlier_filter = false,float outlier_std_scale = 3.0,
//                            int reliable_neighbor_grid_num_thre = 0, int ground_random_down_rate = 1,float neighbor_height_diff = 25,
//                             float max_height_difference = 0.3,int estimate_ground_normal_method = 3,
//                            float normal_estimation_radius = 2.0, bool fixed_num_downsampling = false,int ground_random_down_down_rate = 2

//                            ,  //estimate_ground_normal_method, 0: directly use (0,0,1), 1: estimate normal in fix radius neighborhood , 2: estimate normal in k nearest neighborhood, 3: use ransac to estimate plane coeffs in a grid
//                               //standard distance: the distance where the distance_weight is 1
//                           , int down_ground_fixed_num = 1000,
//                            bool detect_curb_or_not = false,
){

    //0.主要功能是计算点云统计信息,为地面点提取做参数准备。计算出平均高度、地面点判定阈值、
    //地面最小高度、地下噪声点阈值等,并获取点云范围信息。这些都是构建grid和提取地面点需要的重要参数
    //For some points,  calculating the approximate mean height

    int reliable_grid_pts_count_thre = FrontEndConfig::min_grid_pt_num - 1;
    int count_checkpoint = 0;
    float sum_height = 0.001;
    float appro_mean_height;//当前帧的平均高度
    float min_ground_height = max_ground_height;
    float underground_noise_thre = -FLT_MAX;
    float non_ground_height_thre;//等于平均高度+设定的地面最大高度
    float distance_weight;
    // int ground_random_down_rate_temp = ground_random_down_rate;
    // int nonground_random_down_rate_temp = nonground_random_down_rate;
    EZLOG(INFO)<<"cloud_in->size() = "<<cloud_in->size()<<endl;

    for (int j = 0; j < cloud_in->size(); j++)
    {
        if (j % 100 == 0)
        {
            sum_height += cloud_in->points[j].z;
            count_checkpoint++;
        }
    }

    appro_mean_height = sum_height / count_checkpoint; //calculate around height
    non_ground_height_thre = appro_mean_height + max_ground_height;//max_ground_height  = 1.5
    EZLOG(INFO)<<"appro_mean_height "<<appro_mean_height;
    EZLOG(INFO)<<"non_ground_height_thre "<<non_ground_height_thre;

    bounds_t bounds;
    centerpoint_t center_pt;
    get_cloud_bbx_cpt(cloud_in, bounds, center_pt);

    //1.构建grid map
    int row, col, num_grid;
    row = ceil((bounds.max_y - bounds.min_y) / FrontEndConfig::grid_resolution);//grid_resolution 默认参数 = 3.0
    col = ceil((bounds.max_x - bounds.min_x) / FrontEndConfig::grid_resolution);
    num_grid = row * col;
    EZLOG(INFO)<<"num_grid "<<num_grid;

    grid_t *grid = new grid_t[num_grid];

    //Each grid
    for (int i = 0; i < num_grid; i++)
    {
        grid[i].min_z = FLT_MAX;
        grid[i].neighbor_min_z = FLT_MAX;
    }

    //Each point ---> determine the grid to which the point belongs
    for (int j = 0; j < cloud_in->points.size(); j++)
    {
        int temp_row, temp_col, temp_id;
        temp_col = floor((cloud_in->points[j].x - bounds.min_x) / FrontEndConfig::grid_resolution);
        temp_row = floor((cloud_in->points[j].y - bounds.min_y) / FrontEndConfig::grid_resolution);
        temp_id = temp_row * col + temp_col;
        if (temp_id >= 0 && temp_id < num_grid)
        {
            ///if use distance_weight_downsampling and the point is the first point in grid
            ///calculate distance from grid to origin
            if (FrontEndConfig::distance_weight_downsampling_method > 0 && !grid[temp_id].pts_count)
            {
                grid[temp_id].dist2station = std::sqrt(cloud_in->points[j].x * cloud_in->points[j].x + cloud_in->points[j].y * cloud_in->points[j].y + cloud_in->points[j].z * cloud_in->points[j].z);
            }

            if (cloud_in->points[j].z > non_ground_height_thre)//non_ground_height_thre = 等于平均高度+设定的地面最大高度
            {
                //standard_distance 默认参数 = 15.0
                //要注意这里作者对非地面点做了一个随机下采样
                distance_weight = 1.0 * FrontEndConfig::standard_distance / (grid[temp_id].dist2station + 0.0001); //avoiding Floating point exception
                int nonground_random_down_rate_temp = FrontEndConfig::nonground_random_down_rate;
                if (FrontEndConfig::distance_weight_downsampling_method == 1) //linear weight
                    nonground_random_down_rate_temp = (int)(distance_weight * FrontEndConfig::nonground_random_down_rate + 1);
                else if (FrontEndConfig::distance_weight_downsampling_method == 2) //quadratic weight
                    nonground_random_down_rate_temp = (int)(distance_weight * distance_weight * FrontEndConfig::nonground_random_down_rate + 1);

                ///对点进行随机下采样,满足下采样率要求或者强度高的点会被保留,save in cloud_unground.
                if (j % nonground_random_down_rate_temp == 0 || cloud_in->points[j].intensity > intensity_thre)//intensity_thre = double最大值
                {
                    cloud_in->points[j].data[3] = cloud_in->points[j].z - (appro_mean_height - 3.0); //data[3] stores the approximate point height above ground
                    cloud_unground->points.push_back(cloud_in->points[j]);
                }
            }
                ///处理可能属于地面的点(高度低于非地面阈值,但高于地下噪声阈值)
            else if (cloud_in->points[j].z > underground_noise_thre)
            {
                grid[temp_id].pts_count++;
                grid[temp_id].point_id.push_back(j);
                if (cloud_in->points[j].z < grid[temp_id].min_z) //update min_z and neighbor_min_z
                {
                    grid[temp_id].min_z = cloud_in->points[j].z;
                    grid[temp_id].neighbor_min_z = cloud_in->points[j].z;
                }
            }
        } // end if (temp_id >= 0 && temp_id < num_grid)
    } // end for (int j = 0; j < cloud_in.cloud_ptr->points.size(); j++)

//        EZLOG(INFO)<<"num_unground = "<<num_unground<<endl;
//        EZLOG(INFO)<<"cloud_unground->points.size() = "<<cloud_unground->points.size()<<endl;
//
//        TicToc time_3;

    if (FrontEndConfig::apply_grid_wise_outlier_filter)
    {
        //Each grid: Check outlier //calculate mean and standard deviation of z in one grid, then set mean-2*std as the threshold for outliers
        for (int i = 0; i < num_grid; i++)
        {
            if (grid[i].pts_count >= FrontEndConfig::min_grid_pt_num)///判断该grid中的点数量是否满足最小阈值min_grid_pt_num
            {
                double sum_z = 0, sum_z2 = 0, std_z = 0, mean_z = 0;
                for (int j = 0; j < grid[i].point_id.size(); j++)
                    sum_z += cloud_in->points[grid[i].point_id[j]].z;
                mean_z = sum_z / grid[i].pts_count;
                for (int j = 0; j < grid[i].point_id.size(); j++)
                    sum_z2 += (cloud_in->points[grid[i].point_id[j]].z - mean_z) * (cloud_in->points[grid[i].point_id[j]].z - mean_z);
                std_z = std::sqrt(sum_z2 / grid[i].pts_count);
                grid[i].min_z_outlier_thre = mean_z - FrontEndConfig::outlier_std_scale * std_z;///calculate outlier threshold
                grid[i].min_z = max_(grid[i].min_z, grid[i].min_z_outlier_thre);///更新grid最小高度grid[i].min_z,取原始最小高度和离群点阈值中的最大值
                grid[i].neighbor_min_z = grid[i].min_z;///将更新后的最小高度同步到相邻grid中
            }
        }
    } //end if (apply_grid_wise_outlier_filter)

//        EZLOG(INFO)<<"time_3.toc() = "<<time_3.toc();
//
//        TicToc time_4;

    //2.遍历所有的grid
    for (int m = 0; m < num_grid; m++)
    {
        int temp_row, temp_col;
        temp_row = m / col;
        temp_col = m % col;
        if (temp_row >= 1 && temp_row <= row - 2 && temp_col >= 1 && temp_col <= col - 2)///判断是否位于边界之内
        {
            ///遍历上下左右的相邻grid
            for (int j = -1; j <= 1; j++) //row
            {
                for (int k = -1; k <= 1; k++) //col
                {
                    ///将当前grid的neighbor_min_z更新为相邻grid的min_z的最小值,同时统计相邻grid中点数大于阈值的grid数量,记为reliable_neighbor_grid_num
                    grid[m].neighbor_min_z = min_(grid[m].neighbor_min_z, grid[m + j * col + k].min_z);
//                        EZLOG(INFO)<<"grid[m].neighbor_min_z = "<<grid[m].neighbor_min_z<<endl;
                    if (grid[m + j * col + k].pts_count > reliable_grid_pts_count_thre)
                        grid[m].reliable_neighbor_grid_num++;
                }
            }
        }
    }
//
    //***********
    std::vector<std::shared_ptr<pcl::PointCloud<PointXYZICOLRANGE>>> grid_ground_pcs(num_grid);
    std::vector<std::shared_ptr<pcl::PointCloud<PointXYZICOLRANGE>>> grid_unground_pcs(num_grid);
    for (int i = 0; i < num_grid; i++)
    {
        std::shared_ptr<pcl::PointCloud<PointXYZICOLRANGE>> grid_ground_pc_temp(new pcl::PointCloud<PointXYZICOLRANGE>);
        grid_ground_pcs[i] = grid_ground_pc_temp;
        std::shared_ptr<pcl::PointCloud<PointXYZICOLRANGE>> grid_unground_pc_temp(new pcl::PointCloud<PointXYZICOLRANGE>);
        grid_unground_pcs[i] = grid_unground_pc_temp;
    }

//        int use_grid_num = 0;
//        EZLOG(INFO)<<"num_grid = "<<num_grid;
    int estimate_normal_num = 0;
    double estimate_normal_time = 0;
    double ransac_time = 0;

//        ///use for thread
//        {
//            void dealwith_grid_thread(const int start_index, const int end_index,...){
//                for(int i = start_index; i < end_index; ++i){
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr grid_ground(new pcl::PointCloud<PointXYZICOLRANGE>);
//                    //do_something
//                }
//            }
//            int num_each_grid = num_grid / 4;
//            int num_thread = 4;
//            std::vector<std::thread> threads;
//            for(int i = 0; i < num_thread; ++i){
//                int start = i * num_each_grid;
//                int end = start + num_each_grid;
//                end = std::min(end, num_grid);
//                std::thread th(dealwith_grid_thread, start, end, ...);
//                threads.push_back( std::move(th));
//            }
//            for(int i = 0; i < num_thread; ++i){
//                threads[i].join();
//            }
//        }


//        //For each grid
    for (int i = 0; i < num_grid; i++)
    {
        pcl::PointCloud<PointXYZICOLRANGE>::Ptr grid_ground(new pcl::PointCloud<PointXYZICOLRANGE>);
        ///过滤掉点数太少的grid
        //min_grid_pt_num = 8,reliable_neighbor_grid_num_thre = 0
//            EZLOG(INFO)<<"grid[i].pts_count = "<<grid[i].pts_count<<endl;
//            EZLOG(INFO)<<"grid[i].reliable_neighbor_grid_num = "<<grid[i].reliable_neighbor_grid_num<<endl;
        if (grid[i].pts_count >= FrontEndConfig::min_grid_pt_num && grid[i].reliable_neighbor_grid_num >= FrontEndConfig::reliable_neighbor_grid_num_thre)
        {
//                ++use_grid_num;

            ///计算距离权重,用于地面点和非地面点的下采样率
            int ground_random_down_rate_temp = FrontEndConfig::ground_random_down_rate;
            int nonground_random_down_rate_temp = FrontEndConfig::nonground_random_down_rate;
            distance_weight = 1.0 * FrontEndConfig::standard_distance / (grid[i].dist2station + 0.0001);
            if (FrontEndConfig::distance_weight_downsampling_method == 1) //linear weight
            {
                ground_random_down_rate_temp = (int)(distance_weight * FrontEndConfig::ground_random_down_rate + 1);
                nonground_random_down_rate_temp = (int)(distance_weight * FrontEndConfig::nonground_random_down_rate + 1);
            }
            else if (FrontEndConfig::distance_weight_downsampling_method == 2) //quadratic weight
            {
                ground_random_down_rate_temp = (int)(distance_weight * distance_weight * FrontEndConfig::ground_random_down_rate + 1);
                nonground_random_down_rate_temp = (int)(distance_weight * distance_weight * FrontEndConfig::nonground_random_down_rate + 1);
            }

            bool last_point_is_ground = false;

            ///如果grid与附近grid差小于阈值,说明是地面grid:
//                EZLOG(INFO)<<"grid[i].min_z  = "<<grid[i].min_z<<endl;
//                EZLOG(INFO)<<"grid[i].neighbor_min_z  = "<<grid[i].neighbor_min_z<<endl;
//                EZLOG(INFO)<<"grid[i].min_z - grid[i].neighbor_min_z = "<<grid[i].min_z - grid[i].neighbor_min_z<<endl;
            if (grid[i].min_z - grid[i].neighbor_min_z < FrontEndConfig::neighbor_height_diff)
            {
//                    TicToc time_7;

                ///遍历grid中的每个点,根据阈值提取地面点
                for (int j = 0; j < grid[i].point_id.size(); ++j )
                {

                    if(last_point_is_ground){
                        grid_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                        last_point_is_ground = false;
                        continue;
                    }
                        /// 判断点的高度是否超过该grid的离群点高度阈值min_z_outlier_thre
                    else if (cloud_in->points[grid[i].point_id[j]].z > grid[i].min_z_outlier_thre)
                    {
                        ///继续判断是否满足地面点条件(点与grid最小高度之差小于阈值max_height_difference)
                        if (cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z < FrontEndConfig::max_height_difference)
                        {
                            /// groud point
                            //cloud_ground_full->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                            if (FrontEndConfig::estimate_ground_normal_method == 3)///默认使用这种方法 use ransac to estimate plane coeffs in a grid
                            {
                                grid_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                                last_point_is_ground = true;
//                                    EZLOG(INFO)<<"grid_ground_pcs.size() "<<grid_ground_pcs.size();
                            }
                            else
                            {
                                ///否则进行随机下采样,按下采样率将地面点加入grid_ground_pcs
                                if (j % ground_random_down_rate_temp == 0) // for example 10
                                {
                                    if (FrontEndConfig::estimate_ground_normal_method == 0)///directly use (0,0,1)
                                    {
                                        cloud_in->points[grid[i].point_id[j]].normal_x = 0.0;
                                        cloud_in->points[grid[i].point_id[j]].normal_y = 0.0;
                                        cloud_in->points[grid[i].point_id[j]].normal_z = 1.0;
                                    }
                                    grid_ground_pcs[i]->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                                    //cloud_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to ground points
                                }
//                                    EZLOG(INFO)<<"grid_ground_pcs.size() "<<grid_ground_pcs.size();
                            }

                        }
                        else /// inner grid unground points
                        {

                            if (j % nonground_random_down_rate_temp == 0 || cloud_in->points[grid[i].point_id[j]].intensity > intensity_thre) //extract more points on signs and vehicle license plate
                            {
                                cloud_in->points[grid[i].point_id[j]].data[3] = cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z; //data[3] stores the point height above ground
                                grid_unground_pcs[i]->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                                //cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
                            }
//                                EZLOG(INFO)<<"grid_unground_pcs.size() "<<grid_unground_pcs.size();
                        }
                    }//end if (cloud_in->points[grid[i].point_id[j]].z > grid[i].min_z_outlier_thre)
                }
            }
//                else //unground grid
//                {
//                    for (int j = 0; j < grid[i].point_id.size(); j++)
//                    {
//                        ///random downsample
//                        if (cloud_in->points[grid[i].point_id[j]].z > grid[i].min_z_outlier_thre &&
//                            (j % nonground_random_down_rate_temp == 0 || cloud_in->points[grid[i].point_id[j]].intensity > intensity_thre))
//                        {
//                            ///在data[3]通道记录非地面点的高度(相对本grid地面高度) save unground point in grid_unground_pcs
//                            cloud_in->points[grid[i].point_id[j]].data[3] = cloud_in->points[grid[i].point_id[j]].z - grid[i].neighbor_min_z; //data[3] stores the point height above ground
//                            grid_unground_pcs[i]->points.push_back(cloud_in->points[grid[i].point_id[j]]);
//                            //cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
//                        }
//                    }
//                }

//                EZLOG(INFO)<<"grid_unground_pcs.size() "<<grid_unground_pcs.size();
//                TicToc time_8;

            for (int j = 0; j < grid_ground->points.size(); j++)
            {
                grid_ground_pcs[i]->points.push_back(grid_ground->points[j]); //Add to ground points
            }

            //使用所有的点拟合平面，并更新地面点的法向量
            ///使用RANSAC算法对grid的地面点进行平面拟合,以获取地面法向量
//                if (estimate_ground_normal_method == 3 && grid_ground->points.size() >= min_grid_pt_num)
//                {
//                    ++estimate_normal_num;
////                    std::chrono::steady_clock::time_point tic_ransac = std::chrono::steady_clock::now();
//                    float normal_x, normal_y, normal_z;
//
//                    //RANSAC iteration number equation: p=1-(1-r^N)^M,
//                    //r is the inlier ratio (> 0.75 in our case), N is 3 in our case (3 points can fit a plane), to get a confidence > 0.99, we need about 20 iteration (M=20)
//                    /// use estimate_ground_normal_by_ransac函数进行RANSAC平面拟合,获取法向量坐标
//
//                    //pcl::PointCloud<PointXYZICOLRANGE>::Ptr grid_ground_tmp(new pcl::PointCloud<PointXYZICOLRANGE>);
//                    for (int j = 0; j < grid_ground->points.size(); j += 10)
//                    {
////                        if (j % ground_random_down_rate_temp == 0 && std::abs(normal_z) > 0.8) //53 deg
//
//
//                            grid_ground->points.push_back(grid_ground->points[j]);
//                            //cloud_ground->points.push_back(grid_ground->points[j]); //Add to ground points
//
//                    }
//
////                    EZLOG(INFO)<<"grid_ground->points.size() = "<<grid_ground->points.size();
////                    TicToc time_9;
//                    estimate_ground_normal_by_ransac(grid_ground, 0.3 * max_height_difference, 20, normal_x, normal_y, normal_z);
//                    ransac_time = ransac_time +time_9.toc();
////                    for (int j = 0; j < grid_ground->points.size(); j++)
////                    {
//////                        if (j % ground_random_down_rate_temp == 0 && std::abs(normal_z) > 0.8) //53 deg
////
////                        if (j % 10 == 0 && std::abs(normal_z) > 0.8) //53 deg
////                        {
////                            grid_ground->points[j].normal_x = normal_x;
////                            grid_ground->points[j].normal_y = normal_y;
////                            grid_ground->points[j].normal_z = normal_z;
////                            grid_ground_pcs[i]->points.push_back(grid_ground->points[j]); //Add to ground points
////                            //cloud_ground->points.push_back(grid_ground->points[j]); //Add to ground points
////                        }
////                    }
//                    for (int j = 0; j < grid_ground->points.size(); j++)
//                    {
////                        if (j % ground_random_down_rate_temp == 0 && std::abs(normal_z) > 0.8) //53 deg
//
//                        if (std::abs(normal_z) > 0.8) //53 deg
//                        {
//                            grid_ground->points[j].normal_x = normal_x;
//                            grid_ground->points[j].normal_y = normal_y;
//                            grid_ground->points[j].normal_z = normal_z;
//                            grid_ground_pcs[i]->points.push_back(grid_ground->points[j]); //Add to ground points
//                            //cloud_ground->points.push_back(grid_ground->points[j]); //Add to ground points
//                        }
//                    }
////                    std::chrono::steady_clock::time_point toc_ransac = std::chrono::steady_clock::now();
////                    std::chrono::duration<double> ground_ransac_time_per_grid = std::chrono::duration_cast<std::chrono::duration<double>>(toc_ransac - tic_ransac);
////                    consuming_time_ransac += ground_ransac_time_per_grid.count() * 1000.0; //unit: ms
////                    EZLOG(INFO)<<"consuming_time_ransac "<<consuming_time_ransac;
//                }// end if (estimate_ground_normal_method == 3 && grid_ground->points.size() >= min_grid_pt_num)

            pcl::PointCloud<PointXYZICOLRANGE>().swap(*grid_ground);
//                estimate_normal_time = estimate_normal_time + time_8.toc();
//                EZLOG(INFO)<<"time_8.toc() = "<<time_8.toc();
        }
    }//end for (int i = 0; i < num_grid; i++)]

    //combine the ground and unground points
    ///根据grid中的所有点更新最终的地面点和非地面点
    for (int i = 0; i < num_grid; i++)
    {
        cloud_ground->points.insert(cloud_ground->points.end(), grid_ground_pcs[i]->points.begin(), grid_ground_pcs[i]->points.end());
        cloud_unground->points.insert(cloud_unground->points.end(), grid_unground_pcs[i]->points.begin(), grid_unground_pcs[i]->points.end());
    }
    EZLOG(INFO)<<"cloud_ground->points.size() = "<<cloud_ground->points.size();
    EZLOG(INFO)<<"cloud_unground->points.size() = "<<cloud_unground->points.size();
//
    //free memory
    delete[] grid;

//        ///根据设置的estimate_ground_normal_method,对合并后的地面点cloud_ground计算法线
//        int normal_estimation_neighbor_k = 2 * min_grid_pt_num;
//        pcl::PointCloud<pcl::Normal>::Ptr ground_normal(new pcl::PointCloud<pcl::Normal>);
//        if (estimate_ground_normal_method == 1)
////            pca_estimator.get_normal_pcar(cloud_ground, normal_estimation_radius, ground_normal);
//            get_normal_pcar(cloud_ground, normal_estimation_radius, ground_normal);
//        else if (estimate_ground_normal_method == 2)
////            pca_estimator.get_normal_pcak(cloud_ground, normal_estimation_neighbor_k, ground_normal);
//            get_normal_pcak(cloud_ground, normal_estimation_neighbor_k, ground_normal);
//
//        //3.对地面点进行随机下采样,并对每个地面点赋值法线
//        for (int i = 0; i < cloud_ground->points.size(); i++)
//        {
//            if (estimate_ground_normal_method == 1 || estimate_ground_normal_method == 2)
//            {
//                cloud_ground->points[i].normal_x = ground_normal->points[i].normal_x;
//                cloud_ground->points[i].normal_y = ground_normal->points[i].normal_y;
//                cloud_ground->points[i].normal_z = ground_normal->points[i].normal_z;
//            }
//            if (!fixed_num_downsampling)
//            {
//                //LOG(INFO)<<cloud_ground->points[i].normal_x << "," << cloud_ground->points[i].normal_y << "," << cloud_ground->points[i].normal_z;
//                if (i % ground_random_down_down_rate == 0)
//                    cloud_ground_down->points.push_back(cloud_ground->points[i]);
//            }
//        }

//
////        if (fixed_num_downsampling)
////            random_downsample_pcl(cloud_ground, cloud_ground_down, down_ground_fixed_num);

    EZLOG(INFO)<<"finish ground_filter "<<endl;
//
} // end function fast_ground_filter

void check_normal(pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    //It is advisable to check the normals before the call to compute()
    for (int i = 0; i < normals->points.size(); i++)
    {
        if (!pcl::isFinite<pcl::Normal>(normals->points[i]))
        {
            normals->points[i].normal_x = 0.577; // 1/ sqrt(3)
            normals->points[i].normal_y = 0.577;
            normals->points[i].normal_z = 0.577;
            //normals->points[i].curvature = 0.0;
        }
    }
}

bool get_normal_pcar(pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                     float radius,
                     pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    // Create the normal estimation class, and pass the input dataset to it;
//        pcl::NormalEstimation<PointXYZICOLRANGE, pcl::Normal> ne;
//        ne.setInputCloud(in_cloud);
//        // Create an empty kd-tree representation, and pass it to the normal estimation object;
//        pcl::search::KdTree<PointXYZICOLRANGE>::Ptr tree(new pcl::search::KdTree<PointXYZICOLRANGE>());
//
//        ne.setSearchMethod(tree);
//        // Use all neighbors in a sphere of radius;
//        ne.setRadiusSearch(radius);
//        // Compute the normal
//        ne.compute(*normals);
//        check_normal(normals);
    return true;
}

bool get_normal_pcak(pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                     int K,
                     pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    // Create the normal estimation class, and pass the input dataset to it;
//        pcl::NormalEstimation<PointXYZICOLRANGE, pcl::Normal> ne;
////        ne.setNumberOfThreads(2); //More threads sometimes would not speed up the procedure
//        ne.setInputCloud(in_cloud);
//        // Create an empty kd-tree representation, and pass it to the normal estimation object;
//        pcl::search::KdTree<PointXYZICOLRANGE>::Ptr tree(new pcl::search::KdTree<PointXYZICOLRANGE>());
//        ne.setSearchMethod(tree);
//        // Use all neighbors in a sphere of radius;
//        ne.setKSearch(K);
//        // Compute the normal
//        ne.compute(*normals);
//        check_normal(normals);
    return true;
}

bool plane_seg_ransac(const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud,
                      float threshold, int max_iter,
                      pcl::PointCloud<PointXYZICOLRANGE>::Ptr &planecloud,
                      pcl::ModelCoefficients::Ptr &coefficients) //Ransac
{
//        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//
//        // Create the segmentation object
//        pcl::SACSegmentation<PointXYZICOLRANGE> sacseg;//**********************************
//
//        // Optional
//        sacseg.setOptimizeCoefficients(true);
//
//        // Mandatory
//        sacseg.setModelType(pcl::SACMODEL_PLANE);
//        sacseg.setMethodType(pcl::SAC_RANSAC);
//        sacseg.setDistanceThreshold(threshold);
//        sacseg.setMaxIterations(max_iter);
//
//        sacseg.setInputCloud(cloud);
//        sacseg.segment(*inliers, *coefficients);
//
//        if (inliers->indices.size() == 0)
//        {
//            PCL_ERROR("Could not estimate a planar model for the given dataset.");
//        }
//
//        /*cout << "Model coefficients: " << coefficients->values[0] << " "
//        << coefficients->values[1] << " "
//        << coefficients->values[2] << " "
//        << coefficients->values[3] << std::endl;*/
//
//        //LOG(INFO) << "Model inliers number: " << inliers->indices.size() << std::endl;
//
//        for (size_t i = 0; i < inliers->indices.size(); ++i)
//        {
//            planecloud->push_back(cloud->points[inliers->indices[i]]);
//        }
    return 1;
}

bool estimate_ground_normal_by_ransac(pcl::PointCloud<PointXYZICOLRANGE>::Ptr &grid_ground,
                                      float dist_thre, int max_iter, float &nx, float &ny, float &nz)
{

    pcl::PointCloud<PointXYZICOLRANGE>::Ptr grid_ground_fit(new pcl::PointCloud<PointXYZICOLRANGE>);
    pcl::ModelCoefficients::Ptr grid_coeff(new pcl::ModelCoefficients);
    plane_seg_ransac(grid_ground, dist_thre, max_iter, grid_ground_fit, grid_coeff);

    grid_ground.swap(grid_ground_fit);
    nx = grid_coeff->values[0];
    ny = grid_coeff->values[1];
    nz = grid_coeff->values[2];

    //LOG(INFO) << nx << "," << ny << "," << nz;
    return 1;
}




#endif //SEU_LIDARLOC_FRONT_END_H
