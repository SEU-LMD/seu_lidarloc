//#include "erasor/OfflineMapUpdater.h"
#include "erasor/erasor.h"
#include "config_helper.h"
//#include "erasor/utility.h"
#include <pcl/pcl_base.h>
#include <pcl/console/print.h>
#include <pcl/impl/pcl_base.hpp>
#define NUM_PTS_LARGE_ENOUGH 200000
#define NUM_PTS_LARGE_ENOUGH_FOR_MAP 20000000
using namespace erasor;
namespace erasor {
    class OfflineMapUpdater {
    private:
        /**< Parameters of MapUpdater from the rosparam */
        double query_voxel_size_;//query指的是当前帧
        double map_voxel_size_;
        /**< ERASOR does not conduct dynamic object removal at all time steps!
         * removal_interval needs some heursitics */
        int    removal_interval_;
        int    global_voxelization_period_;
        bool   verbose_;
        bool   is_large_scale_;
        bool   is_submap_not_initialized_ = true;

        /**< Params. of Volume of Interest (VoI) */
        //double max_range_;
        double min_h_;
        double max_h_;
        double submap_size_;
        double submap_center_x_;
        double submap_center_y_;

        /**< ERASOR Version
         * v2: Naive
         * v3: More conservative way*/
        int erasor_version_;

        std::string data_name_;
        std::string map_name_;
        std::string environment_;
        std::string save_path_;

        unique_ptr<ERASOR> erasor_;

        /**< ------------------------------------------ */
        int num_pcs_init_;

        ros::NodeHandle nh;

        ros::Subscriber sub_node_;
        ros::Subscriber sub_flag_;

        ros::Publisher pub_path_;

        ros::Publisher pub_map_init_;
        ros::Publisher pub_static_arranged_, pub_dynamic_arranged_;
        ros::Publisher pub_map_rejected_;
        ros::Publisher pub_curr_rejected_;

        ros::Publisher pub_debug_map_arranged_init_;
        ros::Publisher pub_debug_query_egocentric_;
        ros::Publisher pub_debug_map_egocentric_;

        ros::Publisher pub_debug_pc2_curr_;
        ros::Publisher pub_debug_map_;

        /***
         * Variables ralted with global map
         */
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_init_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_init_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_global_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_complement_;
//        pcl::PointCloud<pcl::PointXYZI>::Ptr map_ceilings_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudRaw;
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudRawDS;
        pcl::VoxelGrid<pcl::PointXYZI>:: Ptr downSizeFilterRaw;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurroundingKeyPoses;
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr local_kdtree;
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_map;
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_mapDS;


        /*** Inputs of ERASOR */
        pcl::PointCloud<pcl::PointXYZI>::Ptr query_voi_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi_wrt_origin_; // w.r.t origin, i.e. map frame
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_outskirts_;

        /*** Outputs of ERASOR
         * map_filtered_ = map_static_estimate + map_egocentric_complement
         */
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_static_estimate_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_egocentric_complement_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr query_rejected_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_rejected_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr total_query_rejected_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr total_map_rejected_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_objs_to_viz_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_objs_to_viz_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr FramesArray[40];
        pcl::PointCloud<pcl::PointXYZI>::Ptr submap;

        // Published msgs
        sensor_msgs::PointCloud2 pc2_map_;
        nav_msgs::Path           path_;

        Eigen::Matrix4f tf_lidar2body_; /**< Transformation matrix between the initial pose to the origin */
        Eigen::Matrix4f tf_body2origin_;
        geometry_msgs::Pose pose_curr;

        OfflineMapUpdater(){
            //接收数据集以及各种内部定义的话题
            sub_node_ = nh.subscribe<erasor::node>("/node/combined/optimized", 2000, &OfflineMapUpdater::callback_node, this);
            sub_flag_ = nh.subscribe<std_msgs::Float32>("/saveflag", 10, &OfflineMapUpdater::callback_flag, this);
//主要运行话题
            pub_path_ = nh.advertise<nav_msgs::Path>("/MapUpdater/path_corrected", 100);

           // pub_map_init_      = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_init", 100);
            pub_map_rejected_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_rejected", 100);
            pub_curr_rejected_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/curr_rejected", 100);
//主要运行话题
            pub_debug_pc2_curr_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/pc2_curr", 100);

            pub_debug_map_               = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map", 100);
            pub_debug_query_egocentric_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/pc_curr_body", 100);
            pub_debug_map_egocentric_    = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_body", 100);
            pub_debug_map_arranged_init_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_init_arranged", 100);
//
            // pub_dynamic_arranged_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/dynamic", 100);
            // pub_static_arranged_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/static", 100);

            initialize_ptrs();

            set_params();

            //load_global_map();

            erasor_.reset(new ERASOR(&nh));
        };

        void set_params() {

//    nh = ros::NodeHandle("~");
//    // OfflineMapUpdater Parameters
//    nh.param("/MapUpdater/query_voxel_size", query_voxel_size_, 0.05);
//    nh.param("/MapUpdater/map_voxel_size", map_voxel_size_, 0.05);
//    nh.param("/erasor/version", erasor_version_, 3);
//
//    nh.param("/verbose", verbose_, true);
            std::cout << "Loading " << map_name_ << endl;
            std::cout << "Target env: " << environment_ << std::endl;
            std::cout << "\033[1;32m Version: \033[0m: " << erasor_version_ << std::endl;
            //   std::vector<double> lidar2body;
            // if (nh.getParam("/tf/lidar2body", ErasorConfig::lidar2body)) {
            if (ErasorConfig::lidar2body.size() == 7)
            {
                geometry_msgs::Pose tmp_pose;
                tmp_pose.position.x    = ErasorConfig::lidar2body[0];
                tmp_pose.position.y    = ErasorConfig::lidar2body[1];
                tmp_pose.position.z    = ErasorConfig::lidar2body[2];
                tmp_pose.orientation.x = ErasorConfig::lidar2body[3];
                tmp_pose.orientation.y = ErasorConfig::lidar2body[4];
                tmp_pose.orientation.z = ErasorConfig::lidar2body[5];
                tmp_pose.orientation.w = ErasorConfig::lidar2body[6];
                Eigen::Matrix4f tmp_tf = Eigen::Matrix4f::Identity();
                tf_lidar2body_ = erasor_utils::geoPose2eigen(tmp_pose) * tmp_tf;
                std::cout << tf_lidar2body_ << std::endl;
                tmp_pose = erasor_utils::eigen2geoPose(tf_lidar2body_);
            }
            // }
        }; /**< Set parameters loaded from launch file */

        void initialize_ptrs() {
            map_init_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_arranged_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_arranged_init_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_arranged_complement_.reset(new pcl::PointCloud<pcl::PointXYZI>());
//    map_ceilings_.reset(new pcl::PointCloud<pcl::PointXYZI>());

            query_voi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_voi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_voi_wrt_origin_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_outskirts_.reset(new pcl::PointCloud<pcl::PointXYZI>());

            map_static_estimate_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_egocentric_complement_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_filtered_.reset(new pcl::PointCloud<pcl::PointXYZI>());

            query_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            total_query_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            total_map_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());

            dynamic_objs_to_viz_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            static_objs_to_viz_.reset(new pcl::PointCloud<pcl::PointXYZI>());

        }

        void load_global_map(){};

//下采样后保存静态地图
/***
 * 改成保存每一帧角点和面点地图 comment by wxy
 * @param voxel_size
 */

      //该函数实现
      //frame_id有必要吗？
      //pose有必要更新吗？


        void updatePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr newFrame, int end_idx){

            if (is_submap_not_initialized_){
                int maxFrames = 40;
                for (int end_idx = 0; end_idx < maxFrames; ++end_idx){
                    FramesArray[end_idx] =newFrame;
                }
                is_submap_not_initialized_ = false;
            }else{
                FramesArray[(end_idx+1) % 40 ] = newFrame;
            };
        }

//
//        void addPointCloudFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr newFrame,int frame_id,
//                            pcl::PointCloud<pcl::PointXYZI>::Ptr localmap) {
//            int maxFrames = 50;
//            if (is_submap_not_initialized_){
//
//               for (int i = 0; i < maxFrames; ++i){
//                 *localmap += *newFrame;
//               }
//               is_submap_not_initialized_ = false;
//            }else{
//            // 如果窗口大小超过最大帧数，弹出最早的一帧点云
//               if (localmap->size() <= maxFrames) {
//                  localmap->push_back(newFrame->points[frame_id]);
//               }else{
//                  localmap->erase(localmap->begin());
//              }
//           }
//
//       }

void callback_node(const erasor::node::ConstPtr &msg) {

    signal(SIGINT, erasor_utils::signal_callback_handler);

    static int stack_count = 0;
    stack_count++;
   //堆栈次数除余为0，即为删除间隔的整数倍
    if (stack_count % removal_interval_ == 0) {
        if (verbose_) ROS_INFO_STREAM("\033[01;32m" << msg->header.seq << "th frame\033[0m is comming");

        if (!is_large_scale_) {
            pub_debug_map_arranged_init_.publish(erasor_utils::cloud2msg(*map_arranged_init_));
        }
        /***
         * Assumption: Coming nodes are equal to those that are used to build initial map
         * And there exists a premise that node is already somehow optimized
         */
        tf_body2origin_ = erasor_utils::geoPose2eigen(msg->odom);
        //坐标系转换结果的地图路径给path_
        set_path(path_, "corrected", *msg, tf_body2origin_);

        /**<
         * 1. Set query pointcloud -> query VOI
         *    Note that query is on lidar frame.
         *    So the coordinate is tranformed from lidar coord. to body coord. (in fact, it depends on your own env.)
         *    读取当前帧VOI, comment by wxy
         * */
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_voxel(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_body(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_viz(new pcl::PointCloud<pcl::PointXYZI>);

        //开辟空间
        ptr_query->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_voxel->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_body->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_viz->reserve(NUM_PTS_LARGE_ENOUGH);


         //msg变成cloud
        pcl::fromROSMsg(msg->lidar, *ptr_query);

        //将输入的点云进行降采样
        erasor_utils::voxelize_preserving_labels(ptr_query, *ptr_query_voxel, query_voxel_size_);
        //将lidar系下的点云转换到车体系
        pcl::transformPointCloud(*ptr_query_voxel, *ptr_query_body, tf_lidar2body_);

        *query_voi_ = *ptr_query_body;
        //共享指针指向载体数据转变成original点云位置
        body2origin(*ptr_query_body, *ptr_query_viz);
        // - - - - - - - - - - - - - - - - - - - -

        /**< 2. Set map pointcloud -> map VOI
         *  1.把每一帧地图加载进来，构建局部地图
         *  2.设置map的VOI
         * */
        //定义指针
        //(new pcl::PointCloud<pcl::PointXYZI>);
       // pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudRawDS(new pcl::PointCloud<pcl::PointXYZI>);
       // pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr local_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        //pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr surroundingKeyPoses(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<pcl::PointXYZI>());

        map<int, pcl::PointCloud<pcl::PointXYZI>>laserCloudMapContainer;
        vector <pcl::PointCloud<pcl::PointXYZI>::Ptr> cornerCloudKeyFrames;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        erasor::node node = *msg;

        //开辟空间
        laserCloudRaw->reserve(NUM_PTS_LARGE_ENOUGH);
        laserCloudRawDS->reserve(NUM_PTS_LARGE_ENOUGH);
        //将点云从msg变到Cloud;
          pcl::fromROSMsg(msg->rawData, *laserCloudRaw);

        //PoseT pose_world = erasor_utils::geoPose2eigen(msg->odom);
        //取出点云的位姿
        Eigen::Matrix4f pose_world = erasor_utils::geoPose2eigen(msg->odom);
//        Eigen::Vector3f t_world_cur;
//        Eigen::Quaternionf  q_world_cur;
//        t_world_cur[0] = msg->odom.position.x;
//        t_world_cur[1] = msg->odom.position.y;
//        t_world_cur[2] = msg->odom.position.z;
//        q_world_cur.x() = msg->odom.orientation.x;
//        q_world_cur.y() = msg->odom.orientation.y;
//        q_world_cur.z() = msg->odom.orientation.z;
//        q_world_cur.w() = msg->odom.orientation.w;
      //取出帧数的索引
        //  int frame_id = msg->header.seq;
        //  const int maxFrames = 100;
        //取出里程计的位姿x y
       // double x_curr = tf_body2origin_(0, 3);
      //  double y_curr = tf_body2origin_(1, 3);

        //构建局部地图
        for (int i = 0; i<40;++i){
            *submap = *submap + *FramesArray[i];
        }

        //生成局部地图
//        const float rawMapFilterSize = 0.5;
//        downSizeFilterRaw->setLeafSize(rawMapFilterSize, rawMapFilterSize,
//                                    rawMapFilterSize);
//        downSizeFilterRaw->setInputCloud(local_map);
//        downSizeFilterRaw->filter(*local_mapDS);


        auto start_voi = ros::Time::now().toSec();
        //分配成VOI，根据模式设置数据存储格式，返回指向Voi的指针
        //得到了在子地图中的VOI
//        fetch_VoI(x_curr, y_curr, *map_voi_, *map_outskirts_);
        *map_voi_ = *local_mapDS;

       // fetch_VoI(x_curr, y_curr, *map_voi_, *map_outskirts_,"naive");
        auto end_voi = ros::Time::now().toSec();

        ROS_INFO_STREAM("\033[1;32m" << setw(22) << "Extracting VoI takes " << end_voi - start_voi << "s\033[0m");
        //发布全局Voi与当前帧Voi
        pub_debug_map_egocentric_.publish(erasor_utils::cloud2msg(*map_voi_));
        pub_debug_query_egocentric_.publish(erasor_utils::cloud2msg(*query_voi_));

        /**< 3. Conduct Scan Ratio Test & set map_static_estimate and map_egocentric_complement
         * Note that inputs should be previously transformed into egocentric frame */
        auto start = ros::Time::now().toSec();
       //将当前帧和map中的VOI分成Bin,并计算map中有多少个bin;
        erasor_->set_inputs(*map_voi_, *query_voi_);
        if (erasor_version_ == 2) {
            //判断是否是动态点，静态点放入selected.动态点放入rejected;
            erasor_->compare_vois_and_revert_ground(msg->header.seq);
            //将被占据的点提取出来并可视化，发布被占据点的话题，以及bin范围外的点云消息话题
            erasor_->get_static_estimate(*map_static_estimate_, *map_egocentric_complement_);
        } else if (erasor_version_ == 3) {
            erasor_->compare_vois_and_revert_ground_w_block(msg->header.seq);
            erasor_->get_static_estimate(*map_static_estimate_, *map_egocentric_complement_);
        } else {
            throw invalid_argument("Other version is not implemented!");
        }

        /***
         * 将标记为角点和面点的点云分别拿出来后，在角点和面点地图中将标记为rejected的点剔除；
         */

        auto middle = ros::Time::now().toSec();

        ROS_INFO_STREAM("\033[1;32m" << setw(22) << "ERASOR takes " << middle - start << "s\033[0m");
        //Bin中的点和范围之外的点

        *map_filtered_ = *map_static_estimate_ + *map_egocentric_complement_;

        /*** Get currently rejected pts */
        erasor_->get_outliers(*map_rejected_, *query_rejected_);

        body2origin(*map_filtered_, *map_filtered_);
        body2origin(*map_rejected_, *map_rejected_);
        body2origin(*query_rejected_, *query_rejected_);

        *local_mapDS = *map_filtered_ + *map_outskirts_;

        auto end = ros::Time::now().toSec();
        //根据标签将map_arranged_点云分为动态静态
        // erasor_utils::parse_dynamic_obj(*map_arranged_, *dynamic_objs_to_viz_, *static_objs_to_viz_);

        /*** Just for debugging */
        //相当于只过滤被rejected的点
        *total_map_rejected_ += *map_rejected_;
        *total_query_rejected_ += *query_rejected_;

        /***
         * 特征提取 comment by wxy
         */

        if (environment_ != "outdoor") { throw invalid_argument("Other modes are not supported"); }

        print_status();
//相当于用话题发布器将前面的消息变成话题发布
        publish(*ptr_query_viz, pub_debug_pc2_curr_);
        // publish(*static_objs_to_viz_, pub_static_arranged_);

        // publish(*dynamic_objs_to_viz_, pub_dynamic_arranged_);
        publish(*map_rejected_, pub_map_rejected_);
        publish(*query_rejected_, pub_curr_rejected_);
//
//        if (!is_large_scale_) {
//            publish(*map_init_, pub_map_init_);
//        }
//发布路径话题，载体坐标系转换成世界坐标系map
                pub_path_.publish(path_);
            } else {
                ROS_INFO_STREAM("\033[1;32m PASS! \033[0m");
            }
        }


        void save_static_map(float voxel_size) {
            // 1. Voxelization
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
            ptr_src->reserve(num_pcs_init_);

            if (is_large_scale_) {
                std::cout << "Merging submap and complements..." << std::endl;
                *ptr_src = *map_arranged_ + *map_arranged_complement_;
            } else {
                *ptr_src = *map_arranged_;
            }
            pcl::PointCloud<pcl::PointXYZI> map_to_be_saved;
            erasor_utils::voxelize_preserving_labels(ptr_src, map_to_be_saved, voxel_size); // 0.05m is the criteria!
            // 2. Save the cloudmap
            map_to_be_saved.width  = map_to_be_saved.points.size();
            map_to_be_saved.height = 1;

            std::cout << "\033[1;32mTARGET: " << save_path_ + "/" + data_name_ + "_result.pcd" << "\033[0m" << std::endl;
            std::cout << "Voxelization operated with " << voxel_size << " voxel size" << std::endl;
            pcl::io::savePCDFileASCII(save_path_ + "/" + data_name_ + "_result.pcd", map_to_be_saved);
            std::cout << "\033[1;32mComplete to save the final static map\033[0m" << std::endl;

        }


        /**< Flag is used when saving result pointcloud into pcd file */
        void callback_flag(const std_msgs::Float32::ConstPtr &msg) {
            std::cout << "Flag comes!" << std::endl;
            save_static_map(msg->data);
        }

        void body2origin(
                const pcl::PointCloud<pcl::PointXYZI> src,
                pcl::PointCloud<pcl::PointXYZI> &dst) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
            *ptr_src = src;
            pcl::transformPointCloud(*ptr_src, *ptr_transformed, tf_body2origin_);
            dst = *ptr_transformed;
        }

//        void set_submap(
//                const pcl::PointCloud<pcl::PointXYZI> &map_global, pcl::PointCloud<pcl::PointXYZI>& submap,
//                pcl::PointCloud<pcl::PointXYZI>& submap_complement,
//                double x, double y, double submap_size) {
//
//            submap.clear();
//            submap.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
//            submap_complement.clear();
//            submap_complement.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
//
//            for (const auto pt: map_global.points) {
//                double diff_x = fabs(x - pt.x);
//                double diff_y = fabs(y - pt.y);
//                if ((diff_x < submap_size) && (diff_y < submap_size)) {
//                    submap.points.emplace_back(pt);
//                } else {
//                    submap_complement.points.emplace_back(pt);
//                }
//            }
//        }


        void fetch_VoI(
                double x_criterion, double y_criterion, pcl::PointCloud<pcl::PointXYZI> &dst,
                pcl::PointCloud<pcl::PointXYZI> &outskirts, std::string mode) {
            // 1. Divide map_arranged into map_central and map_outskirts
            //1.把输入的地图划分成地图中心和地图边缘；
            static double margin = 0;
            if (!dst.empty()) dst.clear();
            if (!outskirts.empty()) outskirts.clear();
            if (!map_voi_wrt_origin_->points.empty()) map_voi_wrt_origin_->points.clear(); // Inliers are still on the map frame
            /***
             *   将点根据距离分成边缘点和中心点,两种模式，
             *   1.naive:60米内是中心点，其余是边缘点
             *   2.kdtree:最近邻搜索60.5米内的点云，通过isTrue判断是否是边缘点还是中心点。
             */

                if (mode == "naive") {
                   double max_dist_square = pow(ErasorConfig::max_range + margin, 2);

                       for (auto const &pt: (*local_mapDS).points) {
                            double dist_square = pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
                            if (dist_square < max_dist_square) {
                                map_voi_wrt_origin_->points.emplace_back(pt);
                            } else {
                                outskirts.points.emplace_back(pt);
                            }
                     }
                } else if (mode == "kdtree") {
                    pcl::PointXYZI searchPoint;
                    searchPoint.x = x_criterion;
                    searchPoint.y = y_criterion;
                    searchPoint.z = 0.5;
                    std::cout << "\033[1;32mKDTREE mode " << (*local_mapDS).points.size() << "\033[0m" << std::endl;
                    std::vector<int> pointIdxRadiusSearch;
                    std::vector<float> pointRadiusSquaredDistance;
                    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
                    //为什么要深拷贝一次？
                   *cloud = *local_mapDS;
                    kdtree.setInputCloud(cloud);

                    if (kdtree.radiusSearch(searchPoint, ErasorConfig::max_range + 0.5, pointIdxRadiusSearch,
                                        pointRadiusSquaredDistance) > 0) {
                    // To get outlier
                         std::vector<char> isTrue(local_mapDS->points.size(), false);
                         std::cout << "what?? " << pointIdxRadiusSearch.size();
                         std::cout << "    " << isTrue.size();
                         for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                            auto pt = (*cloud)[pointIdxRadiusSearch[i]];
                            map_voi_wrt_origin_->points.emplace_back(pt);
                            isTrue[pointIdxRadiusSearch[i]] = true;
                        }
                        for (size_t j = 0; j < local_mapDS->points.size(); ++j) {
                            if (!isTrue[j]) {
                                outskirts.push_back(local_mapDS->points[j]);
                            }
                        }
                   }
                }
            ROS_INFO_STREAM(
                    local_mapDS->points.size() << "=" << map_voi_wrt_origin_->points.size() + outskirts.points.size()
                                               << "| "
                                               << map_voi_wrt_origin_->points.size() << " + \033[4;32m"
                                               << outskirts.points.size()
                                               << "\033[0m");

            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*map_voi_wrt_origin_, *ptr_transformed, tf_body2origin_.inverse());
            //点云到本体坐标系了，现在的dst变成了在本体坐标系的中心点云

            dst = *ptr_transformed;
        }

        void print_status() {
            ROS_INFO_STREAM("ERASOR Input: \033[1;33m" << map_voi_->points.size() << "\033[0m = ");
            ROS_INFO_STREAM(map_static_estimate_->points.size() << " + " << map_egocentric_complement_->points.size() << " - "
                                                                << map_rejected_->points.size());
            ROS_INFO_STREAM(" = \033[1;33m" << map_static_estimate_->points.size() + map_egocentric_complement_->points.size() -
                                               map_rejected_->points.size() << "\033[0m");
            ROS_INFO_STREAM(map_arranged_->points.size() << " " << map_filtered_->points.size() << " \033[4;32m"
                                                         << map_outskirts_->points.size() << "\033[0m");

            std::cout << "[Debug] Total: " << map_arranged_->points.size() << std::endl;
            // std::cout << "[Debug] " << (double) dynamic_objs_to_viz_->points.size() / num_pcs_init_ * 100 << setw(7) << " % <- "
           //           << dynamic_objs_to_viz_->points.size() << " / " << num_pcs_init_ << std::endl;
           // std::cout << "[Debug] " << (double) static_objs_to_viz_->points.size() / num_pcs_init_ * 100 << setw(7) << "% <- "
           //           << static_objs_to_viz_->points.size() << " / " << num_pcs_init_ << std::endl;
        }

        void set_path(nav_msgs::Path &path, std::string mode,const erasor::node &node, const Eigen::Matrix4f &body2mapprev) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header          = node.header;
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose            = erasor_utils::eigen2geoPose(body2mapprev);

            path.header = pose_stamped.header;
            path.poses.push_back(pose_stamped);
       }

        void publish(const pcl::PointCloud<pcl::PointXYZI> &map,const ros::Publisher &publisher) {
            pcl::toROSMsg(map, pc2_map_);
            pc2_map_.header.frame_id = "map";
            publisher.publish(pc2_map_);
        }


    };

}

//void OfflineMapUpdater::load_global_map() {
//
//    /***
//     * map_init         : Raw initial map
//     * map_arranged_init: Initial map to be compared with map arranged
//     *                    In case of indoor env, map_arranged_init = map_init - ceilings
//     *                    In case of outdoor env, map_arranged_init = map_init
//     * map_arranged     : Target cloud to be filtered via ERASOR
//     */
//
//    cout<<"[MapUpdater] On loading naively accumulated map...it takes few seconds..."<<endl;
//    map_init_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
//    map_arranged_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
//    map_arranged_init_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
//    map_arranged_global_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
//   //把原始点云地图加载进来
//
//    int failure_flag = erasor_utils::load_raw_pcd(map_name_, map_init_);
//
//    if (failure_flag == -1) {
//        throw invalid_argument("Maybe intiial map path is not correct!");
//    } else {
//        std::cout << "Loading global map complete!" << std::endl;
//    }
//
//    num_pcs_init_ = map_init_->points.size();
//
//    if (environment_ == "outdoor") {
//        //子映射
//        *map_arranged_      = *map_init_;
//        *map_arranged_init_ = *map_arranged_;
//        if (is_large_scale_) {
//            /*** In case of large-scale static map building,
//             * `map_arranged_` is used as submap
//             * Thereafter, `map_arranged_` is updated to the `map_arranged_global_`
//             */
//            *map_arranged_global_ = *map_arranged_;
//            std::cout << "Large-scale mode is on!" << std::endl;
//            std::cout << "Submap size is: " << submap_size_ << std::endl;
//        }
//
//    } else if (environment_ == "indoor") {
////        // Erase the rooftop in case of indoor environments
////        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_dst(new pcl::PointCloud<pcl::PointXYZI>);
////        // ToDo It may not work on the large-scale map or env which includes slope regions
////        throw invalid_argument("This `indoor` mode is not perfect!");
////        pcl::PassThrough<pcl::PointXYZI> ptfilter;
////        ptfilter.setInputCloud(map_init_);
////        ptfilter.setFilterFieldName("z");
////        ptfilter.setFilterLimits(min_h_, max_h_);
////        ptfilter.filter(*ptr_dst);
////
////        *map_arranged_init_ = *map_init_;
////        *map_arranged_      = *map_arranged_init_;
////
////        ptfilter.setFilterLimitsNegative(true);
////        ptfilter.filter(*ptr_dst);
////        *map_ceilings_ = *ptr_dst;
//    }
//    if (!is_large_scale_) {
//        // In case of large scale, it takes too time...
//        pub_map_init_.publish(erasor_utils::cloud2msg(*map_arranged_init_));
//    }
//}


/***
 * 1.对点云进行体素滤波
 * 2.存入点云
 * @param voxel_size
 */
/***
 * 对局部地图进行初始化，以当前帧的位姿为中心点，局部地图包括周围submap_size大小，为200米；
 * 当激光所对应的位姿距离中心点超过100时，则重新建立新的局部地图；
 * map_arranged_对应当前帧对应的子地图里的点；complement对应全局地图里除子地图之外的点
 */
void reassign_submap(double pose_x, double pose_y){
//    if (is_submap_not_initialized_) {
//        set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
//        submap_center_x_ = pose_x;
//        submap_center_y_ = pose_y;
//        is_submap_not_initialized_ = false;
//
//        ROS_INFO_STREAM("\033[1;32mComplete to initialize submap!\033[0m");
//        ROS_INFO_STREAM(map_arranged_global_->points.size() <<" to " << map_arranged_->points.size() <<" | " <<map_arranged_complement_->points.size());
//    } else {
//        double diff_x = abs(submap_center_x_ - pose_x);
//        double diff_y = abs(submap_center_y_ - pose_y);
//        static double half_size = submap_size_ / 2.0;
//        if ( (diff_x > half_size) ||  (diff_y > half_size) ) {
//            // Reassign submap
//            map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());
//            map_arranged_global_->reserve(num_pcs_init_);
//            *map_arranged_global_ = *map_arranged_ + *map_arranged_complement_;
//
//            set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
//            submap_center_x_ = pose_x;
//            submap_center_y_ = pose_y;
//            ROS_INFO_STREAM("\033[1;32mComplete to initialize submap!\033[0m");
//            ROS_INFO_STREAM(map_arranged_global_->points.size() <<" to " << map_arranged_->points.size() <<" | " <<map_arranged_complement_->points.size());
//        }
//    }
}
//选择子地图之内的点存入submap.points，之外存入submap_complement.points
//void OfflineMapUpdater::set_submap
//分配VOI，设置数据存储模式，返回VOI数据存储位置

//void OfflineMapUpdater::publish(
//        const sensor_msgs::PointCloud2 &map,
//        const ros::Publisher &publisher) {
//    pc2_map_.header.frame_id = "map";
//    publisher.publish(map);
//    if (verbose_) ROS_INFO_STREAM("PC2 is Published!");
//}


