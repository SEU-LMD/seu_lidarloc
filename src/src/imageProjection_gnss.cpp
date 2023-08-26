#include "utility.h"
#include "lio_sam_6axis/cloud_info.h"

#include "ivsensorgps.h"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

//定义点云类型：VelodynePointXYZIRT,PandarPointXYZIRT,OusterPointXYZIRT
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    uint8_t intensity;
    uint16_t ring;
    double latency;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
                                           (uint16_t, ring, ring)(double, latency, latency)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

static GeographicLib::LocalCartesian geoConverter;
class ImageProjection  {  //定义ImageProjection类,继承ParamServer

public:
    //Eigen::Matrix3d extrinc_rot;
private:

    std::mutex imuLock;
    std::mutex odoLock;
//    std::mutex gnssLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubFullCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    std::deque<nav_msgs::Odometry> gnssQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
//    int gnssPointerCur; //指向当前gnss数据
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;


    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam_6axis::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;
    Eigen::Matrix3d extrinc_rotation;
    Eigen::Vector3d extrinc_translation;
    Eigen::Isometry3d T_b_n;

public:
    ros::NodeHandle nh;

    ImageProjection() :deskewFlag(0) {
        subImu = nh.subscribe<gps_imu::ivsensorgps>(Config::imuTopic,
                                                    2000,
                                                    &ImageProjection::gnssHandler,
                                                    this,
                                                    ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(Config::pointCloudTopic,
                                                               5,
                                                               &ImageProjection::cloudHandler,
                                                               this,
                                                               ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_6axis/deskew/cloud_deskewed", 1);
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_6axis/deskew/cloud_full_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam_6axis::cloud_info>("lio_sam_6axis/deskew/cloud_info", 1);

        allocateMemory();//内存分配
        resetParameters();//变量重置

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR); //设置PCL的打印日志级别为错误

        Eigen::Matrix3d extrinc_rot;
        extrinc_rot <<  0.000646759294, 0.999921769,    0.0124914668,
                        0.999989044,    0.000588790089, 0.00464382452,
                        0.00463610638,  0.0124943334,   0.999911195;
        extrinc_rotation = extrinc_rot.inverse();
        Eigen::Vector3d extrinc_translation(-0.33555608, -0.22477218, 0.38088802);

        Eigen::Isometry3d T_b_n = Eigen::Isometry3d::Identity();

        T_b_n.translation() = extrinc_translation;
        T_b_n.linear() = extrinc_rot;


    }

    void allocateMemory() {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(Config::N_SCAN * Config::Horizon_SCAN);//将fullCloud的点集大小初始化为N_SCAN * Horizon_SCAN

        cloudInfo.startRingIndex.assign(Config::N_SCAN, 0);//为cloudInfo中的startRingIndex、endRingIndex向量分配大小N_SCAN,并初始化为0
        cloudInfo.endRingIndex.assign(Config::N_SCAN, 0);

        cloudInfo.pointColInd.assign(Config::N_SCAN * Config::Horizon_SCAN, 0);//为pointColInd和pointRange向量分配N_SCAN * Horizon_SCAN大小,并初始化为0
        cloudInfo.pointRange.assign(Config::N_SCAN * Config::Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters() {  //重置变量，每次处理完一帧点云后调用,目的是为下一次点云的处理重新初始化相关变量,避免使用到上一帧数据的状态
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(Config::N_SCAN, Config::Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i) {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(Config::N_SCAN, 0);
    }

    ~ImageProjection() {}

    int init = false;
    //static GeographicLib::LocalCartesian geoConverter;


    void gnssHandler(const gps_imu::ivsensorgpsConstPtr &gnss_msg) { //接收GNSS数据，存入队列

//        Eigen::Matrix3d extrinc_rot;
//        extrinc_rot <<  0.000646759294, 0.999921769,    0.0124914668,
//                        0.999989044,    0.000588790089, 0.00464382452,
//                        0.00463610638,  0.0124943334,   0.999911195;
//        Eigen::Matrix3d extrinc_rotation = extrinc_rot.inverse();
//        Eigen::Vector3d extrinc_translation(-0.33555608, -0.22477218, 0.38088802);
//
//        Eigen::Isometry3d T_b_n = Eigen::Isometry3d::Identity();
//
//        T_b_n.translation() = extrinc_translation;
//        T_b_n.linear() = extrinc_rot;
        //static GeographicLib::LocalCartesian geoConverter;

        if(!init)
        {
            geoConverter.Reset(gnss_msg->lat, gnss_msg->lon, gnss_msg->height);
            init = true;
            return;
        }

        double xyz[3];
        geoConverter.Forward(gnss_msg->lat, gnss_msg->lon, gnss_msg->height,xyz[0],xyz[1],xyz[2]);

        Eigen::Matrix3d z_matrix;
        Eigen::Matrix3d x_matrix;
        Eigen::Matrix3d y_matrix;
        double heading_Z = gnss_msg->heading;
        double pitch_Y = gnss_msg->pitch;
        double roll_X = gnss_msg->roll;

        z_matrix << cos(heading_Z), -sin(heading_Z), 0,
                sin(heading_Z), cos(heading_Z),  0,
                0,                 0,            1;

        x_matrix << 1,                 0,              0,
                0,            cos(roll_X),     sin(roll_X),
                0,            -sin(roll_X),    cos(roll_X);

        y_matrix << cos(pitch_Y) ,     0,        -sin(pitch_Y),
                0,                 1,               0 ,
                sin(pitch_Y),      0,         cos(pitch_Y);

        //n -> b
        Eigen::Matrix3d R_n_b = y_matrix * x_matrix * z_matrix;
        Eigen::Matrix3d R_b_n = R_n_b.transpose();
        Eigen::Quaterniond q_b_n(R_b_n);//获得局部坐标系的四元数
        Eigen::Vector3d gnss_xyz(xyz[0],xyz[1],xyz[2]);
        Eigen::Vector3d gnss_lidar = extrinc_rotation * (gnss_xyz - extrinc_translation);//获得雷达坐标系的位置//////////////////////////////////////////

        nav_msgs::Odometry::Ptr thisgnss(new nav_msgs::Odometry());

        thisgnss->header.stamp = gnss_msg->header.stamp;
        thisgnss->pose.pose.position.x = gnss_lidar[0];
        thisgnss->pose.pose.position.y = gnss_lidar[1];
        thisgnss->pose.pose.position.z = gnss_lidar[2];
        thisgnss->pose.pose.orientation.w = q_b_n.w();
        thisgnss->pose.pose.orientation.x = q_b_n.x();
        thisgnss->pose.pose.orientation.y = q_b_n.y();
        thisgnss->pose.pose.orientation.z = q_b_n.z();
        //hisgnss->pose.pose.orientation = q_b_n;

        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*thisgnss);

    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {  //点云回调
        if (!cachePointCloud(laserCloudMsg))  //缓存点云，转换格式
            return;

        if (!deskewInfo())  //获取陀螺仪和里程计数据
            return;

        projectPointCloud();  //投影点云

        cloudExtraction();  //提取分割点云

        publishClouds();  //发布去畸变点云

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);//将点云数据加入队列尾部
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());  //将队列前面的消息保存到currentCloudMsg
        cloudQueue.pop_front();
        if (Config::sensor == LidarType::VELODYNE || Config::sensor == LidarType::LIVOX) {//将VELODYNE、LIVOX点云转换为PCL点云类型
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }

        // get timestamp  获得时间戳信息，找到每帧开始和结束的时间
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        // timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
        timeScanEnd = timeScanCur + laserCloudIn->points.back().latency;

        if (Config::debugLidarTimestamp) {
            std::cout << std::fixed << std::setprecision(12) << "end time from pcd and size: "
                      << laserCloudIn->points.back().latency
                      << ", " << laserCloudIn->points.size() << std::endl;
        }

        // check dense flag  检查是否dense
        if (laserCloudIn->is_dense == false) {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel  检查点云是否包含ring通道，检查其存在可以避免在无ring通道点云上运行导致不正确结果
        static int ringFlag = 0;
        if (ringFlag == 0) {
            ringFlag = -1;
            for (int i = 0; i < (int) currentCloudMsg.fields.size(); ++i) {  //遍历当前点云字段field，判断name中是否有ring，如果有，将ringFlag置1
                if (currentCloudMsg.fields[i].name == "ring") {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1) {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // check point time  检查点云是否包含时间，点云中需要包含每点的时间戳信息,才能利用这些信息进行运动畸变校正，检查时间戳的存在,可以避免在没有时间信息的点云上执行去畸变导致错误
        if (deskewFlag == 0) {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields) {
                if (field.name == "time" || field.name == "t" || field.name == "timestamp") {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN(
                        "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool deskewInfo() {
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure GNSS data available for the scan
        if (odomQueue.empty() || odomQueue.front().header.stamp.toSec() > timeScanCur  //检查gnssQueue队列是否有数据,以及时间戳是否包含当前帧的开始和结束时间
            || odomQueue.back().header.stamp.toSec() < timeScanEnd) {
            ROS_DEBUG("Waiting for GNSS data ...");
            return false;
        }

        odomDeskewInfo();

        return true;
    }

    nav_msgs::Odometry startgnssmsg;//每帧数据起始时刻
    Eigen::Vector3d startT;//第一帧输入的位置
    Eigen::Quaterniond startQ;//旋转

    void odomDeskewInfo() {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty()) {  //将里程计中时间早于当前帧前0.01秒的数据取出
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())//如果队列为空或者队列头时间早于当前帧时间，直接返回
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        //nav_msgs::Odometry startgnssmsg;//每帧数据起始时刻
        double ktime;
        for (int i = 0; i < (int) odomQueue.size(); i++) {  //遍历里程计队列，找到处于当前帧时间之前的第一个里程计数据作为起始位姿

            if (odomQueue[i].header.stamp.toSec() > timeScanCur){
                ktime = (timeScanCur - odomQueue[i-1].header.stamp.toSec()) / (odomQueue[i].header.stamp.toSec() - odomQueue[i-1].header.stamp.toSec());
                //平移
                startgnssmsg.pose.pose.position.x = odomQueue[i-1].pose.pose.position.x +
                                                    ktime * (odomQueue[i].pose.pose.position.x-odomQueue[i-1].pose.pose.position.x);
                startgnssmsg.pose.pose.position.y = odomQueue[i-1].pose.pose.position.y +
                                                    ktime * (odomQueue[i].pose.pose.position.y-odomQueue[i-1].pose.pose.position.y);
                startgnssmsg.pose.pose.position.z = odomQueue[i-1].pose.pose.position.z +
                                                    ktime * (odomQueue[i].pose.pose.position.z-odomQueue[i-1].pose.pose.position.z);
                //旋转
                Eigen::Quaterniond frontQ;
                Eigen::Quaterniond rearQ;
                frontQ.w() = odomQueue[i-1].pose.pose.orientation.w;
                frontQ.x() = odomQueue[i-1].pose.pose.orientation.x;
                frontQ.y() = odomQueue[i-1].pose.pose.orientation.y;
                frontQ.z() = odomQueue[i-1].pose.pose.orientation.z;

                rearQ.w() = odomQueue[i].pose.pose.orientation.w;
                rearQ.x() = odomQueue[i].pose.pose.orientation.x;
                rearQ.y() = odomQueue[i].pose.pose.orientation.y;
                rearQ.z() = odomQueue[i].pose.pose.orientation.z;

                //startT = Eigen::Vector3d(startgnssmsg.pose.pose.position.x,startgnssmsg.pose.pose.position.y,startgnssmsg.pose.pose.position.z);
                startT << startgnssmsg.pose.pose.position.x, startgnssmsg.pose.pose.position.y, startgnssmsg.pose.pose.position.z;
                startQ = frontQ.slerp(ktime,rearQ);
                // startgnssmsg.pose.pose.orientation.w = currentQ.w();
                // startgnssmsg.pose.pose.orientation.x = currentQ.x();
                // startgnssmsg.pose.pose.orientation.y = currentQ.y();
                // startgnssmsg.pose.pose.orientation.z = currentQ.z();

            }
            continue;


        }

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endgnssMsg;

        for (int i = 0; i < (int) odomQueue.size(); ++i) {  //遍历里程计队列，得到帧结束时间的里程计数据
            endgnssMsg = odomQueue[i];

            if (ROS_TIME(&endgnssMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        odomDeskewFlag = true;
    }



    void findRotation(double pointTime, Eigen::Isometry3d *transformation_matrix) {//IMU数据中找到与当前点对应时刻的旋转数据

        //Eigen::Isometry3d transformation_matrix = Eigen::Isometry3d::Identity();

        for(int i = 0;i < (int)odomQueue.size(); i++){
            nav_msgs::Odometry thisgnssmsg = odomQueue[i];
            double currentgnsstime = thisgnssmsg.header.stamp.toSec();

            if (currentgnsstime > pointTime)
            {
                nav_msgs::Odometry frontgnss = odomQueue[i-1];
                nav_msgs::Odometry reargnss = odomQueue[i];

                double k_time = (pointTime - frontgnss.header.stamp.toSec())
                                /(reargnss.header.stamp.toSec() - frontgnss.header.stamp.toSec());

                //平移
                thisgnssmsg.pose.pose.position.x = frontgnss.pose.pose.position.x +
                                                   k_time * (reargnss.pose.pose.position.x-frontgnss.pose.pose.position.x);
                thisgnssmsg.pose.pose.position.y = frontgnss.pose.pose.position.y +
                                                   k_time * (reargnss.pose.pose.position.y-frontgnss.pose.pose.position.y);
                thisgnssmsg.pose.pose.position.z = frontgnss.pose.pose.position.z +
                                                   k_time * (reargnss.pose.pose.position.z-frontgnss.pose.pose.position.z);

                //旋转
                Eigen::Quaterniond frontQ;
                Eigen::Quaterniond rearQ;
                frontQ.w() = frontgnss.pose.pose.orientation.w;
                frontQ.x() = frontgnss.pose.pose.orientation.x;
                frontQ.y() = frontgnss.pose.pose.orientation.y;
                frontQ.z() = frontgnss.pose.pose.orientation.z;

                rearQ.w() = reargnss.pose.pose.orientation.w;
                rearQ.x() = reargnss.pose.pose.orientation.x;
                rearQ.y() = reargnss.pose.pose.orientation.y;
                rearQ.z() = reargnss.pose.pose.orientation.z;
                // Eigen::Quaterniond frontQ = frontgnss.pose.pose.orientation;
                // Eigen::Quaterniond rearQ = reargnss.pose.pose.orientation;
                Eigen::Quaterniond currentQ = frontQ.slerp(k_time,rearQ);

                Eigen::Vector3d thispoint(thisgnssmsg.pose.pose.position.x,thisgnssmsg.pose.pose.position.y,thisgnssmsg.pose.pose.position.z);    //当前点

                Eigen::Vector3d translation = thispoint - startT; //平移向量
                Eigen::Matrix3d rotation_matrix = currentQ.toRotationMatrix() * startQ.toRotationMatrix().transpose();

                Eigen::Matrix3d rotation = T_b_n.inverse() * rotation_matrix;

                transformation_matrix->translation() = translation;
                transformation_matrix->linear() = rotation;

            }
            continue;

        }

    }

    PointType deskewPoint(PointType *point, double relTime) {  //激光点去畸变
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)//判断是否可以去畸变
            return *point;

        double pointTime = timeScanCur + relTime;//scan时间加相对时间，获得点的时间

        Eigen::Isometry3d transformation_matrix;
        findRotation(pointTime, &transformation_matrix);

        Eigen::Vector3d newpoint;
        newpoint << point->x,point->y,point->z;
        Eigen::Vector3d deskewnewpoint = transformation_matrix.inverse() * newpoint;


        PointType newPoint;//使用transBt将原始点坐标变换到起始时刻坐标系下，返回新得到的去畸变后的点newPoint
        newPoint.x = deskewnewpoint.x();
        newPoint.y = deskewnewpoint.y();
        newPoint.z = deskewnewpoint.z();
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud() {  //
        int cloudSize = laserCloudIn->points.size(); //获取原始点云大小
//    std::cout << "point size raw: " << cloudSize << std::endl;
        // range image projection
        for (int i = 0; i < cloudSize; ++i) {  //遍历每一个点
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;//获取xyz、强度
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint); //计算点的距离
            if (range < Config::lidarMinRange || range > Config::lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;//获取行数
            if (rowIdn < 0 || rowIdn >= Config::N_SCAN)
                continue;

            if (rowIdn % Config::downsampleRate != 0)
                continue;

            int columnIdn = -1;//获取列号
            if (Config::sensor == LidarType::VELODYNE || Config::sensor == LidarType::OUSTER || Config::sensor == LidarType::HESAI) {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;//水平角分辨率
                static float ang_res_x = 360.0 / float(Config::Horizon_SCAN);//Horizon_SCAN=1800,每格0.2度
                //horizonAngle 为[-180,180],horizonAngle -90 为[-270,90],-round 为[-90,270], /ang_res_x 为[-450,1350]
                //+Horizon_SCAN/2后为[450,2250]
                // 即把horizonAngle从[-180,180]映射到[450,2250]
                columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Config::Horizon_SCAN / 2;
                //大于1800，则减去1800，相当于把1801～2250映射到1～450
                //先把columnIdn从horizonAngle:(-PI,PI]转换到columnIdn:[H/4,5H/4],
                //然后判断columnIdn大小，把H到5H/4的部分切下来，补到0～H/4的部分。
                //将它的范围转换到了[0,H] (H:Horizon_SCAN)。
                //这样就把扫描开始的地方角度为0与角度为360的连在了一起，非常巧妙。
                //如果前方是x，左侧是y，那么正后方左边是180，右边是-180。这里的操作就是，把它展开成一幅图:
                //                   0
                //   90                        -90
                //          180 || (-180)
                //  (-180)   -----   (-90)  ------  0  ------ 90 -------180
                //变为:  90 ----180(-180) ---- (-90)  ----- (0)    ----- 90

                if (columnIdn >= Config::Horizon_SCAN)
                    columnIdn -= Config::Horizon_SCAN;
            } else if (Config::sensor == LidarType::LIVOX) {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            }

            if (columnIdn < 0 || columnIdn >= Config::Horizon_SCAN)//检查列号是否在范围内
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].latency);//进行点云去畸变

            rangeMat.at<float>(rowIdn, columnIdn) = range;//将去畸变后的点范围和坐标存入rangeMat

            int index = columnIdn + rowIdn * Config::Horizon_SCAN;//计算索引index,将去畸变后的点存入fullCloud
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction() {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < Config::N_SCAN; ++i) {
            //提取特征的时候，每一行的前5个和最后5个不考虑
            cloudInfo.startRingIndex[i] = count - 1 + 5;//从第六个点开始找（前面5个不考虑）

            for (int j = 0; j < Config::Horizon_SCAN; ++j) {
                if (rangeMat.at<float>(i, j) != FLT_MAX) {
                    // mark the points' column index for marking occlusion later
                    //记录激光点对应的Horizon_SCAN方向上的索引
                    cloudInfo.pointColInd[count] = j;
                    // save range info  激光点距离
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
                    // save extracted cloud 加入有效激光点
                    extractedCloud->push_back(fullCloud->points[j + i * Config::Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;//
        }
    }

    void publishClouds() {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, Config::lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);

        publishCloud(pubFullCloud, fullCloud, cloudHeader.stamp, Config::lidarFrame);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lio_sam_6axis");

    ImageProjection IP;

    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
