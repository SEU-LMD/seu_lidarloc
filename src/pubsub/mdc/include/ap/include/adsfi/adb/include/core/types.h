/* *
 * FUNCTION: Define Common Types
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 */

#ifndef HAF_CORE_TYPES_H
#define HAF_CORE_TYPES_H

#include <cstddef>
#include <vector>
#include <string>
#include <array>
#include <memory>
#include "core/timer.h"
#include "core/basic_types.h"
namespace Adsfi {
/* * Defines a 3x3 matrix of floating point numbers. To
 * access an element of the matrix: item(row,col) = _array[row + col*3].
 *           */
struct HafMatrix3f {
    float32_t array[3 * 3];
};

/* * Defines a 3x3 matrix of float64_t floating point numbers. To
 * access an element of the matrix: item(row,col) = _array[row + col*3].
 *           */
struct HafMatrix3d {
    float64_t array[3 * 3];
};

/* * Defines a 3x4 matrix of floating point numbers (column major).
 * access an element of the matrix: item(row,col) = _array[row + col*3].
 *           */
struct HafMatrix34f {
    float32_t array[3 * 4];
};

/* * Defines a 4x4 matrix of floating point numbers (column major).
 * access an element of the matrix: item(row,col) = _array[row + col*4].
 *           */
struct HafMatrix4f {
    float32_t array[4 * 4];
};

struct HafMatrix4d {
    float64_t array[4 * 4];
};

template <typename T>
struct Point2D {
    static_assert(std::is_pod<T>::value, "T must be a POD type.");
    T x{};
    T y{};
};
using Point2i = Point2D<int32_t>;
using Point2f = Point2D<float32_t>;
using Point2d = Point2D<float64_t>;

const int32_t g_rectangleVertexNum = 4;
// / Defines a rectangle.
template <typename T> struct HafRect2D {
    Point2D<T> center;                        // 边框中心点
    Point2D<T> location;                      // 边框左上角
    Point2D<float64_t> centerCovariance;      // 边框中心点方差
    Point2D<T> size;                          // 边框大小
    Point2D<float64_t> sizeCovariance;        // 边框大小方差
    float32_t orientation;                    // 边框在当前坐标系下的偏转角
    float32_t orientationCovariance;          // 边框偏转角的方差
    Point2D<T> corners[g_rectangleVertexNum]; // 边框顶点
};
using HafRect2i = HafRect2D<int32_t>;
using HafRect2f = HafRect2D<float32_t>;
using HafRect2d = HafRect2D<float64_t>;

template <typename T>
struct Point3D {
    static_assert(std::is_pod<T>::value, "T must be a POD type.");
    T x{};
    T y{};
    T z{};
    Point3D() = default;
    Point3D(const T& valueX, const T& valueY, const T& valueZ) : x(valueX), y(valueY), z(valueZ)
    {}
    Point3D operator+(const Point3D& rightValue) const
    {
        return Point3D(x + rightValue.x, y + rightValue.y, z + rightValue.z);
    }
    Point3D operator-(const Point3D& rightValue) const
    {
        return Point3D(x - rightValue.x, y - rightValue.y, z - rightValue.z);
    }
};
using Point3i = Point3D<int32_t>;
using Point3f = Point3D<float32_t>;
using Point3d = Point3D<float64_t>;

template <typename T>
struct PointXYZW {
    static_assert(std::is_pod<T>::value, "T must be a POD type.");
    T x{};
    T y{};
    T z{};
    T w{};
    PointXYZW() = default;
    PointXYZW(const T valueX, const T valueY, const T valueZ, const T valueW)
        : x(valueX), y(valueY), z(valueZ), w(valueW)
    {}
};
using Point4i = PointXYZW<int32_t>;
using Point4f = PointXYZW<float32_t>;
using Point4d = PointXYZW<float64_t>;
const int32_t g_cubeVertexNum = 8;

template <typename T> struct HafRect3D {
    Point3D<T> center;                               // 边框中心点
    Point3D<float64_t> centerCovariance;             // 边框中心点方差
    Point3D<T> size;                                 // 边框大小
    Point3D<float64_t> sizeCovariance;               // 边框大小方差
    float32_t orientation;                           // 边框在当前坐标系下的偏转角
    float32_t orientationCovariance;                 // 边框偏转角的方差
    std::array<Point3D<T>, g_cubeVertexNum> corners; // 边框顶点

    HafRect3D() = default;
    HafRect3D(const Point3D<T> &cent, const Point3D<float64_t> &centCo, const Point3D<T> &size0,
        const Point3D<float64_t> &sizeCo, const float32_t &oriCo)
        : center(cent), centerCovariance(centCo), size(size0), sizeCovariance(sizeCo), orientation(oriCo)
    {}
    HafRect3D(const HafRect3D &rect)
        : center(rect.center),
          centerCovariance(rect.centerCovariance),
          size(rect.size),
          sizeCovariance(rect.sizeCovariance),
          orientation(rect.orientation),
          orientationCovariance(rect.orientationCovariance),
          corners(rect.corners)
    {}
};
using HafRect3i = HafRect3D<int32_t>;
using HafRect3f = HafRect3D<float32_t>;
using HafRect3d = HafRect3D<float64_t>;

// / Defines a line segment.
struct HafLine3f {
    Point3f first;
    Point3f second;
};

// / Defines a polyline.
struct HafPolyline3f {
    const std::unique_ptr<Point3f> points; // !< pointer to the first element in the container.
    uint32_t pointCount;   // !< number of points.
};

/* ******************************************************************************
    功能描述		:  Lidar输出的点
****************************************************************************** */
struct PointXYZI : public Point3D<float32_t> {
    uint16_t intensity;
    PointXYZI() : Point3D<float32_t>(), intensity(0U) {}
};
struct PointXYZIR : public PointXYZI {
    uint16_t ring;
    PointXYZIR() : PointXYZI(), ring(0U) {}
};
struct PointXYZIRT : public PointXYZIR {
    uint32_t latency; // 从当前帧第一个扫描点开始的延时
    PointXYZIRT() : PointXYZIR(), latency(0U) {}
};

/* ******************************************************************************
    结构 名		:  LidarFrame
    功能描述		:  Lidar每帧输出的数据
****************************************************************************** */
template <typename T> struct LidarFrame {
    HafTime timestamp;
    uint32_t seq;
    std::string frameID;
    std::vector<T> pointCloud;
};

/* ******************************************************************************
    功能描述		:  HafRadarModeDataInfo数据
****************************************************************************** */
struct HafRadarModeDataInfo : public Point3D<float64_t> {
    Point3d rms;
    Point3d quality;

    HafRadarModeDataInfo() = default;
};

/* ******************************************************************************
    功能描述		:  HafRadarTrackOrient数据
****************************************************************************** */
struct HafRadarTrackOrient {
    float64_t orientation;
    float64_t rms;
    float64_t quality;
};

/* ******************************************************************************
    功能描述		:  HafRadarTrackYaw数据
****************************************************************************** */
struct HafRadarTrackYaw {
    float64_t yawRate;
    float64_t rms;
    float64_t quality;
};

/* ******************************************************************************
    功能描述		:  HafRadarTrackData数据
****************************************************************************** */
struct HafRadarTrackData {
    uint8_t id;
    uint8_t idState;
    uint8_t lifetime;
    HafRadarModeDataInfo position;
    HafRadarModeDataInfo velocity;
    HafRadarModeDataInfo acceleration;
    HafRadarModeDataInfo size;
    HafRadarTrackOrient orient;
    HafRadarTrackYaw yaw;
    float64_t rcs;
    float64_t snr;
    float64_t underpassProbability;
    float64_t overpassProbability;
    uint8_t existProbability;
    uint8_t movProperty;
    uint8_t trackState;
    uint8_t trackType;
};

/* *******************************************************************************
    功能描述		:  HafRadarStateFrameError数据
******************************************************************************* */
struct HafRadarStateFrameError {
    uint8_t persistentError;
    uint8_t temporaryError;
    uint8_t interferenceError;
    uint8_t temperatureError;
    uint8_t voltageError;
    uint8_t blockError;
    uint8_t broadcastError;
    uint8_t electricAxisError;
    uint8_t configError;
    uint8_t calibrationError;
};

/* *******************************************************************************
    功能描述		:  HafRadarStateFrameMode数据
******************************************************************************* */
struct HafRadarStateFrameMode {
    uint8_t can0WorkMode;
    uint8_t can1WorkMode;
    uint8_t dualCanMode;
    uint8_t timmingMode;
    uint8_t powerMode;
    uint8_t performanceMode;
};

/* *******************************************************************************
    功能描述		:  HafRadarStateFrame数据
******************************************************************************* */
struct HafRadarStateFrame {
    uint8_t sensorId;
    uint8_t nvmReadStatus;
    uint8_t nvmWriteStatus;
    HafRadarStateFrameError radarStateFrameError;
    HafRadarStateFrameMode radarStateFrameMode;
    uint16_t maxDistance;
    uint8_t sortIndex;
    uint8_t radarPower;
    uint8_t ctrlRelay;
    uint8_t outputType;
    uint8_t sendQuality;
    uint8_t sendExtinfo;
    uint8_t motionRxState;
    uint8_t rcsThreshold;
    uint8_t connectorDirection;
    uint8_t radarPosition;
    uint8_t hwError;
};

using HafTrkList = std::vector<HafRadarTrackData>;
/* ******************************************************************************
    功能描述		:  HafRadarTrackArrayFrame数据
****************************************************************************** */
struct HafRadarTrackArrayFrame {
    HafTime timestamp;
    std::string frameID;
    uint8_t sensorID;
    uint32_t seq;
    HafRadarStateFrame radarState;
    HafTrkList trackList;
};

/* ******************************************************************************
    功能描述		:  HafRadarDetectData数据
****************************************************************************** */
struct HafRadarDetectData {
    uint8_t id;
    uint8_t idPair;
    uint8_t coordinate;
    HafRadarModeDataInfo position;
    HafRadarModeDataInfo velocity;
    HafRadarModeDataInfo acceleration;
    float64_t rcs;
    float64_t snr;
    uint8_t existProbability;
    uint8_t falseProbability;
    uint8_t movProperty;
    uint8_t invalidState;
    uint8_t ambiguity;
};

using HafDetList = std::vector<HafRadarDetectData>;
/* ******************************************************************************
    功能描述		:  HafRadarDetectArrayFrame数据
****************************************************************************** */
struct HafRadarDetectArrayFrame {
    HafTime timestamp;
    std::string frameID;
    uint8_t sensorID;
    uint32_t seq;
    HafRadarStateFrame radarState;
    HafDetList detectList;
    std::vector<float64_t> maxDistanceOverAzimuthList;
    std::vector<float64_t> azimuthForMaxDistanceList;
    std::vector<float64_t> factorDistanceOverElevationList;
    std::vector<float64_t> elevationForFactorMaxDistanceList;
    float64_t maxDistanceDueToProgram;
    float64_t minDistanceDueToProgram;
};

/* *******************************************************************************
    功能描述		:  HafRadarConditionFrameError数据
******************************************************************************* */
struct HafRadarConditionFrameError {
    uint8_t persistentError;
    uint8_t temporaryError;
    uint8_t interferenceError;
    uint8_t temperatureError;
    uint8_t voltageError;
};

/* *******************************************************************************
    功能描述		:  HafRadarConditionFrame数据
******************************************************************************* */
struct HafRadarConditionFrame {
    uint8_t nvmReadStatus;
    uint8_t nvmWriteStatus;
    HafRadarConditionFrameError radarStatusFrameError;
    uint16_t maxDistance;
    uint8_t sortIndex;
    uint8_t radarPower;
    uint8_t ctrlRelay;
    uint8_t outputType;
    uint8_t sendQuality;
    uint8_t sendExtinfo;
    uint8_t motionRxState;
    uint8_t rcsThreshold;
};

enum HafBufferType {
    HAF_BUFFER_CPU = 0,
    HAF_BUFFER_DVPP = 1,
    HAF_BUFFER_AICORE = 2,
};

enum HafImageType {
    HAF_IMAGE_RGB_UINT8 = 0,
    HAF_IMAGE_BGR_UINT8 = 1,
    HAF_IMAGE_YUV420SP_NV12_UINT8 = 2,
    HAF_IMAGE_GRAY_UINT8 = 3,
    HAF_IMAGE_RGB_FLOAT32 = 4,
    HAF_IMAGE_BGR_FLOAT32 = 5,
    HAF_IMAGE_YUV420SP_NV12_FLOAT32 = 6,
    HAF_IMAGE_GRAY_FLOAT32 = 7,
};

struct HafRoi {
    uint32_t left;
    uint32_t top;
    uint32_t right;
    uint32_t bottom;
};

struct HafRoi2 {
    HafRoi cropArea;
    HafRoi pasteArea;
};

struct ImageDataV2 {
    uint32_t width{};
    uint32_t height{};
    uint32_t dataSize{};
    HafBufferType bufferType{HAF_BUFFER_CPU};
    HafImageType imageType{HAF_IMAGE_RGB_UINT8};
    uint32_t seq{};
    std::vector<uint8_t> rawData;
};

struct ImageData {
    uint32_t width{};
    uint32_t height{};
    uint32_t dataSize{};
    HafBufferType bufferType{HAF_BUFFER_CPU};
    HafImageType imageType{HAF_IMAGE_RGB_UINT8};
    uint32_t seq{};
    uint8_t* rawData{};
    ImageData() = default;
    explicit ImageData(ImageDataV2& img2)
    {
        width = img2.width;
        height = img2.height;
        dataSize = img2.dataSize;
        bufferType = img2.bufferType;
        imageType = img2.imageType;
        seq = img2.seq;
        rawData = static_cast<decltype(rawData)>(img2.rawData.data());
    }
};

struct ImageFrameV2 {
    HafTime timestamp;
    std::string frameID;
    ImageDataV2 data{};
};

/* ******************************************************************************
    结构 名		:  ImageFrame
    功能描述		:  Camera每帧输出的数据
****************************************************************************** */
struct ImageFrame {
    HafTime timestamp;
    std::string frameID;
    ImageData data{};
    ImageFrame() = default;
    explicit ImageFrame(ImageFrameV2& img2)
    {
        timestamp = img2.timestamp;
        frameID = img2.frameID;
        data = ImageData(img2.data);
    }
};

enum HafImageInterpolationType {
    HAF_INTER_NEAREST = 0,  // 最近邻插值
    HAF_INTER_LINEAR = 1,   // 双线性插值
    HAF_INTER_AREA = 2,     // 使用像素区域关系进行重采样
    HAF_INTER_CUBIC = 3,    // 4x4，像素邻域的双三次插值
    HAF_INTER_LANCZOS4 = 4, // 8x8像素邻域的Lanczos插值
    HAF_INTER_KAISER = 5    // 凯撒窗，当前输入仅支持此种缩放
};

struct HafImageProperties {
    HafImageType imageType; // RGB BGR YUV INT8 FLOAT32
    HafBufferType bufferType;
    int32_t width;
    int32_t height;
    int32_t frameID;
};

/* * Specifies a 3d rigid transformation.
 * The transformation is a 4x4 matrix in column-major order.
 * The top left 3x3 represents rotation, and the right column is the translation.
 * The bottom row is expected to be [0 0 0 1]
 * To access an element of the matrix: item(row,col) = _array[row + col*4].
 */
using HafTransformation4f = struct HafMatrix4f;
using HafTransformation4d = struct HafMatrix4d;

/* *
 * The top left 2 x 2 represents rotation and scaling, the right column is the translation.
 * The bottom row is expected to be [0 0 1]
 * To access an element of the matrix: item(row,col) = _array[row + col*3].
 */
using HafTransformation3f = struct HafMatrix3f;
using HafTransformation3d = struct HafMatrix3d;

enum Coordinate {
    FRAME,    // 像素坐标系（2D）
    IMAGE,    // 图像坐标系（2D）
    // 以下都是立体坐标系
    CAMERA,   // 相机坐标系
    LIDAR,    // 激光雷达坐标系
    RADAR,    // 毫米波雷达坐标系
    VEHICLE,  // 自车坐标系
    ABSOLUTE  // 世界坐标系
};

/* ******************************************************************************
    结构 名		:  Accel
    功能描述		:  定位输出的线加速度、角加速度
****************************************************************************** */
struct HafAccel {
    Point3d linear;
    Point3d angular;
};

/* ******************************************************************************
    结构 名		:  AccelWithCovariance
    功能描述		:  含有方差的加速度信息
****************************************************************************** */
struct HafAccelWithCovariance {
    HafAccel accel;
    float64_t covariance[6 * 6];
};

/* ******************************************************************************
    结构 名		:  Quaternion
    功能描述		:  定位四元数
****************************************************************************** */
struct HafQuaternion {
    float64_t x;
    float64_t y;
    float64_t z;
    float64_t w;
};

/* ******************************************************************************
    结构 名		:  Pose
    功能描述		:  位姿信息，包括UTM坐标和四元数
****************************************************************************** */
struct HafPose {
    Point3d position;
    HafQuaternion orientation;
};

/* ******************************************************************************
    结构 名		:  PoseWithCovariance
    功能描述		:  含有方差信息的位姿信息
****************************************************************************** */
struct HafPoseWithCovariance {
    HafPose pose;
    float64_t covariance[6 * 6];
};

/* ******************************************************************************
    结构 名		:  Twist
    功能描述		:  定位输出的线速度、角速度
****************************************************************************** */
struct HafTwist {
    Point3d linear;
    Point3d angular;
};

/* ******************************************************************************
    结构 名		:  Twist
    功能描述		:  含有方差信息的速度结构体
****************************************************************************** */
struct HafTwistWithCovariance {
    HafTwist twist;
    float64_t covariance[6 * 6];
};

/* ******************************************************************************
    结构 名		:  Header
    功能描述		:  含有时间戳信息的结构体
****************************************************************************** */
struct HafHeader {
    uint32_t seq;
    HafTime timestamp;
    std::string frameID;
};
/* ******************************************************************************
    结构 名		:  Header
    功能描述		:  含有置信度的int类型
****************************************************************************** */
struct HafUint8WithValid {
    uint8_t value;
    uint8_t confidence;
};
/* ******************************************************************************
    结构 名		:  Header
    功能描述		:  含有置信度的float类型
****************************************************************************** */
struct HafFloat32WithValid {
    float32_t value;
    uint8_t confidence;
};
/* ******************************************************************************
    结构 名		:  HafGear
    功能描述		:  档位值
****************************************************************************** */
struct HafGear {
    uint8_t value;
};

/* * Defines a 3-dimension vector of floating point numbers. To
 * access an element of the vector: item(index) = _array[index].
 *           */
struct HafEulerAngle3f {
    float32_t roll;
    float32_t pitch;
    float32_t yaw;
};

enum HafCoordinateType {
    HAF_CAMERA_FRAME = 0, // X轴向右， Y轴向下，Z轴向前
    HAF_LASER_FRAME = 1   // X轴向前， Y轴向左，Z轴向上
};

struct PointWithRange : public Point3D<float32_t> {
    float32_t range; // 包含深度的点
    explicit PointWithRange() : Point3D(), range(0.0F) {}
    explicit PointWithRange(const float32_t valueX, const float32_t valueY, const float32_t valueZ,
        const float32_t valueRange)
        : Point3D(valueX, valueY, valueZ), range(valueRange)
    {}
};

struct RangeImageData {
    int32_t width{};
    int32_t height{};
    int32_t dataSize{};
    float32_t startAngle{};
    float32_t fovAngle{};
    std::vector<PointWithRange> rangeImageData;
};

/* ******************************************************************************
    结构 名		:  RangeImageFrame
    功能描述		:  RangeImage每帧的数据
****************************************************************************** */
struct RangeImageFrame {
    HafTime timestamp;
    uint32_t seq;
    std::string frameID;
    RangeImageData data;
};
}
/* * @} */
#endif // HAF_CORE_TYPES_H
