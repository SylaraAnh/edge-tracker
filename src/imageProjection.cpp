#include "utility.h"
#include "edge_tracker/cloud_info.h"

#include <livox_ros_driver/CustomMsg.h>

// LIVOX AVIA
#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)
enum E_jump
{
    Nr_nor,
    Nr_zero,
    Nr_180,
    Nr_inf,
    Nr_blind
};
enum Feature
{
    Nor,
    Poss_Plane,
    Real_Plane,
    Edge_Jump,
    Edge_Plane,
    Wire,
    ZeroPoint
};
enum Surround
{
    Prev,
    Next
};
struct orgtype
{
    double  range;
    double  dista;
    double  angle[2];
    double  intersect;
    E_jump  edj[2];
    Feature ftype;
    orgtype()
    {
        range     = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype     = Nor;
        intersect = 2;
    }
};

// Velodyne
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float    time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                intensity)(uint16_t, ring,
                                                                           ring)(float, time, time))

// Ouster
// struct PointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// }EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )

const int queueLength = 500;

class ImageProjection : public ParamServer
{
private:
    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subLaserPointCloud;
    ros::Publisher  pubLaserCloud;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber              subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber                subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2>    cloudQueue;
    std::deque<livox_ros_driver::CustomMsg> livoxCloudQueue;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];  // ??????????????????????????????????????????bias???noise
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int             imuPointerCur;  //????????????imu?????????
    bool            firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int     deskewFlag;
    cv::Mat rangeMat;

    bool  odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    edge_tracker::cloud_info cloudInfo;
    double              timeScanCur;   // ?????????scan????????????
    double              timeScanNext;  // ??????scan?????????
    std_msgs::Header    cloudHeader;

    vector<int> columnIdnCountVec;  // for livox

public:
    ImageProjection() : deskewFlag(0)
    {
        ROS_INFO("\033[1;32m Subscribe to IMU Topic: %s.\033[0m", imuTopic.c_str());
        subImu  = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this,
                                                ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>(
            PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 2000,
            &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        switch (lidarType)
        {
            ROS_INFO("\033[1;32m Lidar Type: %s.\033[0m", lidarType == VELO16 ? "VELO16" : "AVIA");
            ROS_INFO("\033[1;32m Subscribe to Lidar Topic: %s.\033[0m", pointCloudTopic.c_str());
            case VELO16:
                subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
                    pointCloudTopic, 5, &ImageProjection::velo16CloudHandler, this,
                    ros::TransportHints().tcpNoDelay());
                break;
            case AVIA:
                subLaserCloud = nh.subscribe<livox_ros_driver::CustomMsg>(
                    pointCloudTopic, 5, &ImageProjection::aviaCloudHandler, this,
                    ros::TransportHints().tcpNoDelay());
                break;
            default:
                printf("Lidar type is wrong.\n");
                exit(0);
                break;
        }
        // ????????????????????????????????????
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>(
            PROJECT_NAME + "/lidar/deskew/cloud_deskewed", 5);
        pubLaserCloudInfo =
            nh.advertise<edge_tracker::cloud_info>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur  = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection() {}

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        sensor_msgs::Imu            thisImu = imuConverter(*imuMsg);
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
        // cout << "thisImu.header.stamp.toSec() = " << to_string(thisImu.header.stamp.toSec())
        //      << endl;

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << ", y: " <<
        // thisImu.linear_acceleration.y
        //      << ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << ", y: " << thisImu.angular_velocity.y
        //      << ", z: " << thisImu.angular_velocity.z << endl;
        // double         imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl
        //      << endl;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void velo16CloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // 1 ??????????????????????????????????????????????????? ????????????????????????
        if (!cachePointCloud(laserCloudMsg))
            return;

        // 2 ????????????????????????scan??????IMU???VO??????
        if (!deskewInfo())
            return;

        // 3 ???????????????????????????????????????
        projectPointCloud();

        // 4 ????????????
        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    void aviaCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg)
    {
        // 1 ??????????????????????????????????????????????????? ????????????????????????
        if (!cachePointCloud(laserCloudMsg))
            return;

        // 2 ????????????????????????scan??????IMU???VO??????
        if (!deskewInfo())
            return;

        // 3 ???????????????????????????????????????
        projectPointCloud();

        // 4 ????????????
        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    void aviaCloudHandler(const livox_ros_driver::CustomMsg::ConstPtr &laserCloudMsg)
    {
        // 1 ??????????????????????????????????????????????????? ????????????????????????
        if (!cachePointCloud(laserCloudMsg))
            return;

        // 2 ????????????????????????scan??????IMU???VO??????
        if (!deskewInfo())
            return;

        // 3 ???????????????????????????????????????
        projectPointCloud();

        // 4 ????????????
        cloudExtraction();

        publishClouds();

        resetParameters();
    }
    //?????????????????????????????????????????????
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        sensor_msgs::PointCloud2 currentCloudMsg = cloudQueue.front();
        cloudQueue.pop_front();

        cloudHeader  = currentCloudMsg.header;
        timeScanCur  = cloudHeader.stamp.toSec();
        timeScanNext = cloudQueue.front().header.stamp.toSec();

        // convert cloud
        pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

        // check dense flag
        if (laserCloudIn->is_dense == false)  ///??????????????????????????????????????????NaN???
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        // ?????? ??????????????????ring??????(??????)
        // ???????????????????????????rowIdn
        // sensor_msgs::PointCloud2.fields:
        // https://blog.csdn.net/qq_45954434/article/details/116179169
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point "
                          "cloud data!");
                ros::shutdown();
            }
        }

        // check point time
        // yaml????????? timeField: "time"    # point timestamp field, Velodyne - "time", Ouster - "t"
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                //??????????????????????????????????????????
                if (currentCloudMsg.fields[i].name == timeField)
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system "
                         "will drift significantly!");
        }

        return true;
    }

    //?????????????????????????????????????????????
    bool cachePointCloud(const livox_ros_driver::CustomMsg::ConstPtr &laserCloudMsg)
    {
        // cache point cloud
        livoxCloudQueue.push_back(*laserCloudMsg);
        if (livoxCloudQueue.size() <= 2)
            return false;

        livox_ros_driver::CustomMsg currentCloudMsg = livoxCloudQueue.front();
        livoxCloudQueue.pop_front();

        cloudHeader  = currentCloudMsg.header;
        cloudHeader.frame_id  = "imu_link";
        timeScanCur  = cloudHeader.stamp.toSec();
        timeScanNext = livoxCloudQueue.front().header.stamp.toSec();
        // timeScanNext =
        //     timeScanCur + 1e-9 * static_cast<double>(currentCloudMsg.points.back().offset_time);
        // ;
        for (int i = 0; i < currentCloudMsg.point_num; i++)
        {
            if ((currentCloudMsg.points[i].line < N_SCAN) &&
                (!IS_VALID(currentCloudMsg.points[i].x)) &&
                (!IS_VALID(currentCloudMsg.points[i].y)) &&
                (!IS_VALID(currentCloudMsg.points[i].z)) && currentCloudMsg.points[i].x > 2)
            {
                // https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol
                // See [3.4 Tag Information]
                if ((currentCloudMsg.points[i].x > 2.0) &&
                    (((currentCloudMsg.points[i].tag & 0x03) != 0x00) ||
                     ((currentCloudMsg.points[i].tag & 0x0C) != 0x00)))
                {
                    // Remove the bad quality points
                    continue;
                }
                PointXYZIRT point;
//                point.x         = currentCloudMsg.points[i].x;
//                point.y         = currentCloudMsg.points[i].y;
//                point.z         = currentCloudMsg.points[i].z;
//                point.intensity = currentCloudMsg.points[i].reflectivity;
//                point.ring      = currentCloudMsg.points[i].line;
//                point.time      = currentCloudMsg.points[i].offset_time;
                point.x         = currentCloudMsg.points[i].x;
                point.y         = currentCloudMsg.points[i].y;
                point.z         = currentCloudMsg.points[i].z;
                point.intensity = currentCloudMsg.points[i].reflectivity;
                point.ring      = currentCloudMsg.points[i].line;
                point.time      = currentCloudMsg.points[i].offset_time;

                laserCloudIn->push_back(point);
            }
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur ||
            imuQueue.back().header.stamp.toSec() < timeScanNext)
        {
            // cout << "imuQueue.front().header.stamp.toSec()="
            //      << to_string(imuQueue.front().header.stamp.toSec()) << endl;
            // cout << "imuQueue.back().header.stamp.toSec()="
            //      << to_string(imuQueue.back().header.stamp.toSec()) << endl;
            // cout << "timeScanCur=" << to_string(timeScanCur) << endl;
            // cout << "timeScanNext=" << to_string(timeScanNext) << endl;
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }
        // 2.1 ??????lidar???IMU??????
        // TODO: ???????????????lidar?????????10hz
        imuDeskewInfo();

        // 2.1 ??????lidar???VO??????
        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() <
                timeScanCur - 0.01)  // 0.01s ?????????????????????????????????????????????10hz??????
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg     = imuQueue[i];
            double           currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                //???message?????????IMU????????????tf???????????????
                //???????????????float????????????????????? ????????????????????? ?????????????????????
                //???imu???????????????????????? ??????????????????imu????????????????????????????????????????????????
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit,
                              &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanNext + 0.01)
                break;

            if (imuPointerCur == 0)
            {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff        = currentImuTime - imuTime[imuPointerCur - 1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        // TODO??? ????????????rpy??????????????????tf???????????????????????????tf????????????????????????????????????
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        // ????????????????????????????????????????????????????????????
        cloudInfo.odomX       = startOdomMsg.pose.pose.position.x;
        cloudInfo.odomY       = startOdomMsg.pose.pose.position.y;
        cloudInfo.odomZ       = startOdomMsg.pose.pose.position.z;
        cloudInfo.odomRoll    = roll;
        cloudInfo.odomPitch   = pitch;
        cloudInfo.odomYaw     = yaw;
        cloudInfo.odomResetId = (int)round(startOdomMsg.pose.covariance[0]);

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;
        // ????????????????????????????????????????????????????????????lidar????????? ?????????????????????????????????
        if (odomQueue.back().header.stamp.toSec() < timeScanNext)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanNext)
                continue;
            else
                break;
        }

        // ???visual_estimator/utility/visualization.cpp????????????LIO???????????????????????????????????????failureCount
        // ???????????????-1?????????clearState????????????++failureCount
        // ???????????????failureCount??????????????????Lidar??????????????????????????????????????????????????????????????????????????????????????????
        // ?????????????????????????????????????????????
        if (int(round(startOdomMsg.pose.covariance[0])) !=
            int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(
            startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y,
            startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd =
            pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y,
                                   endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        // ??????VO????????????scsan????????????scan???????????????
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre,
                                          pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    // ????????????????????????imu??????,???????????????????????????
    // ????????????????????????,?????????????????????,???imu??????????????????????????????????????????(??????)?????????
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        int imuPointerFront = 0;
        //???????????????????????????????????????imu?????????
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        // IMU????????????????????????or??????imu??????
        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            //?????????????????????????????????IMU?????????
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        else
        {
            // ???????????????????????????
            // ?????????: imuPointerBack < imuPointerCur < imuPointerFront
            int    imuPointerBack = imuPointerFront - 1;
            double ratioFront     = (pointTime - imuTime[imuPointerBack]) /
                                (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) /
                               (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    // 3.1.2 ??????????????????????????????
    // odomIncre??????VO??????????????????VO??????????????????????????????????????????or?????????????????????????????????????????? todo
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanNext - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    // 3.1????????????????????????????????????
    //  ???????????????????????????????????????????????????????????????????????????????????????xyz???????????????????????????lidar???
    //  ??????????????????????????????????????????????????????lidar????????????????????????xyzrpy??????????????????????????????
    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        //?????????????????????+??????????????????????????????
        double pointTime = timeScanCur + relTime;

        // 3.1.1???????????????????????? ?????????????????????????????????????????????????????????
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        // ????????????(start)???????????????lidar???????????????
        // ???????????????????????????????????? firstPointFlag = true
        if (firstPointFlag == true)
        {
            // T_l(idar)s(tart) -> T_s(tart)l(idar)
            transStartInverse =
                (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur))
                    .inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal =
            pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        // T_sf(inal) = T_sl * T_lf
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y +
                     transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y +
                     transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y +
                     transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    // ????????????????????????RangeImage???
    // ??????????????????????????? TODO: ????????????????????????????????????
    void projectPointCloud()
    {
        int cloudSize = (int)laserCloudIn->points.size();
        auto time_end = laserCloudIn->points.back().time;
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            // ?????????????????????: x?????????, y?????????, z?????????
            PointType thisPoint;
            thisPoint.x         = laserCloudIn->points[i].x;
            thisPoint.y         = laserCloudIn->points[i].y;
            thisPoint.z         = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            // ????????????????????????????????????N_SCAN???????????????????????????
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;
            // ?????????
            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (lidarType == VELO16)
            {
                // ?????????????????????
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                // ???????????????(??)
                static float ang_res_x = 360.0 / float(Horizon_SCAN);
                // ????????????????????????columnId
                // ?????????????????????????????????y????????????,???????????????
                columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }
            else if (lidarType == AVIA)
            {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn]++;
            }

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            float range = pointDistance(thisPoint);

            if (range < 1.0)
                continue;
            // ??????????????????????????????????????????rangeMat????????????FLT_MAX
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)  // FLOAT_MAX
                continue;

            // for the amsterdam dataset
            // if (range < 6.0 && rowIdn <= 7 && (columnIdn >= 1600 || columnIdn <= 200))
            //     continue;
            // if (thisPoint.z < -2.0)
            //     continue;

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            // 3.1 ??????????????????
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);  // livox
            float s = float(laserCloudIn->points[i].time / (float)time_end);
            thisPoint.curvature = laserCloudIn->points[i].ring + s * 0.1; // integer part: line number; decimal part: timestamp
            // // Ouster

            int index                = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    // ??????????????????
    // ???????????????projectPointCloud???????????????????????????rangeMat????????????????????????
    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;  //???????????????????????????

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i, j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;  //???????????????????????????
        }
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed =
            publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "imu_link");
        pubLaserCloudInfo.publish(cloudInfo); // TODO: ??? ??????cloudInfo????????????????????????????????????
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar");

    ImageProjection IP;

    ROS_INFO("\033[1;32m----> Lidar Cloud Deskew Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}