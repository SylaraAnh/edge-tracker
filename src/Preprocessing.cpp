#include "common.h"
#include "timer.h"
#include "math_tools.h"
#include "edge_tracker/cloud_info.h"
#include <std_msgs/Float64.h>

class Preprocessing {
private:
    ros::NodeHandle nh;

    ros::Subscriber sub_Lidar_cloud;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_yaw_statues;

    ros::Publisher pub_view;
    ros::Publisher pub_surf;
    ros::Publisher pub_edge;
    ros::Publisher pubLaserCloudInfo;

    double last_view_cmd = 0.0;
    int last_cmd = 0;
    int pre_num = 0;
    int firstflag = 1;

    pcl::PointCloud<PointXYZINormal> lidar_cloud_in;
//    pcl::PointCloud<PointXYZINormal> lidar_cloud_in;
    std_msgs::Header cloud_header;

    vector<sensor_msgs::ImuConstPtr> imu_buf;
    vector<geometry_msgs::TwistConstPtr> cmd_vel_buf;
    double cmd_angular = 0;
    double fb_angular = 0;
    int idx_imu = 0;
    double current_time_imu = -1;

    Eigen::Vector3d gyr_0;
    Eigen::Quaterniond q_iMU = Eigen::Quaterniond::Identity();
    bool first_imu = false;

    std::deque<edge_tracker::cloud_info> cloud_queue; // editted by wjc
    edge_tracker::cloud_info current_cloud_msg; // editted by wjc
    double time_scan_next;

    int N_SCANS = 6;
    int H_SCANS = 4000;

    string frame_id = "lili_om";
    string imu_frame_id = "imu_link";
    double edge_thres, surf_thres;
    double edge_curthres;
    double angle_max, angle_min;
    double runtime = 0;

public:
    Preprocessing(): nh("~") {

        if (!getParameter("/edge_tracker/surf_thres", surf_thres))
        {
            ROS_WARN("surf_thres not set, use default value: 0.2");
            surf_thres = 0.28;
        }

        if (!getParameter("/edge_tracker/edge_thres", edge_thres))
        {
            ROS_WARN("edge_thres not set, use default value: 4.0");
            edge_thres = 4.0;
        }
        if (!getParameter("/edge_tracker/edge_curthres", edge_curthres))
        {
            ROS_WARN("edge_curthres not set, use default value: 0.02");
            edge_curthres = 0.02;
        }
        if (!getParameter("/edge_tracker/angle_min", angle_min))
        {
            ROS_WARN("angle_min not set, use default value: -0.78");
            angle_min = -0.78;
        }

        if (!getParameter("/edge_tracker/angle_max", angle_max))
        {
            ROS_WARN("angle_max not set, use default value: 0.78");
            angle_max = 0.78;
        }

        if (!getParameter("/edge_tracker/frame_id", frame_id))
        {
            ROS_WARN("frame_id not set, use default value: lili_om");
            frame_id = "lili_om";
        }

        sub_Lidar_cloud = nh.subscribe<edge_tracker::cloud_info>("/edge_tracker/lidar/deskew/cloud_info", 1000,
                                                            &Preprocessing::cloudHandler, this,
                                                            ros::TransportHints().tcpNoDelay());
        sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 20000, &Preprocessing::velHandler, this);
        sub_yaw_statues = nh.subscribe<control_msgs::JointControllerState>("/gimbal_lidar/yaw_joint_position_controller/state", 20000, &Preprocessing::control_msgsHandler, this);

        pub_view = nh.advertise<std_msgs::Float64>("/gimbal_lidar/yaw_joint_position_controller/command", 100);
        pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/edge_tracker/lidar/feature/cloud_surface", 100);
        pub_edge = nh.advertise<sensor_msgs::PointCloud2>("/edge_tracker/lidar/feature/cloud_corner", 100);
        pubLaserCloudInfo = nh.advertise<edge_tracker::cloud_info>("/edge_tracker/lidar/feature/cloud_info", 5);
    }

    ~Preprocessing(){}

    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float thres) {

        if (&cloud_in != &cloud_out) {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i) {
            if (cloud_in.points[i].x * cloud_in.points[i].x +
                    cloud_in.points[i].y * cloud_in.points[i].y +
                    cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size()) {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    template <typename PointT>
    double getDepth(PointT pt) {
        return sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
    }

    template <typename PointT>
    void checkIntensity(PointT pt, PointT &pt1) {
        if (pt1.intensity == 0) pt1 = pt;
    }

    template <typename PointT>
    double getRelativeDepth(PointT pt, PointT pt1, PointT pt2, PointT pt3, PointT pt4, PointT pt5, PointT pt6, PointT pt7, PointT pt8) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        checkIntensity(pt, pt1);
        checkIntensity(pt, pt2);
        checkIntensity(pt, pt3);
        checkIntensity(pt, pt4);
        checkIntensity(pt, pt5);
        checkIntensity(pt, pt6);
        checkIntensity(pt, pt7);
        checkIntensity(pt, pt8);
        x = pt1.x + 2*pt2.x + 3*pt3.x + 4*pt4.x + 4*pt5.x + 3*pt6.x + 2*pt7.x + pt8.x - 20 * pt.x;
        y = pt1.y + 2*pt2.y + 3*pt3.y + 4*pt4.y + 4*pt5.y + 3*pt6.y + 2*pt7.y + pt8.y - 20 * pt.y;
        z = pt1.z + 2*pt2.z + 3*pt3.z + 4*pt4.z + 4*pt5.z + 3*pt6.z + 2*pt7.z + pt8.z - 20 * pt.z;
//        x = pt1.x + 2*pt2.x + 3*pt3.x + 4*pt4.x + 4*pt5.x + 3*pt6.x + 2*pt7.x + pt8.x - 20 * pt.x;
//        y = pt1.y + 2*pt2.y + 3*pt3.y + 4*pt4.y + 4*pt5.y + 3*pt6.y + 2*pt7.y + pt8.y - 20 * pt.y;
//        z = pt1.z + 2*pt2.z + 3*pt3.z + 4*pt4.z + 4*pt5.z + 3*pt6.z + 2*pt7.z + pt8.z - 20 * pt.z;
        return sqrt(x*x + y*y + z*z);
    }

    void solveRotation(double dt, Eigen::Vector3d angular_velocity) {
        Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity);
        q_iMU *= deltaQ(un_gyr * dt);
        gyr_0 = angular_velocity;
    }

    void velHandler(const geometry_msgs::TwistConstPtr& cmd_vel_in) {
        cmd_angular = cmd_vel_in->angular.z;
    }

    void control_msgsHandler(const control_msgs::JointControllerStateConstPtr& control_msgs_in) {
        fb_angular = control_msgs_in->process_value;
    }

    void imuHandler(const sensor_msgs::ImuConstPtr& imu_in) {
        imu_buf.push_back(imu_in);

        if(imu_buf.size() > 600)
            imu_buf[imu_buf.size() - 601] = nullptr;

        if (current_time_imu < 0)
            current_time_imu = imu_in->header.stamp.toSec();

        if (!first_imu)
        {
            first_imu = true;
            double rx = 0, ry = 0, rz = 0;
            rx = imu_in->angular_velocity.x;
            ry = imu_in->angular_velocity.y;
            rz = imu_in->angular_velocity.z;
            Eigen::Vector3d angular_velocity(rx, ry, rz);
            gyr_0 = angular_velocity;
        }
    }

    void cloudHandler( const edge_tracker::cloud_infoConstPtr &lidar_cloud_msg) {

        current_cloud_msg = *lidar_cloud_msg;

        cloud_header = current_cloud_msg.header;
        cloud_header.frame_id = current_cloud_msg.header.frame_id;
        int tmp_idx = 0;
        int num_left = 0;
        int num_medleft = 0;
        int num_med = 0;
         int num_medright = 0;
        int num_right = 0;
        int num_max = 0;
        int curr_cmd = 0;
        double view_cmd = 0.0;
        double view_left = 0.57;
        double view_med = 0.17;
        double view_right = -0.57;
        double time_msgsin = ros::Time::now().toSec();
        double time_msgsout = ros::Time::now().toSec();
//        if (firstflag == 1 && cloud_header.stamp.toSec() == 1627807320.900339365)
//        if (firstflag == 1 && cloud_header.stamp.toSec() == 1627807363.300291061)
//        {
//            firstflag = 0;
//        }
//        else{
//            return;
//        }

        Timer t_pre("Preprocessing");
//        pcl::fromROSMsg(current_cloud_msg.cloud_deskewed, lidar_cloud_in);
        pcl::fromROSMsg(current_cloud_msg.cloud_deskewed, lidar_cloud_in);

        std::vector<int> indices;

        pcl::removeNaNFromPointCloud(lidar_cloud_in, lidar_cloud_in, indices); //todo failed
        removeClosedPointCloud(lidar_cloud_in, lidar_cloud_in, 0.1);

        int cloud_size = lidar_cloud_in.points.size();


        PointXYZINormal point;
        PointXYZINormal point_undis;
        PointXYZINormal mat[N_SCANS][H_SCANS];
        double t_interval = 0.1 / (H_SCANS-1);
        pcl::PointCloud<PointXYZINormal>::Ptr surf_features(new pcl::PointCloud<PointXYZINormal>());
        pcl::PointCloud<PointXYZINormal>::Ptr edge_features(new pcl::PointCloud<PointXYZINormal>());

        for (int i = 0; i < cloud_size; i++) {
            point.x = lidar_cloud_in.points[i].x;
            point.y = lidar_cloud_in.points[i].y;
            point.z = lidar_cloud_in.points[i].z;
            point.intensity = lidar_cloud_in.points[i].intensity;
            point.curvature = lidar_cloud_in.points[i].curvature;

            int scan_id = 0;
            if (N_SCANS == 6)
                scan_id = (int)point.curvature; // editted by wjc
            if(scan_id < 0)
                continue;

            double dep = point.x*point.x + point.y*point.y + point.z*point.z; // editted by wjc
            if(dep > 40000.0 || dep < 4.0 || point.intensity < 0.5 || point.intensity > 254.5)  //editted by wjc
                continue;
            int col = int(round((point.curvature - scan_id) / t_interval));  // editted by wjc
            if (col >= H_SCANS || col < 0)
                continue;
            if (mat[scan_id][col].intensity != 0)
                continue;
            mat[scan_id][col] = point;
        }

        for(int i = 5; i < H_SCANS - 6; i++)
        {
            for (int k = 0; k < N_SCANS; k++) {
                double g1 = getRelativeDepth(mat[k][i], mat[k][i-4], mat[k][i-3], mat[k][i-2], mat[k][i-1],
                                             mat[k][i+1], mat[k][i+2], mat[k][i+3], mat[k][i+4]);
                g1 = g1 / (20 * getDepth(mat[k][i]) + 1e-3);
                mat[k][i].curvature = g1;
            }
        }

        for(int i = 5; i < H_SCANS - 12; i = i + 6) {
            int num = 36;
            vector<PointXYZINormal> surf_temp1;
            vector<PointXYZINormal> surf_temp2;
//            surf_features->points.clear(); // debug
            for(int j = 0; j < 6; j++) {
                for(int k = 0; k < N_SCANS; k++) {
                    if(mat[k][i+j].intensity <= 0) {
                        num--;
                        continue;
                    }
                    if (mat[k][i+j].x < 2)
                    {
                        mat[k][i+j].intensity = -1;
                        num--;
                        continue;
                    }
                    mat[k][i+j].intensity = k + 1;
                    Eigen::Vector3d pt(mat[k][i+j].x,
                            mat[k][i+j].y,
                            mat[k][i+j].z);
//                    surf_features->points.push_back(mat[k][i+j]); // debug
                }
            }
            // debug
//            sensor_msgs::PointCloud2 surf_features_msg;
//            pcl::toROSMsg(*surf_features, surf_features_msg);
//            surf_features_msg.header.stamp = cloud_header.stamp;
//            surf_features_msg.header.frame_id = cloud_header.frame_id;
//            if (pub_surf.getNumSubscribers() != 0)
//                pub_surf.publish(surf_features_msg);

            if(num < 20)
                continue;
            //
            vector<int> idsx_edge;
            vector<int> idsy_edge;
            int num_edge = 0;
            for(int k = 0; k < N_SCANS; k++) {
                double max_s = 0;
                double max_s1 = 0;
                int idx = i;
                for(int j = 0; j < 6; j++) {
                    if(mat[k][i+j].intensity <= 0) {
                        continue;
                    }
//                    double g1 = getDepth(mat[k][i+j-4]) + getDepth(mat[k][i+j-3]) +
//                                getDepth(mat[k][i+j-2]) + getDepth(mat[k][i+j-1]) - 8*getDepth(mat[k][i+j]) +
//                                getDepth(mat[k][i+j+1]) + getDepth(mat[k][i+j+2]) + getDepth(mat[k][i+j+3]) +
//                                getDepth(mat[k][i+j+4]);
                    double g1 = mat[k][i+j].curvature;

                    if(g1 > edge_curthres) {
                        if(g1 > max_s) {
                            max_s = g1;
                            idx = i+j;
                        }
                    }
                }
                if(max_s != 0) {
                    idsx_edge.push_back(k);
                    idsy_edge.push_back(idx);
                }
                if (idx == i && idx == i+5)
                {
                    num_edge++;
                }
            }
            if (idsx_edge.size() == 0)
            {
                for(int k = 0; k < N_SCANS; k++) {
                    double max_s = 0;
                    double max_s1 = 0;
                    int idx = i;
                    for(int j = 0; j < 6; j++) {
                        surf_features->points.push_back(mat[k][i+j]);
                    }
                }
            }
            if (num_edge > idsx_edge.size() / 2)
            {
                idsx_edge.clear();
                continue;
            }
            vector<Eigen::Vector3d> near_pts_edge;
            Eigen::Vector3d center_edge(0, 0, 0);
            for(int j = 0; j < idsx_edge.size(); j++) {
                Eigen::Vector3d pt(mat[idsx_edge[j]][idsy_edge[j]].x,
                        mat[idsx_edge[j]][idsy_edge[j]].y,
                        mat[idsx_edge[j]][idsy_edge[j]].z);
                center_edge += pt;
                near_pts_edge.push_back(pt);
            }
            center_edge /= idsx_edge.size();
            // Covariance matrix
            Eigen::Matrix3d matA_edge = Eigen::Matrix3d::Zero();
            for (int j = 0; j < near_pts_edge.size(); j++)
            {
                Eigen::Vector3d zero_mean = near_pts_edge[j] - center_edge;
                matA_edge += (zero_mean * zero_mean.transpose());
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver_edge(matA_edge);
            //???????????????
            if(eigen_solver_edge.eigenvalues()[2] > edge_thres * eigen_solver_edge.eigenvalues()[1] && idsx_edge.size() > 4) {
                Eigen::Matrix3d matA1 = Eigen::Matrix3d::Zero();
                Eigen::Matrix3d matA2 = Eigen::Matrix3d::Zero();
                vector<Eigen::Vector3d> near_pts1;
                vector<Eigen::Vector3d> near_pts2;
                Eigen::Vector3d center1(0, 0, 0);
                Eigen::Vector3d center2(0, 0, 0);
                for(int k = 0; k < idsx_edge.size(); k++) {
                    for(int j = idsy_edge[k] - i - 4; j <= idsy_edge[k] - i + 4; j++) {
                        if(mat[idsx_edge[k]][idsy_edge[k]].intensity <= 0) {
                            continue;
                        }
                        if (i + j < idsy_edge[k]) //  && mat[idsx_edge[k]][idsy_edge[k] + 1].curvature > mat[idsx_edge[k]][idsy_edge[k]].curvature
                        {
                            Eigen::Vector3d pt(mat[k][i+j].x, mat[k][i+j].y, mat[k][i+j].z);
                            center1 += pt;
                            near_pts1.push_back(pt);
                            if(j >= 0)
                            {
                                surf_temp1.push_back(mat[k][i+j]);
                            }
                        }
                        else if (i + j > idsy_edge[k]) // && mat[idsx_edge[k]][idsy_edge[k] - 1].curvature > mat[idsx_edge[k]][idsy_edge[k]].curvature
                        {
                            Eigen::Vector3d pt(mat[k][i+j].x, mat[k][i+j].y, mat[k][i+j].z);
                            center2 += pt;
                            near_pts2.push_back(pt);
                            if(j < 6)
                            {
                                surf_temp2.push_back(mat[k][i+j]);
                            }
                        }
                    }
                }
                center1 /= near_pts1.size();
                center2 /= near_pts2.size();
                for (int j = 0; j < near_pts1.size(); j++)
                {
                    Eigen::Vector3d zero_mean = near_pts1[j] - center1;
                    matA1 += (zero_mean * zero_mean.transpose());
                }
                for (int j = 0; j < near_pts2.size(); j++)
                {
                    Eigen::Vector3d zero_mean = near_pts2[j] - center2;
                    matA2 += (zero_mean * zero_mean.transpose());
                }

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver1(matA1);
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver2(matA2);

                if(eigen_solver1.eigenvalues()[0] < surf_thres * eigen_solver1.eigenvalues()[1] &&
                eigen_solver1.eigenvalues()[1] * eigen_solver1.eigenvalues()[1] > surf_thres * eigen_solver1.eigenvalues()[2] * eigen_solver1.eigenvalues()[0] &&
                eigen_solver2.eigenvalues()[1] * eigen_solver2.eigenvalues()[1] > surf_thres * eigen_solver2.eigenvalues()[2] * eigen_solver2.eigenvalues()[0] &&
                eigen_solver2.eigenvalues()[0] < surf_thres * eigen_solver2.eigenvalues()[1]) {
                    Eigen::Vector3d unitDirection1 = eigen_solver1.eigenvectors().col(0);
                    Eigen::Vector3d unitDirection2 = eigen_solver2.eigenvectors().col(0);
                    Eigen::Vector3d unitDirection = eigen_solver_edge.eigenvectors().col(2);

//                    double flag1 = (unitDirection1.cross(unitDirection2)).norm();
//                    double flag2 = abs((center1 - center2).dot(unitDirection1));
//                    double flag3 = (center1 - center2).norm();
//                    if(flag1 >= 0.2 ||
//                    (flag3 > 0.1 && flag2 > 0.1 * flag3)) // || (center1 - center2).dot(unitDirection1) > 0.2 // || (center1 - center2).dot(unitDirection) < 0.2 * (center1 - center2).norm()
                    double flag1 = abs(unitDirection1.dot(unitDirection));
                    double flag2 = abs(unitDirection2.dot(unitDirection));
                    double flag3 = unitDirection2.cross(unitDirection1).norm();
                    double flag4 = abs(unitDirection1.dot(center1) / center1.norm());
                    double flag5 = abs(unitDirection2.dot(center2) / center2.norm());
                    if(flag1 < 0.2 && flag2 < 0.2 && flag3 > 0.1 && (flag4 > 0.05 || flag5 > 0.05))
                    {
                        for(int j = 0; j < idsx_edge.size(); j++) {
                            if(mat[idsx_edge[j]][idsy_edge[j]].intensity <= 0 && mat[idsx_edge[j]][idsy_edge[j]].curvature <= 0)
                                continue;
                            mat[idsx_edge[j]][idsy_edge[j]].normal_x = unitDirection.x();
                            mat[idsx_edge[j]][idsy_edge[j]].normal_y = unitDirection.y();
                            mat[idsx_edge[j]][idsy_edge[j]].normal_z = unitDirection.z();

                            edge_features->points.push_back(mat[idsx_edge[j]][idsy_edge[j]]);
                            mat[idsx_edge[j]][idsy_edge[j]].intensity *= -1;
                        }
                        double tan_theta = center_edge[1] / center_edge[0];
                        if (tan_theta > view_left)
                        {
                            num_left += idsx_edge.size();
                        }
                        else if (tan_theta > view_med)
                        {
                            num_medleft += idsx_edge.size();
                        }
                        else if (tan_theta > -view_med)
                        {
                            num_med += idsx_edge.size();
                        }
                        else if (tan_theta > view_right)
                        {
                            num_medright += idsx_edge.size();
                        }
                        else
                        {
                            num_right += idsx_edge.size();
                        }
                        surf_features->points.insert(surf_features->points.end(),surf_temp1.begin(),surf_temp1.end());
                        surf_features->points.insert(surf_features->points.end(),surf_temp2.begin(),surf_temp2.end());
                    }
                }

            }
        }
        time_msgsout = ros::Time::now().toSec();
//        num_left += 20 * (4 - abs(2 - last_cmd));
//        num_medleft += 20 * (4 - abs(1 - last_cmd));
//        num_med += 20 * (4 - abs(0 - last_cmd));
//        num_medright += 20 * (4 - abs(1 + last_cmd));
//        num_right += 20 * (4 - abs(2 + last_cmd));
        ROS_INFO("time: %.8f, left: %d, medleft: %d, med: %d, medright: %d, right: %d ", time_msgsout - time_msgsin, num_left, num_medleft, num_med, num_medright, num_right);

        if (abs(0.52 + fb_angular) > angle_max) num_left = -1;
        if (abs(0.17 + fb_angular) > angle_max) num_medleft = -1;
        if (abs(-0.17 + fb_angular) > angle_max) num_medright = -1;
        if (abs(-0.52 + fb_angular) > angle_max) num_right = -1;
        num_max = num_med;
        curr_cmd = 0;
        view_cmd = 0.0;
        if (20 < (num_left - num_right) * 2 + (num_medleft - num_medright)) {
            num_max = num_medleft;
            curr_cmd = 1;
            view_cmd = 0.17;
        }
        if (30 < (num_left - num_right) * 2 + (num_medleft - num_medright)) {
            num_max = num_medleft;
            curr_cmd = 2;
            view_cmd = 0.52;
        }
        if (-20 > (num_left - num_right) * 2 + (num_medleft - num_medright)) {
            num_max = num_medleft;
            curr_cmd = -1;
            view_cmd = -0.17;
        }
        if (-30 > (num_left - num_right) * 2 + (num_medleft - num_medright)) {
            num_max = num_medleft;
            curr_cmd = -2;
            view_cmd = -0.52;
        }

//        num_max = num_left;
//        curr_cmd = 2;
//        view_cmd = 0.52;
//        if (num_max < num_medleft) {
//            num_max = num_medleft;
//            curr_cmd = 1;
//            view_cmd = 0.17;
//        }
//        if (num_max < num_med) {
//            num_max = num_med;
//            curr_cmd = 0;
//            view_cmd = 0.0;
//        }
//        if (num_max < num_medright) {
//            num_max = num_medright;
//            curr_cmd = -1;
//            view_cmd = -0.17;
//        }
//        if (num_max < num_right) {
//            num_max = num_right;
//            curr_cmd = -2;
//            view_cmd = -0.52;
//        }

        std_msgs::Float64 yaw_angle;
        yaw_angle.data = std::max(angle_min, 0.98 * fb_angular + (view_cmd) * 0.6 - cmd_angular * (0.5));
        yaw_angle.data = std::min(angle_max, yaw_angle.data);
        pub_view.publish(yaw_angle);
        last_view_cmd = yaw_angle.data;
        last_cmd = curr_cmd;

        sensor_msgs::PointCloud2 surf_features_msg;
        pcl::toROSMsg(*surf_features, surf_features_msg);
        surf_features_msg.header.stamp = cloud_header.stamp;
        surf_features_msg.header.frame_id = cloud_header.frame_id;
        if (pub_surf.getNumSubscribers() != 0)
            pub_surf.publish(surf_features_msg);

        sensor_msgs::PointCloud2 edge_features_msg;
        pcl::toROSMsg(*edge_features, edge_features_msg);
        edge_features_msg.header.stamp = cloud_header.stamp;
        edge_features_msg.header.frame_id = cloud_header.frame_id;
        if (pub_edge.getNumSubscribers() != 0)
            pub_edge.publish(edge_features_msg);
        current_cloud_msg.cloud_corner = edge_features_msg;
        // publish to mapOptimization
        pubLaserCloudInfo.publish(current_cloud_msg);
        
        q_iMU = Eigen::Quaterniond::Identity();
        //t_pre.tic_toc();
        runtime += t_pre.toc();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lili_feature_processing");

    Preprocessing Pre;

    ROS_INFO("\033[1;32m---->\033[0m Preprocessing Started.");
    ros::spin();
    return 0;
}
