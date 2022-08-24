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

    ros::Publisher pub_view;
    ros::Publisher pub_surf;
    ros::Publisher pub_edge;
    ros::Publisher pubLaserCloudInfo;

    double last_view_cmd = 0.0;
    int last_cmd = 0;
    int pre_num = 0;

    pcl::PointCloud<PointXYZINormal> lidar_cloud_in;
//    pcl::PointCloud<PointXYZINormal> lidar_cloud_in;
    std_msgs::Header cloud_header;

    vector<sensor_msgs::ImuConstPtr> imu_buf;
    vector<geometry_msgs::TwistConstPtr> cmd_vel_buf;
    double cmd_angular = 0;
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

        if (!getParameter("/edge_tracker/frame_id", frame_id))
        {
            ROS_WARN("frame_id not set, use default value: lili_om");
            frame_id = "lili_om";
        }

        sub_Lidar_cloud = nh.subscribe<edge_tracker::cloud_info>("/edge_tracker/lidar/deskew/cloud_info", 1000,
                                                            &Preprocessing::cloudHandler, this,
                                                            ros::TransportHints().tcpNoDelay());
        sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 20000, &Preprocessing::velHandler, this);

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
        x = pt1.x + pt2.x + pt3.x + pt4.x + pt5.x + pt6.x + pt7.x + pt8.x - 8 * pt.x;
        y = pt1.y + pt2.y + pt3.y + pt4.y + pt5.y + pt6.y + pt7.y + pt8.y - 8 * pt.y;
        z = pt1.z + pt2.z + pt3.z + pt4.z + pt5.z + pt6.z + pt7.z + pt8.z - 8 * pt.z;
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
        double time_msgsin = current_cloud_msg.header.stamp.toSec();

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

        for(int i = 5; i < H_SCANS - 12; i = i + 6) {
            vector<Eigen::Vector3d> near_pts;
            Eigen::Vector3d center(0, 0, 0);
            int num = 36;
            for(int j = 0; j < 6; j++) {
                for(int k = 0; k < N_SCANS; k++) {
                    if(mat[k][i+j].intensity <= 0) {
                        num--;
                        continue;
                    }
                    Eigen::Vector3d pt(mat[k][i+j].x,
                            mat[k][i+j].y,
                            mat[k][i+j].z);
                    center += pt;
                    near_pts.push_back(pt);
                }
            }
            if(num < 25)
                continue;
            center /= num;
            // Covariance matrix
            vector<int> idsx_edge;
            vector<int> idsy_edge;
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
                    double g1 = getRelativeDepth(mat[k][i+j], mat[k][i+j-4], mat[k][i+j-3], mat[k][i+j-2], mat[k][i+j-1],
                                                 mat[k][i+j+1], mat[k][i+j+2], mat[k][i+j+3], mat[k][i+j+4]);
                    g1 = g1 / (8 * getDepth(mat[k][i+j]) + 1e-3);

                    if(g1 > 0.015) {
                        if(g1 > max_s) {
                            max_s = g1;
                            idx = i+j;
                        }
                    } else if(g1 < -0.01) {
                        if(g1 < max_s1) {
                            max_s1 = g1;
                            idx = i+j;
                        }
                    }
                }
                if(max_s != 0 || max_s1 != 0) {
                    idsx_edge.push_back(k);
                    idsy_edge.push_back(idx);
                }
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

            if(eigen_solver_edge.eigenvalues()[2] > edge_thres * eigen_solver_edge.eigenvalues()[1] && idsx_edge.size() > 3) {
                Eigen::Vector3d unitDirection = eigen_solver_edge.eigenvectors().col(2);
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
            }
        }

        num_left += 20 * (4 - abs(2 - last_cmd));
        num_medleft += 20 * (4 - abs(1 - last_cmd));
        num_med += 20 * (4 - abs(0 - last_cmd));
        num_medright += 20 * (4 - abs(1 + last_cmd));
        num_right += 20 * (4 - abs(2 + last_cmd));
        ROS_INFO("left: %d, medleft: %d, med: %d, medright: %d, right: %d ", num_left, num_medleft, num_med, num_medright, num_right);

        if (abs(0.52 + last_view_cmd) > 2) num_left = -1;
        if (abs(0.17 + last_view_cmd) > 2) num_medleft = -1;
        if (abs(-0.17 + last_view_cmd) > 2) num_medright = -1;
        if (abs(-0.52 + last_view_cmd) > 2) num_right = -1;
        num_max = num_left;
        curr_cmd = 2;
        view_cmd = 0.52;
        if (num_max < num_medleft) {
            num_max = num_medleft;
            curr_cmd = 1;
            view_cmd = 0.17;
        }
        if (num_max < num_med) {
            num_max = num_med;
            curr_cmd = 0;
            view_cmd = 0.0;
        }
        if (num_max < num_medright) {
            num_max = num_medright;
            curr_cmd = -1;
            view_cmd = -0.17;
        }
        if (num_max < num_right) {
            num_max = num_right;
            curr_cmd = -2;
            view_cmd = -0.52;
        }

        std_msgs::Float64 yaw_angle;
        yaw_angle.data = std::max(-2.0, last_view_cmd + (view_cmd - cmd_angular * (ros::Time::now().toSec() - time_msgsin)) * 0.2);
        yaw_angle.data = std::min(2.0, yaw_angle.data);
        pub_view.publish(yaw_angle);
        last_view_cmd = yaw_angle.data;
        last_cmd = curr_cmd;

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
