#include "imls_icp.h"
#include "pl_icp.h"
#include <csm/csm_all.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <chrono>

// pcl::visualization::CloudViewer g_cloudViewer("cloud_viewer");
//此处bag包的地址需要自行修改

std::string bagfile = "src/bag/imls_icp.bag";

class imlsDebug {
public:
    imlsDebug()
    {
        odom_to_base = Eigen::Matrix3d::Identity();
        odom_to_base(0, 2) = 0.315;
        odom_to_base(1, 2) = 0.00185;
        Eigen::Quaterniond q(0, 0, 0, 1);

        odom_to_base(0, 0) = cos(M_PI);
        odom_to_base(1, 1) = cos(M_PI);
        odom_to_base(0, 1) = -sin(M_PI);
        odom_to_base(1, 0) = sin(M_PI);

        m_imlsPathPub = m_node.advertise<nav_msgs::Path>("imls_path_pub_", 1, true);
        m_imlsPath.header.stamp = ros::Time::now();
        m_imlsPath.header.frame_id = "odom";
        m_plicpPathPub = m_node.advertise<nav_msgs::Path>("plicp_path_pub_", 1, true);
        m_plicpPath.header.stamp = ros::Time::now();
        m_plicpPath.header.frame_id = "odom";
        m_odomPathPub = m_node.advertise<nav_msgs::Path>("odom_path_pub_", 1, true);
        m_odomPath.header.stamp = ros::Time::now();
        m_odomPath.header.frame_id = "odom";

        m_isFirstFrame = true;

        rosbag::Bag bag;
        bag.open(bagfile, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string("/sick_scan"));
        topics.push_back(std::string("/odom"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        //按顺序读取bag内激光的消息和里程计的消息
        BOOST_FOREACH (rosbag::MessageInstance const m, view)
        {
            champion_nav_msgs::ChampionNavLaserScanConstPtr scan =
                m.instantiate<champion_nav_msgs::ChampionNavLaserScan>();
            if (scan != NULL) championLaserScanCallback(scan);

            nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
            if (odom != NULL) odomCallback(odom);

            ros::spinOnce();
            if (!ros::ok()) break;
        }
        // m_laserscanSub = m_nh.subscribe("sick_scan",5,&imlsDebug::championLaserScanCallback,this);
    }

    //将激光消息转换为激光坐标系下的二维点云
    void ConvertChampionLaserScanToEigenPointCloud(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg,
                                                   std::vector<Eigen::Vector2d>&                          eigen_pts)
    {
        eigen_pts.clear();
        for (int i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) continue;

            double lx = msg->ranges[i] * std::cos(msg->angles[i]);
            double ly = msg->ranges[i] * std::sin(msg->angles[i]);

            if (std::isnan(lx) || std::isinf(ly) || std::isnan(ly) || std::isinf(ly)) continue;

            eigen_pts.push_back(Eigen::Vector2d(lx, ly));
        }
    }

    void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
    {
        // 计算里程计增量
        Eigen::Matrix3d trans;
        Eigen::Vector3d delta_pose = Eigen::Vector3d::Zero();
        Eigen::Vector3d delta_pose_inv;
        base_pose = odom_to_base.inverse() * odom_pose;
        trans << cos(base_pose[2]), -sin(base_pose[2]), 0, sin(base_pose[2]), cos(base_pose[2]), 0, 0, 0, 1;
        delta_pose = trans.inverse() * (base_pose - base_pose_prev);
        base_pose_prev = base_pose;
        delta_pose_inv = odom_to_base.inverse() * delta_pose;

        // IMLS-ICP
        std::cout << "start imls matching!" << std::endl;
        auto            t1 = std::chrono::system_clock::now();
        Eigen::Matrix3d rPose, rCovariance;
        if (m_isFirstFrame == true)
        {
            std::cout << "First Frame" << std::endl;
            m_isFirstFrame = false;
            m_prevLaserPose = Eigen::Vector3d(0, 0, 0);
            pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
            ConvertChampionLaserScanToEigenPointCloud(msg, m_prevPointCloud);
            // return;
        } else
        {
            std::vector<Eigen::Vector2d> nowPts;
            ConvertChampionLaserScanToEigenPointCloud(msg, nowPts);
            //调用imls进行icp匹配，并输出结果．
            m_imlsMatcher.setSourcePointCloud(nowPts);
            m_imlsMatcher.setTargetPointCloud(m_prevPointCloud);

            if (m_imlsMatcher.Match(rPose, rCovariance))
            {
                std::cout << "IMLS Match Successful:" << rPose(0, 2) << "," << rPose(1, 2) << ","
                          << atan2(rPose(1, 0), rPose(0, 0)) * 180.0 / M_PI << std::endl;
                Eigen::Matrix3d lastPose;
                lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0),
                    sin(m_prevLaserPose(2)), cos(m_prevLaserPose(2)), m_prevLaserPose(1), 0, 0, 1;
                Eigen::Matrix3d nowPose = lastPose * rPose;
                m_prevLaserPose = Eigen::Vector3d(nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1, 0), nowPose(0, 0)));
                pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
            } else
            {
                std::cout << "IMLS Match Failed!!!!" << std::endl;
            }

            auto t2 = std::chrono::system_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
            std::cout << "IMLS match time: " << ms.count() << " ms\n";
            std::cout << "========================" << std::endl;
            m_prevPointCloud = nowPts;
        }
        // IMLS-ICP finished

        // PL-ICP
        auto            t3 = std::chrono::system_clock::now();
        Eigen::Vector3d d_point_odom = Eigen::Vector3d::Zero(); //里程计计算的dpose
        Eigen::Vector3d d_point_scan = Eigen::Vector3d::Zero(); //激光的scanmatch计算的dpose
        Eigen::MatrixXd transform_matrix(3, 3);                 //临时的变量
        LDP             currentLDP;

        if (m_plicpMatcher.m_isFirstFrame == false)
        {
            m_plicpMatcher.LaserScanToLDP(msg, currentLDP);
            auto start = std::chrono::system_clock::now();
            if (std::abs(delta_pose[0]) > 0.2)
            {
                delta_pose = Eigen::Vector3d::Zero();
            }
            d_point_scan = m_plicpMatcher.PIICPBetweenTwoFrames(currentLDP, delta_pose);
            auto now = std::chrono::system_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            std::cout << "PLICP Match Successful:" << d_point_scan(0) << "," << d_point_scan(1) << ","
                      << d_point_scan(2) * 180.0 / M_PI << std::endl;
            std::cout << "ICP time: " << timestamp.count() << std::endl;
        } else
        {
            m_plicpMatcher.m_isFirstFrame = false;
            // 注意：需要初始化d_point_scan，否则初值可能出现无限大
            std::cout << "First Frame" << std::endl;
            d_point_scan = Eigen::Vector3d::Zero();
            m_plicpMatcher.LaserScanToLDP(msg, m_plicpMatcher.m_prevLDP);
        }

        // 两针scan计算本身累计的位姿 for laser_path visualization
        double c, s;
        c = cos(m_plicpMatcher.scan_pos_cal(2));
        s = sin(m_plicpMatcher.scan_pos_cal(2));
        transform_matrix = Eigen::Matrix3d::Identity();
        transform_matrix << c, -s, 0, s, c, 0, 0, 0, 1;
        Eigen::Matrix3d d_point_scan_mat = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d scan_pos_cal_prev_mat = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d scan_pos_cal_cur_mat = Eigen::Matrix3d::Identity();
        // 用矩阵代替向量计算积分位姿
        scan_pos_cal_prev_mat(0, 2) = m_plicpMatcher.scan_pos_cal[0];
        scan_pos_cal_prev_mat(1, 2) = m_plicpMatcher.scan_pos_cal[1];
        scan_pos_cal_prev_mat(0, 0) = cos(m_plicpMatcher.scan_pos_cal[2]);
        scan_pos_cal_prev_mat(1, 1) = cos(m_plicpMatcher.scan_pos_cal[2]);
        scan_pos_cal_prev_mat(0, 1) = -sin(m_plicpMatcher.scan_pos_cal[2]);
        scan_pos_cal_prev_mat(1, 0) = sin(m_plicpMatcher.scan_pos_cal[2]);
        d_point_scan_mat(0, 2) = d_point_scan[0];
        d_point_scan_mat(1, 2) = d_point_scan[1];
        d_point_scan_mat(0, 0) = cos(d_point_scan[2]);
        d_point_scan_mat(1, 1) = cos(d_point_scan[2]);
        d_point_scan_mat(0, 1) = -sin(d_point_scan[2]);
        d_point_scan_mat(1, 0) = sin(d_point_scan[2]);
        scan_pos_cal_cur_mat = scan_pos_cal_prev_mat * d_point_scan_mat;
        m_plicpMatcher.scan_pos_cal[0] = scan_pos_cal_cur_mat(0, 2);
        m_plicpMatcher.scan_pos_cal[1] = scan_pos_cal_cur_mat(1, 2);
        m_plicpMatcher.scan_pos_cal[2] = atan2(scan_pos_cal_cur_mat(1, 0), scan_pos_cal_cur_mat(0, 0));
        // m_plicpMatcher.scan_pos_cal += (transform_matrix * d_point_scan);

        //放到路径当中 //for visualization
        pubPath(m_plicpMatcher.scan_pos_cal, m_plicpPath, m_plicpPathPub);

        auto t4 = std::chrono::system_clock::now();
        auto ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3);
        std::cout << "PLICP match time: " << ms2.count() << " ms\n";
        std::cout << "========================" << std::endl;
        // PL-ICP finished
        // 计算误差
        std::cout << "Delta Odom :" << delta_pose(0) << "," << delta_pose(1) << "," << delta_pose(2) * 180.0 / M_PI
                  << std::endl;
        std::cout << "diff IMLS  :" << delta_pose(0) - rPose(0, 2) << "," << delta_pose(0) - rPose(1, 2) << ","
                  << (delta_pose(2) - atan2(rPose(1, 0), rPose(0, 0))) * 180.0 / M_PI << std::endl;
        std::cout << "diff PLICP :" << delta_pose(0) - d_point_scan(0) << "," << delta_pose(1) - d_point_scan(1) << ","
                  << (delta_pose(2) - d_point_scan(2)) * 180.0 / M_PI << std::endl;
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        if (m_isFirstFrame == true)
        {
            return;
        }
        // 转换到laser坐标系下
        odom_pose[0] = msg->pose.pose.position.x;
        odom_pose[1] = msg->pose.pose.position.y;
        odom_pose[2] = tf::getYaw(msg->pose.pose.orientation);
        base_pose_tmp = odom_to_base.inverse() * odom_pose;
        // pubPath(odom_pose, m_odomPath, m_odomPathPub);
        pubPath(base_pose_tmp, m_odomPath, m_odomPathPub);
    }

    //发布路径消息
    void pubPath(const Eigen::Vector3d& pose, nav_msgs::Path& path, ros::Publisher& mcu_path_pub_)
    {
        ros::Time                  current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = pose(0);
        this_pose_stamped.pose.position.y = pose(1);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    void pubPath(const nav_msgs::OdometryConstPtr& msg, nav_msgs::Path& path, ros::Publisher& mcu_path_pub_)
    {
        ros::Time                  current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
        this_pose_stamped.pose.position.y = msg->pose.pose.position.y;

        this_pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
        this_pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
        this_pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
        this_pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    bool                         m_isFirstFrame;
    ros::NodeHandle              m_nh;
    IMLSICPMatcher               m_imlsMatcher;
    Eigen::Vector3d              m_prevLaserPose;
    std::vector<Eigen::Vector2d> m_prevPointCloud;

    Eigen::Matrix3d odom_to_base;
    Eigen::Vector3d odom_pose;
    Eigen::Vector3d base_pose;
    Eigen::Vector3d base_pose_prev;
    Eigen::Vector3d base_pose_tmp;

    PLICPMatcher m_plicpMatcher;

    nav_msgs::Path m_imlsPath;
    nav_msgs::Path m_plicpPath;
    nav_msgs::Path m_odomPath;

    tf::TransformListener m_tfListener;
    ros::NodeHandle       m_node;

    ros::Subscriber m_laserscanSub;
    ros::Publisher  m_imlsPathPub;
    ros::Publisher  m_plicpPathPub;
    ros::Publisher  m_odomPathPub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imls_debug");

    imlsDebug imls_debug;

    ros::spin();

    return (0);
}
