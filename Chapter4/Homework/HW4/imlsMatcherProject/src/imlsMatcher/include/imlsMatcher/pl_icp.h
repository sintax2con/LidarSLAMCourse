#ifndef PL_ICP_H
#define PL_ICP_H

#include <nav_core/recovery_behavior.h>
#include <nav_msgs/Odometry.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <chrono>

#include "boost/asio.hpp" //包含boost库函数
#include "boost/bind.hpp"
#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "stdio.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"

// pcl

#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>

// Eigen
#include <csm/csm_all.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class PLICPMatcher {
public:
    PLICPMatcher();
    
    //进行PI-ICP需要的变量
    LDP       m_prevLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;

    bool m_isFirstFrame;

    // scan　进行位姿积分
    Eigen::Vector3d scan_pos_cal;
    Eigen::Vector3d prev_laser_pose;

    std::vector<Eigen::Vector3d> odom_increments; //用来储存两帧之间的里程计的增量

    std::string odom_frame_;
    std::string base_frame_;

    // tf树查询里程计位姿
    bool getOdomPose(Eigen::Vector3d &pose, const ros::Time &t);

    //进行pl-icp的相关函数.
    void            SetPIICPParams();
    void            LaserScanToLDP(const champion_nav_msgs::ChampionNavLaserScanConstPtr &pScan, LDP &ldp);
    Eigen::Vector3d PIICPBetweenTwoFrames(LDP &currentLDPScan, Eigen::Vector3d tmprPose);
};

#endif // PL_ICP_H
