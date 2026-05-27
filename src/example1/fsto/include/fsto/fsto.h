#ifndef FSTO_H
#define FSTO_H

#include "fsto/config.h"
#include "fsto/glbmap.h"
#include "fsto/r3planner.h"
#include "fsto/traj_gen.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"

#include <iostream>
#include <memory>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

enum class ReplanState
{
    INIT,
    WAIT_TARGET,
    EXEC_TRAJ,
    REPLAN_TRAJ,
    EMERGENCY_STOP
};


class MavGlobalPlanner
{
public:
    MavGlobalPlanner(Config &conf, ros::NodeHandle &nh_);
    ~MavGlobalPlanner();

    Config config;
    ros::NodeHandle nh;

    ros::Subscriber odomSub;
    void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    ros::Subscriber imuSub;
    void imuCallBack(const sensor_msgs::Imu::ConstPtr &msg);
    ros::Subscriber mapSub;
    void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    ros::Subscriber targetSub;
    void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    ros::Subscriber trajTriggerSub;
    void trajTriggerCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    ros::Publisher trajPub;
    ros::Publisher autoManualPub;
    ros::Publisher inflate_map_pub; // 膨胀地图发布器

    static void polynomialTrajConverter(const Trajectory &traj,
                                        quadrotor_msgs::PolynomialTrajectory &trajMsg,
                                        Eigen::Isometry3d tfR2L, ros::Time &iniStamp);

    Eigen::Isometry3d curOdomPose;
    ros::Time odomStamp;
    Eigen::Vector3d curOdomVel;
    bool odomInitialized;
    Eigen::Vector3d curOdomAcc;
    bool accInitialized;
    bool mapInitialized;
    bool localMapInitialized;

    // ===== Online replanning =====
    bool hasTarget;
    Eigen::Vector3d globalGoal;

    bool hasActiveTraj;
    Trajectory currentTraj;
    ros::Time currentTrajStartTime;

    ros::Time lastReplanTime;

    void tryReplan(const ros::Time &stamp);

    bool planAndPublishLocalTraj(const Eigen::Vector3d &startPos,
                                 const Eigen::Vector3d &startVel,
                                 const Eigen::Vector3d &startAcc,
                                 const Eigen::Vector3d &goal,
                                 const ros::Time &stamp);

    bool checkCurrentTrajSafe(const ros::Time &stamp) const;

    Eigen::Vector3d selectLocalGoal(const Eigen::Vector3d &startPos) const;

    bool getReplanStartState(const ros::Time &stamp,
                             Eigen::Vector3d &startPos,
                             Eigen::Vector3d &startVel,
                             Eigen::Vector3d &startAcc,
                             ros::Time &trajStartStamp) const;

    bool publishEmergencyStopTraj(const ros::Time &stamp);

    bool isTrajectorySafe(const Trajectory &traj,
                          const ros::Time &trajStartStamp,
                          const ros::Time &stamp) const;

    bool shouldReplaceCurrentTraj(const Trajectory &newTraj,
                                  const ros::Time &newTrajStartStamp,
                                  const ros::Time &stamp) const;

    std::shared_ptr<PriorGlobalMap> glbMapPtr;    // 先验全局地图
    std::shared_ptr<LocalPerceptionMap> localMapPtr;  // 当前局部感知地图
    R3Planner r3planner;
    TrajGen trajGen;
    Visualization visualization;
    ReplanState replanState;

    // ===== Flight mode / waypoint mission =====
    std::vector<Eigen::Vector3d> presetWaypoints;
    int currentWaypointId;
    void initPresetWaypoints();
    void startPresetWaypointMission();
    void updateWaypointMission();
};

#endif