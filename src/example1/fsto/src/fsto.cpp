#include <fsto/fsto.h>
#include <limits>

using namespace std;
using namespace ros;
using namespace Eigen;

MavGlobalPlanner::MavGlobalPlanner(Config &conf, NodeHandle &nh_)
    : config(conf), nh(nh_), odomInitialized(false),
      accInitialized(false), mapInitialized(false), localMapInitialized(false),
      hasTarget(false),
      hasActiveTraj(false),
      currentWaypointId(0),
      lastReplanTime(ros::Time(0)),
      glbMapPtr(make_shared<PriorGlobalMap>(config)),
      localMapPtr(make_shared<LocalPerceptionMap>(config)),
      r3planner(config, localMapPtr),
      trajGen(config, localMapPtr),
      visualization(config, nh)
{
    odomSub = nh.subscribe(config.odomTopic, 3, &MavGlobalPlanner::odomCallBack,
                           this, TransportHints().tcpNoDelay());
    imuSub = nh.subscribe(config.imuTopic, 3, &MavGlobalPlanner::imuCallBack, this,
                          TransportHints().tcpNoDelay());

    mapSub = nh.subscribe(config.mapTopic, 1, &MavGlobalPlanner::mapCallBack, this,
                          TransportHints().tcpNoDelay());
    targetSub = nh.subscribe(config.targetTopic, 1, &MavGlobalPlanner::targetCallBack, this,
                             TransportHints().tcpNoDelay());
    trajTriggerSub = nh.subscribe(config.trajTriggerTopic, 1, &MavGlobalPlanner::trajTriggerCallBack, this,
                                  TransportHints().tcpNoDelay());
    trajPub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>(config.trajectoryTopic, 1);
    autoManualPub = nh.advertise<sensor_msgs::Joy>(config.autoManualTopic, 1);
    inflate_map_pub = nh.advertise<sensor_msgs::PointCloud2>(config.inflateMapTopic, 1);
    odomStamp = Time::now();

    initPresetWaypoints();
}

MavGlobalPlanner::~MavGlobalPlanner()
{
}

void MavGlobalPlanner::odomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    Translation3d tranOdomBody(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Quaterniond quatOdomBody(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    curOdomPose = tranOdomBody * quatOdomBody;
    curOdomVel = Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    odomStamp = msg->header.stamp;
    odomInitialized = true;

    // ======= 降频 更新与发布 局部感知膨胀地图 =======
    static ros::Time last_local_map_update_time = ros::Time(0);
    double local_map_update_dt = config.local_map_update_dt;

    if (mapInitialized && (msg->header.stamp - last_local_map_update_time).toSec() > local_map_update_dt)
    {
        // 更新
        Eigen::Vector3d current_pos = curOdomPose.translation();
        localMapPtr->buildLocalMapFromGlobal(*glbMapPtr, current_pos, config.sensingRadius);
        localMapInitialized = true;
        // 发布
        sensor_msgs::PointCloud2 inflate_msg;
        localMapPtr->getLocalInflatedMap(inflate_msg, current_pos, config.sensingRadius);
        inflate_msg.header.stamp = msg->header.stamp;
        inflate_msg.header.frame_id = config.odomFrame;
        inflate_map_pub.publish(inflate_msg);

        last_local_map_update_time = msg->header.stamp;

        if (config.flight_mode == 2)
        {
            updateWaypointMission();
        }

        if (hasTarget)
        {
            tryReplan(msg->header.stamp);
        }
    }
}

void MavGlobalPlanner::imuCallBack(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (odomInitialized)
    {

        Eigen::Quaterniond quatGimbal(msg->orientation.w,
                                      msg->orientation.x,
                                      msg->orientation.y,
                                      msg->orientation.z);
        curOdomAcc = curOdomPose.rotation() * quatGimbal.inverse() *
                     Vector3d(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);
        accInitialized = true;
    }
}

void MavGlobalPlanner::mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (!mapInitialized)
    {
        glbMapPtr->initialize(msg);
        mapInitialized = true;
        ROS_WARN("[mapCallBack] Map Initialized.");
    }
}

void MavGlobalPlanner::targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!mapInitialized || !localMapInitialized || !odomInitialized)
    {
        ROS_WARN("[targetCallBack] Waiting for odom, global map and local map.");
        return;
    }

    if (config.flight_mode == 2)
    {
        ROS_WARN("[targetCallBack] Click trigger received. Start preset waypoint mission.");
        startPresetWaypointMission();
        return;
    }

    // mode 1: click goal mode
    double zGoal = fabs(msg->pose.orientation.z) *
                       (config.r3Bound[5] - config.r3Bound[4] - 2 * config.r3SafeRadius) +
                   config.r3Bound[4] + config.r3SafeRadius;

    globalGoal = Eigen::Vector3d(msg->pose.position.x,
                                 msg->pose.position.y,
                                 zGoal);

    hasTarget = true;
    hasActiveTraj = false;
    lastReplanTime = ros::Time(0);

    ROS_WARN("[targetCallBack] New clicked global goal received.");

    tryReplan(odomStamp);
}

void MavGlobalPlanner::trajTriggerCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    sensor_msgs::Joy joyMsg;
    const int JOY_AUTO = 7;
    for (int i = 0; i < JOY_AUTO + 1; i++)
    {
        joyMsg.buttons.push_back(0);
    }
    joyMsg.buttons.at(JOY_AUTO) = 1;
    autoManualPub.publish(joyMsg);
}

void MavGlobalPlanner::tryReplan(const ros::Time &stamp)
{
    if (!hasTarget || !odomInitialized || !mapInitialized || !localMapInitialized) return;

    if (config.flight_mode == 1 && (globalGoal - curOdomPose.translation()).norm() < config.waypoint_reach_thresh)
    {
        ROS_WARN("[tryReplan] Global goal reached.");
        hasTarget = false;
        hasActiveTraj = false;
        return;
    }

    bool needReplan = false;
    bool urgentReplan = false;

    // 刚启动，没有轨迹，需规划
    if (!hasActiveTraj)
    {
        ROS_WARN("[tryReplan] There are no active traj, require replan.");
        needReplan = true;
    }
    // 当前时刻 Collision_check_horizon 内发生碰撞需重规划
    if (hasActiveTraj && !checkCurrentTrajSafe(stamp))
    {
        ROS_WARN("[tryReplan] checkCurrentTrajSafe: current trajectory is unsafe, require replan.");
        needReplan = true;
        urgentReplan = true;
    }
    // 轨迹剩余时间不足需要初始化
    if (hasActiveTraj)
    {
        const double t_now = (stamp - currentTrajStartTime).toSec();
        const double remaining_time = currentTraj.getTotalDuration() - t_now;
        if (remaining_time < config.min_traj_remaining_time)
        {
            ROS_WARN("[tryReplan] Remain time is not enough, require replan.");
            needReplan = true;
        }
    }
    // // 达到设置的重规划周期需要初始化
    // if ((stamp - lastReplanTime).toSec() > config.replan_dt)
    // {
    //     needReplan = true;
    // }

    if (!needReplan) return;

    // // 冷却机制：非紧急重规划不能过于频繁发布新轨迹
    // if (!urgentReplan &&
    //     hasActiveTraj &&
    //     (stamp - lastReplanTime).toSec() < config.min_replan_interval)
    // {
    //     return;
    // }

    // 获取重规划的起点和终点
    Eigen::Vector3d startPos, startVel, startAcc;
    ros::Time trajStartStamp;
    getReplanStartState(stamp, startPos, startVel, startAcc, trajStartStamp);
    const Eigen::Vector3d localGoal = selectLocalGoal(startPos);

    if (!localMapPtr->safeQuery(localGoal, config.bodySafeRadius))
    {
        ROS_WARN("[tryReplan] localGoal (%.2f,%.2f,%.2f) is unsafe.", localGoal(0),localGoal(1),localGoal(2));

        if (hasActiveTraj && checkCurrentTrajSafe(stamp))
        {
            ROS_WARN("[tryReplan] Keep executing old safe trajectory.");
            return;
        }

        ROS_ERROR("[tryReplan] Unsafe local goal and current trajectory unsafe. Publish emergency stop.");
        publishEmergencyStopTraj(stamp);
        return;
    }

    const bool success = planAndPublishLocalTraj(startPos, startVel, startAcc, localGoal, trajStartStamp);

    if (success)
    {
        lastReplanTime = stamp;
        return;
    }

    ROS_WARN("[tryReplan] Local replanning failed.");
    // 失败时：如果旧轨迹仍安全，则继续执行旧轨迹
    if (hasActiveTraj && checkCurrentTrajSafe(stamp))
    {
        ROS_WARN("[tryReplan] Keep executing old safe trajectory.");
        return;
    }
    // 失败且旧轨迹不安全：发布刹停轨迹
    ROS_ERROR("[tryReplan] Replan failed and current trajectory unsafe. Publish emergency stop.");
    publishEmergencyStopTraj(stamp);
}

/*
如果当前没有旧轨迹：
    从当前 odom 状态开始规划。

如果当前已有旧轨迹：
    不直接从当前 odom 状态开始；
    而是从旧轨迹未来某个时间点开始规划，
    使新轨迹和旧轨迹在拼接点处保持 p、v、a 连续。
*/
bool MavGlobalPlanner::getReplanStartState(const ros::Time &stamp,
                                           Eigen::Vector3d &startPos,
                                           Eigen::Vector3d &startVel,
                                           Eigen::Vector3d &startAcc,
                                           ros::Time &trajStartStamp) const
{
    if (!hasActiveTraj || currentTraj.getPieceNum() <= 0)
    {
        startPos = curOdomPose.translation();
        startVel = curOdomVel;
        startAcc = accInitialized ? curOdomAcc : Eigen::Vector3d::Zero();
        trajStartStamp = stamp;
        return true;
    }

    double t_now = (stamp - currentTrajStartTime).toSec();
    if (t_now < 0.0)
    {
        t_now = 0.0;
    }

    const double totalT = currentTraj.getTotalDuration();

    if (totalT <= 1e-3)
    {
        startPos = curOdomPose.translation();
        startVel = curOdomVel;
        startAcc = accInitialized ? curOdomAcc : Eigen::Vector3d::Zero();
        trajStartStamp = stamp;
        return true;
    }

    double t_replan = t_now + config.replan_time_ahead;

    if (t_replan < totalT)
    {
        startPos = currentTraj.getPos(t_replan);
        startVel = currentTraj.getVel(t_replan);
        startAcc = currentTraj.getAcc(t_replan);
        trajStartStamp = currentTrajStartTime + ros::Duration(t_replan);
        return true;
    }

    // 旧轨迹剩余太短时，尝试使用旧轨迹末端前的一个点。
    // 但必须保证该点仍然位于当前时刻之后。
    const double tail_margin = std::min(0.10, 0.5 * totalT);
    const double t_tail = totalT - tail_margin;

    if (t_tail > t_now)
    {
        startPos = currentTraj.getPos(t_tail);
        startVel = currentTraj.getVel(t_tail);
        startAcc = currentTraj.getAcc(t_tail);
        trajStartStamp = currentTrajStartTime + ros::Duration(t_tail);
        return true;
    }

    // 如果旧轨迹末端附近的点也已经在过去，
    // 则不能再从旧轨迹采样，只能从当前 odom 状态开始。
    startPos = curOdomPose.translation();
    startVel = curOdomVel;
    startAcc = accInitialized ? curOdomAcc : Eigen::Vector3d::Zero();
    trajStartStamp = stamp;

    return true;
}

/*
1. 如果全局目标很近，直接返回全局目标；
2. 否则沿全局目标方向选一个前视局部目标 nominal；
3. 如果 nominal 安全，直接返回 nominal；
4. 如果 nominal 不安全，则在水平面左右采样多个候选点；
5. 只保留安全候选点；
6. 从安全候选点中选择“离全局目标近且偏转角小”的点；
7. 返回该点作为局部重规划目标。
*/
Eigen::Vector3d MavGlobalPlanner::selectLocalGoal(const Eigen::Vector3d &startPos) const
{
    Eigen::Vector3d diff = globalGoal - startPos;
    const double dist = diff.norm();

    if (dist < config.sensingRadius-0.5) 
    {
        return globalGoal;
    }

    const double localGoalDist =
        std::min(dist, config.local_goal_ratio * config.sensingRadius);

    Eigen::Vector3d dir = diff.normalized();    // 指向全局目标的单位方向

    auto clampZ = [&](Eigen::Vector3d p) {
        p(2) = std::max(config.r3Bound[4] + config.r3SafeRadius,
                        std::min(config.r3Bound[5] - config.r3SafeRadius, p(2)));
        return p;
    };

    // 先尝试直线前视局部目标
    Eigen::Vector3d nominal = startPos + dir * localGoalDist;
    nominal(2) = globalGoal(2);
    nominal = clampZ(nominal);
    if (localMapPtr->safeQuery(nominal, config.bodySafeRadius))
    {
        return nominal;
    }

    // 水平面左右采样候选点
    Eigen::Vector3d dir_xy(dir(0), dir(1), 0.0);
    if (dir_xy.norm() < 1e-3)
    {
        return nominal;
    }
    dir_xy.normalize();

    const double angle_step = config.local_goal_sample_angle * M_PI / 180.0;

    Eigen::Vector3d bestGoal = nominal;
    double bestScore = std::numeric_limits<double>::infinity();

    for (int k = -3; k <= 3; ++k)
    {
        const double angle = k * angle_step;
        const double c = std::cos(angle);
        const double s = std::sin(angle);

        Eigen::Vector3d rot_dir;
        rot_dir << c * dir_xy(0) - s * dir_xy(1),
                   s * dir_xy(0) + c * dir_xy(1),
                   0.0;

        Eigen::Vector3d candidate = startPos + rot_dir * localGoalDist;
        candidate(2) = globalGoal(2);
        candidate = clampZ(candidate);

        if (!localMapPtr->safeQuery(candidate, config.bodySafeRadius))
        {
            continue;
        }

        // 越接近全局目标越好；轻微惩罚大角度偏转
        const double score =
            (candidate - globalGoal).norm() + 0.2 * std::abs(angle);

        if (score < bestScore)
        {
            bestScore = score;
            bestGoal = candidate;
        }
    }

    return bestGoal;
}

bool MavGlobalPlanner::publishEmergencyStopTraj(const ros::Time &stamp)
{
    Eigen::Vector3d startPos = curOdomPose.translation();
    Eigen::Vector3d startVel = curOdomVel;
    Eigen::Vector3d startAcc = accInitialized ? curOdomAcc : Eigen::Vector3d::Zero();

    Eigen::Vector3d stopPos = startPos;

    const double v_norm = startVel.norm();
    if (v_norm > 1e-3)
    {
        const double stop_dist =
            std::min(0.5 * v_norm * config.emergency_stop_duration,
                     0.5 * config.sensingRadius);

        stopPos = startPos + startVel.normalized() * stop_dist;
    }

    if (!localMapPtr->safeQuery(stopPos, config.bodySafeRadius))
    {
        stopPos = startPos;
    }

    std::vector<Eigen::Vector3d> route;
    route.push_back(startPos);
    route.push_back(stopPos);

    Eigen::Vector3d finVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d finAcc = Eigen::Vector3d::Zero();

    Trajectory stopTraj = trajGen.generate(route,
                                           startVel,
                                           startAcc,
                                           finVel,
                                           finAcc,
                                           config.alg,
                                           visualization);

    if (stopTraj.getPieceNum() <= 0)
    {
        ROS_ERROR("[publishEmergencyStopTraj] Failed to generate stop trajectory.");
        return false;
    }

    quadrotor_msgs::PolynomialTrajectory trajMsg;
    ros::Time mutableStamp = stamp;

    polynomialTrajConverter(stopTraj,
                            trajMsg,
                            Eigen::Isometry3d::Identity(),
                            mutableStamp);

    trajPub.publish(trajMsg);
    visualization.visualize(stopTraj, route, ros::Time::now(), 1);

    currentTraj = stopTraj;
    currentTrajStartTime = stamp;
    hasActiveTraj = true;
    lastReplanTime = stamp;

    ROS_ERROR("[publishEmergencyStopTraj] Stop trajectory published.");

    return true;
}

bool MavGlobalPlanner::shouldReplaceCurrentTraj(
    const Trajectory &newTraj,
    const ros::Time &newTrajStartStamp,
    const ros::Time &stamp) const
{
    if (!hasActiveTraj || currentTraj.getPieceNum() <= 0)
    {
        return true;
    }

    // 如果旧轨迹已经不安全，则允许替换
    if (!checkCurrentTrajSafe(stamp))
    {
        return true;
    }

    const double compare_horizon = 0.8;
    const double compare_dt = 0.1;

    double max_pos_diff = 0.0;
    double max_vel_diff = 0.0;

    for (double tau = 0.0; tau <= compare_horizon; tau += compare_dt)
    {
        double t_old =
            (newTrajStartStamp + ros::Duration(tau) - currentTrajStartTime).toSec();

        double t_new = tau;

        if (t_old < 0.0 ||
            t_old > currentTraj.getTotalDuration() ||
            t_new > newTraj.getTotalDuration())
        {
            continue;
        }

        Eigen::Vector3d p_old = currentTraj.getPos(t_old);
        Eigen::Vector3d v_old = currentTraj.getVel(t_old);

        Eigen::Vector3d p_new = newTraj.getPos(t_new);
        Eigen::Vector3d v_new = newTraj.getVel(t_new);

        max_pos_diff = std::max(max_pos_diff, (p_new - p_old).norm());
        max_vel_diff = std::max(max_vel_diff, (v_new - v_old).norm());
    }

    if (max_pos_diff > 1.0 || max_vel_diff > 1.5)
    {
        ROS_WARN("[shouldReplaceCurrentTraj] Reject new trajectory: too different from old safe trajectory. pos_diff=%.2f vel_diff=%.2f",
                 max_pos_diff,
                 max_vel_diff);
        return false;
    }

    return true;
}

void MavGlobalPlanner::startPresetWaypointMission()
{
    if (presetWaypoints.empty())
    {
        ROS_ERROR("[startPresetWaypointMission] No preset waypoints.");
        return;
    }

    currentWaypointId = 0;
    globalGoal = presetWaypoints[currentWaypointId];
    hasTarget = true;
    hasActiveTraj = false;
    lastReplanTime = ros::Time(0);

    ROS_WARN("[startPresetWaypointMission] Start mission. Current waypoint = %d / %lu",
             currentWaypointId + 1,
             presetWaypoints.size());

    tryReplan(odomStamp);
}

void MavGlobalPlanner::updateWaypointMission()
{
    if (!hasTarget || presetWaypoints.empty()) return;

    Eigen::Vector3d curPos = curOdomPose.translation();

    if ((curPos - globalGoal).norm() > config.waypoint_reach_thresh) return;

    ROS_WARN("[updateWaypointMission] Reached waypoint %d / %lu", currentWaypointId + 1, presetWaypoints.size());

    currentWaypointId++;

    if (currentWaypointId >= static_cast<int>(presetWaypoints.size()))
    {
        ROS_WARN("[updateWaypointMission] Mission completed.");
        hasTarget = false;
        hasActiveTraj = false;
        return;
    }

    globalGoal = presetWaypoints[currentWaypointId];
    hasTarget = true;
    // hasActiveTraj = false;
    lastReplanTime = ros::Time(0);

    ROS_WARN("[updateWaypointMission]] Next waypoint = %d / %lu", currentWaypointId + 1, presetWaypoints.size());

    tryReplan(odomStamp);
}

void MavGlobalPlanner::initPresetWaypoints()
{
    presetWaypoints.clear();

    if (config.preset_waypoints.size() % 3 != 0)
    {
        ROS_ERROR("[initPresetWaypoints] Preset_waypoints size must be multiple of 3.");
        return;
    }

    for (size_t i = 0; i + 2 < config.preset_waypoints.size(); i += 3)
    {
        Eigen::Vector3d p(config.preset_waypoints[i],
                          config.preset_waypoints[i + 1],
                          config.preset_waypoints[i + 2]);

        p(2) = std::max(config.r3Bound[4] + config.r3SafeRadius,
                        std::min(config.r3Bound[5] - config.r3SafeRadius, p(2)));

        presetWaypoints.push_back(p);
    }

    ROS_WARN("[initPresetWaypoints] Loaded %lu preset waypoints.", presetWaypoints.size());
}

bool MavGlobalPlanner::planAndPublishLocalTraj(const Eigen::Vector3d &startPos,
                                               const Eigen::Vector3d &startVel,
                                               const Eigen::Vector3d &startAcc,
                                               const Eigen::Vector3d &goal,
                                               const ros::Time &trajStartStamp)
{
    std::vector<Eigen::Vector3d> route;

    std::cout << "[planAndPublishLocalTraj] start of r3planner:" << startPos << "goal of r3planner:" << goal << std::endl;
    double rrt_cost = r3planner.planOnce(startPos, startVel, goal, route);
    visualization

    if (!std::isfinite(rrt_cost) || route.size() <= 1)
    {
        ROS_WARN("[planAndPublishLocalTraj] Directional R3Planner failed. Retry without direction constraint.");

        route.clear();
        rrt_cost = r3planner.planOnce(startPos, goal, route);
    }

    if (!std::isfinite(rrt_cost) || route.size() <= 1)
    {
        ROS_WARN("[planAndPublishLocalTraj] R3Planner failed.");
        return false;
    }

    route = trajGen.routeSimplify(route, config.spatialResolution);
    if (route.size() <= 1)
    {
        ROS_WARN("[planAndPublishLocalTraj] Route invalid after LOS simplify.");
        return false;
    }

    // ============================================================
    // RRT 搜索耗时后，重新用当前 odom 修正 route 头部和轨迹初始状态
    // ============================================================
    Eigen::Vector3d trajStartPos = startPos;
    Eigen::Vector3d trajStartVel = startVel;
    Eigen::Vector3d trajStartAcc = startAcc;
    ros::Time actualTrajStartStamp = trajStartStamp;

    // 备份 RRT 原始 route，防止 repair 失败后 route 被部分修改
    std::vector<Eigen::Vector3d> routeBeforeRepair = route;

    bool repairSuccess = repairRouteFromCurrentOdom(route,
                                                    trajStartPos,
                                                    trajStartVel,
                                                    trajStartAcc,
                                                    actualTrajStartStamp);

    if (!repairSuccess)
    {
        ROS_WARN("[planAndPublishLocalTraj] Route repair from current odom failed. "
                 "Fallback to original route and original start state.");

        route = routeBeforeRepair;

        trajStartPos = startPos;
        trajStartVel = startVel;
        trajStartAcc = startAcc;
        actualTrajStartStamp = trajStartStamp;

        if (!route.empty() && (route.front() - trajStartPos).norm() > 1e-3)
        {
            route.front() = trajStartPos;
        }
    }

    if (route.size() <= 1)
    {
        ROS_WARN("[planAndPublishLocalTraj] Route invalid after repair/fallback.");
        return false;
    }

    const double global_goal_thresh =
        std::max(config.spatialResolution, config.waypoint_reach_thresh);

    const bool localGoalIsGlobalGoal =
        ((goal - globalGoal).norm() < global_goal_thresh);

    if (repairSuccess &&
        (goal - trajStartPos).norm() < 0.2 &&
        !localGoalIsGlobalGoal)
    {
        ROS_WARN("[planAndPublishLocalTraj] Local goal is too close after route repair.");
        return false;
    }

    Eigen::Vector3d finVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d finAcc = Eigen::Vector3d::Zero();

    if (!localGoalIsGlobalGoal)
    {
        // 根据 route 末端切线方向设置局部轨迹终端速度
        Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
        const Eigen::Vector3d routeEnd = route.back();

        // 1. 从 route 末端往前找一个与终点不重合的点，用于计算末端切线方向。
        for (int i = static_cast<int>(route.size()) - 2; i >= 0; --i)
        {
            Eigen::Vector3d diff = routeEnd - route[i];

            if (diff.norm() > 0.1)
            {
                tangent = diff.normalized();
                break;
            }
        }

        // 2. 如果退化，使用 goal - trajStartPos
        if (tangent.norm() < 1e-3)
        {
            Eigen::Vector3d diff = goal - trajStartPos;
            if (diff.norm() > 1e-3)
            {
                tangent = diff.normalized();
            }
        }

        // 3. 如果仍退化，使用 trajStartVel
        if (tangent.norm() < 1e-3 && trajStartVel.norm() > 1e-3)
        {
            tangent = trajStartVel.normalized();
        }

        // 4. 最后统一赋值
        if (tangent.norm() > 1e-3)
        {
            finVel = config.local_goal_speed * tangent.normalized();
        }
        else
        {
            finVel = Eigen::Vector3d::Zero();
        }
    }

    ROS_WARN("[planAndPublishLocalTraj] Terminal velocity set by route tangent: "
             "vx=%.3f, vy=%.3f, vz=%.3f, speed=%.3f",
             finVel(0), finVel(1), finVel(2), finVel.norm());

    Trajectory traj = trajGen.generate(route,
                                       trajStartVel,
                                       trajStartAcc,
                                       finVel,
                                       finAcc,
                                       config.alg,
                                       visualization);

    if (traj.getPieceNum() <= 0)
    {
        ROS_WARN("[planAndPublishLocalTraj] TrajGen failed.");
        return false;
    }

    if (!shouldReplaceCurrentTraj(traj, actualTrajStartStamp, odomStamp))
    {
        return false;
    }

    quadrotor_msgs::PolynomialTrajectory trajMsg;
    ros::Time mutableStamp = actualTrajStartStamp;

    polynomialTrajConverter(traj, trajMsg, Eigen::Isometry3d::Identity(), mutableStamp);

    trajPub.publish(trajMsg);
    visualization.visualize(traj, route, ros::Time::now(), 1);

    currentTraj = traj;
    currentTrajStartTime = actualTrajStartStamp;
    hasActiveTraj = true;

    ROS_WARN("[planAndPublishLocalTraj] New local trajectory published. Duration = %.3f",
             traj.getTotalDuration());

    return true;
}

bool MavGlobalPlanner::repairRouteFromCurrentOdom(
    std::vector<Eigen::Vector3d> &route,
    Eigen::Vector3d &trajStartPos,
    Eigen::Vector3d &trajStartVel,
    Eigen::Vector3d &trajStartAcc,
    ros::Time &trajStartStamp) const
{
    if (!odomInitialized || route.size() < 2)
    {
        return false;
    }

    const Eigen::Vector3d curPos = curOdomPose.translation();
    const Eigen::Vector3d curVel = curOdomVel;
    const Eigen::Vector3d curAcc = accInitialized ? curOdomAcc : Eigen::Vector3d::Zero();

    if (!localMapPtr->safeQuery(curPos, config.bodySafeRadius))
    {
        ROS_WARN("[repairRouteFromCurrentOdom] Current odom position is unsafe.");
        return false;
    }

    // 从当前 odom 位置尝试直接连接到 route 中尽可能靠后的点。
    // 找到后删除其前面的所有旧 route 点。
    int connect_id = -1;

    for (int j = static_cast<int>(route.size()) - 1; j >= 0; --j)
    {
        if (trajGen.segmentSafe(curPos,
                                route[j],
                                config.spatialResolution,
                                config.r3SafeRadius))
        {
            connect_id = j;
            break;
        }
    }

    if (connect_id < 0)
    {
        ROS_WARN("[repairRouteFromCurrentOdom] Cannot connect current odom to any route point.");
        return false;
    }

    std::vector<Eigen::Vector3d> repairedRoute;
    repairedRoute.reserve(route.size() - connect_id + 1);

    repairedRoute.push_back(curPos);

    // 如果 curPos 已经非常接近 route[connect_id]，不要重复加入。
    if ((route[connect_id] - curPos).norm() > 1e-3)
    {
        repairedRoute.push_back(route[connect_id]);
    }

    for (size_t k = static_cast<size_t>(connect_id + 1); k < route.size(); ++k)
    {
        if ((route[k] - repairedRoute.back()).norm() > 1e-3)
        {
            repairedRoute.push_back(route[k]);
        }
    }

    if (repairedRoute.size() < 2)
    {
        ROS_WARN("[repairRouteFromCurrentOdom] Repaired route has less than 2 points.");
        return false;
    }

    // 可选：只对修正后的短 route 再做一次 LOS 简化。
    // 因为前缀已经被裁剪，点数通常很少，这一步开销较小。
    repairedRoute = trajGen.routeSimplify(repairedRoute, config.spatialResolution);

    if (repairedRoute.size() < 2)
    {
        ROS_WARN("[repairRouteFromCurrentOdom] Route invalid after LOS simplify.");
        return false;
    }

    route = repairedRoute;

    trajStartPos = curPos;
    trajStartVel = curVel;
    trajStartAcc = curAcc;
    trajStartStamp = odomStamp;

    ROS_WARN("[repairRouteFromCurrentOdom] connect_id=%d, repaired_route_size=%lu, "
             "startVel=%.3f",
             connect_id,
             route.size(),
             trajStartVel.norm());

    return true;
}

bool MavGlobalPlanner::checkCurrentTrajSafe(const ros::Time &stamp) const
{
    if (!hasActiveTraj)
    {
        return false;
    }

    return isTrajectorySafe(currentTraj, currentTrajStartTime, stamp);
}

bool MavGlobalPlanner::isTrajectorySafe(const Trajectory &traj,
                                        const ros::Time &trajStartStamp,
                                        const ros::Time &stamp) const
{
    if (traj.getPieceNum() <= 0)
    {
        return false;
    }

    double t_now = (stamp - trajStartStamp).toSec();
    if (t_now < 0.0)
    {
        t_now = 0.0;
    }

    const double totalT = traj.getTotalDuration();
    if (t_now >= totalT)
    {
        return false;
    }

    const double t_end = std::min(totalT, t_now + config.collision_check_horizon);

    for (double t = t_now; t <= t_end; t += config.collision_check_dt)
    {
        const Eigen::Vector3d p = traj.getPos(t);

        if (!localMapPtr->safeQuery(p, config.bodySafeRadius))
        {
            return false;
        }
    }

    return true;
}

void MavGlobalPlanner::polynomialTrajConverter(const Trajectory &traj,
                                               quadrotor_msgs::PolynomialTrajectory &trajMsg,
                                               Eigen::Isometry3d tfR2L, Time &iniStamp)
{
    trajMsg.header.stamp = iniStamp;
    static uint32_t traj_id = 0;
    traj_id++;
    trajMsg.trajectory_id = traj_id;
    trajMsg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    trajMsg.num_order = traj[0].getOrder();
    trajMsg.num_segment = traj.getPieceNum();
    Eigen::Vector3d initialVel, finalVel;
    initialVel = tfR2L * traj.getVel(0.0);  // 将轨迹从规划器所使用的坐标系（如 Robot-centric 或 Reference frame）转换到无人机控制器所需的坐标系（如 Local/Body frame）
    finalVel = tfR2L * traj.getVel(traj.getTotalDuration());
    trajMsg.start_yaw = atan2(initialVel(1), initialVel(0));
    trajMsg.final_yaw = atan2(finalVel(1), finalVel(0));    // 偏航角与速度方向对齐

    for (size_t p = 0; p < (size_t)traj.getPieceNum(); p++)
    {
        trajMsg.time.push_back(traj[p].getDuration());
        trajMsg.order.push_back(traj[p].getCoeffMat().cols() - 1);

        Eigen::VectorXd linearTr(2);
        linearTr << 0.0, trajMsg.time[p];
        std::vector<Eigen::VectorXd> linearTrCoeffs;
        linearTrCoeffs.emplace_back(1);
        linearTrCoeffs[0] << 1;
        for (size_t k = 0; k < trajMsg.order[p]; k++)
        {
            linearTrCoeffs.push_back(RootFinder::polyConv(linearTrCoeffs[k], linearTr));
        }

        Eigen::MatrixXd coefMat(3, traj[p].getCoeffMat().cols());
        for (int i = 0; i < coefMat.cols(); i++)
        {
            coefMat.col(i) = tfR2L.rotation() * traj[p].getCoeffMat().col(coefMat.cols() - i - 1).head<3>();
        }
        coefMat.col(0) = (coefMat.col(0) + tfR2L.translation()).eval();

        for (int i = 0; i < coefMat.cols(); i++)
        {
            double coefx(0.0), coefy(0.0), coefz(0.0);
            for (int j = i; j < coefMat.cols(); j++)
            {
                coefx += coefMat(0, j) * linearTrCoeffs[j](i);
                coefy += coefMat(1, j) * linearTrCoeffs[j](i);
                coefz += coefMat(2, j) * linearTrCoeffs[j](i);
            }
            trajMsg.coef_x.push_back(coefx);
            trajMsg.coef_y.push_back(coefy);
            trajMsg.coef_z.push_back(coefz);
        }
    }

    trajMsg.mag_coeff = 1.0;
    trajMsg.debug_info = "";
}
