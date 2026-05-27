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
        ROS_WARN("[tryReplan] localGoal is unsafe.");

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

    double t_replan = t_now + config.replan_time_ahead;
    const double totalT = currentTraj.getTotalDuration();

    // 旧轨迹剩余太短时，不再强行从旧轨迹采样未来点
    if (t_replan >= totalT)
    {
        startPos = curOdomPose.translation();
        startVel = curOdomVel;
        startAcc = accInitialized ? curOdomAcc : Eigen::Vector3d::Zero();
        trajStartStamp = stamp;
        return true;
    }

    startPos = currentTraj.getPos(t_replan);
    startVel = currentTraj.getVel(t_replan);
    startAcc = currentTraj.getAcc(t_replan);

    // 新轨迹从旧轨迹未来拼接点开始
    trajStartStamp = currentTrajStartTime + ros::Duration(t_replan);

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

    if (dist < 0.1) 
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
    r3planner.planOnce(startPos, goal, route);
    if (route.size() <= 1)
    {
        ROS_WARN("[planAndPublishLocalTraj] R3Planner failed.");
        return false;
    }

    Eigen::Vector3d finVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d finAcc = Eigen::Vector3d::Zero();
    Trajectory traj = trajGen.generate(route, startVel, startAcc,
                                       finVel, finAcc,
                                       config.alg,
                                       visualization);

    if (traj.getPieceNum() <= 0)
    {
        ROS_WARN("[planAndPublishLocalTraj] TrajGen failed.");
        return false;
    }

    if (!shouldReplaceCurrentTraj(traj, trajStartStamp, odomStamp))
    {
        return false;
    }

    quadrotor_msgs::PolynomialTrajectory trajMsg;
    ros::Time mutableStamp = trajStartStamp;

    polynomialTrajConverter(traj, trajMsg, Eigen::Isometry3d::Identity(), mutableStamp);

    trajPub.publish(trajMsg);
    visualization.visualize(traj, route, ros::Time::now(), 1);

    currentTraj = traj;
    currentTrajStartTime = trajStartStamp;
    hasActiveTraj = true;

    ROS_WARN("[planAndPublishLocalTraj] New local trajectory published. Duration = %.3f", traj.getTotalDuration());

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

Visualization::Visualization(Config &conf, NodeHandle &nh_)
    : config(conf), nh(nh_)
{
    routePub = nh.advertise<visualization_msgs::Marker>("/fsto/visualization/route", 1);
    wayPointsPub = nh.advertise<visualization_msgs::Marker>("/fsto/visualization/waypoints", 1);
    appliedTrajectoryPub = nh.advertise<visualization_msgs::Marker>("/fsto/visualization/applied_trajectory", 1);
}

void Visualization::visualize(const Trajectory &appliedTraj, const vector<Vector3d> &route, Time timeStamp, int id)
{
    visualization_msgs::Marker routeMarker, wayPointsMarker, appliedTrajMarker;

    routeMarker.id = id;
    routeMarker.type = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp = timeStamp;
    routeMarker.header.frame_id = config.odomFrame;
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action = visualization_msgs::Marker::ADD;
    routeMarker.ns = "route";
    routeMarker.color.r = 1.00;
    routeMarker.color.g = 0.00;
    routeMarker.color.b = 0.00;
    routeMarker.color.a = 1.00;
    routeMarker.scale.x = 0.10;

    wayPointsMarker = routeMarker;
    wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    wayPointsMarker.ns = "waypoints";
    wayPointsMarker.color.r = 0.00;
    wayPointsMarker.color.g = 0.00;
    wayPointsMarker.color.b = 1.00;
    wayPointsMarker.scale.x = 0.20;
    wayPointsMarker.scale.y = 0.20;
    wayPointsMarker.scale.z = 0.20;

    appliedTrajMarker = routeMarker;
    appliedTrajMarker.header.frame_id = config.odomFrame;
    appliedTrajMarker.id = id;
    appliedTrajMarker.ns = "trajectory";
    appliedTrajMarker.scale.x = 0.15;
    if (id == 0)
    {
        appliedTrajMarker.color.r = 0.85;
        appliedTrajMarker.color.g = 0.10;
        appliedTrajMarker.color.b = 0.10;
    }
    else if (id == 1)
    {
        appliedTrajMarker.color.r = 1.00;
        appliedTrajMarker.color.g = 0.65;
        appliedTrajMarker.color.b = 0.00;
    }        
    else if (id == 2)
    {
        appliedTrajMarker.color.r = 0.00;
        appliedTrajMarker.color.g = 0.45;
        appliedTrajMarker.color.b = 0.74;
    }
    else if (id == 3)
    {
        appliedTrajMarker.color.r = 1.00;
        appliedTrajMarker.color.g = 0.00;
        appliedTrajMarker.color.b = 1.00;            
    }
    else if (id == 4)
    {
        appliedTrajMarker.color.r = 0.10;
        appliedTrajMarker.color.g = 0.65;
        appliedTrajMarker.color.b = 0.10;
    }
    else if (id == 5)
    {
        appliedTrajMarker.color.r = 0.00;
        appliedTrajMarker.color.g = 0.00;
        appliedTrajMarker.color.b = 0.00;
    }
    else
    {
        appliedTrajMarker.color.r = 0.93;
        appliedTrajMarker.color.g = 0.48;
        appliedTrajMarker.color.b = 0.26;
    }
        
    if (route.size() > 0)
    {
        bool first = true;
        Vector3d last;
        for (auto it : route)
        {
            if (first)
            {
                first = false;
                last = it;
                continue;
            }
            geometry_msgs::Point point;

            point.x = last(0);
            point.y = last(1);
            point.z = last(2);
            routeMarker.points.push_back(point);
            point.x = it(0);
            point.y = it(1);
            point.z = it(2);
            routeMarker.points.push_back(point);
            last = it;

            wayPointsMarker.points.push_back(point);
        }

        routePub.publish(routeMarker);
    }

    if (route.size() > 0)
    {
        for (auto it : route)
        {
            geometry_msgs::Point point;
            point.x = it(0);
            point.y = it(1);
            point.z = it(2);
            wayPointsMarker.points.push_back(point);
        }

        wayPointsPub.publish(wayPointsMarker);
    }

    if (appliedTraj.getPieceNum() > 0)
    {
        double T = 0.01;
        Vector3d lastX = appliedTraj.getPos(0.0);
        for (double t = T; t < appliedTraj.getTotalDuration(); t += T)
        {
            geometry_msgs::Point point;
            Vector3d X = appliedTraj.getPos(t);
            point.x = lastX(0);
            point.y = lastX(1);
            point.z = lastX(2);
            appliedTrajMarker.points.push_back(point);
            point.x = X(0);
            point.y = X(1);
            point.z = X(2);
            appliedTrajMarker.points.push_back(point);
            lastX = X;
        }
        appliedTrajectoryPub.publish(appliedTrajMarker);
    }
}
