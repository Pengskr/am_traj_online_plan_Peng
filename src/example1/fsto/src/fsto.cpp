#include <fsto/fsto.h>

using namespace std;
using namespace ros;
using namespace Eigen;

MavGlobalPlanner::MavGlobalPlanner(Config &conf, NodeHandle &nh_)
    : config(conf), nh(nh_), odomInitialized(false),
      accInitialized(false), mapInitialized(false), localMapInitialized(false),
      hasTarget(false),
      hasActiveTraj(false),
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

    // ======= 降频更新与发布局部感知膨胀地图 =======
    static ros::Time last_local_map_update_time = ros::Time(0);
    double local_map_update_dt = config.local_map_update_dt;

    if (mapInitialized && (msg->header.stamp - last_local_map_update_time).toSec() > local_map_update_dt)
    {
        Eigen::Vector3d current_pos = curOdomPose.translation();

        localMapPtr->buildLocalMapFromGlobal(*glbMapPtr,
                                             current_pos,
                                             config.sensingRadius);
        localMapInitialized = true;

        sensor_msgs::PointCloud2 inflate_msg;
        localMapPtr->getLocalInflatedMap(inflate_msg,
                                         current_pos,
                                         config.sensingRadius);

        inflate_msg.header.stamp = msg->header.stamp;
        inflate_msg.header.frame_id = config.odomFrame;
        inflate_map_pub.publish(inflate_msg);

        last_local_map_update_time = msg->header.stamp;

        // 新增：局部地图更新后检查是否需要重规划
        if (hasTarget)
        {
            tryReplan(msg->header.stamp);
        }
    }
    // ============================================
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
        ROS_WARN("Map Initialized !!!");
    }
}

void MavGlobalPlanner::targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!mapInitialized || !localMapInitialized || !odomInitialized)
    {
        ROS_WARN("Waiting for odom, global map and local map.");
        return;
    }

    double zGoal = fabs(msg->pose.orientation.z) *
                       (config.r3Bound[5] - config.r3Bound[4] - 2 * config.r3SafeRadius) +
                   config.r3Bound[4] + config.r3SafeRadius;

    globalGoal = Eigen::Vector3d(msg->pose.position.x,
                                 msg->pose.position.y,
                                 zGoal);

    hasTarget = true;
    ROS_WARN("[Replan] New global goal received.");

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
    if (!hasTarget || !odomInitialized || !mapInitialized || !localMapInitialized)
    {
        return;
    }

    // if ((globalGoal - curOdomPose.translation()).norm() < config.r3SafeRadius)
    if ((globalGoal - curOdomPose.translation()).norm() < 0.1)
    {
        ROS_WARN("[Replan] Global goal reached.");
        hasTarget = false;
        return;
    }

    bool needReplan = false;

    // 1. 当前没有轨迹，必须规划
    if (!hasActiveTraj)
    {
        needReplan = true;
    }

    // 2. 当前轨迹未来一段时间不安全，立即重规划
    if (hasActiveTraj && !checkCurrentTrajSafe(stamp))
    {
        ROS_WARN("[Replan] Current trajectory is unsafe. Trigger replanning.");
        needReplan = true;
    }

    // 3. 周期性重规划
    if ((stamp - lastReplanTime).toSec() > config.replan_dt)
    {
        needReplan = true;
    }

    if (!needReplan)
    {
        return;
    }

    Eigen::Vector3d startPos = curOdomPose.translation();
    Eigen::Vector3d startVel = curOdomVel;
    Eigen::Vector3d startAcc = accInitialized ? curOdomAcc : Eigen::Vector3d::Zero();

    Eigen::Vector3d localGoal = selectLocalGoal(startPos);

    bool success = planAndPublishLocalTraj(startPos, startVel, startAcc, localGoal, stamp);

    if (success)
    {
        lastReplanTime = stamp;
    }
    else
    {
        ROS_WARN("[Replan] Local replanning failed.");
    }
}

Eigen::Vector3d MavGlobalPlanner::selectLocalGoal(const Eigen::Vector3d &startPos) const
{
    Eigen::Vector3d diff = globalGoal - startPos;
    double dist = diff.norm();

    if (dist < 1e-3)
    {
        return globalGoal;
    }

    double localGoalDist = std::min(dist, config.local_goal_ratio * config.sensingRadius);

    Eigen::Vector3d localGoal = startPos + diff.normalized() * localGoalDist;

    // 保持目标高度朝 globalGoal 过渡
    localGoal(2) = globalGoal(2);

    // 限制在全局高度边界内
    localGoal(2) = std::max(config.r3Bound[4] + config.r3SafeRadius,
                            std::min(config.r3Bound[5] - config.r3SafeRadius,
                                     localGoal(2)));

    return localGoal;
}

bool MavGlobalPlanner::planAndPublishLocalTraj(const Eigen::Vector3d &startPos,
                                               const Eigen::Vector3d &startVel,
                                               const Eigen::Vector3d &startAcc,
                                               const Eigen::Vector3d &goal,
                                               const ros::Time &stamp)
{
    std::vector<Eigen::Vector3d> route;

    r3planner.planOnce(startPos, goal, route);

    if (route.size() <= 1)
    {
        ROS_WARN("[Replan] R3Planner failed.");
        return false;
    }

    Eigen::Vector3d finVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d finAcc = Eigen::Vector3d::Zero();

    Trajectory traj = trajGen.generate(route,
                                       startVel,
                                       startAcc,
                                       finVel,
                                       finAcc,
                                       config.alg,
                                       visualization);

    if (traj.getPieceNum() <= 0)
    {
        ROS_WARN("[Replan] TrajGen failed.");
        return false;
    }

    quadrotor_msgs::PolynomialTrajectory trajMsg;
    ros::Time trajStartStamp = stamp;

    polynomialTrajConverter(traj,
                            trajMsg,
                            Eigen::Isometry3d::Identity(),
                            trajStartStamp);

    trajPub.publish(trajMsg);
    visualization.visualize(traj, route, ros::Time::now(), 1);

    currentTraj = traj;
    currentTrajStartTime = trajStartStamp;
    hasActiveTraj = true;

    ROS_WARN("[Replan] New local trajectory published. Duration = %.3f",
             traj.getTotalDuration());

    return true;
}

bool MavGlobalPlanner::checkCurrentTrajSafe(const ros::Time &stamp) const
{
    if (!hasActiveTraj || currentTraj.getPieceNum() <= 0)
    {
        return false;
    }

    double t_now = (stamp - currentTrajStartTime).toSec();

    if (t_now < 0.0)
    {
        t_now = 0.0;
    }

    double totalT = currentTraj.getTotalDuration();

    if (t_now >= totalT)
    {
        return false;
    }

    double t_end = std::min(totalT, t_now + config.collision_check_horizon);

    for (double t = t_now; t <= t_end; t += config.collision_check_dt)
    {
        Eigen::Vector3d p = currentTraj.getPos(t);

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
