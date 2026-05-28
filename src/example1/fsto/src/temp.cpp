bool MavGlobalPlanner::planAndPublishLocalTraj(const Eigen::Vector3d &startPos,
                                               const Eigen::Vector3d &startVel,
                                               const Eigen::Vector3d &startAcc,
                                               const Eigen::Vector3d &goal,
                                               const ros::Time &trajStartStamp)
{
    // ----------------------- 路径规划 r3planner ----------------------- //
    std::vector<Eigen::Vector3d> route;

    // std::cout << "[planAndPublishLocalTraj] start of r3planner:" << startPos << "goal of r3planner:" << goal << std::endl;
    double rrt_cost = r3planner.planOnce(startPos, startVel, goal, route);
    
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
    // visualization.visualizeRoute(route, ros::Time::now(), 1);

    // LOS简化路径
    route = trajGen.routeSimplify(route, config.spatialResolution);
    if (route.size() <= 1)
    {
        ROS_WARN("[planAndPublishLocalTraj] Route invalid after LOS simplify.");
        return false;
    }
    visualization.visualizeRoute(route, ros::Time::now(), 2);

    //----------------------- 设置局部目标点的速度 ----------------------- //
    Eigen::Vector3d finVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d finAcc = Eigen::Vector3d::Zero();
    const double global_goal_thresh = std::max(config.spatialResolution, config.waypoint_reach_thresh);
    const bool localGoalIsGlobalGoal = ((goal - globalGoal).norm() < global_goal_thresh);

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

        // 2. 如果退化，使用 goal - startPos
        if (tangent.norm() < 1e-3)
        {
            Eigen::Vector3d diff = goal - startPos;
            if (diff.norm() > 1e-3)
            {
                tangent = diff.normalized();
            }
        }

        // 3. 如果仍退化，使用 startVel
        if (tangent.norm() < 1e-3 && startVel.norm() > 1e-3)
        {
            tangent = startVel.normalized();
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
             "vx=%.3f, vy=%.3f, vz=%.3f, speed=%.3f", finVel(0), finVel(1), finVel(2), finVel.norm());
    
    // ----------------------- 轨迹规划 AM ----------------------- //
    Trajectory traj = trajGen.generate(route,
                                       startVel, startAcc,
                                       finVel, finAcc,
                                       config.alg, visualization);

    // ----------------------- 轨迹发布 ----------------------- //
    if (traj.getPieceNum() <= 0)
    {
        ROS_WARN("[planAndPublishLocalTraj] TrajGen failed.");
        return false;
    }

    if (!shouldReplaceCurrentTraj(traj, trajStartStamp, odomStamp)) return false;
        
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