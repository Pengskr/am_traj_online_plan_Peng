#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include "fsto/config.h"
#include "am_traj/am_traj.hpp"
#include "fsto/glbmap.h"

#include <Eigen/Eigen>

class Visualization
{
public:
    Visualization(Config &conf, ros::NodeHandle &nh_);

    Config config;
    ros::NodeHandle nh;

    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher appliedTrajectoryPub;

    void visualize(const Trajectory &appliedTraj, const std::vector<Eigen::Vector3d> &route, ros::Time timeStamp, int id);
};

class TrajGen
{
public:
    TrajGen(const Config &conf, std::shared_ptr<const GlobalMap> mapPtr);
    Trajectory generate(std::vector<Eigen::Vector3d> &route,
                        Eigen::Vector3d initialVel,
                        Eigen::Vector3d initialAcc,
                        Eigen::Vector3d finalVel,
                        Eigen::Vector3d finalAcc,
                        int id_alg,
                        Visualization visualization) const;

public:
    Config config;
    std::shared_ptr<const GlobalMap> glbMapPtr;
    AmTraj trajOpt;

    bool trajSafeCheck(const Trajectory &traj, std::vector<Eigen::Vector3d> &route) const;
    std::vector<Eigen::Vector3d> routeSimplify(const std::vector<Eigen::Vector3d> &route, double resolution) const;
};

#endif
