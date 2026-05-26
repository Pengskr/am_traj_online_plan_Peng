#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>

#include <ros/ros.h>

struct Config
{
    // Subscribed Topics
    std::string odomTopic;
    std::string imuTopic;
    std::string mapTopic;
    std::string targetTopic;
    std::string trajTriggerTopic;

    // Advertised Topics
    std::string trajectoryTopic;
    std::string autoManualTopic;
    std::string inflateMapTopic;

    // Frame Names
    std::string odomFrame;

    // Params
    double unitScaleInSI;
    double r3SafeRadius;
    double bodySafeRadius;
    double sensingRadius;
    double edfResolution;
    std::vector<double> r3Bound;
    double searchDuration;
    int tryOut;
    double maxAccRate;
    double maxVelRate;
    double weightT;
    double weightAcc;
    double weightJerk;
    int iterations;
    double temporalResolution;
    double spatialResolution;
    double epsilon;
    double local_map_update_dt;
    double replan_dt;
    double collision_check_horizon;
    double collision_check_dt;
    double local_goal_ratio;
    double replan_time_ahead;
    double min_traj_remaining_time;
    double min_replan_interval;
    double emergency_stop_duration;
    double local_goal_sample_angle;
    double alg;

    static void loadParameters(Config &conf, const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("OdomTopic", conf.odomTopic);
        nh_priv.getParam("ImuTopic", conf.imuTopic);
        nh_priv.getParam("MapTopic", conf.mapTopic);
        nh_priv.getParam("TargetTopic", conf.targetTopic);
        nh_priv.getParam("TrajTriggerTopic", conf.trajTriggerTopic);
        nh_priv.getParam("TrajectoryTopic", conf.trajectoryTopic);
        nh_priv.getParam("AutoManualTopic", conf.autoManualTopic);
        nh_priv.getParam("InflateMapTopic", conf.inflateMapTopic);       
        nh_priv.getParam("OdomFrame", conf.odomFrame);
        nh_priv.getParam("UnitScaleInSI", conf.unitScaleInSI);
        nh_priv.getParam("R3SafeRadius", conf.r3SafeRadius);
        nh_priv.getParam("BodySafeRadius", conf.bodySafeRadius);
        nh_priv.getParam("SensingRadius", conf.sensingRadius);
        nh_priv.getParam("EdfResolution", conf.edfResolution);
        nh_priv.getParam("R3Bound", conf.r3Bound);
        nh_priv.getParam("SearchDuration", conf.searchDuration);
        nh_priv.getParam("TryOut", conf.tryOut);
        nh_priv.getParam("MaxAccRate", conf.maxAccRate);
        nh_priv.getParam("MaxVelRate", conf.maxVelRate);
        nh_priv.getParam("WeightT", conf.weightT);
        nh_priv.getParam("WeightAcc", conf.weightAcc);
        nh_priv.getParam("WeightJerk", conf.weightJerk);
        nh_priv.getParam("Iterations", conf.iterations);
        nh_priv.getParam("TemporalResolution", conf.temporalResolution);
        nh_priv.getParam("SpatialResolution", conf.spatialResolution);
        nh_priv.getParam("Epsilon", conf.epsilon);
        nh_priv.getParam("Local_map_update_dt", conf.local_map_update_dt);
        nh_priv.getParam("Replan_dt", conf.replan_dt);
        nh_priv.getParam("Collision_check_horizon", conf.collision_check_horizon);
        nh_priv.getParam("Collision_check_dt", conf.collision_check_dt);
        nh_priv.getParam("Local_goal_ratio", conf.local_goal_ratio);
        nh_priv.getParam("Replan_time_ahead", conf.replan_time_ahead);
        nh_priv.getParam("Min_traj_remaining_time", conf.min_traj_remaining_time);
        nh_priv.getParam("Min_replan_interval", conf.min_replan_interval);
        nh_priv.getParam("Emergency_stop_duration", conf.emergency_stop_duration);
        nh_priv.getParam("Local_goal_sample_angle", conf.local_goal_sample_angle);
        nh_priv.getParam("Alg", conf.alg);
    }
};

#endif