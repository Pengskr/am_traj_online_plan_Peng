#include "am_traj/am_traj.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <fstream>
#include <iomanip>

using namespace std;
using namespace ros;
using namespace Eigen;

class Config
{
public:
    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("TrajectoryTopic", trajectoryTopic);
        nh_priv.getParam("WayPointsTopic", wayPointsTopic);
        nh_priv.getParam("RouteTopic", routeTopic);
        nh_priv.getParam("AccTopic", accTopic);
        nh_priv.getParam("FrameName", frameName);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("WeightAcc", weightAcc);
        nh_priv.getParam("WeightJerk", weightJerk);
        nh_priv.getParam("MaxAccRate", maxAccRate);
        nh_priv.getParam("MaxVelRate", maxVelRate);
        nh_priv.getParam("Iterations", iterations);
        nh_priv.getParam("Epsilon", epsilon);
    }

    // Advertised Topics
    string trajectoryTopic;
    string wayPointsTopic;
    string routeTopic;
    string accTopic;

    // Frame Name
    std::string frameName;

    // Params
    double weightT;
    double weightAcc;
    double weightJerk;
    double maxAccRate;
    double maxVelRate;
    int iterations;
    double epsilon;
};

class Visualizer
{
public:
    Visualizer(const Config &conf, ros::NodeHandle &nh) : config(conf)
    {
        trajectoryPub = nh.advertise<visualization_msgs::Marker>(config.trajectoryTopic, 1);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>(config.wayPointsTopic, 1);
        routePub = nh.advertise<visualization_msgs::Marker>(config.routeTopic, 1);
        accPub = nh.advertise<visualization_msgs::MarkerArray>(config.accTopic, 1);
    }

private:
    Config config;
    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher accPub;

public:
    void visualize(const Trajectory &traj, const vector<Vector3d> &route, int id = 0)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker, accMarker;
        visualization_msgs::MarkerArray accMarkers;

        routeMarker.id = id;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = config.frameName;
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.05;

        wayPointsMarker = routeMarker;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 0.20;
        wayPointsMarker.color.g = 0.20;
        wayPointsMarker.color.b = 0.80;
        wayPointsMarker.scale.x = 0.50;
        wayPointsMarker.scale.y = 0.50;
        wayPointsMarker.scale.z = 0.50;

        trajMarker = routeMarker;
        trajMarker.ns = "trajectory";
        trajMarker.scale.x = 0.15;
        if (id == 0)
        {
            trajMarker.color.r = 0.85;
            trajMarker.color.g = 0.10;
            trajMarker.color.b = 0.10;
        }
        else if (id == 1)
        {
            trajMarker.color.r = 1.00;
            trajMarker.color.g = 0.65;
            trajMarker.color.b = 0.00;
        }        
        else if (id == 2)
        {
            trajMarker.color.r = 0.00;
            trajMarker.color.g = 0.45;
            trajMarker.color.b = 0.74;
        }
        else if (id == 3)
        {
            trajMarker.color.r = 1.00;
            trajMarker.color.g = 0.00;
            trajMarker.color.b = 1.00;            
        }
        else if (id == 4)
        {
            trajMarker.color.r = 0.10;
            trajMarker.color.g = 0.65;
            trajMarker.color.b = 0.10;
        }
        else if (id == 5)
        {
            trajMarker.color.r = 0.00;
            trajMarker.color.g = 0.00;
            trajMarker.color.b = 0.00;
        }
        else
        {
            trajMarker.color.r = 0.93;
            trajMarker.color.g = 0.48;
            trajMarker.color.b = 0.26;
        }


        accMarker = routeMarker;
        accMarker.type = visualization_msgs::Marker::ARROW;
        accMarker.header.stamp = ros::Time::now();
        accMarker.ns = "acc";
        if (id == 0)
        {
            accMarker.color.r = 255.0 / 255.0;
            accMarker.color.g = 20.0 / 255.0;
            accMarker.color.b = 147.0 / 255.0;
        }
        else if (id == 1)
        {
            accMarker.color.r = 60.0 / 255.0;
            accMarker.color.g = 179.0 / 255.0;
            accMarker.color.b = 113.0 / 255.0;
        }
        else if (id == 2)
        {
            accMarker.color.r = 30.0 / 255.0;
            accMarker.color.g = 144.0 / 255.0;
            accMarker.color.b = 255.0 / 255.0;
        } 
        else if (id == 3)
        {
            accMarker.color.r = 100.0 / 255.0;
            accMarker.color.g = 144.0 / 255.0;
            accMarker.color.b = 255.0 / 255.0;            
        }
        else
        {
            accMarker.color.r = 179.0 / 255.0;
            accMarker.color.g = 179.0 / 255.0;
            accMarker.color.b = 113.0 / 255.0;
        }
        accMarker.scale.x = 0.05;
        accMarker.scale.y = 0.15;
        accMarker.scale.z = 0.30;

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

        if (traj.getPieceNum() > 0)
        {
            double T = 0.01;
            Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            trajectoryPub.publish(trajMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            if (id == 0)
            {
                accMarker.action = visualization_msgs::Marker::DELETEALL;   // 发送一个 DELETEALL 动作的加速度标记，以确保清除上一次发布的所有旧加速度标记
                accMarkers.markers.push_back(accMarker);
                accPub.publish(accMarkers);
                accMarkers.markers.clear();
                accMarker.action = visualization_msgs::Marker::ADD;
            }

            double T = 0.10;
            for (double t = 0; t < traj.getTotalDuration(); t += T)
            {
                accMarker.id += 3;
                accMarker.points.clear();
                geometry_msgs::Point point;
                Vector3d X = traj.getPos(t);    // 加速度起点：当前位置
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                accMarker.points.push_back(point);
                X += traj.getAcc(t); // 加速度终点：位置向量+加速度向量等
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                accMarker.points.push_back(point);
                accMarkers.markers.push_back(accMarker);
            }
            accPub.publish(accMarkers);
        }
    }
};

class RandomRouteGenerator
{
public:
    RandomRouteGenerator(Array3d l, Array3d u)
        : lBound(l), uBound(u), uniformReal(0.0, 1.0) {}

    inline vector<Vector3d> generate(int N)
    {
        vector<Vector3d> route;
        Array3d temp;
        route.emplace_back(0.0, 0.0, 0.0);  // 起点设为 (0,0,0)
        for (int i = 0; i < N; i++)
        {
            temp << uniformReal(gen), uniformReal(gen), uniformReal(gen);
            temp = (uBound - lBound) * temp + lBound;
            route.emplace_back(temp);
        }
        return route;
    }

private:
    Array3d lBound;
    Array3d uBound;
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> uniformReal;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example0_node");
    ros::NodeHandle nh_, nh_priv("~");

    ros::Duration(1).sleep();    // 等待一会儿，rviz初始化

    Config config(nh_priv);
    Visualizer viz(config, nh_);    // 可视化类
    RandomRouteGenerator routeGen(Array3d(-16, -16, 0), Array3d(16, 16, 16));
    AmTraj amTrajOpt(config.weightT, config.weightAcc, config.weightJerk,
                     config.maxVelRate, config.maxAccRate, config.iterations, config.epsilon);

    vector<Vector3d> route;
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Trajectory traj, traj_GD;
    Rate lp(0.25);
    int M_max = 7;
    int groupSize = 100;
    std::vector<double> durs;
    durs.clear();

    std::chrono::high_resolution_clock::time_point tc0, tc1;
    double d0, d1, d2, d3, d4, d5, d6, d0_sum, d1_sum, d2_sum, d3_sum, d4_sum, d5_sum, d6_sum, d0_mean = 0.0, d1_mean = 0.0, d2_mean = 0.0, d3_mean = 0.0, d4_mean = 0.0, d5_mean = 0.0, d6_mean = 0.0;
    double t_lap_0, t_lap_1, t_lap_2, t_lap_3, t_lap_4, t_lap_5,t_lap_6, t_lap_sum_0, t_lap_sum_1, t_lap_sum_2, t_lap_sum_3, t_lap_sum_4, t_lap_sum_5, t_lap_sum_6, t_lap_mean_0, t_lap_mean_1, t_lap_mean_2, t_lap_mean_3, t_lap_mean_4, t_lap_mean_5, t_lap_mean_6;
    double cost_0, cost_1, cost_2, cost_3, cost_4, cost_5, cost_6, cost_sum_0, cost_sum_1, cost_sum_2, cost_sum_3, cost_sum_4, cost_sum_5, cost_sum_6, cost_mean_0, cost_mean_1, cost_mean_2, cost_mean_3, cost_mean_4, cost_mean_5, cost_mean_6;
    double v_max_0, v_max_1, v_max_2, v_max_3, v_max_4, v_max_5, v_max_6, v_max_sum_0, v_max_sum_1, v_max_sum_2, v_max_sum_3, v_max_sum_4, v_max_sum_5, v_max_sum_6, v_max_mean_0, v_max_mean_1, v_max_mean_2, v_max_mean_3, v_max_mean_4, v_max_mean_5, v_max_mean_6;
    double a_max_0, a_max_1, a_max_2, a_max_3, a_max_4, a_max_5, a_max_6, a_max_sum_0, a_max_sum_1, a_max_sum_2, a_max_sum_3, a_max_sum_4, a_max_sum_5, a_max_sum_6, a_max_mean_0, a_max_mean_1, a_max_mean_2, a_max_mean_3, a_max_mean_4, a_max_mean_5, a_max_mean_6;

    // 保存csv文件：每个方法的结果保存为1个csv
    // 打开7个输出文件（每种方法一个文件）
    std::string pkg_path = ros::package::getPath("example2_1");
    std::string result_dir = pkg_path + "/results/";

    std::ofstream csv_red(result_dir + "RED_Un-constrained-AM.csv");
    std::ofstream csv_yellow(result_dir + "YELLOW_constrained-AM.csv");
    std::ofstream csv_blue(result_dir + "BLUE_Un-constrained-NLOPT.csv");
    std::ofstream csv_purple(result_dir + "PURPLE_constrained-NLOPT.csv");
    std::ofstream csv_green(result_dir + "GREEN_constrained-AM_with_whole_scale.csv");
    std::ofstream csv_black(result_dir + "BLACK_Un-constrained-AM_old.csv");
    std::ofstream csv_orange(result_dir + "ORANGE_constrained-AM_old.csv");

    for (int M = 6; M < M_max && ok(); M++)    // 段数
    {
        d0_sum = 0.0; d1_sum = 0.0; d2_sum = 0.0; d3_sum = 0.0; d4_sum = 0.0; d5_sum = 0.0; d6_sum = 0.0;
        t_lap_sum_0 = 0.0; t_lap_sum_1 = 0.0; t_lap_sum_2 = 0.0; t_lap_sum_3 = 0.0; t_lap_sum_4 = 0.0; t_lap_sum_5 = 0.0; t_lap_sum_6 = 0.0;
        cost_sum_0 = 0.0, cost_sum_1 = 0.0, cost_sum_2 = 0.0, cost_sum_3 = 0.0, cost_sum_4 = 0.0, cost_sum_5 = 0.0, cost_sum_6 = 0.0; 
        v_max_sum_0 = 0.0, v_max_sum_1 = 0.0, v_max_sum_2 = 0.0, v_max_sum_3 = 0.0, v_max_sum_4 = 0.0, v_max_sum_5 = 0.0, v_max_sum_6 = 0.0;
        a_max_sum_0 = 0.0, a_max_sum_1 = 0.0, a_max_sum_2 = 0.0, a_max_sum_3 = 0.0, a_max_sum_4 = 0.0, a_max_sum_5 = 0.0, a_max_sum_6 = 0.0;

        for (int j = 0; j < groupSize && ok(); j++)
        {
            std::ofstream csv_red_vel(result_dir + "RED_Un-constrained-AM_vel.csv");
            std::ofstream csv_red_acc(result_dir + "RED_Un-constrained-AM_acc.csv");
            std::ofstream csv_yellow_vel(result_dir + "YELLOW_constrained-AM-vel.csv");
            std::ofstream csv_yellow_acc(result_dir + "YELLOW_constrained-AM-acc.csv");
            std::ofstream csv_blue_vel(result_dir + "BLUE_Un-constrained-NLOPT-vel.csv");
            std::ofstream csv_blue_acc(result_dir + "BLUE_Un-constrained-NLOPT-acc.csv");
            std::ofstream csv_purple_vel(result_dir + "PURPLE_constrained-NLOPT-vel.csv");
            std::ofstream csv_purple_acc(result_dir + "PURPLE_constrained-NLOPT-acc.csv");            
            std::ofstream csv_green_vel(result_dir + "GREEN_constrained-AM_with_whole_scale-vel.csv");
            std::ofstream csv_green_acc(result_dir + "GREEN_constrained-AM_with_whole_scale-acc.csv");
            std::ofstream csv_black_vel(result_dir + "BLACK_Un-constrained-AM_vel_old.csv");
            std::ofstream csv_black_acc(result_dir + "BLACK_Un-constrained-AM_acc_old.csv");
            std::ofstream csv_orange_vel(result_dir + "ORANGE_constrained-AM-vel_old.csv");
            std::ofstream csv_orange_acc(result_dir + "ORANGE_constrained-AM-acc_old.csv");


            std::cout << "---------------------------------------------------------------------------------------" << std::endl;
            std::cout << "Number of Segments: " << M << ", Group: " << j+1 << std::endl;

            route = routeGen.generate(M);

            amTrajOpt.maxIterations = config.iterations;
            amTrajOpt.epsilon = config.epsilon;
            tc0 = std::chrono::high_resolution_clock::now();
            traj = amTrajOpt.genOptimalTrajDTs3(route, zeroVec, zeroVec, zeroVec, zeroVec);   // 只实现了 s=3
            tc1 = std::chrono::high_resolution_clock::now();
            d0 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count(); 
            d0_sum += d0;
            t_lap_0 = traj.getTotalDuration(); 
            t_lap_sum_0 += t_lap_0;
            cost_0 = amTrajOpt.evaluateObjective(traj);
            cost_sum_0 += cost_0;
            v_max_0 = traj.getMaxVelRate();
            v_max_sum_0 += v_max_0;
            a_max_0 = traj.getMaxAccRate();
            a_max_sum_0 += a_max_0;
            viz.visualize(traj, route, 0);
            std::cout << "---------------------------------------------------------------------------------------" << std::endl;
            std::cout << "RED: Un-constrained AM Spatial-Temporal Optimal Trajectory" << std::endl
                      << "      Planning time:" << d0*1000 << " ms" << std::endl
                      << "      Lap Time: " << t_lap_0 << " s" << std::endl
                      << "      Cost: " << cost_0 << std::endl
                      << "      Maximum Velocity Rate: " << v_max_0 << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << a_max_0 << " m/s^2" << std::endl;
            // 保存 速度，加速度曲线，用于MATLAB绘制
            for(double t_cur = 0.0; t_cur <= t_lap_0; t_cur += 0.01)
            {
                
                csv_red_vel << t_cur << "," << traj.getVel(t_cur).norm() << "\n";
                csv_red_acc << t_cur << "," << traj.getAcc(t_cur).norm() << "\n";
            }
            csv_red_vel.close();
            csv_red_acc.close();



            tc0 = std::chrono::high_resolution_clock::now();
            traj = amTrajOpt.genOptimalTrajDTCs3(route, zeroVec, zeroVec, zeroVec, zeroVec);  // 只实现了 s=3
            tc1 = std::chrono::high_resolution_clock::now();
            d1 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count(); 
            d1_sum += d1;
            t_lap_1 = traj.getTotalDuration();
            t_lap_sum_1 += t_lap_1;
            cost_1 = amTrajOpt.evaluateObjective(traj);
            cost_sum_1 += cost_1;
            v_max_1 = traj.getMaxVelRate();
            v_max_sum_1 += v_max_1;
            a_max_1 = traj.getMaxAccRate();
            a_max_sum_1 += a_max_1;
            viz.visualize(traj, route, 1);
            std::cout << "YELLOW: Constrained AM Spatial-Temporal Optimal Trajectory" << std::endl
                      << "      Planning time:" << d1*1000 << " ms" << std::endl
                      << "      Lap Time: " << t_lap_1 << " s" << std::endl
                      << "      Cost: " << cost_1 << std::endl
                      << "      Maximum Velocity Rate: " << v_max_1 << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << a_max_1 << " m/s^2" << std::endl;
            // 保存 速度，加速度曲线，用于MATLAB绘制
            for(double t_cur = 0.0; t_cur <= t_lap_1; t_cur += 0.01)
            {
                
                csv_yellow_vel << t_cur << "," << traj.getVel(t_cur).norm() << "\n";
                csv_yellow_acc << t_cur << "," << traj.getAcc(t_cur).norm() << "\n";
            }
            csv_yellow_vel.close();
            csv_yellow_acc.close();
            


            // tc0 = std::chrono::high_resolution_clock::now();
            // traj = amTrajOpt.genOptimalTrajDs3(route, zeroVec, zeroVec, zeroVec, zeroVec);   // 只实现了 s=3
            // tc1 = std::chrono::high_resolution_clock::now();
            // d2 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count(); 
            // d2_sum += d2;
            // t_lap_2 = traj.getTotalDuration();
            // t_lap_sum_2 += t_lap_2;
            // cost_2 = amTrajOpt.evaluateObjective(traj);
            // cost_sum_2 += cost_2;
            // v_max_2 = traj.getMaxVelRate();
            // v_max_sum_2 += v_max_2;
            // a_max_2 = traj.getMaxAccRate();
            // a_max_sum_2 += a_max_2;
            // // viz.visualize(traj, route, 2);
            // // std::cout << "BLUE: Un-Constrained Spatial Optimal Trajectory with Trapezoidal Time Allocation" << std::endl
            // //           << "      Planning time:" << d2*1000 << " ms" << std::endl
            // //           << "      Lap Time: " << t_lap_2 << " s" << std::endl
            // //           << "      Cost: " << cost_2 << std::endl
            // //           << "      Maximum Velocity Rate: " << v_max_2 << " m/s" << std::endl
            // //           << "      Maximum Acceleration Rate: " << a_max_2 << " m/s^2" << std::endl;

            mav_trajectory_generation::Vertex::Vector vertices;
            const int dimension = 3;
            // const int derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;
            const int derivative_to_optimize = 2;
            mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

            start.makeStartOrEnd(route[0], derivative_to_optimize);
            vertices.push_back(start);
            for (size_t k = 1; k < route.size()-1; k++)
            {
                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, route[k]);
                vertices.push_back(middle);
            }
            end.makeStartOrEnd(route[route.size()-1], derivative_to_optimize);
            vertices.push_back(end);

            mav_trajectory_generation::NonlinearOptimizationParameters parameters;
            parameters.max_iterations = 23;
            parameters.f_rel = 0.05;
            parameters.x_rel = 0.1;
            parameters.time_penalty = config.weightT;
            parameters.use_soft_constraints = false;
            parameters.print_debug_info = false;
            parameters.print_debug_info_time_allocation = false;
            parameters.initial_stepsize_rel = 0.1;
            parameters.inequality_constraint_tolerance = 0.1;
            parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kRichterTimeAndConstraints;
            
            std::vector<double> segment_times;
            const double v_max = config.maxVelRate;
            const double a_max = config.maxAccRate;

            tc0 = std::chrono::high_resolution_clock::now();
            segment_times = amTrajOpt.allocateTime(route, 1.0);
            const int N = 6;
            mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
            opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
            // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
            // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
            opt.optimize();
            tc1 = std::chrono::high_resolution_clock::now();
            mav_traj2am_traj(opt, durs, traj_GD);
            d2 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count(); 
            d2_sum += d2;
            t_lap_2 = traj_GD.getTotalDuration();
            t_lap_sum_2 += t_lap_2;
            cost_2 = amTrajOpt.evaluateObjective(traj_GD);
            cost_sum_2 += cost_2;
            v_max_2 = traj_GD.getMaxVelRate();
            v_max_sum_2 += v_max_2;
            a_max_2 = traj_GD.getMaxAccRate();
            a_max_sum_2 += a_max_2;
            viz.visualize(traj_GD, route, 2);
            std::cout << "BLUE: Un-constrained-NLOPT" << std::endl
                      << "      Planning time:" << d2*1000 << " ms" << std::endl
                      << "      Lap Time: " << t_lap_2 << " s" << std::endl
                      << "      Cost: " << cost_2 << std::endl
                      << "      Maximum Velocity Rate: " << v_max_2 << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << a_max_2 << " m/s^2" << std::endl;
            // 保存 速度，加速度曲线，用于MATLAB绘制
            for(double t_cur = 0.0; t_cur <= t_lap_2; t_cur += 0.01)
            {
                
                csv_blue_vel << t_cur << "," << traj_GD.getVel(t_cur).norm() << "\n";
                csv_blue_acc << t_cur << "," << traj_GD.getAcc(t_cur).norm() << "\n";
            }
            csv_blue_vel.close();
            csv_blue_acc.close();


            // tc0 = std::chrono::high_resolution_clock::now();
            // traj = amTrajOpt.genOptimalTrajDCs3(route, zeroVec, zeroVec, zeroVec, zeroVec);   // 只实现了 s=3
            // tc1 = std::chrono::high_resolution_clock::now();
            // d3 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count();
            // d3_sum += d3;
            // t_lap_3 = traj.getTotalDuration();
            // t_lap_sum_3 += t_lap_3;
            // cost_3 = amTrajOpt.evaluateObjective(traj);
            // cost_sum_3 += cost_3;
            // v_max_3 = traj.getMaxVelRate();
            // v_max_sum_3 += v_max_3;
            // a_max_3 = traj.getMaxAccRate();
            // a_max_sum_3 += a_max_3;
            // // viz.visualize(traj, route, 3);
            // // std::cout << "PURPLE: Constrained Spatial Optimal Trajectory with Trapezoidal Time Allocation" << std::endl
            // //           << "      Planning time:" << d3*1000 << " ms" << std::endl
            // //           << "      Lap Time: " << t_lap_3 << " s" << std::endl
            // //           << "      Cost: " << cost_3 << std::endl
            // //           << "      Maximum Velocity Rate: " << v_max_3 << " m/s" << std::endl
            // //           << "      Maximum Acceleration Rate: " << a_max_3 << " m/s^2" << std::endl;
            parameters.use_soft_constraints = true;
            mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt_cons(dimension, parameters);
            opt_cons.setupFromVertices(vertices, segment_times, derivative_to_optimize);
            opt_cons.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
            opt_cons.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
            opt_cons.optimize();
            tc1 = std::chrono::high_resolution_clock::now();
            mav_traj2am_traj(opt_cons, durs, traj_GD);
            //软约束，常违背约束
            double ratio = std::max(traj_GD.getMaxVelRate() / v_max / (1.0 - config.epsilon * config.epsilon),
                                    sqrt(traj_GD.getMaxAccRate() / a_max / (1.0 - config.epsilon * config.epsilon)));
            traj_GD.scaleTime(1 / ratio);
            //    
            d3 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count();
            d3_sum += d3;
            t_lap_3 = traj_GD.getTotalDuration();
            t_lap_sum_3 += t_lap_3;
            cost_3 = amTrajOpt.evaluateObjective(traj_GD);
            cost_sum_3 += cost_3;
            v_max_3 = traj_GD.getMaxVelRate();
            v_max_sum_3 += v_max_3;
            a_max_3 = traj_GD.getMaxAccRate();
            a_max_sum_3 += a_max_3;
            viz.visualize(traj_GD, route, 3);
            std::cout << "PURPLE: Constrained Spatial Optimal Trajectory with Trapezoidal Time Allocation" << std::endl
                      << "      Planning time:" << d3*1000 << " ms" << std::endl
                      << "      Lap Time: " << t_lap_3 << " s" << std::endl
                      << "      Cost: " << cost_3 << std::endl
                      << "      Maximum Velocity Rate: " << v_max_3 << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << a_max_3 << " m/s^2" << std::endl;
            // 保存 速度，加速度曲线，用于MATLAB绘制
            for(double t_cur = 0.0; t_cur <= t_lap_3; t_cur += 0.01)
            {
                
                csv_purple_vel << t_cur << "," << traj_GD.getVel(t_cur).norm() << "\n";
                csv_purple_acc << t_cur << "," << traj_GD.getAcc(t_cur).norm() << "\n";
            }
            csv_purple_vel.close();
            csv_purple_acc.close();


            tc0 = std::chrono::high_resolution_clock::now();
            traj = amTrajOpt.genOptimalTrajDTCWholeScales3(route, zeroVec, zeroVec, zeroVec, zeroVec);   // 只实现了 s=3
            tc1 = std::chrono::high_resolution_clock::now();
            d4 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count();
            d4_sum += d4;
            t_lap_4 = traj.getTotalDuration();
            t_lap_sum_4 += t_lap_4;
            cost_4 = amTrajOpt.evaluateObjective(traj);
            cost_sum_4 += cost_4;
            v_max_4 = traj.getMaxVelRate();
            v_max_sum_4 += v_max_4;
            a_max_4 = traj.getMaxAccRate();
            a_max_sum_4 += a_max_4;
            viz.visualize(traj, route, 4);
            std::cout << "GREEN: Constrained AM Spatial-Temporal Optimal Trajectory with whole scale" << std::endl
                      << "      Planning time:" << d4*1000 << " ms" << std::endl
                      << "      Lap Time: " << t_lap_4 << " s" << std::endl
                      << "      Cost: " << cost_4 << std::endl
                      << "      Maximum Velocity Rate: " << v_max_4 << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << a_max_4 << " m/s^2" << std::endl;
            // 保存 速度，加速度曲线，用于MATLAB绘制
            for(double t_cur = 0.0; t_cur <= t_lap_4; t_cur += 0.01)
            {
                
                csv_green_vel << t_cur << "," << traj.getVel(t_cur).norm() << "\n";
                csv_green_acc << t_cur << "," << traj.getAcc(t_cur).norm() << "\n";
            }
            csv_green_vel.close();
            csv_green_acc.close();


            amTrajOpt.maxIterations = config.iterations * 4;
            amTrajOpt.epsilon = config.epsilon * 0.01;
            tc0 = std::chrono::high_resolution_clock::now();
            traj = amTrajOpt.genOptimalTrajDTs3(route, zeroVec, zeroVec, zeroVec, zeroVec);   // 只实现了 s=3
            tc1 = std::chrono::high_resolution_clock::now();
            d5 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count(); 
            d5_sum += d5;
            t_lap_5 = traj.getTotalDuration(); 
            t_lap_sum_5 += t_lap_5;
            cost_5 = amTrajOpt.evaluateObjective(traj);
            cost_sum_5 += cost_5;
            v_max_5 = traj.getMaxVelRate();
            v_max_sum_5 += v_max_5;
            a_max_5 = traj.getMaxAccRate();
            a_max_sum_5 += a_max_5;
            viz.visualize(traj, route, 5);
            std::cout << "---------------------------------------------------------------------------------------" << std::endl;
            std::cout << "BLACK: Un-constrained AM Spatial-Temporal Optimal Trajectory_old" << std::endl
                      << "      Planning time:" << d5*1000 << " ms" << std::endl
                      << "      Lap Time: " << t_lap_5 << " s" << std::endl
                      << "      Cost: " << cost_5 << std::endl
                      << "      Maximum Velocity Rate: " << v_max_5 << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << a_max_5 << " m/s^2" << std::endl;
            // 保存 速度，加速度曲线，用于MATLAB绘制
            for(double t_cur = 0.0; t_cur <= t_lap_5; t_cur += 0.01)
            {
                
                csv_black_vel << t_cur << "," << traj.getVel(t_cur).norm() << "\n";
                csv_black_acc << t_cur << "," << traj.getAcc(t_cur).norm() << "\n";
            }
            csv_black_vel.close();
            csv_black_acc.close();


            tc0 = std::chrono::high_resolution_clock::now();
            traj = amTrajOpt.genOptimalTrajDTCs3(route, zeroVec, zeroVec, zeroVec, zeroVec);  // 只实现了 s=3
            tc1 = std::chrono::high_resolution_clock::now();
            d6 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count(); 
            d6_sum += d6;
            t_lap_6 = traj.getTotalDuration();
            t_lap_sum_6 += t_lap_6;
            cost_6 = amTrajOpt.evaluateObjective(traj);
            cost_sum_6 += cost_6;
            v_max_6 = traj.getMaxVelRate();
            v_max_sum_6 += v_max_6;
            a_max_6 = traj.getMaxAccRate();
            a_max_sum_6 += a_max_6;
            viz.visualize(traj, route, 6);
            std::cout << "ORANGE: Constrained AM Spatial-Temporal Optimal Trajectory_old" << std::endl
                      << "      Planning time:" << d6*1000 << " ms" << std::endl
                      << "      Lap Time: " << t_lap_6 << " s" << std::endl
                      << "      Cost: " << cost_6 << std::endl
                      << "      Maximum Velocity Rate: " << v_max_6 << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << a_max_6 << " m/s^2" << std::endl;
            // 保存 速度，加速度曲线，用于MATLAB绘制
            for(double t_cur = 0.0; t_cur <= t_lap_6; t_cur += 0.01)
            {
                
                csv_orange_vel << t_cur << "," << traj.getVel(t_cur).norm() << "\n";
                csv_orange_acc << t_cur << "," << traj.getAcc(t_cur).norm() << "\n";
            }
            csv_orange_vel.close();
            csv_orange_acc.close();


            // spinOnce();
            // lp.sleep();
        }

        // 计算平均Planning time，Lap Time，Cost， Maximum Velocity Rate， Maximum Acceleration Rate 并保存为csv
        d0_mean = d0_sum / groupSize;
        d1_mean = d1_sum / groupSize;
        d2_mean = d2_sum / groupSize;
        d3_mean = d3_sum / groupSize;
        d4_mean = d4_sum / groupSize;
        d5_mean = d5_sum / groupSize;
        d6_mean = d6_sum / groupSize;

        t_lap_mean_0 = t_lap_sum_0 / groupSize;
        t_lap_mean_1 = t_lap_sum_1 / groupSize;
        t_lap_mean_2 = t_lap_sum_2 / groupSize;
        t_lap_mean_3 = t_lap_sum_3 / groupSize;
        t_lap_mean_4 = t_lap_sum_4 / groupSize;
        t_lap_mean_5 = t_lap_sum_5 / groupSize;
        t_lap_mean_6 = t_lap_sum_6 / groupSize;

        cost_mean_0 = cost_sum_0 / groupSize;
        cost_mean_1 = cost_sum_1 / groupSize;
        cost_mean_2 = cost_sum_2 / groupSize;
        cost_mean_3 = cost_sum_3 / groupSize;
        cost_mean_4 = cost_sum_4 / groupSize;
        cost_mean_5 = cost_sum_5 / groupSize;
        cost_mean_6 = cost_sum_6 / groupSize;

        v_max_mean_0 = v_max_sum_0 / groupSize;
        v_max_mean_1 = v_max_sum_1 / groupSize;
        v_max_mean_2 = v_max_sum_2 / groupSize;
        v_max_mean_3 = v_max_sum_3 / groupSize;
        v_max_mean_4 = v_max_sum_4 / groupSize;
        v_max_mean_5 = v_max_sum_5 / groupSize;
        v_max_mean_6 = v_max_sum_6 / groupSize;

        a_max_mean_0 = a_max_sum_0 / groupSize;
        a_max_mean_1 = a_max_sum_1 / groupSize;
        a_max_mean_2 = a_max_sum_2 / groupSize;
        a_max_mean_3 = a_max_sum_3 / groupSize;
        a_max_mean_4 = a_max_sum_4 / groupSize;
        a_max_mean_5 = a_max_sum_5 / groupSize;
        a_max_mean_6 = a_max_sum_6 / groupSize;

        std::cout << "-------------------------------------Statistical---------------------------------------" << std::endl;
        std::cout << "RED:" << std::endl
        << "      Planning time mean: " << d0_mean*1000 << " ms" << std::endl
        << "      Lap Time mean: " << t_lap_mean_0 << " s" << std::endl
        << "      Cost mean: " << cost_mean_0 << std::endl
        << "      Maximum Velocity Rate mean: " << v_max_mean_0 << " m/s" << std::endl
        << "      Maximum Acceleration Rate mean: " << a_max_mean_0 << " m/s^2" << std::endl;

        std::cout << "YELLOW:" << std::endl
        << "      Planning time mean: " << d1_mean*1000 << " ms" << std::endl
        << "      Lap Time mean: " << t_lap_mean_1 << " s" << std::endl
        << "      Cost mean: " << cost_mean_1 << std::endl
        << "      Maximum Velocity Rate mean: " << v_max_mean_1 << " m/s" << std::endl
        << "      Maximum Acceleration Rate mean: " << a_max_mean_1 << " m/s^2" << std::endl;

        std::cout << "BLUE:" << std::endl
        << "      Planning time mean: " << d2_mean*1000 << " ms" << std::endl
        << "      Lap Time mean: " << t_lap_mean_2 << " s" << std::endl
        << "      Cost mean: " << cost_mean_2 << std::endl
        << "      Maximum Velocity Rate mean: " << v_max_mean_2 << " m/s" << std::endl
        << "      Maximum Acceleration Rate mean: " << a_max_mean_2 << " m/s^2" << std::endl;

        std::cout << "PURPLE:" << std::endl
        << "      Planning time mean: " << d3_mean*1000 << " ms" << std::endl
        << "      Lap Time mean: " << t_lap_mean_3 << " s" << std::endl
        << "      Cost mean: " << cost_mean_3 << std::endl
        << "      Maximum Velocity Rate mean: " << v_max_mean_3 << " m/s" << std::endl
        << "      Maximum Acceleration Rate mean: " << a_max_mean_3 << " m/s^2" << std::endl;

        std::cout << "GREEN:" << std::endl
        << "      Planning time mean: " << d4_mean*1000 << " ms" << std::endl
        << "      Lap Time mean: " << t_lap_mean_4 << " s" << std::endl
        << "      Cost mean: " << cost_mean_4 << std::endl
        << "      Maximum Velocity Rate mean: " << v_max_mean_4 << " m/s" << std::endl
        << "      Maximum Acceleration Rate mean: " << a_max_mean_4 << " m/s^2" << std::endl;

        std::cout << "BLACK:" << std::endl
        << "      Planning time mean: " << d5_mean*1000 << " ms" << std::endl
        << "      Lap Time mean: " << t_lap_mean_5 << " s" << std::endl
        << "      Cost mean: " << cost_mean_5 << std::endl
        << "      Maximum Velocity Rate mean: " << v_max_mean_5 << " m/s" << std::endl
        << "      Maximum Acceleration Rate mean: " << a_max_mean_5 << " m/s^2" << std::endl;

        std::cout << "ORANGE:" << std::endl
        << "      Planning time mean: " << d6_mean*1000 << " ms" << std::endl
        << "      Lap Time mean: " << t_lap_mean_6 << " s" << std::endl
        << "      Cost mean: " << cost_mean_6 << std::endl
        << "      Maximum Velocity Rate mean: " << v_max_mean_6 << " m/s" << std::endl
        << "      Maximum Acceleration Rate mean: " << a_max_mean_6 << " m/s^2" << std::endl;

        // 将平均值写入 CSV
        csv_red   << M << "," << d0_mean*1000 << "," << t_lap_mean_0 << "," << cost_mean_0 << "," << v_max_mean_0 << "," << a_max_mean_0 << "\n";
        csv_yellow<< M << "," << d1_mean*1000 << "," << t_lap_mean_1 << "," << cost_mean_1 << "," << v_max_mean_1 << "," << a_max_mean_1 << "\n";
        csv_blue  << M << "," << d2_mean*1000 << "," << t_lap_mean_2 << "," << cost_mean_2 << "," << v_max_mean_2 << "," << a_max_mean_2 << "\n";
        csv_purple<< M << "," << d3_mean*1000 << "," << t_lap_mean_3 << "," << cost_mean_3 << "," << v_max_mean_3 << "," << a_max_mean_3 << "\n";
        csv_green << M << "," << d4_mean*1000 << "," << t_lap_mean_4 << "," << cost_mean_4 << "," << v_max_mean_4 << "," << a_max_mean_4 << "\n";
        csv_black << M << "," << d5_mean*1000 << "," << t_lap_mean_5 << "," << cost_mean_5 << "," << v_max_mean_5 << "," << a_max_mean_5 << "\n";
        csv_orange<< M << "," << d6_mean*1000 << "," << t_lap_mean_6 << "," << cost_mean_6 << "," << v_max_mean_6 << "," << a_max_mean_6 << "\n";
    }
    
    csv_red.close();
    csv_yellow.close();
    csv_blue.close();
    csv_purple.close();
    csv_green.close();
    csv_black.close();
    csv_orange.close();

    std::cout << "-------------------------------------" << std::endl;
    std::cout << "All statistical results saved to CSV files." << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    return 0;
}
