#include "fsto/traj_gen.h"
#include <ros/package.h>
#include <fstream>
#include <iomanip>

#include <ctime>

using namespace std;
using namespace Eigen;

TrajGen::TrajGen(const Config &conf, std::shared_ptr<const GlobalMap> mapPtr)
    : config(conf), glbMapPtr(mapPtr),
      trajOpt(config.weightT, config.weightAcc, config.weightJerk,
              config.maxVelRate, config.maxAccRate, config.iterations, config.epsilon)
{
}

Trajectory TrajGen::generate(vector<Vector3d> &route,
                             Vector3d initialVel,
                             Vector3d initialAcc,
                             Vector3d finalVel,
                             Vector3d finalAcc,
                             int id_alg) const
{
    Trajectory traj;

    route = routeSimplify(route, config.spatialResolution);     // 道格拉斯-普克（Douglas-Peucker）算法

    if (route.size() < 2 || (route[0] - route[1]).squaredNorm() < FLT_EPSILON)
    {
        return traj;
    }

    std::string result_dir = "/home/peng/Desktop/am_traj_Peng/src/example1/results/";

    int tries = 0;
    do
    {
        clock_t start = clock();

        tries++;
        if (tries > config.tryOut)
        {
            traj.clear();
            break;
        }

        if(id_alg == 2){
            traj = trajOpt.genOptimalTrajDTCWholeScales3(route, initialVel, initialAcc, finalVel, finalAcc);    // 带时域缩放的有约束交替优化
            clock_t stop = clock();
            cout << "Fast Spatial-Tenporal Traj Opt(AM with scale): " << (stop - start) * 1000.0 / CLOCKS_PER_SEC << " ms" << endl;
            cout << "Number of Traj Pieces: " << traj.getPieceNum() << endl;
            cout << "t_lap: " << traj.getTotalDuration() << endl;
            cout << "cost: " << trajOpt.evaluateObjective(traj) << endl;

            // 保存 位置，速度，加速度曲线，用于MATLAB绘制
            std::ofstream csv_green_pva(result_dir + "GREEN_constrained-AM-pva.csv");
            for(double t_cur = 0.0; t_cur <= traj.getTotalDuration(); t_cur += 0.01)
            {
                Eigen::Vector3d pos = traj.getPos(t_cur);
                double x = pos(0);
                double y = pos(1);
                double z = pos(2);
                csv_green_pva << t_cur << "," << x << "," << y << "," << z << "," << traj.getVel(t_cur).norm() << "," << traj.getAcc(t_cur).norm() << "\n";

            }
            csv_green_pva.close();
            std::cout << "Saved CSV file" << std::endl;
        }
        else if(id_alg == 3){
            traj = trajOpt.genOptimalTrajDTCs3(route, initialVel, initialAcc, finalVel, finalAcc);    //精细化有约束交替优化
            clock_t stop = clock();
            cout << "Fast Spatial-Tenporal Traj Opt(AM): " << (stop - start) * 1000.0 / CLOCKS_PER_SEC << " ms" << endl;
            cout << "Number of Traj Pieces: " << traj.getPieceNum() << endl;
            cout << "t_lap: " << traj.getTotalDuration() << endl;
            cout << "cost: " << trajOpt.evaluateObjective(traj) << endl;

            // 保存 位置，速度，加速度曲线，用于MATLAB绘制
            std::ofstream csv_yellow_pva(result_dir + "YELLOW_constrained-AM-pva.csv");
            for(double t_cur = 0.0; t_cur <= traj.getTotalDuration(); t_cur += 0.01)
            {
                Eigen::Vector3d pos = traj.getPos(t_cur);
                double x = pos(0);
                double y = pos(1);
                double z = pos(2);
                csv_yellow_pva << t_cur << "," << x << "," << y << "," << z << "," << traj.getVel(t_cur).norm() << "," << traj.getAcc(t_cur).norm() << "\n";

            }
            csv_yellow_pva.close();
            std::cout << "Saved CSV file" << std::endl;
        }
        else{
            traj = trajOpt.genOptimalTrajDTCs3(route, initialVel, initialAcc, finalVel, finalAcc);
            clock_t stop = clock();
            cout << "Fast Spatial-Tenporal Traj Opt(AM_old): " << (stop - start) * 1000.0 / CLOCKS_PER_SEC << " ms" << endl;
            cout << "Number of Traj Pieces: " << traj.getPieceNum() << endl;
            cout << "t_lap: " << traj.getTotalDuration() << endl;
            cout << "cost: " << trajOpt.evaluateObjective(traj) << endl;

            // 保存 位置，速度，加速度曲线，用于MATLAB绘制
            std::ofstream csv_orange_pva(result_dir + "ORANGE_constrained-AM-pva.csv");
            for(double t_cur = 0.0; t_cur <= traj.getTotalDuration(); t_cur += 0.01)
            {
                Eigen::Vector3d pos = traj.getPos(t_cur);
                double x = pos(0);
                double y = pos(1);
                double z = pos(2);
                csv_orange_pva << t_cur << "," << x << "," << y << "," << z << "," << traj.getVel(t_cur).norm() << "," << traj.getAcc(t_cur).norm() << "\n";

            }
            csv_orange_pva.close();
            std::cout << "Saved CSV file" << std::endl;
        }
    } while (!trajSafeCheck(traj, route));  // 避障安全检查

    return traj;
}

// 采样检查轨迹安全性，在发生碰撞的段中间加入新航点 
bool TrajGen::trajSafeCheck(const Trajectory &traj, std::vector<Eigen::Vector3d> &route) const
{
    bool safe = true;

    vector<Vector3d> discretePoints;

    double t;
    int step;
    Vector3d tempTranslation;
    bool tempSafe;
    for (int i = 0; i < traj.getPieceNum(); i++)
    {
        step = traj[i].getDuration() / config.temporalResolution;
        t = 0.0;
        discretePoints.push_back(route[i]);
        for (int j = 0; j < step - 1; j++)
        {
            t += config.temporalResolution;
            tempTranslation = traj[i].getPos(t);
            tempSafe = glbMapPtr->safeQuery(tempTranslation, config.bodySafeRadius);
            safe &= tempSafe;
            if (!tempSafe)
            {
                discretePoints.push_back((route[i] + route[i + 1]) / 2.0);  // 两点中间插入新点
                break;
            }
        }
    }
    discretePoints.push_back(route[traj.getPieceNum()]);
    route = discretePoints;

    return safe;
}

// 经典的道格拉斯-普克（Douglas-Peucker）算法（或类似原理的递归路径简化方法），用于减少路径中的点数，同时保持路径形状的近似度在一个可接受的范围内。
std::vector<Eigen::Vector3d> TrajGen::routeSimplify(const vector<Vector3d> &route, double resolution) const
{
    vector<Vector3d> subRoute;
    if (route.size() == 1 || route.size() == 2)
    {
        subRoute = route;
    }
    else if (route.size() >= 3)
    {
        vector<Vector3d>::const_iterator maxIt;
        double maxDist = -INFINITY, tempDist;
        Vector3d vec((route.back() - route.front()).normalized());

        for (auto it = route.begin() + 1; it != (route.end() - 1); it++)
        {
            tempDist = (*it - route.front() - vec.dot(*it - route.front()) * vec).norm();
            if (maxDist < tempDist)
            {
                maxDist = tempDist;
                maxIt = it;
            }
        }

        if (maxDist > resolution)
        {
            subRoute.insert(subRoute.end(), route.begin(), maxIt + 1);
            subRoute = routeSimplify(subRoute, resolution);
            vector<Vector3d> tempRoute(maxIt, route.end());
            tempRoute = routeSimplify(tempRoute, resolution);
            subRoute.insert(subRoute.end(), tempRoute.begin() + 1, tempRoute.end());
        }
        else
        {
            subRoute.push_back(route.front());
            subRoute.push_back(route.back());
        }
    }

    return subRoute;
}
