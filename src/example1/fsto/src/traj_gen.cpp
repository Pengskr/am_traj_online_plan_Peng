#include "fsto/traj_gen.h"
#include <ros/package.h>
#include <fstream>
#include <iomanip>

#include <ctime>

using namespace std;
using namespace Eigen;

TrajGen::TrajGen(const Config &conf, std::shared_ptr<const LocalPerceptionMap> mapPtr)
    : config(conf), MapPtr(mapPtr),
      trajOpt(config.weightT, config.weightAcc, config.weightJerk,
              config.maxVelRate, config.maxAccRate, config.iterations, config.epsilon)
{
}

Trajectory TrajGen::generate(vector<Vector3d> &route,
                             Vector3d initialVel,
                             Vector3d initialAcc,
                             Vector3d finalVel,
                             Vector3d finalAcc,
                             int id_alg,
                             Visualization visualization) const
{
    Trajectory traj;

    route = routeSimplify(route, config.spatialResolution);     // 道格拉斯-普克（Douglas-Peucker）算法

    if (route.size() < 2 || (route[0] - route[1]).squaredNorm() < FLT_EPSILON)
    {
        return traj;
    }

    // std::string result_dir = "/home/peng/Desktop/am_traj_Peng/src/example1/results/";

    int tries = 0;
    do
    {
        clock_t start = clock();

        tries++;
        if (tries > config.tryOut)  // 超过最大规划次数
        {            
            ROS_WARN("AM_traj planning Fails: tries > tryOut");
            // visualization.visualize(traj, route, ros::Time::now(), 1);   
            // traj.clear();
            break;
        }

        if(id_alg == 2){
            traj = trajOpt.genOptimalTrajDTCWholeScales3(route, initialVel, initialAcc, finalVel, finalAcc);    // 带时域缩放的有约束交替优化
            clock_t stop = clock();
            cout << "Fast Spatial-Tenporal Traj Opt(AM with scale): " << (stop - start) * 1000.0 / CLOCKS_PER_SEC << " ms" << endl;
            cout << "Number of Traj Pieces: " << traj.getPieceNum() << endl;
            cout << "t_lap: " << traj.getTotalDuration() << endl;
            cout << "cost: " << trajOpt.evaluateObjective(traj) << endl;

            // // 保存 位置，速度，加速度曲线，用于MATLAB绘制
            // std::ofstream csv_green_pva(result_dir + "GREEN_constrained-AM-pva.csv");
            // for(double t_cur = 0.0; t_cur <= traj.getTotalDuration(); t_cur += 0.01)
            // {
            //     Eigen::Vector3d pos = traj.getPos(t_cur);
            //     double x = pos(0);
            //     double y = pos(1);
            //     double z = pos(2);
            //     csv_green_pva << t_cur << "," << x << "," << y << "," << z << "," << traj.getVel(t_cur).norm() << "," << traj.getAcc(t_cur).norm() << "\n";

            // }
            // csv_green_pva.close();
            // std::cout << "Saved CSV file" << std::endl;
        }
        else if(id_alg == 3){
            traj = trajOpt.genOptimalTrajDTCs3(route, initialVel, initialAcc, finalVel, finalAcc);    //精细化有约束交替优化
            clock_t stop = clock();
            cout << "Fast Spatial-Tenporal Traj Opt(AM): " << (stop - start) * 1000.0 / CLOCKS_PER_SEC << " ms" << endl;
            cout << "Number of Traj Pieces: " << traj.getPieceNum() << endl;
            cout << "t_lap: " << traj.getTotalDuration() << endl;
            cout << "cost: " << trajOpt.evaluateObjective(traj) << endl;

            // // 保存 位置，速度，加速度曲线，用于MATLAB绘制
            // std::ofstream csv_yellow_pva(result_dir + "YELLOW_constrained-AM-pva.csv");
            // for(double t_cur = 0.0; t_cur <= traj.getTotalDuration(); t_cur += 0.01)
            // {
            //     Eigen::Vector3d pos = traj.getPos(t_cur);
            //     double x = pos(0);
            //     double y = pos(1);
            //     double z = pos(2);
            //     csv_yellow_pva << t_cur << "," << x << "," << y << "," << z << "," << traj.getVel(t_cur).norm() << "," << traj.getAcc(t_cur).norm() << "\n";

            // }
            // csv_yellow_pva.close();
            // std::cout << "Saved CSV file" << std::endl;
        }
    } while (!trajSafeCheck(traj, route));  // 避障安全检查

    return traj;
}

// 采样检查轨迹安全性，在发生碰撞的段中间加入新航点 
// 采样检查轨迹安全性。
// 若发现碰撞，每次仅在第一处碰撞段插入一个新航点，避免航点数量指数级增长。
// 插入点优先选择线段法向偏移点，使 route 尽量远离障碍物边缘。
bool TrajGen::trajSafeCheck(const Trajectory &traj,
                            std::vector<Eigen::Vector3d> &route) const
{
    if (traj.getPieceNum() <= 0 || route.size() < 2)
    {
        ROS_WARN("[TrajSafeCheck] Empty trajectory or invalid route.");
        return false;
    }

    if (static_cast<size_t>(traj.getPieceNum()) + 1 > route.size())
    {
        ROS_WARN("[TrajSafeCheck] traj piece number and route size mismatch. piece_num = %d, route_size = %lu",
                 traj.getPieceNum(),
                 route.size());
        return false;
    }

    // 防止线段被无限二分。
    // 后续可以改成 yaml 参数。
    const double min_insert_segment_length = 0.20;  // m
    const double min_point_separation = 0.05;       // m

    for (int i = 0; i < traj.getPieceNum(); ++i)
    {
        const double duration = traj[i].getDuration();

        const int step = std::max(
            1,
            static_cast<int>(std::ceil(duration / config.temporalResolution)));

        double t = 0.0;

        for (int j = 0; j < step - 1; ++j)
        {
            t += config.temporalResolution;

            const Eigen::Vector3d pos = traj[i].getPos(t);

            // 当前采样点安全，则继续检查该 piece 后续采样点。
            if (MapPtr->safeQuery(pos, config.bodySafeRadius))
            {
                continue;
            }

            // ============================================================
            // 发现第一处碰撞：只处理这一处，然后立即 return false。
            // 这样一次 trajSafeCheck() 最多只增加一个航点。
            // ============================================================
            const Eigen::Vector3d p0 = route[i];
            const Eigen::Vector3d p1 = route[i + 1];

            const Eigen::Vector3d seg = p1 - p0;
            const double seg_len = seg.norm();

            if (seg_len < min_insert_segment_length)
            {
                ROS_WARN("[TrajSafeCheck] Collision segment too short for further insertion. len = %.3f",
                         seg_len);
                return false;
            }

            const Eigen::Vector3d mid = 0.5 * (p0 + p1);

            Eigen::Vector3d insertPt = mid;

            // ============================================================
            // 优先尝试水平面左右法向偏移点。
            // 这样比简单插中点更可能把 route 推离障碍物边缘。
            // ============================================================
            Eigen::Vector3d dir = seg;
            dir(2) = 0.0;

            if (dir.norm() > 1e-3)
            {
                dir.normalize();

                const Eigen::Vector3d left(-dir(1), dir(0), 0.0);
                const Eigen::Vector3d right(dir(1), -dir(0), 0.0);

                // 偏移距离：至少取 r3SafeRadius，也给一个 0.5m 的下限。
                const double offset = std::max(config.r3SafeRadius, 0.50);

                Eigen::Vector3d candLeft = mid + offset * left;
                Eigen::Vector3d candRight = mid + offset * right;

                // 保持高度不变，先只在水平面绕障。
                candLeft(2) = mid(2);
                candRight(2) = mid(2);

                const bool leftSafe = MapPtr->safeQuery(candLeft, config.r3SafeRadius);
                const bool rightSafe = MapPtr->safeQuery(candRight, config.r3SafeRadius);

                if (leftSafe && rightSafe)
                {
                    // 两侧都安全时，选择更接近终点的一侧。
                    if ((candLeft - route.back()).norm() <
                        (candRight - route.back()).norm())
                    {
                        insertPt = candLeft;
                    }
                    else
                    {
                        insertPt = candRight;
                    }
                }
                else if (leftSafe)
                {
                    insertPt = candLeft;
                }
                else if (rightSafe)
                {
                    insertPt = candRight;
                }
                else
                {
                    // 左右偏移都不安全，退化为中点。
                    insertPt = mid;
                }
            }
            else
            {
                insertPt = mid;
            }

            // 防止插入点和原端点过近。
            if ((insertPt - p0).norm() < min_point_separation ||
                (insertPt - p1).norm() < min_point_separation)
            {
                ROS_WARN("[TrajSafeCheck] Insert point too close to segment endpoint.");
                return false;
            }

            route.insert(route.begin() + i + 1, insertPt);

            ROS_WARN("[TrajSafeCheck] Collision at piece %d. Insert one waypoint. route_size = %lu",
                     i,
                     route.size());

            return false;
        }
    }

    return true;
}

bool TrajGen::segmentSafe(const Eigen::Vector3d &a,
                          const Eigen::Vector3d &b,
                          double resolution,
                          double safeRadius) const
{
    Eigen::Vector3d diff = b - a;
    double len = diff.norm();

    if (len < 1e-6)
    {
        return MapPtr->safeQuery(a, safeRadius);
    }

    int steps = std::ceil(len / resolution);

    for (int i = 0; i <= steps; ++i)
    {
        double s = static_cast<double>(i) / steps;
        Eigen::Vector3d p = a + s * diff;

        if (!MapPtr->safeQuery(p, safeRadius))
        {
            return false;
        }
    }

    return true;
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
            if (segmentSafe(route.front(),
                            route.back(),
                            config.spatialResolution,
                            config.r3SafeRadius))
            {
                subRoute.push_back(route.front());
                subRoute.push_back(route.back());
            }
            else
            {
                // 如果直连线段不安全，则不要过度简化，保留原路径
                subRoute = route;
            }
        }
    }

    return subRoute;
}


