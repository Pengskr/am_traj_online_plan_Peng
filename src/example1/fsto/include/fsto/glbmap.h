#ifndef GLBMAP_H
#define GLBMAP_H

#include "fsto/config.h"

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>

// 替换掉原有的 EuclidDistField
class BinaryGridField
{
public:
    BinaryGridField(Eigen::Vector3i xyz, Eigen::Vector3d offset, double scale);
    ~BinaryGridField();
    BinaryGridField(const BinaryGridField &) = delete;

    void setOccupied(const Eigen::Vector3d &pos);
    void inflateObstacles(double inflateRadius); // 障碍物膨胀
    bool queryOccupied(const Eigen::Vector3d &pos) const; // 查询是否被占据
    void getLocalPointCloud(sensor_msgs::PointCloud2 &msg, const Eigen::Vector3d &pos, double radius) const;

private:
    Eigen::Vector3i sizeXYZ;
    Eigen::Vector3d originVec;
    double linearScale;

    bool *occupancyPtr; // 二值占据数组

    size_t stepX, stepY, stepZ;
};

class GlobalMap
{
public:
    GlobalMap(const Config &conf);
    ~GlobalMap();
    void initialize(const sensor_msgs::PointCloud2::ConstPtr &msg);
    bool safeQuery(const Eigen::Vector3d &p, double safeRadius) const;
    void publishLocalInflatedMap(sensor_msgs::PointCloud2 &msg, const Eigen::Vector3d &pos, double radius) const;

private:
    Config config;
    BinaryGridField *gridPtr; // 将 edfPtr 替换为 gridPtr
};

#endif