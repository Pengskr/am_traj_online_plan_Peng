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
    void getPointCloud(sensor_msgs::PointCloud2 &msg) const; // 转换为点云

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
    void publishInflatedMap(sensor_msgs::PointCloud2 &msg) const; // 供上层调用的接口

private:
    Config config;
    BinaryGridField *gridPtr; // 将 edfPtr 替换为 gridPtr
};

#endif