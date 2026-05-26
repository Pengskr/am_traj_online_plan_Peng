#ifndef GLBMAP_H
#define GLBMAP_H

#include "fsto/config.h"

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>

class BinaryGridField
{
public:
    BinaryGridField(Eigen::Vector3i xyz, Eigen::Vector3d offset, double scale);
    ~BinaryGridField();

    BinaryGridField(const BinaryGridField &) = delete;
    BinaryGridField &operator=(const BinaryGridField &) = delete;

    void setOccupied(const Eigen::Vector3d &pos);

    void setOccupiedAndInflate(const std::vector<Eigen::Vector3d> &obsPts,
                               double inflateRadius);

    bool queryOccupied(const Eigen::Vector3d &pos) const;

    bool queryOccupiedWithRadius(const Eigen::Vector3d &pos,
                             double radius) const;

    void getLocalPointCloud(sensor_msgs::PointCloud2 &msg,
                            const Eigen::Vector3d &pos,
                            double radius) const;

    void getOccupiedPointsInRadius(std::vector<Eigen::Vector3d> &pts,
                                   const Eigen::Vector3d &center,
                                   double radius) const;

private:
    Eigen::Vector3i sizeXYZ;
    Eigen::Vector3d originVec;
    double linearScale;

    bool *occupancyPtr;

    size_t stepX, stepY, stepZ;
};


class PriorGlobalMap
{
public:
    PriorGlobalMap(const Config &conf);
    ~PriorGlobalMap();
    void initialize(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void getOccupiedPointsInRadius(std::vector<Eigen::Vector3d> &pts,
                                   const Eigen::Vector3d &center,
                                   double radius) const;

private:
    Config config;
    BinaryGridField *globalGridPtr;
};

class LocalPerceptionMap
{
public:
    LocalPerceptionMap(const Config &conf);
    ~LocalPerceptionMap();
    void buildLocalMapFromGlobal(const PriorGlobalMap &globalMap,
                                 const Eigen::Vector3d &center,
                                 double radius);
    bool safeQuery(const Eigen::Vector3d &p, double safeRadius) const;
    void getLocalInflatedMap(sensor_msgs::PointCloud2 &msg,
                                 const Eigen::Vector3d &pos,
                                 double radius) const;

private:
    Config config;
    BinaryGridField *localGridPtr;
    Eigen::Vector3d mapCenter;
    double visibleRadius;
};

#endif