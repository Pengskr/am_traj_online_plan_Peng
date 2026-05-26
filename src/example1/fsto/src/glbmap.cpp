#include "fsto/glbmap.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <limits>
#include <unordered_set>

using namespace std;
using namespace Eigen;

BinaryGridField::BinaryGridField(Vector3i xyz, Vector3d offset, double scale)
    : sizeXYZ(xyz), originVec(offset), linearScale(scale), occupancyPtr(nullptr),
      stepX(1), stepY(sizeXYZ(0)), stepZ(size_t(sizeXYZ(1)) * sizeXYZ(0))
{
    size_t total = size_t(sizeXYZ(0)) * sizeXYZ(1) * sizeXYZ(2);
    occupancyPtr = new bool[total];
    std::fill_n(occupancyPtr, total, false); // 默认全为 free (false)
}

BinaryGridField::~BinaryGridField()
{
    if (occupancyPtr != nullptr)
    {
        delete[] occupancyPtr;
    }
}

void BinaryGridField::setOccupied(const Eigen::Vector3d &pos)
{
    int tempXi = (pos(0) - originVec(0)) / linearScale;
    int tempYi = (pos(1) - originVec(1)) / linearScale;
    int tempZi = (pos(2) - originVec(2)) / linearScale;
    if (!(tempXi < 0 || tempYi < 0 || tempZi < 0 || tempXi >= sizeXYZ(0) || tempYi >= sizeXYZ(1) || tempZi >= sizeXYZ(2)))
    {
        occupancyPtr[tempXi + tempYi * stepY + tempZi * stepZ] = true;
    }
}

void BinaryGridField::setOccupiedAndInflate(
    const std::vector<Eigen::Vector3d> &obsPts,
    double inflateRadius)
{
    if (occupancyPtr == nullptr)
    {
        return;
    }

    const int radiusInVoxels = std::ceil(inflateRadius / linearScale);
    const double sqrInflateRadius = inflateRadius * inflateRadius;

    std::vector<Eigen::Vector3i> seedVoxels;
    seedVoxels.reserve(obsPts.size());

    std::unordered_set<size_t> seedAddresses;
    seedAddresses.reserve(obsPts.size() * 2 + 1);

    // 1. 将感知范围内的障碍物点转为局部地图中的 occupied seed voxel
    for (const auto &pos : obsPts)
    {
        int ix = std::floor((pos(0) - originVec(0)) / linearScale);
        int iy = std::floor((pos(1) - originVec(1)) / linearScale);
        int iz = std::floor((pos(2) - originVec(2)) / linearScale);

        if (ix < 0 || iy < 0 || iz < 0 ||
            ix >= sizeXYZ(0) || iy >= sizeXYZ(1) || iz >= sizeXYZ(2))
        {
            continue;
        }

        const size_t adr = ix + iy * stepY + iz * stepZ;

        // 同一个体素可能包含多个点云点，只保留一个 seed
        if (seedAddresses.insert(adr).second)
        {
            occupancyPtr[adr] = true;
            seedVoxels.emplace_back(ix, iy, iz);
        }
    }

    // 2. 只围绕 seed voxel 做球形膨胀，不再扫描整个局部地图
    for (const auto &v : seedVoxels)
    {
        for (int dx = -radiusInVoxels; dx <= radiusInVoxels; ++dx)
        {
            for (int dy = -radiusInVoxels; dy <= radiusInVoxels; ++dy)
            {
                for (int dz = -radiusInVoxels; dz <= radiusInVoxels; ++dz)
                {
                    const double distSqr =
                        static_cast<double>(dx * dx + dy * dy + dz * dz) *
                        linearScale * linearScale;

                    if (distSqr > sqrInflateRadius)
                    {
                        continue;
                    }

                    const int nx = v(0) + dx;
                    const int ny = v(1) + dy;
                    const int nz = v(2) + dz;

                    if (nx < 0 || ny < 0 || nz < 0 ||
                        nx >= sizeXYZ(0) || ny >= sizeXYZ(1) || nz >= sizeXYZ(2))
                    {
                        continue;
                    }

                    occupancyPtr[nx + ny * stepY + nz * stepZ] = true;
                }
            }
        }
    }
}

bool BinaryGridField::queryOccupied(const Eigen::Vector3d &pos) const
{
    int tempXi = (pos(0) - originVec(0)) / linearScale;
    int tempYi = (pos(1) - originVec(1)) / linearScale;
    int tempZi = (pos(2) - originVec(2)) / linearScale;
    
    // 如果查询点超出地图范围，默认视为不安全（被占据）
    if (tempXi < 0 || tempYi < 0 || tempZi < 0 || tempXi >= sizeXYZ(0) || tempYi >= sizeXYZ(1) || tempZi >= sizeXYZ(2))
    {
        return true; 
    }
    else
    {
        return occupancyPtr[tempXi + tempYi * stepY + tempZi * stepZ];
    }
}

bool BinaryGridField::queryOccupiedWithRadius(const Eigen::Vector3d &pos,
                                              double radius) const
{
    if (radius <= 1e-6)
    {
        return queryOccupied(pos);
    }

    int cx = std::floor((pos(0) - originVec(0)) / linearScale);
    int cy = std::floor((pos(1) - originVec(1)) / linearScale);
    int cz = std::floor((pos(2) - originVec(2)) / linearScale);

    if (cx < 0 || cy < 0 || cz < 0 ||
        cx >= sizeXYZ(0) || cy >= sizeXYZ(1) || cz >= sizeXYZ(2))
    {
        return true;
    }

    const int r_vox = std::ceil(radius / linearScale);
    const double radius_sqr = radius * radius;

    for (int dx = -r_vox; dx <= r_vox; ++dx)
    {
        for (int dy = -r_vox; dy <= r_vox; ++dy)
        {
            for (int dz = -r_vox; dz <= r_vox; ++dz)
            {
                const double dist_sqr =
                    static_cast<double>(dx * dx + dy * dy + dz * dz) *
                    linearScale * linearScale;

                if (dist_sqr > radius_sqr)
                {
                    continue;
                }

                int nx = cx + dx;
                int ny = cy + dy;
                int nz = cz + dz;

                if (nx < 0 || ny < 0 || nz < 0 ||
                    nx >= sizeXYZ(0) || ny >= sizeXYZ(1) || nz >= sizeXYZ(2))
                {
                    return true;
                }

                if (occupancyPtr[nx + ny * stepY + nz * stepZ])
                {
                    return true;
                }
            }
        }
    }

    return false;
}

void BinaryGridField::getLocalPointCloud(sensor_msgs::PointCloud2 &msg, const Eigen::Vector3d &pos, double radius) const
{
    // 1. 将无人机的物理坐标转换为网格索引
    int cx = (pos(0) - originVec(0)) / linearScale;
    int cy = (pos(1) - originVec(1)) / linearScale;
    int cz = (pos(2) - originVec(2)) / linearScale;
    int r_vox = std::ceil(radius / linearScale);

    // 2. 计算包围盒并进行严格的边界截断 (Boundary Clipping)
    int x_min = std::max(0, cx - r_vox);
    int x_max = std::min(sizeXYZ(0) - 1, cx + r_vox);
    int y_min = std::max(0, cy - r_vox);
    int y_max = std::min(sizeXYZ(1) - 1, cy + r_vox);
    int z_min = std::max(0, cz - r_vox);
    int z_max = std::min(sizeXYZ(2) - 1, cz + r_vox);

    double radiusSqr = radius * radius;

    // 3. 第一遍遍历：统计球体内的碰撞点数量（用于分配点云内存）
    size_t num_points = 0;
    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            for (int z = z_min; z <= z_max; z++) {
                if (occupancyPtr[x + y * stepY + z * stepZ]) {
                    // 还原为物理坐标，计算到中心的平方距离
                    double px = x * linearScale + originVec(0) + linearScale / 2.0;
                    double py = y * linearScale + originVec(1) + linearScale / 2.0;
                    double pz = z * linearScale + originVec(2) + linearScale / 2.0;
                    double distSqr = (px - pos(0))*(px - pos(0)) + (py - pos(1))*(py - pos(1)) + (pz - pos(2))*(pz - pos(2));
                    
                    if (distSqr <= radiusSqr) {
                        num_points++;
                    }
                }
            }
        }
    }

    // 4. 初始化 PointCloud2
    msg.height = 1;
    msg.width = num_points;
    msg.is_bigendian = false;
    msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    // 5. 第二遍遍历：填充点云数据
    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            for (int z = z_min; z <= z_max; z++) {
                if (occupancyPtr[x + y * stepY + z * stepZ]) {
                    double px = x * linearScale + originVec(0) + linearScale / 2.0;
                    double py = y * linearScale + originVec(1) + linearScale / 2.0;
                    double pz = z * linearScale + originVec(2) + linearScale / 2.0;
                    double distSqr = (px - pos(0))*(px - pos(0)) + (py - pos(1))*(py - pos(1)) + (pz - pos(2))*(pz - pos(2));
                    
                    if (distSqr <= radiusSqr) {
                        *iter_x = px;
                        *iter_y = py;
                        *iter_z = pz;
                        ++iter_x; ++iter_y; ++iter_z;
                    }
                }
            }
        }
    }
}

void BinaryGridField::getOccupiedPointsInRadius(
    std::vector<Eigen::Vector3d> &pts,
    const Eigen::Vector3d &center,
    double radius) const
{
    pts.clear();

    Eigen::Vector3i min_id;
    Eigen::Vector3i max_id;

    min_id << std::floor((center(0) - radius - originVec(0)) / linearScale),
              std::floor((center(1) - radius - originVec(1)) / linearScale),
              std::floor((center(2) - radius - originVec(2)) / linearScale);

    max_id << std::ceil((center(0) + radius - originVec(0)) / linearScale),
              std::ceil((center(1) + radius - originVec(1)) / linearScale),
              std::ceil((center(2) + radius - originVec(2)) / linearScale);

    min_id = min_id.cwiseMax(Eigen::Vector3i(0, 0, 0));
    max_id = max_id.cwiseMin(sizeXYZ - Eigen::Vector3i(1, 1, 1));

    const double radius_sqr = radius * radius;

    for (int x = min_id(0); x <= max_id(0); ++x)
    {
        for (int y = min_id(1); y <= max_id(1); ++y)
        {
            for (int z = min_id(2); z <= max_id(2); ++z)
            {
                int adr = x + y * stepY + z * stepZ;

                if (!occupancyPtr[adr])
                    continue;

                Eigen::Vector3d p;
                p << originVec(0) + (x + 0.5) * linearScale,
                     originVec(1) + (y + 0.5) * linearScale,
                     originVec(2) + (z + 0.5) * linearScale;

                if ((p - center).squaredNorm() <= radius_sqr)
                {
                    pts.push_back(p);
                }
            }
        }
    }
}

PriorGlobalMap::PriorGlobalMap(const Config &conf)
    : config(conf),
      globalGridPtr(nullptr)
{
}

PriorGlobalMap::~PriorGlobalMap()
{
    if (globalGridPtr != nullptr)
    {
        delete globalGridPtr;
    }
}

void PriorGlobalMap::initialize(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // ====== 修改点：为网格数组增加 Padding，防止边界处的膨胀被截断 ======
    // 预留的额外空间：无人机半径 + 0.5米余量
    double pad = config.bodySafeRadius + 0.5; 

    // 网格的物理长宽高都需要加上两侧的 pad
    Vector3i xyz((config.r3Bound[1] - config.r3Bound[0] + 2.0 * pad) / config.edfResolution,
                 (config.r3Bound[3] - config.r3Bound[2] + 2.0 * pad) / config.edfResolution,
                 (config.r3Bound[5] - config.r3Bound[4] + 2.0 * pad) / config.edfResolution);   

    // 地图的原点偏移向负方向后退 pad
    Vector3d offset(config.r3Bound[0] - pad, 
                    config.r3Bound[2] - pad, 
                    config.r3Bound[4] - pad);  
    // ================================================================ 

    if (globalGridPtr != nullptr) {
        delete globalGridPtr;
    }
    globalGridPtr = new BinaryGridField(xyz, offset, config.edfResolution);

    size_t cur = 0;
    size_t total = msg->data.size() / msg->point_step;
    float *fdata = (float *)(&msg->data[0]);
    Vector3d tempVec;
    
    // 读入原始障碍物点
    for (size_t i = 0; i < total; i++)
    {
        cur = msg->point_step / sizeof(float) * i;

        if (isnan(fdata[cur + 0]) || isinf(fdata[cur + 0]) ||
            isnan(fdata[cur + 1]) || isinf(fdata[cur + 1]) ||
            isnan(fdata[cur + 2]) || isinf(fdata[cur + 2]))
        {
            continue;
        }
        tempVec << config.unitScaleInSI * fdata[cur + 0],
                   config.unitScaleInSI * fdata[cur + 1],
                   config.unitScaleInSI * fdata[cur + 2];
        globalGridPtr->setOccupied(tempVec);
    }
}

void PriorGlobalMap::getOccupiedPointsInRadius(std::vector<Eigen::Vector3d> &pts,
                                        const Eigen::Vector3d &center,
                                        double radius) const
{
    if (globalGridPtr == nullptr)
    {
        pts.clear();
        return;
    }

    globalGridPtr->getOccupiedPointsInRadius(pts, center, radius);
}


LocalPerceptionMap::LocalPerceptionMap(const Config &conf)
    : config(conf),
      localGridPtr(nullptr),
      mapCenter(Eigen::Vector3d::Zero()),
      visibleRadius(0.0)
{
}

LocalPerceptionMap::~LocalPerceptionMap()
{
    if (localGridPtr != nullptr)
    {
        delete localGridPtr;
    }
}

void LocalPerceptionMap::buildLocalMapFromGlobal(
    const PriorGlobalMap &globalMap,
    const Eigen::Vector3d &center,
    double radius)
{
    const double pad = config.bodySafeRadius + config.edfResolution;
    const double map_radius = radius + pad;

    Eigen::Vector3i xyz;
    xyz << std::ceil(2.0 * map_radius / config.edfResolution),
           std::ceil(2.0 * map_radius / config.edfResolution),
           std::ceil((config.r3Bound[5] - config.r3Bound[4] + 2.0 * pad) /
                     config.edfResolution);

    Eigen::Vector3d offset;
    offset << center(0) - map_radius,
              center(1) - map_radius,
              config.r3Bound[4] - pad;

    if (localGridPtr != nullptr)
    {
        delete localGridPtr;
        localGridPtr = nullptr;
    }

    localGridPtr = new BinaryGridField(xyz, offset, config.edfResolution);

    std::vector<Eigen::Vector3d> local_obs;
    globalMap.getOccupiedPointsInRadius(local_obs, center, radius);

    localGridPtr->setOccupiedAndInflate(local_obs, config.bodySafeRadius);

    mapCenter = center;
    visibleRadius = radius;
}

bool LocalPerceptionMap::safeQuery(const Eigen::Vector3d &p, double safeRadius) const
{
    if (localGridPtr == nullptr)
    {
        return true;
    }

    if ((p - mapCenter).norm() > visibleRadius)
    {
        return true;
    }

    // localGridPtr 已经按 bodySafeRadius 膨胀。
    // 如果调用方要求更大的 safeRadius，则额外检查邻域。
    double extraRadius = std::max(0.0, safeRadius - config.bodySafeRadius);

    return !localGridPtr->queryOccupiedWithRadius(p, extraRadius);
}


void LocalPerceptionMap::getLocalInflatedMap(sensor_msgs::PointCloud2 &msg, const Eigen::Vector3d &pos, double radius) const
{
    if (localGridPtr != nullptr) {
        localGridPtr->getLocalPointCloud(msg, pos, radius);
    }
}



