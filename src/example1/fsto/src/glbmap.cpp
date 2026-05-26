#include "fsto/glbmap.h"
#include <sensor_msgs/point_cloud2_iterator.h>

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

void BinaryGridField::inflateObstacles(double inflateRadius)
{
    int radiusInVoxels = std::ceil(inflateRadius / linearScale);
    double sqrRadius = inflateRadius * inflateRadius;

    // 1. 收集初始原始障碍物体素的索引，防止在膨胀过程中边膨胀边读取，造成无限制蔓延
    std::vector<Eigen::Vector3i> obsVoxels;
    for (int x = 0; x < sizeXYZ(0); x++) {
        for (int y = 0; y < sizeXYZ(1); y++) {
            for (int z = 0; z < sizeXYZ(2); z++) {
                if (occupancyPtr[x + y * stepY + z * stepZ]) {
                    obsVoxels.emplace_back(x, y, z);
                }
            }
        }
    }

    // 2. 对每个障碍物体素进行球形邻域膨胀
    for (const auto& v : obsVoxels) {
        for (int dx = -radiusInVoxels; dx <= radiusInVoxels; dx++) {
            for (int dy = -radiusInVoxels; dy <= radiusInVoxels; dy++) {
                for (int dz = -radiusInVoxels; dz <= radiusInVoxels; dz++) {
                    int nx = v(0) + dx;
                    int ny = v(1) + dy;
                    int nz = v(2) + dz;

                    // 越界检查
                    if (nx < 0 || ny < 0 || nz < 0 || nx >= sizeXYZ(0) || ny >= sizeXYZ(1) || nz >= sizeXYZ(2)) 
                        continue;

                    // 距离检查 (球形膨胀)
                    double distSqr = (dx*dx + dy*dy + dz*dz) * linearScale * linearScale;
                    if (distSqr <= sqrRadius) {
                        occupancyPtr[nx + ny * stepY + nz * stepZ] = true;
                    }
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


GlobalMap::GlobalMap(const Config &conf)
    : config(conf), gridPtr(nullptr)
{
}

GlobalMap::~GlobalMap()
{
    if (gridPtr != nullptr)
    {
        delete gridPtr;
    }
}

// 使用点云数据PointCloud2 来构建二值膨胀地图
void GlobalMap::initialize(const sensor_msgs::PointCloud2::ConstPtr &msg)
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

    if (gridPtr != nullptr) {
        delete gridPtr;
    }
    gridPtr = new BinaryGridField(xyz, offset, config.edfResolution);

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
        gridPtr->setOccupied(tempVec);
    }
    
    // 调用二值地图膨胀，以无人机安全半径膨胀
    gridPtr->inflateObstacles(config.bodySafeRadius);
}

bool GlobalMap::safeQuery(const Vector3d &p, double safeRadius) const
{
    // 因为在 initialize 时已经执行过 inflateObstacles 膨胀了地图
    // 此处直接查询查询点 p 是否在膨胀后的占据空间即可，忽略传入的 safeRadius(保证接口兼容性)
    return !gridPtr->queryOccupied(p);
}

void BinaryGridField::getPointCloud(sensor_msgs::PointCloud2 &msg) const
{
    // 1. 统计当前有多少个被占据（包含膨胀后）的体素
    size_t num_points = 0;
    size_t total_voxels = size_t(sizeXYZ(0)) * sizeXYZ(1) * sizeXYZ(2);
    for (size_t i = 0; i < total_voxels; ++i) {
        if (occupancyPtr[i]) {
            num_points++;
        }
    }

    // 2. 初始化 PointCloud2 消息结构
    msg.height = 1;
    msg.width = num_points;
    msg.is_bigendian = false;
    msg.is_dense = true;

    // 使用 Modifier 设置字段为标准 xyz 格式并分配内存
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);

    // 创建安全迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    // 3. 将被占据体素的网格中心坐标填入点云
    for (int x = 0; x < sizeXYZ(0); x++) {
        for (int y = 0; y < sizeXYZ(1); y++) {
            for (int z = 0; z < sizeXYZ(2); z++) {
                if (occupancyPtr[x + y * stepY + z * stepZ]) {
                    *iter_x = x * linearScale + originVec(0) + linearScale / 2.0;
                    *iter_y = y * linearScale + originVec(1) + linearScale / 2.0;
                    *iter_z = z * linearScale + originVec(2) + linearScale / 2.0;
                    ++iter_x; ++iter_y; ++iter_z;
                }
            }
        }
    }
}

void GlobalMap::publishInflatedMap(sensor_msgs::PointCloud2 &msg) const
{
    if (gridPtr != nullptr) {
        gridPtr->getPointCloud(msg);
    }
}