#ifndef _ORNEFLAT_H_
#define _ORNEFLAT_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/obstacle_layer.h>
#include <queue>

using costmap_2d::ObservationBuffer;

namespace orne_flat_layer_namespace
{
class OrneFlatLayer : public costmap_2d::ObstacleLayer
{
public:
    OrneFlatLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void laserScanCallback(const sensor_msgs::LaserScanPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer);
    virtual void laserScanVaildInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message, const boost::shared_ptr<ObservationBuffer>& buffer);
private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
};
}
#endif
