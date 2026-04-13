/*********************************************************************
 * Common types for standalone TEB local planner (ROS-free).
 *********************************************************************/

#ifndef TEB_TYPES_H_
#define TEB_TYPES_H_

#include <Eigen/Core>

namespace teb_local_planner
{

/**
 * @brief Simple velocity command struct replacing geometry_msgs::Twist
 */
struct Twist
{
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;

    Twist() = default;
    Twist(double lx, double ly, double az) : linear_x(lx), linear_y(ly), angular_z(az) {}

    struct Linear {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    } linear;

    struct Angular {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    } angular;
};

} // namespace teb_local_planner

#endif // TEB_TYPES_H_
