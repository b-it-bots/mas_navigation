/*
 * collision_velocity_filter.cpp
 *
 *  Created on: Jul 27, 2014
 *      Author: Frederik Hegger
 */

#include <mcr_collision_velocity_filter/collision_velocity_filter.h>

CollisionVelocityFilter::CollisionVelocityFilter() :
        soft_padding_front_rear_(0.15), hard_padding_front_rear_(0.02), soft_padding_left_right_(0.15), hard_padding_left_right_(0.06), linear_velocity_in_soft_padding_(0.02), angular_velocity_in_soft_padding_(
                0.1)
{
    zero_twist_velocity_.linear.x = zero_twist_velocity_.linear.y = zero_twist_velocity_.linear.z = 0.0;
    zero_twist_velocity_.angular.x = zero_twist_velocity_.angular.y = zero_twist_velocity_.angular.z = 0.0;
}

CollisionVelocityFilter::~CollisionVelocityFilter()
{
}

void CollisionVelocityFilter::setSoftPaddingParameter(const double &front_rear, const double &left_right)
{
    soft_padding_front_rear_ = front_rear;
    soft_padding_left_right_ = left_right;
}

void CollisionVelocityFilter::setHardPaddingParameter(const double &front_rear, const double &left_right)
{
    hard_padding_front_rear_ = front_rear;
    hard_padding_left_right_ = left_right;
}

void CollisionVelocityFilter::setVelocitiesInSoftPadding(const double &linear_velocity, const double &angular_velocity)
{
    linear_velocity_in_soft_padding_ = linear_velocity;
    angular_velocity_in_soft_padding_ = angular_velocity;
}

geometry_msgs::Twist CollisionVelocityFilter::calculateSafeBaseVelocities(const geometry_msgs::Twist &desired_twist, const pcl::PointCloud<pcl::PointXYZ> &scans_as_pointcloud)
{
    geometry_msgs::Twist safe_twist;
    int roi_start_index = 0, roi_end_index = 0;

    // result is stored in member variables hard_padding_footprint and soft_padding_footprint
    calculateSoftAndHardFootprints(desired_twist);

    pcl::PointCloud < pcl::PointXYZ > soft_footprint_cloud;
    pcl::PointCloud < pcl::PointXYZ > hard_footprint_cloud;

    // check if the footprints have the same point size
    if (soft_padding_footprint_.polygon.points.size() != hard_padding_footprint_.polygon.points.size())
    {
        ROS_ERROR_STREAM("soft- and hard padding footprint do not have the same length: " << soft_padding_footprint_.polygon.points.size() << " != " << hard_padding_footprint_.polygon.points.size());
        return zero_twist_velocity_;
    }

    // copy the polygon data into a pointcloud structure
    for (unsigned i = 0; i < soft_padding_footprint_.polygon.points.size(); ++i)
    {
        soft_footprint_cloud.points.push_back(pcl::PointXYZ(soft_padding_footprint_.polygon.points[i].x, soft_padding_footprint_.polygon.points[i].y, soft_padding_footprint_.polygon.points[i].z));
        hard_footprint_cloud.points.push_back(pcl::PointXYZ(hard_padding_footprint_.polygon.points[i].x, hard_padding_footprint_.polygon.points[i].y, hard_padding_footprint_.polygon.points[i].z));
    }

    if ((soft_footprint_cloud.points.size() <= 0) || (hard_footprint_cloud.points.size() <= 0))
    {
        ROS_ERROR_STREAM("either the soft- or hard footprint has no data points");
        return zero_twist_velocity_;
    }

    // the function isXYPointIn2DXYPolygon later on needs a closed polygon, thus the first point of the polygon is appended as last point to "close" the polygon
    soft_footprint_cloud.points.push_back(soft_footprint_cloud.points[0]);
    hard_footprint_cloud.points.push_back(hard_footprint_cloud.points[0]);

    bool in_soft_padding = false;

    for (size_t i = 0; i < scans_as_pointcloud.points.size(); ++i)
    {
        if (pcl::isXYPointIn2DXYPolygon(scans_as_pointcloud.points[i], hard_footprint_cloud))
            return zero_twist_velocity_;

        else if (pcl::isXYPointIn2DXYPolygon(scans_as_pointcloud.points[i], soft_footprint_cloud) && !in_soft_padding)
            in_soft_padding = true;
    }

    // initialize with the actually commanded velocity
    safe_twist.linear = desired_twist.linear;
    safe_twist.angular = desired_twist.angular;

    // reduce the velocity if the robot is close to an obstacle and if the commanded velocity is greater than the velocity allowed in the soft padding region
    if (in_soft_padding)
    {
        if (desired_twist.linear.x != 0 && fabs(desired_twist.linear.x) > linear_velocity_in_soft_padding_)
            safe_twist.linear.x = linear_velocity_in_soft_padding_ * (desired_twist.linear.x / fabs(desired_twist.linear.x));
        if (desired_twist.linear.y != 0 && fabs(desired_twist.linear.y) > linear_velocity_in_soft_padding_)
            safe_twist.linear.y = linear_velocity_in_soft_padding_ * (desired_twist.linear.y / fabs(desired_twist.linear.y));
        if (desired_twist.angular.z != 0 && fabs(desired_twist.angular.z) > angular_velocity_in_soft_padding_)
            safe_twist.angular.z = angular_velocity_in_soft_padding_ * (desired_twist.angular.z / fabs(desired_twist.angular.z));
    }

    return safe_twist;
}

void CollisionVelocityFilter::updateRealFootprint(const geometry_msgs::PolygonStamped &footprint)
{
    real_footprint_ = footprint;
}

void CollisionVelocityFilter::calculateSoftAndHardFootprints(const geometry_msgs::Twist &desired_twist)
{
    // clear previously calculated soft-/hard footprints
    soft_padding_footprint_ = hard_padding_footprint_ = real_footprint_;

    // calculate soft and hard padding footprints
    for (size_t i = 0; i < soft_padding_footprint_.polygon.points.size(); ++i)
    {
        // if driving to the backwards, extend the polygon in this direction
        if ((desired_twist.linear.x < 0) && (soft_padding_footprint_.polygon.points[i].x < 0))
        {
            soft_padding_footprint_.polygon.points[i].x -= soft_padding_front_rear_;
            hard_padding_footprint_.polygon.points[i].x -= hard_padding_front_rear_;
        }
        // if driving to the forwards, extend the polygon in this direction
        else if ((desired_twist.linear.x > 0) && (soft_padding_footprint_.polygon.points[i].x > 0))
        {
            soft_padding_footprint_.polygon.points[i].x += soft_padding_front_rear_;
            hard_padding_footprint_.polygon.points[i].x += hard_padding_front_rear_;
        }

        // if turning extend both sides or if shifting to the left/right extend to the particular side
        if (((fabs(desired_twist.angular.z) > 0) || (desired_twist.linear.y < 0)) && (soft_padding_footprint_.polygon.points[i].y < 0))
        {
            soft_padding_footprint_.polygon.points[i].y -= soft_padding_left_right_;
            hard_padding_footprint_.polygon.points[i].y -= hard_padding_left_right_;
        }
        if (((fabs(desired_twist.angular.z) > 0) || (desired_twist.linear.y > 0)) && (soft_padding_footprint_.polygon.points[i].y > 0))
        {
            soft_padding_footprint_.polygon.points[i].y += soft_padding_left_right_;
            hard_padding_footprint_.polygon.points[i].y += hard_padding_left_right_;
        }
    }
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getRealFootprint()
{
    return real_footprint_;
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getSoftPaddingFootprint()
{
    return soft_padding_footprint_;
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getHardPaddingFootprint()
{
    return hard_padding_footprint_;
}
