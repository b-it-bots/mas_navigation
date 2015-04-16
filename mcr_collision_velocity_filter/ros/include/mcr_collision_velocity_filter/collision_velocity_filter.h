/*
 * collision_velocity_checker.h
 *
 *  Created on: Jul 27, 2014
 *      Author: Frederik Hegger
 */

#ifndef COLLISION_VELOCITY_FILTER_H_
#define COLLISION_VELOCITY_FILTER_H_

#include <limits>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//TODO: remove
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>

class CollisionVelocityFilter
{
public:
    CollisionVelocityFilter();
    virtual ~CollisionVelocityFilter();

    geometry_msgs::Twist calculateSafeBaseVelocities(const geometry_msgs::Twist &desired_twist, const pcl::PointCloud<pcl::PointXYZ> &scans_as_pointcloud);
    void updateRealFootprint(const geometry_msgs::PolygonStamped &footprint);

    void setSoftPaddingParameter(const double &front_rear, const double &left_right);
    void setHardPaddingParameter(const double &front_rear, const double &left_right);
    void setVelocitiesInSoftPadding(const double &linear_velocity, const double &angular_velocity);

    geometry_msgs::PolygonStamped getRealFootprint();
    geometry_msgs::PolygonStamped getSoftPaddingFootprint();
    geometry_msgs::PolygonStamped getHardPaddingFootprint();

private:
    void calculateSoftAndHardFootprints(const geometry_msgs::Twist &desired_twist);

    geometry_msgs::PolygonStamped real_footprint_;
    geometry_msgs::PolygonStamped soft_padding_footprint_;
    geometry_msgs::PolygonStamped hard_padding_footprint_;

    geometry_msgs::Twist zero_twist_velocity_;

    // Parameter
    double soft_padding_front_rear_;
    double hard_padding_front_rear_;
    double soft_padding_left_right_;
    double hard_padding_left_right_;

    double linear_velocity_in_soft_padding_;
    double angular_velocity_in_soft_padding_;

};

#endif /* COLLISION_VELOCITY_FILTER_H_ */
