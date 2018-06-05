/*********************************************************************
 * Software License Agreement (GPLv3 License)
 *
 *  Copyright (c) 2018, Hochschule Bonn-Rhein-Sieg.
 *  All rights reserved.
 *
 *********************************************************************/
/**
 * Author: Santosh Thoduka
 * Based on code by Frederik Hegger in mcr_collision_velocity_filter
 */

#ifndef MCR_NAVIGATION_TOOLS_LASER_DISTANCES_NODE_H
#define MCR_NAVIGATION_TOOLS_LASER_DISTANCES_NODE_H

#include <exception>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;

class LaserDistances
{
public:
    LaserDistances();
    ~LaserDistances();

    void update();

private:
    void oneLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void twoSynchronizedLaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_1,
        const sensor_msgs::LaserScan::ConstPtr &scan_2);
    void threeSynchronizedLaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_1,
        const sensor_msgs::LaserScan::ConstPtr &scan_2, const sensor_msgs::LaserScan::ConstPtr &scan_3);

    void accumulateLaserScansToPointCloud(const std::vector<sensor_msgs::LaserScan> &scans);

    std::vector<std::string> readScanTopicsFromParameterServer();

    bool getFootprintFromParameterServer(const std::string &parameter_name);

    sensor_msgs::PointCloud2 getCloudFromLaserScan(const sensor_msgs::LaserScan &scan);

    float getMaxX(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices);
    float getMinX(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices);
    float getMaxY(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices);
    float getMinY(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices);

    ros::Subscriber sub_single_scan_;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan_1;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan_2;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan_3;
    boost::shared_ptr<Synchronizer<ApproximateTime<sensor_msgs::LaserScan,
        sensor_msgs::LaserScan> > > two_synced_laser_scans_;
    boost::shared_ptr<Synchronizer<ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan,
        sensor_msgs::LaserScan> > > three_synced_laser_scans_;

    // Publisher
    ros::Publisher pub_distances_;

    laser_geometry::LaserProjection laser_projector_;
    tf::TransformListener transform_listener_;

    pcl::PointCloud<pcl::PointXYZ> laser_scans_as_pcl_cloud;
    bool laser_scans_as_pcl_cloud_received;

    sensor_msgs::PointCloud2 laser_scans_as_cloud_;

    std::string target_frame_;

    geometry_msgs::Polygon footprint_;
};

#endif  // MCR_NAVIGATION_TOOLS_LASER_DISTANCES_NODE_H
