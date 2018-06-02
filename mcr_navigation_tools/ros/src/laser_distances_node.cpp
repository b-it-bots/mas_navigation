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

#include <mcr_navigation_tools/laser_distances_node.h>
#include <vector>
#include <string>

LaserDistances::LaserDistances() :
        laser_scans_as_pcl_cloud_received(false)
{
    std::string footprint_parameter_name = "footprint";
    std::vector<std::string> scan_topics;

    ros::NodeHandle nh("~");

    nh.param<std::string>("target_frame", target_frame_, "/base_link");

    // try to read parameter from parameter server
    ROS_INFO("Reading parameter from parameter server ...");
    if (!getFootprintFromParameterServer(footprint_parameter_name))
    {
        ROS_ERROR_STREAM("Could not read parameter <" << footprint_parameter_name
            << "> from the parameter server. Try to receive it from the specified topic ...");
        exit(0);
    }
    ROS_INFO("Got footprint!");

    scan_topics = readScanTopicsFromParameterServer();

    if (scan_topics.size() == 0)
    {
        ROS_ERROR("No scan topics specified. Exiting ...");
        exit(0);
    }
    else if (scan_topics.size() == 1)
    {
        ROS_INFO_STREAM("Subscribing to one scan topic: " << scan_topics[0]);
        sub_single_scan_ = nh.subscribe < sensor_msgs::LaserScan > (scan_topics[0], 10,
            &LaserDistances::oneLaserScanCallback, this);
    }
    else
    {
        ROS_INFO("Subscribing to multiple scan topics: ");
        for (size_t i = 0; i < scan_topics.size(); ++i)
            ROS_INFO_STREAM("    " << scan_topics[i]);

        if (scan_topics.size() == 2)
        {
            sub_scan_1.subscribe(nh, scan_topics[0], 10);
            sub_scan_2.subscribe(nh, scan_topics[1], 10);

            two_synced_laser_scans_ = boost::make_shared<Synchronizer<ApproximateTime<sensor_msgs::LaserScan,
                sensor_msgs::LaserScan> > >(3);
            two_synced_laser_scans_->connectInput(sub_scan_1, sub_scan_2);
            two_synced_laser_scans_->registerCallback(boost::bind(
                &LaserDistances::twoSynchronizedLaserscanCallback, this, _1, _2));
        }
        else if (scan_topics.size() == 3)
        {
            sub_scan_1.subscribe(nh, scan_topics[0], 10);
            sub_scan_2.subscribe(nh, scan_topics[1], 10);
            sub_scan_3.subscribe(nh, scan_topics[0], 10);

            three_synced_laser_scans_ = boost::make_shared<Synchronizer<ApproximateTime<sensor_msgs::LaserScan,
                sensor_msgs::LaserScan, sensor_msgs::LaserScan> > >(3);
            three_synced_laser_scans_->connectInput(sub_scan_1, sub_scan_2, sub_scan_3);
            three_synced_laser_scans_->registerCallback(boost::bind(
                &LaserDistances::threeSynchronizedLaserscanCallback, this, _1, _2, _3));
        }
        else
        {
            ROS_ERROR_STREAM("The number of subscribed scan topics is not supported");
            exit(0);
        }
    }

    // Publisher
    pub_distances_ = nh.advertise <std_msgs::Float32MultiArray> ("distances", 1);
}

LaserDistances::~LaserDistances()
{
    pub_distances_.shutdown();
}

std::vector<std::string> LaserDistances::readScanTopicsFromParameterServer()
{
    std::vector<std::string> scan_topics;
    ros::NodeHandle nh("~");

    if (!nh.getParam("scan_topics", scan_topics))
        ROS_ERROR_STREAM("No scan topics specified on the parameter server");

    return scan_topics;
}

bool LaserDistances::getFootprintFromParameterServer(const std::string &parameter_name)
{
    ros::NodeHandle nh("~");

    XmlRpc::XmlRpcValue footprint_param_list;

    if (!nh.getParam(parameter_name, footprint_param_list))
        return false;

    ROS_ASSERT(footprint_param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(footprint_param_list.size() == 4);

    for (int32_t i = 0; i < footprint_param_list.size(); ++i)
    {
        ROS_ASSERT(footprint_param_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(footprint_param_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(footprint_param_list[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        geometry_msgs::Point32 point;
        point.x = static_cast<double>(footprint_param_list[i][0]);
        point.y = static_cast<double>(footprint_param_list[i][1]);

        footprint_.points.push_back(point);
    }

    return true;
}

void LaserDistances::oneLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    fromROSMsg(getCloudFromLaserScan(*scan), laser_scans_as_pcl_cloud);

    laser_scans_as_pcl_cloud_received = true;
}
void LaserDistances::twoSynchronizedLaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_1,
    const sensor_msgs::LaserScan::ConstPtr &scan_2)
{
    std::vector<sensor_msgs::LaserScan> scans;

    scans.push_back(*scan_1);
    scans.push_back(*scan_2);

    accumulateLaserScansToPointCloud(scans);
}

void LaserDistances::threeSynchronizedLaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_1,
    const sensor_msgs::LaserScan::ConstPtr &scan_2,
        const sensor_msgs::LaserScan::ConstPtr &scan_3)
{
    std::vector<sensor_msgs::LaserScan> scans;

    scans.push_back(*scan_1);
    scans.push_back(*scan_2);
    scans.push_back(*scan_3);

    accumulateLaserScansToPointCloud(scans);
}

void LaserDistances::accumulateLaserScansToPointCloud(const std::vector<sensor_msgs::LaserScan> &scans)
{
    sensor_msgs::PointCloud2 accumulated_scans, scan_as_cloud;

    for (size_t i = 0; i < scans.size(); ++i)
    {
        // get laser scan as pcl pointcloud
        scan_as_cloud = getCloudFromLaserScan(scans[i]);

        // concatenate both clouds
        pcl::concatenatePointCloud(accumulated_scans, scan_as_cloud, accumulated_scans);
    }

    pcl::fromROSMsg(accumulated_scans, laser_scans_as_pcl_cloud);

    laser_scans_as_pcl_cloud_received = true;
}

sensor_msgs::PointCloud2 LaserDistances::getCloudFromLaserScan(const sensor_msgs::LaserScan &scan)
{
    sensor_msgs::PointCloud2 cloud;
    try
    {
        transform_listener_.waitForTransform(scan.header.frame_id, target_frame_, scan.header.stamp,
            ros::Duration(0.1));
        laser_projector_.transformLaserScanToPointCloud(target_frame_, scan, cloud, transform_listener_);
    }
    catch (std::exception &e)
    {
        ROS_ERROR_STREAM("Could not transform laser scan into target frame: " << e.what());
    }

    return cloud;
}

void LaserDistances::update()
{
    if (laser_scans_as_pcl_cloud_received)
    {
        std::vector<int> indices;
        Eigen::Vector4f min_pt;
        Eigen::Vector4f max_pt;
        if (!laser_scans_as_pcl_cloud.points.empty())
        {
            min_pt[2] = laser_scans_as_pcl_cloud.points[0].z;
            max_pt[2] = laser_scans_as_pcl_cloud.points[0].z;
        }

        // footprint points are specified in the order:
        // rear right, rear left, front left, front right

        float length = footprint_.points[2].x - footprint_.points[1].x;
        float width = footprint_.points[2].y - footprint_.points[3].y;
        // rear
        min_pt[0] = footprint_.points[0].x - 10.0;
        min_pt[1] = footprint_.points[0].y;
        max_pt[0] = footprint_.points[0].x;
        max_pt[1] = footprint_.points[1].y;
        pcl::getPointsInBox(laser_scans_as_pcl_cloud, min_pt, max_pt, indices);
        float rear_distance = std::abs(getMaxX(laser_scans_as_pcl_cloud, indices)) - (length / 2.0);

        // left
        min_pt[0] = footprint_.points[1].x;
        min_pt[1] = footprint_.points[1].y;
        max_pt[0] = footprint_.points[2].x;
        max_pt[1] = footprint_.points[1].y + 10.0;
        pcl::getPointsInBox(laser_scans_as_pcl_cloud, min_pt, max_pt, indices);
        float left_distance = getMinY(laser_scans_as_pcl_cloud, indices) - (width / 2.0);

        // front
        min_pt[0] = footprint_.points[2].x;
        min_pt[1] = footprint_.points[3].y;
        max_pt[0] = footprint_.points[2].x + 10.0;
        max_pt[1] = footprint_.points[2].y;
        pcl::getPointsInBox(laser_scans_as_pcl_cloud, min_pt, max_pt, indices);
        float front_distance = getMinX(laser_scans_as_pcl_cloud, indices) - (length / 2.0);

        // right
        min_pt[0] = footprint_.points[0].x;
        min_pt[1] = footprint_.points[3].y;
        max_pt[0] = footprint_.points[3].x;
        max_pt[1] = footprint_.points[3].y - 10.0;
        pcl::getPointsInBox(laser_scans_as_pcl_cloud, min_pt, max_pt, indices);
        float right_distance = std::abs(getMaxY(laser_scans_as_pcl_cloud, indices)) - (width / 2.0);

        // output distances are in the order:
        // front, right, rear, left
        std_msgs::Float32MultiArray msg;
        msg.data.push_back(front_distance);
        msg.data.push_back(right_distance);
        msg.data.push_back(rear_distance);
        msg.data.push_back(left_distance);

        pub_distances_.publish(msg);

        laser_scans_as_pcl_cloud_received = false;
    }
}

float LaserDistances::getMaxX(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices)
{
    float max_x = -10.0;
    for (int i = 0; i < indices.size(); i++)
    {
        if (cloud.points[indices[i]].x > max_x)
        {
            max_x = cloud.points[indices[i]].x;
        }
    }
    return max_x;
}

float LaserDistances::getMinX(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices)
{
    float min_x = 10.0;
    for (int i = 0; i < indices.size(); i++)
    {
        if (cloud.points[indices[i]].x < min_x)
        {
            min_x = cloud.points[indices[i]].x;
        }
    }
    return min_x;
}

float LaserDistances::getMaxY(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices)
{
    float max_y = -10.0;
    for (int i = 0; i < indices.size(); i++)
    {
        if (cloud.points[indices[i]].y > max_y)
        {
            max_y = cloud.points[indices[i]].y;
        }
    }
    return max_y;
}

float LaserDistances::getMinY(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices)
{
    float min_y = 10.0;
    for (int i = 0; i < indices.size(); i++)
    {
        if (cloud.points[indices[i]].y < min_y)
        {
            min_y = cloud.points[indices[i]].y;
        }
    }
    return min_y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_distances");

    LaserDistances laser_distances_node;

    ros::NodeHandle nh("~");
    double loop_rate_hz = 100.0;
    nh.param<double>("loop_rate", loop_rate_hz, 100.0);
    ros::Rate loop_rate(loop_rate_hz);

    ROS_INFO("Node started");
    while (ros::ok())
    {
        ros::spinOnce();

        laser_distances_node.update();

        loop_rate.sleep();
    }
}
