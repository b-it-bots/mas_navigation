/* Force field recovery behavior
 * 
 * Original idea from : https://github.com/daenny
 * 
 * Main code idea mostly taken from : 
 * 
 * https://github.com/smARTLab-liv/smartlabatwork-release/blob/master/slaw_registration/src/forcefield_recovery.cpp
 * 
 * Refactored, enhaced and mantained by: 	Oscar Lima (olima_84@yahoo.com)
 * 
 * About this code: Plugin for the mobile base (move_base ros) : Recovery behavior
 * that moves away from obstacles by using the costmap. It calculates the vector resultant
 * in a pointcloud of obstacles and moves away from the cluster by publishing on cmd_vel topic
 * 
 */

#ifndef FORCE_FIELD_RECOVERY_H_
#define FORCE_FIELD_RECOVERY_H_

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

// For transforming costmap occupied cells into pointcloud
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// For moving the mobile base (publish in cmd_vel)
#include <geometry_msgs/Twist.h>

// Ros PCL includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

//For publishing local costmap tf
#include <tf/transform_broadcaster.h>

//For pointcloud reference frame transformer
#include <pcl_ros/transforms.h>

#define LETHAL_COST 254

namespace force_field_recovery
{
	/**
	* @class ForceFieldRecovery
	* @brief A recovery behavior that moves the base away from obstacles when stucked
	*/
	class ForceFieldRecovery : public nav_core::RecoveryBehavior 
	{
		public:
		/**
		* @brief  Constructor, make sure to call initialize in addition to actually initialize the object
		* @param  Empty constructor, no arguments
		*/
		ForceFieldRecovery();

		/**
		* @brief  Initialization function for the ForceField recovery behavior
		* Receives tf, global costmap, local costmap and copies their values into member variables
		* Also setups the publishers and reads values from the parameter server
		* 
		* @param tf A pointer to a transform listener
		* @param global_costmap A pointer to the global_costmap used by the navigation stack 
		* @param local_costmap A pointer to the local_costmap used by the navigation stack 
		*/
		void initialize(std::string name, tf::TransformListener* tf, 
			costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

		/**
		* @brief  Run the ForceFieldRecovery recovery behavior.
		* This function checks if the costmaps are empty (if so then exits) and calls move_base_away \
		* function
		*/
		void runBehavior();

		private:
			
		/**
		* @brief  This function moves the mobile base away from obstacles based on a costmap
		*
		* step 1. Get the current costmap
		* step 2. Convert all obstacles (cost = 254) inside local costmap into pointcloud
		* step 3. Publish the previous obtained obstacle pointcloud for debugging purposes
		* step 4. Change cloud to the reference frame of the robot (base footprint) since 
		*         it was in map frame and needs to be in the reference frame of the robot
		* step 5. Publish base link obstacle cloud for debugging purposes, the cloud must match
		*         with the previous obstacle pointcloud
		* step 6. Compute force field vector as the negative of the resultant of all points 
		*         assuming that they are vectors
		* step 7. Publish force field vector as marker for visualization in rviz (not implemented yet)
		* step 8. move base in the direction of the force field vector by multiplying the normalized
		*         vector times a scale factor which is a parameter that can be changed
		*
		* @param costmap A pointer to the local_costmap used by the navigation stack 
		*/
		void move_base_away(costmap_2d::Costmap2DROS* costmap);
		
		/**
		* @brief  Iterates over the costmap and when it finds a cost = 254 (obstacle or lethal cost)
		* it appends this point to a pointcloud called obstacle cloud
		* 
		* @param costmap A pointer to the local_costmap used by the navigation stack 
		*/
		pcl::PointCloud<pcl::PointXYZ> costmap_to_pointcloud(const costmap_2d::Costmap2D* costmap);
		
		/**
		* @brief  Receives a cloud transforms to ros pcl format and publishes the cloud for debugging purposes
		* @param cloud A pcl pointcloud to be published
		* @param cloud_pub The ros publisher object to execute the method .publish
		* @param frame_id The frame id of the pointcloud to be published
		*/
		sensor_msgs::PointCloud2 publish_cloud(pcl::PointCloud<pcl::PointXYZ> cloud, ros::Publisher &cloud_pub, std::string frame_id);
		
		/**
		* @brief  Transform all points inside the pointcloud to a different target reference frame
		* @param ros_cloud A ros pointcloud to be transformed
		* @param target_reference_frame The reference frame in which you want the pointcloud to be converted
		*/
		pcl::PointCloud<pcl::PointXYZ> change_cloud_reference_frame(sensor_msgs::PointCloud2 ros_cloud, std::string target_reference_frame);
		
		/**
		* @brief  Inputs a pointcloud and returns the negative of the resultant of the cloud
		* assuming that the points inside the cloud are vectors
		* @param cloud pointcloud of obstacles expresed in the reference frame of the robot
		*/
		Eigen::Vector3f compute_force_field(pcl::PointCloud<pcl::PointXYZ> cloud);
		
		/**
		* @brief  Move the base by publishing on cmd_vel a certain Vx and Vy
		* @param x The x velocity to be send to the mobile base
		* @param y The y velocity to be send to the mobile base
		*/
		void move_base(double x, double y);
		
		/*
		 * private member variables
		 *
		 */
		
		// Flag used for preventing the class to initialize more than one time
		bool initialized_; 
		
		// A pointer to the transform listener sent by move_base
		tf::TransformListener* tf_; 
		
		// Pointers to receive the costmaps from move_base
		costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_; 
		
		// The distance in meters away from the robot to include obstacles in the force field calculation
		double obstacle_neightborhood_;
		
		// The factor to which the force field will be multyplied before it is sent to the mobile base as velocity
		double force_field_to_velocity_scale_;
		
		// A twist publisher for cmd_vel used to publish a velocity to the mobile base
		ros::Publisher twist_pub_;
		
		// A pointcloud publisher that will publish the obstacle cloud in map reference frame
		ros::Publisher map_cloud_pub_;
		
		// A pointcloud publisher that will publish the obstacle cloud in base_footprint reference frame
		ros::Publisher base_footprint_cloud_pub_;
	};
};

#endif  
