/* Force field recovery behavior
 * 
 * Author : https://github.com/daenny
 * 
 * code mostly taken from : 
 * 
 * https://github.com/smARTLab-liv/smartlabatwork-release/blob/master/slaw_registration/src/forcefield_recovery.cpp
 * 
 * refactored by: 	Oscar Lima
 * 					olima_84@yahoo.com
 * 
 * About this library:	Inputs a pointcloud in the constructor and computes the -determinant
 * (sum of vectors). This is useful for example : when you want to get away from obstacles.
 * This code provides you with the best direction for getting away from a cluster.
 * 
 */

#ifndef FORCE_FIELD_RECOVERY_H_
#define FORCE_FIELD_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>

#include <pluginlib/class_list_macros.h>
#include <vector>

#include <costmap_2d/costmap_2d.h>

//for transforming costmap occupied cells into pointcloud
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//for moving the mobile base (publish in cmd_vel)
#include <geometry_msgs/Twist.h>

//debug
//#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

//for publishing local costmap tf
#include <tf/transform_broadcaster.h>

//for pointcloud reference frame transformer
#include <pcl_ros/transforms.h>

namespace force_field_recovery
{
	/**
	* @class ForceFieldRecovery
	* @brief A recovery behavior that moves the base away from obstacles
	*/
	class ForceFieldRecovery : public nav_core::RecoveryBehavior 
	{
		public:
		/**
		* @brief  Constructor, make sure to call initialize in addition to actually initialize the object
		* @param  
		* @return 
		*/
		ForceFieldRecovery();

		/**
		* @brief  Initialization function for the ForceField recovery behavior
		* @param tf A pointer to a transform listener
		* @param global_costmap A pointer to the global_costmap used by the navigation stack 
		* @param local_costmap A pointer to the local_costmap used by the navigation stack 
		*/
		void initialize(std::string name, tf::TransformListener* tf, 
			costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

		/**
		* @brief  Run the ForceFieldRecovery recovery behavior. 
		* 1. transform occupied cells into vectors
		* 2. sum all vectors to get the resultant
		* 3. move the base into opposite direction from the resultant
		*/
		void runBehavior();

		private:
			
		//private functions
		void move_base_away(costmap_2d::Costmap2DROS* costmap);
		pcl::PointCloud<pcl::PointXYZ> costmap_to_pointcloud(const costmap_2d::Costmap2D* costmap);
		Eigen::Vector3f compute_force_field(pcl::PointCloud<pcl::PointXYZ> cloud);
		void broadcast_costmap_tf(const costmap_2d::Costmap2D* costmap);
		sensor_msgs::PointCloud2 publish_cloud(pcl::PointCloud<pcl::PointXYZ> cloud, ros::Publisher &cloud_pub, std::string frame_id);
		pcl::PointCloud<pcl::PointXYZ> change_cloud_reference_frame(sensor_msgs::PointCloud2 ros_cloud, std::string target_reference_frame);
		void move_base(float x, float y);
		
		//private member variables
		bool initialized_;
		std::string name_;
		tf::TransformListener* tf_;
		costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
		float max_range_;
		float scale_; //the scale factor for multypling the force field vector
		ros::Publisher twist_pub_;
		
		//debug
		ros::Publisher map_cloud_pub_;
		ros::Publisher base_footprint_cloud_pub_;
	};
};

#endif  
