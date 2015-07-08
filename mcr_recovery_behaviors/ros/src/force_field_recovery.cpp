#include <force_field_recovery/force_field_recovery.h>

// Register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(force_field_recovery, ForceFieldRecovery, force_field_recovery::ForceFieldRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace force_field_recovery 
{
	ForceFieldRecovery::ForceFieldRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
	tf_(NULL), initialized_(false) 
	{
		//empty constructor
	}

	void ForceFieldRecovery::initialize(std::string name, tf::TransformListener* tf,
		costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
	{
		if(!initialized_)
		{
			//initialization, this code will be executed only once
			
			//receiving move_base variables and copying them over to class variables
			tf_ = tf;
			global_costmap_ = global_costmap;
			local_costmap_ = local_costmap;
			
			ros::NodeHandle private_nh("~/" + name);

			ROS_INFO("Initializing Force field recovery behavior...");
			
			// Getting values from parameter server and storing into class variables
			private_nh.param("velocity_scale_factor", velocity_scale_, 0.6);
			private_nh.param("obstacle_neighborhood", obstacle_neighborhood_, 0.6);
			private_nh.param("max_velocity", max_velocity_, 0.3);
			private_nh.param("timeout", timeout_, 3.0);
			private_nh.param("update_frequency", recovery_behavior_update_frequency_, 5.0);
			
			// Inform user about which parameters will be used for the recovery behavior
			ROS_INFO("Recovery behavior, using Force field velocity_scale parameter : %f", (float) velocity_scale_);
			ROS_INFO("Recovery behavior, using Force field obstacle_neighborhood parameter : %f", (float) obstacle_neighborhood_);
			ROS_INFO("Recovery behavior, using Force field max_velocity parameter : %f", (float) max_velocity_);
			ROS_INFO("Recovery behavior, using Force field timeout parameter : %f", (float) timeout_);
			ROS_INFO("Recovery behavior, using Force field recovery_behavior_update_frequency_ parameter : %f", (float) recovery_behavior_update_frequency_);
			
			//set up cmd_vel publisher
			twist_pub_ = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
			
			//set up cloud publishers topic
			map_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2> ("/obstacle_cloud_map", 1);
			base_footprint_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2> ("/obstacle_cloud_base_link", 1);
			
			//setting initialized flag to true, preventing this code to be executed twice
			initialized_ = true;
		}
		else
		{
			ROS_ERROR("You should not call initialize twice on this object, doing nothing");
		}
	}

	void ForceFieldRecovery::runBehavior()
	{
		//preventing the use of this code before initialization
		if(!initialized_)
		{
			ROS_ERROR("This object must be initialized before runBehavior is called");
			return;
		}

		//checking if the received costmaps are empty, if so exit
		if(global_costmap_ == NULL || local_costmap_ == NULL)
		{
			ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
			return;
		}
		
		ROS_INFO("Running force field recovery behavior");
		
		// Moving base away from obstacles
		move_base_away(local_costmap_);
	}
	
	void ForceFieldRecovery::move_base_away(costmap_2d::Costmap2DROS* costmap_ros)
	{
		//this function moves the mobile base away from obstacles based on a costmap
		
		ros::Time start_time = ros::Time::now();
		
		bool no_obstacles_in_radius = false;
		bool detect_oscillation_is_enabled =false;
		int oscillations = 0;
		int loop_number = 0;
		double current_angle, previous_angle, angle_difference;
		
		ros::Rate loop_rate(recovery_behavior_update_frequency_);
		
		//while certain time (timeout) or no obstacles inside radius do the loop 
		while((ros::Duration(ros::Time::now() - start_time).toSec() < timeout_) && !no_obstacles_in_radius)
		{
			//1. getting a snapshot of the costmap
			costmap_2d::Costmap2D* costmap_snapshot = costmap_ros->getCostmap();
			
			//2. convert obstacles inside costmap into pointcloud
			pcl::PointCloud<pcl::PointXYZ> obstacle_cloud = costmap_to_pointcloud(costmap_snapshot);
			
			//3. publish obstacle cloud
			sensor_msgs::PointCloud2 ros_obstacle_cloud = publish_cloud(obstacle_cloud, map_cloud_pub_, "/map");
			
			//4. Change cloud to the reference frame of the robot
			pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_bf = change_cloud_reference_frame(ros_obstacle_cloud, "/base_footprint");
			
			//5. publish base link obstacle cloud
			publish_cloud(obstacle_cloud_bf, base_footprint_cloud_pub_, "/base_footprint");
			
			//6. compute force field
			Eigen::Vector3f force_field = compute_force_field(obstacle_cloud_bf);
			
			if(force_field(0) == 0 && force_field(1) == 0)
			{
				no_obstacles_in_radius = true;
				break;
			}
			else if(oscillations > 0)
			{
				ROS_INFO("Oscillation detected! , will stop now...");
				break;
			}
				
			//7. detect oscillation on the force field
			
			ROS_INFO("loop : %d", loop_number++);
			ROS_INFO("force field : x = %f, y = %f", (float) force_field(0), (float) force_field(1));

			//first time do not check for oscillations
			if(detect_oscillation_is_enabled)
			{
				
				//get the new force field angle
				current_angle = atan2(force_field(1) , force_field(0));
				
				ROS_INFO("previous angle : %f", (float) previous_angle);
				ROS_INFO("current angle : %f", (float) current_angle);

				//compare the angles
				angle_difference = atan2(sin(current_angle - previous_angle), cos(current_angle - previous_angle));
				
				ROS_INFO("angle_difference = %f", (float) angle_difference);
				
				if(fabs(angle_difference) > 1.8)
				{
					ROS_INFO("A big change in direction of the force filed was detected");
					oscillations ++;
				}

				//making backup of the previous force field angle
				previous_angle = current_angle;
			}
			else
			{
				//compute angle of the first force field
				previous_angle = atan2(force_field(1) , force_field(0));
				
				ROS_INFO("--------");
				ROS_INFO("force field x : %f", (float) force_field(0));
				ROS_INFO("force field y : %f", (float) force_field(1));
				ROS_INFO("angle : %f", (float) previous_angle);
				ROS_INFO("--------");

				//from second time, check for oscillations
				detect_oscillation_is_enabled = true;
			}
			
			//8. move base in the direction of the force field
			move_base(force_field(0)*velocity_scale_, force_field(1)*velocity_scale_);
			
			//9. Control the frequency update for costmap update
			loop_rate.sleep();

		}
		
		if(no_obstacles_in_radius)
		{
			ROS_INFO("Force field recovery succesfull");
		}
		else
		{
			ROS_WARN("Force field recovery behavior time out exceeded");
		}
		
		//10. stop the base
		move_base(0.0, 0.0);
	}
	
	pcl::PointCloud<pcl::PointXYZ> ForceFieldRecovery::costmap_to_pointcloud(const costmap_2d::Costmap2D* costmap)
	{
		
		// This function transforms occupied regions of a costmap, letal cost = 254 to 
		// pointcloud xyz coordinate
		
		//for storing and return the pointcloud
		pcl::PointCloud<pcl::PointXYZ> cloud;
		
		int x_size_ = costmap->getSizeInCellsX();
		int y_size_ = costmap->getSizeInCellsY();
		
		int current_cost = 0;
		
		//for transforming map to world coordinates
		double world_x;
		double world_y;
		
		for(int i = 0; i < x_size_ ; i++)
		{
			for(int j = 0; j < y_size_ ; j++)
			{
				//getting each cost
				current_cost = costmap->getCost(i, j);
				
				ROS_DEBUG("i, j = %d, %d : cost = %d ", i, j, current_cost);
				ROS_DEBUG("costmap cost [%d][%d] = %d", i, j, current_cost);
				
				//if cell is occupied by obstacle then add the centroid of the cell to the cloud
				if(current_cost == LETHAL_COST)
				{
					//get world coordinates of current occupied cell
					costmap->mapToWorld(i, j, world_x, world_y);
					
					ROS_DEBUG("point %d, %d = %f, %f ",i ,j , (float) world_x, (float) world_y);
					
					//adding occupied cell centroid coordinates to cloud
					cloud.push_back (pcl::PointXYZ (world_x, world_y, 0));
				}
			}
		}
		
		return cloud;
	}
	
	sensor_msgs::PointCloud2 ForceFieldRecovery::publish_cloud(pcl::PointCloud<pcl::PointXYZ> cloud, ros::Publisher &cloud_pub, std::string frame_id)
	{
		// This function receives a pcl pointcloud, transforms into ros pointcloud and then publishes the cloud
		
		ROS_DEBUG("Publishing obstacle cloud");
		
		// Print points of the cloud in terminal
		pcl::PointCloud<pcl::PointXYZ>::const_iterator cloud_iterator = cloud.begin();
		
		int numPoints = 0;
		
		while (cloud_iterator != cloud.end())
		{
			ROS_DEBUG("cloud [%d] = %f, %f, %f ", numPoints, (float)cloud_iterator->x, (float)cloud_iterator->y, (float)cloud_iterator->z);
			++cloud_iterator;
			numPoints++;
		}
		
		ROS_DEBUG("total number of points in the cloud = %d", numPoints);
		
		// Creating a pointcloud2 data type
		pcl::PCLPointCloud2 cloud2;
		
		// Converting normal cloud to pointcloud2 data type
		pcl::toPCLPointCloud2(cloud, cloud2);
		
		//declaring a ros pointcloud data type
		sensor_msgs::PointCloud2 ros_cloud;
		
		//converting pointcloud2 to ros pointcloud
		pcl_conversions::fromPCL(cloud2, ros_cloud);
		
		//assigning a frame to ros cloud
		ros_cloud.header.frame_id = frame_id;
		
		//publish the cloud
		cloud_pub.publish(ros_cloud);
		
		//returning the cloud, it could be useful for other components
		return ros_cloud;
	}
	
	pcl::PointCloud<pcl::PointXYZ> ForceFieldRecovery::change_cloud_reference_frame(sensor_msgs::PointCloud2 ros_cloud, std::string target_reference_frame)
	{
		//This function receives a ros cloud (with an associated tf) and tranforms 
		//all the points to another reference frame (target_reference_frame)
		
		//declaring the target ros pcl data type
		sensor_msgs::PointCloud2 target_ros_pointcloud;
		
		//changing pointcloud reference frame
   
		// declaring normal PCL clouds (not ros related)
		pcl::PointCloud<pcl::PointXYZ> cloud_in;
		pcl::PointCloud<pcl::PointXYZ> cloud_trans;
		
		//convert from rospcl to pcl
		pcl::fromROSMsg(ros_cloud, cloud_in);
		
		//STEP 1 Convert xb3 message to center_bumper frame (i think it is better this way)
		tf::StampedTransform transform;
		try
		{
			tf_->lookupTransform(target_reference_frame, ros_cloud.header.frame_id, ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
		}
		
		// Transform point cloud
		pcl_ros::transformPointCloud (cloud_in, cloud_trans, transform);  
		
		return cloud_trans;
	}
	
	Eigen::Vector3f ForceFieldRecovery::compute_force_field(pcl::PointCloud<pcl::PointXYZ> cloud)
	{
		// This function receives a cloud and returns the negative of the resultant
		// assuming that all points in the cloud are vectors
		
		Eigen::Vector3f force_vector(0, 0, 0);
		
		pcl::PointCloud<pcl::PointXYZ>::const_iterator cloud_iterator = cloud.begin();
		int numPoints = 0;

		while (cloud_iterator != cloud.end())
		{
			Eigen::Vector3f each_point(cloud_iterator->x, cloud_iterator->y, 0);
			
			ROS_DEBUG("Norm of the point : %f", each_point.norm());
			
			if (each_point.norm() < obstacle_neighborhood_)
			{
				force_vector -= each_point;
				numPoints++;
			}
		
			++cloud_iterator;
		}

		if (numPoints == 0) 
		{
			//Cloud is empty
			
			return Eigen::Vector3f(0, 0, 0);
		}
		
		force_vector.normalize();
		force_vector = force_vector * 1.0;
		
		ROS_DEBUG("Force vector = (%f, %f)", (float) force_vector(0), (float) force_vector(1));
		
		return force_vector;
	}
	
	void ForceFieldRecovery::move_base(double x, double y)
	{
		geometry_msgs::Twist twist_msg;
		
		//clamping x and y to maximum speed value
		if(x > max_velocity_) x = max_velocity_;
		else if(x < -max_velocity_) x = -max_velocity_;
		
		if(y > max_velocity_) y = max_velocity_;
		else if(y < -max_velocity_) y = -max_velocity_;
		
		//ROS_INFO("Moving base into the direction of the force field x = %f, y = %f", (float) x, (float) y);
		
		twist_msg.linear.x = x;
		twist_msg.linear.y = y;
		twist_msg.linear.z = 0.0;
		twist_msg.angular.x = 0.0;
		twist_msg.angular.y = 0.0;
		twist_msg.angular.z = 0.0;
		
		twist_pub_.publish(twist_msg);
	}

};