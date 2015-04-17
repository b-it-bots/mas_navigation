#include <force_field_recovery/force_field_recovery.h>

//register this planner as a RecoveryBehavior plugin
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
			name_ = name;
			tf_ = tf;
			global_costmap_ = global_costmap;
			local_costmap_ = local_costmap;

			ROS_INFO("Initializing Force field recovery behavior...");
			
			//get some parameters from the parameter server
			ros::NodeHandle private_nh("~/" + name_);
			
			//set up cmd_vel publisher
			twist_pub_ = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
			
			//set up cloud publishers
			map_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2> ("/obstacle_cloud_map", 1);
			base_footprint_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2> ("/obstacle_cloud_base_link", 1);
			
			//later on get this parameter from param server, at the moment just for testing
			scale_ = 0.75;
			max_range_ = 0.75;
			
			//force_field_distance : how far away from obstacles the robot should move away
			//private_nh.param("force_field_distance", force_field_distance_, 0.2);

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

		//checking if the received costmaps are empty
		if(global_costmap_ == NULL || local_costmap_ == NULL)
		{
			ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
			return;
		}
		
		ROS_INFO("Running force field recovery behavior");
		
		//moving base away from obstacles
		move_base_away(local_costmap_);
	}
	
	void ForceFieldRecovery::move_base_away(costmap_2d::Costmap2DROS* costmap_ros)
	{
		//this function moves the mobile base away from obstacles based on a costmap
		
		//1. getting a snapshot of the costmap
		costmap_2d::Costmap2D* costmap_snapshot = costmap_ros->getCostmap();
		
		//2. publish local_costmap_frame
		broadcast_costmap_tf(costmap_snapshot);
		
		//3. convert obstacles inside costmap into pointcloud
		pcl::PointCloud<pcl::PointXYZ> obstacle_cloud = costmap_to_pointcloud(costmap_snapshot);
		
		//4. publish obstacle cloud
		sensor_msgs::PointCloud2 ros_obstacle_cloud = publish_cloud(obstacle_cloud, map_cloud_pub_, "/map");
		
		//5. Change cloud to the reference frame of the robot
		pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_bf = change_cloud_reference_frame(ros_obstacle_cloud, "/base_footprint");
		
		//6. publish base link obstacle cloud
		publish_cloud(obstacle_cloud_bf, base_footprint_cloud_pub_, "/base_footprint");
		
		//7. compute force field
		Eigen::Vector3f force_field = compute_force_field(obstacle_cloud_bf);
		
		//8. publish force field as marker for visualization in rviz
		//todo
		
		//9. move base in the direction of the force field
		move_base(force_field(0)*scale_, force_field(1)*scale_);
		
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
	
	sensor_msgs::PointCloud2 ForceFieldRecovery::publish_cloud(pcl::PointCloud<pcl::PointXYZ> cloud, ros::Publisher &cloud_pub, std::string frame_id)
	{
		//this function receives a pcl pointcloud, transforms into ros pointcloud and then publishes
		
		//debug
		//ROS_INFO("Publishing obstacle cloud");
		//Print points of the cloud in terminal
		pcl::PointCloud<pcl::PointXYZ>::const_iterator cloud_iterator = cloud.begin();
		int numPoints = 0;
		while (cloud_iterator != cloud.end())
		{
			//ROS_INFO("cloud [%d] = %f, %f, %f ", numPoints, (float)cloud_iterator->x, (float)cloud_iterator->y, (float)cloud_iterator->z);
			++cloud_iterator;
			numPoints++;
		}
		//ROS_INFO("total number of points in the cloud = %d", numPoints);
		
		//creating a pointcloud2 data type
		pcl::PCLPointCloud2 cloud2;
		
		//converting normal cloud to pointcloud2 data type
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
	
	void ForceFieldRecovery::broadcast_costmap_tf(const costmap_2d::Costmap2D* costmap)
	{
		//this function receives a costmap and publishes the origin of it as a tf
	
		float local_costmap_origin_x = costmap->getOriginX();
		float local_costmap_origin_y = costmap->getOriginY();
		
		tf::StampedTransform local_costmap_tf;
		static tf::TransformBroadcaster tf_broadcaster;
		
		//creating local costmap frame, setting origin
		local_costmap_tf.setOrigin( tf::Vector3(local_costmap_origin_x, local_costmap_origin_y, 0.0) );
		
		//no rotation for the local costmap
		local_costmap_tf.setRotation( tf::createQuaternionFromRPY(0.0, 0.0, 0.0) );
		
		//broadcasting transform with map as parent frame
		tf_broadcaster.sendTransform(tf::StampedTransform(local_costmap_tf, ros::Time::now(), "/map", "/local_costmap"));
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
		const int LETHAL_COST = 254;
		
		const double x_origin_costmap = costmap->getOriginX();
		const double y_origin_costmap = costmap->getOriginY();
		
		//for transforming map to world coordinates
		unsigned int map_x;
		unsigned int map_y;
		double world_x;
		double world_y;
		
		//debug
		bool lethal_flag = false;
		
		for(int i = 0; i < x_size_ ; i++) //attention i=0
		{
			for(int j = 0; j < y_size_ ; j++) //attention j=0
			{
				//getting each cost
				current_cost = costmap->getCost(i, j);
				
				//debug
				//ROS_INFO("i, j = %d, %d : cost = %d ", i, j, current_cost);
				
				//debug
				//ROS_INFO("costmap cost [%d][%d] = %d", i, j, current_cost);
				
				//if cell is occupied by obstacle then add the centroid of the cell to the cloud
				if(current_cost == LETHAL_COST)
				{
					//debug
					lethal_flag = true;
					
					map_x = i;
					map_y = j;
					
					//get world coordinates of current occupied cell
					costmap->mapToWorld(map_x, map_y, world_x, world_y);
					
					//debug
					//ROS_INFO("point %d, %d = %f, %f ",i ,j , (float)world_x, (float)world_y);
					
					//adding occupied cell centroid coordinates to cloud
					cloud.push_back (pcl::PointXYZ (world_x, world_y, 0));
				}
			}
		}
		
		//debug
		if(lethal_flag)
		{
			//ROS_INFO("lethal cost detected!!!!!!!!!!");
		}
		
		return cloud;
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
			
			//debug
			//ROS_INFO("Norm of the point : %f", each_point.norm());
			
			if (each_point.norm() < max_range_)
			{
				force_vector -= each_point;
				numPoints++;
			}
		
			++cloud_iterator;
		}

		if (numPoints == 0) 
		{
			//Cloud is empty
			//ROS_INFO("Null force field, cloud is empty");
			return Eigen::Vector3f(0, 0, 0);
		}
		
		force_vector.normalize();
		force_vector = force_vector*1.0;
		
		return force_vector;
	}
	
	void ForceFieldRecovery::move_base(float x, float y)
	{
		geometry_msgs::Twist twist_msg;
		
		//ROS_INFO("Moving base into the direction of the force field");
		
		twist_msg.linear.x = x;
		twist_msg.linear.y = y;
		twist_msg.linear.z = 0.0;
		twist_msg.angular.x = 0.0;
		twist_msg.angular.y = 0.0;
		twist_msg.angular.z = 0.0;
		//debug
		//twist_pub_.publish(twist_msg);
	}

};
