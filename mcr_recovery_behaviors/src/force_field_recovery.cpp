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
			
			//setting up cmd_vel publisher
			twist_pub_ = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
			
			//later on get this parameter from param server, at the moment just for testing
			scale_ = 0.75;
			
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
		
		//moving base away from obstacles
		move_base_away(local_costmap_);
	}
	
	void ForceFieldRecovery::move_base_away(costmap_2d::Costmap2DROS* costmap_ros)
	{
		//this function moves the mobile base away from obstacles based on a costmap
		
		//1. getting a snapshot of the costmap
		costmap_2d::Costmap2D* costmap_snapshot = costmap_ros->getCostmap();
		
		//2. convert obstacles inside costmap into pointcloud
		pcl::PointCloud<pcl::PointXYZ> cloud = costmap_to_pointcloud(costmap_snapshot);
		
		//3. compute force field
		Eigen::Vector3f force_field = compute_force_field(cloud);
		
		//4. publish force field as marker for visualization in rviz
		//todo
		
		//5. move base in the direction of the force field
		move_base(force_field(0)*scale_, force_field(1)*scale_);
		
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
		
		int x_cloud, y_cloud, z_cloud;
		const int LETHAL_COST = 254;
		
		const double x_origin_costmap = costmap->getOriginX();
		const double y_origin_costmap = costmap->getOriginY();
		const double resolution = costmap->getResolution();
		
		//for transforming map to world coordinates
		unsigned int map_x;
		unsigned int map_y;
		double world_x;
		double world_y;
		
		for(int i = 0; i < x_size_ ; i++)
		{
			for(int j = 0; j < y_size_ ; j++)
			{
				//getting each cost
				current_cost = costmap->getCost(i, j);
				
				//for debugging, remove later on
				ROS_INFO("costmap cost [%d][%d] = %d", i, j, current_cost);
				
				//if cell is occupied by obstacle then add the centroid of the cell to the cloud
				if(current_cost == LETHAL_COST)
				{
					//get world coordinates of current occupied cell
					costmap->mapToWorld(map_x, map_y, world_x, world_y);
					
					x_cloud = world_x;
					y_cloud = world_y;
					z_cloud = 0;
					
					//adding occupied cell centroid coordinates to cloud
					cloud.push_back (pcl::PointXYZ (x_cloud, y_cloud, z_cloud));
				}
			}
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
			ROS_INFO("Null force field, cloud is empty");
			return Eigen::Vector3f(0, 0, 0);
		}
		
		force_vector.normalize();
		force_vector = force_vector*1.0;
		
		return force_vector;
	}
	
	void ForceFieldRecovery::move_base(float x, float y)
	{
		geometry_msgs::Twist twist_msg;
		
		ROS_INFO("Moving base into the direction of the force field");
		
		twist_msg.linear.x = x;
		twist_msg.linear.y = y;
		twist_msg.linear.z = 0.0;
		twist_msg.angular.x = 0.0;
		twist_msg.angular.y = 0.0;
		twist_msg.angular.z = 0.0;
		twist_pub_.publish(twist_msg);
	}

};

/* example backup code:
 * 
 * 
 

std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();
tf::Stamped<tf::Pose> pose;

if(!costmap->getRobotPose(pose))
{
	ROS_ERROR("Cannot clear map because pose cannot be retrieved");
	return;
}

double x = pose.getOrigin().x();
double y = pose.getOrigin().y();

for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) 
{
	boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
	std::string name = plugin->getName();
	int slash = name.rfind('/');
	if( slash != std::string::npos )
	{
		name = name.substr(slash+1);
	}

	if(clearable_layers_.count(name)!=0)
	{
		boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
		costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
		clearMap(costmap, x, y);
	}
}

void ForceFieldRecovery::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, 
										double pose_x, double pose_y)
{
	boost::unique_lock< boost::shared_mutex > lock(*(costmap->getLock()));
	
	double start_point_x = pose_x - reset_distance_ / 2;
	double start_point_y = pose_y - reset_distance_ / 2;
	double end_point_x = start_point_x + reset_distance_;
	double end_point_y = start_point_y + reset_distance_;

	int start_x, start_y, end_x, end_y;
	costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
	costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

	unsigned char* grid = costmap->getCharMap();
	
	for(int x=0; x<(int)costmap->getSizeInCellsX(); x++)
	{
		bool xrange = x>start_x && x<end_x;
					
		for(int y=0; y<(int)costmap->getSizeInCellsY(); y++)
		{
			if(xrange && y>start_y && y<end_y)
				continue;
			int index = costmap->getIndex(x,y);
			if(grid[index]!=NO_INFORMATION)
			{
				grid[index] = NO_INFORMATION;
			}
		}
	}

	double ox = costmap->getOriginX(), oy = costmap->getOriginY();
	double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
	
	costmap->addExtraBounds(ox, oy, ox + width, oy + height);
	
	return;
}*/