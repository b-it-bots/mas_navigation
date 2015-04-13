#include <force_field_recovery/force_field_recovery.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(force_field_recovery, ClearCostmapRecovery, force_field_recovery::ForceFieldRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace force_field_recovery 
{
	ForceFieldRecovery::ForceFieldRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
	tf_(NULL), initialized_(false) {} 

	void ForceFieldRecovery::initialize(std::string name, tf::TransformListener* tf,
		costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
	{
		if(!initialized_)
		{
			name_ = name;
			tf_ = tf;
			global_costmap_ = global_costmap;
			local_costmap_ = local_costmap;

			//get some parameters from the parameter server
			ros::NodeHandle private_nh("~/" + name_);

			private_nh.param("reset_distance", reset_distance_, 3.0);
			
			std::vector<std::string> clearable_layers_default, clearable_layers;
			clearable_layers_default.push_back( std::string("obstacles") );
			private_nh.param("layer_names", clearable_layers, clearable_layers_default);

			for(unsigned i=0; i < clearable_layers.size(); i++) 
			{
				ROS_INFO("Recovery behavior will clear layer %s", clearable_layers[i].c_str());
				clearable_layers_.insert(clearable_layers[i]);
			}

			initialized_ = true;
		}
		else
		{
			ROS_ERROR("You should not call initialize twice on this object, doing nothing");
		}
	}

	void ForceFieldRecovery::runBehavior()
	{
		if(!initialized_)
		{
			ROS_ERROR("This object must be initialized before runBehavior is called");
			return;
		}

		if(global_costmap_ == NULL || local_costmap_ == NULL)
		{
			ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
			return;
		}
		
		ROS_WARN("Clearing costmap to unstuck robot (%fm).", reset_distance_);
		
		clear(global_costmap_);
		clear(local_costmap_);
	}

	void ForceFieldRecovery::clear(costmap_2d::Costmap2DROS* costmap)
	{
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
	}

};
