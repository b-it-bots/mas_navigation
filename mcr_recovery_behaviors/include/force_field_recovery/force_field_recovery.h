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

namespace force_field_recovery{
  /**
   * @class ForceFieldRecovery
   * @brief A recovery behavior that moves the base away from obstacles
   */
  class ForceFieldRecovery : public nav_core::RecoveryBehavior {
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
      void clear(costmap_2d::Costmap2DROS* costmap);      
      void clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, double pose_x, double pose_y);
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_;
      double reset_distance_;
      std::set<std::string> clearable_layers_; ///< Layer names which will be cleared.
  };
};
#endif  
