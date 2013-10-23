#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <XmlRpcValue.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <angles/angles.h>

#include <mcr_navigation_msgs/MoveRelative.h>

#include "mcr_relative_movements/homogenous_transform.h"


using namespace std;

double Velocity = 0.1;


class BaseMotionController

{

   private:

   // Preferred Displacement and Orientation
   float xval;
   float yval;
   float rollval;
   float pitchval;
   float yawval;

   // Init Odom value
   float x_initodom;
   float y_initodom;
   float theta_initodom;

   // Init Odom value
   float x_tempodom;
   float y_tempodom;
   float theta_tempodom;


   // Current Odom value
   float x_currentodom;
   float y_currentodom;
   float theta_currentodom;

   //temporary variables
   double roll, pitch, yaw;
   tf::Quaternion q;

   bool odom_received;
   // Odometry subscriber and Base Velocity Publisher
   ros::Publisher   base_velocities_publisher;   
   ros::Subscriber  base_odom;

   //base velcoity topic message
   geometry_msgs::Twist youbot_base_velocities;

   ros::NodeHandle node_handler;

   public:
   BaseMotionController( ros::NodeHandle &n ,float x,float y, float roll,float pitch,float yaw):node_handler(n)
   {
        xval = x;
        yval = y;
        rollval=roll;
        pitchval=pitch;
        yawval = yaw;
        // Velocity control for the YouBot base.
        base_velocities_publisher = node_handler.advertise<geometry_msgs::Twist>( "/cmd_vel", 1 );
        base_odom = node_handler.subscribe("/odom", 1, &BaseMotionController::OdomCallback, this);
        odom_received = false;
   }
   ~BaseMotionController()
   {
        base_odom.shutdown(); 
        base_velocities_publisher.shutdown(); 
   }

   bool movebase()
   {
	   if( rotate() )
		   if( moveY() )
			   return moveX();
                           
   } 
   bool moveX()
   {
        bool isReached=false;
        double init_time;
        double curr_time;
        geometry_msgs::Twist zero_vel; 

        base_velocities_publisher.publish(zero_vel);

	    while(!odom_received)
        {
        	ros::spinOnce();   
        }
        odom_received = false;
        
        x_initodom = x_tempodom;
        y_initodom = y_tempodom;
        double max_time = (fabs(xval)*500) + 50;

        std::cout << "shift in x: " << xval << std::endl;
        std::cout << "init odom: x:" << x_initodom << " y: " << y_initodom << std::endl;

        theta_initodom = theta_tempodom;
        x_currentodom = x_tempodom;
        y_currentodom = y_tempodom;
        theta_currentodom = theta_tempodom;
        cout<<x_initodom<<endl;
        /*
        while(ros::Time::now().toSec() <= 0)
        {
           init_time = ros::Time::now().toSec();
           
        }*/
        init_time = 0; 
        cout<<init_time<<endl;
        while(isReached != true)
        {
            base_velocities_publisher.publish(zero_vel);

            float valuediff = (x_currentodom-x_initodom);
            if( fabs(valuediff) >= fabs(xval))
            {
            	youbot_base_velocities.linear.x = 0.0;
                isReached =true;
            }
            else
            {
             xval>0?(youbot_base_velocities.linear.x = Velocity):(youbot_base_velocities.linear.x = -Velocity);
             base_velocities_publisher.publish( youbot_base_velocities );  
              
            }
            while(!odom_received)
            {
            	ros::spinOnce();   
            }
            odom_received = false;

            x_currentodom = x_tempodom;
            y_currentodom = y_tempodom;
            theta_currentodom = theta_tempodom; 

            //curr_time = ros::Time::now().toSec();
            init_time += 1;
            //cout<<init_time<<endl;
            if(init_time > max_time)
            {
               base_velocities_publisher.publish(zero_vel);
               return false;  
            }
            
        }   

        
        base_velocities_publisher.publish(zero_vel);

        cout<<x_currentodom<<endl;

        return true;
   }
   bool moveY()
   {
        geometry_msgs::Twist zero_vel;

        base_velocities_publisher.publish(zero_vel);

        bool isReached=false;

        while(!odom_received)
        {
        	ros::spinOnce();   
        }
        odom_received = false;

        x_initodom = x_tempodom;
        y_initodom = y_tempodom;


        std::cout << "shift in y: " << yval << std::endl;
        std::cout << "init odom: x:" << x_initodom << " y: " << y_initodom << std::endl;

        theta_initodom = theta_tempodom;
        x_currentodom = x_tempodom;
        y_currentodom = y_tempodom;
        theta_currentodom = theta_tempodom;

        while(isReached != true)
        {
            //ROS_INFO("Run");  
            base_velocities_publisher.publish(zero_vel);
            float valuediff = (y_currentodom-y_initodom);
            
            std::cout << "diff y: " << valuediff << std::endl;
            
            if( fabs(valuediff) >= fabs(yval))
            {
                std::cout << "pose reached, current odom: " << y_currentodom <<  std::endl;
                youbot_base_velocities.linear.y = 0.0;
                isReached =true;
            }
            else
            {
                yval>0?(youbot_base_velocities.linear.y = Velocity):(youbot_base_velocities.linear.y = -Velocity);
                
                base_velocities_publisher.publish( youbot_base_velocities );  
              
            }
            while(!odom_received)
            {
            	ros::spinOnce();   
            }
            odom_received = false;
            
            x_currentodom = x_tempodom;
            y_currentodom = y_tempodom;
            theta_currentodom = theta_tempodom;
  
        }  
        
        base_velocities_publisher.publish(zero_vel);
        
        return true; 

   }
   bool rotate()
   {
        bool isReached=false;

        while(!odom_received)
        {
           	ros::spinOnce();   
        }
        odom_received = false;

        x_initodom = x_tempodom;
        y_initodom = y_tempodom;
        theta_initodom = theta_tempodom;
        x_currentodom = x_tempodom;
        y_currentodom = y_tempodom;
        theta_currentodom = theta_tempodom;
        while(isReached != true)
        {
            float valuediff = (theta_currentodom-theta_initodom);
            if(fabs(valuediff)>3.1416){valuediff=theta_currentodom+theta_initodom;}
            if( fabs(valuediff) >= fabs(yawval))
            {
            	youbot_base_velocities.angular.z = 0.0;
            	isReached =true;
            }
            else
            {
                yawval>0?(youbot_base_velocities.angular.z = Velocity):(youbot_base_velocities.angular.z = -Velocity);
                base_velocities_publisher.publish( youbot_base_velocities );  
              
            }
            
            while(!odom_received)
            {
            	ros::spinOnce();   
            }
            odom_received = false;

            x_currentodom = x_tempodom;
            y_currentodom = y_tempodom;
            theta_currentodom = theta_tempodom;
  
        }   

        geometry_msgs::Twist zero_vel;
        base_velocities_publisher.publish(zero_vel);

       return true;
   } 

    void OdomCallback( const nav_msgs::Odometry& Odom )
    {
        x_tempodom = Odom.pose.pose.position.x ;
        y_tempodom  = Odom.pose.pose.position.y ;
        tf::quaternionMsgToTF(Odom.pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);  
        theta_tempodom = yaw ;
	    odom_received = true; 
    } 
      

};

bool move_base_relative(mcr_navigation_msgs::MoveRelative::Request  &req, mcr_navigation_msgs::MoveRelative::Response &res)
{
    tf::Quaternion q;
    double roll, pitch, yaw;

    float X_dist = req.pose.pose.position.x ;
    float Y_dist = req.pose.pose.position.y ;

    Velocity = req.pose.pose.position.z;

    tf::quaternionMsgToTF(req.pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);  

    float Theta = yaw ;
 
    ros::NodeHandle node;
    
    BaseMotionController bm(node,X_dist,Y_dist,0,0,Theta);

    bool status = bm.movebase();
    if (status == true)
    {
        res.status.data = "success";
    }
    else
    {
        res.status.data = "failure_obtacle_front";
     
    }

    return true;
} 

 

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "relative_movements");

  ros::NodeHandle n;

  ros::ServiceServer shift_base = n.advertiseService( "move_base_relative", move_base_relative);


  ROS_INFO("Ready to move base position");

  ros::spin();
  return 0;
}
