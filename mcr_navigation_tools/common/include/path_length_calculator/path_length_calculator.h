/*  
* Copyright [2015] <Bonn-Rhein-Sieg University>  
*  
* Author: Oscar Lima (olima_84@yahoo.com)
* 
* Listens to nav_msgs Path topic (which contains a global plan for the mobile base) as an array  
* of poses and calculates the path lenght based on the distance between two points of each pose.  
*   
*/ 

#ifndef PATH_LENGTH_CALCULATOR_PATH_LENGTH_CALCULATOR_H
#define PATH_LENGTH_CALCULATOR_PATH_LENGTH_CALCULATOR_H

#include <math.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <pcl/registration/distances.h>

class PathLengthCalculator
{
    public:
        PathLengthCalculator();

        explicit PathLengthCalculator(nav_msgs::Path plan);

        // to set the value of member variable plan_ from outside of the class
        void set_path(nav_msgs::Path &plan);

        // to calculate the length of the path
        double compute_length();

    private:
        double distance_between_two_points(double &x0, double &y0, double &x1, double &y1);

        // to store the plan path (array of poses) from which the distance will be calculated
        nav_msgs::Path plan_;

        // flag that controls if compute_length function can be called
        bool plan_is_set_;
};

#endif  // PATH_LENGTH_CALCULATOR_PATH_LENGTH_CALCULATOR_H
