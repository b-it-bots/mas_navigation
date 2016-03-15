/* 
 * Copyright [2015] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Listens to nav_msgs Path topic (which contains a global plan for the mobile base) as an array  
 * of poses and calculates the path lenght based on the distance between two points of each pose.  
 * 
 */

#include <mcr_navigation_tools/path_length_calculator_node.h>
#include <string>

PathLengthCalcNode::PathLengthCalcNode() : nh_("~")
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &PathLengthCalcNode::eventInCallback, this);
    global_plan_sub_ = nh_.subscribe("plan", 1, &PathLengthCalcNode::globalPlanCallback, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 2);
    path_length_pub_ = nh_.advertise<std_msgs::Float64>("path_length", 1);
}

PathLengthCalcNode::~PathLengthCalcNode()
{
    // shut down publishers and subscribers
    sub_event_in_.shutdown();
    global_plan_sub_.shutdown();
    pub_event_out_.shutdown();
    path_length_pub_.shutdown();
}

void PathLengthCalcNode::init()
{
    // set initial member variables values
    callback_received_ = false;
    node_frequency_ = 0.0;
    global_plan_is_available_ = false;
    ROS_INFO("Path length calculator node initialized...");
}

void PathLengthCalcNode::getParams()
{
    // getting required parameters from parameter server
    nh_.param("node_frequency", node_frequency_, 10.0);

    // informing the user about the parameters which will be used
    ROS_INFO("Node will run at : %lf [hz]", node_frequency_);
}

void PathLengthCalcNode::eventInCallback(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    callback_received_ = true;
}

void PathLengthCalcNode::globalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    global_plan_ = *msg;
    global_plan_is_available_ = true;
}

void PathLengthCalcNode::update()
{
    // for publishing event_out string msg
    std_msgs::String even_out_msg;
    even_out_msg.data = "not_set";

    // setting the frequency at which the node will run
    ros::Rate loop_rate(node_frequency_);

    while (ros::ok())
    {
        // listen to callbacks
        ros::spinOnce();

        if (callback_received_)
        {
            // lower flag
            callback_received_ = false;

            // checking for event in msg content
            if (event_in_msg_.data == "e_trigger")
            {
                if (global_plan_is_available_)
                {
                    // stores the result of the generic length class to be published later on
                    std_msgs::Float64 path_length;

                    // set global plan
                    path_length_calculator_.setPath(global_plan_);

                    // calculate path length of global plan (array of poses)
                    path_length.data = path_length_calculator_.computeLength();

                    if (path_length.data == -1)
                    {
                        ROS_ERROR("Error while calculating path length");

                        // publish even_out : "e_failure"
                        even_out_msg.data = std::string("e_failure");
                        pub_event_out_.publish(even_out_msg);
                    }
                    else
                    {
                        ROS_INFO("path length = %lf [m]", path_length.data);

                        // publish result
                        path_length_pub_.publish(path_length);

                        ROS_INFO("Path length succesfully calculated !");
                        // publish even_out : "e_success"
                        even_out_msg.data = std::string("e_success");
                        pub_event_out_.publish(even_out_msg);
                    }

                    // reset flag
                    global_plan_is_available_ = false;
                }
                else
                {
                    ROS_ERROR("event_in : trigger was received, but there is not path");
                    ROS_WARN("Did you already query about its length before?");

                    // publish even_out : "e_failure"
                    even_out_msg.data = std::string("e_failure");
                    pub_event_out_.publish(even_out_msg);
                }
            }
            else
            {
                // publish even_out : "e_failure"
                even_out_msg.data = std::string("e_failure");
                pub_event_out_.publish(even_out_msg);
                ROS_ERROR("event_in message received not known, admissible strings are : e_trigger");
            }
        }

        // sleep to control the node frequency
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "path_length_calculator_node");

    // create object of this node class
    PathLengthCalcNode path_length_calc_node;

    // initialize
    path_length_calc_node.init();

    // get parameters
    path_length_calc_node.getParams();

    // main loop function
    path_length_calc_node.update();

    return 0;
}

