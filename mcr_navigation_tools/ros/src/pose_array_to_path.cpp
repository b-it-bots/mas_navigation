/* 
 * Copyright [2016] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Subscribes to pose array topic, republishes as nav_msgs/Path topic
 * 
 */

#include <mcr_navigation_tools/pose_array_to_path.h>

PoseArrayToPath::PoseArrayToPath() : nh_("~")
{
    // subscriptions
    sub_ = nh_.subscribe("pose_array_to_convert", 1, &PoseArrayToPath::poseArrayCallback, this);

    // publications
    pub_ = nh_.advertise<nav_msgs::Path>("pose_array_to_path_converted", 2);
}

PoseArrayToPath::~PoseArrayToPath()
{
    // shut down publishers and subscribers
    sub_.shutdown();
    pub_.shutdown();
}

void PoseArrayToPath::init()
{
    // initial message
    ROS_INFO("Pose array to path converter node initialized...");

    // set initial member variables values
    callback_received_ = false;
    node_frequency_ = 0.0;
}

void PoseArrayToPath::getParams()
{
    // getting required parameters from parameter server
    nh_.param("node_frequency", node_frequency_, 10.0);

    // informing the user about the parameters which will be used
    ROS_INFO("Node will run at : %lf [hz]", node_frequency_);
}

void PoseArrayToPath::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    callback_received_ = true;
    pose_array_msg_ = *msg;
}

void PoseArrayToPath::update()
{
    // setting the frequency at which the node will run
    ros::Rate loop_rate(node_frequency_);

    // to publish the pose array as path
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose_stamped_msg;

    while (ros::ok())
    {
        if (callback_received_)
        {
            // lower flag
            callback_received_ = false;

            // republish pose array msg as path msg
            path_msg.header = pose_array_msg_.header;
            pose_stamped_msg.header = pose_array_msg_.header;

            path_msg.poses.resize(pose_array_msg_.poses.size());

            // loop over all pose arrary poses
            for (int i = 0; i < pose_array_msg_.poses.size(); i++)
            {
                // create intermediate pose stamped
                pose_stamped_msg.pose = pose_array_msg_.poses[i];

                // append the last pose stamped to path
                path_msg.poses[i] = pose_stamped_msg;
            }

            // publish path
            pub_.publish(path_msg);
        }

        // listen to callbacks
        ros::spinOnce();

        // sleep to control the node frequency
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "pose_array_to_path_node");

    // create object of this node class
    PoseArrayToPath pose_array_to_path_node;

    // initialize
    pose_array_to_path_node.init();

    // get parameters
    pose_array_to_path_node.getParams();

    // main loop function
    pose_array_to_path_node.update();

    return 0;
}
