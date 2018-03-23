#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This component moves the mobile base in Cartesian space until a pose is reached.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_monitoring_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_monitoring_msgs.msg
import dynamic_reconfigure.server
from mcr_common_converters_ros.transform_to_pose_converter import TransformToPoseConverter
from mcr_manipulation_measurers_ros.component_wise_pose_error_calculator import ComponentWisePoseErrorCalculator
from mcr_geometric_relation_monitors_ros.component_wise_pose_error_monitor import ComponentWisePoseErrorMonitor
from mcr_twist_controller_ros.twist_controller import TwistController
from mcr_twist_limiter_ros.twist_limiter import TwistLimiter
from mcr_twist_synchronizer_ros.twist_synchronizer import TwistSynchronizer
import mcr_direct_base_controller.cfg.DirectBaseControllerConfig as DynamicBaseControllerConfig


class DirectBaseControllerCoordinator(object):
    """
    Components that move the base until a pose is reached.

    """
    def __init__(self):
        # params
        self.started_components = False
        self.event = None
        self.pose_monitor_feedback = None
        self.pose_2 = None
        self.has_pose_error_data = False

        self.transform_to_pose_converter = TransformToPoseConverter();
        self.component_wise_pose_error_calculator = ComponentWisePoseErrorCalculator();
        self.component_wise_pose_error_monitor = ComponentWisePoseErrorMonitor();
        self.feedback = mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback()

        self.twist_controller = TwistController();
        self.twist_limiter = TwistLimiter();
        self.twist_synchronizer = TwistSynchronizer();

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100.0))
        self.near_zero = rospy.get_param('~near_zero', 0.001)

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)

        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(
            DynamicBaseControllerConfig, self.dynamic_reconfigure_cb
        )

        self.pose_error_monitor_event_in = rospy.Publisher(
            '~pose_error_monitor_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )

        self.base_twist = rospy.Publisher('~twist', geometry_msgs.msg.Twist, queue_size=1)

        # Setup for component_wise_pose_error_calculator

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~pose_monitor_feedback", mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback,
                         self.pose_monitor_feedback_cb)
        rospy.Subscriber('~pose_2', geometry_msgs.msg.PoseStamped, self.pose_2_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def pose_2_cb(self, msg):
        """
        Obtains the second pose.

        """
        self.pose_2 = msg

    def pose_monitor_feedback_cb(self, msg):
        """
        Obtains the feedback from the pose error monitor component

        """
        self.pose_monitor_feedback = msg

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """import group3_direct_base_controller.cfg.TransformToPoseConverterConfig as TransformToPoseConverterConfig

        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            self.event = None
            return 'RUNNING'
        else:
            return 'INIT'


    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        if self.event == 'e_stop':
            self.event_out.publish('e_stopped')
            self.event = None
            self.pose_monitor_feedback = None
            self.started_components = False

            self.publish_zero_velocities()

            return 'INIT'

        # Get converted pose and calculate the pose error and publish
        converted_pose = self.transform_to_pose_converter.get_converted_pose()
        if(converted_pose != None):
            pose_error = self.component_wise_pose_error_calculator.get_component_wise_pose_error(converted_pose, self.pose_2)
            if self.component_wise_pose_error_monitor.isComponentWisePoseErrorWithinThreshold(pose_error):
                self.event_out.publish('e_success')
                self.event = None
                self.pose_monitor_feedback = None
                self.started_components = False
                self.publish_zero_velocities()
                return 'INIT'
            else:
                cartesian_velocity = self.twist_controller.get_cartesian_velocity(pose_error)
                # rospy.loginfo(cartesian_velocity)
                if cartesian_velocity:
                    limited_twist = self.twist_limiter.get_limited_twist(cartesian_velocity)
                    # rospy.loginfo(limited_twist)
                    if (limited_twist != None):
                        synchronized_twist = self.twist_synchronizer.synchronize_twist(limited_twist, pose_error)
                        if (synchronized_twist != None):
                            self.base_twist.publish(synchronized_twist.twist)
                        else:
                            self.event_out.publish('e_failure')
                        self.twist_synchronizer.reset_component_data()

        return 'RUNNING'

    def publish_zero_velocities(self):
        zero_twist = geometry_msgs.msg.Twist()

        self.base_twist.publish(zero_twist)

    def dynamic_reconfigure_cb(self, config, level):
        """
        Obtains an update for the dynamic reconfigurable parameters.

        """
        self.component_wise_pose_error_monitor.set_parameters(config.threshold_linear_x,\
            config.threshold_linear_y,\
            config.threshold_linear_z,\
            config.threshold_angular_x,\
            config.threshold_angular_y,
            config.threshold_angular_z,
            )
        self.component_wise_pose_error_calculator.set_parameters(config.wait_for_transform)
        self.twist_controller.set_parameters(config.p_gain_x,\
            config.p_gain_y,\
            config.p_gain_z,\
            config.p_gain_roll,\
            config.p_gain_yaw,\
            config.p_gain_pitch,\
            )
        self.twist_limiter.set_parameters(config.max_velocity_x,\
            config.max_velocity_y,\
            config.max_velocity_z,\
            config.max_velocity_yaw,\
            config.max_velocity_roll,\
            config.max_velocity_pitch\
            )
        return config

def main():
    rospy.init_node("direct_base_controller", anonymous=True)
    direct_base_controller_coordinator = DirectBaseControllerCoordinator()
    direct_base_controller_coordinator.start()
