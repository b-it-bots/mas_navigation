#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This component moves the mobile base in Cartesian space until a pose is reached.
The input pose must be provided in some static world frame.
"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_monitoring_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_monitoring_msgs.msg
import dynamic_reconfigure.server
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
        self.event = None
        self.target_pose = None
        self.collision_filter_feedback = None

        self.component_wise_pose_error_calculator = ComponentWisePoseErrorCalculator()
        self.component_wise_pose_error_monitor = ComponentWisePoseErrorMonitor()
        self.feedback = mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback()

        self.twist_controller = TwistController()
        self.twist_limiter = TwistLimiter()
        self.twist_synchronizer = TwistSynchronizer()

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100.0))
        self.idle_loop_rate = rospy.Rate(rospy.get_param('~idle_loop_rate', 1.0))

        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        self.use_collision_avoidance = rospy.get_param('~use_collision_avoidance', True)

        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(
            DynamicBaseControllerConfig, self.dynamic_reconfigure_cb
        )

        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)

        self.base_twist = rospy.Publisher('~twist', geometry_msgs.msg.Twist, queue_size=1)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~target_pose', geometry_msgs.msg.PoseStamped, self.target_pose_cb)

        if self.use_collision_avoidance:
            rospy.Subscriber("/mcr_navigation/collision_velocity_filter/event_out", std_msgs.msg.String,
                             self.collision_filter_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def target_pose_cb(self, msg):
        """
        Obtains the target pose.

        """
        self.target_pose = msg

    def collision_filter_cb(self, msg):
        """
        Obtains collision velocity filter feedback.
        """
        self.collision_filter_feedback = msg.data

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
            if state == 'INIT':
                self.idle_loop_rate.sleep()
            elif state == 'RUNNING':
                self.loop_rate.sleep()

    def init_state(self):
        """

        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            self.event = None
            self.collision_filter_feedback = None
            return 'RUNNING'
        elif self.event == 'e_stop':
            self.event = None
            self.collision_filter_feedback = None
            self.publish_zero_velocities()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            return 'INIT'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        if self.event == 'e_stop':
            self.event = None
            self.publish_zero_velocities()
            self.event_out.publish('e_stopped')
            return 'INIT'

        origin_pose = geometry_msgs.msg.PoseStamped()
        origin_pose.header.frame_id = self.base_frame
        origin_pose.pose.orientation.w = 1.0

        pose_error = self.component_wise_pose_error_calculator.get_component_wise_pose_error(origin_pose, self.target_pose)
        if not pose_error:
            self.event_out.publish('e_success')
            self.publish_zero_velocities()
            return 'INIT'

        if self.component_wise_pose_error_monitor.isComponentWisePoseErrorWithinThreshold(pose_error):
            self.event_out.publish('e_success')
            self.publish_zero_velocities()
            return 'INIT'
        else:
            if self.use_collision_avoidance and \
                        (self.collision_filter_feedback == "e_zero_velocities_forwarded"):
                self.event_out.publish('e_failure')
                self.publish_zero_velocities()
                return 'INIT'

            cartesian_velocity = self.twist_controller.get_cartesian_velocity(pose_error)
            if cartesian_velocity:
                limited_twist = self.twist_limiter.get_limited_twist(cartesian_velocity)
                if (limited_twist is not None):
                    synchronized_twist = self.twist_synchronizer.synchronize_twist(limited_twist, pose_error)
                    if (synchronized_twist is not None):
                        self.base_twist.publish(synchronized_twist.twist)
                    else:
                        self.event_out.publish('e_failure')
                        self.publish_zero_velocities()
                        return 'INIT'
                else:
                    self.event_out.publish('e_failure')
                    self.publish_zero_velocities()
                    return 'INIT'

        return 'RUNNING'

    def publish_zero_velocities(self):
        zero_twist = geometry_msgs.msg.Twist()
        self.base_twist.publish(zero_twist)

    def dynamic_reconfigure_cb(self, config, level):
        """
        Obtains an update for the dynamic reconfigurable parameters.

        """
        self.component_wise_pose_error_monitor.set_parameters(config.threshold_linear_x,
                                                              config.threshold_linear_y,
                                                              config.threshold_linear_z,
                                                              config.threshold_angular_x,
                                                              config.threshold_angular_y,
                                                              config.threshold_angular_z)
        self.component_wise_pose_error_calculator.set_parameters(config.wait_for_transform)
        self.twist_controller.set_parameters(config.p_gain_x,
                                             config.p_gain_y,
                                             config.p_gain_z,
                                             config.p_gain_roll,
                                             config.p_gain_yaw,
                                             config.p_gain_pitch)
        self.twist_limiter.set_parameters(config.max_velocity_x,
                                          config.max_velocity_y,
                                          config.max_velocity_z,
                                          config.max_velocity_yaw,
                                          config.max_velocity_roll,
                                          config.max_velocity_pitch)
        self.twist_synchronizer.set_angular_synchronization(config.angular_synchronization)
        self.loop_rate = rospy.Rate(config.loop_rate)
        return config


def main():
    rospy.init_node("direct_base_controller", anonymous=True)
    direct_base_controller_coordinator = DirectBaseControllerCoordinator()
    direct_base_controller_coordinator.start()
