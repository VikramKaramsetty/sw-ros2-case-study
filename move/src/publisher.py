#!/usr/bin/env python3
"""Starter node for the Lunabotics ROS 2 case study.

Fill in TASKS 1-3 here. See README.md for the full description of each task.

Run it with:

    ros2 run move publisher

As shipped this node starts, spins, and does nothing -- that is intentional. Use it to
confirm your workspace is built and sourced before you write any logic.

Each task is marked with a TASK n.n comment matching the README. Commented-out lines are
deliberate: uncomment and complete them.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64


class RobotController(Node):
    """Drives the robot, tracks its position error, and flags obstacles."""

    def __init__(self):
        super().__init__('robot_controller')

        # Publish to /error when the position delta exceeds this (TASK 2.3).
        # This is a starting value -- justify whatever you settle on.
        self.error_thresh = 0.5

        # ---- TASK 1.2: publisher that drives the robot ---------------------
        # Which topic moves the robot? Find it first (TASK 1.1), then uncomment.
        #
        self.move_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #
        # Then drive it on a timer:
        self.move_timer = self.create_timer(0.1, self.send_move_cmd)

        # ---- TASK 1.3: the path you chose ----------------------------------
        # Pick a route that gets the robot around the wall, and represent it
        # however you think is best -- a list of waypoints, a sequence of
        # timed velocity commands, a parametric curve, something else.
        
        # Document HERE why you chose this path and this representation.

        # I want to drive the robot with a list of waypoints and a simple driving approach. 
        # I think this is the best approach for this task since it allows for the most amount of control and change over time
        # In the future, since the waypoints are already established I could easily develop splines or parametric curves to go around the wall
        # For right now, waypoints make it easy to visualize, easy to change, and easy to implement.
        # I'm implementing it in a list of tuples since it's simple and easy to parse for when implementing actual movement.
    # wall y coordinates are -3 to 3, x is 5.75 to 6.25
        self.path =[
            (0.0, 0.0),
            (5, 4.0),
            (6.3,4.0),
            (7.5,0)
        ]
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.path_commands = self.build_path_commands()
        self.command_index = 0
        self.command_ticks = 0

        # That reasoning is a large part of what we are evaluating.

        # ---- TASK 2.2: subscriber for the robot's 6D pose ------------------
        # One of the two onboard sensors reports 6D data. Find it (TASK 2.1).
        #
        # self.robot_pos_sub = self.create_subscription(
        #     <TODO: msg type>,
        #     '<TODO: topic name>',
        #     self.on_robot_pos,
        #     qos_profile_sensor_data,
        # )

        # ---- TASK 2.3: where the measured-vs-actual error goes -------------
        # self.error_pub = self.create_publisher(Float64, '/error', 10)
        #
        # Hint: ground truth for "actual" is published by the simulator on the
        # robot's odometry topic (nav_msgs/Odometry). Deciding what to compare,
        # and in which frame, is part of the task.

        # ---- TASK 3: lidar in, filtered obstacles out ----------------------
        # The lidar has a single vertical sample, so this cloud is one flat
        # row of points at the sensor's height -- not a 3D volume.
        #
        # self.lidar_sub = self.create_subscription(
        #     PointCloud2,
        #     '/lidar/points',
        #     self.on_lidar,
        #     qos_profile_sensor_data,
        # )
        # self.obstacle_pub = self.create_publisher(
        #     PointCloud2, '/obstacle_cloud', 10)

        self.get_logger().info(
            'robot_controller started (scaffold -- nothing wired up yet)')

    # -----------------------------------------------------------------------
    # TASK 1.2 -- publish a velocity command
    # -----------------------------------------------------------------------
    def send_move_cmd(self):
        """Publish one Twist that moves the robot along self.path.

        Works by:
        1. Going through the self.path
        2. Taking index 0 as the current position and the rest as targets.
        3. Define a set linearx and angular z speed.
        4. Converting each segment into a separate turn and forward command
            4a. This happens by calculating the angle to move and the distance to travel.
            4b. The command is appended to the command list and returned
        5. After commands are built, each tick we go through the commadn lsit and execute
        """
        command = Twist()

        if self.command_index < len(self.path_commands):
            linear_x, angular_z, duration = self.path_commands[self.command_index]
            command.linear.x = linear_x
            command.angular.z = angular_z
            self.command_ticks += 1

            if self.command_ticks >= duration:
                self.command_index += 1
                self.command_ticks = 0

        self.move_pub.publish(command)

    def build_path_commands(self):
        commands = []
        current_x, current_y = self.path[0]
        current_heading = 0.0

        for target_x, target_y in self.path[1:]:
            delta_x = target_x - current_x
            delta_y = target_y - current_y
            distance = math.hypot(delta_x, delta_y)
            target_heading = math.atan2(delta_y, delta_x)
            turn = math.atan2(
                math.sin(target_heading - current_heading),
                math.cos(target_heading - current_heading),
            )

            if abs(turn) > 0.0001:
                commands.append((0.0, math.copysign(self.angular_speed, turn),
                                 max(1, round(abs(turn) / self.angular_speed / 0.1))))
            if distance > 0.0001:
                commands.append((self.linear_speed, 0.0,
                                 max(1, round(distance / self.linear_speed / 0.1))))

            #updating all values for the next iteration
            current_x, current_y = target_x, target_y
            current_heading = target_heading

        return commands

    # -----------------------------------------------------------------------
    # TASK 2.3 -- compare reported position against ground truth
    # -----------------------------------------------------------------------
    def on_robot_pos(self, msg):
        """Compare the sensor's idea of where we are against the truth.

        Publish a Float64 on self.error_pub when the delta exceeds
        self.error_thresh.

        TODO: decide what "delta" means here and justify it in a comment.
        """
        raise NotImplementedError('TASK 2.3')

    # -----------------------------------------------------------------------
    # TASK 3.3 -- classify a single lidar point
    # -----------------------------------------------------------------------
    def is_obstacle(self, point):
        """Return True if `point` is something we must avoid.

        The barrier is passable -- treat it like dust in the air. The poles are
        not. `point` is an (x, y, z) tuple in the lidar's frame.

        TODO: decide what separates a pole from the barrier and implement it.
        """
        raise NotImplementedError('TASK 3.3')

    # -----------------------------------------------------------------------
    # TASK 3.2 -- filter the scan and republish what matters
    # -----------------------------------------------------------------------
    def on_lidar(self, msg):
        """Filter incoming points through is_obstacle and republish.

        point_cloud2.read_points(msg, field_names=('x', 'y', 'z')) iterates the
        cloud; point_cloud2.create_cloud_xyz32(msg.header, pts) builds the
        outgoing one.

        TODO: keep only the obstacle points and publish on self.obstacle_pub.
        """
        raise NotImplementedError('TASK 3.2')


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
