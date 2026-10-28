#!/usr/bin/env python3
"""Send the robot on a tour of the four rooms using Nav2 Simple Commander."""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# (name, x, y, yaw) in the map frame - one point per room, through the doorways
GOALS = [('NE room', 3.0, 2.0, 0.0),
         ('SE room', 2.0, -2.0, math.pi),
         ('SW room', -3.0, -2.0, math.pi / 2),
         ('NW room (start)', -3.0, 1.5, 0.0)]


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    if '--slam' in sys.argv:          # no AMCL in SLAM mode
        nav._waitForNodeToActivate('bt_navigator')
    else:
        nav.waitUntilNav2Active()

    for name, x, y, yaw in GOALS:
        nav.get_logger().info(f'Going to {name} ({x}, {y})')
        nav.goToPose(make_pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb:
                nav.get_logger().info(f'  distance remaining: {fb.distance_remaining:.2f} m',
                                      throttle_duration_sec=3.0)
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Reached {name}')
        else:
            nav.get_logger().error(f'Failed to reach {name}: {result}')
            break
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
