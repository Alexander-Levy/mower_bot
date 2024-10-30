from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav_api.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


def main():
    rclpy.init()

    navigator = BasicNavigator()

    security_route = [
        [0.0, 0.2],
        [3.0, 0.2],
        [3.0, 1.0],
        [0.0, 1.0],
        [0.0, 1.8],
        [3.0, 1.8],
        [3.0, 2.5],
        [0.0, 2.5]]

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    #navigator.setInitialPose(initial_pose)
    
    # Wait for navigation to fully activate
    #navigator.waitUntilNav2Active('bt_navigator', 'slam_toolbox')

    # Do security route until dead
    while rclpy.ok():
        # Send our route
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in security_route[1:]:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))
        
        nav_start = navigator.get_clock().now()
        #navigator.followWaypoints(route_poses)
        navigator.goThroughPoses(route_poses)

        # Do something during our route (e.x. AI detection on camera images for anomalies)
        # Simply print ETA for the demonstation
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                # Some navigation timeout to demo cancellation
                now = navigator.get_clock().now()
                if now - nav_start > Duration(seconds=600.0):
                    navigator.cancelTask()

        # If at end of route, reverse the route to restart
        security_route.reverse()

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('Route complete! Restarting...')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().info('Security route was canceled, exiting.')
            rclpy.shutdown()
        elif result == TaskResult.FAILED:
            navigator.get_logger().info('Security route failed! Restarting from other side...')

    rclpy.shutdown()


if __name__ == '__main__':
    main()