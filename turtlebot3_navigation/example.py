

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def set_route(navigator) -> list:
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    inspection_pose.pose.orientation.z = 1.0
    inspection_pose.pose.orientation.w = 0.0
    return inspection_points

def set_pose(navigator) -> PoseStamped():
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0
    initial_pose.pose.position.y = 0
    initial_pose.pose.orientation.z = 0.816279
    initial_pose.pose.orientation.w = 0.577657
    return initial_pose

def print_results(result) -> None:
    if result == TaskResult.SUCCEEDED:
        print('PATH Finding completed')
    elif result == TaskResult.CANCELED:
        print('PATH Finding interrupted')
        exit(1)
    elif result == TaskResult.FAILED:
        print('PATH Finding failed')

def send_route(inspection_points, inspection_pose, inspection_route):
    for pt in inspection_route:
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_points.append(deepcopy(inspection_pose))

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    inspection_route = [
        [0.533665, -0.544118],
        [4.27509, 5.92226],
        [2.28073, 8.95063],
        [-3.29666, 5.04603]]

    # Set your demo's initial pose
    initial_pose = set_pose(navigator)
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully 
    navigator.waitUntilNav2Active()

    # Send your route
    inspection_points = set_route(navigator)
    send_route(inspection_points, inspection_pose, inspection_route)
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(inspection_points)

    # Do something during your route (e.x. AI to analyze stock information or upload to the cloud)
    # Print the current waypoint ID for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

    result = navigator.getResult()
    print_results(result)

    # go back to start
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()
