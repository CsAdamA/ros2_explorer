#!/usr/bin/env python3

import numpy as np
from enum import IntEnum
import math 
from threading import Thread
import time
 
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
"""
from turtlebot4_navigation.action import Explorer

class ExplorerServer(Node):
    def __init__(self):
        super().__init__('discoverer_server')
        self._action_server = ActionServer(self, Explorer, 'discover', self.execute_callback)
        #self.watchtower_subscription = self.create_subscription(Float32, 'map_progress', self.watchtower_callback, 10)
        self.watchtower_subscription  # prevent unused variable warning
        self.navigation_client = TurtleBot4Explorer()
        self.stop_discovering = False
        self.map_completed_thres=1.0 #Initialize threshold to max (100%)
        self.get_logger().info("Discoverer Server is ready")

    def watchtower_callback(self, msg):
        # If map_progress is higher than the threshold send stop wandering signal
        if msg.data > self.map_completed_thres:
            self.stop_discovering = True

    def execute_callback(self, goal_handle):
        self.get_logger().info("Discoverer Server received a goal")
        #self.map_completed_thres=goal_handle.request.map_completed_thres
        #self.get_logger().info("Map completed threshold set to: %s" %self.map_completed_thres)
        #while not self.stop_discovering:
        while True:
            self.navigation_client.startToPose()

        self.get_logger().info('Discovering Finished')
        goal_handle.succeed()
        return Explorer.Result()
"""
class TurtleBot4Directions(IntEnum):
    NORTH = 0
    NORTH_WEST = 45
    WEST = 90
    SOUTH_WEST = 135
    SOUTH = 180
    SOUTH_EAST = 225
    EAST = 270
    NORTH_EAST = 315


class TurtleBot4Explorer(BasicNavigator):
    is_docked = None
    creating_path = False

    def __init__(self):
        super().__init__()
        self.create_subscription(DockStatus,
                                 'dock_status',
                                 self._dockCallback,
                                 qos_profile_sensor_data)

        self.create_subscription(PoseWithCovarianceStamped,
                                 'initialpose',
                                 self._poseEstimateCallback,
                                 qos_profile_system_default)

        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')

        self.cartographer = CartographerSubscriber()  # a cartographer subscription is created to access the occupancy
        rclpy.spin_once(self.cartographer)

        self.undock_result_future = None
        self.dock_result_future = None

        self.get_logger().info("Explorer Server is ready")
        if not self.getDockedStatus():
            self.info('Docking before intialising pose')
            self.dock()
        self.get_logger().info("Robot docked")

        initial_pose = self.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.setInitialPose(initial_pose)

        self.get_logger().info("Initial pose set")
        self.get_logger().info("Waiting for navigation server init")
        self.waitUntilNav2Active()
        self.get_logger().info("Navigation server active")


    def getPoseStamped(self, position, rotation): 
        """
        Fill and return a PoseStamped message.

        :param position: A list consisting of the x and y positions for the Pose. e.g [0.5, 1.2]
        :param rotation: Rotation of the pose about the Z axis in degrees.
        :return: PoseStamped message
        """
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]

        # Convert Z rotation to quaternion
        pose.pose.orientation.z = math.sin(math.radians(rotation) / 2)
        pose.pose.orientation.w = math.cos(math.radians(rotation) / 2)

        return pose

    def stampPose(self, pose):
        """
        Stamp a Pose message and return a PoseStamped message.

        :param pose: Pose message
        :return: PoseStamped message
        """
        poseStamped = PoseStamped()

        poseStamped.header.frame_id = 'map'
        poseStamped.header.stamp = self.get_clock().now().to_msg()

        poseStamped.pose = pose

        return poseStamped

    def createPath(self):
        """
        Create a path using the '2D Pose Estimate' tool in Rviz.

        :return: List of PoseStamped poses
        """
        poses = []
        self.new_pose = None
        self.creating_path = True

        self.info('Creating a path. Press Enter to finish.')
        self.info('Use the "2D Pose Estimate" tool in Rviz to add a pose to the path.')

        def wait_for_key():
            input()

        input_thread = Thread(target=wait_for_key, daemon=True)
        input_thread.start()

        while self.creating_path:
            while self.new_pose is None:
                if input_thread.is_alive():
                    rclpy.spin_once(self, timeout_sec=0.1)
                else:
                    self.creating_path = False
                    break
            if self.new_pose:
                self.info('Pose added.')
                poses.append(self.stampPose(self.new_pose))
                self.new_pose = None
                self.clearAllCostmaps()
        if len(poses) > 0:
            self.info('Path created.')
            for i, p in enumerate(poses):
                self.info('Pose {0} [x,y]=[{1:.3f},{2:.3f}]'.format(
                    i, p.pose.position.x, p.pose.position.y) +
                    '[x,y,z,w]=[{0:.3f},{1:.3f},{2:.3f},{3:.3f}]'.format(
                    p.pose.orientation.x, p.pose.orientation.y,
                    p.pose.orientation.z, p.pose.orientation.w))
        return poses

    # 2D Pose Estimate callback
    def _poseEstimateCallback(self, msg: PoseWithCovarianceStamped):
        if self.creating_path:
            self.new_pose = msg.pose.pose

    # DockStatus subscription callback
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def getDockedStatus(self):
        """
        Get current docked status.

        :return: ``True`` if docked, ``False`` otherwise.
        """
        # Spin to get latest dock status
        rclpy.spin_once(self, timeout_sec=0.1)
        # If dock status hasn't been published yet, spin until it is
        while self.is_docked is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.is_docked

    def undock(self):
        """Perform Undock action."""
        self.info('Undocking...')
        self.undock_send_goal()

        while not self.isUndockComplete():
            time.sleep(0.1)

    def undock_send_goal(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        goal_future = self.undock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()

    def isUndockComplete(self):
        """
        Get status of Undock action.

        :return: ``True`` if undocked, ``False`` otherwise.
        """
        if self.undock_result_future is None or not self.undock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)

        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Undock succeeded')
        return True

    def dock(self):
        """Perform Undock action."""
        self.info('Docking...')
        self.dock_send_goal()

        while not self.isDockComplete():
            time.sleep(0.1)

    def dock_send_goal(self):
        goal_msg = Dock.Goal()
        self.dock_action_client.wait_for_server()
        goal_future = self.dock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.dock_goal_handle = goal_future.result()

        if not self.dock_goal_handle.accepted:
            self.error('Dock goal rejected')
            return

        self.dock_result_future = self.dock_goal_handle.get_result_async()

    def isDockComplete(self):
        """
        Get status of Dock action.

        :return: ``True`` if docked, ``False`` otherwise.
        """
        if self.dock_result_future is None or not self.dock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.dock_result_future, timeout_sec=0.1)

        if self.dock_result_future.result():
            self.dock_status = self.dock_result_future.result().status
            if self.dock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Dock succeeded')
        return True

    def startToPose(self):
        """
        Perform goToPose action and print feedback.

        :param pose: Goal pose.
        """
        
        rclpy.spin_once(self.cartographer)  # refresh the list of accessible waypoints
        waypoint = self.cartographer.sorted_accessible_waypoints[0]  # grab the first waypoint
        self.cartographer.sorted_accessible_waypoints = self.cartographer.sorted_accessible_waypoints[1:]  # pop the

        pose = self.getPoseStamped([float(waypoint[0]), float(waypoint[1])], TurtleBot4Directions.NORTH)

        self.get_logger().info(
            'Sending navigation goal request x: ' + str(round(float(waypoint[0]), 2)) + ' y: ' + str(
                round(float(waypoint[1]), 2)))

        i = 0
        self.goToPose(pose)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            i = i + 1
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + '{0: <20}'.format('seconds.'), end='\r')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancelTask()

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')

    def startThroughPoses(self, poses):
        """
        Perform goThroughPoses action and print feedback.

        :param poses: List of goal poses.
        """
        i = 0
        self.goThroughPoses(poses)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            i = i + 1
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + '{0: <20}'.format(' seconds.'), end='\r')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancelTask()

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')

    def startFollowWaypoints(self, poses):
        """
        Perform followWaypoint action and print feedback.

        :param poses: List of goal poses.
        """
        i = 0
        self.followWaypoints(poses)

        while not self.isTaskComplete():
            i = i + 1
            feedback = self.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: {0}/{1: <5}'.format(
                    str(feedback.current_waypoint + 1), str(len(poses))), end='\r')
                
class CartographerSubscriber(Node):
    def __init__(self):
        super().__init__('cartographer_subscriber')
        self.occupancy_subscription = self.create_subscription(OccupancyGrid, 'map', self.occupancy_callback, 10)

        self.waypoints = self.generate_list_of_waypoints(n_of_waypoints=100, step=0.2)
        self.accessible_waypoints = np.array([])
        self.sorted_accessible_waypoints = np.array([])
        self.occupancy_value = np.array([])

    def occupancy_callback(self, msg):
        """

        The cartographer subscriber callback function refreshes the list of accessible waypoints. It sorts them and
        saves them in the self.sorted_accessible_waypoints variable.

        :param msg: OccupancyGrid message. Includes map metadata and an array with the occupancy probability values
        :return: None
        """

        data = np.array(msg.data)  # download the occupancy grid
        current_map_width = msg.info.width  # get the current map width
        current_map_height = msg.info.height  # get the current map height
        resolution = msg.info.resolution  # get the resolution

        # reshape the data so it resembles the map shape
        data = np.reshape(data, (current_map_height, current_map_width))

        # Here we go through every waypoint and save the ones that are accessible.
        # An accessible waypoint is one which has no obstacles, and has few or no unknown squares in the vicinity.
        self.accessible_waypoints = np.array([])
        self.occupancy_value = np.array([])
        for waypoint in self.waypoints:
            try:
                occupancy_grid_coordinates = [int((waypoint[1] + 2.3) / resolution), int((waypoint[0] + 2.3) /
                                                                                         resolution)]
                conv = self.convolute(data, occupancy_grid_coordinates, size=9)  # perform convolution

                # if the convolution returns True, it means the WP is accessible, so it is stored in
                # self.accessible_waypoints
                if conv[0]:
                    self.accessible_waypoints = np.append(self.accessible_waypoints, waypoint)
                    self.occupancy_value = np.append(self.occupancy_value, conv[1])
            # because the waypoint array is over-sized, we need to remove the values that are out of range
            except IndexError:
                pass

        # reshape the accessible waypoints array to shape (n, 2)
        self.accessible_waypoints = self.accessible_waypoints.reshape((-1, 2))

        # Sorting waypoints according to occupancy value. This allows the robot to prioritize the waypoints with
        # more uncertainty (it wont access the areas that are completely clear, thus going to the discovery frontier)
        occupancy_value_idxs = self.occupancy_value.argsort()
        self.sorted_accessible_waypoints = self.accessible_waypoints[occupancy_value_idxs[::-1]]

        # At the beginning, when all values are uncertain, we add some hardcoded waypoints so it begins to navigate
        # and has time to discover accessible areas
        if np.size(self.sorted_accessible_waypoints) == 0:
            self.sorted_accessible_waypoints = np.array([[1.5, 0.0], [0.0, 1.5], [-1.5, 0.0], [0.0, -1.5]])

        # Once we have the new waypoints, they are saved in self.sorted_accessible_waypoints for use by the Navigator
        # client
        self.get_logger().info('Accessible waypoints have been updated...')

    @staticmethod
    def convolute(data, coordinates, size=3, threshold=40):
        """
        This function calculates the average occupancy probability at 'coordinates' for an area of size (size x size)
        around said point.

        :param data: Occupancy Grid Data (shaped to (x, y) map dimensions)
        :param coordinates: the coordinates of the OccupancyGrid to convolute around
        :param size: size of the kernel
        :param threshold: threshold of accessibility
        :return: True or False, depending on whether the waypoint is accessible or not.
        :return: average: average occupancy probability of the convolution
        """
        sum = 0
        for x in range(int(coordinates[0] - size / 2), int(coordinates[0] + size / 2)):
            for y in range(int(coordinates[1] - size / 2), int(coordinates[1] + size / 2)):
                # if the area is unknown, we add 100 to sum.
                if data[x, y] == -1:
                    sum += 100
                # if occupancy state is above 50 (occupied), we add 1M to the sum so that the robot DOES NOT
                # access areas near walls.
                elif data[x, y] > 50:
                    sum += 1000000
                # if the occupancy state is below 50 and known, just add the value to sum.
                else:
                    sum += data[x, y]

        # average value for the square is computed
        average = sum / (size * size)
        if average < threshold:
            # if the average of the squares is below the threshold, the waypoint is accessible
            return True, average
        else:
            # if the average is above the threshold, the waypoint has either too many unknowns, or an obstacle
            return False, average

    def generate_list_of_waypoints(self, n_of_waypoints, step):
        """

        Generates a grid of waypoints of size ('n_of_waypoints' * 'n_of_waypoints') and step size 'step'

        :param n_of_waypoints: number of total waypoints to generate per side
        :param step: float resolution of the waypoints
        :return waypoints: 2D numpy array of a list of coordinates of size dim x 2,
        where dim is the number of waypoints
        """

        waypoints = np.zeros((n_of_waypoints * n_of_waypoints, 2))

        i = 0
        for index_y in range(n_of_waypoints):
            for index_x in range(n_of_waypoints):
                waypoints[i] = [float(index_x) / (1/step), float(index_y) / (1/step)]
                i += 1

        self.get_logger().info("Grid of waypoints has been generated.")
        return waypoints
