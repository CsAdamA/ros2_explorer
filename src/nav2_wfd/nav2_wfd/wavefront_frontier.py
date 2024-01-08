#! /usr/bin/env python3
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from enum import Enum

import numpy as np
import math
import time

OCC_THRESHOLD = 10
MIN_FRONTIER_SIZE = 5

class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        #print(f"Map size: {self.map.info.width, self.map.info.height}, index {self.__getIndex(mx, my)}, coord {mx, my}")
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception(f"World coordinates out of bounds {wx, wy} {self.map.info.origin.position.x, self.map.info.origin.position.y}")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception(f"Out of bounds: {mx, my}, {self.map.info.height, self.map.info.width}")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx

class FrontierCache():
    cache = {}

    def getPoint(self, x, y):
        idx = self.__cantorHash(x, y)

        if idx in self.cache:
            return self.cache[idx]

        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def __cantorHash(self, x, y):
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear(self):
        self.cache = {}

class FrontierPoint():
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y

class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8

def centroid(arr):
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x/length, sum_y/length

def findFree(mx, my, costmap):
    fCache = FrontierCache()

    bfs = [fCache.getPoint(mx, my)]

    while len(bfs) > 0:
        loc = bfs.pop(0)

        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)

def getFrontier(pose, costmap, logger):
    fCache = FrontierCache()

    fCache.clear()

    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)

    freePoint = findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = []

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        if isFrontierPoint(p, costmap, fCache):
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = []

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)

                if q.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:
                    continue

                if isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        if w.classification & (PointClassification.FrontierOpen.value | PointClassification.FrontierClosed.value | PointClassification.MapClosed.value) == 0:
                            w.classification = w.classification | PointClassification.FrontierOpen.value
                            frontierQueue.append(w)

                q.classification = q.classification | PointClassification.FrontierClosed.value

            
            newFrontierCords = []
            for x in newFrontier:
                x.classification = x.classification | PointClassification.MapClosed.value
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))

            if len(newFrontier) > MIN_FRONTIER_SIZE:
                frontiers.append(tuple(centroid(newFrontierCords)))

        for v in getNeighbors(p, costmap, fCache):
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers
        
def getNeighbors(point, costmap, fCache):
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))

    return neighbors

def isFrontierPoint(point, costmap, fCache):
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)

        if cost > OCC_THRESHOLD:
            return False

        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree

class WaypointExplorer(Node):
    def __init__(self):
        super().__init__(node_name='explorer', namespace='')
        self.currentPose = None
        self.currentOrientation = None
        self.selectedLocations = {}
        self.invalidLocations = []
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose', 10)
        self.audio_publisher = self.create_publisher(AudioNoteVector, '/cmd_audio', 1)
        self.goal_pubisher = self.create_publisher(PointStamped, '/clicked_point', 1)
        self.frontier_publisher = self.create_publisher(Marker, '/frontier_markers', 50)
        self.costmapClient = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.costmapClient.wait_for_service(timeout_sec=1.0):
            self.info_msg('service not available, waiting again...')
        self.initial_pose_received = True
        self.goal_handle = None

        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.model_pose_sub = self.create_subscription(Odometry,
                                                       '/odom', self.poseCallback, pose_qos)
        self.initial_pose_sub = self.create_subscription(Pose,
                                                       '/new_initialpose', self.initPoseCallback, pose_qos)
        self.costmapSub = self.create_subscription(OccupancyGrid(), '/map', self.occupancyGridCallback, pose_qos)
        self.costmap = None

        self.new_init_pose = None
        while self.new_init_pose == None:
            self.info_msg("Waiting for initial pose...")
            rclpy.spin_once(self.initial_pose_sub, timeout_sec=1.0)

    
    def initPoseCallback(self, msg):
        self.info_msg(f"Initial pose received: {msg.position.x}, {msg.position.y}")
        self.new_init_pose = msg
    
    def occupancyGridCallback(self, msg):
        self.costmap = OccupancyGrid2d(msg)
    
    def explored_percentage(self):
        map_width, map_height = self.costmap.getSize()

        # Initialize variables to count explored and total cells
        num_explored_cells = 0
        total_cells = map_width * map_height

        # Count unoccupied (explored) cells
        costs = []
        for mx in range(map_width):
            for my in range(map_height):
                cost = self.costmap.getCost(mx, my)
                costs.append(cost)
                if cost == 0:
                    num_explored_cells += 1
        # Calculate the explored percentage
        explored_percentage = (num_explored_cells / total_cells) * 100.0
        return explored_percentage

    def moveToFrontiers(self):
        validLocation = False
        while not validLocation:
            frontiers = getFrontier(self.currentPose, self.costmap, self.info_msg)
            self.frontier_all_publish(frontiers)
            #Remove already tried unnavigable frontiers
            frontiers = [f for f in frontiers if f not in self.invalidLocations]
            self.frontier_valid_publish(frontiers)
            #Select navigation location
            location = self.frontier_goal_selector(frontiers, 0.5, 2) #Set smallDistThresh and largeDistThresh
            if location == None:
                self.info_msg('No more navigable Frontiers')
                self.play_finish_audio()
                return
            #Check if it has been set more than 3 times
            validLocation = self.location_repetition_check(3, location)
            new_location = self.is_navigable(location, 0.35, 0.5) #Set robot_radius and max_search_dist
            if not validLocation or new_location is None:
                validLocation = False
                self.invalidLocations.append(location)
            else:
                #Save selected location
                self.count_selected_location(location)
                location = new_location #Navigable alternative
        
        explored_percentage = self.explored_percentage()
        self.info_msg(f"Frontier count: {len(frontiers)}, current explored percentage: {explored_percentage}")

        #Send current goal to clicked point
        goal_for_rviz = PointStamped(point=Point(x=location[0], y=location[1], z=0.0))
        goal_for_rviz.header.frame_id = 'map'
        self.goal_pubisher.publish(goal_for_rviz)

        #Assemble Nav goal action request
        action_request = self.createActionMsg(location)
        
        self.info_msg(f'Sending goal request {location}')
        send_goal_future = self.action_client.send_goal_async(action_request)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            #continue
            self.moveToFrontiers()

        self.info_msg('Goal accepted')

        get_result_future = self.goal_handle.get_result_async()

        self.info_msg("NavigateToPose action in progress...")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        self.moveToFrontiers()

    def is_navigable(self, goal, robot_radius, max_search_radius):
        #If goal has obstacles near, than tries to recommend close alternative with no obstacles around in max_search_radius
        goal_x = goal[0]
        goal_y = goal[1]
        
        if not self.is_obstacle_in_radius(goal, robot_radius):
            return goal_x, goal_y
        
        min_x = goal_x - max_search_radius
        max_x = goal_x + max_search_radius
        min_y = goal_y - max_search_radius
        max_y = goal_y + max_search_radius

        closest_point = None
        closest_distance = (max_search_radius + 1) ** 2

        for x in np.linspace(min_x, max_x, 20):
            for y in np.linspace(min_y, max_y, 20):
                if (x-goal_x)**2 + (y-goal_y**2) <= max_search_radius**2:
                    if self.is_valid((x, y)):
                        if self.costmap.getCost(self.costmap.worldToMap(x, y)[0], self.costmap.worldToMap(x, y)[1]) == 0:
                            if not self.is_obstacle_in_radius((x, y), robot_radius):
                                distance_to_goal = (x - goal_x)**2 + (y - goal_y)**2
                                if distance_to_goal < closest_distance:
                                    closest_point = (x, y)
                                    closest_distance = distance_to_goal       
        return closest_point 
        
    
    def is_valid(self, goal):
        #Checks if goal is on costmap
        mx = int((goal[0] - self.costmap.map.info.origin.position.x) / self.costmap.map.info.resolution)
        my = int((goal[1] - self.costmap.map.info.origin.position.y) / self.costmap.map.info.resolution)
        if goal[0] < self.costmap.map.info.origin.position.x or goal[1] < self.costmap.map.info.origin.position.y:
            return False

        return 0 <= mx < self.costmap.getSizeX() and 0 <= my < self.costmap.getSizeY()

    def is_obstacle_in_radius(self, goal, robot_radius):
        #Check if goal has any obstacles in robot_radius
        goal_x, goal_y = goal

        #Bounding box around the circular region to check
        min_x = goal_x - robot_radius
        max_x = goal_x + robot_radius
        min_y = goal_y - robot_radius
        max_y = goal_y + robot_radius

        for x in np.linspace(min_x, max_x, 20):
            for y in np.linspace(min_y, max_y, 20):
                #Check if the cell is within the circular region
                if (x - goal_x) ** 2 + (y - goal_y) ** 2 <= robot_radius ** 2:
                    if self.is_valid((x, y)):
                        #Check if the cell is an obstacle
                        if self.costmap.getCost(self.costmap.worldToMap(x, y)[0], self.costmap.worldToMap(x, y)[1]) == 100:
                            return True  #Obstacle present
        #No obstacle
        return False

    def frontier_goal_selector(self, frontiers, smallDistThresh, largeDistThresh):
        location = None

        if len(frontiers) == 0:
            return location

        dists = []
        largeDists = []
        smallDists = []
        #Sort frontiers by distance from current position of the robot
        #1. smallDistThresh < x < largeDistThrash
        #2. smallDistThresh > x
        #3. largeDistThrash < x
        for f in frontiers:
            dist = math.sqrt(((f[0] - self.currentPose.position.x)**2) + ((f[1] - self.currentPose.position.y)**2))
            dists.append(dist)
            if  dist >= smallDistThresh and dist <= largeDistThresh:
                location = f
            elif dist < smallDistThresh:
                smallDists.append(dist)
            elif dist > largeDistThresh:
                largeDists.append(dist)
        
        if location == None:
            if len(smallDists) != 0:
                location = frontiers[dists.index(max(smallDists))]
            elif len(largeDists) != 0:
                location = frontiers[dists.index(min(largeDists))]

        return location

    def count_selected_location(self, newLocation):
        #Count and add locations
        if newLocation in self.selectedLocations:
            self.selectedLocations[newLocation] += 1
        else:
            self.selectedLocations[newLocation] = 1

    def location_repetition_check(self, trynumber, newLocation):
        #Check if location has been tried to be explored more than 3 times
        if newLocation in self.selectedLocations and self.selectedLocations[newLocation] >= trynumber:
            return False
        else:
            return True

    def setInitialPose(self, pose):
        self.info_msg("Setting initial pose")
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.header.frame_id = 'map'
        self.currentPose = self.init_pose.pose.pose
        self.publishInitialPose()
        time.sleep(5)

    def poseCallback(self, msg):
        #self.info_msg('Received amcl_pose')
        self.currentPose = msg.pose.pose
        #self.currentOrientation = msg.pose.orientation
        self.initial_pose_received = True

    def createActionMsg(self, goal):
        #Creates the action_request in the right form, with orientation (vector from current pos to goal)
        action_request = NavigateToPose.Goal()
        action_request.pose = PoseStamped()
        action_request.pose.header.frame_id = 'map'
        print(goal[0], goal[1])
        action_request.pose.pose.position.x = goal[0]
        action_request.pose.pose.position.y = goal[1]
        
        dx = action_request.pose.pose.position.x - self.currentPose.position.x
        dy = action_request.pose.pose.position.y - self.currentPose.position.y
        orientation_angle = math.atan2(dy, dx)

        action_request.pose.pose.orientation = Quaternion()
        action_request.pose.pose.orientation.x = 0.0
        action_request.pose.pose.orientation.y = 0.0
        action_request.pose.pose.orientation.z = math.sin(orientation_angle / 2.0)
        action_request.pose.pose.orientation.w = math.cos(orientation_angle / 2.0)

        return action_request

    def publishInitialPose(self):
        self.initial_pose_pub.publish(self.init_pose)

    def frontier_valid_publish(self, frontiers):
        #Visualize accessible frontiers
        for i in range(len(frontiers)):
            marker = Marker()
            marker.id = i+len(frontiers)
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ''
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontiers[i][0]
            marker.pose.position.y = frontiers[i][1]
            marker.pose.position.z = 0.0
            marker.color.a = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0 
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            self.frontier_publisher.publish(marker)

    def frontier_all_publish(self, frontiers):
        #Visualize all frontiers
        delete_markers = Marker()
        delete_markers.header.frame_id = 'map'
        delete_markers.header.stamp = self.get_clock().now().to_msg()
        delete_markers.action = 3 #delete all
        self.frontier_publisher.publish(delete_markers)
        for i in range(len(frontiers)):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ''
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontiers[i][0]
            marker.pose.position.y = frontiers[i][1]
            marker.pose.position.z = 0.0
            marker.color.a = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0 
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.26
            marker.scale.y = 0.26
            marker.scale.z = 0.26
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.frontier_publisher.publish(marker)

    def play_finish_audio(self):
        audio_msg = AudioNoteVector()
        audio_msg.header.frame_id = 'map'
        audio = [AudioNote(frequency=392, max_runtime=Duration(sec=0, nanosec=177500000)), AudioNote(frequency=523, max_runtime=Duration(sec=0, nanosec=355000000)), AudioNote(frequency=587, max_runtime=Duration(sec=0, nanosec=177500000)), AudioNote(frequency=784, max_runtime=Duration(sec=0, nanosec=533000000))]
        audio_msg.notes = audio
        audio_msg.append = False

        self.audio_publisher.publish(audio_msg)
        self.info_msg("Victory song")

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)


def main(argv=sys.argv[1:]):
    rclpy.init()

    starting_pose = [0.0, 0.0]

    explorer = WaypointExplorer()

    if explorer.new_init_pose != None:
        explorer.setInitialPose(starting_pose)
        #explorer.setInitialPose(explorer.new_init_pose)
        explorer.info_msg('Setting initial pose')
        while not explorer.initial_pose_received:
            explorer.info_msg('Waiting for amcl_pose to be received')
            rclpy.spin_once(explorer, timeout_sec=1.0)  #Wait for poseCallback
    
    while explorer.costmap == None:
        explorer.info_msg('Getting initial map')
        rclpy.spin_once(explorer, timeout_sec=1.0)

    explorer.info_msg('WaypointExplorer ready!')
    explorer.moveToFrontiers()

    rclpy.spin(explorer)
                
if __name__ == '__main__':
    main()
