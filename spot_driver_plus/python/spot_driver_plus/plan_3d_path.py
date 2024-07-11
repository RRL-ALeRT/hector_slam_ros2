#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose, Twist
from std_srvs.srv import SetBool

from copy import deepcopy
import numpy as np
import math
import heapq

INCREASED_MAP_RESOLUTION = 0.05
SPEED = 0.4
LOOKAHEAD_DISTANCE = 0.4
TARGET_ERROR = 0.2
EXPANSION_SIZE = 0
Z_THRESHOLD = 0.24


def costmap(map_data: OccupancyGrid, expansion_size) -> OccupancyGrid:
    width = map_data.info.width
    height = map_data.info.height
    data = np.array(map_data.data).reshape((height, width))

    # Find walls (occupied cells)
    walls = np.where(data == 100)

    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = walls[0]+i
            y = walls[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100

    # Convert back to int8 array
    flattened_data = data.flatten().astype(np.int8)

    # Update the map_data with the expanded map
    map_data.data = flattened_data.tolist()  # Convert numpy array to list of int8
    return map_data


def pure_pursuit(current_x, current_y, current_heading, path, index, speed, lookahead_distance, forward=True):
    closest_point = None
    if forward:
        v = speed  # Set the speed to a negative value to make the robot go in reverse
    else:
        v = -speed  # Set the speed to a negative value to make the robot go in reverse
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        if forward:
            target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        else:
            target_heading = math.atan2(current_y - closest_point[1], current_x - closest_point[0])  # Reverse the atan2 arguments
        desired_steering_angle = target_heading - current_heading
    else:
        if forward:
            target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        else:
            target_heading = math.atan2(current_y - path[-1][1], current_x - path[-1][0])  # Reverse the atan2 arguments
        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi / 6 or desired_steering_angle < -math.pi / 6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = (sign * math.pi / 4)
        v = 0.0
    return v, desired_steering_angle, index


def world_to_map_coords(map_msg, world_x, world_y):
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    resolution = map_msg.info.resolution

    map_x = int((world_x - origin_x) / resolution)
    map_y = int((world_y - origin_y) / resolution)
    return map_x, map_y


def map_to_world_coords(map_msg, map_x, map_y):
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    resolution = map_msg.info.resolution

    world_x = map_x * resolution + origin_x
    world_y = map_y * resolution + origin_y
    return world_x, world_y


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def astar(array, start, goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    open_set = {start}

    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data.reverse()
            return data, True
        
        close_set.add(current)
        
        if current in open_set:
            open_set.remove(current)
        
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j

            if neighbor[1] < 0 or neighbor[1] >= array.shape[0] or neighbor[0] < 0 or neighbor[0] >= array.shape[1]:
                continue

            if array[neighbor[1]][neighbor[0]] == 100:
                continue

            if array[neighbor[1]][neighbor[0]] == -1:
                continue

            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in open_set:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
                open_set.add(neighbor)

    return [], False


def get_nearest_free_space(map_data, frontier):
    width = map_data.info.width
    height = map_data.info.height

    map_data_array = np.array(map_data.data).reshape((height, width))

    search_radius = 10

    # Convert frontier point to map coordinates
    map_x, map_y = world_to_map_coords(map_data, frontier[0], frontier[1])

    for i in range(-search_radius, search_radius + 1):
        for j in range(-search_radius, search_radius + 1):
            x = map_x + i
            y = map_y + j

            # Check if the indices are within the bounds of the map
            if 0 <= x < width and 0 <= y < height:
                if map_data_array[y, x] == 0:  # Note the swapped indices for correct access
                    # Convert back to world coordinates
                    world_x, world_y = map_to_world_coords(map_data, x, y)
                    return [world_x, world_y], True

    # If no free space is found within the radius, return the original frontier
    return frontier, False

def change_resolution(msg, old_z_values, new_resolution):
    original_resolution = msg.info.resolution
    scale_factor = original_resolution / new_resolution

    new_width = int(msg.info.width * scale_factor)
    new_height = int(msg.info.height * scale_factor)

    new_data = [0] * (new_width * new_height)
    z_values = [0] * (new_width * new_height)

    for y in range(new_height):
        for x in range(new_width):
            old_x = int(x / scale_factor)
            old_y = int(y / scale_factor)
            new_data[y * new_width + x] = msg.data[old_y * msg.info.width + old_x]
            z_values[y * new_width + x] = old_z_values[old_y * msg.info.width + old_x]

    new_msg = OccupancyGrid()
    new_msg.header = msg.header
    new_msg.info = msg.info
    new_msg.info.resolution = new_resolution
    new_msg.info.width = new_width
    new_msg.info.height = new_height
    new_msg.data = new_data

    return new_msg, z_values


class PointCloudToGrid(Node):
    def __init__(self):
        super().__init__('pointcloud_to_grid_node')

        # Parameters
        self.declare_parameter('cloud_in_topic', '/navigation/octomap_point_cloud_centers')
        self.declare_parameter('grid_topic_name', '/two_d')
        self.declare_parameter('cell_size', 0.2)
        self.declare_parameter('z_threshold', Z_THRESHOLD)
        self.declare_parameter('verbose', False)

        self.cloud_in_topic = self.get_parameter('cloud_in_topic').get_parameter_value().string_value
        self.grid_topic_name = self.get_parameter('grid_topic_name').get_parameter_value().string_value
        self.cell_size = self.get_parameter('cell_size').get_parameter_value().double_value
        self.z_threshold = self.get_parameter('z_threshold').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        # Publisher and Subscriber
        self.pub_grid = self.create_publisher(OccupancyGrid, self.grid_topic_name, 1)
        self.sub_pc2 = self.create_subscription(PointCloud2, self.cloud_in_topic, self.pointcloud2_callback, 1)

        self.listen_to_pointcloud = True
        self.create_timer(1, self.timer_cb)

        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.pose_cb, 1)
        self.path_publisher = self.create_publisher(Marker, 'path_marker', 1)
        self.create_timer(0.1, self.path_follower_timer_callback)

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        self.in_motion = False
        self.srv = self.create_service(SetBool, '/navigation/follow_path', self.set_in_motion_callback)

    def set_in_motion_callback(self, request, response):
        self.in_motion = request.data
        response.success = True
        response.message = f"in_motion set to {self.in_motion}"
        return response

    def path_follower_timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "body", rclpy.time.Time())

            # Extract the robot's position and orientation in the "map" frame
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y
            self.robot_yaw = yaw_from_quaternion(transform.transform.rotation.x,
                                                transform.transform.rotation.y,
                                                transform.transform.rotation.z,
                                                transform.transform.rotation.w)

            self.robot_position = [self.x, self.y]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            # Log a warning if the transform cannot be obtained
            self.get_logger().warn("Could not get transform from map to body: {}".format(ex))
            return

        if not self.in_motion:
            return

        if not hasattr(self, "current_path_world"):
            self.get_logger().warn("Path Not Yet Planned")
            return
        
        if len(self.current_path_world) == 0:
            return
        
        linear_velocity, angular_velocity, self.pursuit_index = pure_pursuit(
            self.x,
            self.y,
            self.robot_yaw,
            self.current_path_world,
            self.pursuit_index,
            SPEED,
            LOOKAHEAD_DISTANCE
        )

        if(abs(self.x - self.current_path_world[-1][0]) < TARGET_ERROR and abs(self.y - self.current_path_world[-1][1]) < TARGET_ERROR):
            self.in_motion = False
            print("Target reached")
            linear_velocity = 0
            angular_velocity = 0
            self.current_path_world = []

        # Publish the twist commands
        twist_command = Twist()
        twist_command.linear.x = float(linear_velocity)
        twist_command.angular.z = float(angular_velocity)
        self.twist_publisher.publish(twist_command)

    def pose_cb(self, msg):
        if self.in_motion:
            return

        if not hasattr(self, "robot_position"):
            return

        if not hasattr(self, "current_map"):
            return

        current_map, z_values = change_resolution(self.current_map, self.z_values, INCREASED_MAP_RESOLUTION)
        current_map = costmap(current_map, EXPANSION_SIZE)

        goal_coordinates = [msg.pose.pose.position.x, msg.pose.pose.position.y]

        nearest_free_cell, nearest_free_cell_found = get_nearest_free_space(current_map, goal_coordinates)
        if not nearest_free_cell_found:
            self.get_logger().warn("Could not find nearest cell")

        reshaped_map = np.array(current_map.data).reshape(current_map.info.height, current_map.info.width)
        path, is_target_reachable = astar(
            reshaped_map, world_to_map_coords(current_map, self.x, self.y),
            world_to_map_coords(current_map, nearest_free_cell[0], nearest_free_cell[1])
        )

        if not is_target_reachable:
            self.get_logger().warn("Could not reach target")
            return

        path_marker = Marker()
        path_marker.header = current_map.header
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.1  # Line width
        path_marker.color.a = 1.0  # Alpha
        path_marker.color.r = 0.0  # Red
        path_marker.color.g = 1.0  # Green
        path_marker.color.b = 0.0  # Blue

        self.current_path_world = []
        for p in path:
            pose = Pose()

            # Convert from grid coordinates (p[0], p[1]) to world coordinates
            pose.position.x, pose.position.y = map_to_world_coords(current_map, p[0], p[1])

            z_values = np.array(z_values).reshape(current_map.info.height, current_map.info.width)
            pose.position.z = z_values[int(p[1])][int(p[0])] + 0.4

            path_marker.points.append(pose.position)

            self.current_path_world.append([pose.position.x, pose.position.y])

        self.pursuit_index = 0
        self.path_publisher.publish(path_marker)

    def timer_cb(self):
        self.listen_to_pointcloud = True

    def pointcloud2_callback(self, msg):
        if not self.listen_to_pointcloud:
            return
        self.listen_to_pointcloud = False

        try:
            if self.verbose:
                self.get_logger().info(f'PointCloud2 fields: {[field.name for field in msg.fields]}')
                self.get_logger().info(f'PointCloud2 height: {msg.height}, width: {msg.width}')

            # Convert PointCloud2 to numpy array
            points = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in point_cloud2.read_points(msg, skip_nans=True)])

            # Check the shape of the points array
            if self.verbose:
                self.get_logger().info(f'Points shape: {points.shape}')
                self.get_logger().info(f'First few points: {points[:5]}')

            if points.size == 0:
                self.get_logger().info('No points received in the point cloud')
                return

            if points.ndim != 2 or points.shape[1] != 3:
                self.get_logger().error(f'Unexpected points shape: {points.shape}')
                return

            min_x, min_y = np.min(points[:, :2], axis=0)
            max_x, max_y = np.max(points[:, :2], axis=0)
            grid_size_x = int((max_x - min_x) / self.cell_size) + 1
            grid_size_y = int((max_y - min_y) / self.cell_size) + 1

            # Initialize grid with -1 (unknown) and -inf for max_z values
            occupancy_grid = np.full((grid_size_y, grid_size_x), -1, dtype=int)
            max_z_values = np.full((grid_size_y, grid_size_x), -np.inf)
            explored_cells = np.zeros((grid_size_y, grid_size_x), dtype=bool)

            # Process each point to find the topmost point in each cell
            for point in points:
                x, y, z = point
                grid_x = int((x - min_x) / self.cell_size)
                grid_y = int((y - min_y) / self.cell_size)
                if max_z_values[grid_y, grid_x] < z:
                    max_z_values[grid_y, grid_x] = z
                    explored_cells[grid_y, grid_x] = True

            # Populate the occupancy grid based on topmost points
            for (grid_y, grid_x), z in np.ndenumerate(max_z_values):
                if z == -np.inf:
                    continue
                if explored_cells[grid_y, grid_x]:
                    neighbors = [
                        (grid_y + 1, grid_x), (grid_y - 1, grid_x),
                        (grid_y, grid_x + 1), (grid_y, grid_x - 1)
                    ]
                    is_wall = False
                    for ny, nx in neighbors:
                        if 0 <= ny < grid_size_y and 0 <= nx < grid_size_x:
                            if explored_cells[ny, nx] and abs(z - max_z_values[ny, nx]) > self.z_threshold:
                                is_wall = True
                                break
                    occupancy_grid[grid_y, grid_x] = 100 if is_wall else 0
                else:
                    occupancy_grid[grid_y, grid_x] = -1  # Mark as unexplored

            self.publish_occupancy_grid(occupancy_grid, min_x, min_y, grid_size_x, grid_size_y, max_z_values)
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def publish_occupancy_grid(self, occupancy_grid, origin_x, origin_y, width, height, z_values):
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"

        grid_msg.info.resolution = self.cell_size
        grid_msg.info.width = width
        grid_msg.info.height = height
        grid_msg.info.origin.position.x = origin_x
        grid_msg.info.origin.position.y = origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = occupancy_grid.flatten().tolist()
        self.current_map = deepcopy(grid_msg)
        self.z_values = z_values.flatten().tolist()

        self.pub_grid.publish(grid_msg)
        if self.verbose:
            self.get_logger().info('Published occupancy grid')

def main(args=None):
    rclpy.init(args=args)
    pointcloud_to_grid_node = PointCloudToGrid()
    rclpy.spin(pointcloud_to_grid_node)
    pointcloud_to_grid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
