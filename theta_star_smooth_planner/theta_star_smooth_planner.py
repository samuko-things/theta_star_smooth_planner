#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException
from queue import PriorityQueue
from math import sqrt, pow, hypot, acos, degrees

class GraphNode:
    def __init__(self, x, y, cost=0, heuristic=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.prev = prev
    
    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])

class ThetaStarSmoothPlanner(Node):
    def __init__(self):
        super().__init__("theta_star_smooth_planner")

        self.declare_parameter("cost_limit", 20)
        self.cost_limit = self.get_parameter("cost_limit").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/costmap", self.map_callback, map_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.path_pub = self.create_publisher(Path, "/theta_star/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/theta_star/visited_map", 10)

        self.map_ = None
        self.visited_map_ = OccupancyGrid()
        self.map_resolution_ = None

        self.get_logger().info("theta_star_planner node has just started")

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)
        self.map_resolution_ = map_msg.info.resolution

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().error("No map received!")
            return

        self.visited_map_.data = [-1] * (self.visited_map_.info.height * self.visited_map_.info.width)

        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_link", rclpy.time.Time()
            )
        except LookupException:
            self.get_logger().error("Could not transform from map to base_link")
            return

        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation

        path = self.plan(map_to_base_pose, pose.pose)
        if path.poses:
            self.get_logger().info("Shortest path found!")
            self.get_logger().info(f'path_count = {len(path.poses)}')
            full_path = self.fill_up_path(path)
            self.get_logger().info(f'full_path_count = {len(full_path.poses)}')
            self.path_pub.publish(full_path)
        else:
            self.get_logger().warn("No path found to the goal.")

    def plan(self, start: Pose, goal: Pose):

        explore_directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        open_set = PriorityQueue()
        closed_set = set()
        g_score = {}

        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        start_node.cost = 0.0
        start_node.heuristic = self.euclidean_distance(start_node, goal_node)
        start_node.prev = start_node

        g_score[(start_node.x, start_node.y)] = 0.0
        open_set.put(start_node)

        goal_reached = False

        while not open_set.empty() and rclpy.ok():
            current = open_set.get()

            if (current.x, current.y) in closed_set:
                continue

            if current == goal_node:
                goal_node = current
                goal_reached = True
                break

            closed_set.add((current.x, current.y))

            for dx, dy in explore_directions:
                neighbor = current + (dx, dy)

                if not self.pose_on_map(neighbor):
                    continue

                if not (0 <= self.map_.data[self.pose_to_cell(neighbor)] < self.cost_limit):
                    continue

                if (neighbor.x, neighbor.y) in closed_set:
                    continue

                # -------- Theta* Core Logic --------
                if current.prev and self.line_of_sight(current.prev, neighbor):
                    parent = current.prev
                    new_cost = g_score[(parent.x, parent.y)] + self.euclidean_distance(parent, neighbor)
                else:
                    parent = current
                    new_cost = g_score[(current.x, current.y)] + self.euclidean_distance(current, neighbor)

                key = (neighbor.x, neighbor.y)

                if new_cost < g_score.get(key, float('inf')):
                    g_score[key] = new_cost
                    neighbor.cost = new_cost
                    neighbor.heuristic = self.euclidean_distance(neighbor, goal_node)
                    neighbor.prev = parent
                    open_set.put(neighbor)

            # Visualization (optional)
            # self.visited_map_.data[self.pose_to_cell(current)] = -106
            # self.map_pub.publish(self.visited_map_)

        # -------- Path Reconstruction --------
        path = Path()
        path.header.frame_id = self.map_.header.frame_id

        if not goal_reached:
            return path

        node = goal_node

        while rclpy.ok():
            pose = self.grid_to_world(node)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_.header.frame_id
            pose_stamped.pose = pose
            path.poses.append(pose_stamped)

            if node == node.prev:   # reached start
                break

            node = node.prev

        path.poses.reverse()
        return path

    def euclidean_distance(self, node: GraphNode, goal_node: GraphNode):
        return sqrt( pow(node.x - goal_node.x, 2) + pow(node.y - goal_node.y, 2) )
    
    def manhattan_distance(self, node: GraphNode, goal_node: GraphNode):
        return abs(node.x - goal_node.x) + abs(node.y - goal_node.y)

    def pose_on_map(self, node: GraphNode):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x
    
    def bresenham_line(self, start: GraphNode, end: GraphNode):
        x0, y0 = start.x, start.y
        x1, y1 = end.x, end.y

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        line = []

        while True:
            line.append(GraphNode(x0, y0))

            if x0 == x1 and y0 == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return line
    
    def line_crosses_obstacle(self, line):
        width = self.map_.info.width
        height = self.map_.info.height
        data = self.map_.data
        cost_limit = self.cost_limit

        for grid_pose in line:
            x = int(grid_pose.x)
            y = int(grid_pose.y)

            # Bounds check (CRITICAL)
            if not (0 <= x < width and 0 <= y < height):
                return True   # treat outside map as obstacle

            index = y * width + x

            if not (0 <= data[index] < cost_limit):
                return True

        return False
    
    def line_of_sight(self, start: GraphNode, end: GraphNode):
        line = self.bresenham_line(start, end)
        crosses = self.line_crosses_obstacle(line)
        return not crosses   # ← THIS is the fix
    
    def add_straight_line_poses(self, start_pose: PoseStamped, end_pose: PoseStamped, resolution):
        poses = []

        dx = end_pose.pose.position.x - start_pose.pose.position.x
        dy = end_pose.pose.position.y - start_pose.pose.position.y
        distance = hypot(dx, dy)

        if distance == 0:
            return []

        no_of_iter = int(distance / resolution)

        if no_of_iter == 0:
            return []

        x_increment = dx / no_of_iter
        y_increment = dy / no_of_iter

        for i in range(no_of_iter):
            pose = PoseStamped()   # ← NEW OBJECT EACH TIME
            pose.header.frame_id = start_pose.header.frame_id
            pose.pose.position.x = start_pose.pose.position.x + x_increment * i
            pose.pose.position.y = start_pose.pose.position.y + y_increment * i
            pose.pose.orientation = start_pose.pose.orientation
            poses.append(pose)

        # Add exact end pose
        poses.append(end_pose)

        return poses
    
    def fill_up_path(self, path: Path):

        filled_path = Path()
        filled_path.header.frame_id = path.header.frame_id

        if not path.poses:
            return filled_path

        pose_list = path.poses
        filled_path.poses.append(pose_list[0])

        for i in range(1, len(pose_list)):
            poses = self.add_straight_line_poses(
                pose_list[i - 1],
                pose_list[i],
                self.map_.info.resolution
            )
            filled_path.poses.extend(poses[1:])

        return filled_path


def main(args=None):
    rclpy.init(args=args)
    node = ThetaStarSmoothPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()