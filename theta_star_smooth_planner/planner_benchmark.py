#!/usr/bin/env python3

import math
import csv
import rclpy
import time

import os
from datetime import datetime

from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped


class PlannerBenchmark(Node):

    def __init__(self):
        super().__init__("planner_benchmark")

        self.client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose"
        )

        # --------------------------------------------------
        # RESULTS DIRECTORY
        # --------------------------------------------------

        results_dir = os.path.expanduser(
            "~/planner_benchmark_results"
        )

        os.makedirs(results_dir, exist_ok=True)

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        self.results_file = os.path.join(
            results_dir,
            f"planner_results_{timestamp}.csv"
        )

        self.initialize_csv()

        self.get_logger().info("Planner Benchmark Node Started")

    # ----------------------------------------------------------
    # CSV INITIALIZATION
    # ----------------------------------------------------------

    def initialize_csv(self):

        with open(self.results_file, "w", newline="") as file:

            writer = csv.writer(file)

            writer.writerow([
                "ser_num",
                "planner",
                "start_x",
                "start_y",
                "goal_x",
                "goal_y",
                "planning_time_ms",
                "path_length_m",
                "sparce_waypoint_count",
                "dense_waypoint_count",
                "smoothness_deg",
                "success"
            ])

    # ----------------------------------------------------------
    # SEND GOAL
    # ----------------------------------------------------------

    def send_goal(
        self,
        ser_num,
        planner_name,
        start_x,
        start_y,
        goal_x,
        goal_y
    ):

        goal_msg = ComputePathToPose.Goal()

        goal_msg.use_start = True

        # ---------------- START ----------------

        goal_msg.start.header.frame_id = "map"

        goal_msg.start.pose.position.x = start_x
        goal_msg.start.pose.position.y = start_y
        goal_msg.start.pose.orientation.w = 1.0

        # ---------------- GOAL ----------------

        goal_msg.goal.header.frame_id = "map"

        goal_msg.goal.pose.position.x = goal_x
        goal_msg.goal.pose.position.y = goal_y
        goal_msg.goal.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Sending Goal: "
            f"({start_x}, {start_y}) -> ({goal_x}, {goal_y})"
        )

        self.client.wait_for_server()

        future = self.client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        self.process_result(
            ser_num,
            planner_name,
            start_x,
            start_y,
            goal_x,
            goal_y,
            result
        )

    # ----------------------------------------------------------
    # PROCESS RESULT
    # ----------------------------------------------------------

    def process_result(
        self,
        ser_num,
        planner_name,
        start_x,
        start_y,
        goal_x,
        goal_y,
        result
    ):

        if result.error_code != 0:

            self.get_logger().error(
                f"Planning failed: {result.error_msg}"
            )

            success = False

            planning_time_ms = 0.0
            path_length = 0.0
            sparce_waypoint_count = 0
            dense_waypoint_count = 0
            smoothness = 0.0

        else:

            success = True

            path = result.path

            # -----------------------------------
            # PLANNING TIME
            # -----------------------------------

            planning_time_ms = (
                result.planning_time.sec * 1000.0 +
                result.planning_time.nanosec / 1e6
            )

            # -----------------------------------
            # PATH LENGTH
            # -----------------------------------

            path_length = self.compute_path_length(path)

            # -----------------------------------
            # DENSE WAYPOINT COUNT
            # -----------------------------------

            dense_waypoint_count = len(path.poses)

            # -----------------------------------
            # EXTRACT PATH COORDINATES
            # -----------------------------------

            path_x = []
            path_y = []

            for pose in path.poses:

                path_x.append(pose.pose.position.x)
                path_y.append(pose.pose.position.y)

            # -----------------------------------
            # SPARSE WAYPOINT COUNT
            # -----------------------------------

            sparce_waypoint_count = self.compute_sparse_waypoint_count(
                path_x,
                path_y
            )

            # -----------------------------------
            # SMOOTHNESS
            # -----------------------------------

            smoothness = self.compute_path_smoothness(
                path_x,
                path_y
            )

            # -----------------------------------
            # PRINT BENCHMARK RESULTS
            # -----------------------------------

            self.get_logger().info(
                f"-"*20
            )

            self.get_logger().info(
                f"Ser Num: {ser_num}"
            )

            self.get_logger().info(
                f"Planning Time: {planning_time_ms:.3f} ms"
            )

            self.get_logger().info(
                f"Path Length: {path_length:.3f} m"
            )

            self.get_logger().info(
                f"Sparse Waypoint Count: {sparce_waypoint_count}"
            )

            self.get_logger().info(
                f"Dense Waypoint Count: {dense_waypoint_count}"
            )

            self.get_logger().info(
                f"Normalized Smoothness: {smoothness:.3f}"
            )

            self.get_logger().info(
                f"-"*20
            )

        # ---------------------------------------
        # SAVE TO CSV
        # ---------------------------------------

        with open(self.results_file, "a", newline="") as file:

            writer = csv.writer(file)

            writer.writerow([
                ser_num,
                planner_name,
                start_x,
                start_y,
                goal_x,
                goal_y,
                planning_time_ms,
                path_length,
                sparce_waypoint_count,
                dense_waypoint_count,
                smoothness,
                success
            ])

    # ----------------------------------------------------------
    # PATH LENGTH
    # ----------------------------------------------------------

    def compute_path_length(self, path):

        total = 0.0

        for i in range(1, len(path.poses)):

            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position

            dx = p2.x - p1.x
            dy = p2.y - p1.y

            total += math.hypot(dx, dy)

        return total

    # ----------------------------------------------------------
    # TRIANGLE ANGLE
    # ----------------------------------------------------------

    def tri_angle(self, p0, p1, p2):

        a = math.hypot(
            p0[0] - p1[0],
            p0[1] - p1[1]
        )

        b = math.hypot(
            p1[0] - p2[0],
            p1[1] - p2[1]
        )

        c = math.hypot(
            p0[0] - p2[0],
            p0[1] - p2[1]
        )

        if a == 0 or b == 0:
            return 180.0

        cos_angle = (
            (a**2 + b**2 - c**2) /
            (2 * a * b)
        )

        cos_angle = max(min(cos_angle, 1.0), -1.0)

        angle = math.degrees(math.acos(cos_angle))

        return angle

    # ----------------------------------------------------------
    # GET PATH ANGLES
    # ----------------------------------------------------------

    def get_path_angles(self, path_x, path_y):

        path_angles = []

        if len(path_x) < 3:
            return [180.0]

        for i in range(len(path_x) - 2):

            p0 = [path_x[i], path_y[i]]
            p1 = [path_x[i + 1], path_y[i + 1]]
            p2 = [path_x[i + 2], path_y[i + 2]]

            angle = self.tri_angle(p0, p1, p2)

            path_angles.append(angle)

        return path_angles

    # ----------------------------------------------------------
    # NORMALIZED SMOOTHNESS
    # ----------------------------------------------------------

    def compute_path_smoothness(self, path_x, path_y):

        angles = self.get_path_angles(path_x, path_y)

        if len(angles) == 0:
            return 0.0

        smoothness = 0.0

        for angle in angles:
            smoothness += (180.0 - angle)

        normalized_smoothness = (
            smoothness / len(angles)
        )

        return normalized_smoothness

    # ----------------------------------------------------------
    # SPARSE WAYPOINT COUNT
    # ----------------------------------------------------------

    def compute_sparse_waypoint_count(
        self,
        path_x,
        path_y,
        angle_threshold=175.0
    ):

        angles = self.get_path_angles(path_x, path_y)
        # print(angles)

        turn_count = 0

        for angle in angles:

            if angle < angle_threshold:
                turn_count += 1

        return turn_count


# --------------------------------------------------------------
# MAIN
# --------------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = PlannerBenchmark()

    # ----------------------------------------------------------
    # TEST CASES
    # ----------------------------------------------------------

    tests = [
        #(start_x, start_y, goal_x, goal_y)
        (0.0, -4.0, 3.0, -4.0),
        (-2.0, -2.0, 2.0, 3.0),
        (-2.0, 4.0, -2.0, -4.0),
        (0.0, -4.0, -2.5, -4.0),
        (-2.5, -4.0, 2.0, 4.0),
        (-2.5, 4.0, 2.0, 4.0)
    ]

    # planner_name = "NavThetaStar"
    planner_name = "CThetaStar"

    for test in tests:
        for count in range(20):

            node.send_goal(
                count,
                planner_name,
                test[0],
                test[1],
                test[2],
                test[3]
            )

            time.sleep(0.5)

    node.get_logger().info("Benchmark Complete")

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()