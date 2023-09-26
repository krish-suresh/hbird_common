from hbird_msgs.msg import Waypoint
from planner_utils import Environment
from copy import copy
from typing import List, Tuple
from geometry_msgs.msg import Vector3
import numpy as np

DIST_THRESH_TO_GOAL = 0.5
STEP_SIZE = 0.5


class PathPlanner:
    """
    This class computes a waypoint-based path from the start to the goal poses.

    You are to implement the __init__ and plan methods. Feel free to add any more methods.
    """

    def __init__(self, env: Environment):
        """
        Inputs:
        - env (dict):    dictionary storing relevant environment parameters
                         (see Environment class in planner_utils.py)
        """
        self.env = env

        self.current_pose = Waypoint()
        self.current_pose = copy(env.start_pose)
        self.goal_pose = copy(env.goal_pose)

        self.trunc_obstacles = {
            "x": [round(x, 2) for x in env.obstacles["x"]],
            "y": [round(y, 2) for y in env.obstacles["y"]],
        }

    def compute_heuristic(self, query_point: Waypoint):
        return (
            (self.goal_pose.position.x - query_point.position.x) ** 2
            + (self.goal_pose.position.y - query_point.position.y) ** 2
        ) ** 0.5

    def get_robot_corners(self, query_point: Waypoint) -> List[Tuple[int, int]]:
        lf_corner = (
            query_point.position.x - self.env.robot_radius,
            query_point.position.y + self.env.robot_radius,
        )
        rf_corner = (
            query_point.position.x + self.env.robot_radius,
            query_point.position.y + self.env.robot_radius,
        )
        lr_corner = (
            query_point.position.x - self.env.robot_radius,
            query_point.position.y - self.env.robot_radius,
        )
        rr_corner = (
            query_point.position.x + self.env.robot_radius,
            query_point.position.y - self.env.robot_radius,
        )

        return [lf_corner, rf_corner, lr_corner, rr_corner]

    def is_colliding(self, query_point: Waypoint):
        for corner in self.get_robot_corners(query_point):
            if (
                corner[0] in self.trunc_obstacles["x"]
                and corner[1] in self.trunc_obstacles["y"]
            ):
                return True
        return False

    def get_query_points(self) -> List[Waypoint]:
        n_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x,
                y=round(self.current_pose.position.y + STEP_SIZE, 2),
            )
        )
        ne_query = Waypoint(
            position=Vector3(
                x=round(self.current_pose.position.x + STEP_SIZE, 2),
                y=round(self.current_pose.position.y + STEP_SIZE, 2),
            )
        )
        e_query = Waypoint(
            position=Vector3(
                x=round(self.current_pose.position.x + STEP_SIZE, 2),
                y=self.current_pose.position.y,
            )
        )
        se_query = Waypoint(
            position=Vector3(
                x=round(self.current_pose.position.x + STEP_SIZE, 2),
                y=round(self.current_pose.position.y - STEP_SIZE, 2),
            )
        )
        s_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x,
                y=round(self.current_pose.position.y - STEP_SIZE, 2),
            )
        )
        sw_query = Waypoint(
            position=Vector3(
                x=round(self.current_pose.position.x - STEP_SIZE, 2),
                y=round(self.current_pose.position.y - STEP_SIZE, 2),
            )
        )
        w_query = Waypoint(
            position=Vector3(
                x=round(self.current_pose.position.x - STEP_SIZE, 2),
                y=self.current_pose.position.y,
            )
        )
        nw_query = Waypoint(
            position=Vector3(
                x=round(self.current_pose.position.x - STEP_SIZE, 2),
                y=round(self.current_pose.position.y + STEP_SIZE, 2),
            )
        )

        return [
            n_query,
            ne_query,
            e_query,
            se_query,
            s_query,
            sw_query,
            w_query,
            nw_query,
        ]

    def plan(self) -> List[Waypoint]:
        """
        Main method that computes and returns the path

        Returns:
        - path (list of Waypoint objects)
        """

        path = []
        path.append(self.current_pose)

        while self.compute_heuristic(self.current_pose) > DIST_THRESH_TO_GOAL:
            query_points = [
                query
                for query in self.get_query_points()
                if not self.is_colliding(query)
            ]
            heuristics = [self.compute_heuristic(pt) for pt in query_points]
            new_pt = query_points[np.argmin(heuristics)]
            path.append(new_pt)
            self.current_pose = new_pt

        breakpoint()
        return path
