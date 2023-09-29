from hbird_msgs.msg import Waypoint
from planner_utils import Environment
from copy import copy
from typing import List, Tuple
from geometry_msgs.msg import Vector3
import numpy as np

DIST_THRESH_TO_GOAL = 0.5
STEP_SIZE = 1.0


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
            "x": [x for x in env.obstacles["x"]],
            "y": [y for y in env.obstacles["y"]],
        }

    def compute_heuristic(self, query_point: Waypoint):
        return (
            (self.goal_pose.position.x - query_point.position.x) ** 2
            + (self.goal_pose.position.y - query_point.position.y) ** 2
        ) ** 0.5

    def is_colliding(self, query_point: Waypoint):
        for idx in range(len(self.trunc_obstacles["x"])):
            dist_to_obs = ((self.trunc_obstacles["x"][idx] - query_point.position.x)**2 + (self.trunc_obstacles["y"][idx] - query_point.position.y) ** 2) ** 0.5
            if dist_to_obs < self.env.robot_radius:
                return True
        return False

    def get_query_points(self) -> List[Waypoint]:
        n_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x,
                y=self.current_pose.position.y + STEP_SIZE,
            )
        )
        ne_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x + STEP_SIZE / 2**.5,
                y=self.current_pose.position.y + STEP_SIZE / 2**.5,
            )
        )
        e_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x + STEP_SIZE,
                y=self.current_pose.position.y,
            )
        )
        se_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x + STEP_SIZE / 2**.5,
                y=self.current_pose.position.y - STEP_SIZE / 2**.5,
            )
        )
        s_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x,
                y=self.current_pose.position.y - STEP_SIZE,
            )
        )
        sw_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x - STEP_SIZE / 2**.5,
                y=self.current_pose.position.y - STEP_SIZE / 2**.5,
            )
        )
        w_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x - STEP_SIZE,
                y=self.current_pose.position.y,
            )
        )
        nw_query = Waypoint(
            position=Vector3(
                x=self.current_pose.position.x - STEP_SIZE / 2**.5,
                y=self.current_pose.position.y + STEP_SIZE / 2**.5,
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
                if not self.is_colliding(query) and query not in path
            ]
            heuristics = [self.compute_heuristic(pt) for pt in query_points]
            new_pt = query_points[np.argmin(heuristics)]
            path.append(new_pt)
            self.current_pose = new_pt

        return path
