"""
Robot + algoritmo Tangent Bug.
"""

import math
from typing import List, Optional, Tuple

import numpy as np

from .geometry import Obstacle


class Robot:
    """
    Robot con lidar 360° e due comportamenti (move_to_goal / boundary_following).

    Attributi principali:
        position, goal, max_range, robot_radius
        current_behavior: "move_to_goal" | "boundary_following" | "goal_reached" | "stuck"
        path: lista posizioni percorse
        d_reach, d_followed, m_point, followed_obstacle
    """

    def __init__(
        self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float], max_range: float, robot_radius: float = 0.2
    ) -> None:
        self.position = start_pos
        self.goal = goal_pos
        self.max_range = max_range
        self.robot_radius = robot_radius

        self.current_behavior = "move_to_goal"
        self.d_followed = float("inf")
        self.d_reach = float("inf")
        self.m_point: Optional[Tuple[float, float]] = None

        self.path: List[Tuple[float, float]] = [start_pos]
        self.previous_min_heuristic_dist = float("inf")

        self.followed_obstacle: Optional[Obstacle] = None
        self.follow_direction = 1
        self.dist_to_boundary_target = 0.5

        self.just_switched_to_boundary_following = False
        self.last_sensor_data: List[Tuple[float, float, Optional[Tuple[float, float]], Optional[Obstacle]]] = []

    # ----------------------------- Collisioni & Step -----------------------------
    def check_collision(self, next_pos: Tuple[float, float], obstacles: List[Obstacle]) -> bool:
        for obs in obstacles:
            if (
                obs.x_min < next_pos[0] + self.robot_radius
                and obs.x_max > next_pos[0] - self.robot_radius
                and obs.y_min < next_pos[1] + self.robot_radius
                and obs.y_max > next_pos[1] - self.robot_radius
            ):
                return True
        return False

    def move_robot_step(self, target_pos: Tuple[float, float], obstacles: List[Obstacle], step_size: float = 0.30) -> bool:
        cx, cy = self.position
        vx, vy = target_pos[0] - cx, target_pos[1] - cy
        n = math.hypot(vx, vy)

        if n < step_size:
            nx, ny = target_pos
        else:
            nx = cx + vx / n * step_size
            ny = cy + vy / n * step_size

        if self.check_collision((nx, ny), obstacles):
            return False
        if not (0 + self.robot_radius <= nx <= 20 - self.robot_radius and 0 + self.robot_radius <= ny <= 15 - self.robot_radius):
            return False

        self.position = (nx, ny)
        self.path.append(self.position)
        return True

    # -------------------------------- Percezione --------------------------------
    def sense_environment(
        self, obstacles: List[Obstacle], resolution_deg: int = 1
    ) -> Tuple[
        List[Tuple[float, float, Optional[Tuple[float, float]], Optional[Obstacle]]],
        List[Tuple[float, float]],
    ]:
        """
        Simula il lidar 360° e rileva i punti di discontinuità.
        Ritorna:
            - lista di tuple (angolo°, distanza, punto_hit, ostacolo)
            - lista dei punti di discontinuità unici (O_k)
        """
        discontinuity_points: List[Tuple[float, float]] = []
        dap: List[Tuple[float, float, Optional[Tuple[float, float]], Optional[Obstacle]]] = []

        angles = np.linspace(0, 2 * math.pi, int(360 / resolution_deg), endpoint=False)
        for ang in angles:
            best = self.max_range
            best_pt = None
            best_ref = None
            for obs in obstacles:
                d, p, r = obs.intersect_ray(self.position, ang)
                if d is not None and d < best:
                    best = d
                    best_pt = p
                    best_ref = r
            dap.append((math.degrees(ang), best, best_pt, best_ref))

        prev_ref = None
        for i, (_, _, pt, ref) in enumerate(dap):
            if ref != prev_ref:
                if pt:
                    discontinuity_points.append(pt)
                if prev_ref and i > 0 and dap[i - 1][2]:
                    discontinuity_points.append(dap[i - 1][2])
            prev_ref = ref

        for obs in obstacles:
            discontinuity_points += obs.get_corners_in_range(self.position, self.max_range)

        uniq: List[Tuple[float, float]] = []
        for dp in discontinuity_points:
            if all(math.dist(dp, u) >= 0.1 for u in uniq):
                uniq.append(dp)

        self.last_sensor_data = dap
        return dap, uniq

    # --------------------------------- Utility ---------------------------------
    def calculate_heuristic_distance(self, obstacle_point: Tuple[float, float]) -> float:
        return math.dist(self.position, obstacle_point) + math.dist(obstacle_point, self.goal)

    def is_goal_reachable(self, dap) -> bool:
        ang = math.degrees(math.atan2(self.goal[1] - self.position[1], self.goal[0] - self.position[0])) % 360
        d_goal = math.dist(self.position, self.goal)
        if d_goal < self.robot_radius * 1.5:
            return True

        min_diff = 360
        closest_dist = self.max_range
        closest_pt = None
        for a_deg, dist, pt, _ in dap:
            diff = abs(a_deg - ang)
            diff = diff if diff <= 180 else 360 - diff
            if diff < min_diff:
                min_diff = diff
                closest_dist = dist
                closest_pt = pt
        return not (closest_pt is not None and closest_dist < d_goal - self.robot_radius * 0.5)

    # ------------------------------ Behavior: move_to_goal ------------------------------
    def move_to_goal_behavior(self, obstacles: List[Obstacle]) -> str:
        dap, dps = self.sense_environment(obstacles)
        self.d_reach = float("inf")
        closest_pt_to_goal = None
        min_d = float("inf")
        blocking = None

        for _, dist, pt, ref in dap:
            if pt is not None and dist < self.max_range:
                dg = math.dist(pt, self.goal)
                if dg < min_d:
                    min_d = dg
                    closest_pt_to_goal = pt
                    blocking = ref
        if closest_pt_to_goal:
            self.d_reach = min_d

        if self.is_goal_reachable(dap):
            if not self.move_robot_step(self.goal, obstacles):
                self.current_behavior = "boundary_following"
                self.just_switched_to_boundary_following = True
                if blocking:
                    self.followed_obstacle = blocking
                    self.m_point = self.followed_obstacle.get_closest_point_on_boundary(self.goal)
                    self.d_followed = math.dist(self.m_point, self.goal)
                else:
                    closest_obs = None
                    best = float("inf")
                    for obs in obstacles:
                        cp = obs.get_closest_point_on_boundary(self.position)
                        d = math.dist(self.position, cp)
                        if d < best:
                            best = d
                            closest_obs = obs
                    if closest_obs:
                        self.followed_obstacle = closest_obs
                        self.m_point = self.followed_obstacle.get_closest_point_on_boundary(self.goal)
                        self.d_followed = math.dist(self.m_point, self.goal)
                    else:
                        return "stuck"
                return "boundary_following"

            if math.dist(self.position, self.goal) < self.robot_radius:
                return "goal_reached"
            return "move_to_goal"

        best_h = float("inf")
        best_dp = None
        best_obs = None
        valid = [dp for dp in dps if math.dist(self.position, dp) <= self.max_range]
        if not valid:
            return "stuck"

        for dp in valid:
            h = self.calculate_heuristic_distance(dp)
            if h < best_h:
                best_h = h
                best_dp = dp
                for _, _, _, ref in dap:
                    if ref and math.dist(ref.get_closest_point_on_boundary(dp), dp) < 0.1:
                        best_obs = ref
                        break

        if best_dp and best_h > self.previous_min_heuristic_dist + 1e-4 and self.previous_min_heuristic_dist != float("inf"):
            self.current_behavior = "boundary_following"
            self.just_switched_to_boundary_following = True
            self.previous_min_heuristic_dist = float("inf")
            if best_obs:
                self.followed_obstacle = best_obs
                self.m_point = self.followed_obstacle.get_closest_point_on_boundary(self.goal)
                self.d_followed = math.dist(self.m_point, self.goal)
            else:
                closest_obs = None
                best = float("inf")
                for obs in obstacles:
                    cp = obs.get_closest_point_on_boundary(self.position)
                    d = math.dist(self.position, cp)
                    if d < best:
                        best = d
                        closest_obs = obs
                if closest_obs and best < self.max_range * 1.5:
                    self.followed_obstacle = closest_obs
                    self.m_point = self.followed_obstacle.get_closest_point_on_boundary(self.goal)
                    self.d_followed = math.dist(self.m_point, self.goal)
                else:
                    return "stuck"
            return "boundary_following"
        else:
            self.previous_min_heuristic_dist = best_h

        if best_dp:
            if not self.move_robot_step(best_dp, obstacles):
                self.current_behavior = "boundary_following"
                self.just_switched_to_boundary_following = True
                if best_obs:
                    self.followed_obstacle = best_obs
                    self.m_point = self.followed_obstacle.get_closest_point_on_boundary(self.goal)
                    self.d_followed = math.dist(self.m_point, self.goal)
                else:
                    closest_in_range = None
                    best = float("inf")
                    for _, dist, _, ref in dap:
                        if ref and dist < best:
                            best = dist
                            closest_in_range = ref
                    if closest_in_range:
                        self.followed_obstacle = closest_in_range
                        self.m_point = self.followed_obstacle.get_closest_point_on_boundary(self.goal)
                        self.d_followed = math.dist(self.m_point, self.goal)
                    else:
                        return "stuck"
                return "boundary_following"
        else:
            return "stuck"

        return "move_to_goal"

    # --------------------------- Behavior: boundary_following ---------------------------
    def boundary_following_behavior(self, obstacles: List[Obstacle]) -> str:
        dap, _ = self.sense_environment(obstacles)
        self.d_reach = float("inf")
        closest_pt_to_goal = None
        min_d = float("inf")

        if self.is_goal_reachable(dap):
            self.d_reach = math.dist(self.position, self.goal)
        else:
            for _, dist, pt, _ in dap:
                if pt is not None and dist < self.max_range:
                    dg = math.dist(pt, self.goal)
                    if dg < min_d:
                        min_d = dg
                        closest_pt_to_goal = pt
            if closest_pt_to_goal:
                self.d_reach = min_d

        if self.followed_obstacle:
            self.m_point = self.followed_obstacle.get_closest_point_on_boundary(self.goal)
            self.d_followed = math.dist(self.m_point, self.goal)
        else:
            closest_obs = None
            best = float("inf")
            for obs in obstacles:
                cp = obs.get_closest_point_on_boundary(self.position)
                d = math.dist(self.position, cp)
                if d < best:
                    best = d
                    closest_obs = obs
            if closest_obs:
                self.followed_obstacle = closest_obs
                self.m_point = self.followed_obstacle.get_closest_point_on_boundary(self.goal)
                self.d_followed = math.dist(self.m_point, self.goal)
            else:
                self.current_behavior = "stuck"
                return "stuck"

            self.current_behavior = "move_to_goal"
            self.previous_min_heuristic_dist = float("inf")
            self.just_switched_to_boundary_following = False
            return "move_to_goal"

        if math.dist(self.position, self.goal) < self.robot_radius:
            self.just_switched_to_boundary_following = False
            return "goal_reached"
        elif not self.just_switched_to_boundary_following and self.d_reach < self.d_followed:
            self.current_behavior = "move_to_goal"
            self.previous_min_heuristic_dist = float("inf")
            self.just_switched_to_boundary_following = False
            return "move_to_goal"

        target = None
        best_err = float("inf")
        ideal = self.robot_radius + 0.1
        best_angle = None

        for a_deg, dist, pt, ref in dap:
            if ref == self.followed_obstacle:
                err = abs(dist - ideal)
                if err < best_err:
                    best_err = err
                    target = pt
                    best_angle = math.radians(a_deg)

        if target and best_angle is not None:
            cur_d = math.dist(self.position, target)
            error = cur_d - ideal
            k = 0.5
            desired = (best_angle + (math.pi / 2) * self.follow_direction) - k * error * self.follow_direction
            nx = self.position[0] + 0.1 * math.cos(desired)
            ny = self.position[1] + 0.1 * math.sin(desired)
            if self.move_robot_step((nx, ny), obstacles):
                self.just_switched_to_boundary_following = False
            else:
                self.follow_direction *= -1
                rnx = self.position[0] + 0.1 * math.cos(desired + math.pi)
                rny = self.position[1] + 0.1 * math.sin(desired + math.pi)
                if not self.move_robot_step((rnx, rny), obstacles):
                    return "stuck"
                self.just_switched_to_boundary_following = False
            return "boundary_following"

        self.current_behavior = "move_to_goal"
        self.followed_obstacle = None
        self.previous_min_heuristic_dist = float("inf")
        self.just_switched_to_boundary_following = False
        return "move_to_goal"

    # --------------------------------- Step ---------------------------------
    def step(self, obstacles: List[Obstacle]) -> str:
        if self.current_behavior == "move_to_goal":
            return self.move_to_goal_behavior(obstacles)
        if self.current_behavior == "boundary_following":
            return self.boundary_following_behavior(obstacles)
        if self.current_behavior == "goal_reached":
            return "goal_reached"
        if self.current_behavior == "stuck":
            return "stuck"
        return "stuck"
