"""
Geometria degli ostacoli
"""

import math
from typing import List, Optional, Tuple

import matplotlib.patches as patches


class Obstacle:
    """
    Ostacolo rettangolare.

    Parametri:
        x_min, y_min, x_max, y_max: estremi del rettangolo.

    Metodi utili:
        - get_patch(): patch Matplotlib per la visualizzazione.
        - contains_point(p): verifica se p è dentro il rettangolo.
        - intersects(other, buffer): test di intersezione con un altro rettangolo.
        - get_closest_point_on_boundary(q): punto del bordo più vicino a q.
        - intersect_ray(origin, angle_rad): intersezione raggio - rettangolo.
        - is_within_range(robot_pos, max_range): test distanza.
        - get_corners_in_range(robot_pos, max_range): angoli nel raggio.
    """

    def __init__(self, x_min: float, y_min: float, x_max: float, y_max: float) -> None:
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max

    def get_patch(self) -> patches.Rectangle:
        """Ritorna la patch per disegnare l’ostacolo."""
        return patches.Rectangle(
            (self.x_min, self.y_min),
            self.x_max - self.x_min,
            self.y_max - self.y_min,
            facecolor="lightgray",
            edgecolor="dimgray",
        )

    def contains_point(self, point: Tuple[float, float]) -> bool:
        """True se il punto è all’interno (inclusi i bordi)."""
        return self.x_min <= point[0] <= self.x_max and self.y_min <= point[1] <= self.y_max

    def intersects(self, other_obstacle: "Obstacle", buffer: float = 0.1) -> bool:
        """True se questo rettangolo interseca l’altro."""
        return not (
            self.x_max + buffer < other_obstacle.x_min
            or self.x_min - buffer > other_obstacle.x_max
            or self.y_max + buffer < other_obstacle.y_min
            or self.y_min - buffer > other_obstacle.y_max
        )

    def get_closest_point_on_boundary(self, query_point: Tuple[float, float]) -> Tuple[float, float]:
        """
        Punto sul bordo dell’ostacolo più vicino a 'query_point'.
        Utile per calcolare d_followed.
        """
        px, py = query_point
        closest_x = max(self.x_min, min(px, self.x_max))
        closest_y = max(self.y_min, min(py, self.y_max))

        # Se il punto è dentro, proiettalo sull’edge più vicino
        if self.x_min < px < self.x_max and self.y_min < py < self.y_max:
            dist_left = abs(px - self.x_min)
            dist_right = abs(px - self.x_max)
            dist_bottom = abs(py - self.y_min)
            dist_top = abs(py - self.y_max)
            md = min(dist_left, dist_right, dist_bottom, dist_top)
            if md == dist_left:
                return (self.x_min, py)
            if md == dist_right:
                return (self.x_max, py)
            if md == dist_bottom:
                return (px, self.y_min)
            if md == dist_top:
                return (px, self.y_max)

        return (closest_x, closest_y)

    def intersect_ray(
        self, origin: Tuple[float, float], angle_rad: float
    ) -> Tuple[Optional[float], Optional[Tuple[float, float]], Optional["Obstacle"]]:
        """
        Intersezione raggio-rettangolo.
        Ritorna: (distanza t > 0, punto_intersezione, self) oppure (None, None, None).
        """
        ox, oy = origin
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        t_values: List[float] = []
        eps = 1e-9

        if abs(dx) > eps:
            t_values += [(self.x_min - ox) / dx, (self.x_max - ox) / dx]
        if abs(dy) > eps:
            t_values += [(self.y_min - oy) / dy, (self.y_max - oy) / dy]

        valid = []
        for t in t_values:
            if t > 1e-6:
                ix = ox + t * dx
                iy = oy + t * dy
                if (
                    self.x_min - eps <= ix <= self.x_max + eps
                    and self.y_min - eps <= iy <= self.y_max + eps
                ):
                    valid.append((t, (ix, iy)))

        if not valid:
            return None, None, None

        tmin, pmin = min(valid, key=lambda x: x[0])
        return tmin, pmin, self

    def is_within_range(self, robot_pos: Tuple[float, float], max_range: float) -> bool:
        """distanza minima (bounding box) dall’ostacolo."""
        cx = max(self.x_min, min(robot_pos[0], self.x_max))
        cy = max(self.y_min, min(robot_pos[1], self.y_max))
        return math.dist(robot_pos, (cx, cy)) <= max_range

    def get_corners_in_range(
        self, robot_pos: Tuple[float, float], max_range: float
    ) -> List[Tuple[float, float]]:
        """Ritorna gli angoli dell’ostacolo entro il raggio del sensore."""
        corners = [
            (self.x_min, self.y_min),
            (self.x_max, self.y_min),
            (self.x_min, self.y_max),
            (self.x_max, self.y_max),
        ]
        return [c for c in corners if math.dist(robot_pos, c) <= max_range]
