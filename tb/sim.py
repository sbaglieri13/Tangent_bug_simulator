"""
Simulazione, visualizzazione e salvataggio (GIF + PNG).
"""

import math
import random
from typing import List, Tuple

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from matplotlib.animation import PillowWriter
from matplotlib.patches import Patch

from .geometry import Obstacle
from .robot import Robot

random.seed(47)  

def _generate_obstacles(
    start_position: Tuple[float, float],
    goal_position: Tuple[float, float],
    env_x_min: float,
    env_x_max: float,
    env_y_min: float,
    env_y_max: float,
    robot_radius: float,
    num_obstacles: int = 10,
    min_obs_size: float = 1.0,
    max_obs_size: float = 4.0,
    max_attempts: int = 200,
) -> List[Obstacle]:
    """Crea una lista di ostacoli non sovrapposti, lontani da start e goal."""
    obstacles: List[Obstacle] = []
    for _ in range(num_obstacles):
        for _ in range(max_attempts):
            w = random.uniform(min_obs_size, max_obs_size)
            h = random.uniform(min_obs_size, max_obs_size)
            x_min = random.uniform(env_x_min + robot_radius, env_x_max - w - robot_radius)
            y_min = random.uniform(env_y_min + robot_radius, env_y_max - h - robot_radius)
            x_max = x_min + w
            y_max = y_min + h
            new = Obstacle(x_min, y_min, x_max, y_max)

            start_zone = Obstacle(
                start_position[0] - robot_radius * 3,
                start_position[1] - robot_radius * 3,
                start_position[0] + robot_radius * 3,
                start_position[1] + robot_radius * 3,
            )
            goal_zone = Obstacle(
                goal_position[0] - robot_radius * 3,
                goal_position[1] - robot_radius * 3,
                goal_position[0] + robot_radius * 3,
                goal_position[1] + robot_radius * 3,
            )

            if new.intersects(start_zone, 0) or new.intersects(goal_zone, 0):
                continue
            if any(new.intersects(o, buffer=robot_radius * 2) for o in obstacles):
                continue
            obstacles.append(new)
            break
    return obstacles


def run_simulation() -> None:
    """
    Esegue la simulazione completa, mostra il plot e salva:
        - GIF dell’animazione
        - PNG del grafico (theta vs distance).
    """
    # Limiti ambiente
    env_x_min, env_x_max = 0, 20
    env_y_min, env_y_max = 0, 15

    # Parametri robot
    robot_radius = 0.15
    sensing_range = 1.0

    # Start/Goal
    start_position = (env_x_min + robot_radius * 2, env_y_min + robot_radius * 2)
    goal_position = (env_x_max - robot_radius * 2, env_y_max - robot_radius * 2)

    # File di output
    gif_path = "media/tangent_bug_sim.gif"
    png_path = "media/plot.png"
    gif_fps = 30

    # Ostacoli
    obstacles = _generate_obstacles(
        start_position, goal_position,
        env_x_min, env_x_max, env_y_min, env_y_max,
        robot_radius
    )

    # Robot
    robot = Robot(start_position, goal_position, sensing_range, robot_radius=robot_radius)

    # ---------------------- Figura ambiente ----------------------
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect("equal")
    ax.set_xlim(env_x_min, env_x_max)
    ax.set_ylim(env_y_min, env_y_max)
    ax.set_title("Tangent Bug Algorithm")
    ax.grid(False)
    ax.tick_params(bottom=False, left=False, labelbottom=False, labelleft=False)

    # Start & Goal 
    start_patch = patches.Circle(
        start_position, radius=robot_radius * 1.5, facecolor="tab:blue", edgecolor="black", linewidth=1.2, label="Start"
    )
    ax.add_patch(start_patch)
    goal_patch = patches.Circle(
        goal_position, radius=robot_radius * 1.5, facecolor="green", edgecolor="black", linewidth=1.2, label="Goal"
    )
    ax.add_patch(goal_patch)

    # Ostacoli
    for obs in obstacles:
        ax.add_patch(obs.get_patch())

    # Robot + path
    robot_patch = patches.Circle(
        robot.position, radius=robot.robot_radius, facecolor="tab:blue", edgecolor="black", label="Robot"
    )
    ax.add_patch(robot_patch)

    path_line, = ax.plot([], [], "-", linewidth=2.0, color="tab:blue", label="Robot Path")
    path_line.set_solid_joinstyle("round")
    path_line.set_solid_capstyle("round")

    # Raggi del sensore
    sensor_rays = []
    for _ in range(360):
        rl, = ax.plot([], [], color="cyan", linewidth=0.04, alpha=0.6)
        sensor_rays.append(rl)

    sensor_circle = patches.Circle(
        robot.position, radius=robot.max_range, edgecolor="lightgray", linestyle="--", facecolor="none"
    )
    ax.add_patch(sensor_circle)

    # Discontinuità
    disc_plot, = ax.plot([], [], "rx", markersize=6, label="Discontinuities")

    # Legenda
    behavior_handles = [
        Patch(facecolor="tab:blue", edgecolor="black", label="Robot color: Move-to-goal"),
        Patch(facecolor="tab:orange", edgecolor="black", label="Robot color: Boundary-following"),
    ]
    handles, labels = ax.get_legend_handles_labels()
    handles.extend(behavior_handles)
    labels.extend([h.get_label() for h in behavior_handles])
    ax.legend(handles, labels, loc="upper left")

    # Badge comportamento
    behavior_text = ax.text(
        0.98, 0.04, "", transform=ax.transAxes,
        ha="right", va="bottom",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )

    plt.ion()
    plt.show()

    best_sweep = None
    best_hits = -1

    writer = PillowWriter(fps=gif_fps)
    max_steps = 2000

    # ---------------------- Loop + registrazione GIF ----------------------
    with writer.saving(fig, gif_path, dpi=100):
        for _ in range(max_steps):
            dap, dps = robot.sense_environment(obstacles, resolution_deg=1)

            hits = sum((pt is not None) and (dist < robot.max_range) for _, dist, pt, _ in dap)
            if hits > best_hits:
                best_hits = hits
                best_sweep = dap

            for i, (adeg, dist, pt, _) in enumerate(dap[: len(sensor_rays)]):
                if pt:
                    sensor_rays[i].set_data([robot.position[0], pt[0]], [robot.position[1], pt[1]])
                else:
                    ex = robot.position[0] + robot.max_range * math.cos(math.radians(adeg))
                    ey = robot.position[1] + robot.max_range * math.sin(math.radians(adeg))
                    sensor_rays[i].set_data([robot.position[0], ex], [robot.position[1], ey])

            if dps:
                disc_plot.set_data([p[0] for p in dps], [p[1] for p in dps])
            else:
                disc_plot.set_data([], [])

            status = robot.step(obstacles)

            if len(robot.path) >= 2:
                xs, ys = map(list, zip(*robot.path))
            else:
                xs, ys = [robot.path[0][0]], [robot.path[0][1]]
            path_line.set_data(xs, ys)

            robot_patch.set_center(robot.position)
            robot_patch.set_facecolor("tab:blue" if robot.current_behavior == "move_to_goal" else "tab:orange")
            sensor_circle.set_center(robot.position)

            human = (
                "Move-to-goal"
                if robot.current_behavior == "move_to_goal"
                else "Boundary-following"
                if robot.current_behavior == "boundary_following"
                else robot.current_behavior
            )
            behavior_text.set_text(f"Behavior: {human}")

            fig.canvas.draw()
            writer.grab_frame()
            fig.canvas.flush_events()
            plt.pause(0.03)

            if status in ("goal_reached", "stuck"):
                behavior_text.set_text("Goal reached" if status == "goal_reached" else "Stuck")
                fig.canvas.draw()
                writer.grab_frame()
                break

    # ---------------------- Grafico (theta vs distance) ----------------------
    sweep = best_sweep if best_sweep is not None else robot.last_sensor_data
    if sweep:
        thetas, dists = [], []
        for angle_deg, dist, hit_point, _ in sweep:
            if (hit_point is None) or (dist >= robot.max_range - 1e-9):
                thetas.append(np.nan)
                dists.append(np.nan)
            else:
                thetas.append(angle_deg)
                dists.append(dist)

        fig2, ax2 = plt.subplots(figsize=(4.5, 3.2))
        ax2.plot(thetas, dists, ".", markersize=2)
        ax2.set_xlim(0, 360)
        ax2.set_ylim(0, max(0.01, robot.max_range * 1.05))
        ax2.set_xlabel("theta (degrees)")
        ax2.set_ylabel("distance")
        ax2.grid(False)
        ax2.set_title("")
        fig2.savefig(png_path, dpi=150, bbox_inches="tight")

    plt.ioff()
    plt.show()

    print(f"GIF salvata: {gif_path}")
    print(f"PNG grafico finale: {png_path}")


if __name__ == "__main__":
    run_simulation()
