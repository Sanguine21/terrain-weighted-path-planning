"""
Terrain-Weighted Path Planning Demo
------------------------------------
This script builds a small "disaster terrain" grid, then finds paths across it
using three methods:
  1. Standard A*        (baseline - ignores terrain difficulty)
  2. Standard Theta*     (any-angle baseline - ignores terrain difficulty)
  3. Terrain-weighted Theta*  (our proposed-style method - accounts for terrain)
"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import time

GRID_SIZE = 30

def build_grid(size=GRID_SIZE, seed=85):
    rng = np.random.default_rng(seed)
    terrain = rng.integers(low=1, high=6, size=(size, size)).astype(float)

    for _ in range(6):
        cx, cy = rng.integers(0, size, size=2)
        radius = rng.integers(2, 5)
        for x in range(max(0, cx - radius), min(size, cx + radius)):
            for y in range(max(0, cy - radius), min(size, cy + radius)):
                if (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2:
                    terrain[x, y] = min(5, terrain[x, y] + 2)

    obstacles = np.zeros((size, size), dtype=bool)
    num_obstacles = int(size * size * 0.05)
    obs_coords = rng.integers(0, size, size=(num_obstacles, 2))
    for x, y in obs_coords:
        obstacles[x, y] = True

    return terrain, obstacles


def neighbors_8(cell, size):
    x, y = cell
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < size and 0 <= ny < size:
                yield (nx, ny)

def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def is_blocked(cell, obstacles):
    return obstacles[cell[0], cell[1]]

def astar(start, goal, terrain, obstacles, weighted=False):
    size = terrain.shape[0]
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, g_score[goal]

        for nxt in neighbors_8(current, size):
            if is_blocked(nxt, obstacles):
                continue

            step_dist = math.hypot(nxt[0] - current[0], nxt[1] - current[1])
            terrain_cost = terrain[nxt[0], nxt[1]] if weighted else 1.0
            move_cost = step_dist * terrain_cost

            tentative_g = g_score[current] + move_cost

            if nxt not in g_score or tentative_g < g_score[nxt]:
                g_score[nxt] = tentative_g
                priority = tentative_g + heuristic(nxt, goal)
                heapq.heappush(open_set, (priority, nxt))
                came_from[nxt] = current

    return None, float("inf")


def line_of_sight(a, b, obstacles):
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0

    while (x, y) != (x1, y1):
        if obstacles[x, y]:
            return False
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return not obstacles[x1, y1]

def terrain_cost_along_line(a, b, terrain):
    steps = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1)
    total = 0.0
    for i in range(steps + 1):
        t = i / steps
        x = int(round(a[0] + (b[0] - a[0]) * t))
        y = int(round(a[1] + (b[1] - a[1]) * t))
        total += terrain[x, y]
    return total / (steps + 1)

def theta_star(start, goal, terrain, obstacles, weighted=False):
    size = terrain.shape[0]
    open_set = [(0, start)]
    came_from = {start: start}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = [current]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            path.reverse()
            return path, g_score[goal]

        parent = came_from[current]

        for nxt in neighbors_8(current, size):
            if is_blocked(nxt, obstacles):
                continue

            if line_of_sight(parent, nxt, obstacles):
                base_dist = math.hypot(nxt[0] - parent[0], nxt[1] - parent[1])
                t_cost = terrain_cost_along_line(parent, nxt, terrain) if weighted else 1.0
                tentative_g = g_score[parent] + base_dist * t_cost
                candidate_parent = parent
            else:
                base_dist = math.hypot(nxt[0] - current[0], nxt[1] - current[1])
                t_cost = terrain[nxt[0], nxt[1]] if weighted else 1.0
                tentative_g = g_score[current] + base_dist * t_cost
                candidate_parent = current

            if nxt not in g_score or tentative_g < g_score[nxt]:
                g_score[nxt] = tentative_g
                priority = tentative_g + heuristic(nxt, goal)
                heapq.heappush(open_set, (priority, nxt))
                came_from[nxt] = candidate_parent

    return None, float("inf")


if __name__ == "__main__":
    terrain, obstacles = build_grid()
    print("Grid built successfully.")
    print("Terrain cost sample (top-left 5x5 corner):")
    print(terrain[:5, :5])
    print("\nNumber of obstacle cells:", obstacles.sum())

    start = (0, 0)
    goal = (GRID_SIZE - 1, GRID_SIZE - 1)

    path_a, cost_a = astar(start, goal, terrain, obstacles, weighted=False)
    print(f"\nA* (baseline, distance-only)         : {len(path_a)} steps, cost {cost_a:.2f}")

    path_t, cost_t = theta_star(start, goal, terrain, obstacles, weighted=False)
    print(f"Theta* (any-angle, distance-only)     : {len(path_t)} steps, cost {cost_t:.2f}")

    path_tw, cost_tw = theta_star(start, goal, terrain, obstacles, weighted=True)
    print(f"Theta* (any-angle, terrain-weighted)  : {len(path_tw)} steps, cost {cost_tw:.2f}")

    def path_length(path):
        return sum(
            math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1])
            for i in range(len(path)-1)
        )

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    titles = ["A* (baseline)", "Theta* (any-angle)", "Theta* (terrain-weighted)"]
    paths = [path_a, path_t, path_tw]
    costs = [cost_a, cost_t, cost_tw]

    for ax, title, path, cost in zip(axes, titles, paths, costs):
        display_grid = terrain.copy()
        display_grid[obstacles] = np.nan
        ax.imshow(display_grid.T, origin="lower", cmap="YlOrRd", vmin=1, vmax=5)

        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, color="blue", linewidth=2, marker="o", markersize=3)

        ax.plot(start[0], start[1], "g^", markersize=12, label="Start")
        ax.plot(goal[0], goal[1], "b*", markersize=15, label="Goal")
        ax.set_title(f"{title}\nphysical dist: {path_length(path):.1f}, cost: {cost:.1f}")
        ax.legend(loc="upper left", fontsize=8)

    plt.tight_layout()
    plt.savefig("comparison.png", dpi=150)
    print("\nSaved visualization to comparison.png")

    print("\n--- Timing comparison (avg over 5 runs) ---")
    for name, fn, kwargs in [
        ("A* (baseline)", astar, dict(weighted=False)),
        ("Theta* (distance-only)", theta_star, dict(weighted=False)),
        ("Theta* (terrain-weighted)", theta_star, dict(weighted=True)),
    ]:
        t0 = time.time()
        for _ in range(5):
            fn(start, goal, terrain, obstacles, **kwargs)
        elapsed = (time.time() - t0) / 5
        print(f"{name:30s}: {elapsed*1000:.2f} ms")