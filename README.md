# Terrain-Weighted Path Planning Simulation

A Python simulation comparing standard A*, any-angle Theta*, and a terrain-cost-weighted variant of Theta* for path planning on a simulated disaster-like terrain grid. I built this as a hands-on preview of the cost-modeling approach in my Master's research proposal on terrain-informed exploration for tracked ground rescue robots.

## Why I built this

My research proposal argues that path planning for rescue robots should weight traversal cost by terrain difficulty (rubble, slopes) rather than distance alone, building on Theta* and recent terrain-aware planning research. Before starting graduate work, I wanted to actually implement and test this idea at a small scale, rather than only describe it on paper. This project is that test.

## What it does

1. Generates a 30x30 grid representing a disaster site, where each cell has a random terrain difficulty score (1 = flat ground, 5 = rubble/steep slope), plus a few impassable obstacle cells.
2. Finds a path from one corner to the other using three methods:
   - **A\*** — standard grid search, cost based on distance only.
   - **Theta\*** — any-angle search that uses line-of-sight shortcuts to produce shorter, smoother paths than A*.
   - **Terrain-weighted Theta\*** — the same any-angle search, but path cost also factors in the terrain difficulty crossed along each segment, so the planner prefers easier ground even if that means a longer physical path.
3. Visualizes all three paths on the terrain grid and compares path length, total cost, and computation time.

## Example result

Running with a fixed random seed produced:

| Method | Steps | Physical distance | Cost | Time |
|---|---|---|---|---|
| A* (baseline) | 31 | 41.6 | 41.6 | 0.44 ms |
| Theta* (any-angle) | 3 | 41.1 | 41.1 | 1.19 ms |
| Theta* (terrain-weighted) | 7 | 44.0 | 107.3 | 41.76 ms |

The terrain-weighted planner routes around the highest-cost terrain, accepting a longer physical path in exchange for lower traversal difficulty — the exact tradeoff my proposal is built around. Changing the random seed changes the terrain layout and the resulting numbers, confirming the paths are computed live rather than fixed.

![Comparison of A*, Theta*, and terrain-weighted Theta*](comparison.png)

## How to run it

Requirements: Python 3.9+, `numpy`, `matplotlib`

```bash
pip install numpy matplotlib
python terrain_planner.py
```

This prints the comparison results to the terminal and saves `comparison.png` in the same folder. To test on a different random terrain, change the `seed` value passed to `build_grid()` near the bottom of the script.

## Relation to my research proposal

This is a small-scale, simulation-only preview of Objective 1 and part of the methodology in my proposal ("Terrain-Informed Any-Angle Path Planning for Energy-Efficient Exploration of Tracked Ground Rescue Robots in Post-Disaster Environments"), which extends this idea further: a risk-aware cost model, frontier-based exploration target selection under battery and retrieval constraints, and evaluation in ROS2/Gazebo against standard baselines. This repository is not that full system — it's a focused test of the core idea (terrain-weighted any-angle cost) before starting that larger work.

## What I'd extend next

- Add the risk-aware, two-stage cost structure described in my proposal, rather than pure terrain-cost weighting.
- Add frontier-based exploration instead of single-goal path planning.
- Move from a static grid to ROS2/Gazebo simulation with a simulated tracked robot.
