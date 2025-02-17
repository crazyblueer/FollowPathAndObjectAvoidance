import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
import heapq

import numpy as np
import pandas as pd

# Define A* algorithm for pathfinding
def astar(grid_map, start, goal):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    open_set = []
    heapq.heappush(open_set, (0, start))
    g_costs = {start: 0}
    f_costs = {start: heuristic(start, goal)}
    came_from = {}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            if not (0 <= neighbor[0] < grid_map.shape[1] and 0 <= neighbor[1] < grid_map.shape[0]):
                continue
            if grid_map[neighbor[1], neighbor[0]] == 1:
                continue

            tentative_g_cost = g_costs[current] + (1.414 if direction[0] != 0 and direction[1] != 0 else 1)

            if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                came_from[neighbor] = current
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return None

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Define start and goal positions in grid coordinates
start = to_grid_indices(0.0, 0.0, x_min, y_min, resolution)
goal = to_grid_indices(1.0, 1.0, x_min, y_min, resolution)

print(to_grid_indices(0,0,x_min,y_min,resolution))

# Run A* algorithm
path = astar(grid_map, start, goal)

np.save("path7.npy", np.array(path))