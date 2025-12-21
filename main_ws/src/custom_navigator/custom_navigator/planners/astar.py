
# Tuning parameters to address later.
# For now it allows for a bit of tuning.
# They represent the values of open cells and the weights for the A* algorithm.
DEFAULT_OPEN_CELL_THRESHOLD = 50
DEFAULT_OPTIMISTIC_WEIGHT = 5
DEFAULT_LOOSE_THRESHOLD = 55

import heapq

from robot_common.geometry import euclidean as heuristic

#--------------------------------------------------------------------------------
def open_neighbors(node, grid,threshold=50):
    """Generate valid neighboring cells (8-connected)."""
    neigboring_set =  [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
    i, j = node
    map_height, map_width = grid.shape
    map_min_height = 0; map_min_width = 0

    for di, dj in neigboring_set:
        ni, nj = i + di, j + dj
        ni = min(max(map_min_height,nj),map_height-1)
        nj = min(max(map_min_width,nj),map_width-1)
        # ni = map_min_height if ni < map_min_height else map_height-1 if ni >= map_height else ni
        # nj = map_min_width if nj < map_min_width else map_width-1 if nj >= map_width else nj
        # if map_min_height <= ni < map_height\
        # and map_min_width <= nj < map_width:
        if grid[ni, nj] < threshold:  # threshold for free space
            yield (ni, nj)
#--------------------------------------------------------------------------------
def find_path_using_a_star(
        start, goal, grid=None, threshold=DEFAULT_OPEN_CELL_THRESHOLD, 
        weight=1.0, optimistic=True, optimistic_weight=DEFAULT_OPTIMISTIC_WEIGHT, 
        loose_threshold=DEFAULT_LOOSE_THRESHOLD
    ) -> list:
    """
    Compute an A* path on a 2D occupancy grid.

    Args:
        start: [i, j] grid coordinates (row, column)
        goal:  [i, j] grid coordinates (row, column)
    Returns:
        path: list of [i, j] grid coordinates from start to goal
    """
    if grid is None:
        return []

    # --- A* setup ---
    si, sj = start
    gi, gj = goal
    start = (si, sj)

    # --- A* optimistic setup ---
    # Experimental: if goal is not free, try to path to the closest block reached:
    if optimistic:
        weight = optimistic_weight
        threshold = loose_threshold
        height, width = grid.shape
        gi = min(gi, height - 1)   # clip row (i)
        gj = min(gj, width - 1)    # clip column (j)

    goal = (gi, gj)  # Ensure goal is immutable

    open_set = []
    h = heuristic(start, goal) * weight
    heapq.heappush(open_set, (h, 0, start))
    came_from = {}
    g_score = {start: 0}
    current = start
    # --- A* search loop ---
    while open_set:
        _, current_cost, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return [list(p) for p in path]

        for neighbor in open_neighbors(current, grid, threshold):
            tentative_g = g_score[current] + heuristic(current, neighbor)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                h = weight * heuristic(neighbor, goal)
                f_score = tentative_g + h
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                came_from[neighbor] = current
    
    # If goal not reached, try to path to the closest block reached:
    if optimistic and g_score:
        # Find explored node closest to goal
        best_node = min(g_score.keys(), key=lambda n: heuristic(n, goal))

        # Reconstruct path from best_node back to start
        path = [best_node]
        while best_node in came_from:
            best_node = came_from[best_node]
            path.append(best_node)
        path.reverse()
        return [list(p) for p in path]
    

#--------------------------------------------------------------------------------