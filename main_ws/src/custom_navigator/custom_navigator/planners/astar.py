
# Tuning parameters to address later.
# For now it allows for a bit of tuning.
# They represent the values of open cells and the weights for the A* algorithm.
DEFAULT_OPEN_CELL_THRESHOLD = 50
DEFAULT_OPTIMISTIC_WEIGHT = 5
DEFAULT_LOOSE_THRESHOLD = 55

import heapq
import numpy as np

from robot_common.geometry import euclidean as heuristic


class AStarPlanner:

    def __init__(
            self,
            grid: np.ndarray = None,
            threshold: int = DEFAULT_OPEN_CELL_THRESHOLD,
            weight: float = 1.0,
            optimistic: bool = True,
            optimistic_weight: float = DEFAULT_OPTIMISTIC_WEIGHT,
            loose_threshold: int = DEFAULT_LOOSE_THRESHOLD,
            connectivity: int = 8
    ):
        self._grid = grid
        self._threshold = threshold
        self._weight = weight
        self._optimistic = optimistic
        self._optimistic_weight = optimistic_weight
        self._loose_threshold = loose_threshold
        self._connectivity = connectivity
        self._diag_step_cost = np.sqrt(2)  # Cost for diagonal movement
        self._straight_step_cost = 1.0      # Cost for straight movement

    #Getters:
    #----------------------------------------------------------------------------
    def get_grid(self) -> np.ndarray:
        return self._grid
    #--------------------------------------------------------------------------------
    def get_threshold(self) -> int:
        return self._threshold
    #--------------------------------------------------------------------------------
    def get_weight(self) -> float:
        return self._weight
    #--------------------------------------------------------------------------------
    def get_optimistic(self) -> bool:
        return self._optimistic 
    #--------------------------------------------------------------------------------
    def get_optimistic_weight(self) -> float:
        return self._optimistic_weight
    #--------------------------------------------------------------------------------    
    def get_loose_threshold(self) -> int:
        return self._loose_threshold

    #Setters:
    #--------------------------------------------------------------------------------
    def set_grid(self, grid: np.ndarray):
        self._grid = grid

    #--------------------------------------------------------------------------------
    def set_params(
        self,
        *,
        threshold: int | None = None,
        weight: float | None = None,
        optimistic: bool | None = None,
        optimistic_weight: float | None = None,
        loose_threshold: int | None = None,
        connectivity: int | None = None,
        diag_step_cost: float | None = None,
        straight_step_cost: float | None = None
    ):
        """
        Set planning parameters.
        """
        if threshold is not None:
            self._threshold = threshold
        if weight is not None:
            self._weight = weight
        if optimistic is not None:
            self._optimistic = optimistic
        if optimistic_weight is not None:
            self._optimistic_weight = optimistic_weight
        if loose_threshold is not None:
            self._loose_threshold = loose_threshold
        if connectivity is not None:
            self._connectivity = connectivity
        if diag_step_cost is not None:
            self._diag_step_cost = diag_step_cost
        if straight_step_cost is not None:
            self._straight_step_cost = straight_step_cost

    #--------------------------------------------------------------------------------
    def plan(self, start: tuple, goal: tuple) -> list:
        """
        Generate a path from start to goal using A* algorithm.
        
        :param start: Start position as (row, column)
        :type start: tuple
        :param goal: Goal position as (row, column) tuple
        :type goal: tuple
        :return: Path as list of (row, col). Empty list if start invalid or no path (unless optimistic returns best-effort).
        :rtype: list
        """
        return self._find_path_using_a_star(
            start=start,
            goal=goal,
            grid=self._grid,
            threshold=self._threshold,
            weight=self._weight,
            optimistic=self._optimistic,
            optimistic_weight=self._optimistic_weight,
            loose_threshold=self._loose_threshold
        )
    #--------------------------------------------------------------------------------
    
    def _open_neighbors(self,node, grid,threshold=50):
        """Generate valid neighboring cells (8-connected)."""
        
        # Define neighbor offsets based on connectivity
        if self._connectivity == 4:
            neigboring_set =  [(-1,0), (1,0), (0,-1), (0,1)]
        else:  # 8-connected
            neigboring_set =  [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        
        i, j = node
        map_height, map_width = grid.shape

        for di, dj in neigboring_set:
            ni, nj = i + di, j + dj
            
            # skip out of bounds
            if not (0 <= ni < map_height and 0 <= nj < map_width):
                continue

            if grid[ni, nj] < threshold:  # threshold for free space
                yield (ni, nj)
    #--------------------------------------------------------------------------------
    def _find_path_using_a_star(
            self,start, goal, grid=None, threshold=DEFAULT_OPEN_CELL_THRESHOLD, 
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
        # --- A* setup ---
        si, sj = start
        gi, gj = goal
        start = (si, sj)

        if grid is None:
            return []
        # bounds check
        # height, width = np.shape(grid)
        # if not (0 <= si < height and 0 <= sj < width):
        #     return []
        # if not (0 <= gi < height and 0 <= gj < width):
        #     return []

        # # start must be traversable
        # if grid[si, sj] >= plan_threshold:
        #     return []
        # if not optimistic and grid[gi, gj] >= plan_threshold:
        #     return []

        # --- A* optimistic setup ---
        # Experimental: if goal is not free, try to path to the closest block reached:
        plan_weight = optimistic_weight if optimistic else weight
        plan_threshold = loose_threshold if optimistic else threshold
        if optimistic:
            # weight = optimistic_weight
            # threshold = loose_threshold
            height, width = grid.shape
            gi = min(gi, height - 1)   # clip row (i)
            gj = min(gj, width - 1)    # clip column (j)

        goal = (gi, gj)  # Ensure goal is immutable

        open_set = []
        h = heuristic(start, goal) * plan_weight
        heapq.heappush(open_set, (h, 0, start))
        came_from = {}
        g_score = {start: 0}
        closed_set = set()
        current = start
        # --- A* search loop ---
        while open_set:
            _, current_cost, current = heapq.heappop(open_set)
            if current_cost != g_score.get(current, float("inf")):
                continue
            if current in closed_set:
                continue

            # Mark node as closed (finalized)
            closed_set.add(current)
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for neighbor in self._open_neighbors(current, grid, plan_threshold):

                di = neighbor[0] - current[0]
                dj = neighbor[1] - current[1]
                step_cost = self._diag_step_cost if (di != 0 and dj != 0) else self._straight_step_cost
                tentative_g = g_score[current] + step_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    h = plan_weight * heuristic(neighbor, goal)
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
            return path

#--------------------------------------------------------------------------------