
import numpy as np
from scipy.ndimage import binary_dilation

DEFAULT_OPEN_CELL_THRESHOLD = 50

class InflatedCostmap:

    def __init__(
            self, 
            occupancy_grid: np.ndarray, 
            inflation_radius: float, 
            resolution: float,
            threshold: int=DEFAULT_OPEN_CELL_THRESHOLD
    ):
        self._occupancy_grid = occupancy_grid
        self._inflation_radius = inflation_radius
        self._resolution = resolution
        self._threshold = threshold

        self._inflated_costmap = self._inflate_obstacles(
            occ_grid=occupancy_grid,
            inflation_factor=inflation_radius,
            resolution=resolution,
            threshold=threshold
        )

    #Getters:
    #--------------------------------------------------------------------------------
    def get_inflated_costmap(self)-> np.ndarray:
        return self._inflated_costmap
    
    #--------------------------------------------------------------------------------
    def get_occupancy_grid(self)-> np.ndarray:
        return self._occupancy_grid
    
    #--------------------------------------------------------------------------------
    def get_inflation_radius(self)-> float:
        return self._inflation_radius 
    
    #--------------------------------------------------------------------------------
    def get_resolution(self)-> float:
        return self._resolution
    
    #--------------------------------------------------------------------------------
    def get_threshold(self)-> int:
        return self._threshold

    #--------------------------------------------------------------------------------
    @staticmethod
    def _inflate_obstacles(
        occ_grid: np.ndarray, 
        inflation_radius_m: float, 
        resolution: float, 
        threshold: int=DEFAULT_OPEN_CELL_THRESHOLD
    )-> np.ndarray:
        """
        Inflates occupied cells in a binary occupancy grid to account for robot width.

        Args:
            occ_grid: 2D NumPy array (0=free, 100=occupied, 50=unknown)
            inflation_radius_m: inflation radius in meters
            resolution: map resolution (m per cell)
            threshold: occupancy threshold above which cells are considered occupied
        Returns:
            inflated_grid: new occupancy map (same shape)
        """
        # Convert map to boolean: True = obstacle
        obstacle_mask = occ_grid >= threshold

        # Compute inflation radius in cells
        inflation_radius_cells = int(np.ceil(inflation_radius_m / resolution))

        # Build circular structuring element (kernel)
        y, x = np.ogrid[-inflation_radius_cells:inflation_radius_cells+1,
                        -inflation_radius_cells:inflation_radius_cells+1]
        kernel = x**2 + y**2 <= inflation_radius_cells**2

        # Perform binary dilation
        inflated_mask = binary_dilation(obstacle_mask, structure=kernel)

        # Build new map: 100 where inflated, else original
        inflated_grid = occ_grid.copy()
        inflated_grid[inflated_mask] = 100

        return inflated_grid
    
