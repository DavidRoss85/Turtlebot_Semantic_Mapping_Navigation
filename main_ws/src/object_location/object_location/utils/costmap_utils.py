
import numpy as np
from scipy.ndimage import binary_dilation

DEFAULT_OPEN_CELL_THRESHOLD = 50

# --- Helper functions ---
#

def inflate_obstacles(occ_grid, inflation_factor, resolution, threshold=DEFAULT_OPEN_CELL_THRESHOLD):
    """
    Inflates occupied cells in a binary occupancy grid to account for robot width.

    Args:
        occ_grid: 2D NumPy array (0=free, 100=occupied, -1/50=unknown)
        robot_width: robot diameter in meters
        resolution: map resolution (m per cell)
        threshold: occupancy threshold above which cells are considered occupied
    Returns:
        inflated_grid: new occupancy map (same shape)
    """
    # Convert map to boolean: True = obstacle
    obstacle_mask = occ_grid >= threshold

    # Compute inflation radius in cells
    inflation_radius = int(np.ceil((inflation_factor / 2.0) / resolution))

    # Build circular structuring element (kernel)
    y, x = np.ogrid[-inflation_radius:inflation_radius+1,
                    -inflation_radius:inflation_radius+1]
    kernel = x**2 + y**2 <= inflation_radius**2

    # Perform binary dilation
    inflated_mask = binary_dilation(obstacle_mask, structure=kernel)

    # Build new map: 100 where inflated, else original
    inflated_grid = occ_grid.copy()
    inflated_grid[inflated_mask] = 100

    return inflated_grid

#--------------------------------------------------------------------------------
def inflate_obstacles2(occ_grid, inflation_factor, resolution, threshold=DEFAULT_OPEN_CELL_THRESHOLD):
    """
    Inflates occupied cells by spreading their own value outward
    instead of using a generic constant.
    """

    inflated_grid = occ_grid.copy()

    # Compute radius in cells
    inflation_radius = int(np.ceil((inflation_factor / 2.0) / resolution))

    # Create circular structuring element
    y, x = np.ogrid[-inflation_radius:inflation_radius+1,
                    -inflation_radius:inflation_radius+1]
    kernel = (x**2 + y**2) <= inflation_radius**2

    # Iterate through each original obstacle cell
    obstacle_indices = np.argwhere(occ_grid >= threshold)

    for (i, j) in obstacle_indices:
        val = occ_grid[i, j]

        # Compute window bounds
        i_min = max(i - inflation_radius, 0)
        i_max = min(i + inflation_radius, occ_grid.shape[0] - 1)
        j_min = max(j - inflation_radius, 0)
        j_max = min(j + inflation_radius, occ_grid.shape[1] - 1)

        # Extract sub-kernel for border cases
        k_i_min = i_min - (i - inflation_radius)
        k_i_max = kernel.shape[0] - ((i + inflation_radius) - i_max)
        k_j_min = j_min - (j - inflation_radius)
        k_j_max = kernel.shape[1] - ((j + inflation_radius) - j_max)

        mask = kernel[k_i_min:k_i_max, k_j_min:k_j_max]

        # Inflate values: use max so overlapping areas preserve highest occupancy
        inflated_grid[i_min:i_max+1, j_min:j_max+1][mask] = np.maximum(
            inflated_grid[i_min:i_max+1, j_min:j_max+1][mask],
            val
        )

    return inflated_grid

#--------------------------------------------------------------------------------
import numpy as np

def inflate_gaussian_multiclass(
        occ_grid,
        inflation_radius_m,
        resolution,
        sigma=None,
        class_threshold=DEFAULT_OPEN_CELL_THRESHOLD
    ):
    """
    Inflates a semantic occupancy grid using Gaussian spreading.

    Whole number = class ID (e.g., 101 = person)
    Decimal part = confidence (0–1)

    Rules:
      • Only cells whose CLASS_ID >= class_threshold will be inflated.
      • Same-class overlap → probabilistic OR (a + b - ab)
      • Different-class overlap → keep class with higher confidence
      • Gaussian contributes fractional confidence only
    """

    inflated = occ_grid.copy().astype(float)

    # Convert inflation radius from meters to cells
    r = int(np.ceil(inflation_radius_m / resolution))

    # Default sigma is radius/2
    if sigma is None:
        sigma = r / 2.0

    # Precompute Gaussian kernel
    y, x = np.ogrid[-r:r+1, -r:r+1]
    d2 = x**2 + y**2
    kernel = np.exp(-(d2) / (2 * sigma * sigma))

    # Find all cells with class >= threshold
    class_cells = np.argwhere(np.floor(occ_grid) >= class_threshold)

    for (ci, cj) in class_cells:

        base_val = occ_grid[ci, cj]
        base_class = int(base_val)

        # Gaussian window boundaries
        i_min = max(ci - r, 0)
        i_max = min(ci + r, occ_grid.shape[0] - 1)
        j_min = max(cj - r, 0)
        j_max = min(cj + r, occ_grid.shape[1] - 1)

        # Subsample the kernel for edges
        k_i_min = i_min - (ci - r)
        k_i_max = k_i_min + (i_max - i_min + 1)
        k_j_min = j_min - (cj - r)
        k_j_max = k_j_min + (j_max - j_min + 1)

        subkernel = kernel[k_i_min:k_i_max, k_j_min:k_j_max]

        # Apply Gaussian spreading
        for ii in range(i_min, i_max + 1):
            for jj in range(j_min, j_max + 1):

                new_fraction = subkernel[ii - i_min, jj - j_min]

                # If Gaussian influence becomes extremely small, skip it
                if new_fraction < 1e-6:
                    continue

                existing_val = inflated[ii, jj]
                existing_class = int(existing_val)
                existing_fraction = existing_val - existing_class

                # CASE A — Same class → probabilistic OR
                if existing_class == base_class:

                    combined = existing_fraction + new_fraction - (existing_fraction * new_fraction)
                    combined = min(combined, 0.9999)

                    inflated[ii, jj] = base_class + combined

                # CASE B — Different class → keep class with higher confidence
                else:
                    if new_fraction > existing_fraction:
                        inflated[ii, jj] = base_class + min(new_fraction, 0.9999)

    return inflated

#----------------------------------------------------------------------------------
def fetch_origin_and_resolution(map_info):

    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y
    resolution = map_info.resolution
    return [origin_x,origin_y,resolution]
#----------------------------------------------------------------------------------
def convert_world_to_grid( x, y, map_info):
    
    origin_x, origin_y, resolution = fetch_origin_and_resolution(map_info)

    i = round((y - origin_y) / resolution)  # row
    j = round((x - origin_x) / resolution)  # col

    return [i,j]
#----------------------------------------------------------------------------------   
def convert_grid_to_world(i, j, map):
    origin_x, origin_y, resolution = fetch_origin_and_resolution(map)

    x = origin_x + j * resolution
    y = origin_y + i * resolution
    return [x, y]
