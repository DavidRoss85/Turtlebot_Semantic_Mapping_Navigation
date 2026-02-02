
import math


#--------------------------------------------------------------------------------
def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

#--------------------------------------------------------------------------------
def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
#--------------------------------------------------------------------------------

def quaternion_to_yaw(q):
    """
    Convert a quaternion into yaw angle (in radians).
    Args:
        q: geometry_msgs.msg.Quaternion
    Returns:
        yaw angle in radians
    """
    # Convert quaternion to yaw (rotation about Z)
    try:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    except Exception as e:
        return 0
#--------------------------------------------------------------------------------
def transform_2d(tx, ty, theta, x, y):
    # Rotation
    # [x']   [cosθ  -sinθ tx]   [x]
    # [y'] = [sinθ   cosθ ty] * [y]
    # [z']   [ 0      0    1]   [1]
            
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # Apply rotation + translation
    item_x = tx + cos_t * x - sin_t * y
    item_y = ty + sin_t * x + cos_t * y

    return item_x, item_y

#--------------------------------------------------------------------------------
def euclidean(a, b):
    """Euclidean distance between two grid cells."""
    return math.hypot(a[0] - b[0], a[1] - b[1])

#----------------------------------------------------------------------------------
def fetch_origin_and_resolution(map_info)-> list:
    """
    Fetch the origin and resolution from a map info object.
    
    :param map_info: map info object
    :return: [origin_x, origin_y, resolution]
    """
    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y
    resolution = map_info.resolution
    return [origin_x,origin_y,resolution]
#----------------------------------------------------------------------------------
def convert_world_to_grid( x, y, map_info):
    """
    Convert a specific world coordinates to grid index.
    
    :param x: x coordinate in world frame
    :param y: y coordinate in world frame
    :param map_info: map info object
    :return: [i, j] grid coordinates (row, column)
    """
    origin_x, origin_y, resolution = fetch_origin_and_resolution(map_info)

    i = round((y - origin_y) / resolution)  # row
    j = round((x - origin_x) / resolution)  # col

    return [i,j]
#----------------------------------------------------------------------------------   
def convert_grid_to_world(i, j, map):
    """
    Convert grid indices to world coordinates.

    :param i: row index in grid
    :param j: column index in grid
    :param map: map object
    :return: [x, y] coordinates in world frame
    """

    origin_x, origin_y, resolution = fetch_origin_and_resolution(map)

    x = origin_x + j * resolution
    y = origin_y + i * resolution
    return [x, y]


# Updated versions of the above functions that work with NavGridSnapshot
#----------------------------------------------------------------------------------
def snap_origin_resolution(snap):
    # expects NavGridSnapshot with origin_x/origin_y/resolution
    return snap.origin_x, snap.origin_y, snap.resolution

#----------------------------------------------------------------------------------
def snap_world_to_grid(x, y, snap):
    origin_x, origin_y, resolution = snap_origin_resolution(snap)
    i = round((y - origin_y) / resolution)  # row
    j = round((x - origin_x) / resolution)  # col
    return [i, j]

#
def snap_grid_to_world(i, j, snap):
    origin_x, origin_y, resolution = snap_origin_resolution(snap)
    x = origin_x + j * resolution
    y = origin_y + i * resolution
    return [x, y]
