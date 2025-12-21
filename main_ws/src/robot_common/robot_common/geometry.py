
import math

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