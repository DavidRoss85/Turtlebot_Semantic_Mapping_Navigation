import cv2
import numpy as np

from custom_navigator.mapping.map_cache import NavGridSnapshot
class GridPathVisualizerCV:
    """
    Quick & dirty OpenCV-based visualizer for grid + A* path debugging.
    """

    def __init__(self, grid: np.ndarray=None, threshold: int = 50, scale: int = 10):
        """
        grid: 2D numpy array (row, col)
        threshold: occupancy threshold
        scale: pixels per grid cell
        """
        if grid is not None:
             self.grid = grid
        else:
            self.grid = np.zeros((10,10),dtype=np.int16)
        self.threshold = threshold
        self.scale = scale

        cv2.namedWindow("A* Debug View", cv2.WINDOW_NORMAL)

    def update_grid(self, snapshot: NavGridSnapshot):
        
        self.grid= np.array(snapshot.data, dtype=np.int16).reshape(snapshot.height,snapshot.width)

    def _make_canvas(self):
        """
        Convert grid to BGR image.
        """
        h, w = self.grid.shape
        img = np.zeros((h, w, 3), dtype=np.uint8)

        # Free space = white, obstacles = black
        img[self.grid < self.threshold] = (255, 255, 255)
        img[self.grid >= self.threshold] = (0, 0, 0)

        # Upscale for visibility
        img = cv2.resize(
            img,
            (w * self.scale, h * self.scale),
            interpolation=cv2.INTER_NEAREST
        )

        return img

    def show(
        self,
        path=None,
        start=None,
        goal=None,
        window_name="A* Debug View",
        wait=True
    ):
        """
        path:  list of (row, col)
        start: (row, col)
        goal:  (row, col)
        """

        img = self._make_canvas()

        # Draw path
        if path and len(path)>0:
            for i in range(1, len(path)):
                p1 = self._cell_center_px(path[i - 1])
                p2 = self._cell_center_px(path[i])

                # (optional) guard against weird points
                if not (self._in_bounds_px(img, p1) and self._in_bounds_px(img, p2)):
                    continue

                cv2.line(img, p1, p2, (255, 0, 0), thickness=2)

        # Draw start / goal
        if start is not None:
            cv2.circle(img, self._cell_center_px(start), max(1, self.scale // 2), (0, 255, 0), -1)

        if goal is not None:
            cv2.circle(img, self._cell_center_px(goal), max(1, self.scale // 2), (0, 0, 255), -1)

        
        cv2.imshow(window_name, img)
        cv2.waitKey(0)

        if wait:
            cv2.destroyAllWindows()

    def _cell_center_px(self, rc):
        # rc can be (row, col), [row, col], np.array([row, col]), etc.
        r = int(rc[0])
        c = int(rc[1])

        x = int(c * self.scale + self.scale // 2)
        y = int(r * self.scale + self.scale // 2)
        return (x, y)

    def _in_bounds_px(self, img, pt):
        h, w = img.shape[:2]
        x, y = pt
        return (0 <= x < w) and (0 <= y < h)
