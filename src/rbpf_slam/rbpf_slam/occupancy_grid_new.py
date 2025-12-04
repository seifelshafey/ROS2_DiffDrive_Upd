import numpy as np
from skimage.draw import line
from numba import njit


@njit(fastmath=True)
def fast_raycast_update(grid, x0, y0, hits_x, hits_y, lo_free, lo_occ, lo_min, lo_max):

    height, width = grid.shape
    # Loop through every single laser beam (e.g., 360 times)
    for i in range(len(hits_x)):
        # Get the target for this specific beam
        x1 = hits_x[i]
        y1 = hits_y[i]
        
        # --- Setup Bresenham ---
        # Calculate dimensions of the triangle formed by the line
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        # Determine direction of steps
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        # Initial error: diff between horizontal and vertical progress
        err = dx - dy
        
        # Start at the robot
        cx, cy = x0, y0
        
        while True:
            # 1. CHECK IF DONE (Are we at the wall?)
            if cx == x1 and cy == y1:
                # We reached the hit point! This pixel is OCCUPIED.
                # Update logic: Add log-odds, clip to max
                current_val = grid[cy, cx]
                new_val = current_val + lo_occ
                if new_val > lo_max: new_val = lo_max # Manual Clip
                grid[cy, cx] = new_val
                break # Stop this ray, move to next beam
            
            # 2. UPDATE FREE SPACE (We are still travelling)
            # Update logic: Add log-odds (negative), clip to min
            # Bounds check is vital because Numba crashes on index errors
            if 0 <= cx < width and 0 <= cy < height:
                current_val = grid[cy, cx]
                new_val = current_val + lo_free
                if new_val < lo_min: new_val = lo_min # Manual Clip
                grid[cy, cx] = new_val
            
            # 3. CALCULATE NEXT STEP
            # We look at the error term to decide where to move next
            e2 = 2 * err
            
            # If error is sufficient, take a step in X
            if e2 > -dy:
                err -= dy
                cx += sx
            
            # If error is sufficient, take a step in Y
            if e2 < dx:
                err += dx
                cy += sy

class OccupancyGrid:
    def __init__(self, width, height, resolution):
        """
        Optimized Int8 Occupancy Grid
        width, height: Physical dimensions in meters
        resolution: Meters per pixel
        """
        self.resolution = resolution
        self.width = width
        self.height = height
        
        # Grid Dimensions
        self.cols = int(width / resolution)
        self.rows = int(height / resolution)

        # Center the origin
        self.origin_x = self.cols // 2
        self.origin_y = self.rows // 2

        # --- OPTIMIZATION: INT8 STORAGE ---
        # We scale Log-Odds by 20 to fit significant precision into int8
        # Range: -128 to 127
        # Scale: 0.85 (float) * 20 = 17 (int)
        self.SCALE = 20.0
        self.grid = np.zeros((self.rows, self.cols), dtype=np.int8)

        # Scaled Log-Odds Parameters
        # 0.85 * 20 = 17
        self.LO_OCCUPIED = int(0.85 * self.SCALE)
        # -0.4 * 20 = -8
        self.LO_FREE = int(-0.4 * self.SCALE) 
        # 5.0 * 20 = 100
        self.LO_MAX = int(5.0 * self.SCALE)
        # -5.0 * 20 = -100
        self.LO_MIN = int(-5.0 * self.SCALE)

    def world_to_grid(self, x, y):
        """Convert World (meters) to Grid (pixels)"""
        gx = int(x / self.resolution) + self.origin_x
        gy = int(y / self.resolution) + self.origin_y
        
        if 0 <= gx < self.cols and 0 <= gy < self.rows:
            return gx, gy
        return None, None
    
    def copy_grid(self):
        """
        Creates a fast deep copy of the occupancy grid.
        Returns a new OccupancyGrid object.
        """
        # Create new instance (fast, no init overhead if we manually set)
        new_obj = OccupancyGrid(self.width, self.height, self.resolution)
        
        # NumPy copy is much faster than generic deepcopy
        new_obj.grid = np.copy(self.grid)
        
        return new_obj
    
    def update_with_numba(self, pose, local_scans):
        # ... Convert pose/scans to grid coordinates ...
        robot_gx, robot_gy = self.world_to_grid(pose[0], pose[1])

        if robot_gx is None:
            return

        # --- STEP 2: Scans Local -> World ---
        # Rotate and Translate
        theta = pose[2]
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        
        # (N, 2) matrix multiplication
        world_scans = local_scans @ R.T + pose[:2]

        # --- STEP 3: Scans World -> Grid ---
        # Vectorized conversion to integer indices
        grid_hits_x = np.floor((world_scans[:, 0] / self.resolution) + self.origin_x).astype(np.int32)
        grid_hits_y = np.floor((world_scans[:, 1] / self.resolution) + self.origin_y).astype(np.int32)

        np.clip(grid_hits_x, 0, self.cols - 1, out=grid_hits_x)
        np.clip(grid_hits_y, 0, self.rows - 1, out=grid_hits_y)

        # Call the global function, passing 'self.grid' as a raw array
        fast_raycast_update(
            self.grid,          # Pass the numpy array directly
            robot_gx, robot_gy, 
            grid_hits_x, grid_hits_y,
            self.LO_FREE, self.LO_OCCUPIED, self.LO_MIN, self.LO_MAX
        )