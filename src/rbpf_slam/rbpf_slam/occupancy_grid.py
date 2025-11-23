import numpy as np
from skimage.draw import line # Fast line drawing algorithm

class OccupancyGrid:
    def __init__(self, width, height, resolution):

        """
        width, height: Physical dimensions in meters (e.g., 30m x 30m)
        resolution: Meters per pixel (e.g., 0.05m = 5cm)
        """
        self.resolution = resolution
        self.width = width
        self.height = height
        
        # The Grid Dimensions (in pixels)
        self.cols = int(width / resolution)
        self.rows = int(height / resolution)

        # Center the start point (0,0) in the middle of the grid
        self.origin_x = self.cols // 2
        self.origin_y = self.rows // 2

        # The Map Data (Log-Odds)
        # Initialize with 0.0 (Unknown)
        self.grid = np.zeros((self.rows, self.cols), dtype=np.float32)

        # Log-Odds Parameters
        self.LO_OCCUPIED = 0.85   # Add this when we see a wall
        self.LO_FREE = -0.4       # Subtract this when we see free space
        self.LO_MAX = 5.0         # Clamp max (saturated black)
        self.LO_MIN = -5.0        # Clamp min (saturated white)

    def world_to_grid(self, x, y):
        """Convert World (meters) to Grid (pixels)"""
        # 1. Scale by resolution
        # 2. Shift by origin (so 0,0 is in the center, not bottom-left)
        gx = int(x / self.resolution) + self.origin_x
        gy = int(y / self.resolution) + self.origin_y
        
        # Check bounds
        if 0 <= gx < self.cols and 0 <= gy < self.rows:
            return gx, gy
        return None, None
    
    def update(self, robot_x, robot_y, hits_x, hits_y):
        
        """
        Raycasting update using Bresenham's Line Algorithm
        robot_x, robot_y: Particle Position
        hits_x, hits_y: Arrays of Global Coordinates of Lidar Hits
        """
        # Convert Robot Position to Grid
        r_gx, r_gy = self.world_to_grid(robot_x, robot_y)
        if r_gx is None: return # Robot went off map
        
        # Loop through every lidar beam
        for hx, hy in zip(hits_x, hits_y):
            # Convert Beam Endpoint to Grid
            h_gx, h_gy = self.world_to_grid(hx, hy)
            if h_gx is None: continue
            
            # 1. RAYCASTING (Free Space)
            # skimage.draw.line gives us all pixels BETWEEN robot and wall
            # These pixels are definitely empty space.
            rr, cc = line(r_gy, r_gx, h_gy, h_gx)
            
            # Update Free Space (exclude the last point, which is the wall)
            # We subtract log-odds for free space
            self.grid[rr[:-1], cc[:-1]] -= 0.4 # Slightly decrease
            
            # 2. HIT UPDATE (Occupied)
            # The very last pixel (h_gx, h_gy) is the wall.
            self.grid[h_gy, h_gx] += 0.85
            
        # Clamp values to keep them stable
        np.clip(self.grid, self.LO_MIN, self.LO_MAX, out=self.grid)