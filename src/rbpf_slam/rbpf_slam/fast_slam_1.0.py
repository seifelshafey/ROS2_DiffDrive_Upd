import numpy as np
import matplotlib.pyplot as plt

from rbpf_slam.occupancy_grid import OccupancyGrid

# --- NATIVE ROS 2 IMPORTS ---
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan # <--- NEW IMPORT

# --- CONFIGURATION ---
BAG_PATH = './my_slam_data' 
NUM_PARTICLES = 35
#Square Dimensions
MAP_SIZE = 50
#Resolution in m/grid cell
MAP_RES = 0.1

# --- MOTION MODEL PARAMETERS (The "Alphas") ---
# These are percentages (0.1 = 10% error)
ALPHA1 = 0.075  # Back to normal!
ALPHA2 = 0.03
ALPHA3 = 0.075
ALPHA4 = 0.03

ODOM_TOPIC = '/odometry/filtered'
# SCAN TOPIC
SCAN_TOPIC = '/scan'

class Particle:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.weight = 1.0 / NUM_PARTICLES
        self.map = OccupancyGrid(width=MAP_SIZE, height=MAP_SIZE, resolution=MAP_RES)

    def move(self, trans, rot1, rot2, direction):
        """
        Implementation of sample_motion_model_odometry (Table 5.6)
        """
        # 1. Calculate Standard Deviations (Noise)
        sd_rot1 = (ALPHA1 * abs(rot1)) + (ALPHA2 * trans) + 1e-9
        sd_trans = (ALPHA3 * trans) + (ALPHA4 * (abs(rot1) + abs(rot2))) + 1e-9
        sd_rot2 = (ALPHA1 * abs(rot2)) + (ALPHA2 * trans) + 1e-9

        # 2. Sample Noisy Parameters
        # Note: subtracting or adding normal distribution is mathematically equivalent
        # since the distribution is symmetric around 0.
        n_rot1 = rot1 - np.random.normal(0, sd_rot1)
        n_trans = trans - np.random.normal(0, sd_trans)
        n_rot2 = rot2 - np.random.normal(0, sd_rot2)

        # 3. Update Pose
        # Apply the noisy relative maneuver to the particle's current state
        # Note: We used direction earlier to correct the inputs, so we just apply
        # standard forward kinematics here.
        self.x += n_trans * direction * np.cos(self.theta + n_rot1)
        self.y += n_trans * direction * np.sin(self.theta + n_rot1)
        self.theta += n_rot1 + n_rot2

        # Normalize Angle
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

    def copy(self):
        """ Deep Copy Helper for Resampling """
        new_p = Particle()
        new_p.x = self.x
        new_p.y = self.y
        new_p.theta = self.theta
        new_p.weight = self.weight
        # CRITICAL: Copy the map array, don't just point to it
        new_p.map.grid = np.copy(self.map.grid)
        return new_p

# --- OPTIMIZATION 1: Split Scan Processing ---
def get_local_scan(msg):
    """
    Pre-calculates scan points in the robot's local frame.
    """
    # FIX: Robust angle generation. np.arange(min, max, inc) can mismatch len(ranges) due to float precision.
    ranges = np.array(msg.ranges)
    n_points = len(ranges)
    angles = msg.angle_min + np.arange(n_points) * msg.angle_increment
    
    # Downsample (stride 3)
    ranges = ranges[::3]
    angles = angles[::3]

    valid_mask = np.isfinite(ranges) & (ranges < msg.range_max)
    valid_ranges = ranges[valid_mask]
    valid_angles = angles[valid_mask]

    local_x = valid_ranges * np.cos(valid_angles)
    local_y = valid_ranges * np.sin(valid_angles)
    
    return local_x, local_y


def transform_scan(local_x, local_y, robot_x, robot_y, robot_theta):
    """
    Cheaply transform pre-calculated local points to global frame.
    This runs 35x per frame (once per particle), so it must be fast.
    """
    c = np.cos(robot_theta)
    s = np.sin(robot_theta)
    
    # 2D Rotation + Translation
    global_x = robot_x + (local_x * c - local_y * s)
    global_y = robot_y + (local_x * s + local_y * c)
    
    return global_x, global_y


def get_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)

# --- VECTORIZED WEIGHT FUNCTION (RESTORED LOGIC) ---
def calculate_weight_vectorized(particle, hits_x, hits_y):
    """
    Vectorized implementation that MATCHES the original logic exactly.
    """
    hits_x = np.asanyarray(hits_x)
    hits_y = np.asanyarray(hits_y)

    res = particle.map.resolution
    ox = particle.map.origin_x
    oy = particle.map.origin_y
    
    # FIX: Use np.trunc() to match Python's int() casting behavior.
    # np.floor() rounds -0.5 to -1.0, while int(-0.5) is 0. 
    # This ensures we hit the exact same grid cells as your original loop.
    gxs = np.trunc((hits_x/ res)+ox).astype(np.int32)
    gys = np.trunc((hits_y/ res)+oy).astype(np.int32)

    height, width = particle.map.grid.shape
    
    # Bounds Check
    valid_mask = (gxs >= 0) & (gxs < int(width/res)) & (gys >= 0) & (gys < int(height/res))
    
    valid_gxs = gxs[valid_mask]
    valid_gys = gys[valid_mask]

    # Get values
    cell_values = particle.map.grid[valid_gys, valid_gxs]

    # RESTORED: Original thresholds and scores
    conditions = [
        cell_values > 0.5,   
        cell_values < -0.2   
    ]
    choices = [
        3.0, 
        -10.0
    ]
    
    # Default -0.1 corresponds to your original 'else' clause
    scores = np.select(conditions, choices, default=-0.1)

    # RESTORED: Return np.exp() exactly as requested
    return np.exp(np.sum(scores) * 0.1)

def resample(particles):
    """ Roulette Wheel Selection """
    weights = np.array([p.weight for p in particles])
    weights /= np.sum(weights) # Normalize to 1.0
    
    # Effective Particle Count (Do we need to resample?)
    n_eff = 1.0 / np.sum(weights**2)
    if n_eff > len(particles) / 2.0:
        return particles # Distribution is healthy, skip resampling

    # Select indices based on weight
    indices = np.random.choice(len(particles), size=len(particles), p=weights)
    new_particles = [particles[i].copy() for i in indices]
    for p in new_particles: p.weight = 1.0 / NUM_PARTICLES
    return new_particles

def main():
    particles = [Particle() for _ in range(NUM_PARTICLES)]
    prev_x, prev_y, prev_th = None, None, None

    accum_dx = 0.0
    accum_dy = 0.0
    accum_dth = 0.0
    
    # We need to know the heading at the START of the accumulation segment
    # to correctly calculate the first rotation (rot1)
    start_seg_th = None

    # Setup Reader
    storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('cdr', 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Setup Topic Filter
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # Visualizationc
    plt.ion() # Interactive on
    fig, ax = plt.subplots()
    
    # 1. Create the Map Image Object ONCE (Blank start)
    # We set vmin/vmax here so colors don't flicker
    dummy_grid = np.zeros((int(MAP_SIZE / MAP_RES), int(MAP_SIZE / MAP_RES)))
    map_plot = ax.imshow(dummy_grid, origin='lower', cmap='Greys', 
                         extent=[0, MAP_SIZE, 0, MAP_SIZE], vmin=-2, vmax=5)
    
    # 2. Create the Particle Scatter Object ONCE
    # We initialize it with empty data
    particles_plot = ax.scatter([], [], c='r', s=2, alpha=0.5)
    
    plt.show(block=False)

    counter = 0
    
    print("Running SLAM...")

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()

        # --- BLOCK 1: MOTION UPDATE ---
        if topic == ODOM_TOPIC:
            msg = deserialize_message(data, Odometry)
            curr_x = msg.pose.pose.position.x
            curr_y = msg.pose.pose.position.y
            curr_th = get_yaw(msg.pose.pose.orientation)

            if prev_x is None:
                prev_x, prev_y, prev_th = curr_x, curr_y, curr_th
                start_seg_th = curr_th
                continue


           # In main(), inside ODOM_TOPIC block:
            
            # 1. Calculate Global Deltas (Same as before)

            dx = curr_x - prev_x
            dy = curr_y - prev_y
            dth = curr_th - prev_th

            dth = (dth + np.pi) % (2 * np.pi) - np.pi

            # Add to Accumulators
            accum_dx += dx
            accum_dy += dy
            accum_dth += dth

            # Update 'Previous' for the next incremental step
            prev_x, prev_y, prev_th = curr_x, curr_y, curr_th


        # --- BLOCK 2: SENSOR UPDATE ---
        elif topic == SCAN_TOPIC:
            msg = deserialize_message(data, LaserScan)

            if start_seg_th is None: 
                continue
            

            # A. RECOVER MOTION COMMANDS (Decomposition)
            # We turn the Global Accumulation vector back into Local Robot commands
            
            # 1. Distance
            delta_trans = np.sqrt(accum_dx**2 + accum_dy**2)
            
            # 2. First Rotation (Direction of travel relative to start heading)
            # atan2(dy, dx) gives Global Direction. Subtract start_th to get Local Turn.
            delta_rot1 = np.arctan2(accum_dy, accum_dx) - start_seg_th
            delta_rot1 = (delta_rot1 + np.pi) % (2 * np.pi) - np.pi
            
            # 3. Second Rotation (Whatever is left to match total rotation)
            delta_rot2 = accum_dth - delta_rot1
            delta_rot2 = (delta_rot2 + np.pi) % (2 * np.pi) - np.pi

            # B. REVERSE DETECTION
            # If the robot is reversing, rot1 will be ~180 degrees wrong.
            direction = 1.0
            if abs(delta_rot1) > (np.pi / 2):
                direction = -1.0
                # Fix rot1 for reverse logic (flip the vector)
                delta_rot1 = np.arctan2(-accum_dy, -accum_dx) - start_seg_th
                delta_rot1 = (delta_rot1 + np.pi) % (2 * np.pi) - np.pi
                delta_rot2 = accum_dth - delta_rot1
                delta_rot2 = (delta_rot2 + np.pi) % (2 * np.pi) - np.pi

            # C. MOVE PARTICLES
            # Now we have the simplified parameters to feed the model
            if delta_trans > 0.001 or abs(accum_dth) > 0.001:
                for p in particles:
                    p.move(delta_trans, delta_rot1, delta_rot2, direction)

                accum_dx = 0.0
                accum_dy = 0.0
                accum_dth = 0.0

            
            # The start of the NEXT segment is the END of this one

                start_seg_th = prev_th

                # --- OPTIMIZATION START ---
               # 1. Get Local Scan
                local_x, local_y = get_local_scan(msg)
                
                w_sum = 0.0
                
                # 2. Weight (Restored Logic)
                for p in particles:
                    hits_x, hits_y = transform_scan(local_x, local_y, p.x, p.y, p.theta)
                    
                    # Use the function that returns exp() directly
                    w = calculate_weight_vectorized(p, hits_x, hits_y)
                    p.weight = w
                    w_sum += w

                # Normalize
                if w_sum > 0:
                    for p in particles: p.weight /= w_sum

                # --- DEBUG BLOCK ---
                raw_weights = [p.weight for p in particles]
                max_w = np.max(raw_weights)
                min_w = np.min(raw_weights)
                avg_w = np.mean(raw_weights)
                
                # Calculate N_eff for debug
                w_norm = np.array(raw_weights) / np.sum(raw_weights)
                n_eff = 1.0 / np.sum(w_norm**2)

                counter += 1
                if counter % 5 == 0: 
                    best_idx = np.argmax([p.weight for p in particles])
                    best_p = particles[best_idx]
                    map_plot.set_data(best_p.map.grid)
                    px = [(p.x / MAP_RES) + best_p.map.origin_x for p in particles]
                    py = [(p.y / MAP_RES) + best_p.map.origin_y for p in particles]
                    px = np.array([val * MAP_RES for val in px])
                    py = np.array([val * MAP_RES for val in py])
                    particles_plot.set_offsets(np.c_[px, py])
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()

                particles = resample(particles)

                for p in particles:
                    hits_x, hits_y = transform_scan(local_x, local_y, p.x, p.y, p.theta)
                    p.map.update(p.x, p.y, hits_x, hits_y)

            else: continue

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()