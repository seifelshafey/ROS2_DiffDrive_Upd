import numpy as np
import matplotlib

# --- CHANGE THIS LINE ---
# 'TkAgg' is the safest interactive backend for ROS/WSL environments
matplotlib.use('TkAgg') 
# ------------------------sudo apt-get update

import matplotlib.pyplot as plt
from numba import njit

from rbpf_slam.occupancy_grid_new import OccupancyGrid

# --- NATIVE ROS 2 IMPORTS ---
import rosbag2_py 
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

plt.ion()  # Turn on interactive mode

# --- CONFIGURATION ---
BAG_PATH = './my_slam_data' 
NUM_PARTICLES = 80
MAP_SIZE = 50
MAP_RES = 0.1

SENSOR_CONFIDENCE = 5.0 

#Optimization for Sigmoid Calc
_sigmoid_x_vals = np.linspace(-50, 50, 10000)
SIGMOID_TABLE = 1.0 / (1.0 + np.exp(-_sigmoid_x_vals))

# --- MOTION MODEL PARAMETERS for Odom
ALPHA1 = 0.075
ALPHA2 = 0.03
ALPHA3 = 0.075
ALPHA4 = 0.03

ODOM_TOPIC = '/odometry/filtered'
SCAN_TOPIC = '/scan'

def get_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)

#Numba function for sigmoid calculation on scaled Log-Odds
@njit(inline='always')
def fast_sigmoid(x):
    idx = int((x * 100.0) + 5000)
    # Clip to valid indices
    if idx < 0: return 0.0
    if idx > 9999: return 1.0
    
    return SIGMOID_TABLE[idx]

#Return scans from raw data ranges to local robot frame
def get_local_scan(msg):
    ranges = np.array(msg.ranges)
    n_points = len(ranges)
    angles = np.linspace(msg.angle_min, msg.angle_max, n_points)
    
    stride = 2
    ranges = ranges[::stride]
    angles = angles[::stride]

    valid_mask = np.isfinite(ranges) & (ranges < msg.range_max)
    valid_ranges = ranges[valid_mask]
    valid_angles = angles[valid_mask]

    local_x = valid_ranges * np.cos(valid_angles)
    local_y = valid_ranges * np.sin(valid_angles)
    
    return np.column_stack((local_x, local_y))

#Transform from meters into grid coordinates (pixels)
def transform_to_grid_coords(poses_meters):
    """
    Optimized to handle both Poses (N,3) and Points (N,2).
    """
    inv_res = 1.0 / MAP_RES
    
    # Calculate origin (e.g., 250)
    origin_c = (MAP_SIZE * inv_res) // 2
    
    # CASE 1: Full Poses [x, y, theta]
    if poses_meters.shape[1] == 3:
        scale = np.array([inv_res, inv_res, 1.0])
        offset = np.array([origin_c, origin_c, 0.0])
        
    # CASE 2: Just Points [x, y] (e.g., Laser Scans)
    else:
        scale = np.array([inv_res, inv_res])
        offset = np.array([origin_c, origin_c])
    
    poses_grid = (poses_meters * scale) + offset
    
    return poses_grid

#Sub-function of bilinear interpolation to only return map values
@njit(cache=True, fastmath=True)
def get_grid_values(grid, grid_points_pixels):

    """
    Numba-optimized Bilinear Interpolation.
    grid: 2D array (Occupancy Grid)
    grid_points_pixels: Nx2 array of (x, y) coordinates
    """
    height, width = grid.shape
    n_points = grid_points_pixels.shape[0]
    
    # Pre-allocate outputs
    values = np.zeros(n_points, dtype=np.float64)
    grads = np.zeros((n_points, 2), dtype=np.float64)
    mask = np.zeros(n_points, dtype=np.bool_) # Numba handles bool arrays well
    
    for i in range(n_points):
        x = grid_points_pixels[i, 0]
        y = grid_points_pixels[i, 1]
        
        # 1. Floor Coordinates
        x0 = int(np.floor(x))
        y0 = int(np.floor(y))
        x1 = x0 + 1
        y1 = y0 + 1
        
        # 2. Check Bounds (Early exit for invalid points)
        if x0 >= 0 and x1 < width and y0 >= 0 and y1 < height:
            mask[i] = True
            
            # 3. Efficient Lookup
            # Grid is [row, col] -> [y, x]
            raw_a = grid[y0, x0]  # Top-Left
            raw_b = grid[y1, x0]  # Bottom-Left
            raw_c = grid[y0, x1]  # Top-Right
            raw_d = grid[y1, x1]  # Bottom-Right
            
            # Convert to probabilities
            Ia = fast_sigmoid(raw_a)
            Ib = fast_sigmoid(raw_b)
            Ic = fast_sigmoid(raw_c)
            Id = fast_sigmoid(raw_d)
            
            # 4. Calculate Offsets
            dx = x - x0
            dy = y - y0
            
            # Precompute weights to save ops
            inv_dx = 1.0 - dx
            inv_dy = 1.0 - dy
            
            # 5. Interpolation (Value)
            # wa*Ia + wb*Ib + wc*Ic + wd*Id
            val = (inv_dx * inv_dy * Ia) + \
                  (inv_dx * dy * Ib) + \
                  (dx * inv_dy * Ic) + \
                  (dx * dy * Id)
            
            values[i] = val

    return values

#Bilinear Interpolation for smoothing grid over for Jacobian Calculation
@njit(cache=True, fastmath=True)
def bilinear_interpolation_numba(grid, grid_points_pixels):
    """
    Numba-optimized Bilinear Interpolation.
    grid: 2D array (Occupancy Grid)
    grid_points_pixels: Nx2 array of (x, y) coordinates
    """
    height, width = grid.shape
    n_points = grid_points_pixels.shape[0]
    
    # Pre-allocate outputs
    values = np.zeros(n_points, dtype=np.float64)
    grads = np.zeros((n_points, 2), dtype=np.float64)
    mask = np.zeros(n_points, dtype=np.bool_)
    
    for i in range(n_points):
        x = grid_points_pixels[i, 0]
        y = grid_points_pixels[i, 1]
        
        # 1. Floor Coordinates
        x0 = int(np.floor(x))
        y0 = int(np.floor(y))
        x1 = x0 + 1
        y1 = y0 + 1
        
        # 2. Check Bounds (Early exit for invalid points)
        if x0 >= 0 and x1 < width and y0 >= 0 and y1 < height:
            mask[i] = True
            
            # Grid is [row, col] -> [y, x]
            raw_a = grid[y0, x0]  # Top-Left
            raw_b = grid[y1, x0]  # Bottom-Left
            raw_c = grid[y0, x1]  # Top-Right
            raw_d = grid[y1, x1]  # Bottom-Right
            
            # Convert to probabilities (Inline)
            Ia = fast_sigmoid(raw_a)
            Ib = fast_sigmoid(raw_b)
            Ic = fast_sigmoid(raw_c)
            Id = fast_sigmoid(raw_d)
            
            # 4. Calculate Offsets
            dx = x - x0
            dy = y - y0
            
            # Precompute weights to save ops
            inv_dx = 1.0 - dx
            inv_dy = 1.0 - dy
            
            # 5. Interpolation (Value)
            # wa*Ia + wb*Ib + wc*Ic + wd*Id
            val = (inv_dx * inv_dy * Ia) + \
                  (inv_dx * dy * Ib) + \
                  (dx * inv_dy * Ic) + \
                  (dx * dy * Id)
            
            values[i] = val
            
            # 6. Gradients
            # dP/dx = (1-dy)(Ic - Ia) + dy(Id - Ib)
            grad_x = (inv_dy * (Ic - Ia)) + (dy * (Id - Ib))
            
            # dP/dy = (1-dx)(Ib - Ia) + dx(Id - Ic)
            grad_y = (inv_dx * (Ib - Ia)) + (dx * (Id - Ic))
            
            grads[i, 0] = grad_x
            grads[i, 1] = grad_y
            
    return values, grads, mask

#Greedy Hill Climbing algorithm for Wall search
def Coarse_Startup_Search(grid, map_quality, local_scans, odom_pose):

    best_start_pose = odom_pose.copy()
    
    if map_quality < 0.2: 
        # Sparse map: Look wider
        search_step_pix = [0, 1, -1, 2, -2, 3, -3]
        search_range_th = [0.0, 0.1, -0.1]
        coarse_stride = 5 # Skip more beams for speed
    else:
        # Dense map: Look closer
        search_step_pix = [0, 1, -1]
        search_range_th = [0.0, 0.05, -0.05]
        coarse_stride = 3

    # Initial scan projection for coarse search
    coarse_scans = local_scans[::coarse_stride]
    best_search_score = float('inf')

    for d_th in search_range_th:
        th_test = odom_pose[2] + d_th
        c, s = np.cos(th_test), np.sin(th_test)
        R = np.array([[c, -s], [s, c]])
        scan_world_base = coarse_scans @ R.T
        
        for dx in search_step_pix:
            for dy in search_step_pix:
                cand_x = odom_pose[0] + (dx * MAP_RES)
                cand_y = odom_pose[1] + (dy * MAP_RES)
                
                scan_world = scan_world_base + np.array([cand_x, cand_y])
                scan_pix = transform_to_grid_coords(scan_world)
                
                vals = get_grid_values(grid, scan_pix)
                
                #Mask - Keep walls (high val) and free space (low val)
                valid_mask = vals >= 0.0 
                
                if map_quality > 0.01 and np.sum(valid_mask) < len(vals) * 0.2: continue

                # Simple score: Sum of probabilities
                score = np.sum(vals[valid_mask])
                
                if score > best_search_score:
                    best_search_score = score
                    best_start_pose = np.array([cand_x, cand_y, th_test])

    return best_start_pose

#Lavenberg-Marquadt algorithm for scan matching
def LM_Optimization(grid, local_scans, curr_pose, odom_pose, odom_covariance, odom_info, sigma_hit):

    LM_lambda = 0.01 # Initial damping
    MAX_ITERS = 10
    grad_scale = 1.0 / MAP_RES

    H_final = odom_info.copy() # Initialize with Prior Information
    current_total_cost = float('inf')

    # --- STEP 2: JOINT OPTIMIZATION LOOP ---
    for i in range(MAX_ITERS):
        # A. Project Scans
        theta = curr_pose[2]
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        
        global_scans = local_scans @ R.T + curr_pose[:2]
        global_pix = transform_to_grid_coords(global_scans)
        
        # B. Get Gradients & Residuals
        # Expects m_values to be 0.0 to 1.0
        m_values, m_grads_pix, mask = bilinear_interpolation_numba(grid, global_pix)
        
        # Mask: Valid points inside map
        valid_mask = mask & (m_values >= 0.0)
        if np.sum(valid_mask) < 10: return curr_pose, odom_info, False # Lost
        
        # --- RESIDUAL CALCULATION (Weighted) ---
        # 1. SCAN: Minimize (1.0 - MapValue) / sigma_hit
        valid_res_scan = (1.0 - m_values[valid_mask]) / sigma_hit
        
        # 2. ODOM: Minimize (curr - odom) * sqrt(Info)
        diff = curr_pose - odom_pose
        diff[2] = FastSLAM.normalize_angles(diff[2])
        
        # Total Cost = Scan Cost + Odom Cost
        current_cost_scan = 0.5 * np.sum(valid_res_scan**2)
        current_cost_odom = 0.5 * (diff.T @ odom_info @ diff)
        total_cost = current_cost_scan + current_cost_odom
        
        # --- JACOBIAN CONSTRUCTION ---
        # 1. Scan Jacobian
        # Gradient of Residual = -Gradient of Map / sigma_hit
        valid_grads = m_grads_pix[valid_mask] * grad_scale
        
        gx = -valid_grads[:, 0] / sigma_hit
        gy = -valid_grads[:, 1] / sigma_hit
        
        # Chain rule for Theta
        px_rel = global_scans[valid_mask, 0] - curr_pose[0]
        py_rel = global_scans[valid_mask, 1] - curr_pose[1]
        dRes_dth = (gx * (-py_rel)) + (gy * (px_rel))
        
        J_scan = np.stack([gx, gy, dRes_dth], axis=1)
        
        # 2. Build Linear System (H and b)
        # H = J_scan.T * J_scan + Odom_Info
        H_scan = J_scan.T @ J_scan
        b_scan = J_scan.T @ valid_res_scan
        
        b_odom = odom_info @ diff
        
        # Combined System
        H_total = H_scan + odom_info
        b_total = b_scan + b_odom # Total Gradient
        
        valid_step = False

        for sub_iter in range(3):
            # --- LINE A: CONSTRUCT DAMPED SYSTEM ---
            LHS = H_total + (np.eye(3) * LM_lambda * np.trace(H_total)/3.0)
            try:
                delta = np.linalg.solve(LHS, -b_total)
            except np.linalg.LinAlgError:
                # If matrix is singular, increases damping (makes it more Identity-like) and retry
                LM_lambda *= 10.0 
                continue 

            # Check if delta is tiny (convergence)
            if np.linalg.norm(delta) < 1e-5:
                valid_step = True
                break

            # --- LINE C: GENERATE CANDIDATE ---
            # 2. Check Candidate Pose
            cand_pose = curr_pose + delta
            cand_pose[2] = FastSLAM.normalize_angles(cand_pose[2])
            
            # 3. Calculate Cost at Candidate Pose
            # --- START CANDIDATE COST CALC ---
            check_stride = 10
            check_scans = local_scans[::check_stride]
            
            theta = cand_pose[2]
            c, s = np.cos(theta), np.sin(theta)
            R_cand = np.array([[c, -s], [s, c]])
            global_cand = check_scans @ R_cand.T + cand_pose[:2]         
            pix_cand = transform_to_grid_coords(global_cand)
            vals_cand = get_grid_values(grid, pix_cand)
            
            # Mask valid
            mask_cand = vals_cand >= 0.0
            
            # Scan Cost
            err_cand = (1.0 - vals_cand[mask_cand]) / sigma_hit
            cost_scan_cand = 0.5 * np.sum(err_cand**2)
            
            # Odom Cost
            diff_cand = cand_pose - odom_pose
            diff_cand[2] = FastSLAM.normalize_angles(diff_cand[2])
            cost_odom_cand = 0.5 * (diff_cand.T @ odom_info @ diff_cand)
            
            cand_total_cost = cost_scan_cand + cost_odom_cand
            # --- END CANDIDATE COST CALC ---

            if cand_total_cost < current_total_cost:

                curr_pose = cand_pose
                current_total_cost = cand_total_cost
                LM_lambda *= 0.1 # Be more aggressive next time
                LM_lambda = max(LM_lambda, 1e-7)
                valid_step = True
                
                H_final = H_total 
                break
            else:
                # REJECT STEP
                LM_lambda *= 10.0 # Be more conservative
                LM_lambda = min(LM_lambda, 1e4) # Clamp max


        if not valid_step:
            return curr_pose, H_final, True
            
        if np.linalg.norm(delta) < 1e-4:
            return curr_pose, H_final, True
    
    return curr_pose, H_final, True

#Calculating particle weight based on distance, precision, and match accuracy
def Calculate_Weight(grid, final_pose, odom_pose, local_scans, proposal_cov, odom_info, score_stride, SCALING_FACTOR):
    final_diff = final_pose - odom_pose
    final_diff[2] = FastSLAM.normalize_angles(final_diff[2])
    # Distance penalty
    log_w_odom = -0.5 * (final_diff.T @ odom_info @ final_diff)
    
    # 2. Scan Score (Likelihood) - Using Sparse Points + SCALING
    theta = final_pose[2]
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    
    sparse_scans = local_scans[::score_stride]
    sparse_global = sparse_scans @ R.T + final_pose[:2]
    sparse_pix = transform_to_grid_coords(sparse_global)
    sparse_vals = get_grid_values(grid, sparse_pix)
    
    clipped_vals = np.clip(sparse_vals, 1e-9, 1.0)
    
    #Scaling factor to subside the effect of the laser scans vs odom
    TEMPERATURE = 0.2 
    log_w_scan = np.sum(np.log(clipped_vals)) * SCALING_FACTOR * TEMPERATURE
        
    # 3. Determinant Term (Precision)
    sign, logdet = np.linalg.slogdet(proposal_cov)
    log_w_cov = 0.5 * logdet 

    final_log_weight = log_w_scan + log_w_odom + log_w_cov

    return final_log_weight

# Gauss-Newton algorithm for scan matching
def Gauss_Newton_Match_Single(grid, local_scans, odom_pose, odom_covariance, map_quality, 
                              sigma_hit=0.05, score_stride=2):
    """
    Approach B: Joint Optimization (Scan + Odometry)
    Returns: final_pose, proposal_covariance, final_log_weight
    """
    # --- CONSTANTS ---
    SCALING_FACTOR = 1.0 #(Likelihood Gain)
    
    H_final = np.eye(3)
    proposal_cov = np.eye(3)
    
    # Early exit for very poor maps (Safeguard)
    if map_quality < 0.02: 
        return odom_pose, odom_covariance, 0.0
    
    curr_pose = odom_pose.copy()

    try:
        odom_info = np.linalg.inv(odom_covariance + np.eye(3)*1e-9)
    except np.linalg.LinAlgError:
        odom_info = np.eye(3) * 10.0
    
    if map_quality > 0.02:
        best_start_pose = Coarse_Startup_Search(grid, map_quality, local_scans, odom_pose)

    # --- STEP 1: OPTIMIZATION SETUP ---
    curr_pose = best_start_pose.copy()
    
    final_pose, H_final, success = LM_Optimization(grid, local_scans, curr_pose, odom_pose, odom_covariance, odom_info, sigma_hit)

    # --- STEP 3: FINAL COVARIANCE ---
    diff = final_pose - odom_pose
    diff[2] = FastSLAM.normalize_angles(diff[2])
    dist_sq = diff.T @ odom_info @ diff
    MAHALANOBIS_THRESHOLD = 9.0

    use_fallback = False

    if not success:
        use_fallback = True
    elif dist_sq > MAHALANOBIS_THRESHOLD:
        # Optimization pulled us too far from odometry -> Likely a repetitive environment error
        use_fallback = True
    
    if use_fallback:
        # Fallback: Revert to Odometry
        final_pose = odom_pose
        # Covariance: Just use Odom covariance (we gained no info)
        proposal_cov = odom_covariance
    else:
        # Success: Use optimized pose
        # Proposal Covariance is Inverse of Hessian
        try:
            # H_final includes both Scan Information and Odom Information
            proposal_cov = np.linalg.inv(H_final)
            # Inflation for safety (prevent over-convergence)
            proposal_cov += np.eye(3) * 1e-5
        except:
            proposal_cov = odom_covariance.copy()

    # --- STEP 4: WEIGHT CALCULATION (Approach B) ---
    # 1. Odom Score (Prior)
    final_log_weight = Calculate_Weight(grid, final_pose, odom_pose, local_scans, proposal_cov,
                                        odom_info, score_stride, SCALING_FACTOR)
    
    return curr_pose, proposal_cov, final_log_weight

# Main FastSLAM class
class FastSLAM:
    @staticmethod
    def normalize_angles(angles):
        return (angles + np.pi) % (2 * np.pi) - np.pi
    
    #Define vectorized particles, poses, weights, covariances, and maps
    def __init__(self, num_particles, map_size, map_res):
        self.N = num_particles
        self.map_res = map_res
        
        self.poses = np.zeros((self.N, 3))
        self.weights = np.ones(self.N) / self.N
        self.covs = np.zeros((self.N, 3, 3))
        self.maps = [OccupancyGrid(map_size, map_size, map_res) for _ in range(self.N)]
        
        self.map_origin_x = self.maps[0].origin_x
        self.map_origin_y = self.maps[0].origin_y

        # ADD CACHE FOR MAP QUALITY
        self.cached_map_quality = np.zeros(num_particles)
        self.last_quality_update = -5  # Force first update
    
    def predict(self, trans, rot1, rot2, direction):

        # Calculate control noise standard deviations
        sd_rot1 = (ALPHA1 * abs(rot1)) + (ALPHA2 * trans) + 1e-9
        sd_trans = (ALPHA3 * trans) + (ALPHA4 * (abs(rot1) + abs(rot2))) + 1e-9
        sd_rot2 = (ALPHA1 * abs(rot2)) + (ALPHA2 * trans) + 1e-9

        # Sample control noise
        n_rot1 = np.random.normal(0, sd_rot1, self.N)
        n_trans = np.random.normal(0, sd_trans, self.N)
        n_rot2 = np.random.normal(0, sd_rot2, self.N)

        noisy_rot1 = rot1 - n_rot1
        noisy_trans = trans - n_trans
        noisy_rot2 = rot2 - n_rot2

        # Store old poses for Jacobian calculation
        old_poses = self.poses.copy()
        
        # Update poses (vectorized)
        self.poses[:, 0] += noisy_trans * direction * np.cos(self.poses[:, 2] + noisy_rot1)
        self.poses[:, 1] += noisy_trans * direction * np.sin(self.poses[:, 2] + noisy_rot1)
        self.poses[:, 2] += noisy_rot1 + noisy_rot2
        self.poses[:, 2] = self.normalize_angles(self.poses[:, 2])

        # --- VECTORIZED COVARIANCE PROPAGATION ---
        # State transition Jacobian F (3x3) for each particle
        F = np.zeros((self.N, 3, 3))
        F[:, 0, 0] = 1.0
        F[:, 1, 1] = 1.0
        F[:, 2, 2] = 1.0
        F[:, 0, 2] = -noisy_trans * direction * np.sin(old_poses[:, 2] + noisy_rot1)
        F[:, 1, 2] = noisy_trans * direction * np.cos(old_poses[:, 2] + noisy_rot1)

        # Control Jacobian G (3x3) for each particle  
        G = np.zeros((self.N, 3, 3))
        G[:, 0, 0] = -noisy_trans * direction * np.sin(old_poses[:, 2] + noisy_rot1)
        G[:, 0, 1] = direction * np.cos(old_poses[:, 2] + noisy_rot1)
        G[:, 1, 0] = noisy_trans * direction * np.cos(old_poses[:, 2] + noisy_rot1)
        G[:, 1, 1] = direction * np.sin(old_poses[:, 2] + noisy_rot1)
        G[:, 2, 0] = 1.0
        G[:, 2, 2] = 1.0

        # Control noise covariance M (3x3) - same for all particles
        M = np.diag([sd_rot1**2, sd_trans**2, sd_rot2**2])
        M_batch = np.tile(M, (self.N, 1, 1))

        # Vectorized covariance propagation: cov_new = F @ cov_old @ F.T + G @ M @ G.T
        # Using einsum for efficient batch matrix multiplication
        F_cov_Ft = np.einsum('nij,njk->nik', np.einsum('nij,njk->nik', F, self.covs), np.transpose(F, (0, 2, 1)))
        G_M_Gt = np.einsum('nij,njk->nik', np.einsum('nij,njk->nik', G, M_batch), np.transpose(G, (0, 2, 1)))
        
        self.covs = F_cov_Ft + G_M_Gt
        
        # Add minimal regularization for numerical stability
        self.covs += np.eye(3) * 1e-6

# Resampling function
    def update_and_resample(self, local_scans):
        
        odom_poses = self.poses.copy()
        prior_covs = self.covs.copy()

        # Track scan matching performance
        map_qualities = []

       #Store optimized poses for sampling later
        opt_poses = np.zeros_like(odom_poses)
        proposed_covs = np.zeros_like(prior_covs)
        odom_covs_used = []
        log_weights = np.zeros(self.N)


        # ADD GLOBAL COUNTER - you'll need to track this in main()
        global_counter = getattr(self, 'global_counter', 0)

        if global_counter - self.last_quality_update >= 5:
            for i in range(self.N):
                occupied = np.sum(self.maps[i].grid > 20)
                free = np.sum(self.maps[i].grid < -20)
                total_known = occupied + free
                self.cached_map_quality[i] = occupied / total_known if total_known > 0 else 0.0
            self.last_quality_update = global_counter
        
        # Use cached qualities
        map_qualities = self.cached_map_quality.copy()
        
        for i in range(self.N):

            # USE CACHED MAP QUALITY - no recomputation
            map_quality = map_qualities[i]
            odom_cov = prior_covs[i] + np.eye(3) * 1e-9
            
            # --- SCAN MATCHING STEP ---
            p_opt, prop_cov, l_weight = Gauss_Newton_Match_Single(
                self.maps[i].grid,
                local_scans,
                odom_poses[i],
                odom_cov,
                map_quality,
                score_stride= 2
            )

            opt_poses[i] = p_opt
            proposed_covs[i] = prop_cov
            log_weights[i] = l_weight
            odom_covs_used.append(odom_cov)

        # --- SAMPLING FIRST ---
        # Samples the new pose from a Gaussian Distribution of proposed mean and cov
        try:
            L_batch = np.linalg.cholesky(proposed_covs) 
            z_batch = np.random.randn(self.N, 3, 1)
            perturbation = (L_batch @ z_batch).squeeze(-1)
            sampled_poses = opt_poses + perturbation
        except np.linalg.LinAlgError:
            stds = np.sqrt(np.diagonal(proposed_covs, axis1=1, axis2=2))
            sampled_poses = opt_poses + np.random.randn(self.N, 3) * stds

        sampled_poses[:, 2] = self.normalize_angles(sampled_poses[:, 2])

        perturbation_magnitudes = np.linalg.norm(sampled_poses[:, :2] - opt_poses[:, :2], axis=1)

        # 3. Check Raw Score Diversity (Before Scaling)
        # Re-calculate raw score differences to see if optimizer creates clones
        max_log_w = np.max(log_weights)
        weights_update = np.exp(log_weights - max_log_w)

        # 2. ACCUMULATE WEIGHTS (Crucial for when we don't resample!)
        # w_t = w_{t-1} * w_new
        self.weights = self.weights * weights_update
    
        sum_weights = np.sum(self.weights)

        # Better numerical stability check
        if sum_weights < 1e-200 or np.any(np.isnan(self.weights)):
            self.weights = np.ones(self.N) / self.N
        else:
            self.weights /= sum_weights

        best_idx = np.argmax(self.weights)
        curr_best_pose = sampled_poses[best_idx]
        
        if not hasattr(self, 'last_map_update_pose'):
            self.last_map_update_pose = curr_best_pose.copy()
            do_map_update = True
        else:
            dist = np.linalg.norm(curr_best_pose[:2] - self.last_map_update_pose[:2])
            angle_diff = abs(self.normalize_angles(curr_best_pose[2] - self.last_map_update_pose[2]))
            # Threshold: Update map only every 20cm or 15 degrees
            do_map_update = (dist > 0.20) or (angle_diff > 0.25)

        
        # --- CONSERVATIVE RESAMPLING DECISION ---
        n_eff = 1.0 / np.sum(self.weights**2)
        # Only resample when we have good map quality AND low diversity
        should_resample = (n_eff < self.N / 2.0)
        
        if should_resample:
            
            # 1. Generate resampling indices using systematic resampling
            keep_indices = np.zeros(self.N, dtype=int)
            r = np.random.uniform(0, 1.0 / self.N)
            c = self.weights[0]
            i = 0
            
            for m in range(self.N):
                u = r + m * (1.0 / self.N)
                while u > c and i < self.N - 1:
                    i += 1
                    c += self.weights[i]
                keep_indices[m] = i

            unique_indices = np.unique(keep_indices)

            # --- FIXED ANCESTRY MAP BUILDING ---
            # 2. Update maps ONLY for unique ancestors (optimization)
            if do_map_update:
                for idx in unique_indices:
                    self.maps[idx].update_with_numba(sampled_poses[idx], local_scans)
            
            # 3. Create new particle set with proper ancestry
            new_maps = []
            new_poses = []

            for idx in keep_indices:
                # Copy the map from the ancestor
                new_maps.append(self.maps[idx].copy_grid())
                # Use the sampled pose for this particle
                new_poses.append(sampled_poses[idx])
            
            self.poses = np.array(new_poses)
            self.maps = new_maps
            
            self.weights = np.ones(self.N) / self.N
            
        else:
            # Update all particle maps with the SAMPLED poses
            self.poses = sampled_poses
            if do_map_update:
                for i in range(self.N):
                    self.maps[i].update_with_numba(self.poses[i], local_scans)

        if do_map_update:
            self.last_map_update_pose = curr_best_pose.copy()

    def get_best_particle_index(self):
        return np.argmax(self.weights)
        
def main():
    slam = FastSLAM(NUM_PARTICLES, MAP_SIZE, MAP_RES)
    print(f"[MAIN] Initialized FastSLAM with {NUM_PARTICLES} particles")
    print(f"[MAIN] Map size: {MAP_SIZE}x{MAP_SIZE}, resolution: {MAP_RES}")
    
    slam.global_counter = 0

    prev_x, prev_y, prev_th = None, None, None
    accum_dx, accum_dy, accum_dth = 0.0, 0.0, 0.0
    start_seg_th = None

    storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('cdr', 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    fig, ax = plt.subplots()
    dummy_grid = np.zeros((int(MAP_SIZE / MAP_RES), int(MAP_SIZE / MAP_RES)))
    half_size = MAP_SIZE / 2.0
    map_plot = ax.imshow(dummy_grid, origin='lower', cmap='Greys', 
                         extent=[-half_size, half_size, -half_size, half_size], 
                         vmin=-100, vmax=100)
    
    particles_plot = ax.scatter([], [], c='r', s=2, alpha=0.5)
    plt.show(block=False)

    counter = 0
    print("Running FastSLAM 2.0...")

    # Reader logic from ROSBag
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()

        # Handle Odometry updates and accumulate motion for map update
        if topic == ODOM_TOPIC:
            msg = deserialize_message(data, Odometry)
            curr_x = msg.pose.pose.position.x
            curr_y = msg.pose.pose.position.y
            curr_th = get_yaw(msg.pose.pose.orientation)

            # Handle map start
            if prev_x is None:
                prev_x, prev_y, prev_th = curr_x, curr_y, curr_th
                start_seg_th = curr_th
                continue

            dx = curr_x - prev_x
            dy = curr_y - prev_y
            dth = curr_th - prev_th
            dth = FastSLAM.normalize_angles(dth)

            accum_dx += dx
            accum_dy += dy
            accum_dth += dth
            accum_dth = FastSLAM.normalize_angles(accum_dth)

            prev_x, prev_y, prev_th = curr_x, curr_y, curr_th

        #If a scan is received and the motion is enough, update
        # This is due to the lidar frequency being much lower than odom's
        elif topic == SCAN_TOPIC:
            msg = deserialize_message(data, LaserScan)
            local_scan = get_local_scan(msg)

            if start_seg_th is None: continue

            delta_trans = np.sqrt(accum_dx**2 + accum_dy**2)
            
            if delta_trans < 0.01:
                delta_rot1 = 0.0
            else:
                delta_rot1 = np.arctan2(accum_dy, accum_dx) - start_seg_th
                delta_rot1 = FastSLAM.normalize_angles(delta_rot1)
            
            delta_rot2 = accum_dth - delta_rot1
            delta_rot2 = FastSLAM.normalize_angles(delta_rot2)

            # Handles reversing
            direction = 1.0
            if delta_trans > 0.01 and abs(delta_rot1) > (np.pi / 2):
                direction = -1.0
                delta_rot1 = np.arctan2(-accum_dy, -accum_dx) - start_seg_th
                delta_rot1 = FastSLAM.normalize_angles(delta_rot1)
                delta_rot2 = accum_dth - delta_rot1
                delta_rot2 = FastSLAM.normalize_angles(delta_rot2)

            if delta_trans > 0.05 or abs(accum_dth) > 0.05:
                # Significant movement - run full SLAM

                slam.predict(delta_trans, delta_rot1, delta_rot2, direction)
                slam.update_and_resample(local_scan)

                slam.global_counter += 1

                pose_std = np.std(slam.poses, axis=0)
                
                start_seg_th = prev_th
                accum_dx, accum_dy, accum_dth = 0.0, 0.0, 0.0
                counter += 1

                # Update every five iterations for optimization
                if counter % 5 == 0:
                    best_idx = slam.get_best_particle_index()
                    map_plot.set_data(slam.maps[best_idx].grid)
                    px = slam.poses[:, 0]
                    py = slam.poses[:, 1]
                    particles_plot.set_offsets(np.c_[px, py])
                    plt.pause(0.01)

            # else:
            #     for i in range(slam.N):
            #         slam.maps[i].update_with_numba(slam.poses[i], local_scan)
            #     counter += 1

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()