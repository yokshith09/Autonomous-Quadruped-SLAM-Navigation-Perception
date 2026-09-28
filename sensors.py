import math
import numpy as np
import cv2
import os

KINECT_FOV = math.radians(60.0)
DEPTH_MIN_RANGE = 0.1
DEPTH_MAX_RANGE = 8.0

def initialize_sensors(sim, base):
    depth_cam = None
    rgb_cam = None

    print("\n  ========================================")
    print("  SENSOR INITIALIZATION")
    print("  ========================================")

    # Use sibling paths matching the scene hierarchy
    try:
        depth_cam = sim.getObject('/Kinect_camera_support/kinect/kinect_joint/kinect_body/kinect_depth')
        print("  [OK] Depth camera found (absolute path)")
    except Exception:
        try:
            depth_cam = sim.getObject('kinect_depth')
            print("  [OK] Depth camera found (global name)")
        except Exception as e:
            print(f"  [ERROR] Depth camera NOT found: {e}")
            print("  ========================================")
            return None, None, None

    try:
        rgb_cam = sim.getObject('/Kinect_camera_support/kinect/kinect_joint/kinect_body/kinect_rgb')
        print("  [OK] RGB camera found (absolute path)")
    except Exception:
        try:
            rgb_cam = sim.getObject('kinect_rgb')
            print("  [OK] RGB camera found (global name)")
        except Exception as e:
            print(f"  [ERROR] RGB camera NOT found: {e}")

    try:
        sensor_res = sim.getVisionSensorResolution(depth_cam)
    except Exception as e:
        print(f"  [ERROR] Could not get sensor resolution: {e}")
        print("  ========================================")
        return None, None, None

    focal_length = (sensor_res[0] / 2.0) / math.tan(KINECT_FOV / 2.0)
    camera_matrix = {
        'fx': focal_length,
        'fy': focal_length,
        'cx': sensor_res[0] / 2.0,
        'cy': sensor_res[1] / 2.0,
        'width': sensor_res[0],
        'height': sensor_res[1]
    }

    print(f"  Resolution: {sensor_res[0]}x{sensor_res[1]}", flush=True)
    print(f"  Focal length: {focal_length:.1f}px", flush=True)
    print(f"  FOV: {math.degrees(KINECT_FOV):.1f} degrees", flush=True)
    print(f"  Camera Intrinsics: fx={camera_matrix['fx']:.4f}, fy={camera_matrix['fy']:.4f}, cx={camera_matrix['cx']:.4f}, cy={camera_matrix['cy']:.4f}", flush=True)
    T_base_cam = None
    if base is not None and depth_cam is not None:
        try:
            raw_t = list(sim.getObjectMatrix(depth_cam, base))
            world_t = list(sim.getObjectMatrix(depth_cam, sim.handle_world))
            # Align with LizardRobot forward walking frame (heading = o[2] + pi -> rotate XY by pi)
            for idx in range(8):
                raw_t[idx] = -raw_t[idx]
            raw_t[11] = world_t[11]
            T_base_cam = raw_t
            print("  [OK] Calibrated static camera-to-base extrinsics T_base_cam", flush=True)
        except Exception as e:
            print(f"  [WARN] Could not get T_base_cam relative to base: {e}", flush=True)
    camera_matrix['T_base_cam'] = T_base_cam

    return depth_cam, rgb_cam, camera_matrix

def grab_depth_frame(sim, depth_cam_handle, camera_matrix):
    if depth_cam_handle is None:
        return None
    try:
        # Pass 1 for options to get depth in meters directly!
        result = sim.getVisionSensorDepth(depth_cam_handle, 1)
        if isinstance(result, list) or isinstance(result, tuple):
            depth_data = result[0]
            if len(result) > 1:
                res = result[1]
                w, h = res[0], res[1]
            else:
                w, h = int(camera_matrix['width']), int(camera_matrix['height'])
        else:
            depth_data = result
            w, h = int(camera_matrix['width']), int(camera_matrix['height'])

        if not depth_data or len(depth_data) == 0:
            return None

        # copy() to allow write access/clamping
        depth_np = np.frombuffer(depth_data, dtype=np.float32).copy()
        depth_img = depth_np.reshape((h, w))
        # Clamp bounds
        depth_img[(depth_img < DEPTH_MIN_RANGE) | (depth_img > DEPTH_MAX_RANGE)] = 0
        return depth_img
    except Exception as e:
        print(f"  [DEBUG] grab_depth_frame failed: {e}")
        # Fallback to deprecated getVisionSensorDepthBuffer
        try:
            res_val = sim.getVisionSensorDepthBuffer(depth_cam_handle)
            if isinstance(res_val, list) or isinstance(res_val, tuple):
                depth_data = res_val[0]
                res = res_val[1]
                w, h = res[0], res[1]
            else:
                depth_data = res_val
                w, h = int(camera_matrix['width']), int(camera_matrix['height'])

            if depth_data and len(depth_data) > 0:
                depth_np = np.frombuffer(depth_data, dtype=np.float32).copy()
                depth_img = depth_np.reshape((h, w))
                depth_img[(depth_img < DEPTH_MIN_RANGE) | (depth_img > DEPTH_MAX_RANGE)] = 0
                return depth_img
        except Exception as e2:
            print(f"  [DEBUG] fallback grab_depth_frame failed: {e2}")
        return None

def grab_rgb_frame(sim, rgb_cam_handle, camera_matrix):
    if rgb_cam_handle is None:
        return None
    try:
        result = sim.getVisionSensorCharImage(rgb_cam_handle)
        if isinstance(result, list) or isinstance(result, tuple):
            img_data = result[0]
            if len(result) > 1:
                res = result[1]
                if isinstance(res, list) or isinstance(res, tuple):
                    w, h = res[0], res[1]
                else:
                    w, h = result[1], result[2]
            else:
                w, h = int(camera_matrix['width']), int(camera_matrix['height'])
        else:
            img_data = result
            w, h = int(camera_matrix['width']), int(camera_matrix['height'])

        if not img_data or len(img_data) == 0:
            return None

        img_np = np.frombuffer(img_data, dtype=np.uint8).copy().reshape(h, w, 3)
        img_np = cv2.flip(img_np, 0)
        img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        return img_np
    except Exception as e:
        print(f"  [DEBUG] grab_rgb_frame failed: {e}")
        # Fallback to getVisionSensorImg
        try:
            res_val = sim.getVisionSensorImg(rgb_cam_handle)
            if isinstance(res_val, list) or isinstance(res_val, tuple):
                img_data = res_val[0]
                res = res_val[1]
                w, h = res[0], res[1]
            else:
                img_data = res_val
                w, h = int(camera_matrix['width']), int(camera_matrix['height'])

            if img_data and len(img_data) > 0:
                img_np = np.frombuffer(img_data, dtype=np.uint8).reshape(h, w, 3)
                img_np = cv2.flip(img_np, 0)
                img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
                return img_np
        except Exception as e2:
            print(f"  [DEBUG] fallback grab_rgb_frame failed: {e2}")
        return None

def depth_to_world_points(depth_img, camera_matrix, cam_matrix):
    h, w = depth_img.shape
    u_coords = np.arange(w)
    v_coords = np.arange(h)
    uu, vv = np.meshgrid(u_coords, v_coords)

    fx = camera_matrix['fx']
    fy = camera_matrix['fy']
    cx = camera_matrix['cx']
    cy = camera_matrix['cy']
    z = depth_img

    mask = (z > DEPTH_MIN_RANGE) & (z < DEPTH_MAX_RANGE)
    if np.sum(mask) == 0:
        return np.array([]).reshape(0, 3)

    x_cam = -(uu[mask] - cx) * z[mask] / fx
    y_cam = (vv[mask] - cy) * z[mask] / fy
    z_cam = z[mask]
    points_cam = np.column_stack([x_cam, y_cam, z_cam])

    R = np.array([
        [cam_matrix[0], cam_matrix[1], cam_matrix[2]],
        [cam_matrix[4], cam_matrix[5], cam_matrix[6]],
        [cam_matrix[8], cam_matrix[9], cam_matrix[10]]
    ])
    T = np.array([cam_matrix[3], cam_matrix[7], cam_matrix[11]])

    points_world = (R @ points_cam.T).T + T
    return points_world

def depth_to_base_points(depth_img, camera_matrix, T_base_cam=None):
    """Back-project depth image to 3D point cloud strictly in the robot base frame."""
    if T_base_cam is None:
        T_base_cam = camera_matrix.get('T_base_cam', None) if camera_matrix else None
    
    h, w = depth_img.shape
    u_coords = np.arange(w)
    v_coords = np.arange(h)
    uu, vv = np.meshgrid(u_coords, v_coords)

    fx = camera_matrix['fx']
    fy = camera_matrix['fy']
    cx = camera_matrix['cx']
    cy = camera_matrix['cy']
    z = depth_img

    mask = (z > DEPTH_MIN_RANGE) & (z < DEPTH_MAX_RANGE)
    if np.sum(mask) == 0:
        return np.array([]).reshape(0, 3)

    x_cam = -(uu[mask] - cx) * z[mask] / fx
    y_cam = (vv[mask] - cy) * z[mask] / fy
    z_cam = z[mask]
    points_cam = np.column_stack([x_cam, y_cam, z_cam])

    if T_base_cam is not None and len(T_base_cam) >= 12:
        R = np.array([
            [T_base_cam[0], T_base_cam[1], T_base_cam[2]],
            [T_base_cam[4], T_base_cam[5], T_base_cam[6]],
            [T_base_cam[8], T_base_cam[9], T_base_cam[10]]
        ])
        T = np.array([T_base_cam[3], T_base_cam[7], T_base_cam[11]])
        points_base = (R @ points_cam.T).T + T
    else:
        points_base = points_cam

    return points_base

def get_depth_heatmap(depth_matrix):
    dh, dw = depth_matrix.shape
    valid_depths = depth_matrix[depth_matrix > 0]
    d_min = np.min(valid_depths) if len(valid_depths) > 0 else 0.0
    d_max = np.max(depth_matrix)
    
    valid_mask = depth_matrix > 0
    if np.any(valid_mask):
        d_range = d_max - d_min
        if d_range > 0:
            depth_norm = ((depth_matrix - d_min) / d_range * 255).astype(np.uint8)
            depth_norm[~valid_mask] = 0
            depth_heatmap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            depth_heatmap[~valid_mask] = (0, 0, 0)
        else:
            depth_heatmap = np.zeros((dh, dw, 3), dtype=np.uint8)
    else:
        depth_heatmap = np.zeros((dh, dw, 3), dtype=np.uint8)
    return depth_heatmap

def print_and_save_matrices(sim, depth_matrix, rgb_matrix, step):
    dh, dw = depth_matrix.shape
    print("\n" + "=" * 60, flush=True)
    print(f"  [PERCEPTION UPDATE - STEP {step}] DEPTH & RGB MATRICES", flush=True)
    print("=" * 60, flush=True)

    valid_depths = depth_matrix[depth_matrix > 0]
    d_min = np.min(valid_depths) if len(valid_depths) > 0 else 0.0
    d_max = np.max(depth_matrix)
    d_mean = np.mean(depth_matrix)

    print(f"  DEPTH MATRIX: Shape={depth_matrix.shape} | Dtype={depth_matrix.dtype}", flush=True)
    print(f"    Min: {d_min:.3f}m | Max: {d_max:.3f}m | Mean: {d_mean:.3f}m", flush=True)
    print("    Sample Center 5x5 Matrix (distances in meters):", flush=True)
    center_d = depth_matrix[dh//2-2 : dh//2+3, dw//2-2 : dw//2+3]
    for row in center_d:
        print("      " + " ".join(f"{val:5.3f}" for val in row), flush=True)

    print("-" * 60, flush=True)

    print(f"  RGB MATRIX: Shape={rgb_matrix.shape} | Dtype={rgb_matrix.dtype}", flush=True)
    r_mean = np.mean(rgb_matrix[:, :, 2])  # BGR
    g_mean = np.mean(rgb_matrix[:, :, 1])
    b_mean = np.mean(rgb_matrix[:, :, 0])
    print(f"    Channel Means: Red={r_mean:.1f} | Green={g_mean:.1f} | Blue={b_mean:.1f}", flush=True)
    print("    Sample Center 5x5 Matrix (Red Channel values 0-255):", flush=True)
    rh, rw = rgb_matrix.shape[:2]
    center_r = rgb_matrix[rh//2-2 : rh//2+3, rw//2-2 : rw//2+3, 2]
    for row in center_r:
        print("      " + " ".join(f"{val:3d}" for val in row), flush=True)
    print("=" * 60 + "\n", flush=True)

    try:
        msg = (f"[NavPulse] Depth Matrix {depth_matrix.shape} (Mean={d_mean:.2f}m) | "
               f"RGB Matrix {rgb_matrix.shape} (Mean Red={r_mean:.1f})")
        sim.addLog(sim.verbosity_scriptinfos, msg)
    except Exception:
        pass

    output_dir = "./captured_matrices"
    os.makedirs(output_dir, exist_ok=True)
    np.savez_compressed(f"{output_dir}/matrix_data.npz", depth=depth_matrix, rgb=rgb_matrix)
    np.savetxt(f"{output_dir}/depth_matrix.txt", depth_matrix, fmt="%.3f")
    depth_heatmap = get_depth_heatmap(depth_matrix)
    cv2.imwrite(f"{output_dir}/depth_heatmap.png", depth_heatmap)
    cv2.imwrite(f"{output_dir}/rgb_image.png", rgb_matrix)
    cv2.imwrite(f"{output_dir}/depth_step{step:04d}.png", depth_heatmap)
    cv2.imwrite(f"{output_dir}/rgb_step{step:04d}.png", rgb_matrix)
