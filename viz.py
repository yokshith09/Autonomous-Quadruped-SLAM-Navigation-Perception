import cv2
import numpy as np

def create_debug_dashboard(rgb_img, depth_heatmap, grid_img, metrics):
    """
    Creates a high-resolution 4-panel dashboard (800x800) for SLAM demonstration.
    +-----------------------+-----------------------+
    |  RGB Camera (400x400) | Depth Sensor (400x400)|
    +-----------------------+-----------------------+
    | 2D SLAM Map (400x400) | Telemetry & Readout   |
    +-----------------------+-----------------------+
    """
    panel_s = 400
    dashboard = np.zeros((panel_s * 2, panel_s * 2, 3), dtype=np.uint8)
    
    # 1. RGB Camera Feed (Top-Left)
    if rgb_img is not None and rgb_img.size > 0:
        rgb_resized = cv2.resize(rgb_img, (panel_s, panel_s))
    else:
        rgb_resized = np.zeros((panel_s, panel_s, 3), dtype=np.uint8)
    cv2.putText(rgb_resized, "RGB CAMERA FEED", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    dashboard[0:panel_s, 0:panel_s] = rgb_resized
        
    # 2. Depth Heatmap (Top-Right)
    if depth_heatmap is not None and depth_heatmap.size > 0:
        depth_resized = cv2.resize(depth_heatmap, (panel_s, panel_s))
    else:
        depth_resized = np.zeros((panel_s, panel_s, 3), dtype=np.uint8)
    cv2.putText(depth_resized, "DEPTH SENSOR HEATMAP", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    dashboard[0:panel_s, panel_s:panel_s*2] = depth_resized
        
    # 3. Live 2D Occupancy Grid Map (Bottom-Left)
    if grid_img is not None and grid_img.size > 0:
        grid_resized = cv2.resize(grid_img, (panel_s, panel_s))
    else:
        grid_resized = np.zeros((panel_s, panel_s, 3), dtype=np.uint8)
    dashboard[panel_s:panel_s*2, 0:panel_s] = grid_resized
        
    # 4. SLAM & Ground-Truth Metrics Panel (Bottom-Right)
    metrics_panel = np.ones((panel_s, panel_s, 3), dtype=np.uint8) * 18  # Dark slate background
    
    y = 28
    cv2.putText(metrics_panel, "SLAM vs GROUND TRUTH READOUT", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.line(metrics_panel, (12, y + 8), (panel_s - 12, y + 8), (80, 80, 80), 1)
    y += 34

    for key, value in metrics.items():
        if key.startswith("---"):
            y += 8
            cv2.putText(metrics_panel, key.replace("-", "").strip(), (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 200, 255), 1)
            y += 24
            continue

        text = f"{key}: {value}"
        if "Drift" in key or "Error" in key:
            color = (0, 255, 255) # Bright Yellow
        elif "ICP Status" in key:
            color = (0, 255, 0) if "OK" in str(value) else (0, 0, 255)
        elif "SLAM" in key:
            color = (255, 100, 255) # Magenta
        elif "True" in key or "Ground" in key:
            color = (255, 255, 0) # Cyan
        else:
            color = (220, 220, 220)

        cv2.putText(metrics_panel, text, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, color, 1)
        y += 23
        
    dashboard[panel_s:panel_s*2, panel_s:panel_s*2] = metrics_panel
    
    # Add clean dividing lines
    cv2.line(dashboard, (panel_s, 0), (panel_s, panel_s * 2), (255, 255, 255), 2)
    cv2.line(dashboard, (0, panel_s), (panel_s * 2, panel_s), (255, 255, 255), 2)
    
    return dashboard
