import math

FREQ = 5.0
HIP_AMP = 0.40
LEG_LIFT = 0.02
LEG_PLANT = -0.02
OMEGA = 2 * math.pi * FREQ
# Restored to match path_planning_nav.py / path_planning_nav_dstar.py.
# The current tree had these at SLOW_ANGLE=0.2 / HEADING_RATE=0.1, which is a
# 4x more aggressive turn-rate and switches to the fast turn gain at a much
# smaller heading error than either previous version used. apply_gait() itself
# is byte-identical across all three files -- this steering aggressiveness is
# what actually changed the perceived "gait".
SLOW_ANGLE = 0.3
HEADING_RATE = 0.05

def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

class LizardRobot:
    def __init__(self, sim):
        self.sim = sim
        try:
            self.base = sim.getObject('/Base')
            self.fh = sim.getObject('/Base/FrontJoint')
            self.bh = sim.getObject('/Base/BackJoint')
            self.fl = sim.getObject('/Base/FrontJoint/FrontLink/FrontLeftJoint')
            self.fr = sim.getObject('/Base/FrontJoint/FrontLink/FrontRightJoint')
            self.bl = sim.getObject('/Base/BackJoint/BackLink/BackLeftJoint')
            self.br = sim.getObject('/Base/BackJoint/BackLink/BackRightJoint')
        except Exception:
            # Fallback for original LIZARD.ttt scene
            self.base = sim.getObject('/Back_joint')
            self.fh = sim.getObject('/Front_joint')
            self.bh = sim.getObject('/Back_joint')
            self.fl = sim.getObject('/FL_joint')
            self.fr = sim.getObject('/FR_joint')
            self.bl = sim.getObject('/BL_joint')
            self.br = sim.getObject('/BR_joint')

        # Color robot base for distinct 3D visual clarity (Electric Orange/Cyan)
        try:
            self.sim.setShapeColor(self.base, None, self.sim.colorcomponent_ambient_diffuse, [1.0, 0.4, 0.0])
        except Exception:
            pass

        self._prev_imu_yaw = None

    def get_odometry_imu_delta(self, dt: float, speed_cmd: float):
        """Return (forward_displacement_m, gyro_dyaw_rad) using IMU orientation delta and gait kinematics."""
        # Calibrated physical gait stride speed at speed=1.0 is ~0.16 m/s
        d_fwd = 0.16 * speed_cmd * max(0.0, min(dt, 0.25))
        try:
            o = self.sim.getObjectOrientation(self.base, self.sim.handle_world)
            yaw = norm_angle(o[2] + math.pi)
            if self._prev_imu_yaw is None:
                self._prev_imu_yaw = yaw
                return d_fwd, 0.0
            dyaw = norm_angle(yaw - self._prev_imu_yaw)
            self._prev_imu_yaw = yaw
            return d_fwd, dyaw
        except Exception:
            return d_fwd, 0.0

    def get_pose(self):
        p = self.sim.getObjectPosition(self.base, self.sim.handle_world)
        o = self.sim.getObjectOrientation(self.base, self.sim.handle_world)
        # The gait physically walks in the opposite direction from CoppeliaSim o[2].
        # Flip by pi so 'heading' = actual forward walking direction.
        heading = norm_angle(o[2] + math.pi)
        return p[0], p[1], heading, o

    def steer_heading(self, orient, current_heading, desired_heading, step_count=0):
        err = norm_angle(desired_heading - current_heading)
        if abs(err) < 0.05:
            correction = 0
        else:
            gain = 0.4 if abs(err) > SLOW_ANGLE else 0.2
            correction = max(-HEADING_RATE, min(HEADING_RATE, err * gain))
        
        new_gamma = orient[2] + correction
        
        # BUG #4 FIX: Do not silently swallow exceptions here.
        try:
            self.sim.setObjectOrientation(self.base, [orient[0], orient[1], new_gamma], self.sim.handle_world)
        except Exception as e:
            print(f"  [ERROR] steer_heading failed: {e}")

        return abs(err)

    def apply_gait(self, t, speed):
        freq_mod = 0.6 + 0.4 * speed
        wt = OMEGA * freq_mod * t
        s_wt = math.sin(wt)
        c_wt = math.cos(wt)
        
        f_hip = -HIP_AMP * speed * s_wt
        b_hip = -HIP_AMP * speed * math.sin(wt + math.pi)
        
        la = max(0.0, c_wt)
        lb = max(0.0, -c_wt)
        sa = la**2 * (3 - 2 * la)
        sb = lb**2 * (3 - 2 * lb)
        
        try:
            self.sim.setJointTargetPosition(self.fh, f_hip)
            self.sim.setJointTargetPosition(self.bh, b_hip)
            self.sim.setJointTargetPosition(self.fl, LEG_PLANT + (LEG_LIFT - LEG_PLANT) * sa)
            self.sim.setJointTargetPosition(self.fr, LEG_PLANT + (LEG_LIFT - LEG_PLANT) * sb)
            self.sim.setJointTargetPosition(self.bl, LEG_PLANT + (LEG_LIFT - LEG_PLANT) * sb)
            self.sim.setJointTargetPosition(self.br, LEG_PLANT + (LEG_LIFT - LEG_PLANT) * sa)
        except Exception as e:
            print(f"  [ERROR] apply_gait failed: {e}")


    def stop_motors(self):
        for jh in [self.fh, self.bh, self.fl, self.fr, self.bl, self.br]:
            try:
                self.sim.setJointTargetPosition(jh, 0)
            except Exception:
                pass


def patch_coppelasim_scripts(sim):
    print("\n  ========================================")
    print("  COPPELIASIM LUA SCRIPT PATCHING")
    print("  ========================================")
    
    # 1. Patch the Kinect depth camera parent script
    try:
        kinect = sim.getObject('/Kinect_camera_support/kinect')
        # Empty script text disables it
        sim.createScript(sim.scripttype_childscript, "-- patched", kinect, 'patched_kinect')
        print("  [OK] Patched child script on `/Kinect_camera_support/kinect`")
    except Exception as e:
        print(f"  [WARN] Could not patch `/Kinect_camera_support/kinect`: {e}")
        
    # 2. Patch the depth camera specifically
    try:
        depth_cam = sim.getObject('/Kinect_camera_support/kinect/kinect_joint/kinect_body/kinect_depth')
        sim.createScript(sim.scripttype_childscript, "-- patched", depth_cam, 'patched_depth')
        print("  [OK] Patched script on `kinect_depth`")
    except Exception:
        try:
            depth_cam = sim.getObject('kinect_depth')
            sim.createScript(sim.scripttype_childscript, "-- patched", depth_cam, 'patched_depth')
            print("  [OK] Patched script on `kinect_depth`")
        except Exception as e:
            print(f"  [WARN] Could not patch `kinect_depth`: {e}")
            
    print("  ========================================\n")
    
    # script creates/configures the dynamic joint motors.  Disabling it leaves
    # remote target-position commands succeeding but the robot physically
    # stationary, which made this look like a planning or SLAM failure.
    print("  [OK] Preserved `/Base` dynamics/control script for locomotion")
