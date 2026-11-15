"""Whole-system architecture in draw.io style (PNG + .drawio). Solid = built and tested, dashed = planned."""
import os

from evaluation.drawio_kit import build

OUT = os.path.join("docs", "report_phase2", "figures")


def system():
    W, H = 2000, 960
    n, e = {}, []

    def frame(nid, x, y, w, h, title, color, dashed=False):
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="frame", color=color, label=title, size=22, dashed=dashed)

    def item(nid, x, y, w, h, label, dashed=False, size=19):
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="round", color="white", label=label, size=size, sub=17, dashed=dashed)

    frame("env", 20, 230, 290, 380, "Scene (crime / rescue)", "grey")
    item("env1", 45, 300, 240, 70, "Rooms and rubble")
    item("env2", 45, 390, 240, 70, "People, furniture,\nevidence items", size=17)
    item("env3", 45, 480, 240, 90, "Unknown layout\nno prior map")
    frame("sen", 370, 150, 270, 470, "Sensing", "green")
    item("cam", 395, 220, 220, 80, "Colour camera\nKinect RGB")
    item("dep", 395, 330, 220, 80, "Depth camera\nKinect depth, 60 deg")
    item("imu", 395, 440, 220, 80, "Gyro and\nleg odometry")
    frame("rob", 700, 40, 280, 300, "Quadruped robot", "blue")
    n["robi"] = dict(x=720, y=95, w=240, h=178, kind="image", image=os.path.join(OUT, "robot_sim.png"))
    n["robt"] = dict(x=700, y=282, w=280, h=40, kind="text", label="12 joints, trot gait", size=17, bold=False)
    frame("per", 1040, 40, 380, 300, "Scene understanding", "orange")
    item("per1", 1065, 100, 330, 90, "Fine-tuned VideoMamba\n+ HorNet gated convolution")
    item("per2", 1065, 210, 330, 100, "Crime or not, kind of\nemergency, crime type (13)", size=17)
    frame("core", 700, 420, 720, 230, "Robot core: SLAM and exploration", "purple")
    item("icp", 725, 490, 210, 120, "ICP pose\nestimation\nloop closure")
    item("mapc", 955, 490, 210, 120, "Map\nconstruction\noccupancy grid")
    item("fro", 1185, 490, 210, 120, "Frontier\ndetection\noperator route")
    frame("dec", 1490, 420, 290, 230, "Decision", "red")
    item("nav", 1515, 480, 240, 70, "Navigation goal")
    item("cmd", 1515, 565, 240, 70, "Path and\nmovement commands", size=17)
    frame("mot", 700, 730, 520, 190, "Motion control", "orange")
    item("loco", 725, 790, 220, 100, "Locomotion\ntrot gait, watchdog")
    item("drv", 975, 790, 220, 100, "Joint drivers\n12 joints")
    frame("log", 1490, 40, 290, 300, "Logging (planned)", "yellow", dashed=True)
    item("log1", 1515, 100, 240, 60, "Map coordinates", dashed=True)
    item("log2", 1515, 175, 240, 60, "Hash (SHA-256)", dashed=True)
    item("log3", 1515, 250, 240, 60, "ROI metadata", dashed=True)
    n["cloud"] = dict(x=1830, y=140, w=150, h=100, kind="ellipse", color="grey", label="Cloud sync\n(planned)",
                      size=18, dashed=True)

    e.append(dict(src="env", dst="sen", s=(1, 0.408), d="l", label="environment", size=16, ly=-18))
    e.append(dict(src="sen", dst="rob", s="r", d="l", label="sensor data", size=16))
    e.append(dict(src="rob", dst="per", s="r", d="l", label="colour video", size=16))
    e.append(dict(src="rob", dst="core", s="b", d=(0.195, 0), label="depth, odometry, gyro", size=16))
    e.append(dict(src="core", dst="dec", s="r", d="l", label="pose, map, frontiers", size=16))
    e.append(dict(src="per", dst="dec", s="b", d=(0.5, 0), pts=[(1230, 380), (1635, 380)], dashed=True,
                  label="crime alerts (planned)", size=16, li=1))
    e.append(dict(src="per", dst="log", s="r", d="l", dashed=True, label="clips", size=16))
    e.append(dict(src="log", dst="cloud", s="r", d="l", dashed=True, label="upload", size=16))
    e.append(dict(src="dec", dst="mot", s="b", d=(1, 0.16), pts=[(1635, 760)], label="movement commands", size=16, li=1))
    e.append(dict(src="mot", dst="env", s="l", d="b", pts=[(165, 825)], label="robot moves", size=16, li=0))
    e.append(dict(src="mot", dst="sen", s=(0.12, 0), d="b", pts=[(762, 690), (505, 690)], label="leg odometry",
                  size=16, li=1))
    n["leg"] = dict(x=1250, y=810, w=530, h=120, kind="box", color="white", size=17, bold=False,
                    label="Solid: built and tested (navigation in simulation,\nscene understanding on recorded video)."
                          "\nDashed: planned, not yet integrated.")
    build(n, e, W, H, os.path.join(OUT, "fig_system_architecture"), "System architecture")
