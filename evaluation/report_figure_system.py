"""Whole-system architecture, hand-drawn style (PNG + .drawio with draw.io's sketch style), no background fills."""
import os

from evaluation import drawio_kit as K

OUT = os.path.join("docs", "report_phase2", "figures")


def system():
    K.SKETCH, K.NOFILL, font = True, True, K.FONT
    K.FONT = "Segoe Print"
    W, H = 2120, 980
    n, e = {}, []

    def frame(nid, x, y, w, h, title, color):
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="frame", color=color, label=title, size=24, lw=2.2)

    def item(nid, x, y, w, h, label, size=19):
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="round", color="dark", label=label, size=size, sub=17, lw=1.6)

    frame("env", 20, 230, 290, 390, "Scene", "grey")
    item("env1", 45, 305, 240, 70, "Rooms, rubble")
    item("env2", 45, 395, 240, 80, "People, furniture,\nevidence items", size=18)
    item("env3", 45, 495, 240, 90, "Unknown layout\nno prior map")
    frame("sen", 370, 150, 280, 480, "Sensing", "green")
    item("cam", 395, 225, 230, 80, "Colour camera\nKinect RGB")
    item("dep", 395, 335, 230, 80, "Depth camera\nKinect depth")
    item("imu", 395, 445, 230, 80, "Gyro +\nleg odometry")
    frame("rob", 710, 40, 290, 300, "Quadruped robot", "blue")
    item("rob1", 735, 110, 240, 90, "12-joint legs\ntrot gait")
    item("rob2", 735, 220, 240, 90, "Kinect on the body\n60 deg view", size=18)
    frame("per", 1060, 40, 380, 300, "Scene understanding", "orange")
    item("per1", 1085, 110, 330, 90, "Fine-tuned VideoMamba\n+ HorNet gated convolution", size=18)
    item("per2", 1085, 220, 330, 90, "Crime or not, kind of\nemergency, crime type", size=18)
    frame("core", 710, 420, 730, 240, "Robot core: SLAM and exploration", "purple")
    item("icp", 735, 495, 215, 130, "ICP pose\nestimation\nloop closure")
    item("mapc", 967, 495, 215, 130, "Map\nconstruction\noccupancy grid")
    item("fro", 1199, 495, 215, 130, "Frontier\ndetection\noperator route")
    frame("dec", 1620, 420, 300, 240, "Decision", "red")
    item("nav", 1645, 490, 250, 70, "Navigation goal")
    item("cmd", 1645, 575, 250, 70, "Movement commands", size=18)
    frame("mot", 710, 750, 530, 200, "Motion control", "orange")
    item("loco", 735, 820, 225, 100, "Locomotion\ntrot gait, watchdog", size=18)
    item("drv", 985, 820, 225, 100, "Joint drivers\n12 joints")
    frame("log", 1620, 40, 300, 300, "Logging", "yellow")
    item("log1", 1645, 105, 250, 62, "Map coordinates")
    item("log2", 1645, 180, 250, 62, "Hash (SHA-256)")
    item("log3", 1645, 255, 250, 62, "ROI metadata")
    n["cloud"] = dict(x=1955, y=135, w=150, h=110, kind="ellipse", color="dark", label="Cloud\nsync", size=20,
                      bold="all", lw=2.0)

    e.append(dict(src="env", dst="sen", s=(1, 0.397), d="l", label="environment", size=17, ly=-20))
    e.append(dict(src="sen", dst="rob", s="r", d="l", label="sensor data", size=17))
    e.append(dict(src="rob", dst="per", s="r", d="l", label="colour video", size=17, ly=-20))
    e.append(dict(src="rob", dst="core", s="b", d=(0.198, 0), label="depth, odometry, gyro", size=17))
    e.append(dict(src="core", dst="dec", s="r", d="l", label="pose, map, frontiers", size=17, ly=-22))
    e.append(dict(src="per", dst="dec", s="b", d=(0.5, 0), pts=[(1250, 385), (1770, 385)], label="crime alerts",
                  size=17, li=1))
    e.append(dict(src="per", dst="log", s="r", d="l", label="clips", size=17, ly=-20))
    e.append(dict(src="log", dst="cloud", s="r", d="l", label="upload", size=17, ly=-20))
    e.append(dict(src="dec", dst="mot", s="b", d=(1, 0.15), pts=[(1770, 780)], label="movement commands", size=17,
                  li=1))
    e.append(dict(src="mot", dst="env", s="l", d="b", pts=[(165, 850)], label="robot moves", size=17, li=0))
    e.append(dict(src="mot", dst="sen", s=(0.12, 0), d="b", pts=[(774, 700), (510, 700)], label="leg odometry",
                  size=17, li=1))
    for d in (n, e):
        pass
    for nd in n.values():
        nd.setdefault("color", "dark")
    K.build(n, e, W, H, os.path.join(OUT, "fig_system_architecture"), "System architecture")
    K.SKETCH, K.NOFILL, K.FONT = False, False, font
