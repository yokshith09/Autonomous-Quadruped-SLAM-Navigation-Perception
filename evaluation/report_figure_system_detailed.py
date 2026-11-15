"""Whole-system architecture in draw.io style (PNG + .drawio); group titles without background boxes."""
import os

from evaluation.drawio_kit import build

OUT = os.path.join("docs", "report_phase2", "figures")


def system():
    W, H = 2000, 960
    n, e = {}, []

    def frame(nid, x, y, w, h, title, color, dashed=False):
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="frame", color="none", label=title, size=22)

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
    frame("core", 950, 420, 470, 230, "Robot core: SLAM and exploration", "purple")
    item("icp", 725, 490, 210, 120, "ICP pose\nestimation\nloop closure")
    item("mapc", 955, 490, 210, 120, "Map\nconstruction\noccupancy grid")
    item("fro", 1185, 490, 210, 120, "Frontier\ndetection\noperator route")
    frame("dec", 1490, 420, 290, 230, "Decision", "red")
    item("nav", 1515, 480, 240, 70, "Navigation goal")
    item("cmd", 1515, 565, 240, 70, "Path and\nmovement commands", size=17)
    frame("mot", 700, 730, 520, 190, "Motion control", "orange")
    item("loco", 725, 790, 220, 100, "Locomotion\ntrot gait, watchdog")
    item("drv", 975, 790, 220, 100, "Joint drivers\n12 joints")
    frame("log", 1490, 40, 290, 300, "Logging", "yellow")
    item("log1", 1515, 100, 240, 60, "Map coordinates")
    item("log2", 1515, 175, 240, 60, "Hash (SHA-256)")
    item("log3", 1515, 250, 240, 60, "ROI metadata")
    n["cloud"] = dict(x=1830, y=155, w=150, h=100, kind="ellipse", color="white", label="Cloud sync", size=18)

    # arrows start and end on boxes or pictures (the group panels have no background)
    for src, dst, lab, kw in [
        ("env1", "dep", "environment", dict(s=(1, 0.714), d=(0, 0.25), ly=-18)),         # y = 350
        ("dep", "robi", "sensor data", dict(s="r", d="l")),
        ("robi", "per1", "colour video", dict(s=(1, 0.281), d="l", ly=-18)),             # y = 145
        ("robt", "icp", "depth, odometry, gyro", dict(s=(0.5, 1), d=(0.548, 0))),        # x = 840
        ("fro", "nav", "pose, map,\nfrontiers", dict(s=(1, 0.333), d=(0, 0.714), ly=-32)),  # y = 530
        ("per2", "nav", "crime alerts", dict(s="b", d="t", pts=[(1230, 380), (1635, 380)], li=1)),
        ("per1", "log1", "clips", dict(s="r", d=(0, 0.75))),                             # y = 145
        ("log2", "cloud", "upload", dict(s="r", d="l", ly=-18)),                         # y = 205
        ("cmd", "drv", "movement commands", dict(s="b", d="r", pts=[(1635, 840)], li=1)),
        ("loco", "env3", "robot moves", dict(s="l", d="b", pts=[(165, 840)], li=0)),
        ("loco", "imu", "leg odometry", dict(s=(0.795, 0), d="b", pts=[(900, 690), (505, 690)], li=1)),
    ]:
        e.append(dict(src=src, dst=dst, label=lab, size=16, **kw))
    build(n, e, W, H, os.path.join(OUT, "fig_system_architecture_detailed"), "System architecture")


if __name__ == "__main__":
    system()
