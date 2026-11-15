"""Whole-system architecture: big pictures, little text, plain draw.io style in Times New Roman (PNG + .drawio with embedded pictures).

Canvas 1600 x 900 px; titles 30 px and labels 24-26 px print at about 9 pt at full page width.
"""
import os

from evaluation import drawio_kit as K

OUT = os.path.join("docs", "report_phase2", "figures")


def img(name):
    return os.path.join(OUT, name)


def system():
    font = K.FONT
    K.FONT = "Times New Roman"
    W, H = 1600, 900
    n, e = {}, []

    def frame(nid, x, y, w, h, title, color):
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="frame", color=color, label=title, size=32, noborder=True)

    def item(nid, x, y, w, h, label, size=25):
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="round", color="white", label=label, size=size, lw=1.4)

    def pic(nid, x, y, w, h, name):
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="image", image=img(name))

    def text(nid, x, y, w, h, label, size=24):
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="text", label=label, size=size, bold=False)

    # row 1: sensing to understanding
    frame("scene", 20, 20, 300, 380, "Scene", "grey")
    pic("scene_i", 45, 80, 250, 255, "img_scene.png")
    text("scene_t", 20, 342, 300, 46, "unknown, no map")
    frame("sen", 360, 20, 340, 380, "Kinect + IMU", "green")
    pic("sen_i", 380, 85, 300, 135, "img_rgbd.png")
    text("sen_t", 360, 232, 340, 140, "colour + depth\ngyro, leg odometry")
    frame("rob", 740, 20, 280, 380, "Quadruped", "blue")
    pic("rob_i", 760, 85, 240, 197, "img_robot.png")
    text("rob_t", 740, 300, 280, 70, "12 joints")
    frame("und", 1060, 20, 300, 380, "Video model", "orange")
    pic("und_i", 1110, 80, 200, 200, "icon_frames.png")
    text("und_t", 1060, 292, 300, 80, "VideoMamba\ncrime type")
    frame("log", 1400, 20, 180, 380, "Logging", "yellow")
    item("log1", 1415, 90, 150, 70, "map log")
    item("log2", 1415, 180, 150, 70, "hash")
    item("log3", 1415, 270, 150, 70, "ROI clips")

    # row 2: core, decision, motion
    frame("mot", 20, 480, 300, 380, "Motion", "orange")
    item("mot1", 45, 560, 250, 110, "Trot gait\nwatchdog")
    item("mot2", 45, 700, 250, 110, "12 joint\ndrivers")
    frame("core", 360, 480, 660, 380, "SLAM + exploration", "purple")
    pic("core_i", 380, 545, 300, 256, "img_map2.png")
    item("core1", 705, 545, 290, 85, "ICP pose")
    item("core2", 705, 650, 290, 85, "Occupancy map")
    item("core3", 705, 755, 290, 85, "Frontier goal")
    frame("dec", 1060, 480, 300, 380, "Decision", "red")
    item("dec1", 1085, 560, 250, 110, "Navigation\ngoal")
    item("dec2", 1085, 700, 250, 110, "Path +\ncommands")
    n["cloud"] = dict(x=1405, y=590, w=170, h=150, kind="ellipse", color="blue", label="Cloud\nsync", size=28,
                      bold="all", lw=1.6)

    for a, b, lab, kw in [
        ("scene", "sen", "", dict(s="r", d="l")),
        ("sen", "rob", "", dict(s="r", d="l")),
        ("rob", "und", "video", dict(s="r", d="l", ly=-22)),
        ("und", "log", "", dict(s="r", d="l")),
        ("log", "cloud", "upload", dict(s="b", d="t", lx=48)),
        ("rob", "core", "depth + odometry", dict(s="b", d=(0.79, 0), lx=6)),
        ("core", "dec", "", dict(s="r", d="l")),
        ("und", "dec", "alerts", dict(s="b", d="t", lx=40)),
        ("dec", "mot", "commands", dict(s="b", d="b", pts=[(1210, 885), (170, 885)], li=1)),
        ("mot", "scene", "moves", dict(s="t", d="b", lx=42)),
    ]:
        e.append(dict(src=a, dst=b, label=lab, size=24, lw=2.2, head=18, **kw))
    K.build(n, e, W, H, os.path.join(OUT, "fig_system_architecture"), "System architecture")
    K.FONT = font


if __name__ == "__main__":
    system()
