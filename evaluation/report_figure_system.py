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
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="frame", color="none", label=title, size=32, noborder=True)

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
        # every arrow starts and ends on a picture, box or caption; anchors chosen so each arrow is straight
        ("scene_i", "sen_i", "", dict(s=(1, 0.282), d=(0, 0.5))),                    # y = 152
        ("sen_i", "rob_i", "", dict(s=(1, 0.5), d=(0, 0.34))),                       # y = 152
        ("rob_i", "und_i", "video", dict(s=(1, 0.482), d=(0, 0.5), ly=-22)),         # y = 180
        ("und_i", "log2", "", dict(s=(1, 0.675), d="l")),                            # y = 215
        ("log3", "cloud", "upload", dict(s="b", d="t", lx=48)),                      # x = 1490
        ("rob_t", "core1", "depth + odometry", dict(s=(0.5, 1), d=(0.603, 0), lx=-110)),  # x = 880
        ("core3", "dec2", "", dict(s=(1, 0.318), d=(0, 0.745))),                     # y = 782
        ("und_t", "dec1", "alerts", dict(s=(0.5, 1), d="t", lx=40)),                 # x = 1210
        ("dec2", "mot2", "commands", dict(s="b", d="b", pts=[(1210, 885), (170, 885)], li=1)),
        ("mot1", "scene_t", "moves", dict(s="t", d="b", lx=42)),                     # x = 170
    ]:
        e.append(dict(src=a, dst=b, label=lab, size=24, lw=2.2, head=18, **kw))
    K.build(n, e, W, H, os.path.join(OUT, "fig_system_architecture"), "System architecture")
    K.FONT = font


if __name__ == "__main__":
    system()
