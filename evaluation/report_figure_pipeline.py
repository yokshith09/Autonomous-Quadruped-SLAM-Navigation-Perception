"""Architecture as one left-to-right pipeline (slide style: grey boxes, maroon outline and arrows, Times New Roman),
with a picture inside each box where one helps.

Main row: scene to legs. Branch below: colour video to scene understanding and logging.
Arrow above: the robot's movement changes what the sensors see next (closed loop).
Pictures from evaluation/pipeline_icons.py (scan, grid, path) and earlier figure work.
"""
import os

import matplotlib.image as mpimg

from evaluation import drawio_kit as K

OUT = os.path.join("docs", "report_phase2", "figures")
MAROON = "#a3154f"


def pipeline():
    font, K.FONT = K.FONT, "Times New Roman"
    w, h, gap = 230, 250, 45
    X = [30 + i * (w + gap) for i in range(8)]
    Y = [90, 420]
    W, H = X[-1] + w + 30, Y[1] + h + 25
    n, e = {}, []

    def box(nid, col, row, title, caption="", image=None):
        x, y = X[col], Y[row]
        n[nid] = dict(x=x, y=y, w=w, h=h, kind="round", color="maroon", label="" if image else
                      title + ("\n" + caption if caption else ""), size=23, bold=False, lw=1.3)
        if not image:
            return
        n[nid + "_t"] = dict(x=x, y=y + 6, w=w, h=34, kind="text", label=title, size=23, bold=False)
        img = mpimg.imread(os.path.join(OUT, image))
        aw, ah = w - 30, 140                       # picture area
        r = img.shape[1] / img.shape[0]
        iw, ih = (aw, aw / r) if aw / r <= ah else (ah * r, ah)
        n[nid + "_i"] = dict(x=x + (w - iw) / 2, y=y + 44 + (ah - ih) / 2, w=iw, h=ih, kind="image",
                             image=os.path.join(OUT, image), border=False)
        n[nid + "_c"] = dict(x=x, y=y + 190, w=w, h=54, kind="text", label=caption, size=19, bold=False)

    box("scene", 0, 0, "Scene", "unknown, no map", "img_scene.png")
    box("sense", 1, 0, "Kinect + IMU", "colour, depth,\ngyro, leg odometry", "img_rgbd.png")
    box("per", 2, 0, "Perception", "depth to 2D scan\n(60 deg view)", "icon_scan.png")
    box("slam", 3, 0, "SLAM", "ICP pose,\nloop closure", "img_map2.png")
    box("map", 4, 0, "Mapping", "occupancy grid\n(5 cm cells)", "icon_grid.png")
    box("fro", 5, 0, "Exploration", "route or frontier\ngoal (red)", "img_map.png")
    box("plan", 6, 0, "Planning", "A* path", "icon_path.png")
    box("loco", 7, 0, "Locomotion", "trot gait,\n12 joints", "img_robot.png")
    box("vid", 1, 1, "Video model", "VideoMamba\n+ HorNet", "icon_frames.png")
    box("crime", 2, 1, "Classifier", "crime or not, kind,\ncrime type", "icon_nn.png")
    box("log", 3, 1, "Logging", "map, hash, clips")
    box("cloud", 4, 1, "Cloud sync")

    arrow = dict(color=MAROON, lw=1.8, head=14, size=19)
    row = ["scene", "sense", "per", "slam", "map", "fro", "plan", "loco"]
    for a, b in zip(row, row[1:]):
        e.append(dict(src=a, dst=b, s="r", d="l", **arrow))
    for a, b in [("vid", "crime"), ("crime", "log"), ("log", "cloud")]:
        e.append(dict(src=a, dst=b, s="r", d="l", **arrow))
    e.append(dict(src="sense", dst="vid", s="b", d="t", label="colour video", lx=-70, **arrow))
    cx0, cx7 = X[0] + w / 2, X[7] + w / 2
    e.append(dict(src="loco", dst="scene", s="t", d="t", pts=[(cx7, 40), (cx0, 40)], label="robot moves", li=1,
                  **arrow))
    K.build(n, e, W, H, os.path.join(OUT, "fig_architecture_pipeline"), "Architecture pipeline")
    K.FONT = font


if __name__ == "__main__":
    pipeline()
