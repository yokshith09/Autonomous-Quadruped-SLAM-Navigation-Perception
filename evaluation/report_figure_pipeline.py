"""Architecture as one left-to-right pipeline (slide style: grey boxes, maroon outline and arrows, Times New Roman).

Main row: scene to legs. Branch below: colour video to scene understanding and logging.
Arrow above: the robot's movement changes what the sensors see next (closed loop).
"""
import os

from evaluation import drawio_kit as K

OUT = os.path.join("docs", "report_phase2", "figures")
MAROON = "#a3154f"


def pipeline():
    font, K.FONT = K.FONT, "Times New Roman"
    w, h, gap = 200, 110, 40
    X = [30 + i * (w + gap) for i in range(8)]
    Y = [110, 330]
    W, H = X[-1] + w + 30, Y[1] + h + 30
    n, e = {}, []

    def box(nid, col, row, label):
        n[nid] = dict(x=X[col], y=Y[row], w=w, h=h, kind="round", color="maroon", label=label, size=22,
                      bold=False, lw=1.3)

    for i, (nid, lab) in enumerate([
            ("scene", "Scene\nunknown, no map"), ("sense", "Kinect + IMU\ndepth, colour,\ngyro, leg odometry"),
            ("per", "Perception\ndepth to\n2D scan"), ("slam", "SLAM\nICP pose,\nloop closure"),
            ("map", "Mapping\noccupancy grid"), ("fro", "Exploration\nroute or\nfrontier goal"),
            ("plan", "Planning\nA* path"), ("loco", "Locomotion\ntrot gait,\n12 joints")]):
        box(nid, i, 0, lab)
    box("vid", 1, 1, "Video model\nVideoMamba\n+ HorNet")
    box("crime", 2, 1, "Crime or not,\nkind of emergency,\ncrime type")
    box("log", 3, 1, "Logging\nmap, hash, clips")
    box("cloud", 4, 1, "Cloud sync")

    arrow = dict(color=MAROON, lw=1.6, head=13, size=17)
    row = ["scene", "sense", "per", "slam", "map", "fro", "plan", "loco"]
    for a, b in zip(row, row[1:]):
        e.append(dict(src=a, dst=b, s="r", d="l", **arrow))
    for a, b in [("vid", "crime"), ("crime", "log"), ("log", "cloud")]:
        e.append(dict(src=a, dst=b, s="r", d="l", **arrow))
    e.append(dict(src="sense", dst="vid", s="b", d="t", label="colour video", lx=-62, **arrow))
    cx0, cx7 = X[0] + w / 2, X[7] + w / 2
    e.append(dict(src="loco", dst="scene", s="t", d="t", pts=[(cx7, 45), (cx0, 45)], label="robot moves", li=1,
                  **arrow))
    K.build(n, e, W, H, os.path.join(OUT, "fig_architecture_pipeline"), "Architecture pipeline")
    K.FONT = font


if __name__ == "__main__":
    pipeline()
