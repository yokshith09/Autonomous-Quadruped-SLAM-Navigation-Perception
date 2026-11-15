"""Architecture as a simple flow diagram for the slides (16:9, Times New Roman, PNG + .drawio).

Top row: scene understanding on colour video. Middle row: navigation from the scene to the map.
Bottom row: from the map back to the legs, and the robot moving in the scene closes the loop.
"""
import os

from evaluation import drawio_kit as K

OUT = os.path.join("docs", "report_phase2", "figures")


def flow():
    font, K.FONT = K.FONT, "Times New Roman"
    W, H = 1600, 800
    X = [30, 345, 660, 975, 1290]     # five columns
    Y = [40, 330, 620]                # three rows
    w, h = 280, 140
    n, e = {}, []

    def box(nid, col, row, label, color):
        n[nid] = dict(x=X[col], y=Y[row], w=w, h=h, kind="round", color=color, label=label, size=28, sub=23)

    # navigation (middle row, left to right)
    box("scene", 0, 1, "Scene\nunknown, no map", "grey")
    box("sense", 1, 1, "Sensing\nKinect depth + colour\ngyro, leg odometry", "green")
    box("per", 2, 1, "Perception\ndepth to 2D scan", "blue")
    box("slam", 3, 1, "SLAM\nICP pose, loop closure", "blue")
    box("map", 4, 1, "Mapping\noccupancy grid", "blue")
    # back to the legs (bottom row, right to left)
    box("fro", 4, 2, "Exploration\nroute waypoint\nor frontier goal", "purple")
    box("plan", 3, 2, "Planning\nA* path", "purple")
    box("loco", 2, 2, "Locomotion\ntrot gait, watchdog", "red")
    # scene understanding (top row)
    box("vid", 1, 0, "Video model\nVideoMamba + HorNet", "orange")
    box("crime", 2, 0, "Crime or not\nkind, crime type", "orange")
    box("log", 3, 0, "Logging\nmap, hash, clips", "yellow")
    n["cloud"] = dict(x=X[4] + 20, y=Y[0] - 5, w=w - 40, h=h + 10, kind="ellipse", color="white", label="Cloud\nsync",
                      size=28, bold="all")

    for a, b, kw in [("scene", "sense", {}), ("sense", "per", {}), ("per", "slam", {}), ("slam", "map", {}),
                     ("fro", "plan", dict(s="l", d="r")), ("plan", "loco", dict(s="l", d="r")),
                     ("vid", "crime", {}), ("crime", "log", {}), ("log", "cloud", {})]:
        e.append(dict(src=a, dst=b, **({"s": "r", "d": "l"} | kw), lw=2.2, head=18))
    e.append(dict(src="map", dst="fro", s="b", d="t", lw=2.2, head=18))
    e.append(dict(src="sense", dst="vid", s="t", d="b", label="colour video", size=22, lx=80, lw=2.2, head=18))
    e.append(dict(src="loco", dst="scene", s="l", d="b", pts=[(X[0] + w / 2, Y[2] + h / 2)], label="robot moves",
                  size=22, li=0, lw=2.2, head=18))
    K.build(n, e, W, H, os.path.join(OUT, "fig_architecture_flow"), "Architecture flow")
    K.FONT = font


if __name__ == "__main__":
    flow()
