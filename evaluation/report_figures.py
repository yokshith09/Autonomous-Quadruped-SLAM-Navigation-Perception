"""Report figures in draw.io style (PNG + editable .drawio): python -m evaluation.report_figures

  fig_navigation_pipeline   agents side by side on the message bus (they are independent, not a chain)
  fig_control_flow          one mission as a flowchart, ending with the return to the start point
  fig_recognition_model     scene-understanding network, compact (no tall column boxes, no empty margins)
  fig_system_architecture   whole system, large text; planned stages dashed
"""
import os

from evaluation.drawio_kit import build

OUT = os.path.join("docs", "report_phase2", "figures")


def navigation():
    W, H = 1500, 800
    n, e = {}, []
    n["sup"] = dict(x=420, y=20, w=660, h=96, color="purple", size=20,
                    label="Supervisor agent  (2 Hz)\nmission state, knowledge graph of rooms, event timeline")
    n["bus"] = dict(x=30, y=200, w=1440, h=76, color="grey", size=20,
                    label="Message bus (publish / subscribe)\nevery agent reads the latest message on each topic")
    agents = [("per", "Perception agent", "10 Hz", "depth frame to\n2D obstacle scan", "sensor/scan", "blue"),
              ("slam", "SLAM agent", "10 Hz", "pose from odometry,\nICP and loop closure", "pose/slam", "blue"),
              ("map", "Mapping agent", "10 Hz", "occupancy grid,\nrooms, doorways", "map/grid", "green"),
              ("fro", "Frontier agent", "1 Hz", "next goal: route\nwaypoint or frontier", "frontier/target", "green"),
              ("pla", "Planning agent", "5 Hz", "A* path on the\ninflated grid", "plan/path", "yellow"),
              ("loc", "Locomotion agent", "20 Hz", "trot gait, watchdog,\nedge check", "control/actuation", "yellow")]
    x, w, gap = 30, 222, 21.6
    for nid, name, hz, what, topic, col in agents:
        n[nid] = dict(x=x, y=360, w=w, h=200, color=col, size=19, sub=18,
                      label=f"{name}\n{hz}\n{what}\nout: {topic}")
        e.append(dict(src=nid, dst="bus", s="t", d=((x + w / 2 - 30) / 1440, 1), both=True))
        x += w + gap
    e.append(dict(src="sup", dst="bus", s="b", d=(0.5, 0), both=True))
    n["kin"] = dict(x=50, y=640, w=182, h=70, kind="para", color="white", size=18, bold=False,
                    label="Kinect depth\n+ colour frames")
    n["legs"] = dict(x=1240, y=640, w=215, h=70, kind="para", color="white", size=18, bold=False,
                     label="joint commands\nto the 12 leg joints")
    e.append(dict(src="kin", dst="per", s="t", d="b"))
    e.append(dict(src="loc", dst="legs", s="b", d="t"))
    n["note"] = dict(x=330, y=640, w=840, h=120, kind="box", color="grey", size=18, bold=False, dashed=True,
                     label="One 20 Hz scheduler runs every agent at its own rate.\nAgents never call each other: "
                           "each one only reads and writes\nmessages on the bus, so any agent can be changed or "
                           "replaced alone.\nLeg odometry and gyro reach SLAM through control/actuation.")
    build(n, e, W, H, os.path.join(OUT, "fig_navigation_pipeline"), "Navigation architecture")


def control_flow():
    W, H = 1150, 1560
    n, e = {}, []
    A, B = 400, 870          # column centres
    def box(nid, cx, y, w, h, label, kind="round", color="blue", **k):
        n[nid] = dict(x=cx - w / 2, y=y, w=w, h=h, kind=kind, color=color, label=label, size=19, sub=17, **k)
    box("start", A, 20, 220, 64, "Start", kind="term", color="green")
    box("load", A, 120, 400, 84, "Load scene profile\nmap size, operator route, return point")
    box("cal", A, 240, 400, 70, "Stand still: calibrate the gyro")
    box("read", A, 350, 420, 84, "Read sensors\ndepth frame, leg odometry, gyro", kind="para", color="white")
    box("loc", A, 470, 400, 96, "Localise\nodometry prediction, scan-to-submap\nICP, loop closure")
    box("map", A, 602, 400, 70, "Update the occupancy map")
    box("d1", A, 710, 330, 140, "Route waypoint\nleft?", kind="rhombus", color="yellow", bold="all")
    box("d2", A, 900, 330, 140, "Unexplored\nfrontier left?", kind="rhombus", color="yellow", bold="all")
    box("done", A, 1090, 400, 70, "Exploration complete", color="green")
    box("home", A, 1200, 400, 90, "Return to the start point\n(or the point given by the operator)", color="green")
    box("end", A, 1330, 320, 70, "End: map and run report saved", kind="term", color="green", bold=False)
    box("tw", B, 745, 400, 70, "Goal = next route waypoint")
    box("tf", B, 935, 400, 70, "Goal = best frontier viewpoint")
    box("plan", B, 1060, 400, 70, "Plan an A* path to the goal")
    box("walk", B, 1170, 400, 84, "Walk the path (trot gait)\ncollision and floor-edge checks")
    box("d3", B, 1290, 300, 130, "Making\nprogress?", kind="rhombus", color="yellow", bold="all")
    box("wd", B, 1460, 400, 70, "Watchdog: turn away, skip goal", color="red")
    for a, b in [("start", "load"), ("load", "cal"), ("cal", "read"), ("read", "loc"), ("loc", "map"), ("map", "d1"),
                 ("done", "home"), ("home", "end"), ("plan", "walk"), ("walk", "d3")]:
        e.append(dict(src=a, dst=b, s="b", d="t"))
    e.append(dict(src="d1", dst="tw", s="r", d="l", label="yes", size=16))
    e.append(dict(src="d1", dst="d2", s="b", d="t", label="no", size=16))
    e.append(dict(src="d2", dst="tf", s="r", d="l", label="yes", size=16))
    e.append(dict(src="d2", dst="done", s="b", d="t", label="no", size=16))
    e.append(dict(src="tw", dst="plan", s="r", d="r", pts=[(1092, 780), (1092, 1095)]))
    e.append(dict(src="tf", dst="plan", s="b", d="t"))
    e.append(dict(src="d3", dst="wd", s="b", d="t", label="no", size=16))
    # yes -> back to "Read sensors" (next 20 Hz cycle); the watchdog joins the same return line
    n["j"] = dict(x=1124, y=1354, w=2, h=2, kind="text", color="none", label="")
    e.append(dict(src="d3", dst="j", s="r", d="l", label="yes", size=16, noarrow=True))
    e.append(dict(src="wd", dst="j", s="r", d="b", pts=[(1125, 1495)], noarrow=True))
    e.append(dict(src="j", dst="read", s="t", d="r", pts=[(1125, 392)], label="next cycle (20 Hz)", size=16, li=1))
    build(n, e, W, H, os.path.join(OUT, "fig_control_flow"), "Control flow")


def recognition():
    W, H = 1760, 440
    n, e = {}, []
    y0, h = 40, 330
    def col(nid, x, w, title, color, sub):
        n[nid] = dict(x=x, y=y0, w=w, h=h, kind="text", color="none", label=title, size=21, bold="all", dy=-130)
        n[nid + "_s"] = dict(x=x + 6, y=y0 + h - 92, w=w - 12, h=80, kind="text", color="none", label=sub, size=17,
                             bold=False)
    col("in", 10, 200, "Input video", "grey", "8 frames\nB x 3 x 8 x 224 x 224")
    n["in_i"] = dict(x=45, y=y0 + 70, w=130, h=130, kind="image", image=os.path.join(OUT, "icon_frames.png"), border=False)
    col("pe", 250, 200, "3D patch\nembedding", "blue", "video to spatio-\ntemporal patches")
    n["pe_i"] = dict(x=295, y=y0 + 90, w=110, h=110, kind="image", image=os.path.join(OUT, "icon_cube.png"), border=False)
    n["hb"] = dict(x=490, y=y0, w=640, h=h, kind="frame", color="grey", dashed=True, label="VideoMamba hybrid block",
                   size=22)
    n["ln"] = dict(x=700, y=y0 + 52, w=220, h=48, color="purple", label="LayerNorm", size=18)
    n["c1"] = dict(x=515, y=y0 + 132, w=270, h=70, color="blue", label="1D Conv\nlocal token mixing", size=18)
    n["hn"] = dict(x=835, y=y0 + 132, w=270, h=70, color="yellow", label="HorNet (gnConv)\nrecursive spatial gating",
                   size=18)
    n["add"] = dict(x=788, y=y0 + 214, w=44, h=44, kind="ellipse", color="white", label="+", size=22)
    n["ssm"] = dict(x=640, y=y0 + 270, w=340, h=52, color="green", label="Mamba SSM scanner (selective scan)", size=17)
    e += [dict(src="ln", dst="c1", s="b", d="t"), dict(src="ln", dst="hn", s="b", d="t"),
          dict(src="c1", dst="add", s="b", d="l", pts=[(650, y0 + 236)]),
          dict(src="hn", dst="add", s="b", d="r", pts=[(970, y0 + 236)]),
          dict(src="add", dst="ssm", s="b", d="t")]
    col("ap", 1170, 180, "Average\npooling", "red", "sequence to vector\n(B x L x C) to (B x C)")
    n["ap_i"] = dict(x=1195, y=y0 + 95, w=130, h=98, kind="image", image=os.path.join(OUT, "icon_pool.png"), border=False)
    col("ch", 1390, 160, "Classifier\nhead", "blue", "fully connected\n+ softmax")
    n["ch_i"] = dict(x=1425, y=y0 + 85, w=90, h=136, kind="image", image=os.path.join(OUT, "icon_nn.png"), border=False)
    col("out", 1590, 160, "Output", "green", "predicted\nclass")
    n["o1"] = dict(x=1602, y=y0 + 70, w=136, h=40, kind="box", color="white", label="Abuse      20%", size=15, bold=False)
    n["o2"] = dict(x=1602, y=y0 + 118, w=136, h=40, kind="box", color="green", label="Shooting  50%", size=15, bold=True)
    n["o3"] = dict(x=1602, y=y0 + 166, w=136, h=40, kind="box", color="white", label="Robbery   10%", size=15, bold=False)
    for a, b in [("in", "pe"), ("pe", "hb"), ("hb", "ap"), ("ap", "ch"), ("ch", "out")]:
        e.append(dict(src=a, dst=b, s="r", d="l"))
    n["br1"] = dict(x=10, y=y0 + h + 22, w=1340, h=4, kind="box", color="blue")
    n["br1t"] = dict(x=10, y=y0 + h + 30, w=1340, h=36, kind="text", label="Spatio-temporal feature extraction", size=19)
    n["br2"] = dict(x=1390, y=y0 + h + 22, w=360, h=4, kind="box", color="green")
    n["br2t"] = dict(x=1390, y=y0 + h + 30, w=360, h=36, kind="text", label="Classification", size=19)
    build(n, e, W, H, os.path.join(OUT, "fig_recognition_model"), "Scene-understanding network")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    navigation()
    control_flow()
    recognition()
    from evaluation.report_figure_system import system
    system()
    print("ok")
