"""Tiny diagram kit: one description -> a PNG (matplotlib, draw.io look) and an editable .drawio file.

Coordinates are draw.io pixels (x right, y down). Node: dict(x, y, w, h, label, kind, color, size, bold, dashed,
image). kinds: box, round, term (pill), rhombus, para (parallelogram), ellipse, text, frame (labelled group), image.
Edge: dict(src, dst, s='b', d='t', pts=[(x, y), ...], label, dashed, both, color). Sides: t, b, l, r, or a (x, y)
fraction of the box.
"""
import base64
import html
import io

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Polygon, Ellipse, Rectangle  # noqa: E402

PALETTE = {  # draw.io default fills / strokes
    "blue": ("#dae8fc", "#6c8ebf"), "green": ("#d5e8d4", "#82b366"), "yellow": ("#fff2cc", "#d6b656"),
    "orange": ("#ffe6cc", "#d79b00"), "red": ("#f8cecc", "#b85450"), "purple": ("#e1d5e7", "#9673a6"),
    "grey": ("#f5f5f5", "#666666"), "white": ("#ffffff", "#000000"), "none": ("none", "none"),
    "dark": ("#ffffff", "#333333"),
}
FONT = "Arial"
SKETCH = False     # hand-drawn look: wobbly lines (matplotlib path.sketch, draw.io sketch=1)
NOFILL = False     # no background fills


def _anchor(n, side):
    x, y, w, h = n["x"], n["y"], n["w"], n["h"]
    if isinstance(side, tuple):
        return x + side[0] * w, y + side[1] * h
    return {"t": (x + w / 2, y), "b": (x + w / 2, y + h), "l": (x, y + h / 2), "r": (x + w, y + h / 2)}[side]


def _route(n1, n2, e):
    p0, p1 = _anchor(n1, e.get("s", "b")), _anchor(n2, e.get("d", "t"))
    if not e.get("pts"):   # nearly straight: snap so the arrow is straight and its head points the right way
        if abs(p1[1] - p0[1]) < 3:
            p1 = (p1[0], p0[1])
        elif abs(p1[0] - p0[0]) < 3:
            p1 = (p0[0], p1[1])
    pts = [p0] + list(e.get("pts", [])) + [p1]
    if not e.get("pts") and p0[0] != p1[0] and p0[1] != p1[1]:   # automatic orthogonal elbow
        if e.get("s", "b") in ("t", "b"):
            midy = (p0[1] + p1[1]) / 2 if e.get("d", "t") in ("t", "b") else p1[1]
            pts = [p0, (p0[0], midy), (p1[0], midy), p1] if e.get("d", "t") in ("t", "b") else [p0, (p0[0], p1[1]), p1]
        else:
            midx = (p0[0] + p1[0]) / 2 if e.get("d", "t") in ("l", "r") else p1[0]
            pts = [p0, (midx, p0[1]), (midx, p1[1]), p1] if e.get("d", "t") in ("l", "r") else [p0, (p1[0], p0[1]), p1]
    return pts


def _arrowhead(ax, a, b, color, size):
    import math
    ang = math.atan2(b[1] - a[1], b[0] - a[0])
    L, W = size, size * 0.42
    tip = b
    base = (b[0] - L * math.cos(ang), b[1] - L * math.sin(ang))
    left = (base[0] + W * math.sin(ang), base[1] - W * math.cos(ang))
    right = (base[0] - W * math.sin(ang), base[1] + W * math.cos(ang))
    ax.add_patch(Polygon([tip, left, right], closed=True, fc=color, ec=color, lw=0.8, zorder=6))


def _lines(label, bold_first):
    parts = label.split("\n")
    return parts, bold_first


def render_png(nodes, edges, W, H, path, scale=2.0):
    with plt.rc_context({"path.sketch": (1.4, 110, 2.2) if SKETCH else None}):
        _render(nodes, edges, W, H, path, scale)


def _render(nodes, edges, W, H, path, scale):
    fig = plt.figure(figsize=(W / 100 * scale / 2, H / 100 * scale / 2))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis("off")
    pt = 0.72 * scale / 2          # draw.io px -> matplotlib pt at this figure size
    for nid, n in nodes.items():
        x, y, w, h = n["x"], n["y"], n["w"], n["h"]
        fc, ec = PALETTE[n.get("color", "white")]
        if NOFILL and fc != "none":
            fc = "white"
        if n.get("noborder"):
            ec = "none"
        kind = n.get("kind", "round")
        ls = (0, (5, 3)) if n.get("dashed") else "-"
        lw = n.get("lw", 1.3)
        if kind in ("box", "round", "frame"):
            r = 0 if kind == "box" else min(10, h * 0.12) if kind == "round" else 8
            ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}", fc=fc, ec=ec, lw=lw,
                                        ls=ls, zorder=1 if kind == "frame" else 3))
        elif kind == "term":
            ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={h / 2}", fc=fc, ec=ec,
                                        lw=lw, zorder=3))
        elif kind == "rhombus":
            ax.add_patch(Polygon([(x + w / 2, y), (x + w, y + h / 2), (x + w / 2, y + h), (x, y + h / 2)], fc=fc, ec=ec,
                                 lw=lw, zorder=3))
        elif kind == "para":
            k = h * 0.35
            ax.add_patch(Polygon([(x + k, y), (x + w, y), (x + w - k, y + h), (x, y + h)], fc=fc, ec=ec, lw=lw, zorder=3))
        elif kind == "ellipse":
            ax.add_patch(Ellipse((x + w / 2, y + h / 2), w, h, fc=fc, ec=ec, lw=lw, zorder=3))
        elif kind == "image":
            img = plt.imread(n["image"])
            ax.imshow(img, extent=(x, x + w, y + h, y), zorder=4, aspect="auto")
            if n.get("border", True):
                ax.add_patch(Rectangle((x, y), w, h, fc="none", ec="#666666", lw=1.0, zorder=5))
        lab = n.get("label", "")
        if lab:
            size = n.get("size", 16) * pt
            parts = lab.split("\n")
            if kind == "frame":    # title at the top-left
                ax.text(x + 12, y + 10, parts[0], ha="left", va="top", fontsize=size, fontweight="bold",
                        family=FONT, color="#000000", zorder=7)
                continue
            ha = n.get("align", "center")
            tx = x + w / 2 if ha == "center" else x + 10
            bold_first = n.get("bold", True) and len(parts) > 1
            lh = size * 1.35 / pt     # line height in px
            sub = n["sub"] * pt if "sub" in n else size * 0.82
            nb = len(parts) if n.get("bold") == "all" else 1
            nbh = len(parts) if n.get("bold") == "all" else 1
            heights = [lh if (i < nbh and bold_first) or not bold_first else lh * sub / size for i in range(len(parts))]
            total = sum(heights)
            cy = y + h / 2 - total / 2 + n.get("dy", 0)
            for i, line in enumerate(parts):
                first = i < nb and (bold_first or (n.get("bold", True) and len(parts) == 1))
                fs = size if (first or not bold_first) else sub
                ax.text(tx, cy + heights[i] / 2, line, ha=ha, va="center", fontsize=fs,
                        fontweight="bold" if first else "normal", family=FONT, color="#000000", zorder=7)
                cy += heights[i]
    for e in edges:
        pts = _route(nodes[e["src"]], nodes[e["dst"]], e)
        col = e.get("color", "#333333")
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=col, lw=e.get("lw", 1.6), ls=(0, (5, 3)) if e.get("dashed") else "-", zorder=2,
                solid_capstyle="butt")
        if not e.get("noarrow"):
            _arrowhead(ax, pts[-2], pts[-1], col, e.get("head", 13))
        if e.get("both"):
            _arrowhead(ax, pts[1], pts[0], col, e.get("head", 13))
        if e.get("label"):
            segs = list(zip(pts[:-1], pts[1:]))
            a, b = max(segs, key=lambda s: abs(s[1][0] - s[0][0]) + abs(s[1][1] - s[0][1])) if "li" not in e else segs[e["li"]]
            lx, ly = (a[0] + b[0]) / 2 + e.get("lx", 0), (a[1] + b[1]) / 2 + e.get("ly", 0)
            ax.text(lx, ly, e["label"], ha="center", va="center", fontsize=e.get("size", 13) * pt, family=FONT,
                    color="#000000", zorder=8, bbox=dict(fc="white", ec="none", pad=1.5))
    fig.savefig(path, dpi=200, facecolor="white")
    plt.close(fig)


def _style(n):
    fc, ec = PALETTE[n.get("color", "white")]
    kind = n.get("kind", "round")
    base = {"box": "rounded=0;", "round": "rounded=1;arcSize=12;", "term": "rounded=1;arcSize=50;",
            "rhombus": "rhombus;", "para": "shape=parallelogram;perimeter=parallelogramPerimeter;fixedSize=1;",
            "ellipse": "ellipse;", "text": "text;strokeColor=none;fillColor=none;",
            "frame": "rounded=1;arcSize=4;verticalAlign=top;align=left;spacingLeft=10;container=1;",
            "image": "shape=image;imageAspect=0;aspect=fixed;"}[kind]
    if NOFILL and fc != "none":
        fc = "#ffffff"
    if n.get("noborder"):
        ec = "none"
    s = base + "whiteSpace=wrap;html=1;fontFamily=%s;fontSize=%d;" % (FONT if FONT != "Arial" else "Helvetica", n.get("size", 16))
    if SKETCH:
        s += "sketch=1;curveFitting=1;jiggle=2;"
    if kind != "text":
        s += f"fillColor={fc};strokeColor={ec};"
    if n.get("dashed"):
        s += "dashed=1;"
    if kind == "image":
        b = base64.b64encode(open(n["image"], "rb").read()).decode()
        s += "image=data:image/png," + b + ";"
    return s


def _html_label(n):
    parts = [html.escape(p) for p in n.get("label", "").split("\n")]
    if not parts or not parts[0]:
        return ""
    if n.get("bold") == "all":
        return "<b>" + "<br>".join(parts) + "</b>"
    if n.get("bold", True):
        parts[0] = f"<b>{parts[0]}</b>"
    if len(parts) > 1:
        sub = int(n.get("size", 16) * 0.82)
        return parts[0] + "<br>" + "<br>".join(f'<font style="font-size:{sub}px">{p}</font>' for p in parts[1:])
    return parts[0]


def write_drawio(nodes, edges, W, H, path, name):
    cells = ['<mxCell id="0"/>', '<mxCell id="1" parent="0"/>']
    for nid, n in nodes.items():
        cells.append(f'<mxCell id="{nid}" value="{html.escape(_html_label(n))}" style="{html.escape(_style(n))}" '
                     f'vertex="1" parent="1"><mxGeometry x="{n["x"]}" y="{n["y"]}" width="{n["w"]}" height="{n["h"]}" '
                     f'as="geometry"/></mxCell>')
    sidefrac = {"t": (0.5, 0), "b": (0.5, 1), "l": (0, 0.5), "r": (1, 0.5)}
    for i, e in enumerate(edges):
        s = sidefrac.get(e.get("s", "b"), e.get("s")) if not isinstance(e.get("s"), tuple) else e["s"]
        d = sidefrac.get(e.get("d", "t"), e.get("d")) if not isinstance(e.get("d"), tuple) else e["d"]
        st = ("edgeStyle=orthogonalEdgeStyle;rounded=0;html=1;endArrow=block;endFill=1;strokeWidth=1.5;"
              + ("sketch=1;curveFitting=1;jiggle=2;" if SKETCH else "") +
              ("fontFamily=%s;" % FONT if FONT != "Arial" else "") +
              f"fontSize={e.get('size', 13)};exitX={s[0]};exitY={s[1]};entryX={d[0]};entryY={d[1]};")
        if e.get("dashed"):
            st += "dashed=1;"
        if e.get("both"):
            st += "startArrow=block;startFill=1;"
        if e.get("noarrow"):
            st += "endArrow=none;"
        pts = "".join(f'<mxPoint x="{p[0]}" y="{p[1]}"/>' for p in e.get("pts", []))
        geo = (f'<mxGeometry relative="1" as="geometry"><Array as="points">{pts}</Array></mxGeometry>' if pts else
               '<mxGeometry relative="1" as="geometry"/>')
        cells.append(f'<mxCell id="e{i}" value="{html.escape(e.get("label", ""))}" style="{html.escape(st)}" edge="1" '
                     f'parent="1" source="{e["src"]}" target="{e["dst"]}">{geo}</mxCell>')
    xml = (f'<mxfile host="app.diagrams.net"><diagram name="{html.escape(name)}"><mxGraphModel grid="1" gridSize="10" '
           f'page="1" pageWidth="{W}" pageHeight="{H}"><root>' + "".join(cells) + "</root></mxGraphModel></diagram></mxfile>")
    open(path, "w", encoding="utf-8").write(xml)


def build(nodes, edges, W, H, out_base, name):
    render_png(nodes, edges, W, H, out_base + ".png")
    write_drawio(nodes, edges, W, H, out_base + ".drawio", name)
