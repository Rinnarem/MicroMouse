#!/usr/bin/env python3
"""
maze_solver.py  -  MTRN3100 Micromouse Maze Solver
===================================================
UPDATE ON ASSESSMENT DAY:
  Set IMAGE_FILE, MANUAL_CORNERS, START_ROW/COL/DIR and GOAL_ROW/COL.
  Start is (row, column, heading); the goal has no required heading.

Maze is 9x9 OCTAGONAL — 12 corner cells are NOT navigable (69 total).
  e.g. Example Start: (1, 5, North)   Example Goal: (7, 2, West)

Output:  warped.png  |  solved_path.png  |  Arduino command string
"""

import cv2
import numpy as np
import math
from collections import deque
from matplotlib import pyplot as plt

# ============================================================
# CONFIGURATION  — UPDATE ON ASSESSMENT DAY
# ============================================================
IMAGE_FILE = "pic005.jpg"

# Tutor gives: (Coord1=row, Coord2=col, Heading)
START_ROW = 1;  START_COL = 5;  START_DIR = 'N'   # example from slide
GOAL_ROW  = 7;  GOAL_COL  = 2
GOAL_DIR  = None  # staff clarification: face the direction used to enter the goal

MAZE_SIZE   = 9
# Assessment maze: 3 blocked squares per corner, 69 navigable squares total.
OCTAGON_CUT = 2

# Per-edge wall detection — more permissive to catch grey & shiny walls
WALL_DARK_THRESH     = 130
WALL_RATIO_THRESHOLD = 0.25   # walls ~70-100%, open gaps ~0-5%

# Canny edges catch shiny/reflective walls that appear BRIGHT in the image
USE_CANNY  = True
CANNY_LOW  = 80
CANNY_HIGH = 200

OUTPUT_SIZE = 900   # warped image side length in px

# Paste corners here after first run to skip clicking:
# MANUAL_CORNERS = [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
MANUAL_CORNERS = None

# ============================================================
# OCTAGON CORNER EXCLUSION
# ============================================================
def is_blocked(r, c, n=MAZE_SIZE, cut=OCTAGON_CUT):
    """True if (r,c) is a non-navigable corner cell."""
    return (r + c < cut) or \
           (r + (n-1-c) < cut) or \
           ((n-1-r) + c < cut) or \
           ((n-1-r) + (n-1-c) < cut)

# ============================================================
# GRAPH
# ============================================================
class Graph:
    def __init__(self):
        self.nodes = {}   # nid -> (x, y)
        self.edges = {}   # nid -> {nid: 1}

    def add_node(self, nid, x, y):
        self.nodes[nid] = (x, y)
        self.edges[nid] = {}

    def add_edge(self, a, b):
        self.edges[a][b] = 1
        self.edges[b][a] = 1

# ============================================================
# PERSPECTIVE WARP
# ============================================================
def _order_pts(pts):
    pts  = np.array(pts, dtype="float32")
    s    = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).flatten()
    return np.array([pts[np.argmin(s)], pts[np.argmin(diff)],
                     pts[np.argmax(s)], pts[np.argmax(diff)]], dtype="float32")

def warp_image(image_bgr, corners=None):
    if corners is None:
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        fig, ax = plt.subplots(figsize=(11, 11))
        ax.imshow(rgb)
        ax.set_title("Click the 4 corners of the OUTER METAL FRAME (any order)\n"
                     "then close this window", fontsize=12)
        pts = plt.ginput(4, timeout=0)
        plt.close(fig)
        if len(pts) != 4:
            raise ValueError(f"Expected 4 clicks, got {len(pts)}")
        corners = [(float(x), float(y)) for x, y in pts]
        print("*** Copy-paste into MANUAL_CORNERS to skip clicking: ***")
        print(f"MANUAL_CORNERS = {corners}")
    src = _order_pts(corners)
    dst = np.array([[0,0],[OUTPUT_SIZE-1,0],
                    [OUTPUT_SIZE-1,OUTPUT_SIZE-1],[0,OUTPUT_SIZE-1]], dtype="float32")
    return cv2.warpPerspective(image_bgr, cv2.getPerspectiveTransform(src, dst),
                               (OUTPUT_SIZE, OUTPUT_SIZE))

# ============================================================
# AUTO-DETECT WALL GRID FROM CYAN WALL CLIPS
# ============================================================
def _cyan_post_centres(image_bgr):
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([75, 50, 35]),
                       np.array([110, 255, 255]))
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    points = []
    for i in range(1, count):
        area = stats[i, cv2.CC_STAT_AREA]
        width = stats[i, cv2.CC_STAT_WIDTH]
        height = stats[i, cv2.CC_STAT_HEIGHT]
        if 40 <= area <= 600 and width <= 40 and height <= 40:
            points.append(centroids[i])
    return np.asarray(points, dtype=float)


def _fit_regular_axis(values, size, count=MAZE_SIZE + 1):
    values = np.asarray(values, dtype=float)
    best = None
    for spacing in np.linspace(size * 0.075, size * 0.115, 145):
        max_origin = size - 25 - (count - 1) * spacing
        if max_origin <= 20:
            continue
        for origin in np.linspace(20, max_origin, 180):
            grid = origin + spacing * np.arange(count)
            errors = np.min(np.abs(values[:, None] - grid[None, :]), axis=1)
            score = np.exp(-0.5 * (errors / 5.0) ** 2).sum()
            if best is None or score > best[0]:
                best = (score, grid, errors)

    if best is None or np.count_nonzero(best[2] < 9.0) < 20:
        raise ValueError("Could not fit the 9x9 lattice from the cyan maze posts")
    return best[1]


def make_grid(image_bgr, n=MAZE_SIZE):
    points = _cyan_post_centres(image_bgr)
    if len(points) < 20:
        raise ValueError(f"Only found {len(points)} cyan posts; need at least 20")

    h_walls = _fit_regular_axis(points[:, 1], image_bgr.shape[0], n + 1)
    v_walls = _fit_regular_axis(points[:, 0], image_bgr.shape[1], n + 1)
    row_ys = (h_walls[:-1] + h_walls[1:]) / 2.0
    col_xs = (v_walls[:-1] + v_walls[1:]) / 2.0
    posts = np.array([[(v_walls[c], h_walls[r]) for c in range(n + 1)]
                      for r in range(n + 1)], dtype=float)
    centres = np.array([[(col_xs[c], row_ys[r]) for c in range(n)]
                        for r in range(n)], dtype=float)
    print(f"  Grid X: {[f'{x:.1f}' for x in v_walls]}")
    print(f"  Grid Y: {[f'{y:.1f}' for y in h_walls]}")
    return posts, centres

# ============================================================
# WALL MASK  (dark pixels + Canny for shiny walls)
# ============================================================
def make_wall_mask(gray):
    dark = np.uint8(gray < WALL_DARK_THRESH) * 255
    if not USE_CANNY:
        combined = dark
    else:
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
        combined = cv2.bitwise_or(dark, edges)

    # The overhead-camera image can contain a round black puck/marker. It is
    # not a maze wall, but a pixel-only test mistakes it for one when it lies
    # across a cell boundary. Remove compact circular dark blobs; long thin
    # wall contours are deliberately retained.
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0 or not (400 <= area <= 6000):
            continue
        x, y, w, h = cv2.boundingRect(contour)
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        aspect = w / float(h)
        if 0.65 <= aspect <= 1.55 and circularity >= 0.50:
            cv2.drawContours(combined, [contour], -1, 0, thickness=-1)
    return combined > 0

# ============================================================
# WALL DETECTION  (narrow band around cell boundary midpoint)
# ============================================================
def wall_between(mask, x1, y1, x2, y2):
    dx, dy = float(x2-x1), float(y2-y1)
    L = math.hypot(dx, dy)
    if L == 0: return False
    ux, uy = dx/L, dy/L
    px, py = -dy/L, dx/L
    mx, my = (x1+x2)/2.0, (y1+y2)/2.0
    half_length = 0.32 * L

    hit = tot = 0
    for along in np.linspace(-half_length, half_length, 45):
        cx, cy = mx + px*along, my + py*along
        for across in np.linspace(-5, 5, 11):
            xi = int(round(cx + ux*across))
            yi = int(round(cy + uy*across))
            if 0 <= yi < mask.shape[0] and 0 <= xi < mask.shape[1]:
                tot += 1
                if mask[yi, xi]: hit += 1
    return (hit/tot) > WALL_RATIO_THRESHOLD if tot else False

# ============================================================
# GRAPH CONSTRUCTION  (skip blocked corner cells)
# ============================================================
def build_graph(mask, centres, n=MAZE_SIZE):
    g = Graph()
    for r in range(n):
        for c in range(n):
            if not is_blocked(r, c):
                g.add_node(r*n+c, *centres[r, c])

    open_e = 0
    for r in range(n):
        for c in range(n):
            if is_blocked(r, c): continue
            nid    = r*n+c
            x1, y1 = centres[r, c]
            if c < n-1 and not is_blocked(r, c+1):
                if not wall_between(mask, x1, y1, *centres[r, c+1]):
                    g.add_edge(nid, nid+1); open_e += 1
            if r < n-1 and not is_blocked(r+1, c):
                if not wall_between(mask, x1, y1, *centres[r+1, c]):
                    g.add_edge(nid, nid+n); open_e += 1

    nav = sum(1 for r in range(n) for c in range(n) if not is_blocked(r,c))
    print(f"  {nav} navigable cells, {open_e} open passages detected")
    return g

# ============================================================
# BFS
# ============================================================
def bfs(g, start, goal):
    if start not in g.nodes:
        print(f"  ERROR: start {start} not in graph — is it a blocked cell?"); return None
    if goal not in g.nodes:
        print(f"  ERROR: goal {goal} not in graph — is it a blocked cell?");  return None
    vis = {start: None}
    q   = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            path, node = [], goal
            while node is not None:
                path.append(node); node = vis[node]
            return list(reversed(path))
        for nb in g.edges[cur]:
            if nb not in vis:
                vis[nb] = cur; q.append(nb)
    return None

# ============================================================
# PATH → ARDUINO COMMANDS
# ============================================================
CW = ['N', 'E', 'S', 'W']

def _dir(a, b, n=MAZE_SIZE):
    dr, dc = b//n - a//n, b%n - a%n
    return 'N' if dr==-1 else 'S' if dr==1 else 'E' if dc==1 else 'W'

def path_to_cmds(path, s_dir=START_DIR, g_dir=GOAL_DIR):
    if len(path) < 2: return ""
    cmd, cur = "", s_dir
    for i in range(len(path)-1):
        need  = _dir(path[i], path[i+1])
        delta = (CW.index(need) - CW.index(cur)) % 4
        cmd  += ('f' if delta==0 else 'rf' if delta==1 else 'lf' if delta==3 else 'llf')
        cur   = need
    # final turn to face goal direction
    if g_dir and g_dir != cur:
        delta = (CW.index(g_dir) - CW.index(cur)) % 4
        cmd  += ('r' if delta==1 else 'l' if delta==3 else 'll')
    return cmd

# ============================================================
# VISUALISATION  (cv2 lines, octagonal boundary, shaded corner cells)
# ============================================================
WALL_C  = (0,   0, 200)   # red walls
PATH_C  = (0, 220,   0)   # green path line
DOT_C   = (0, 200,   0)   # green dots on path nodes
S_C     = (0, 255,   0)   # bright green START
G_C     = (0, 215, 255)   # gold GOAL
BLK_C   = (40,  40,  40)  # dark fill for blocked corner cells

def draw_result(warped, g, posts, centres, path=None,
                start_id=None, goal_id=None, n=MAZE_SIZE):
    img = warped.copy()
    cut = OCTAGON_CUT

    if cut > 0:
        # Shade blocked corner cells dark
        for r in range(n):
            for c in range(n):
                if is_blocked(r, c):
                    pt1 = (int(posts[r,   c  ][0]), int(posts[r,   c  ][1]))
                    pt2 = (int(posts[r+1, c+1][0]), int(posts[r+1, c+1][1]))
                    cv2.rectangle(img, pt1, pt2, BLK_C, -1)
        # Octagonal outer boundary (8 segments)
        bp   = [posts[0,cut], posts[0,n-cut], posts[cut,n], posts[n-cut,n],
                posts[n,n-cut], posts[n,cut], posts[n-cut,0], posts[cut,0]]
        bp_i = [(int(p[0]), int(p[1])) for p in bp]
        for i in range(8):
            cv2.line(img, bp_i[i], bp_i[(i+1)%8], WALL_C, 3)
    else:
        # Full rectangular outer boundary — all 81 cells included
        tl = (int(posts[0, 0][0]), int(posts[0, 0][1]))
        tr = (int(posts[0, n][0]), int(posts[0, n][1]))
        br = (int(posts[n, n][0]), int(posts[n, n][1]))
        bl = (int(posts[n, 0][0]), int(posts[n, 0][1]))
        cv2.line(img, tl, tr, WALL_C, 3)
        cv2.line(img, tr, br, WALL_C, 3)
        cv2.line(img, br, bl, WALL_C, 3)
        cv2.line(img, bl, tl, WALL_C, 3)

    # Interior walls — draw where no edge exists between two navigable cells
    drawn = set()
    for r in range(n):
        for c in range(n):
            if is_blocked(r, c): continue
            nid = r*n+c
            if c < n-1 and not is_blocked(r, c+1):
                k = ('v', r, c+1)
                if k not in drawn and (nid+1) not in g.edges[nid]:
                    cv2.line(img,
                             (int(posts[r,  c+1][0]), int(posts[r,  c+1][1])),
                             (int(posts[r+1,c+1][0]), int(posts[r+1,c+1][1])),
                             WALL_C, 3)
                    drawn.add(k)
            if r < n-1 and not is_blocked(r+1, c):
                k = ('h', r+1, c)
                if k not in drawn and (nid+n) not in g.edges[nid]:
                    cv2.line(img,
                             (int(posts[r+1,c  ][0]), int(posts[r+1,c  ][1])),
                             (int(posts[r+1,c+1][0]), int(posts[r+1,c+1][1])),
                             WALL_C, 3)
                    drawn.add(k)

    # Solved path
    if path and len(path) >= 2:
        for i in range(len(path)-1):
            p1 = tuple(int(v) for v in g.nodes[path[i]])
            p2 = tuple(int(v) for v in g.nodes[path[i+1]])
            cv2.line(img, p1, p2, PATH_C, 5)
        for nid in path:
            cv2.circle(img, tuple(int(v) for v in g.nodes[nid]), 8, DOT_C, -1)

    # Start / Goal markers
    for nid, col, lbl in [(start_id, S_C, "START"), (goal_id, G_C, "GOAL")]:
        if nid is not None and nid in g.nodes:
            x, y = (int(v) for v in g.nodes[nid])
            cv2.circle(img, (x, y), 14, col, -1)
            cv2.putText(img, lbl, (x+16, y+6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

    cv2.imwrite("solved_path.png", img)
    print("  Saved solved_path.png")
    plt.figure(figsize=(11,11))
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.axis('off'); plt.tight_layout(); plt.show()

# ============================================================
# MAIN
# ============================================================
def main():
    sep = "="*60
    print(sep, "\nMTRN3100 Maze Solver\n", sep)

    raw = cv2.imread(IMAGE_FILE)
    if raw is None: raise FileNotFoundError(f"Cannot read '{IMAGE_FILE}'")

    print("[1] Warping...")
    warped = warp_image(raw, corners=MANUAL_CORNERS)
    cv2.imwrite("warped.png", warped)

    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

    print("[2] Auto-detecting wall grid...")
    posts, centres = make_grid(warped)

    print(f"[3] Building wall mask (WALL_DARK_THRESH={WALL_DARK_THRESH}, CANNY={USE_CANNY})...")
    mask = make_wall_mask(gray)

    blocked = sum(1 for r in range(MAZE_SIZE) for c in range(MAZE_SIZE) if is_blocked(r,c))
    print(f"[4] Building graph ({MAZE_SIZE*MAZE_SIZE - blocked} navigable cells, {blocked} blocked)...")
    g = build_graph(mask, centres)

    start_id = START_ROW * MAZE_SIZE + START_COL
    goal_id  = GOAL_ROW  * MAZE_SIZE + GOAL_COL

    if is_blocked(START_ROW, START_COL):
        print(f"  WARNING: START ({START_ROW},{START_COL}) is a blocked corner cell!")
    if is_blocked(GOAL_ROW, GOAL_COL):
        print(f"  WARNING: GOAL ({GOAL_ROW},{GOAL_COL}) is a blocked corner cell!")

    print(f"[5] BFS: ({START_ROW},{START_COL}) → ({GOAL_ROW},{GOAL_COL})...")
    path = bfs(g, start_id, goal_id)

    if path is None:
        vis = {start_id}; q = deque([start_id])
        while q:
            cur = q.popleft()
            for nb in g.edges.get(cur, {}):
                if nb not in vis: vis.add(nb); q.append(nb)
        print(f"  No path found. Reachable from start: {len(vis)} nodes")
        print(f"  → Lower WALL_RATIO_THRESHOLD if reachable count is too low (too many walls)")
        print(f"  → Raise WALL_RATIO_THRESHOLD if too high (false open passages)")
        draw_result(warped, g, posts, centres, start_id=start_id, goal_id=goal_id)
        return

    print(f"  Path: {len(path)-1} moves → {path}")
    cmd = path_to_cmds(path)
    print(f"\n{sep}")
    print("ARDUINO COMMAND STRING:")
    print(f'  #define PATH "{cmd}"')
    print(f"  ({len(cmd)} chars)")
    print(sep)

    draw_result(warped, g, posts, centres, path=path,
                start_id=start_id, goal_id=goal_id)

if __name__ == "__main__":
    main()
