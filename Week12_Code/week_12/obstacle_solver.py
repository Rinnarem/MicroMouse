#!/usr/bin/env python3
"""
obstacle_solver.py  -  MTRN3100 Micromouse  Task 4.2  Continuous Planning
==========================================================================
The 5x5 obstacle section is an OPEN space (no internal maze walls) containing
~5-6 cylindrical obstacles of 100 mm diameter placed at RANDOM positions
(Ed #204, #270).  A Cartesian cell-to-cell path therefore cannot be used.

This solver:
  1. Warps the maze photo (same 4-corner + cyan-post pipeline as maze_solver.py)
  2. Crops the 5x5 section
  3. Builds a metric OCCUPANCY MAP of the section from dark pixels
     (cylinders AND any leftover wall stubs are treated identically)
  4. Inflates obstacles into CONFIGURATION SPACE using the robot footprint
  5. Plans a continuous route with fine-resolution 8-connected A*
  6. Simplifies the route to a few metric WAYPOINTS via line-of-sight
  7. Emits an Arduino waypoint table in global maze millimetres
  8. Renders the 5x5 occupancy map with the trajectory (this is the image
     you show the demonstrator for the 1 map mark)

ON ASSESSMENT DAY you only ever change the CONFIGURATION block below.
No CV tuning is permitted during marking (Ed #270).

Outputs:  warped_obs.png | occupancy_map.png | obstacle_overlay.png
          + Arduino waypoint table printed to stdout
"""

import cv2
import numpy as np
import math
import heapq
from matplotlib import pyplot as plt

# ============================================================
# CONFIGURATION  --  THE ONLY BLOCK YOU EDIT ON THE DAY
# ============================================================
IMAGE_FILE     = "pic014.jpg"
# Four corners of the maze frame: TL, TR, BR, BL.
#   Set to None to click them yourself when the script runs.
#   Or run  python3 find_corners.py  and paste the line it prints.
MANUAL_CORNERS = None
# --- How the section and its openings are decided.
#     AUTO_DETECT = True  -> the script works them out from the photo. This is
#                            the normal way to run it: drop in any 4.2 image,
#                            click the corners, done.
#     AUTO_DETECT = False -> the MANUAL_* values below are used instead.
AUTO_DETECT = True

# The section has exactly two openings (spec: "only ever one entrance and
# exit"). Auto-detect finds both but cannot know which way you drive through
# them, so pick: 0 or 1, in the order they are printed. If the robot comes out
# the wrong side, change this to the other value and re-run. Nothing else.
ENTRY_OPENING = 1

# Used only when AUTO_DETECT = False. An opening is (local (row, col), EDGE),
# where EDGE is the side of the 5x5 that the gap sits on.
MANUAL_SECTION_TOP_ROW  = 0
MANUAL_SECTION_LEFT_COL = 2
MANUAL_ENTRY = ((1, 4), 'E')   # pic012: gap in the EAST edge at local row 1
MANUAL_EXIT  = ((3, 0), 'W')   # pic012: gap in the WEST edge at local row 3

SECTION_SIZE = 5

# ---- Filled in at run time by resolve_config(); do not edit. --------------
SECTION_TOP_ROW  = MANUAL_SECTION_TOP_ROW
SECTION_LEFT_COL = MANUAL_SECTION_LEFT_COL
ENTRY_LOCAL, ENTRY_EDGE = MANUAL_ENTRY
EXIT_LOCAL,  EXIT_EDGE  = MANUAL_EXIT
ENTRY_DIR = 'W'   # heading the robot has as it ENTERS
EXIT_DIR  = 'W'   # heading the robot must face to LEAVE

# --- Robot footprint.  MEASURE THESE.  Under-stating them will crash the robot.
ROBOT_WIDTH_MM  = 95.0     # measured: widest point across (wheels at the back)
ROBOT_LENGTH_MM = 76.0     # measured: nose to tail
SAFETY_MARGIN_MM = 12.0    # extra padding on top of the footprint

# --- Course geometry
CELL_MM             = 180.0
CYLINDER_DIAMETER_MM = 100.0

# How big to treat each cylinder.
#   'measured' -- use the detected blob, which comes out a little larger than
#                 the real base because the camera sees some of the cylinder's
#                 side wall. Conservative, and the right default.
#   'nominal'  -- use the true 100 mm base. Switch to this only if 'measured'
#                 reports the robot does not fit AND you have checked
#                 obstacle_overlay.png and can see the circles are drawn
#                 noticeably bigger than the real cylinders.
CYLINDER_RADIUS_MODE = 'measured'

# --- Planner
PLANNER_RES_MM  = 5.0      # occupancy/planning grid resolution
CLEARANCE_BONUS = 0.55     # 0 = shortest path, higher = hugs the middle of gaps
SIMPLIFY_COMFORT_MM = 95.0 # straightened legs aim for at least this clearance

# --- Vision
DARK_THRESH     = 100      # pixels darker than this are obstacle candidates
PERIM_WALL_MM   = 12.0     # thickness of the section's own boundary walls
POST_ALLOW_MM   = 12.0     # post intrusion at each side of an opening

# ============================================================
# DERIVED FOOTPRINT RADII
# ============================================================
# Travelling in a straight line the robot sweeps its half-WIDTH.
# Rotating on the spot it sweeps its half-DIAGONAL.  Waypoints are where the
# robot turns, so they need the larger clearance; legs only need the smaller.
ROBOT_HALF_WIDTH_MM = 0.5 * ROBOT_WIDTH_MM
ROBOT_HALF_DIAG_MM  = 0.5 * math.hypot(ROBOT_WIDTH_MM, ROBOT_LENGTH_MM)

R_TRAVEL_MM = ROBOT_HALF_WIDTH_MM + SAFETY_MARGIN_MM   # hard limit along a leg
R_TURN_MM   = ROBOT_HALF_DIAG_MM  + SAFETY_MARGIN_MM   # required at a waypoint

MAZE_SIZE   = 9
OCTAGON_CUT = 2
OUTPUT_SIZE = 900
SECTION_MM  = SECTION_SIZE * CELL_MM

DIR_VEC  = {'N': (0.0, -1.0), 'E': (1.0, 0.0), 'S': (0.0, 1.0), 'W': (-1.0, 0.0)}
DIR_RAD  = {'N': 0.0, 'E': math.pi / 2.0, 'S': math.pi, 'W': -math.pi / 2.0}
DIR_NAME = {'N': 'NORTH', 'E': 'EAST', 'S': 'SOUTH', 'W': 'WEST'}


class SolverError(Exception):
    """Raised when the map or the plan cannot be trusted.  Never return a
    dangerous path silently -- there is no practice time on the day."""


# ============================================================
# 1.  PERSPECTIVE WARP   (unchanged from maze_solver.py)
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
        ax.set_title("Click the 4 corners of the OUTER METAL FRAME (any order),\n"
                     "then close this window", fontsize=12)
        pts = plt.ginput(4, timeout=0)
        plt.close(fig)
        if len(pts) != 4:
            raise SolverError(f"Expected 4 corner clicks, got {len(pts)}")
        corners = [(float(x), float(y)) for x, y in pts]
        print("*** Paste into MANUAL_CORNERS to skip clicking next time: ***")
        print(f"MANUAL_CORNERS = {corners}")
    src = _order_pts(corners)
    dst = np.array([[0, 0], [OUTPUT_SIZE - 1, 0],
                    [OUTPUT_SIZE - 1, OUTPUT_SIZE - 1], [0, OUTPUT_SIZE - 1]],
                   dtype="float32")
    return cv2.warpPerspective(image_bgr, cv2.getPerspectiveTransform(src, dst),
                               (OUTPUT_SIZE, OUTPUT_SIZE))


# ============================================================
# 2.  CYAN-POST LATTICE   (unchanged from maze_solver.py)
# ============================================================
def _cyan_post_centres(image_bgr):
    hsv  = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([75, 50, 35]), np.array([110, 255, 255]))
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    pts = []
    for i in range(1, count):
        if (40 <= stats[i, cv2.CC_STAT_AREA] <= 600
                and stats[i, cv2.CC_STAT_WIDTH] <= 40
                and stats[i, cv2.CC_STAT_HEIGHT] <= 40):
            pts.append(centroids[i])
    return np.asarray(pts, dtype=float)


def _fit_regular_axis(values, size, count=MAZE_SIZE + 1):
    values = np.asarray(values, dtype=float)
    best = None
    for spacing in np.linspace(size * 0.075, size * 0.115, 145):
        max_origin = size - 25 - (count - 1) * spacing
        if max_origin <= 20:
            continue
        for origin in np.linspace(20, max_origin, 180):
            grid   = origin + spacing * np.arange(count)
            errors = np.min(np.abs(values[:, None] - grid[None, :]), axis=1)
            score  = np.exp(-0.5 * (errors / 5.0) ** 2).sum()
            if best is None or score > best[0]:
                best = (score, grid, errors)
    if best is None or np.count_nonzero(best[2] < 9.0) < 20:
        raise SolverError("Could not fit the 9x9 lattice from the cyan posts")
    return best[1]


def make_grid(image_bgr):
    pts = _cyan_post_centres(image_bgr)
    if len(pts) < 20:
        raise SolverError(f"Only {len(pts)} cyan posts found; need >= 20")
    h_walls = _fit_regular_axis(pts[:, 1], image_bgr.shape[0])
    v_walls = _fit_regular_axis(pts[:, 0], image_bgr.shape[1])
    print(f"  Grid X: {[f'{x:.1f}' for x in v_walls]}")
    print(f"  Grid Y: {[f'{y:.1f}' for y in h_walls]}")

    # Maze cells are square, so the two spacings must agree. If they do not,
    # the four corner points were wrong and everything downstream -- cylinder
    # positions, section location, waypoints -- would be quietly distorted.
    sx = float(np.mean(np.diff(v_walls)))
    sy = float(np.mean(np.diff(h_walls)))
    skew = abs(sx - sy) / max(sx, sy)
    print(f"  Cell spacing: {sx:.1f} px across, {sy:.1f} px down "
          f"({100*skew:.1f}% difference)")
    if skew > 0.05:
        raise SolverError(
            f"The fitted grid is {100*skew:.0f}% out of square "
            f"({sx:.1f} vs {sy:.1f} px). The four corner points are wrong -- "
            f"they must be the corners of the maze's own frame, clicked in the "
            f"photo. Open warped_obs.png: the maze should fill it as a square.")
    return h_walls, v_walls


def is_blocked(r, c, n=MAZE_SIZE, cut=OCTAGON_CUT):
    """True for the 12 octagon corner cells that are not part of the maze."""
    return ((r + c < cut) or (r + (n - 1 - c) < cut)
            or ((n - 1 - r) + c < cut) or ((n - 1 - r) + (n - 1 - c) < cut))


# ============================================================
# 3.  SECTION CROP + OCCUPANCY MAP
# ============================================================
def crop_section(warped, h_walls, v_walls):
    x0 = int(round(v_walls[SECTION_LEFT_COL]))
    x1 = int(round(v_walls[SECTION_LEFT_COL + SECTION_SIZE]))
    y0 = int(round(h_walls[SECTION_TOP_ROW]))
    y1 = int(round(h_walls[SECTION_TOP_ROW + SECTION_SIZE]))
    section = warped[y0:y1, x0:x1].copy()
    # mm per pixel from the fitted lattice (do NOT assume 100 px per cell)
    mm_per_px = SECTION_MM / float(((x1 - x0) + (y1 - y0)) / 2.0)
    print(f"  Section pixel rect: x[{x0}:{x1}] y[{y0}:{y1}]  "
          f"({x1-x0}x{y1-y0} px)   {mm_per_px:.4f} mm/px")
    return section, (x0, y0, x1, y1), mm_per_px


def detect_cylinders_px(warped, h_walls, v_walls, mm_per_px):
    """Find the cylinders anywhere in the warped maze.

    Runs on the whole image, not a pre-chosen section, because the section is
    not known yet -- where the cylinders are is what tells us where it is.

    Cylinders sit at random positions and routinely straddle cell boundaries,
    so a per-cell brightness test cannot see them. Connected dark components
    filtered by size, circularity and aspect ratio can.

    Returns [(cx_px, cy_px, r_px)] and the mask used."""
    cell_px = CELL_MM / mm_per_px
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, DARK_THRESH)

    # Ignore everything outside the lattice: the metal frame, the octagon
    # corners and the floor beyond it are all dark and none of them are
    # cylinders.
    m = int(round(0.25 * cell_px))
    keep = np.zeros_like(mask)
    keep[max(0, int(h_walls[0]) - m): int(h_walls[-1]) + m,
         max(0, int(v_walls[0]) - m): int(v_walls[-1]) + m] = 255
    mask = cv2.bitwise_and(mask, keep)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        per  = cv2.arcLength(c, True)
        if per <= 0:
            continue
        _, _, bw, bh = cv2.boundingRect(c)
        circularity = 4.0 * math.pi * area / (per * per)
        aspect      = bw / float(bh) if bh else 99.0
        if (0.08 * cell_px * cell_px <= area <= 0.55 * cell_px * cell_px
                and circularity >= 0.55 and 0.60 <= aspect <= 1.60):
            (cx, cy), r = cv2.minEnclosingCircle(c)
            out.append((cx, cy, r))
    out.sort(key=lambda t: (t[1], t[0]))
    return out, mask


def cylinder_cells(cyl_px, h_walls, v_walls):
    """Which 9x9 maze cell each cylinder centre falls in."""
    cells = []
    for (cx, cy, _) in cyl_px:
        c = int(np.searchsorted(v_walls, cx) - 1)
        r = int(np.searchsorted(h_walls, cy) - 1)
        cells.append((int(np.clip(r, 0, MAZE_SIZE - 1)),
                      int(np.clip(c, 0, MAZE_SIZE - 1))))
    return cells


def cylinders_to_local_mm(cyl_px, rect, mm_per_px):
    """Convert whole-image pixel centres to section-local millimetres."""
    x0, y0, _, _ = rect
    return [((cx - x0) * mm_per_px, (cy - y0) * mm_per_px, r * mm_per_px)
            for (cx, cy, r) in cyl_px]


def opening_spans(local_rc, direction):
    """Pixel-free span (in mm along the boundary) of the opening at a cell."""
    r, c = local_rc
    if direction in ('W', 'E'):
        lo, hi = r * CELL_MM, (r + 1) * CELL_MM
    else:
        lo, hi = c * CELL_MM, (c + 1) * CELL_MM
    return lo + POST_ALLOW_MM, hi - POST_ALLOW_MM


def boundary_side(local_rc):
    """Which edge of the 5x5 a cell touches.  Returns a set (corners touch 2)."""
    r, c = local_rc
    sides = set()
    if r == 0:                 sides.add('N')
    if r == SECTION_SIZE - 1:  sides.add('S')
    if c == 0:                 sides.add('W')
    if c == SECTION_SIZE - 1:  sides.add('E')
    return sides


def build_occupancy(section_bgr, cylinders, mm_per_px):
    """Metric occupancy grid of the section at PLANNER_RES_MM.
    True = occupied.  Combines: dark pixels (cylinders + any leftover wall
    stubs), idealised cylinder discs, and the section's perimeter walls with
    gaps cut at the configured entry/exit openings."""
    n = int(round(SECTION_MM / PLANNER_RES_MM))
    occ = np.zeros((n, n), dtype=bool)

    # --- (a) raw dark pixels, resampled to the planner grid ------------------
    gray = cv2.cvtColor(section_bgr, cv2.COLOR_BGR2GRAY)
    dark = cv2.inRange(gray, 0, DARK_THRESH)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # drop the border band; perimeter walls are added analytically below so the
    # openings end up in exactly the right place
    cell_px = CELL_MM / mm_per_px
    m = int(round(0.12 * cell_px))
    dark[:m, :] = 0; dark[-m:, :] = 0; dark[:, :m] = 0; dark[:, -m:] = 0
    occ |= cv2.resize(dark, (n, n), interpolation=cv2.INTER_AREA) > 40

    # --- (b) idealised cylinder discs (nominal 100 mm, never smaller) --------
    nominal_r = CYLINDER_DIAMETER_MM / 2.0
    use_measured = (CYLINDER_RADIUS_MODE == 'measured')
    yy, xx = np.mgrid[0:n, 0:n]
    px_mm = (xx + 0.5) * PLANNER_RES_MM
    py_mm = (yy + 0.5) * PLANNER_RES_MM
    for (cx, cy, r) in cylinders:
        rad = max(nominal_r, r) if use_measured else nominal_r
        occ |= ((px_mm - cx) ** 2 + (py_mm - cy) ** 2) <= rad ** 2

    # --- (c) perimeter walls with entry/exit gaps ----------------------------
    t = max(1, int(round(PERIM_WALL_MM / PLANNER_RES_MM)))
    occ[:t, :] = True; occ[-t:, :] = True
    occ[:, :t] = True; occ[:, -t:] = True

    for local_rc, d in ((ENTRY_LOCAL, ENTRY_DIR), (EXIT_LOCAL, EXIT_DIR)):
        # the opening is on the edge the robot crosses: entering heading W means
        # it comes through the EAST edge; leaving heading W means the WEST edge
        edge = {'W': 'E', 'E': 'W', 'N': 'S', 'S': 'N'}[d] \
            if local_rc is ENTRY_LOCAL else d
        lo, hi = opening_spans(local_rc, edge)
        a = int(round(lo / PLANNER_RES_MM))
        b = int(round(hi / PLANNER_RES_MM))
        if   edge == 'N': occ[:t, a:b] = False
        elif edge == 'S': occ[-t:, a:b] = False
        elif edge == 'W': occ[:, :t][a:b, :] = False
        elif edge == 'E': occ[:, -t:][a:b, :] = False
    return occ


# ============================================================
# 4.  CONFIGURATION SPACE
# ============================================================
def clearance_map(occ):
    """Distance in millimetres from every free cell to the nearest obstacle."""
    free = np.uint8(~occ) * 255
    dist_px = cv2.distanceTransform(free, cv2.DIST_L2, 5)
    return dist_px * PLANNER_RES_MM


# ============================================================
# 5.  FINE-GRID A*
# ============================================================
def astar(clear_mm, start_ij, goal_ij, r_travel):
    n = clear_mm.shape[0]
    passable = clear_mm >= r_travel
    if not passable[start_ij]:
        raise SolverError(
            f"START is inside the inflated obstacle region "
            f"(clearance {clear_mm[start_ij]:.0f} mm < {r_travel:.0f} mm required)")
    if not passable[goal_ij]:
        raise SolverError(
            f"GOAL is inside the inflated obstacle region "
            f"(clearance {clear_mm[goal_ij]:.0f} mm < {r_travel:.0f} mm required)")

    # prefer routes that keep their distance from the cylinders
    max_c = float(clear_mm.max()) or 1.0
    penalty = CLEARANCE_BONUS * (1.0 - np.clip(clear_mm / max_c, 0, 1))

    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

    gi, gj = goal_ij
    h = lambda i, j: math.hypot(i - gi, j - gj)
    g = {start_ij: 0.0}
    parent = {start_ij: None}
    pq = [(h(*start_ij), start_ij)]
    seen = set()

    while pq:
        _, cur = heapq.heappop(pq)
        if cur in seen:
            continue
        seen.add(cur)
        if cur == goal_ij:
            path, node = [], cur
            while node is not None:
                path.append(node)
                node = parent[node]
            return path[::-1]
        ci, cj = cur
        for di, dj, step in nbrs:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < n and 0 <= nj < n) or not passable[ni, nj]:
                continue
            # no cutting a diagonal past the corner of an obstacle
            if di and dj and not (passable[ci + di, cj] and passable[ci, cj + dj]):
                continue
            ng = g[cur] + step * (1.0 + penalty[ni, nj])
            if ng < g.get((ni, nj), float('inf')):
                g[(ni, nj)] = ng
                parent[(ni, nj)] = cur
                heapq.heappush(pq, (ng + h(ni, nj), (ni, nj)))
    raise SolverError("A* found no route from entry to exit. The gaps may be "
                      "too narrow for the configured robot footprint.")


# ============================================================
# 6.  LINE-OF-SIGHT SIMPLIFICATION
# ============================================================
def _los_min_clearance(clear_mm, a, b):
    """Smallest clearance found anywhere along the straight segment a->b."""
    (ai, aj), (bi, bj) = a, b
    steps = int(max(abs(bi - ai), abs(bj - aj))) * 2 + 1
    worst = float('inf')
    for k in range(steps + 1):
        t = k / steps
        i = int(round(ai + (bi - ai) * t))
        j = int(round(aj + (bj - aj) * t))
        worst = min(worst, clear_mm[i, j])
    return worst


def simplify(path_ij, clear_mm, r_travel):
    """Greedy line-of-sight reduction.

    Straightening a route can only push it closer to an obstacle than the A*
    result, so a shortcut has to earn its place: it is accepted only if it
    keeps as much clearance as the wiggly sub-path it replaces (up to a comfort
    value beyond which extra room does not matter).

    The comparison is deliberately LOCAL. Using the whole path's minimum would
    mean one tight pinch -- typically a cylinder sitting in the entry cell --
    imposes its clearance on every other shortcut, and the route degenerates
    into hundreds of one-step legs.

    Whether the robot actually fits is settled afterwards by verify_plan(),
    which sweeps the real rectangle rather than guessing with a radius."""
    if len(path_ij) < 2:
        return list(path_ij)

    out = [path_ij[0]]
    i = 0
    while i < len(path_ij) - 1:
        best = i + 1
        for j in range(len(path_ij) - 1, i, -1):
            local_raw = float(min(clear_mm[p] for p in path_ij[i:j + 1]))
            required = max(r_travel, min(SIMPLIFY_COMFORT_MM, local_raw))
            if _los_min_clearance(clear_mm, path_ij[i], path_ij[j]) < required:
                continue
            best = j
            break
        out.append(path_ij[best])
        i = best
    return out


# ============================================================
# 6b. EXACT SWEPT-FOOTPRINT VERIFICATION
# ============================================================
def _footprint_points(cx, cy, heading, n=7):
    """Sample the robot's actual rectangle at a given pose (local mm).

    A single radius cannot describe a 96 x 120 mm rectangle: the corners sit at
    the half-diagonal but the sides only at the half-width, so an isotropic
    test is either too tight to find a route or too loose to be safe. Sampling
    the real footprint avoids the guess entirely."""
    s, c = math.sin(heading), math.cos(heading)
    fx, fy = s, -c        # forward (heading 0 = north = -y)
    rx, ry = c, s         # right
    hw, hl = ROBOT_WIDTH_MM / 2.0, ROBOT_LENGTH_MM / 2.0
    pts = []
    for a in np.linspace(-hl, hl, n):
        for b in np.linspace(-hw, hw, n):
            pts.append((cx + fx * a + rx * b, cy + fy * a + ry * b))
    return pts


def _min_clear_at(clear_mm, pts):
    n = clear_mm.shape[0]
    worst = float('inf')
    for (x, y) in pts:
        i = int(round(y / PLANNER_RES_MM - 0.5))
        j = int(round(x / PLANNER_RES_MM - 0.5))
        if not (0 <= i < n and 0 <= j < n):
            return -1.0          # off the map counts as a collision
        worst = min(worst, clear_mm[i, j])
    return worst


def verify_plan(waypoints_mm, entry_dir, exit_dir, clear_mm):
    """Drive the plan in simulation with the real rectangle and report the
    smallest gap between the robot and anything solid.

    Covers both the in-place rotations and the straight runs. A positive
    result means the robot physically fits; the number is how much room to
    spare, in millimetres."""
    heading = DIR_RAD[entry_dir]
    worst = float('inf')
    detail = []
    for k, (a, b) in enumerate(zip(waypoints_mm, waypoints_mm[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        dist = math.hypot(dx, dy)
        want = math.atan2(dx, -dy)
        turn = (want - heading + math.pi) % (2 * math.pi) - math.pi

        w_rot = float('inf')
        for t in np.linspace(0, 1, 17):
            w_rot = min(w_rot, _min_clear_at(
                clear_mm, _footprint_points(a[0], a[1], heading + t * turn)))
        w_run = float('inf')
        for t in np.linspace(0, 1, max(8, int(dist / 10))):
            w_run = min(w_run, _min_clear_at(
                clear_mm, _footprint_points(a[0] + dx * t, a[1] + dy * t, want)))

        heading = want
        detail.append((k, math.degrees(turn), dist, min(w_rot, w_run)))
        worst = min(worst, w_rot, w_run)

    final_turn = (DIR_RAD[exit_dir] - heading + math.pi) % (2 * math.pi) - math.pi
    w_fin = float('inf')
    for t in np.linspace(0, 1, 17):
        w_fin = min(w_fin, _min_clear_at(
            clear_mm, _footprint_points(waypoints_mm[-1][0], waypoints_mm[-1][1],
                                        heading + t * final_turn)))
    worst = min(worst, w_fin)
    return worst, detail, math.degrees(final_turn)


# ============================================================
# 7.  VALIDATION
# ============================================================
def validate_config():
    if not (0 <= SECTION_TOP_ROW <= MAZE_SIZE - SECTION_SIZE):
        raise SolverError("SECTION_TOP_ROW puts the section outside the maze")
    if not (0 <= SECTION_LEFT_COL <= MAZE_SIZE - SECTION_SIZE):
        raise SolverError("SECTION_LEFT_COL puts the section outside the maze")

    entry_edge = {'W': 'E', 'E': 'W', 'N': 'S', 'S': 'N'}[ENTRY_DIR]
    if entry_edge not in boundary_side(ENTRY_LOCAL):
        raise SolverError(
            f"ENTRY_LOCAL {ENTRY_LOCAL} does not touch the {entry_edge} edge, so a "
            f"robot heading {DIR_NAME[ENTRY_DIR]} cannot enter there")
    if EXIT_DIR not in boundary_side(EXIT_LOCAL):
        raise SolverError(
            f"EXIT_LOCAL {EXIT_LOCAL} does not touch the {EXIT_DIR} edge, so the "
            f"robot cannot leave heading {DIR_NAME[EXIT_DIR]}")
    if ENTRY_LOCAL == EXIT_LOCAL:
        raise SolverError("ENTRY_LOCAL and EXIT_LOCAL are the same cell")


def _wall_ratio_fn(warped, h_walls, v_walls):
    """Returns a function giving the 'wall-ness' of the boundary between two
    cell centres: ~1.0 means a solid wall, ~0.0 means an open passage."""
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    dark = np.uint8(gray < 130) * 255
    edges = cv2.Canny(cv2.GaussianBlur(gray, (3, 3), 0), 80, 200)
    comb = cv2.bitwise_or(dark, edges)
    cs, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cs:  # erase circular blobs so cylinders are not read as walls
        a, p = cv2.contourArea(c), cv2.arcLength(c, True)
        if p == 0 or not (400 <= a <= 6000):
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if 0.65 <= bw / float(bh) <= 1.55 and 4 * math.pi * a / (p * p) >= 0.50:
            cv2.drawContours(comb, [c], -1, 0, -1)
    mask = comb > 0
    cx = (v_walls[:-1] + v_walls[1:]) / 2.0
    cy = (h_walls[:-1] + h_walls[1:]) / 2.0

    def ratio(ra, ca, rb, cb):
        """Wall-ness of the boundary between maze cells (ra,ca) and (rb,cb)."""
        x1, y1, x2, y2 = cx[ca], cy[ra], cx[cb], cy[rb]
        dx, dy = float(x2 - x1), float(y2 - y1)
        L = math.hypot(dx, dy)
        ux, uy, px, py = dx / L, dy / L, -dy / L, dx / L
        mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        hit = tot = 0
        for al in np.linspace(-0.32 * L, 0.32 * L, 45):
            qx, qy = mx + px * al, my + py * al
            for ac in np.linspace(-5, 5, 11):
                xi, yi = int(round(qx + ux * ac)), int(round(qy + uy * ac))
                if 0 <= yi < mask.shape[0] and 0 <= xi < mask.shape[1]:
                    tot += 1
                    hit += mask[yi, xi]
        return hit / tot if tot else 1.0

    return ratio


OPEN_RATIO = 0.25          # below this a boundary counts as an open passage


def detect_section(cylinders_global_cells, ratio):
    """Work out where the 5x5 section is.

    Two facts pin it down. Every cylinder must be inside it, which usually
    leaves only a couple of candidate windows; and its interior is open space,
    so the winner is whichever candidate has the fewest interior walls."""
    S = SECTION_SIZE
    candidates = []
    for top in range(MAZE_SIZE - S + 1):
        for left in range(MAZE_SIZE - S + 1):
            if not all(top <= r < top + S and left <= c < left + S
                       for r, c in cylinders_global_cells):
                continue
            walls = 0
            for r in range(top, top + S):
                for c in range(left, left + S):
                    if c + 1 < left + S and ratio(r, c, r, c + 1) > OPEN_RATIO:
                        walls += 1
                    if r + 1 < top + S and ratio(r, c, r + 1, c) > OPEN_RATIO:
                        walls += 1
            # tie-break: prefer the window that sits most symmetrically around
            # the cylinders, so a stray wall stub cannot drag the answer away
            rs = [r for r, _ in cylinders_global_cells]
            cs = [c for _, c in cylinders_global_cells]
            skew = abs((min(rs) - top) - (top + S - 1 - max(rs))) + \
                   abs((min(cs) - left) - (left + S - 1 - max(cs)))
            candidates.append((walls, skew, top, left))
    if not candidates:
        raise SolverError(
            "No 5x5 window contains every detected cylinder. Either a cylinder "
            "was misdetected or the corner clicks were wrong -- check "
            "warped_obs.png before going further.")
    candidates.sort()
    walls, skew, top, left = candidates[0]
    if walls > 2:
        print(f"  !! the best 5x5 window still shows {walls} interior walls; "
              f"the section is supposed to be open. Check warped_obs.png.")
    return top, left, candidates


def detect_openings(top, left, ratio):
    """Find the gaps in the section boundary. Returns [(local (r,c), edge)]."""
    S = SECTION_SIZE
    found = []
    for lr in range(S):
        r = top + lr
        if left - 1 >= 0 and ratio(r, left, r, left - 1) < OPEN_RATIO:
            found.append(((lr, 0), 'W'))
        if left + S < MAZE_SIZE and ratio(r, left + S - 1, r, left + S) < OPEN_RATIO:
            found.append(((lr, S - 1), 'E'))
    for lc in range(S):
        c = left + lc
        if top - 1 >= 0 and ratio(top, c, top - 1, c) < OPEN_RATIO:
            found.append(((0, lc), 'N'))
        if top + S < MAZE_SIZE and ratio(top + S - 1, c, top + S, c) < OPEN_RATIO:
            found.append(((S - 1, lc), 'S'))
    return found


def resolve_config(warped, h_walls, v_walls, cylinders_global_cells):
    """Decide the section position and the entry/exit, then publish them as
    module globals so the rest of the file (and task42_full.py) can use them."""
    global SECTION_TOP_ROW, SECTION_LEFT_COL
    global ENTRY_LOCAL, ENTRY_EDGE, ENTRY_DIR, EXIT_LOCAL, EXIT_EDGE, EXIT_DIR

    ratio = _wall_ratio_fn(warped, h_walls, v_walls)
    opposite = {'W': 'E', 'E': 'W', 'N': 'S', 'S': 'N'}

    if AUTO_DETECT:
        top, left, _ = detect_section(cylinders_global_cells, ratio)
        SECTION_TOP_ROW, SECTION_LEFT_COL = top, left
        print(f"  Section detected at rows {top}-{top+SECTION_SIZE-1}, "
              f"cols {left}-{left+SECTION_SIZE-1}")

        openings = detect_openings(top, left, ratio)
        print(f"  Openings detected: {len(openings)}")
        for k, (rc, edge) in enumerate(openings):
            g = (top + rc[0], left + rc[1])
            print(f"    [{k}] local {rc} = global {g}, gap in the "
                  f"{DIR_NAME[edge]} edge")
        if len(openings) != 2:
            raise SolverError(
                f"Expected exactly 2 openings (one entrance, one exit) but "
                f"found {len(openings)}. Check warped_obs.png -- a clipped "
                f"corner or a bad warp is the usual cause.")
        if ENTRY_OPENING not in (0, 1):
            raise SolverError("ENTRY_OPENING must be 0 or 1")

        (ENTRY_LOCAL, ENTRY_EDGE) = openings[ENTRY_OPENING]
        (EXIT_LOCAL,  EXIT_EDGE)  = openings[1 - ENTRY_OPENING]
    else:
        SECTION_TOP_ROW, SECTION_LEFT_COL = (MANUAL_SECTION_TOP_ROW,
                                             MANUAL_SECTION_LEFT_COL)
        (ENTRY_LOCAL, ENTRY_EDGE) = MANUAL_ENTRY
        (EXIT_LOCAL,  EXIT_EDGE)  = MANUAL_EXIT
        print(f"  Using MANUAL config: section rows {SECTION_TOP_ROW}-"
              f"{SECTION_TOP_ROW+SECTION_SIZE-1}, cols {SECTION_LEFT_COL}-"
              f"{SECTION_LEFT_COL+SECTION_SIZE-1}")

    # Driving in through a gap in the EAST edge means heading WEST; driving out
    # through a gap in the WEST edge means heading WEST.
    ENTRY_DIR = opposite[ENTRY_EDGE]
    EXIT_DIR  = EXIT_EDGE

    eg = (SECTION_TOP_ROW + ENTRY_LOCAL[0], SECTION_LEFT_COL + ENTRY_LOCAL[1])
    xg = (SECTION_TOP_ROW + EXIT_LOCAL[0],  SECTION_LEFT_COL + EXIT_LOCAL[1])
    print(f"  ENTRY: local {ENTRY_LOCAL} = global {eg}, "
          f"robot enters heading {DIR_NAME[ENTRY_DIR]}")
    print(f"  EXIT : local {EXIT_LOCAL} = global {xg}, "
          f"robot leaves heading {DIR_NAME[EXIT_DIR]}")
    if AUTO_DETECT:
        print(f"  (if entry and exit are the wrong way round, set "
              f"ENTRY_OPENING = {1 - ENTRY_OPENING} and re-run)")


# ============================================================
# 8.  COORDINATE HELPERS
# ============================================================
def local_cell_centre_mm(local_rc):
    r, c = local_rc
    return ((c + 0.5) * CELL_MM, (r + 0.5) * CELL_MM)


def local_mm_to_global_mm(x_mm, y_mm):
    return (SECTION_LEFT_COL * CELL_MM + x_mm,
            SECTION_TOP_ROW * CELL_MM + y_mm)


def mm_to_ij(x_mm, y_mm):
    return (int(round(y_mm / PLANNER_RES_MM - 0.5)),
            int(round(x_mm / PLANNER_RES_MM - 0.5)))


def ij_to_mm(ij):
    i, j = ij
    return ((j + 0.5) * PLANNER_RES_MM, (i + 0.5) * PLANNER_RES_MM)


def nudge_to_free(clear_mm, target_mm, r_required, search_mm=200.0):
    """Move a start/goal point to the nearest spot with enough clearance.

    Ed #290 says the entry and exit cells are *typically* left empty, but that
    is a courtesy, not a guarantee -- and in practice setups a cylinder does
    sometimes land there. Rather than refuse outright, shift the point just far
    enough to clear the obstacle and report the offset, since the robot can
    still drive in through the opening and go round.

    Returns (point_mm, offset_mm), or (None, None) if nowhere near is free."""
    i0, j0 = mm_to_ij(*target_mm)
    n = clear_mm.shape[0]
    if 0 <= i0 < n and 0 <= j0 < n and clear_mm[i0, j0] >= r_required:
        return target_mm, 0.0

    rad = int(round(search_mm / PLANNER_RES_MM))
    best = None
    for di in range(-rad, rad + 1):
        for dj in range(-rad, rad + 1):
            i, j = i0 + di, j0 + dj
            if not (0 <= i < n and 0 <= j < n):
                continue
            if clear_mm[i, j] < r_required:
                continue
            d = math.hypot(di, dj) * PLANNER_RES_MM
            if best is None or d < best[0]:
                best = (d, (i, j))
    if best is None:
        return None, None
    return ij_to_mm(best[1]), best[0]


# ============================================================
# 9.  DRAWING
# ============================================================
def draw_occupancy_map(occ, clear_mm, cylinders, waypoints_mm, path_mm, filename):
    """The 5x5 occupancy map with the planned trajectory. This is the image to
    show the demonstrator (Ed #293: the 5x5 grid alone is sufficient)."""
    n = occ.shape[0]
    scale = 3
    img = np.full((n * scale, n * scale, 3), 245, np.uint8)

    # inflated (configuration-space) region the robot centre must avoid
    infl = (clear_mm < R_TRAVEL_MM) & (~occ)
    big_infl = cv2.resize(np.uint8(infl) * 255, (n * scale, n * scale),
                          interpolation=cv2.INTER_NEAREST) > 0
    img[big_infl] = (185, 200, 255)          # pale red-orange
    big_occ = cv2.resize(np.uint8(occ) * 255, (n * scale, n * scale),
                         interpolation=cv2.INTER_NEAREST) > 0
    img[big_occ] = (70, 70, 70)              # solid obstacle

    px = lambda mm: int(round(mm / PLANNER_RES_MM * scale))

    # 180 mm cell grid, for reference only -- the planner does not use it
    for k in range(SECTION_SIZE + 1):
        v = px(k * CELL_MM)
        cv2.line(img, (v, 0), (v, n * scale), (200, 200, 200), 1)
        cv2.line(img, (0, v), (n * scale, v), (200, 200, 200), 1)

    # cylinders: measured outline + nominal 100 mm circle
    for (cx, cy, r) in cylinders:
        cv2.circle(img, (px(cx), px(cy)), px(max(CYLINDER_DIAMETER_MM / 2, r)),
                   (0, 0, 0), 2)
        cv2.circle(img, (px(cx), px(cy)), 3, (0, 0, 0), -1)

    # planned trajectory
    if path_mm:
        for a, b in zip(path_mm, path_mm[1:]):
            cv2.line(img, (px(a[0]), px(a[1])), (px(b[0]), px(b[1])),
                     (120, 190, 120), 2, cv2.LINE_AA)
    for a, b in zip(waypoints_mm, waypoints_mm[1:]):
        cv2.line(img, (px(a[0]), px(a[1])), (px(b[0]), px(b[1])),
                 (0, 140, 0), 4, cv2.LINE_AA)
    for k, (x, y) in enumerate(waypoints_mm):
        cv2.circle(img, (px(x), px(y)), 7, (0, 140, 0), -1)
        cv2.putText(img, str(k), (px(x) + 10, px(y) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 90, 0), 2, cv2.LINE_AA)

    # entry / exit arrows
    ex, ey = waypoints_mm[0]
    dx, dy = DIR_VEC[ENTRY_DIR]
    cv2.arrowedLine(img, (px(ex - dx * 150), px(ey - dy * 150)), (px(ex), px(ey)),
                    (0, 170, 0), 4, tipLength=0.35)
    gx, gy = waypoints_mm[-1]
    dx, dy = DIR_VEC[EXIT_DIR]
    cv2.arrowedLine(img, (px(gx), px(gy)), (px(gx + dx * 150), px(gy + dy * 150)),
                    (0, 120, 220), 4, tipLength=0.35)
    cv2.putText(img, "ENTRY", (px(ex) - 30, px(ey) + 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 0), 2, cv2.LINE_AA)
    cv2.putText(img, "EXIT", (px(gx) - 20, px(gy) + 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 120, 220), 2, cv2.LINE_AA)

    cv2.imwrite(filename, img)
    print(f"  Saved {filename}")
    return img


def draw_overlay(warped, rect, waypoints_mm, cylinders, mm_per_px, filename):
    """The same trajectory drawn back onto the warped maze photo, as proof the
    map lines up with reality."""
    img = warped.copy()
    x0, y0, x1, y1 = rect
    cv2.rectangle(img, (x0, y0), (x1, y1), (0, 200, 255), 3)
    for (cx, cy, r) in cylinders:
        c = (int(x0 + cx / mm_per_px), int(y0 + cy / mm_per_px))
        cv2.circle(img, c, int((CYLINDER_DIAMETER_MM / 2) / mm_per_px), (0, 0, 220), 2)
    pts = [(int(x0 + x / mm_per_px), int(y0 + y / mm_per_px)) for x, y in waypoints_mm]
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, (0, 220, 0), 4, cv2.LINE_AA)
    for k, p in enumerate(pts):
        cv2.circle(img, p, 7, (0, 180, 0), -1)
        cv2.putText(img, str(k), (p[0] + 9, p[1] - 9), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 120, 0), 2, cv2.LINE_AA)
    cv2.imwrite(filename, img)
    print(f"  Saved {filename}")
    return img


# ============================================================
# 9b. ARDUINO HEADER OUTPUT
# ============================================================
def write_task42_header(global_waypoints_mm, margin_mm, filename="gen_task42.h"):
    """Write the Task 4.2 numbers as a header the sketch includes.

    week_12.ino does #include "gen_task42.h", so re-running this script is all
    it takes to update the robot -- there is no copy-and-paste step that can go
    half-done or pick up numbers from a different photo."""
    import datetime
    er, ec = ENTRY_LOCAL
    xr, xc = EXIT_LOCAL
    lines = [
        "// ===================================================================",
        "// GENERATED BY obstacle_solver.py -- DO NOT EDIT BY HAND",
        "// Re-run  python3 obstacle_solver.py  to change any of this.",
        "// ===================================================================",
        f"// image        : {IMAGE_FILE}",
        f"// generated    : {datetime.datetime.now():%Y-%m-%d %H:%M:%S}",
        f"// section      : rows {SECTION_TOP_ROW}-{SECTION_TOP_ROW+SECTION_SIZE-1}, "
        f"cols {SECTION_LEFT_COL}-{SECTION_LEFT_COL+SECTION_SIZE-1}",
        f"// robot        : {ROBOT_WIDTH_MM:.0f} x {ROBOT_LENGTH_MM:.0f} mm",
        f"// clearance    : {margin_mm:.1f} mm at the tightest point",
        "//",
        "// Waypoints are in GLOBAL maze millimetres.",
        "// x = east (column), y = south (row); cell centre = (index + 0.5) * 180",
        "#pragma once",
        "",
        f'#define GEN_TASK42_IMAGE "{IMAGE_FILE}"',
        "",
        "const mtrn3100::Waypoint OBSTACLE_WAYPOINTS[] = {",
    ]
    for k, (x, y) in enumerate(global_waypoints_mm):
        tag = ("   // entry cell" if k == 0 else
               "   // exit cell" if k == len(global_waypoints_mm) - 1 else "")
        lines.append(f"    {{ {x:8.1f}f, {y:8.1f}f }},{tag}")
    lines += [
        "};",
        f"#define OBSTACLE_WAYPOINT_COUNT  {len(global_waypoints_mm)}",
        "",
        f"#define OBS_ENTRY_ROW   {SECTION_TOP_ROW + er}"
        f"    // robot starts here for TASK_NUM 2",
        f"#define OBS_ENTRY_COL   {SECTION_LEFT_COL + ec}",
        f"#define OBS_ENTRY_DIR   mtrn3100::Maze::{DIR_NAME[ENTRY_DIR]}"
        f"    // facing this way",
        f"#define OBS_EXIT_ROW    {SECTION_TOP_ROW + xr}",
        f"#define OBS_EXIT_COL    {SECTION_LEFT_COL + xc}",
        f"#define OBS_EXIT_DIR    mtrn3100::Maze::{DIR_NAME[EXIT_DIR]}",
        "",
    ]
    with open(filename, "w") as f:
        f.write("\n".join(lines))
    print(f"  Saved {filename}")


# ============================================================
# 10.  MAIN
# ============================================================
def main():
    sep = "=" * 66
    print(sep)
    print("MTRN3100 Task 4.2 -- Continuous Planning through the obstacle course")
    print(sep)
    print(f"  Robot footprint {ROBOT_WIDTH_MM:.0f} x {ROBOT_LENGTH_MM:.0f} mm"
          f"  ->  half-width {ROBOT_HALF_WIDTH_MM:.1f} mm,"
          f" half-diagonal {ROBOT_HALF_DIAG_MM:.1f} mm")
    print(f"  Clearance required: {R_TRAVEL_MM:.1f} mm along a leg,"
          f" {R_TURN_MM:.1f} mm at a waypoint")
    print("  >>> ROBOT_LENGTH_MM is currently "
          f"{ROBOT_LENGTH_MM:.0f} mm. Measure the real chassis nose-to-tail and")
    print("      set it before the assessment -- the turn clearance depends on it.")

    raw = cv2.imread(IMAGE_FILE)
    if raw is None:
        raise SolverError(f"Cannot read '{IMAGE_FILE}'")

    print("\n[1] Warping the maze photo...")
    warped = warp_image(raw, corners=MANUAL_CORNERS)
    cv2.imwrite("warped_obs.png", warped)

    print("[2] Fitting the 9x9 lattice from the cyan posts...")
    h_walls, v_walls = make_grid(warped)
    mm_per_px = CELL_MM / float(np.mean(np.diff(v_walls)))

    print("[3] Detecting cylinders across the whole maze...")
    cyl_px, _ = detect_cylinders_px(warped, h_walls, v_walls, mm_per_px)
    cells = cylinder_cells(cyl_px, h_walls, v_walls)

    # The 12 octagon corner cells are not part of the maze, so anything
    # "found" there is the dark frame or a shadow, never a cylinder.
    kept = [(p, rc) for p, rc in zip(cyl_px, cells) if not is_blocked(*rc)]
    if len(kept) != len(cyl_px):
        dropped = [rc for p, rc in zip(cyl_px, cells) if is_blocked(*rc)]
        print(f"  ignored {len(dropped)} false detection(s) in octagon corner "
              f"cells {dropped}")
    cyl_px = [p for p, _ in kept]
    cells  = [rc for _, rc in kept]
    print(f"  {len(cyl_px)} cylinder(s) found, in maze cells {cells}")
    if not 3 <= len(cyl_px) <= 9:
        print(f"  !! WARNING: expected about 5-6 cylinders, found {len(cyl_px)}. "
              f"Check warped_obs.png before trusting this map.")
    if len(cyl_px) == 0:
        raise SolverError("No cylinders found -- is this a 4.2 photo, and did "
                          "the corner clicks land on the metal frame?")

    print("[4] Locating the 5x5 section and its openings...")
    resolve_config(warped, h_walls, v_walls, cells)
    validate_config()

    print("[5] Cropping the section...")
    section, rect, mm_per_px = crop_section(warped, h_walls, v_walls)
    cylinders = cylinders_to_local_mm(cyl_px, rect, mm_per_px)
    for k, (cx, cy, r) in enumerate(cylinders):
        gx, gy = local_mm_to_global_mm(cx, cy)
        print(f"    #{k}  local ({cx:6.1f},{cy:6.1f}) mm   "
              f"global ({gx:6.1f},{gy:6.1f}) mm   measured radius {r:4.1f} mm")

    print("[6] Building the occupancy map and configuration space...")
    occ = build_occupancy(section, cylinders, mm_per_px)
    clear_mm = clearance_map(occ)
    print(f"  Grid {occ.shape[0]}x{occ.shape[1]} @ {PLANNER_RES_MM:.0f} mm  "
          f"({100.0*occ.mean():.1f}% occupied before inflation)")

    start_mm = local_cell_centre_mm(ENTRY_LOCAL)
    goal_mm  = local_cell_centre_mm(EXIT_LOCAL)
    print(f"  Entry cell {ENTRY_LOCAL} centre  local {start_mm} mm  "
          f"clearance {clear_mm[mm_to_ij(*start_mm)]:.0f} mm")
    print(f"  Exit  cell {EXIT_LOCAL} centre  local {goal_mm} mm  "
          f"clearance {clear_mm[mm_to_ij(*goal_mm)]:.0f} mm")

    # A cylinder occasionally lands in the entry or exit cell. Shift the point
    # rather than give up; the offset is reported so the robot can be placed
    # (or driven) there.
    for label in ("entry", "exit"):
        pt = start_mm if label == "entry" else goal_mm
        moved, off = nudge_to_free(clear_mm, pt, R_TRAVEL_MM)
        if moved is None:
            raise SolverError(
                f"The {label} cell is completely blocked -- no point within "
                f"200 mm of it has {R_TRAVEL_MM:.0f} mm clearance. Check "
                f"obstacle_overlay.png; if that is really the course, the "
                f"robot cannot fit through and you should ask the demonstrator.")
        if off > 0:
            print(f"  !! a cylinder is in the {label} cell. Shifted the "
                  f"{label} point {off:.0f} mm to local "
                  f"({moved[0]:.0f}, {moved[1]:.0f}) mm to clear it.")
            if label == "entry":
                start_mm = moved
            else:
                goal_mm = moved
    start_ij, goal_ij = mm_to_ij(*start_mm), mm_to_ij(*goal_mm)

    # Plan, then check the plan by sweeping the real rectangle along it. If the
    # robot does not fit, widen the planning radius and try again -- a wider
    # radius pushes the route further from the cylinders. Planning permissively
    # and verifying exactly beats guessing a single radius up front.
    print("[7] Planning with 8-connected A*, verifying with the real footprint...")
    attempts = [R_TRAVEL_MM + k for k in (0, 8, 16, 24, 32, 40)]
    chosen = None
    for r_plan in attempts:
        try:
            path_ij = astar(clear_mm, start_ij, goal_ij, r_plan)
        except SolverError:
            print(f"  planning radius {r_plan:.0f} mm: no route")
            continue
        wp_ij = simplify(path_ij, clear_mm, r_plan)
        cand = [ij_to_mm(p) for p in wp_ij]
        cand[0], cand[-1] = start_mm, goal_mm
        margin, detail, final_deg = verify_plan(cand, ENTRY_DIR, EXIT_DIR, clear_mm)
        print(f"  planning radius {r_plan:5.0f} mm -> {len(cand):2d} waypoints, "
              f"robot clears obstacles by {margin:6.1f} mm")
        if margin >= SAFETY_MARGIN_MM:
            chosen = (r_plan, path_ij, cand, margin, detail, final_deg)
            break
        if chosen is None or margin > chosen[3]:
            chosen = (r_plan, path_ij, cand, margin, detail, final_deg)

    if chosen is None:
        raise SolverError("No route exists from the entry to the exit.")
    r_plan, path_ij, waypoints_mm, margin, detail, final_deg = chosen
    path_mm = [ij_to_mm(p) for p in path_ij]
    wp_len = sum(math.dist(a, b) for a, b in zip(waypoints_mm, waypoints_mm[1:]))
    print(f"  Using planning radius {r_plan:.0f} mm: "
          f"{len(waypoints_mm)} waypoints, {wp_len:.0f} mm of travel")

    print("\n[9] Leg-by-leg check (real 96 x 120 mm footprint swept along the plan)")
    for k, turn_deg, dist, gap in detail:
        flag = "" if gap >= SAFETY_MARGIN_MM else "   <-- TOO TIGHT"
        print(f"  leg {k}: turn {turn_deg:+7.1f} deg, drive {dist:6.1f} mm,"
              f"  robot clears by {gap:5.1f} mm{flag}")
    print(f"  final: turn {final_deg:+7.1f} deg to face {DIR_NAME[EXIT_DIR]} "
          f"and drive out")

    if margin < 0:
        raise SolverError(
            f"The robot COLLIDES with the course on this plan (overlap of "
            f"{-margin:.0f} mm). The gaps are too narrow for a "
            f"{ROBOT_WIDTH_MM:.0f} x {ROBOT_LENGTH_MM:.0f} mm chassis.\n"
            f"    Things to check, in order:\n"
            f"      1. Are ROBOT_WIDTH_MM and ROBOT_LENGTH_MM actually correct?\n"
            f"      2. Open obstacle_overlay.png. If the red circles look bigger\n"
            f"         than the real cylinders, set CYLINDER_RADIUS_MODE =\n"
            f"         'nominal' and re-run.\n"
            f"      3. If a cylinder is sitting in the entry or exit cell, tell\n"
            f"         the demonstrator -- Ed #290 says those are left clear.")
    if margin < SAFETY_MARGIN_MM:
        print(f"  !! the robot only clears the obstacles by {margin:.1f} mm, "
              f"under the {SAFETY_MARGIN_MM:.0f} mm you asked for.")
        print(f"     It does physically fit, but there is no room for odometry "
              f"drift. Drive it slowly, or ask about the cylinder spacing.")
    else:
        print(f"  PASS: the robot clears everything by at least {margin:.1f} mm.")

    print("\n[10] Rendering...")
    draw_occupancy_map(occ, clear_mm, cylinders, waypoints_mm, path_mm,
                       "occupancy_map.png")
    draw_overlay(warped, rect, waypoints_mm, cylinders, mm_per_px,
                 "obstacle_overlay.png")

    # ---------------- Arduino output ----------------
    # Written straight to a header the sketch includes. Copying numbers by hand
    # is how you end up with waypoints from one photo and paths from another,
    # so there is nothing here to copy.
    gwp = [local_mm_to_global_mm(x, y) for x, y in waypoints_mm]
    write_task42_header(gwp, margin)
    print("\n" + sep)
    print("  gen_task42.h written. NOTHING TO COPY.")
    print(sep)
    print(f"  Set TASK_NUM to 2 in week_12.ino, then upload.")
    print(f"  Place the robot in cell "
          f"({SECTION_TOP_ROW + ENTRY_LOCAL[0]}, "
          f"{SECTION_LEFT_COL + ENTRY_LOCAL[1]}) facing "
          f"{DIR_NAME[ENTRY_DIR]}.")
    print(sep)

    fig, axes = plt.subplots(1, 2, figsize=(17, 8.5))
    axes[0].imshow(cv2.cvtColor(cv2.imread("occupancy_map.png"), cv2.COLOR_BGR2RGB))
    axes[0].set_title("5x5 occupancy map + planned trajectory")
    axes[1].imshow(cv2.cvtColor(cv2.imread("obstacle_overlay.png"), cv2.COLOR_BGR2RGB))
    axes[1].set_title("Same trajectory on the warped maze")
    for a in axes:
        a.axis('off')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except SolverError as e:
        print("\n*** SOLVER STOPPED ***")
        print(f"    {e}")
        print("    No path has been produced. Fix the configuration and re-run.")
        raise SystemExit(1)
