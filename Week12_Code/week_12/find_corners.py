#!/usr/bin/env python3
"""
find_corners.py  -  work out MANUAL_CORNERS for a maze photo automatically
===========================================================================
Both maze_solver.py and obstacle_solver.py need the four corners of the maze
frame. Clicking them works but is fiddly and easy to get slightly wrong, and a
few pixels of error shows up later as a skewed grid.

The cyan wall clips are a far better reference than the metal frame: there are
~70 of them, they sit on a perfectly regular lattice, and they are the same
feature the solver uses afterwards. This script finds them, fits the four sides
of the maze octagon, intersects those to get the frame corners, and prints a
MANUAL_CORNERS line to paste.

Usage:
    python3 find_corners.py                 # uses IMAGE_FILE from obstacle_solver
    python3 find_corners.py pic009.jpg      # or name one

Always eyeball corner_check.png afterwards. If the drawn quad does not sit on
the maze frame, fall back to clicking (set MANUAL_CORNERS = None).
"""

import sys
import math
import cv2
import numpy as np

# The lattice of posts sits inside the frame. In a correctly warped 900 px
# image the posts span 63.6..830.9, so the frame is the lattice grown by
# 63.6 / (830.9 - 63.6) = 8.29% on each side.
FRAME_MARGIN = 0.0829


def cyan_blobs(image_bgr):
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([75, 50, 35]), np.array([110, 255, 255]))
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    area = image_bgr.shape[0] * image_bgr.shape[1]
    return np.array([centroids[i] for i in range(1, count)
                     if area * 1.5e-5 <= stats[i, cv2.CC_STAT_AREA] <= area * 5e-4])


def keep_lattice(points):
    """Discard cyan-ish noise (clothing, floor tape) by keeping only blobs that
    have several neighbours at the common post spacing, then taking the largest
    connected cluster."""
    if len(points) < 8:
        return points
    dist = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
    np.fill_diagonal(dist, 1e9)
    spacing = np.median(np.sort(dist, axis=1)[:, 0])
    points = points[(dist < 1.7 * spacing).sum(axis=1) >= 3]
    if len(points) == 0:
        return points

    dist = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
    np.fill_diagonal(dist, 1e9)
    adjacent = dist < 1.7 * spacing
    seen, best = set(), []
    for i in range(len(points)):
        if i in seen:
            continue
        stack, comp = [i], []
        while stack:
            k = stack.pop()
            if k in seen:
                continue
            seen.add(k)
            comp.append(k)
            stack += list(np.where(adjacent[k])[0])
        if len(comp) > len(best):
            best = comp
    return points[best]


def _intersect(l1, l2):
    (x1, y1, x2, y2), (x3, y3, x4, y4) = l1, l2
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(d) < 1e-9:
        return None
    a = x1 * y2 - y1 * x2
    b = x3 * y4 - y3 * x4
    return ((a * (x3 - x4) - (x1 - x2) * b) / d,
            (a * (y3 - y4) - (y1 - y2) * b) / d)


def corners_from_posts(points):
    """Fit the four straight sides of the maze and intersect them.

    A bounding box is not good enough: under perspective the sides are not
    parallel, so box corners drift badly on an angled photo. Instead, reduce
    the post cloud's convex hull to the maze octagon, take its four long edges
    (the short ones are the cut corners), and intersect them.

    The four long edges are matched to top/right/bottom/left by their
    orientation, not by their order around the hull -- ordering breaks whenever
    a cut corner is short enough to be missed."""
    hull = cv2.convexHull(points.astype(np.float32)).reshape(-1, 1, 2)
    peri = cv2.arcLength(hull, True)

    octagon = None
    for eps in np.linspace(0.005, 0.05, 60):
        approx = cv2.approxPolyDP(hull, eps * peri, True).reshape(-1, 2)
        if len(approx) == 8:
            octagon = approx
            break
        if octagon is None and 4 <= len(approx) < 8:
            octagon = approx
    if octagon is None or len(octagon) < 4:
        raise SystemExit("Could not reduce the wall clips to a maze outline; "
                         "click the corners instead.")

    n = len(octagon)
    edges = []
    for i in range(n):
        a, b = octagon[i], octagon[(i + 1) % n]
        edges.append((float(np.linalg.norm(b - a)), a, b))
    edges.sort(key=lambda e: -e[0])
    long_edges = edges[:4]

    # Split into the two near-horizontal and two near-vertical sides, then use
    # position to say which is which.
    horizontal, vertical = [], []
    for length, a, b in long_edges:
        angle = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 180.0
        (horizontal if (angle < 45 or angle > 135) else vertical).append((a, b))
    if len(horizontal) != 2 or len(vertical) != 2:
        raise SystemExit("The maze outline does not look like a square; "
                         "click the corners instead.")

    horizontal.sort(key=lambda e: (e[0][1] + e[1][1]))   # smaller y = top
    vertical.sort(key=lambda e: (e[0][0] + e[1][0]))     # smaller x = left
    as_line = lambda e: (e[0][0], e[0][1], e[1][0], e[1][1])
    top, bottom = as_line(horizontal[0]), as_line(horizontal[1])
    left, right = as_line(vertical[0]), as_line(vertical[1])

    tl = _intersect(top, left)
    tr = _intersect(top, right)
    br = _intersect(bottom, right)
    bl = _intersect(bottom, left)
    if None in (tl, tr, br, bl):
        raise SystemExit("Maze sides came out parallel; click the corners instead.")

    quad = np.array([tl, tr, br, bl], dtype=float)
    c = quad.mean(axis=0)
    quad = c + (quad - c) * (1.0 + 2 * FRAME_MARGIN)   # grow lattice -> frame
    return [(int(round(x)), int(round(y))) for x, y in quad]


def self_check(raw, corners):
    """Warp with the derived corners and measure how square the fitted lattice
    comes out. This is the same test obstacle_solver.py applies, done here so a
    bad guess is caught before it is pasted anywhere."""
    import obstacle_solver as solver
    warped = solver.warp_image(raw, corners=corners)
    cv2.imwrite("corner_check_warped.png", warped)
    pts = solver._cyan_post_centres(warped)
    if len(pts) < 20:
        return None, "too few wall clips survived the warp"
    h = solver._fit_regular_axis(pts[:, 1], warped.shape[0])
    v = solver._fit_regular_axis(pts[:, 0], warped.shape[1])
    sx = float(np.mean(np.diff(v)))
    sy = float(np.mean(np.diff(h)))
    return abs(sx - sy) / max(sx, sy), f"{sx:.1f} px across vs {sy:.1f} px down"


def main():
    if len(sys.argv) > 1:
        image_file = sys.argv[1]
    else:
        import obstacle_solver
        image_file = obstacle_solver.IMAGE_FILE

    raw = cv2.imread(image_file)
    if raw is None:
        raise SystemExit(f"Cannot read '{image_file}'")

    pts = cyan_blobs(raw)
    print(f"  {len(pts)} cyan blobs detected")
    pts = keep_lattice(pts)
    print(f"  {len(pts)} of them form the maze lattice")
    if len(pts) < 25:
        raise SystemExit("Too few wall clips found. The photo may be too dark "
                         "or too small -- click the corners instead.")

    corners = corners_from_posts(pts)

    vis = raw.copy()
    for (x, y) in pts.astype(int):
        cv2.circle(vis, (x, y), 6, (0, 255, 255), -1)
    for i in range(4):
        cv2.line(vis, corners[i], corners[(i + 1) % 4], (0, 0, 255), 4)
    for k, p in enumerate(corners):
        cv2.circle(vis, p, 14, (0, 255, 0), -1)
        cv2.putText(vis, "TL TR BR BL".split()[k], (p[0] + 18, p[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)
    cv2.imwrite("corner_check.png", vis)

    skew, detail = self_check(raw, corners)
    print(f"  Self-check: fitted grid is {100*skew:.1f}% out of square ({detail})"
          if skew is not None else f"  Self-check failed: {detail}")

    print("\n  Saved corner_check.png and corner_check_warped.png.")
    if skew is None or skew > 0.02:
        print("\n  *** DO NOT USE THESE NUMBERS ***")
        print("  The grid did not come out square, which means the corners are")
        print("  off. Set MANUAL_CORNERS = None in obstacle_solver.py and click")
        print("  the four corners by hand instead -- that always works.")
        return
    print("  Grid came out square, so these corners are good. Paste this line")
    print("  into obstacle_solver.py (and maze_solver.py if you want):\n")
    print("MANUAL_CORNERS = [" + ", ".join(f"({x}, {y})" for x, y in corners) + "]")


if __name__ == "__main__":
    main()
