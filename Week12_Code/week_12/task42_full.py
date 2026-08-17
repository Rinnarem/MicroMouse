#!/usr/bin/env python3
"""
task42_full.py  -  MTRN3100 Task 4.2, full-maze run (the extra 2 marks)
========================================================================
Ed #299: showing only the continuous section caps you at 2 marks. Starting at
the maze start and finishing at the maze goal, driving through the obstacle
course on the way, is worth 2 more.

That run is three stages:

    maze start
       |  heading-aware grid path  (walls, LiDAR snapping ON)
    entry cell, facing the opening
       |  continuous waypoint trajectory  (no walls, LiDAR snapping OFF)
    exit cell, facing the opening
       |  heading-aware grid path  (walls, LiDAR snapping ON)
    maze goal

This script produces all three at once. It reuses maze_solver.py for the walled
part and obstacle_solver.py for the continuous part, so neither of those files
needs editing.

The one thing it must do that plain maze_solver.py does not: the obstacle
section reads as wide-open space, so an ordinary BFS would happily route a
Cartesian path straight through it and into a cylinder. Every section cell is
therefore removed from the graph, leaving only the entry and exit cells
connected to the outside world through their single openings.

Set MAZE_START / MAZE_GOAL below from what the demonstrator gives you, then run.
"""

import cv2
import numpy as np
import math

import maze_solver as ms
import obstacle_solver as os_

# ============================================================
# CONFIGURATION  --  set these on the day
# ============================================================
MAZE_START = (6, 2, 'N')
MAZE_GOAL  = (6, 6)     # (row, col)

# Everything else (image, corners, section, entry/exit) is read from
# obstacle_solver.py so there is only ever one place to edit.
IMAGE_FILE     = os_.IMAGE_FILE
MANUAL_CORNERS = os_.MANUAL_CORNERS

N = ms.MAZE_SIZE


def section_cells():
    """Global (row, col) of every cell in the 5x5 obstacle section."""
    return {(os_.SECTION_TOP_ROW + r, os_.SECTION_LEFT_COL + c)
            for r in range(os_.SECTION_SIZE)
            for c in range(os_.SECTION_SIZE)}


def global_of(local_rc):
    return (os_.SECTION_TOP_ROW + local_rc[0], os_.SECTION_LEFT_COL + local_rc[1])


def neighbour_in(rc, direction):
    dr, dc = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}[direction]
    return (rc[0] + dr, rc[1] + dc)


def draw_full_route(warped, posts, pre_cells, waypoints_mm, post_cells,
                    start_rc, goal_rc, cylinders, mm_per_px, rect,
                    filename="full_route.png"):
    """Draw all three stages on the warped photo.

    obstacle_overlay.png only shows the continuous section, which makes it look
    like the robot never leaves the 5x5. This shows the whole run: the grid path
    in to the entry, the weave between the cylinders, and the grid path out to
    the goal."""
    img = warped.copy()
    h_walls = posts[:, 0, 1]
    v_walls = posts[0, :, 0]
    cell_px = float(np.mean(np.diff(v_walls)))

    def cell_px_xy(rc):
        r, c = rc
        return (int((v_walls[c] + v_walls[c + 1]) / 2),
                int((h_walls[r] + h_walls[r + 1]) / 2))

    def mm_px_xy(x_mm, y_mm):
        return (int(v_walls[0] + (x_mm / os_.CELL_MM) * cell_px),
                int(h_walls[0] + (y_mm / os_.CELL_MM) * cell_px))

    # 5x5 section outline
    L, T_, S = os_.SECTION_LEFT_COL, os_.SECTION_TOP_ROW, os_.SECTION_SIZE
    cv2.rectangle(img, (int(v_walls[L]), int(h_walls[T_])),
                  (int(v_walls[L + S]), int(h_walls[T_ + S])), (0, 200, 255), 3)

    # cylinders
    x0, y0 = rect[0], rect[1]
    for (cx, cy, r) in cylinders:
        cv2.circle(img, (int(x0 + cx / mm_per_px), int(y0 + cy / mm_per_px)),
                   int((os_.CYLINDER_DIAMETER_MM / 2) / mm_per_px), (0, 0, 220), 2)

    # stage 1 - grid path in (blue)
    pts = [cell_px_xy(rc) for rc in pre_cells]
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, (230, 120, 0), 5, cv2.LINE_AA)
    for p in pts:
        cv2.circle(img, p, 5, (230, 120, 0), -1)

    # stage 2 - continuous trajectory (green)
    wpts = [mm_px_xy(x, y) for x, y in waypoints_mm]
    for a, b in zip(wpts, wpts[1:]):
        cv2.line(img, a, b, (0, 210, 0), 5, cv2.LINE_AA)
    for p in wpts:
        cv2.circle(img, p, 6, (0, 160, 0), -1)

    # stage 3 - grid path out (orange)
    ppts = [cell_px_xy(rc) for rc in post_cells]
    for a, b in zip(ppts, ppts[1:]):
        cv2.line(img, a, b, (0, 165, 255), 5, cv2.LINE_AA)
    for p in ppts:
        cv2.circle(img, p, 5, (0, 165, 255), -1)

    for rc, col, lbl in ((start_rc, (255, 0, 255), "START"),
                         (goal_rc, (0, 255, 255), "GOAL")):
        p = cell_px_xy(rc)
        cv2.circle(img, p, 15, col, -1)
        cv2.circle(img, p, 15, (0, 0, 0), 2)
        cv2.putText(img, lbl, (p[0] - 26, p[1] - 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, col, 2, cv2.LINE_AA)

    legend = [("1. grid path in  (LiDAR on)", (230, 120, 0)),
              ("2. cylinders     (LiDAR off)", (0, 210, 0)),
              ("3. grid path out (LiDAR on)", (0, 165, 255))]
    for i, (text, col) in enumerate(legend):
        y = 22 + i * 26
        cv2.line(img, (14, y - 5), (44, y - 5), col, 5)
        cv2.putText(img, text, (52, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (30, 30, 30), 2, cv2.LINE_AA)

    cv2.imwrite(filename, img)
    print(f"  Saved {filename}  <- the whole run, all three stages")


def reachable_from(g, cell):
    """Every cell connected to `cell` in the (section-removed) grid graph."""
    from collections import deque
    start = cell[0] * N + cell[1]
    if start not in g.nodes:
        return set()
    seen, queue = {start}, deque([start])
    while queue:
        cur = queue.popleft()
        for nb in g.edges.get(cur, {}):
            if nb not in seen:
                seen.add(nb)
                queue.append(nb)
    return {(n // N, n % N) for n in seen}


def _grid_of(cells, mark=None):
    """Small 9x9 picture of a set of cells, for the error messages."""
    out = ["      " + " ".join(f"c{c}" for c in range(N))]
    for r in range(N):
        row = [f"  r{r} "]
        for c in range(N):
            if mark and (r, c) == mark:      row.append(" * ")
            elif (r, c) in cells:            row.append(" o ")
            elif is_blocked_cell(r, c):      row.append("   ")
            else:                            row.append(" . ")
        out.append("".join(row))
    return "\n".join(out)


def is_blocked_cell(r, c, n=N, cut=2):
    return ((r + c < cut) or (r + (n - 1 - c) < cut)
            or ((n - 1 - r) + c < cut) or ((n - 1 - r) + (n - 1 - c) < cut))


def isolate_section(g, entry_rc, exit_rc, entry_outside, exit_outside):
    """Delete the section from the grid graph, keeping only entry->outside and
    exit->outside links. After this the walled planner physically cannot route
    through the obstacle course."""
    keep = {entry_rc: entry_outside, exit_rc: exit_outside}
    for (r, c) in section_cells():
        nid = r * N + c
        if nid not in g.nodes:
            continue
        if (r, c) in keep:
            allowed = keep[(r, c)]
            allowed_id = allowed[0] * N + allowed[1]
            for nb in list(g.edges[nid]):
                if nb != allowed_id:
                    g.edges[nid].pop(nb, None)
                    g.edges[nb].pop(nid, None)
        else:
            for nb in list(g.edges[nid]):
                g.edges[nb].pop(nid, None)
            g.edges.pop(nid, None)
            g.nodes.pop(nid, None)
    return g


def main():
    sep = "=" * 66
    print(sep)
    print("MTRN3100 Task 4.2 -- full maze run through the obstacle course")
    print(sep)

    print(f"  Image       : {IMAGE_FILE}")
    print(f"  Maze start  : {MAZE_START}")
    print(f"  Maze goal   : {MAZE_GOAL}")
    if MANUAL_CORNERS is None:
        print("\n  MANUAL_CORNERS is None, so you will be asked to click the four")
        print("  corners. To stop doing that every run, either run")
        print("  'python3 find_corners.py' and paste the line it prints into")
        print("  obstacle_solver.py, or copy the line printed below after clicking.")

    print("\n[1] Reading the maze walls (via maze_solver.py)...")
    raw = cv2.imread(IMAGE_FILE)
    if raw is None:
        raise SystemExit(f"Cannot read '{IMAGE_FILE}'")
    warped = ms.warp_image(raw, corners=MANUAL_CORNERS)
    gray   = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    posts, centres = ms.make_grid(warped)
    mask = ms.make_wall_mask(gray)
    g = ms.build_graph(mask, centres)

    # maze_solver returns the lattice as a post array; posts[r][c] is
    # (v_walls[c], h_walls[r]), so pull the two axes back out of it.
    h_walls = posts[:, 0, 1]
    v_walls = posts[0, :, 0]

    # Let obstacle_solver locate the section and openings from the same photo,
    # so the two halves of the plan cannot disagree about where things are.
    mm_pp = os_.CELL_MM / float(np.mean(np.diff(v_walls)))
    cyl_px, _ = os_.detect_cylinders_px(warped, h_walls, v_walls, mm_pp)
    cyl_cells = os_.cylinder_cells(cyl_px, h_walls, v_walls)
    keep = [(p, rc) for p, rc in zip(cyl_px, cyl_cells) if not os_.is_blocked(*rc)]
    cyl_px, cyl_cells = [p for p, _ in keep], [rc for _, rc in keep]
    os_.resolve_config(warped, h_walls, v_walls, cyl_cells)
    os_.validate_config()

    # Only now are the section and openings known, so read them here rather
    # than at the top of main() where they would still be the defaults.
    entry_rc = global_of(os_.ENTRY_LOCAL)
    exit_rc  = global_of(os_.EXIT_LOCAL)
    entry_outside = neighbour_in(entry_rc, {'W': 'E', 'E': 'W',
                                            'N': 'S', 'S': 'N'}[os_.ENTRY_DIR])
    exit_outside  = neighbour_in(exit_rc, os_.EXIT_DIR)
    print(f"  Entry cell  : {entry_rc} entered heading "
          f"{os_.DIR_NAME[os_.ENTRY_DIR]} from {entry_outside}")
    print(f"  Exit cell   : {exit_rc} left heading "
          f"{os_.DIR_NAME[os_.EXIT_DIR]} into {exit_outside}")

    for name, rc in (("MAZE_START", MAZE_START[:2]), ("MAZE_GOAL", MAZE_GOAL)):
        if rc in section_cells():
            raise SystemExit(
                f"\n*** {name} {rc} is inside the obstacle section. "
                f"The demonstrator will place it outside -- re-check the value.")

    print("[2] Removing the obstacle section from the grid graph...")
    isolate_section(g, entry_rc, exit_rc, entry_outside, exit_outside)
    print(f"  {len(g.nodes)} cells remain navigable by the walled planner")

    # Removing the section splits the maze into two pockets: one that can reach
    # the entry, one reachable from the exit. The start must be in the first and
    # the goal in the second, so work out both up front and say so plainly --
    # "no route" on its own gives you nothing to act on.
    can_reach_entry = reachable_from(g, entry_rc) - {entry_rc}
    reach_from_exit = reachable_from(g, exit_rc) - {exit_rc}
    print(f"\n  {len(can_reach_entry)} cells can reach the entry; "
          f"{len(reach_from_exit)} are reachable from the exit.")

    # ---------------- stage 1: start -> entry ----------------
    print("\n[3] Pre-obstacle path...")
    sid = MAZE_START[0] * N + MAZE_START[1]
    eid = entry_rc[0] * N + entry_rc[1]
    pre = ms.bfs(g, sid, eid)
    if pre is None:
        print(f"\n*** MAZE_START {MAZE_START[:2]} cannot reach the entry cell "
              f"{entry_rc}. ***\n")
        print("  Taking the obstacle section out of the maze splits what is left")
        print("  into two halves. Your start has to be in the half that feeds the")
        print("  entry. These are the cells that do ('o' = valid, '*' = your")
        print("  current MAZE_START, '.' = navigable but cut off):\n")
        print(_grid_of(can_reach_entry, mark=MAZE_START[:2]))
        print(f"\n  Valid: {sorted(can_reach_entry)}")
        print("\n  Set MAZE_START to one of those and re-run. If the demonstrator")
        print("  gives you a start that is not in this list, the entry/exit are")
        print("  probably the other way round -- set ENTRY_OPENING = 1 in")
        print("  obstacle_solver.py and try again.")
        raise SystemExit(1)
    pre_cmd = ms.path_to_cmds(pre, s_dir=MAZE_START[2], g_dir=os_.ENTRY_DIR)
    print(f"  {len(pre)-1} moves: \"{pre_cmd}\"")

    # ---------------- stage 2: the continuous section ----------------
    print("\n[4] Continuous section (via obstacle_solver.py)...")
    section, rect, mm_per_px = os_.crop_section(warped, h_walls, v_walls)
    cylinders = os_.cylinders_to_local_mm(cyl_px, rect, mm_per_px)
    print(f"  {len(cylinders)} cylinders")
    occ = os_.build_occupancy(section, cylinders, mm_per_px)
    clear_mm = os_.clearance_map(occ)

    start_mm = os_.local_cell_centre_mm(os_.ENTRY_LOCAL)
    goal_mm  = os_.local_cell_centre_mm(os_.EXIT_LOCAL)
    for label in ("entry", "exit"):
        pt = start_mm if label == "entry" else goal_mm
        moved, off = os_.nudge_to_free(clear_mm, pt, os_.R_TRAVEL_MM)
        if moved is None:
            raise SystemExit(f"The {label} cell is blocked by a cylinder.")
        if off > 0:
            print(f"  !! cylinder in the {label} cell; shifted {off:.0f} mm")
            if label == "entry":
                start_mm = moved
            else:
                goal_mm = moved

    # Same plan-then-verify loop obstacle_solver.py uses.
    chosen = None
    for r_plan in [os_.R_TRAVEL_MM + k for k in (0, 8, 16, 24, 32, 40)]:
        try:
            path_ij = os_.astar(clear_mm, os_.mm_to_ij(*start_mm),
                                os_.mm_to_ij(*goal_mm), r_plan)
        except os_.SolverError:
            continue
        cand = [os_.ij_to_mm(p) for p in os_.simplify(path_ij, clear_mm, r_plan)]
        cand[0], cand[-1] = start_mm, goal_mm
        margin, _, _ = os_.verify_plan(cand, os_.ENTRY_DIR, os_.EXIT_DIR, clear_mm)
        if margin >= os_.SAFETY_MARGIN_MM:
            chosen = (path_ij, cand, margin)
            break
        if chosen is None or margin > chosen[2]:
            chosen = (path_ij, cand, margin)
    if chosen is None:
        raise SystemExit("No route through the obstacle section.")
    path_ij, waypoints_mm, margin = chosen
    if margin < 0:
        raise SystemExit(
            f"The robot collides with the course (overlap {-margin:.0f} mm). "
            f"Run obstacle_solver.py on its own for the full diagnosis.")
    gwp = [os_.local_mm_to_global_mm(x, y) for x, y in waypoints_mm]
    print(f"  {len(gwp)} waypoints, robot clears obstacles by {margin:.1f} mm")

    # ---------------- stage 3: exit -> goal ----------------
    print("\n[5] Post-obstacle path...")
    xid = exit_rc[0] * N + exit_rc[1]
    gid = MAZE_GOAL[0] * N + MAZE_GOAL[1]
    post = ms.bfs(g, xid, gid)
    if post is None:
        print(f"\n*** MAZE_GOAL {MAZE_GOAL} cannot be reached from the exit cell "
              f"{exit_rc}. ***\n")
        print("  These are the cells reachable once the robot leaves the section")
        print("  ('o' = valid, '*' = your current MAZE_GOAL):\n")
        print(_grid_of(reach_from_exit, mark=MAZE_GOAL))
        print(f"\n  Valid: {sorted(reach_from_exit)}")
        print("\n  Set MAZE_GOAL to one of those and re-run.")
        raise SystemExit(1)
    post_cmd = ms.path_to_cmds(post, s_dir=os_.EXIT_DIR, g_dir=None)
    print(f"  {len(post)-1} moves: \"{post_cmd}\"")

    # ---------------- output ----------------
    print("\n" + sep)
    print("PASTE INTO week_12.ino   (set TASK_NUM 4)")
    print(sep)
    print(f'#define PRE_OBSTACLE_PATH   "{pre_cmd}"')
    print(f'#define POST_OBSTACLE_PATH  "{post_cmd}"')
    print(f"#define MAZE_START_ROW      {MAZE_START[0]}")
    print(f"#define MAZE_START_COL      {MAZE_START[1]}")
    print(f"#define MAZE_START_DIR      mtrn3100::Maze::{os_.DIR_NAME[MAZE_START[2]]}")
    print("")
    print("const mtrn3100::Waypoint OBSTACLE_WAYPOINTS[] = {")
    for k, (x, y) in enumerate(gwp):
        tag = "entry" if k == 0 else ("exit" if k == len(gwp) - 1 else "")
        print(f"    {{ {x:7.1f}f, {y:7.1f}f }},{'   // ' + tag if tag else ''}")
    print("};")
    print(f"#define OBSTACLE_WAYPOINT_COUNT  {len(gwp)}")
    print(f"#define OBS_ENTRY_ROW   {entry_rc[0]}")
    print(f"#define OBS_ENTRY_COL   {entry_rc[1]}")
    print(f"#define OBS_ENTRY_DIR   mtrn3100::Maze::{os_.DIR_NAME[os_.ENTRY_DIR]}")
    print(f"#define OBS_EXIT_ROW    {exit_rc[0]}")
    print(f"#define OBS_EXIT_COL    {exit_rc[1]}")
    print(f"#define OBS_EXIT_DIR    mtrn3100::Maze::{os_.DIR_NAME[os_.EXIT_DIR]}")
    print(sep)

    os_.draw_occupancy_map(occ, clear_mm, cylinders, waypoints_mm,
                           [os_.ij_to_mm(p) for p in path_ij],
                           "occupancy_map.png")
    os_.draw_overlay(warped, rect, waypoints_mm, cylinders, mm_per_px,
                     "obstacle_overlay.png")

    # The whole run, not just the middle stage.
    draw_full_route(warped, posts,
                    [(n // N, n % N) for n in pre],
                    gwp,
                    [(n // N, n % N) for n in post],
                    MAZE_START[:2], MAZE_GOAL,
                    cylinders, mm_per_px, rect)


if __name__ == "__main__":
    main()