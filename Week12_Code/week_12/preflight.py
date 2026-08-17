#!/usr/bin/env python3
"""
preflight.py  -  check everything lines up before you upload
=============================================================
The solvers write gen_task41.h / gen_task42.h / gen_task42_full.h, and
week_12.ino includes them. So there is no copy-and-paste to get wrong.

What can still go wrong is STALENESS: changing IMAGE_FILE and forgetting to
re-run, or running one solver but not the other. This checks for that.

    python3 preflight.py

Exit code 0 if everything lines up, 1 if not.
"""

import re
import os
import sys
import time

problems, warnings = [], []


def fail(m):
    problems.append(m)
    print("  [FAIL] " + m)


def warn(m):
    warnings.append(m)
    print("  [warn] " + m)


def ok(m):
    print("  [ok]   " + m)


def header(path):
    """Read a generated header into a dict of #defines, or None if missing."""
    if not os.path.exists(path):
        return None
    s = open(path).read()
    d = {}
    for m in re.finditer(r'^#define\s+(\w+)\s+(.*)$', s, re.MULTILINE):
        d[m.group(1)] = m.group(2).split("//")[0].strip().strip('"')
    d["_mtime"] = os.path.getmtime(path)
    d["_text"] = s
    return d


def check_fresh(hdr, name, image):
    """The header must be newer than the photo it claims to come from."""
    if not os.path.exists(image):
        fail(f"{name} says it used '{image}', but that file is not in this folder")
        return
    if hdr["_mtime"] < os.path.getmtime(image):
        fail(f"{name} is OLDER than {image}. You changed the photo but did not "
             f"re-run the solver. Re-run it.")
    else:
        age = (time.time() - hdr["_mtime"]) / 60.0
        when = f"{age:.0f} min ago" if age < 120 else f"{age/60:.1f} hours ago"
        ok(f"{name} generated {when} from {image}")


def main():
    print("=" * 70)
    print("PREFLIGHT")
    print("=" * 70)

    if not os.path.exists("week_12.ino"):
        fail("week_12.ino not found - are you in the week_12 folder?")
        return summary()

    ino = open("week_12.ino").read()
    m = re.search(r"^#define TASK_NUM\s+(\d+)", ino, re.MULTILINE)
    if not m:
        fail("cannot read TASK_NUM from week_12.ino")
        return summary()
    task = int(m.group(1))

    label = {1: "4.1 maze race",
             2: "4.2 obstacle section only (2 marks)",
             3: "4.3 autonomous mapping",
             4: "4.2 full maze through the section (4 marks)"}.get(task, "???")
    print(f"\n  TASK_NUM = {task}   ->   {label}\n")

    if task == 3:
        fail("Task 4.3 is not implemented - exploreAndSolveMaze() is an empty "
             "stub. The robot will not move. Use 1, 2 or 4.")
        return summary()
    if task not in (1, 2, 4):
        fail(f"TASK_NUM {task} is not a valid task. Use 1, 2 or 4.")
        return summary()

    sys.path.insert(0, ".")
    import maze_solver as ms
    import obstacle_solver as osv

    h41 = header("gen_task41.h")
    h42 = header("gen_task42.h")
    h4f = header("gen_task42_full.h")

    # week_12.ino always compiles all three headers in, so all three must exist
    for name, h in (("gen_task41.h", h41), ("gen_task42.h", h42),
                    ("gen_task42_full.h", h4f)):
        if h is None:
            fail(f"{name} is missing - the sketch will not compile. Run the "
                 f"matching script.")
    if None in (h41, h42, h4f):
        return summary()

    # ---------------- task 1 ----------------
    if task == 1:
        img = h41.get("GEN_TASK41_IMAGE")
        if img != ms.IMAGE_FILE:
            fail(f"gen_task41.h was built from '{img}' but maze_solver.py is now "
                 f"set to '{ms.IMAGE_FILE}'. Re-run: python3 maze_solver.py")
        else:
            check_fresh(h41, "gen_task41.h", img)
        ok(f"PATH is {len(h41.get('PATH',''))} chars")
        print(f"\n  >>> place the robot in cell ({h41['START_ROW']}, "
              f"{h41['START_COL']}) facing "
              f"{h41['START_DIR'].split('::')[-1]}")

    # ---------------- tasks 2 and 4 ----------------
    if task in (2, 4):
        img = h42.get("GEN_TASK42_IMAGE")
        if img != osv.IMAGE_FILE:
            fail(f"gen_task42.h was built from '{img}' but obstacle_solver.py is "
                 f"now set to '{osv.IMAGE_FILE}'. Re-run the solver.")
        else:
            check_fresh(h42, "gen_task42.h", img)

        wp = re.findall(r"\{\s*(-?[0-9.]+)f,\s*(-?[0-9.]+)f\s*\}", h42["_text"])
        n = int(h42.get("OBSTACLE_WAYPOINT_COUNT", -1))
        if n != len(wp):
            fail(f"gen_task42.h: COUNT says {n} but the array has {len(wp)}")
        else:
            ok(f"{n} waypoints, count matches the array")

        if wp:
            er, ec = int(h42["OBS_ENTRY_ROW"]), int(h42["OBS_ENTRY_COL"])
            xr, xc = int(h42["OBS_EXIT_ROW"]), int(h42["OBS_EXIT_COL"])
            f_ = (round(float(wp[0][1]) / 180 - 0.5), round(float(wp[0][0]) / 180 - 0.5))
            l_ = (round(float(wp[-1][1]) / 180 - 0.5), round(float(wp[-1][0]) / 180 - 0.5))
            (ok if f_ == (er, ec) else fail)(
                f"first waypoint is in the entry cell {f_}" if f_ == (er, ec)
                else f"first waypoint is in {f_} but OBS_ENTRY is ({er},{ec})")
            (ok if l_ == (xr, xc) else fail)(
                f"last waypoint is in the exit cell {l_}" if l_ == (xr, xc)
                else f"last waypoint is in {l_} but OBS_EXIT is ({xr},{xc})")

        c = re.search(r"clearance\s*:\s*([0-9.]+) mm", h42["_text"])
        if c:
            v = float(c.group(1))
            (ok if v >= 12 else warn)(f"robot clears the cylinders by {v:.1f} mm"
                                      + ("" if v >= 12 else " - drive slowly"))

    # ---------------- task 4 only ----------------
    if task == 4:
        i42, i4f = h42.get("GEN_TASK42_IMAGE"), h4f.get("GEN_TASK42FULL_IMAGE")
        if i42 != i4f:
            fail(f"THE TWO HALVES OF TASK 4 ARE FROM DIFFERENT PHOTOS: "
                 f"waypoints from '{i42}', paths from '{i4f}'. "
                 f"Re-run: python3 task42_full.py")
        else:
            ok(f"waypoints and paths both come from {i42}")
        # written by the same run, so their timestamps should be seconds apart
        if abs(h42["_mtime"] - h4f["_mtime"]) > 120:
            fail("gen_task42.h and gen_task42_full.h were written more than "
                 "2 minutes apart, so they are probably from different runs. "
                 "Re-run: python3 task42_full.py")
        else:
            ok("both headers written by the same run")
        check_fresh(h4f, "gen_task42_full.h", i4f)
        for k in ("PRE_OBSTACLE_PATH", "POST_OBSTACLE_PATH"):
            if not h4f.get(k):
                fail(f"{k} is empty in gen_task42_full.h")
        print(f"\n  >>> place the robot in cell ({h4f['MAZE_START_ROW']}, "
              f"{h4f['MAZE_START_COL']}) facing "
              f"{h4f['MAZE_START_DIR'].split('::')[-1]}")

    if task == 2:
        print(f"\n  >>> place the robot in cell ({h42['OBS_ENTRY_ROW']}, "
              f"{h42['OBS_ENTRY_COL']}) facing "
              f"{h42['OBS_ENTRY_DIR'].split('::')[-1]}")

    summary()


def summary():
    print("\n" + "=" * 70)
    if problems:
        print(f"  {len(problems)} PROBLEM(S) - DO NOT UPLOAD")
        for p in problems:
            print("   - " + p)
        print("=" * 70)
        sys.exit(1)
    if warnings:
        print(f"  {len(warnings)} warning(s):")
        for w in warnings:
            print("   - " + w)
    print("  ALL CHECKS PASSED - safe to upload")
    print("=" * 70)
    sys.exit(0)


if __name__ == "__main__":
    main()
