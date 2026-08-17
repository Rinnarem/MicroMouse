# Lab checklist

```
cd "$HOME/Desktop/2026_T2/MTRN3100- robot design /micromouse/week_12"
```

---

## There is no copy-and-paste any more

The Python scripts now write header files that the sketch includes:

```
maze_solver.py       ->  gen_task41.h        ->  |
obstacle_solver.py   ->  gen_task42.h        ->  |  week_12.ino
task42_full.py       ->  gen_task42.h            |  includes all three
                         gen_task42_full.h   ->  |
```

So the loop for every task is the same three steps:

1. edit the config at the top of one **Python** file
2. run it
3. set **`TASK_NUM`** in `week_12.ino` — the only line you ever edit there

Then `python3 preflight.py`, then upload.

**Never edit a `gen_*.h` by hand.** They get overwritten every run.

---

## Before you start

```
python3 preflight.py
```

Should say `ALL CHECKS PASSED`. Everything is currently set to `pic013.jpeg`
(Round 1 of the simulator) as a known-good example, so you can see a passing
run before you change anything.

Line numbers below are correct as of now, but if a file gets edited they can
drift — the variable name is what matters, and each appears exactly once.

---

## TASK 1 — 4.1 maze race

**Edit `maze_solver.py`:**

| Line | Set to |
|---|---|
| 24 | `IMAGE_FILE = "your_photo.jpg"` |
| 27 | `START_ROW = ?;  START_COL = ?;  START_DIR = '?'` |
| 28 | `GOAL_ROW  = ?;  GOAL_COL  = ?` |
| 47 | `MANUAL_CORNERS = [...]` from `find_corners.py`, or `None` to click |

Headings are `'N'` `'E'` `'S'` `'W'`.

**Run:**

```
python3 find_corners.py your_photo.jpg      # then paste the line into line 47
python3 maze_solver.py
```

Check `solved_path.png` — green line from start to goal, crossing no wall.

**In `week_12.ino`:** `#define TASK_NUM 1`

```
python3 preflight.py
```

It prints which cell to place the robot in. Upload.

---

## TASK 2 — 4.2 obstacle section only (2 marks)

Your safe result. Do this before Task 4.

**Edit `obstacle_solver.py`:**

| Line | Set to |
|---|---|
| 37 | `IMAGE_FILE = "your_photo.jpg"` |
| 41 | `MANUAL_CORNERS = [...]` from `find_corners.py` |
| 53 | `ENTRY_OPENING = 0` or `1` |

**Run:**

```
python3 find_corners.py your_photo.jpg      # then paste the line into line 41
python3 obstacle_solver.py
```

**Getting `ENTRY_OPENING` right.** The solver prints:

```
[0] local (1, 4) = global (1, 6), gap in the EAST edge
[1] local (3, 0) = global (3, 2), gap in the WEST edge
ENTRY: global (1, 6), robot enters heading WEST
```

If that ENTRY is the cell the demonstrator said you enter, it's right. If it's
the other one, flip `0` ↔ `1` and re-run. Nothing else changes.

**Check before trusting it:**

- [ ] about 5 or 6 cylinders
- [ ] yellow box in `obstacle_overlay.png` sits on the open area
- [ ] ENTRY and EXIT match what you were told
- [ ] output says **`PASS`**

**In `week_12.ino`:** `#define TASK_NUM 2`

```
python3 preflight.py
```

Place the robot in the cell preflight names, facing the heading it names.
Centre it carefully — there are no walls inside the section to correct against.

Serial Monitor at **115200 baud**.

---

## TASK 4 — 4.2 full maze (4 marks)

Only after Task 2 works.

**Edit `task42_full.py`:**

| Line | Set to |
|---|---|
| 42 | `MAZE_START = (row, col, 'N')` |
| 43 | `MAZE_GOAL  = (row, col)` |

The image and corners come from `obstacle_solver.py` — one place, no
duplication.

**Run:**

```
python3 task42_full.py
```

This writes **both** `gen_task42.h` and `gen_task42_full.h` in the same run, so
the waypoints and the paths cannot come from different photos.

If it says your start can't reach the entry, it prints a map of valid cells.
Either you mistyped the start, or `ENTRY_OPENING` is backwards.

**Check `full_route.png`** — blue in, green through the cylinders, orange out.
Blue should start at START, orange should end at GOAL.

**In `week_12.ino`:** `#define TASK_NUM 4`

```
python3 preflight.py
```

---

## TASK 3 — 4.3

`exploreAndSolveMaze()` is an empty stub. `TASK_NUM 3` prints a message and does
nothing. `preflight.py` refuses it. Skip it and put the time into 1, 2 and 4.

---

## What preflight actually checks

Not decoration — each of these is a mistake it caught during testing:

- `TASK_NUM` is readable and valid
- all three `gen_*.h` exist (missing one won't compile)
- the header's image matches what the solver is now pointed at
  → catches *changed the photo, forgot to re-run*
- the header is newer than the photo
  → catches *replaced the photo, forgot to re-run*
- waypoint count matches the array length
- the first waypoint really is in the entry cell, the last in the exit cell
- for Task 4, both headers are from the **same photo**, written by the **same
  run** → catches the exact mismatch that was in your file
- `TASK_NUM 3` is refused

---

## Order tomorrow

1. **Task 1** — exercises the drive and turn tuning everything else needs
2. **Task 2** — the guaranteed 2 marks
3. **Task 4** — the extra 2
4. Task 3 — skip

---

## If something goes wrong

| Symptom | Fix |
|---|---|
| Robot rocks back on start | `PWM_SLEW_UP` → `4.0` in `MotionController.hpp` |
| Drifts off heading in the section | `SECTION_MAX_PWM` → `140.0` in `ObstacleNavigator.hpp` |
| `grid is N% out of square` | Re-click corners — the **metal frame**, not the octagon |
| `cannot reach the entry` | Flip `ENTRY_OPENING` |
| `3 openings found` | Bad warp, re-click corners |
| Path crosses a wall in `solved_path.png` | `WALL_MIN_THICKNESS_PX` → `2` in `maze_solver.py` |
| Walls drawn where there are none | `WALL_MIN_THICKNESS_PX` → `4` |
| `robot COLLIDES` | Check overlay; if a cylinder is in the entry cell, tell the demonstrator |
| `gen_*.h is missing` | Run the matching script |
| `built from X but now set to Y` | Re-run that solver |
| `Low memory` on upload | New `Serial.print("...")` → wrap in `F(...)` |
| Robot does nothing | You're on `TASK_NUM 3` |

## Files

| File | What it is |
|---|---|
| `preflight.py` | **Run before every upload** |
| `find_corners.py` | Works out `MANUAL_CORNERS` |
| `maze_solver.py` | Task 1 → `gen_task41.h` |
| `obstacle_solver.py` | Task 2 → `gen_task42.h` |
| `task42_full.py` | Task 4 → both 4.2 headers |
| `gen_*.h` | Generated. Never edit. |
| `week_12.ino` | Only `TASK_NUM` is yours to change |
| `occupancy_map.png` | Show this for the map mark |
| `full_route.png` | Whole run, all three stages |
| `TEST_DAY_SIMULATOR.md` | Practice rounds with known answers |
