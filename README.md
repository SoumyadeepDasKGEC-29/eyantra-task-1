# Task 1A — Ackermann Steering

Fill in `ackermann_wheel_angles(delta)` in `ackermann_steering.py`.

```sh
conda activate NV_<Team-ID>
cd ~/eYRC_26-27_Niti-Vahan/task1a
python ackermann_steering.py
```

No simulator needed — this subtask is pure geometry. Running the file executes
the test block at the bottom, which sweeps `delta` from -0.35 to 0.35 rad and
prints the pair of wheel angles your function returns for each.

| Constant | Value | Meaning |
|---|---:|---|
| `WHEELBASE` | 0.120 m | front axle to rear axle |
| `TRACK_WIDTH` | 0.110 m | left wheel centre to right wheel centre |
| `WHEEL_OFFSET` | 0.0275 m | kingpin axis to wheel centre |

Read these from the constants — do not hardcode the numbers.

**To submit:** rename your file to `NV_Task1A.py` and upload it. See the
Submission page in the theme book.

# eYRC 2026-27 · Niti Vahan (NV) — Task 1

Everything you need for Task 1 is here, clips included — there is nothing
further to download. Full instructions are in the theme book; this file is
just the map.

The Task 1C clips make this about 50 MB, so the clone takes a moment.

```text
.
├── task1a/
│   └── ackermann_steering.py     boilerplate — fill in ackermann_wheel_angles()
├── task1b/
│   ├── path_tracking.py          boilerplate — fill in ackermann_wheel_angles() and compute_steering()
│   └── Task_1B.ttt               the CoppeliaSim scene, open this before running
└── task1c/
    ├── lane_detection.py         boilerplate — fill in detect_lane()
    └── public/                   the 20 video clips, already here
```

## Before you start

Everything runs inside the `NV_<Team-ID>` conda environment you created in
Task 0 — Python 3.10, NumPy, OpenCV and the CoppeliaSim remote-API client.
Activate it in every terminal you use:

```sh
conda activate NV_<Team-ID>
```

Check it has what Task 1 needs:

```sh
python -c "import numpy, cv2, coppeliasim_zmqremoteapi_client; print('ok')"
```

If that fails, redo [Task 0](https://themes.e-yantra.org/Niti-Vahan_2627/) —
do not install packages into `base`.

## What each subtask needs

| Subtask | Needs CoppeliaSim? | Run it with |
|---|:---:|---|
| **1A** | no | `python ackermann_steering.py` |
| **1B** | **yes** — open `Task_1B.ttt` first, leave the simulation stopped | `python path_tracking.py` |
| **1C** | no | `python lane_detection.py public/clip_01.mp4 --show` |

Each folder has its own README with the details.

## Updating

Pull before you start each task — these files change:

```sh
cd ~/eYRC_26-27_Niti-Vahan
git pull
```
