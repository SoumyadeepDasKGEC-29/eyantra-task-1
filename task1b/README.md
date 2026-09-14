# Task 1B — Path Tracking

Fill in `ackermann_wheel_angles()` and `compute_steering()` in `path_tracking.py`.

## Setup

1. Launch **CoppeliaSim**.
2. **File → Open scene…** and pick `Task_1B.ttt` from this folder.
3. Leave the simulation **stopped** — do not press the ▶ Play button. The
   script starts and stops it for you.
4. In a terminal:

```sh
conda activate NV_<Team-ID>
cd ~/eYRC_26-27_Niti-Vahan/task1b
python path_tracking.py
```

## The scene

| Object in the hierarchy | What it is |
|---|---|
| `/Floor` | the road, 5 m long and 1 m wide |
| `/Niti_Vahan` | the vehicle body |
| `/Niti_Vahan/steeringLeft`, `steeringRight` | front steering joints — your wheel angles go here |
| `/Niti_Vahan/motorLeft`, `motorRight` | driven front wheels, held at a constant speed |
| `/Niti_Vahan/freeAxisLeft`, `freeAxisRight` | rear wheels, free-spinning |

The vehicle drives along world **-x**, so its left-hand side faces **-y**.

| Lane | Lateral position |
|:---:|---|
| `L` | y = 0.00 m |
| `R` | y = 0.20 m |

## Driving it

While the run is going, type `L` or `R` and press Enter to change lane, `q` to
stop. Every run lasts 120 simulated seconds and writes `trajectory.csv`.

```sh
python path_tracking.py --out run1.csv     # write the CSV elsewhere
python path_tracking.py --log-rate 20      # rows per second (default 10)
python path_tracking.py --schedule "15:R,60:L,95:R"  # scripted lane changes
```

**To submit:** see the Submission page in the theme book.
