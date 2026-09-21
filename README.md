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
