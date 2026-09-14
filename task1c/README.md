# Task 1C - Lane Detection Boilerplate

Niti Vahan (NV), eYRC 2026-27.

## Files

| File | What it is |
|---|---|
| `lane_detection.py` | The boilerplate. Fill in `detect_lane()`; leave everything else alone. |
| `public/` | The 20 video clips - 640 x 480, 20 fps, 200 frames each. |

## Setup

Nothing to download or unzip: cloning this repository gives you the clips.

```text
task1c/
├── lane_detection.py
└── public/
    ├── clip_01.mp4
    └── ...          (clip_20.mp4)
```

No simulator needed. Check your environment has what you need:

```sh
conda activate NV_<Team-ID>
cd ~/eYRC_26-27_Niti-Vahan/task1c
python -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"
```

## Usage

```sh
# one clip
python lane_detection.py public/clip_01.mp4

# every clip in the folder (press q to stop) - a folder works on Windows too
python lane_detection.py public --show

# write the per-frame results to a file
python lane_detection.py public --out results.json
```

## What you implement

Exactly one function:

```python
def detect_lane(frame):
    return {"center_x": <int>, "lane": "left" | "right" | "unknown"}
```

- `center_x` - x-pixel of the lane centre in this frame, `-1` if the lane was not found.
- `lane` - `"left"` if the dashed white centre line is to the **right** of the vehicle,
  `"right"` if it is to the **left**, `"unknown"` if you cannot tell.

`detect_lane()` must only compute and return. No `cv2.imshow()`, `cv2.waitKey()`,
`cv2.imwrite()` or `print()` inside it - visualisation goes in `draw_overlay()`,
which is called from `process_video()`.

**To submit:** see the Submission page in the theme book.
