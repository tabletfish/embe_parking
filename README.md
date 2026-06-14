# Jetson Orin Nano Auto Parking Rover

Autonomous parking project for a dual-camera rover.

## First milestone

Build Level 1 first:

1. Open front camera.
2. Warp the floor into BEV/IPM.
3. Detect tape lines in BEV.
4. Estimate parking slot candidates.
5. Drive to the slot with path-following.

YOLO is not required for this milestone. It can be added later for occupied-slot or obstacle detection.

## Layout

```text
main_debug_bev.py              BEV and slot detection visual test
manual_drive.py                Keyboard manual driving
config.yaml                    Camera, BEV, vision, rover parameters
rover/                         Existing BaseController and jetcam code
src/auto_parking/
  camera/                      CSI and video source helpers
  perception/                  BEV, tape mask, slot detection
  planning/                    Pure Pursuit in metric coordinates
  control/                     Wheel command wrapper
  state_machine/               Parking states
```

## Run

```bash
cd /home/dydlz/embe_parking
python3 main_debug_bev.py
python3 manual_drive.py
```

For USB camera or video:

```bash
python3 main_debug_bev.py 0
python3 main_debug_bev.py sample.mp4
```

## Next tuning step

Edit `config.yaml`:

- `bev.src_points`: four floor points in the original camera image
- `bev.dst_points`: rectangle in the BEV image
- `vision.tape_hsv_lower` / `vision.tape_hsv_upper`: tape color threshold

