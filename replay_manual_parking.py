#!/usr/bin/env python3
import argparse
import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from auto_parking.config import load_config  # noqa: E402
from auto_parking.control.drive import RoverDrive, clip, compute_wheel_speeds  # noqa: E402


DEFAULT_INPUT = PROJECT_ROOT / "recordings" / "parking_demo.json"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay a recorded manual parking command sequence.",
    )
    parser.add_argument(
        "recording",
        nargs="?",
        default=str(DEFAULT_INPUT),
        help="recording JSON path",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="config YAML path; defaults to config.yaml plus config.local.yaml",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print commands without opening the rover serial port",
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=1.0,
        help="multiply recorded speed by this value for safer slower replay",
    )
    parser.add_argument(
        "--steer-scale",
        type=float,
        default=1.0,
        help="multiply recorded steering by this value",
    )
    parser.add_argument(
        "--countdown",
        type=float,
        default=3.0,
        help="seconds to wait before replay starts",
    )
    return parser.parse_args()


def load_recording(path):
    with Path(path).expanduser().open("r", encoding="utf-8") as f:
        data = json.load(f)
    samples = data.get("samples", [])
    if not samples:
        raise ValueError("recording has no samples")
    return data, samples


def wait_countdown(seconds):
    if seconds <= 0:
        return
    end = time.monotonic() + seconds
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        print(f"Replay starts in {remaining:0.1f}s")
        time.sleep(min(1.0, remaining))


def main():
    args = parse_args()
    config = load_config(args.config)
    data, samples = load_recording(args.recording)
    drive = None if args.dry_run else RoverDrive(config)

    max_speed = float(config["rover"]["max_speed"])
    max_steer = float(config["rover"]["max_steer"])

    print(
        f"Loaded {len(samples)} samples from {args.recording} "
        f"(duration={data.get('duration', 'unknown')}s).",
    )
    if args.dry_run:
        print("DRY-RUN: serial port is not opened.")
    wait_countdown(args.countdown)

    start_time = time.monotonic()
    last_t = 0.0

    try:
        for sample in samples:
            target_t = float(sample.get("t", last_t))
            wait_until = start_time + target_t
            while True:
                remaining = wait_until - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(0.02, remaining))

            steering = clip(float(sample["steering"]) * args.steer_scale, max_steer)
            speed = clip(float(sample["speed"]) * args.speed_scale, max_speed)
            left, right = compute_wheel_speeds(steering, speed, max_steer, max_speed)
            if drive is not None:
                left, right = drive.send(steering, speed)

            print(
                f"t={target_t:6.2f}s speed={speed:+.2f} steering={steering:+.2f} "
                f"L={left:+.2f} R={right:+.2f}",
            )
            last_t = target_t
    except KeyboardInterrupt:
        print("Replay interrupted.")
    finally:
        if drive is not None:
            drive.stop()

    print("Replay finished. Rover stopped.")


if __name__ == "__main__":
    main()
