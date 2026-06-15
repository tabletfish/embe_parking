#!/usr/bin/env python3
import argparse
from dataclasses import dataclass
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from auto_parking.camera.csi import open_video_source  # noqa: E402
from auto_parking.config import load_config  # noqa: E402
from auto_parking.perception.bev import BirdEyeView  # noqa: E402
from auto_parking.perception.slot_detector import (  # noqa: E402
    detect_slot_candidates,
    detect_vertical_tape_boundaries,
    draw_locked_parking_slot,
    draw_parking_slots,
    draw_slots,
    draw_tape_boundaries,
    infer_parking_slots_from_mask,
)
from auto_parking.perception.tape import tape_mask  # noqa: E402
from auto_parking.planning.pure_pursuit import pure_pursuit_steering  # noqa: E402
from auto_parking.control.drive import compute_wheel_speeds  # noqa: E402
from auto_parking.state_machine.parking_fsm import ParkingFSM, ParkingState  # noqa: E402


@dataclass
class LockedParkingTarget:
    entry_forward_m: float
    entry_lateral_m: float
    center_forward_m: float
    center_lateral_m: float
    entry_px: tuple
    center_px: tuple
    approach_remaining_m: float
    park_remaining_m: float
    lateral_error_m: float

    @classmethod
    def from_slot(cls, bev, slot):
        entry_lateral, entry_forward = bev.pixel_to_vehicle(slot.entry_px)
        center_lateral, center_forward = bev.pixel_to_vehicle(slot.center_px)
        park_distance = max(0.0, center_forward - entry_forward)
        return cls(
            entry_forward_m=entry_forward,
            entry_lateral_m=entry_lateral,
            center_forward_m=center_forward,
            center_lateral_m=center_lateral,
            entry_px=slot.entry_px,
            center_px=slot.center_px,
            approach_remaining_m=max(0.0, entry_forward),
            park_remaining_m=park_distance,
            lateral_error_m=entry_lateral,
        )

    def update_approach_from_detection(self, bev, slot):
        entry_lateral, entry_forward = bev.pixel_to_vehicle(slot.entry_px)
        center_lateral, center_forward = bev.pixel_to_vehicle(slot.center_px)
        self.entry_forward_m = entry_forward
        self.entry_lateral_m = entry_lateral
        self.center_forward_m = center_forward
        self.center_lateral_m = center_lateral
        self.entry_px = slot.entry_px
        self.center_px = slot.center_px
        self.approach_remaining_m = max(0.0, entry_forward)
        self.park_remaining_m = max(0.0, center_forward - entry_forward)
        self.lateral_error_m = entry_lateral

    def advance_approach(self, distance_m):
        self.approach_remaining_m = max(0.0, self.approach_remaining_m - distance_m)

    def advance_parking(self, distance_m):
        self.park_remaining_m = max(0.0, self.park_remaining_m - distance_m)


def parse_source(value):
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return value


def entry_to_path_point(bev, entry_px):
    lateral_m, forward_m = bev.pixel_to_vehicle(entry_px)
    return forward_m, lateral_m


def draw_control_debug(image, state, entry_px, center_px, target, steering, drive_enabled, locked=False):
    out = image.copy()
    status = "DRIVE" if drive_enabled else "DRY-RUN"
    if locked:
        status += " LOCKED"
    cv2.putText(
        out,
        f"{status} {state.name} steer={steering:+.2f} target=({target[0]:.2f}m,{target[1]:+.2f}m)",
        (16, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.arrowedLine(
        out,
        (out.shape[1] // 2, out.shape[0] - 1),
        entry_px,
        (0, 255, 255),
        2,
        tipLength=0.08,
    )
    if center_px is not None:
        cv2.arrowedLine(out, entry_px, center_px, (0, 200, 0), 2, tipLength=0.08)
    return out


def choose_parking_slot(parking_slots, bev):
    if not parking_slots:
        return None
    center_x = bev.width / 2.0
    return min(parking_slots, key=lambda slot: (abs(slot.entry_px[0] - center_x), slot.entry_px[1]))


def target_for_state(state, locked_target):
    if locked_target is None:
        return None
    if state in (ParkingState.ALIGN_TO_ENTRY, ParkingState.DRIVE_TO_ENTRY):
        return locked_target.approach_remaining_m, locked_target.lateral_error_m
    if state == ParkingState.PARKING_MANEUVER:
        lateral = locked_target.center_lateral_m - locked_target.entry_lateral_m
        return locked_target.park_remaining_m, lateral
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", nargs="?", help="camera index, video path, or omitted for CSI cam0")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config.yaml"))
    parser.add_argument("--drive", action="store_true", help="send low-speed commands to the rover")
    parser.add_argument("--speed", type=float, default=None, help="override rover.default_speed")
    parser.add_argument("--entry-threshold", type=float, default=0.08, help="stop when entry is this close in meters")
    parser.add_argument("--park-threshold", type=float, default=0.05, help="stop when slot center is this close in meters")
    parser.add_argument("--timeout", type=float, default=60.0, help="Level 1 time limit in seconds")
    args = parser.parse_args()

    config = load_config(None if args.config == str(PROJECT_ROOT / "config.yaml") else args.config)
    cap = open_video_source(parse_source(args.source), config)
    bev = BirdEyeView(config)

    drive = None
    if args.drive:
        from auto_parking.control.drive import RoverDrive  # noqa: WPS433

        drive = RoverDrive(config)

    speed = float(args.speed if args.speed is not None else config["rover"]["default_speed"])
    speed = min(speed, float(config["rover"]["max_speed"]))
    search_speed = min(speed * 0.7, float(config["rover"]["max_speed"]))
    wheelbase_m = float(config["rover"]["wheelbase_m"])
    last_print = 0.0
    last_loop = time.monotonic()
    start_time = last_loop
    fsm = ParkingFSM(ParkingState.SEARCH_SLOT)
    locked_target = None
    success_reported = False

    try:
        while True:
            now = time.monotonic()
            dt = min(now - last_loop, 0.2)
            last_loop = now
            elapsed = now - start_time

            ok, frame = cap.read()
            if not ok:
                break

            top = bev.warp(frame)
            mask = tape_mask(top, config)
            slots = detect_slot_candidates(mask, config)
            boundaries = detect_vertical_tape_boundaries(mask, config)
            parking_slots = infer_parking_slots_from_mask(mask, config)

            debug = bev.draw_grid(top)
            debug = draw_slots(debug, slots)
            debug = draw_tape_boundaries(debug, boundaries)
            debug = draw_parking_slots(debug, parking_slots)

            steering = 0.0
            command_speed = 0.0
            selected = choose_parking_slot(parking_slots, bev)
            if selected is not None and fsm.state in (ParkingState.SEARCH_SLOT, ParkingState.ALIGN_TO_ENTRY, ParkingState.DRIVE_TO_ENTRY):
                if locked_target is None:
                    locked_target = LockedParkingTarget.from_slot(bev, selected)
                else:
                    locked_target.update_approach_from_detection(bev, selected)

            failed = elapsed >= args.timeout
            at_entry = locked_target is not None and locked_target.approach_remaining_m <= args.entry_threshold
            parked = locked_target is not None and locked_target.park_remaining_m <= args.park_threshold
            slot_found = locked_target is not None
            state = fsm.update(slot_found=slot_found, at_entry=at_entry, parked=parked, failed=failed)

            target = target_for_state(state, locked_target)
            if state == ParkingState.SEARCH_SLOT:
                command_speed = search_speed
            elif state in (ParkingState.SUCCESS, ParkingState.FAIL):
                command_speed = 0.0
            elif target is not None:
                if state == ParkingState.PARKING_MANEUVER and parked:
                    steering = 0.0
                    command_speed = 0.0
                else:
                    command_speed = speed
                path = np.array([target], dtype=float)
                steering, _ = pure_pursuit_steering(path, (0.0, 0.0, 0.0), 0.05, wheelbase_m)

            if target is None:
                target = (0.0, 0.0)

            if locked_target is not None:
                debug = draw_locked_parking_slot(debug, locked_target.center_px, locked_target.entry_px)
                debug = draw_control_debug(
                    debug,
                    state,
                    locked_target.entry_px,
                    locked_target.center_px,
                    target,
                    steering,
                    args.drive,
                    selected is None,
                )

            left_cmd, right_cmd = compute_wheel_speeds(
                steering,
                command_speed,
                float(config["rover"]["max_steer"]),
                float(config["rover"]["max_speed"]),
            )

            driven_distance = max(0.0, command_speed) * dt
            if locked_target is not None:
                if state in (ParkingState.ALIGN_TO_ENTRY, ParkingState.DRIVE_TO_ENTRY):
                    locked_target.advance_approach(driven_distance)
                elif state == ParkingState.PARKING_MANEUVER:
                    locked_target.advance_parking(driven_distance)

            if now - last_print >= 0.5:
                print(
                    f"mode={'DRIVE' if args.drive else 'DRY-RUN'} "
                    f"state={state.name} elapsed={elapsed:.1f}s "
                    f"target_forward={target[0]:.3f}m "
                    f"target_lateral={target[1]:+.3f}m "
                    f"steering={steering:+.3f} speed={command_speed:.2f} "
                    f"L={left_cmd:+.3f} R={right_cmd:+.3f}"
                )
                last_print = now

            if drive is not None:
                if state in (ParkingState.SUCCESS, ParkingState.FAIL):
                    drive.stop()
                else:
                    drive.send(steering, command_speed)

            if state == ParkingState.SUCCESS and not success_reported:
                print(f"SUCCESS elapsed={elapsed:.1f}s")
                success_reported = True
            elif state == ParkingState.FAIL:
                print(f"FAIL timeout elapsed={elapsed:.1f}s")
                if drive is not None:
                    drive.stop()
                break
            elif state == ParkingState.SUCCESS and args.drive:
                break
            elif state == ParkingState.SUCCESS and not args.drive:
                command_speed = 0.0
            elif drive is not None and target is None:
                drive.stop()

            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            if frame.shape[:2] != top.shape[:2]:
                frame_show = cv2.resize(frame, (top.shape[1], top.shape[0]))
            else:
                frame_show = frame
            combined = np.hstack([frame_show, debug, mask_bgr])
            cv2.imshow("front | level1 parking | tape mask", combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        if drive is not None:
            drive.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
