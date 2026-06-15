# Auto Parking Project Requirements

이 문서는 과제 PDF의 요구사항을 구현 체크리스트로 정리한 것입니다.

## Common Setup

- Parking lot: tape-marked 3-slot parking area.
- Slot size: about 1.3 to 1.5 times rover size.
- Start: rover starts in an aisle about 50 to 80 cm away from the slot row.
- Slot locations are unknown before start.
- Detection must use the front camera.

## Common Success Criteria

- Rover stops with all four wheels inside the slot lines.
- Body angle is within plus/minus 10 degrees of the slot direction.
- Touching or crossing a line is partial success with penalty.

## Level 1: Empty Slot Detection and Front-In Perpendicular Parking

Task:

- Drive forward along the aisle.
- Detect parking slot lines with the front camera.
- Park front-first into any empty slot without a preselected target slot.

Technical requirements:

- IPM/BEV top-view transform.
- Slot line detection.
- Slot pose estimation.
- Entry path generation.
- Pure Pursuit path tracking.

Judging:

- Common success criteria.
- Time limit: 60 seconds.

Implementation status:

- BEV tuning tool exists.
- Tape HSV mask exists.
- Vertical tape boundary detection exists.
- Slot center and entry point inference exists.
- Dry-run Level 1 loop exists.
- Low-speed drive mode exists but still needs hardware validation.
- Full SEARCH_SLOT to PARKING_MANEUVER state flow is not complete yet.

## Level 2: Occupancy Detection and Reverse Perpendicular Parking

Task:

- Three slots exist.
- Two slots contain obstacle vehicles such as boxes or miniature cars.
- Rover drives forward, identifies the empty slot, and reverse-parks into it.

Technical requirements:

- Slot occupancy detection by line detection and object existence inside slot.
- Store detected slot pose in odom frame.
- Drive forward to the reverse-entry start pose.
- Follow a two-arc reverse path.

Judging:

- Common success criteria.
- No contact with vehicles on either side.
- Time limit: 90 seconds.

Implementation status:

- Not started.
- Needs odometry or approximate local pose tracking.
- Needs occupancy classifier or OpenCV object-in-slot logic.
- Needs reverse path generation and reverse tracking.

## Level 3: Parallel Parking

Task:

- Parallel parking space is placed near a wall or board.
- Two vehicles are placed front and rear.
- Rover reverse-parks into a gap about 1.5 times rover length.

Technical requirements:

- Measure gap length while passing the space from the side.
- Generate S-shaped/two-arc reverse path.
- Reverse path tracking.

Judging:

- No contact with front and rear vehicles.
- Distance to wall within required range.
- Body parallelism within plus/minus 10 degrees.

Implementation status:

- Not started.
- Needs side-gap sensing strategy.
- Needs wall distance estimate.
- Needs reverse parallel parking path planner.

## Development Order

1. Finish Level 1 perception and low-speed control validation.
2. Implement Level 1 state machine with timeout and success/fail stopping.
3. Add basic pose estimation/odometry needed to remember detected slot pose.
4. Implement Level 2 occupancy detection.
5. Implement Level 2 reverse perpendicular path generation and tracking.
6. Implement Level 3 side-gap detection.
7. Implement Level 3 reverse parallel parking path generation and tracking.
8. Add final run scripts and demo documentation for all levels.
