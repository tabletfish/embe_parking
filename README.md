# Jetson Orin Nano Auto Parking Rover

Jetson Orin Nano 기반 dual-camera rover로 자동주차를 구현하는 프로젝트입니다.

목표는 처음부터 YOLO를 붙이는 것이 아니라, 카메라 영상에서 바닥 주차선 테이프를 검출하고 BEV(Bird's-eye-view, top-view)로 변환한 뒤 슬롯 위치를 추정해서 Pure Pursuit로 주차 경로를 따라가는 것입니다.

## 현재 목표

과제 전체 요구사항은 `docs/PROJECT_REQUIREMENTS.md`에 정리합니다. 최종 목표는 Level 1, Level 2, Level 3 전체 수행입니다.

우선 Level 1을 완성합니다.

```text
전방 카메라
  -> BEV/IPM 변환
  -> 테이프 라인 검출
  -> 빈 주차 슬롯 pose 추정
  -> 슬롯 앞 entry point 생성
  -> Pure Pursuit로 전면 직각주차
```

YOLO는 아직 필요하지 않습니다. Level 2에서 슬롯 내부 박스/미니카 점유 판단이 OpenCV만으로 불안정할 때 보조로 붙이면 됩니다.

## 프로젝트 구조

```text
main_debug_bev.py
  카메라 영상, BEV 변환, 테이프 마스크, 슬롯 후보를 한 화면에서 확인하는 디버그 실행 파일

main_parking_level1.py
  Level 1 자동주차 루프입니다. 기본값은 dry-run이라 모터를 움직이지 않고 entry point와 steering만 표시합니다.

manual_drive.py
  키보드로 rover를 직접 조작하는 파일

config.yaml
  카메라 해상도, BEV 좌표, HSV 색상 범위, rover 속도 파라미터

tools/tune_bev.py
  실제 카메라 화면에서 BEV 변환용 4개 점을 찍는 도구

rover/
  기존 프로젝트에서 가져온 BaseController, jetcam 코드

src/auto_parking/camera/
  CSI camera / USB camera / video source 열기

src/auto_parking/perception/
  BEV 변환, 테이프 마스크, 주차 슬롯 후보 검출

src/auto_parking/planning/
  metric 좌표계 기반 Pure Pursuit

src/auto_parking/control/
  steering/speed를 좌우 바퀴 명령으로 변환하고 BaseController로 송신

src/auto_parking/state_machine/
  주차 상태머신 기본 골격
```

## 해야 할 과정

Jetson에서 처음 가져가서 실행하는 순서는 `docs/JETSON_BRINGUP.md`를 먼저 따릅니다. 아직 Jetson 쪽 변경분을 push하지 않았다면 기존 작업본을 백업하거나 `jetson-baseline` 브랜치로 먼저 저장한 뒤 진행합니다.

### 1. 카메라가 열리는지 확인

```bash
cd /home/dydlz/embe_parking
python3 main_debug_bev.py
```

USB 카메라로 테스트할 때:

```bash
python3 main_debug_bev.py 0
```

영상 파일로 테스트할 때:

```bash
python3 main_debug_bev.py sample.mp4
```

화면은 왼쪽부터 `원본 영상 | BEV 슬롯 디버그 | 테이프 마스크` 순서입니다.

### 2. BEV 4점 잡기

자동주차에서 가장 먼저 튜닝해야 할 부분입니다.

```bash
python3 tools/tune_bev.py
```

카메라 화면에서 바닥 주차판의 사다리꼴 영역 네 점을 순서대로 클릭합니다.

```text
1. 앞쪽 왼쪽
2. 앞쪽 오른쪽
3. 뒤쪽 오른쪽
4. 뒤쪽 왼쪽
```

출력되는 값을 `config.yaml`의 `bev.src_points`에 복사합니다.

```yaml
bev:
  src_points:
    - [x1, y1]
    - [x2, y2]
    - [x3, y3]
    - [x4, y4]
```

### 3. 테이프 색상 HSV 튜닝

`main_debug_bev.py`를 실행했을 때 오른쪽 마스크 화면에서 주차선 테이프만 흰색으로 보여야 합니다.

수정할 위치:

```yaml
vision:
  tape_hsv_lower: [20, 25, 140]
  tape_hsv_upper: [75, 220, 255]
```

문제별 조정 방향:

```text
테이프가 끊겨 보임
  -> saturation/value 범위를 넓히기

바닥 노이즈가 많이 잡힘
  -> saturation lower를 올리거나 value lower를 올리기

조명 때문에 색이 바뀜
  -> 같은 위치에서 여러 프레임을 보면서 범위를 넓게 잡기
```

### 4. 슬롯 후보 검출 확인

```bash
python3 main_debug_bev.py
```

BEV 화면에서 노란 박스와 `slot 0`, `slot 1` 같은 표시가 주차 슬롯 위치에 안정적으로 떠야 합니다.

수정할 위치:

```yaml
vision:
  min_slot_area_px: 8000
  max_slot_area_px: 180000
  min_line_length: 35
  max_line_gap: 18
  hough_threshold: 35
```

현재 `slot_detector.py`는 첫 단계용 단순 contour 기반입니다. 슬롯 검출이 불안정하면 다음 단계에서 Hough line 조합 기반으로 바꿔야 합니다.

### 5. 수동 조작 확인

로버가 안전한 곳에 있는 상태에서 실행합니다.

```bash
python3 manual_drive.py
```

조작:

```text
w: 전진
s: 후진
a: 좌회전
d: 우회전
space: 정지
q: 종료
```

만약 전진/후진 또는 좌우가 반대로 움직이면 `src/auto_parking/control/drive.py`의 `compute_wheel_speeds()`에서 부호 또는 L/R 순서를 조정해야 합니다.

### 5-1. 모범 주차 수동 기록/재생

자동 인식 없이, 사람이 수동으로 성공한 주차 조작을 기록했다가 같은 명령을 그대로 재생할 수 있습니다.

녹화:

```bash
python3 record_manual_parking.py -o recordings/parking_demo.json
```

조작:

```text
w: 가속
s: 후진 가속
a: 좌회전
d: 우회전
space: 정지
q: 녹화 종료 후 저장
```

재생 전에는 로버를 녹화 시작 위치와 같은 위치/방향에 놓아야 합니다.

```bash
python3 replay_manual_parking.py recordings/parking_demo.json
```

안전하게 절반 속도로 재생하려면:

```bash
python3 replay_manual_parking.py recordings/parking_demo.json --speed-scale 0.5
```

모터 연결 없이 파일 형식과 타이밍만 확인하려면:

```bash
python3 replay_manual_parking.py recordings/parking_demo.json --dry-run
```

### 6. Level 1 주차 로직 붙이기

BEV와 수동조작이 안정화되면 다음 순서로 구현합니다.

```text
SEARCH_SLOT
  천천히 전진하면서 슬롯 후보 탐색

ALIGN_TO_ENTRY
  검출한 슬롯 앞 entry point 계산

DRIVE_TO_ENTRY
  entry point까지 이동

PARKING_MANEUVER
  슬롯 중심으로 전면 진입

FINAL_ALIGN
  차체 각도와 위치 미세 보정

SUCCESS
  정지
```

이미 기본 상태머신은 `src/auto_parking/state_machine/parking_fsm.py`에 있습니다. 다음 작업은 `main_parking_level1.py`를 새로 만들어 위 상태들을 실제 카메라/제어 루프와 연결하는 것입니다.

먼저 dry-run으로 실행합니다.

```bash
python3 main_parking_level1.py
```

USB 카메라일 때:

```bash
python3 main_parking_level1.py 0
```

화면에 `DRY-RUN`, `park 0`, `entry`, `steer=...`가 정상적으로 보이면 낮은 속도로 실제 송신을 켭니다.

```bash
python3 main_parking_level1.py --drive --speed 0.18
```

## 추천 개발 순서

```text
1. main_debug_bev.py에서 카메라 정상 출력 확인
2. tools/tune_bev.py로 BEV src_points 튜닝
3. 테이프 HSV 범위 튜닝
4. 슬롯 후보가 안정적으로 뜨는지 확인
5. manual_drive.py로 모터 방향/속도 확인
6. 슬롯 중심과 entry point를 BEV 좌표로 계산
7. Pure Pursuit로 entry point까지 이동
8. 슬롯 중심으로 전면 진입
9. 성공 판정 기준 추가
10. Level 2 점유 판단/후진주차로 확장
```

## 자주 막히는 지점

카메라가 안 열릴 때:

```bash
sudo systemctl restart nvargus-daemon
```

BEV가 이상하게 찌그러질 때:

```text
src_points 순서가 틀렸거나, 네 점이 실제 바닥 평면이 아닐 가능성이 큼
```

슬롯이 안 잡힐 때:

```text
HSV 마스크부터 확인
마스크에서 테이프가 깨끗하게 잡힌 뒤 slot_detector 파라미터를 조정
```

로버가 반대로 움직일 때:

```text
manual_drive.py로 먼저 방향 확인
control/drive.py의 wheel sign 또는 L/R 순서 수정
```

## GitHub

```text
git@github.com:tabletfish/embe_parking.git
```
