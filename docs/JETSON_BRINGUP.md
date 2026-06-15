# Jetson Bring-up Order

Jetson에서 처음 진행할 때는 주차 알고리즘보다 하드웨어 입출력 확인을 먼저 끝냅니다.

## 0. 기존 Jetson 작업본 보존

Jetson에 이미 `embe_parking` 폴더가 있고 아직 push하지 않은 수정이 있으면 먼저 백업합니다.

```bash
cd ~
mv embe_parking embe_parking_backup_$(date +%Y%m%d_%H%M%S)
git clone git@github.com:tabletfish/embe_parking.git
cd embe_parking
```

기존 폴더가 이미 Git 저장소라면 백업 브랜치로 저장합니다.

```bash
cd ~/embe_parking
git checkout -b jetson-baseline
git add -A
git commit -m "Save Jetson baseline"
git push origin jetson-baseline
git checkout main
git pull origin main
```

## 1. Python 의존성 설치

```bash
cd ~/embe_parking
python3 -m pip install -r requirements.txt
```

Jetson의 OpenCV는 JetPack에 포함된 시스템 패키지를 쓰는 편이 더 안정적일 수 있습니다. `opencv-python` 설치가 충돌하면 제거하고 시스템 OpenCV를 사용합니다.

```bash
python3 -m pip uninstall opencv-python
python3 -c "import cv2; print(cv2.__version__)"
```

## 2. 카메라 확인

CSI 카메라 기본 실행:

```bash
python3 main_debug_bev.py
```

USB 카메라:

```bash
python3 main_debug_bev.py 0
```

카메라가 열리지 않으면 Argus 데몬을 재시작합니다.

```bash
sudo systemctl restart nvargus-daemon
```

## 3. BEV 좌표 튜닝

```bash
python3 tools/tune_bev.py
```

화면에서 바닥 주차판의 네 점을 순서대로 클릭합니다.

```text
1. 앞쪽 왼쪽
2. 앞쪽 오른쪽
3. 뒤쪽 오른쪽
4. 뒤쪽 왼쪽
```

출력값을 `config.yaml`의 `bev.src_points`에 반영합니다.

Jetson에서 튜닝한 값은 가능하면 `config.local.yaml`에 저장합니다. 이 파일은 Git에 올라가지 않으므로 이후 코드 업데이트 때 `git pull`과 충돌할 가능성이 낮습니다.

예:

```yaml
bev:
  src_points:
    - [112, 88]
    - [530, 91]
    - [610, 348]
    - [42, 352]

vision:
  tape_hsv_lower: [15, 10, 80]
  tape_hsv_upper: [95, 255, 255]
```

## 4. 테이프 HSV 튜닝

```bash
python3 main_debug_bev.py
```

오른쪽 마스크 화면에서 주차선 테이프만 흰색으로 보여야 합니다.

수정 위치:

```yaml
vision:
  tape_hsv_lower: [20, 25, 140]
  tape_hsv_upper: [75, 220, 255]
```

## 5. 슬롯 후보 확인

BEV 화면에서 노란 박스와 `slot 0`, `slot 1` 표시가 주차 슬롯 위치에 안정적으로 떠야 합니다.

수정 위치:

```yaml
vision:
  min_slot_area_px: 8000
  max_slot_area_px: 180000
  min_line_length: 35
  max_line_gap: 18
  hough_threshold: 35
```

## 6. 수동 주행 확인

로버 바퀴가 바닥에서 떨어진 상태로 먼저 테스트합니다.

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

전후진이나 좌우가 반대로 움직이면 `src/auto_parking/control/drive.py`의 wheel sign 또는 L/R 순서를 수정합니다.

## 7. 다음 구현 단계

카메라, BEV, HSV, 슬롯 후보, 수동 주행이 모두 확인된 뒤 `main_parking_level1.py`를 사용해서 Level 1 루프를 검증합니다.

먼저 dry-run:

```bash
python3 main_parking_level1.py
```

실제 저속 주행:

```bash
python3 main_parking_level1.py --drive --speed 0.18
```
