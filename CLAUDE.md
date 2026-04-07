# CLAUDE.md — Pinky MapAutoLearning & Control

> 이 파일은 Claude/Antigravity AI 코드 어시스턴트가 이 저장소를 이해하고 기여할 수 있도록
> 프로젝트의 핵심 정보를 요약해 놓은 가이드입니다.

---

## 1. 프로젝트 개요

**Pinky MapAutoLearning_Control**은 Raspberry Pi 기반의 소형 자율주행 로봇 **Pinky** 를 위한
ROS 2 (Humble) 기반 군집(Swarm) 탐색·관제 시스템입니다.

핵심 기능:
- **SLAM-free 자율 지도 생성** (`auto_mapper`): LiDAR + Odometry만으로 OccupancyGrid 작성
- **12-bit 그리드 주소 체계**: 64×64 격자를 `[AA]-[BB]` 형식으로 인코딩
- **군집 협력 탐색** (`swarm_coordinator` × N): BFS 구역 분할 + Nav2 액션 기반 이동
- **Fleet Manager**: 미션 등록/배분/완료 자동화 (거리·배터리 최적화)
- **관재 UI** (`control_center`): Matplotlib 기반 실시간 GUI (지도 + 로봇 상태 + 미션 목록)
- **AI 연동**: Ollama sLLM으로 자연어 명령 해석 + 탐색 브리핑 자동 생성
- **텔레그램 양방향**: 미션 탐지 → 사진+좌표 보고 / 텔레그램 명령 → 로봇 제어

---

## 2. 저장소 구조

```
pinky_MapAutoLearning_Control/
├── CLAUDE.md              ← 이 파일
├── README.md              ← GitHub 배포용 문서
└── pinky/
    ├── pinky_pro/         ← ROS 2 워크스페이스 (colcon 빌드 대상)
    │   ├── src/
    │   │   ├── pinky_pro/
    │   │   │   ├── pinky_bringup/        ← 하드웨어 드라이버 (Dynamixel + Odometry)
    │   │   │   ├── pinky_bringup_cpp/    ← C++ 버전 bringup
    │   │   │   ├── pinky_description/    ← URDF/XACRO 로봇 모델
    │   │   │   ├── pinky_emotion/        ← LCD 감정 표현 노드
    │   │   │   ├── pinky_gz_sim/         ← Gazebo 시뮬레이션 설정
    │   │   │   ├── pinky_imu_bno055/     ← IMU 드라이버
    │   │   │   ├── pinky_interfaces/     ← 커스텀 msg/srv/action
    │   │   │   ├── pinky_lamp_control/   ← LED 램프 제어
    │   │   │   ├── pinky_led/            ← WS281x LED 서버
    │   │   │   ├── pinky_mission/        ← ★ 핵심 미션 패키지
    │   │   │   ├── pinky_navigation/     ← Nav2 웹서버 + 설정
    │   │   │   └── pinky_sensor_adc/     ← 배터리 ADC 노드
    │   │   └── sllidar_ros2/             ← SLAMTEC LiDAR 드라이버
    │   ├── build/   (빌드 결과, git 제외)
    │   ├── install/ (설치 결과, git 제외)
    │   └── log/     (빌드 로그, git 제외)
    ├── pinky_devices/     ← 하드웨어 라이브러리 (WiringPi, libcamera, rpi-ws281x)
    ├── pinkylib/          ← 공유 Python 라이브러리 (lcd, sensor)
    ├── pinky_test/        ← 독립 테스트 스크립트
    └── ap/                ← AP 설정, 배터리/LCD 유틸리티
```

---

## 3. 핵심 패키지: `pinky_mission`

### 노드 목록

| 노드 | 실행 명령 | 역할 |
|------|-----------|------|
| `auto_mapper` | `ros2 run pinky_mission auto_mapper` | SLAM-free 지도 생성 |
| `swarm_coordinator` | `ros2 run pinky_mission swarm_coordinator` | 개별 로봇 자율 탐색 |
| `fleet_manager` | `ros2 run pinky_mission fleet_manager` | 미션 배분·관리 |
| `control_center` | `ros2 run pinky_mission control_center` | Matplotlib GUI |
| `yolo_detector` | `ros2 run pinky_mission yolo_detector` | YOLOv8 탐지 |
| `ollama_commander` | `ros2 run pinky_mission ollama_commander` | sLLM 명령 해석 |
| `mission_briefer` | `ros2 run pinky_mission mission_briefer` | sLLM 브리핑 생성 |
| `telegram_reporter` | `ros2 run pinky_mission telegram_reporter` | 텔레그램 연동 |

### 주요 토픽 인터페이스

```
/auto_mapper/command   (String)          IN  - START|STOP|PAUSE|RESUME|SAVE|RESET
/auto_mapper/status    (String, JSON)    OUT - {state, coverage_pct, free_cells, ...}
/map                   (OccupancyGrid)   OUT - 2Hz, TRANSIENT_LOCAL QoS
/cmd_vel               (Twist)           OUT - 모터 제어
/scan                  (LaserScan)       IN  - LiDAR 입력
/odom                  (Odometry)        IN  - 오도메트리

/swarm/map_update      (Int32MultiArray) INOUT - [robot_id, gx, gy, state]
/swarm/robot_pose      (Float32MultiArray) OUT - [robot_id, x, y, yaw]
/swarm/robot_status    (String, JSON)    OUT - {id, ns, x, y, battery_pct, state}
/swarm/fleet_cmd       (String)          IN  - SOLO_SEARCH | RETURN_NOW | RETURN_DELAY
/swarm/direct_cmd      (String)          IN  - STOP | GOTO | WAYPOINTS | FORCE_RETURN_ALL
/swarm/target_alert    (String)          OUT - ALERT|robot=...|label=...|addr=...

/mission/register      (String)          IN  - {type}:{label}:{wx}:{wy}
/mission/command       (String)          IN  - DISPATCH_ALL | EMERGENCY_STOP | ...
/mission/complete      (String)          IN  - {label}
/mission/status_list   (String, JSON)    OUT - 미션 목록
/mission/target_found  (String, JSON)    OUT - 탐지 이벤트
/mission/text_command  (String)          IN  - 자연어 명령 (Ollama 입력)
/mission/llm_response  (String)          OUT - LLM 원문 응답
/mission/briefing      (String)          OUT - 자연어 브리핑
/mission/coverage      (Float32)         OUT - 탐색 커버리지 (0~1)
```

---

## 4. 빌드 및 실행

### 사전 요구사항

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop ros-humble-nav2-bringup \
                 ros-humble-tf-transformations

# Python 의존성
pip install numpy ultralytics requests

# Ollama (선택)
curl -fsSL https://ollama.ai/install.sh | sh
ollama pull llama3
```

### 빌드

```bash
cd ~/pinky_pro
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pinky_mission
source install/setup.bash
```

### 전체 시스템 실행 (런치)

```bash
# 2대 군집 (기본)
ros2 launch pinky_mission mission_launch.py \
    bot_token:=<TELEGRAM_TOKEN> \
    chat_id:=<CHAT_ID>

# 3대 군집
ros2 launch pinky_mission mission_launch.py \
    num_robots:=3 \
    world_width:=500.0 \
    world_height:=150.0 \
    bot_token:=<TOKEN> \
    chat_id:=<CHAT_ID>

# 관재 UI 별도 실행
ros2 run pinky_mission control_center

# 자동 지도 단독 실행
ros2 run pinky_mission auto_mapper \
    --ros-args -p auto_start:=true -p save_path:=/tmp/maps
```

### 유닛 테스트

```bash
cd ~/pinky_pro
colcon test --packages-select pinky_mission
colcon test-result --verbose
```

---

## 5. 코드 스타일 가이드

### Python

- **스타일**: PEP 8 준수, flake8으로 검사 (`colcon test` 시 자동 실행)
- **독스트링**: PEP 257 준수 (모듈·클래스·메서드 단위 필수)
- **타입 힌트**: 함수 인자 및 반환값에 가급적 명시 (`typing` 모듈 활용)
- **임포트 순서**: 표준 라이브러리 → 서드파티(ROS, numpy 등) → 내부 모듈
- **네이밍**:
  - 클래스: `PascalCase` (예: `AutoMapper`, `FleetManagerNode`)
  - 함수/변수: `snake_case`
  - 상수: `UPPER_SNAKE_CASE`
  - private 멤버: `_underscore_prefix`

### ROS 2 규칙

- **파라미터**: 하드코딩 대신 반드시 `declare_parameter()` + `get_parameter()` 사용
- **QoS**: `/map` 등 latching 토픽은 `TRANSIENT_LOCAL` + `RELIABLE` 명시
- **스레드 안전**: 콜백에서 공유 데이터 접근 시 반드시 `threading.Lock()` 사용
- **로깅**: `print()` 금지 → `self.get_logger().info/warn/error()` 사용
- **종료 처리**: `try/finally`로 `node.destroy_node()` + `rclpy.shutdown()` 보장

### C++ (pinky_bringup_cpp)

- **스타일**: Google C++ Style Guide 기반
- **네이밍**: 클래스 `PascalCase`, 멤버 변수 `snake_case_`(trailing underscore)
- **포맷터**: `clang-format` (구성 파일은 패키지 루트에 `.clang-format` 위치)

---

## 6. 보안 주의사항 (Security)

> ⚠️ **커밋 전 반드시 확인**

| 항목 | 위치 | 조치 |
|------|------|------|
| 텔레그램 Bot Token | `telegram_reporter` 파라미터 | 환경변수 또는 `.env` 파일로 분리 |
| 텔레그램 Chat ID | 동상 | 동상 |
| Ollama Host URL | `ollama_commander` 파라미터 | 로컬 전용 (`localhost`)으로 유지 |
| 이미지 저장 경로 | `yolo_detector` `image_path` | `/tmp`는 로봇 내부 전용, 외부 공유 금지 |
| 지도 저장 경로 | `auto_mapper` `save_path` | 기본값 `/tmp`, 배포 시 전용 폴더로 변경 |

**비밀 키 관리 패턴:**

```bash
# launch 시 환경변수로 전달
export PINKY_BOT_TOKEN="1234567890:ABCdef..."
export PINKY_CHAT_ID="-100123456789"

ros2 launch pinky_mission mission_launch.py \
    bot_token:=$PINKY_BOT_TOKEN \
    chat_id:=$PINKY_CHAT_ID
```

---

## 7. 아키텍처 요약 (Mermaid)

```
[LiDAR /scan] ──────────────────────┐
[Odometry /odom] ───────────────────┤
                                    ▼
                            [auto_mapper]  ──→  /map (OccupancyGrid)
                                    │           /auto_mapper/status
                                    ▼
[Camera /{ns}/camera/image_raw] → [yolo_detector] ──→ /mission/target_found
                                                  ──→ /swarm/target_alert

[Nav2 /{ns}/navigate_to_pose] ← [swarm_coordinator] ←── /swarm/fleet_cmd
                                        │                /swarm/direct_cmd
                                        ▼
                               /swarm/map_update
                               /swarm/robot_status

[/mission/register] ──→ [fleet_manager] ──→ /swarm/fleet_cmd
[/mission/command]  ──→                ──→ /mission/status_list
[/mission/complete] ──→

[자연어 입력] ──→ [ollama_commander] ──→ /mission/command
[상태 데이터] ──→ [mission_briefer]  ──→ /mission/briefing

[텔레그램 ←→] ←→ [telegram_reporter] ←→ /mission/command
                                      ←→ /mission/target_found

[control_center GUI] ←→ 모든 상태 토픽 (시각화)
                    ──→ /swarm/direct_cmd, /mission/*, /auto_mapper/command
```

---

## 8. TODO / 리팩토링 과제

- [ ] `bringup.py`: `SERIAL_PORT_NAME`, `BAUDRATE`, `DYNAMIXEL_IDS` 등 하드코딩 상수를 `declare_parameter()`로 전환
- [ ] `bringup.py`: `MAX_RPM = 100.0` 상수를 파라미터화
- [ ] `telegram_reporter.py`: `bot_token` 기본값 `'YOUR_BOT_TOKEN_HERE'` → 빈 문자열로 변경 후 유효성 검사 추가
- [ ] `yolo_detector.py`: 이미지 저장 경로(`/tmp/pinky_detection_*.jpg`) 파라미터화
- [ ] `auto_mapper.py`: `FWD_SPEED`, `ROT_SPEED`, `FRONT_DIST_TH` 모두 파라미터로 노출 (현재 모듈 상수)
- [ ] `swarm_coordinator.py`: `world_w=500.0` / `world_h=150.0` 기본값을 launch 인자와 일치되도록 검토
- [ ] `mission_launch.py`: `target_points` 기본 하드코딩 좌표값 파라미터 파일(`.yaml`)로 분리
- [ ] 전체: `/tmp` 경로 사용 → `ament_index` 또는 XDG 표준 경로로 교체 권장
- [ ] C++ bringup: clang-format 설정 파일 추가
- [ ] `pinky_interfaces`: 커스텀 `msg/srv/action` 문서화
