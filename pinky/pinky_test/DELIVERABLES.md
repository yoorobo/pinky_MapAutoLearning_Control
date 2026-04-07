# Pinky 12-bit SLAM 미션 시스템 — 산출물 완전 요약 v4.1

> **최종 업데이트**: 2026-03-25
>
> **핵심 구성**: 12-bit SLAM · YOLOv8 탐지 · Ollama sLLM · 완전한 군집 주행(Nav2) · 우선순위 복귀 · matplotlib 관재 GUI · C++ 50Hz Bringup

---

## 전체 아키텍처

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           운영자 인터페이스                                │
│                                                                          │
│  텔레그램 앱  ←→  telegram_reporter  ──────────┐                          │
│  자연어 명령  ──→  ollama_commander  ───────────┤──→ /mission/command      │
│  상태 보고   ←──  mission_briefer               │                          │
│                                                 │                          │
│  ★ control_center (matplotlib GUI) ─────────────┤──→ /mission/command      │
│    · 64×64 그리드 실시간 표시                    └──→ /swarm/direct_cmd     │
│    · 로봇 클릭 → 선택 + 즉시 정지                                          │
│    · 경유지 누적 → 순서대로 Nav2 주행                                       │
└────────────────────────────┬─────────────────────────────────────────────┘
                             │ /mission/command
           ┌─────────────────▼────────────────────┐
           │            fleet_manager              │  ★ 우선순위 결정
           │  FIND_TARGET  → 가장 가까운 1대 수색   │
           │                 나머지 배터리순 복귀    │
           │  RETURN_TO_ORIGIN → 전체 배터리순 복귀 │
           │  FORCE_RETURN_ALL → 전체 즉시 복귀    │──→ /swarm/direct_cmd
           │  PATROL_ALL  → 전체 탐색 재개         │
           └──────┬──────────────┬─────────────────┘
     /swarm/fleet_cmd      /swarm/direct_cmd
  ┌──────▼────────┐  ┌──────▼────────┐  ┌──────────────────┐
  │ swarm_coord   │  │ swarm_coord   │  │ swarm_coord ...  │
  │ /robot1       │  │ /robot2       │  │ /robotN          │
  │ 13-state FSM  │  │ 13-state FSM  │  │ 13-state FSM     │
  │ · Nav2 주행   │  │ · Nav2 주행   │  │ · Nav2 주행      │
  │ · BFS 탐색    │  │ · BFS 탐색    │  │ · BFS 탐색       │
  │ · 경유지 큐   │  │ · 경유지 큐   │  │ · 경유지 큐      │
  └───────┬───────┘  └──────┬────────┘  └────────┬─────────┘
          │                 │ /swarm/map_update    │
          └─────────────────┴─────────────────────┘
                     12비트 그리드 실시간 공유 (64×64)

  각 로봇 하드웨어 스택 (독립 실행):
  ┌─────────────────────────────────────────────┐
  │  pinky_bringup_cpp (C++, 50Hz)              │
  │  Nav2 (namespace: /robot1, /robot2 ...)     │
  │  yolo_detector (YOLOv8n, 네임스페이스 분리)  │
  └─────────────────────────────────────────────┘
```

---

## 산출물 파일 목록

### 패키지 1: `pinky_mission` — Python ROS2

| 파일 | 버전 | 설명 |
| :--- | :---: | :--- |
| `pinky_mission/swarm_coordinator.py` | v4 | 군집 자율 주행 핵심 — 13상태 FSM, Nav2 액션, 경유지 큐 |
| `pinky_mission/fleet_manager.py` | v2 | 우선순위 결정 — FIND_TARGET / RETURN / FORCE / PATROL |
| `pinky_mission/control_center.py` | 신규 | matplotlib 관재 GUI — 클릭 선택·정지·경유지·직접 이동 |
| `pinky_mission/yolo_detector.py` | v2 | YOLOv8n 탐지 + 네임스페이스 + JSON 이벤트 발행 |
| `pinky_mission/telegram_reporter.py` | v1 | 텔레그램 보고/명령 수신 |
| `pinky_mission/ollama_commander.py` | v1 | Ollama sLLM 자연어 명령 해석 |
| `pinky_mission/mission_briefer.py` | v1 | Ollama sLLM 탐색 브리핑 자동 생성 |
| `launch/mission_launch.py` | v3 | 전체 스택 런치 — OpaqueFunction 동적 로봇 그룹 |
| `package.xml` | v1 | rclpy, nav2_msgs, cv_bridge 의존성 |
| `setup.py` | v2 | **7개** entry point |

**`setup.py` entry points:**
```
yolo_detector     swarm_coordinator  fleet_manager
telegram_reporter  ollama_commander   mission_briefer
control_center  ← 신규 추가
```

### 패키지 2: `pinky_bringup_cpp` — C++ ROS2 (신규)

| 파일 | 설명 |
| :--- | :--- |
| `src/pinky_bringup_node.cpp` | DynamixelDriver 클래스, 50Hz wall_timer, MultiThreadedExecutor |
| `CMakeLists.txt` | ament_cmake 빌드 설정 |
| `package.xml` | rclcpp, dynamixel_sdk, tf2_ros 의존성 |

### 기타

| 파일 | 설명 |
| :--- | :--- |
| `pinky_test/pinky_12bit_slam.py` | 12비트 SLAM 독립 시뮬레이션 (단독 실행) |
| `pinky_navigation/params/nav2_inflation_params.yaml` | Nav2 충돌 방지 최적화 파라미터 |

---

## 노드별 상세 설명

### `swarm_coordinator.py` — 13상태 FSM (v4)

```
SEARCHING ──────────────────→ NAVIGATING_EXPLORE ──→ (반복)
    ↑  PATROL_ALL                        │ 실패
    │                                    └──→ SEARCHING

SOLO_SEARCH ────────────────→ NAVIGATING_SOLO ─────→ AT_BASE
    (fleet_cmd: SOLO_SEARCH:{ns}:...)

DIRECT_GOTO ★ ──────────────→ NAVIGATING_DIRECT ───→ SEARCHING
    (direct_cmd: GOTO:{ns}:{wx}:{wy})

WAYPOINT_FOLLOW ★ ──────────→ NAVIGATING_WAYPOINT ─→ WAYPOINT_FOLLOW
    (direct_cmd: WAYPOINTS:{ns}:...)              큐 소진시 → SEARCHING

RETURNING ──────────────────→ NAVIGATING_RETURN ───→ AT_BASE
    (fleet_cmd: RETURN_NOW/DELAY  또는  FORCE_RETURN_ALL)

PAUSED ★  (direct_cmd: STOP:{ns}) ← 관재 클릭 시 즉시 진입
EMERGENCY_STOP  (mission/command: EMERGENCY_STOP)
AT_BASE  (원점 대기)
```

**파라미터:**

| 파라미터 | 기본값 | 설명 |
| :--- | :--- | :--- |
| `robot_namespace` | `robot1` | 로봇 네임스페이스 |
| `robot_id` | `1` | 로봇 ID (구역 분할에 사용) |
| `num_robots` | `2` | 전체 로봇 수 (구역 균등 분할) |
| `world_width` | `500.0` | 맵 실제 폭 (m) |
| `world_height` | `150.0` | 맵 실제 높이 (m) |
| `robot_radius` | `0.15` | 로봇 반경 (충돌 팽창) |
| `safety_margin` | `0.25` | 추가 안전 마진 |
| `nav_timeout_sec` | `30.0` | Nav2 타임아웃 |
| `explore_interval` | `1.0` | BFS 탐색 주기 (초) |

---

### `fleet_manager.py` — 우선순위 결정 (v2)

**명령 처리표:**

| 수신 명령 | 처리 내용 | 발행 토픽 |
| :--- | :--- | :--- |
| `FIND_TARGET:sign_1` | 가장 가까운 로봇 단독 수색, 나머지 배터리순 복귀 | `/swarm/fleet_cmd` |
| `RETURN_TO_ORIGIN` | 전체 배터리 오름차순 복귀 (4초 간격) | `/swarm/fleet_cmd` |
| `FORCE_RETURN_ALL` | 즉시 전체 복귀 (우선순위 무시) | `/swarm/direct_cmd` |
| `PATROL_ALL` | 전체 탐색 재개 (실패셀 초기화) | `/swarm/direct_cmd` |

**`FIND_TARGET` 처리 흐름 예시 (3대 로봇):**

```
① 목표 포인트 조회: "sign_1" → (100.0m, 50.0m)

② 거리 계산:  robot1: 85m ← 가장 가까움
               robot2: 142m
               robot3: 210m

③ 단독 수색:  → SOLO_SEARCH:robot1:sign_1:100.00:50.00

④ 배터리순 복귀:
     robot3 (배터리 32%) → RETURN_NOW:robot3        (즉시)
     robot2 (배터리 67%) → RETURN_DELAY:robot2:4.0  (4초 후)
```

**파라미터:**

| 파라미터 | 기본값 | 설명 |
| :--- | :--- | :--- |
| `target_points` | `1:100.0:50.0:sign_1,...` | 목표 포인트 목록 (`id:x:y:label`) |
| `return_stagger_sec` | `4.0` | 복귀 출발 간격 (초) |
| `robot_timeout_sec` | `8.0` | 로봇 상태 타임아웃 (초) |

---

### `control_center.py` — matplotlib 관재 GUI (신규)

**레이아웃:**
```
┌────────────────────────────┬─────────────┬─────────────┐
│                            │  로봇 상태  │  경유지 목록 │
│   64×64 Grid Map           │  ns / 배터리│  ①②③...   │
│   (클릭으로 로봇 선택·경유지) │  / 상태    │             │
│                            ├─────────────┴─────────────┤
│                            │  [전체복귀][순찰시작]       │
│                            │  [긴급정지][탐색재개]       │
│                            │  [경로실행][경유지삭제]     │
└────────────────────────────┴───────────────────────────┘
```

**마우스 조작:**

| 조작 | 동작 |
| :--- | :--- |
| 로봇 원 **왼쪽 클릭** | 해당 로봇 선택 (노란 테두리) + `STOP:{ns}` 즉시 발행 |
| 빈 맵 왼쪽 클릭 | 선택 로봇에 **경유지 추가** (◆ + 번호 표시) |
| 빈 맵 **더블클릭** | 선택 로봇 해당 좌표로 즉시 이동 (`GOTO:{ns}:wx:wy`) |
| **우클릭** | 선택 로봇 경유지 전체 삭제 |

**버튼:**

| 버튼 | 발행 명령 | 토픽 |
| :--- | :--- | :--- |
| 전체 복귀 | `FORCE_RETURN_ALL` | `/swarm/direct_cmd` |
| 순찰 시작 | `PATROL_ALL` | `/swarm/direct_cmd` |
| 긴급 정지 | `EMERGENCY_STOP` | `/mission/command` |
| 탐색 재개 | `RESUME_SEARCH` | `/mission/command` |
| 경로 실행 | `WAYPOINTS:{ns}:wx1,wy1;wx2,wy2;...` | `/swarm/direct_cmd` |
| 경유지 삭제 | (내부 초기화) | — |

**시각화:**

| 색상 | 의미 |
| :--- | :--- |
| 흰색 | UNKNOWN (미탐색) |
| 연녹색 | FREE (이동 가능) |
| 진회색 | WALL (장애물) |
| 연주황 | BUFFER (팽창 영역) |
| 로봇 원 | 로봇별 고유 색상 (최대 6대) |
| 노란 테두리 | 선택된 로봇 강조 |
| ◆ 다이아몬드 | 경유지 + 번호 + 점선 연결 |

---

### `ollama_commander.py` — 자연어 명령 해석

| 입력 예시 | 변환 명령 |
| :--- | :--- |
| "1번 표지판을 찾아" | `FIND_TARGET:sign_1` |
| "강아지 있는지 수색해" | `FIND_TARGET:dog` |
| "다들 원점으로 돌아와" | `RETURN_TO_ORIGIN` |
| "긴급 정지" | `EMERGENCY_STOP` |
| "상태 보고해줘" | `STATUS_REPORT` |

---

### `pinky_bringup_node.cpp` — C++ 고성능 Bringup

| 항목 | Python `bringup.py` | C++ `pinky_bringup_node` |
| :--- | :--- | :--- |
| 제어 주기 | 30 Hz | **50 Hz** |
| 타이머 정밀도 | GIL 영향 있음 | `wall_timer` 고정밀 |
| 직렬 통신 | Python Dynamixel SDK | C++ Dynamixel SDK |
| 스레드 모델 | 단일 | `MultiThreadedExecutor` |
| 토픽 호환성 | — | 완전 하위 호환 |

---

## 토픽 전체 목록

### 로봇별 (네임스페이스 `/{ns}/`)

| 토픽 | 타입 | 발행자 | 설명 |
| :--- | :--- | :--- | :--- |
| `/{ns}/odom` | `Odometry` | Nav2 | 위치·방향 |
| `/{ns}/cmd_vel` | `Twist` | swarm_coord | 긴급 정지용 속도 0 발행 |
| `/{ns}/battery/present` | `Float32` | bringup | 배터리 % (0~100) |
| `/{ns}/camera/image_raw` | `Image` | bringup | 카메라 영상 |
| `/{ns}/navigate_to_pose` | `Action` | swarm_coord | Nav2 목표 전달 |

### 공유 글로벌

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
| :--- | :--- | :--- | :--- | :--- |
| `/swarm/map_update` | `Int32MultiArray` | swarm_coord | 모든 swarm_coord, control_center | 12비트 그리드 동기화 `[robot_id, gx, gy, state]` |
| `/swarm/robot_pose` | `Float32MultiArray` | swarm_coord | 모든 swarm_coord, control_center | 위치·yaw `[robot_id, x, y, yaw]` |
| `/swarm/robot_status` | `String(JSON)` | swarm_coord | fleet_manager, control_center | 배터리·상태·위치 2Hz |
| `/swarm/fleet_cmd` | `String` | fleet_manager | swarm_coord | SOLO_SEARCH / RETURN_NOW / RETURN_DELAY |
| `/swarm/direct_cmd` | `String` | fleet_manager, control_center | swarm_coord | FORCE_RETURN_ALL / PATROL_ALL / STOP / GOTO / WAYPOINTS |
| `/swarm/target_alert` | `String` | yolo_detector | swarm_coord | 탐지 경보 |
| `/mission/command` | `String` | telegram, ollama, control_center | fleet_manager, swarm_coord | 운영자 명령 |
| `/mission/target_found` | `String(JSON)` | yolo_detector | telegram_reporter, mission_briefer | 탐지 이벤트 상세 |
| `/mission/status_update` | `String` | swarm_coord | telegram_reporter | 상태 보고 |
| `/mission/briefing` | `String` | mission_briefer | telegram_reporter | sLLM 브리핑 텍스트 |
| `/mission/coverage` | `Float32` | mission_briefer | — | 탐색 커버리지 (0~1) |
| `/mission/llm_response` | `String` | ollama_commander | — | LLM 원문 응답 |
| `/mission/text_command` | `String` | 외부 입력 | ollama_commander | 텍스트 자연어 명령 |
| `/mission/voice_command` | `String` | STT 노드 | ollama_commander | 음성 인식 결과 |

### `direct_cmd` 명령어 포맷

| 명령 포맷 | 설명 |
| :--- | :--- |
| `FORCE_RETURN_ALL` | 모든 로봇 즉시 복귀 (배터리 우선순위 무시) |
| `PATROL_ALL` | 모든 로봇 탐색 재개 (실패 셀 초기화) |
| `STOP:{ns}` | 해당 로봇 즉시 정지 → PAUSED 상태 |
| `GOTO:{ns}:{wx}:{wy}` | 해당 로봇 좌표로 즉시 Nav2 이동 |
| `WAYPOINTS:{ns}:wx1,wy1;wx2,wy2;...` | 해당 로봇 경유지 큐 순서대로 주행 |

---

## 설치 및 빌드

### 의존성 설치

```bash
# Python 패키지
pip install ultralytics opencv-python requests numpy matplotlib

# ROS2 패키지
sudo apt install ros-jazzy-cv-bridge ros-jazzy-nav2-msgs

# Ollama sLLM 서버
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3       # 기본 (~4GB)
ollama pull qwen2        # 한국어 최적화 (~4GB)
ollama pull gemma3       # 경량 (~2GB)
```

### ROS2 빌드

```bash
cd /home/cho-pc/dev_ws/pinky/pinky_pro

colcon build --packages-select pinky_mission
colcon build --packages-select pinky_bringup_cpp
source install/setup.bash
```

---

## 실행 방법

### 1. 로봇 하드웨어 스택 (로봇별 실행)

```bash
# robot1 Dynamixel + 오도메트리
ros2 run pinky_bringup_cpp pinky_bringup_node \
    --ros-args \
    --remap /odom:=/robot1/odom \
    --remap /cmd_vel:=/robot1/cmd_vel \
    --remap /joint_states:=/robot1/joint_states

# robot1 Nav2
ros2 launch nav2_bringup bringup_launch.py \
    namespace:=/robot1 use_namespace:=true \
    params_file:=<nav2_params.yaml>

# robot2, robot3 도 동일하게 /robot2, /robot3 네임스페이스 적용
```

### 2. 미션 스택 전체 런치

```bash
# Ollama 서버 (백그라운드)
ollama serve &

# 2대 군집 기본 실행
ros2 launch pinky_mission mission_launch.py \
    bot_token:=<YOUR_BOT_TOKEN> \
    chat_id:=<YOUR_CHAT_ID>

# 3대 군집, 커스텀 목표 포인트
ros2 launch pinky_mission mission_launch.py \
    num_robots:=3 \
    bot_token:=<TOKEN> \
    chat_id:=<ID> \
    target_points:="1:100.0:50.0:sign_1,2:250.0:75.0:sign_2,3:400.0:50.0:sign_3" \
    return_stagger_sec:=4.0 \
    ollama_model:=llama3 \
    brief_interval_sec:=120.0
```

### 3. 관재 GUI (별도 터미널)

```bash
ros2 run pinky_mission control_center \
    --ros-args -p world_width:=500.0 -p world_height:=150.0
```

### 4. 개별 노드 (디버그/개발)

```bash
ros2 run pinky_mission swarm_coordinator --ros-args \
    -p robot_namespace:=robot1 -p robot_id:=1 -p num_robots:=2

ros2 run pinky_mission fleet_manager --ros-args \
    -p target_points:="1:100.0:50.0:sign_1,2:250.0:75.0:sign_2"

ros2 run pinky_mission yolo_detector --ros-args \
    -p robot_namespace:=robot1

ros2 run pinky_mission ollama_commander --ros-args \
    -p ollama_model:=llama3

ros2 run pinky_mission mission_briefer --ros-args \
    -p brief_interval_sec:=60.0
```

---

## 명령어 레퍼런스

### 텔레그램 명령

| 메시지 | 동작 |
| :--- | :--- |
| `원점 복귀` | 전체 배터리 순 우선 복귀 |
| `긴급 정지` | 전체 즉시 정지 |
| `수색 재개` | 정지 후 탐색 재시작 |
| `상태 보고` | 위치·배터리·커버리지 보고 |

### 자연어 명령 (터미널 직접 테스트)

```bash
ros2 topic pub --once /mission/text_command std_msgs/msg/String \
    "data: '1번 표지판을 찾아'"

ros2 topic pub --once /mission/text_command std_msgs/msg/String \
    "data: '다들 원점으로 돌아와'"
```

### 관재 명령 직접 발행 (터미널 테스트)

```bash
# 특정 로봇 즉시 정지
ros2 topic pub --once /swarm/direct_cmd std_msgs/msg/String \
    "data: 'STOP:robot1'"

# 특정 좌표로 즉시 이동
ros2 topic pub --once /swarm/direct_cmd std_msgs/msg/String \
    "data: 'GOTO:robot1:100.0:50.0'"

# 경유지 순서 주행
ros2 topic pub --once /swarm/direct_cmd std_msgs/msg/String \
    "data: 'WAYPOINTS:robot1:100.0,50.0;200.0,75.0;300.0,50.0'"

# 전체 즉시 복귀
ros2 topic pub --once /swarm/direct_cmd std_msgs/msg/String \
    "data: 'FORCE_RETURN_ALL'"

# 전체 순찰 재개
ros2 topic pub --once /swarm/direct_cmd std_msgs/msg/String \
    "data: 'PATROL_ALL'"
```

### 모니터링

```bash
ros2 topic echo /swarm/robot_status      # 로봇 상태 JSON
ros2 topic echo /swarm/fleet_cmd         # 우선순위 명령 흐름
ros2 topic echo /swarm/direct_cmd        # 관재 직접 명령 흐름
ros2 topic echo /mission/coverage        # 탐색 커버리지
ros2 topic echo /mission/briefing        # sLLM 브리핑
ros2 topic echo /mission/target_found    # 탐지 이벤트
```

---

## 미션 시나리오

### 시나리오 1: 자율 수색 → 목표 발견 → 우선순위 복귀

```
[시작] 3대 로봇 배치 → 원점(0,0) 자동 기록
  ↓
[탐색] 구역 자동 분할 (robot1: X 0~21, robot2: X 21~42, robot3: X 42~63)
       LiDAR → 12비트 그리드 갱신 (UNKNOWN→FREE/WALL)
  ↓
[명령] "1번 표지판을 찾아" (텔레그램 or 자연어)
  ↓
[결정] fleet_manager:
       · robot2가 sign_1(100m,50m)에서 85m → 단독 수색 지정
       · robot3 배터리 28% → RETURN_NOW 즉시 복귀
       · robot1 배터리 61% → RETURN_DELAY 4초 후 복귀
  ↓
[발견] robot2: YOLOv8 탐지 → 텔레그램: 사진 + [BC]-[DE] 위치 전송
[브리핑] mission_briefer: Ollama로 한국어 상황 보고 자동 생성
```

### 시나리오 2: 관재 GUI로 개별 제어

```
[GUI 실행]  ros2 run pinky_mission control_center
  ↓
[로봇 선택] robot2 원 클릭 → 노란 테두리 + 즉시 정지
  ↓
[경유지 지정] 빈 맵 3회 클릭 → ①②③ 추가
  ↓
[경로 실행] 버튼 클릭
            → WAYPOINTS:robot2:x1,y1;x2,y2;x3,y3
            → robot2: ①→②→③ 순서 Nav2 주행
```

### 시나리오 3: 배터리 순 전체 복귀

```
"원점 복귀" 명령
  ↓
fleet_manager 배터리 정렬:
  robot3: 18% → RETURN_NOW  (즉시)
  robot1: 45% → RETURN_DELAY 4.0초
  robot2: 72% → RETURN_DELAY 8.0초
  ↓
4초 간격 순차 출발 → Nav2 충돌 방지
```

---

## 시뮬레이션 실행

```bash
cd /home/cho-pc/dev_ws/pinky/pinky_test
python3 pinky_12bit_slam.py
```

포함 기능: 12비트 SLAM 64×64 · LiDAR 레이캐스팅 · 팽창 버퍼 · BFS · GUA 코드 표시 · 마우스 장애물 배치

---

## Nav2 최적화 파라미터

파일: `pinky_navigation/params/nav2_inflation_params.yaml`

| 파라미터 | 값 | 설명 |
| :--- | :--- | :--- |
| `robot_radius` | 0.15 m | Pinky 물리 반경 |
| `inflation_radius` | 0.35 m | 충돌 방지 팽창 반경 |
| `cost_scaling_factor` | 3.0 | 경로 부드러움 |
| `max_linear_accel` | 0.5 m/s² | 선형 가감속 제한 |
| `max_angular_accel` | 2.5 rad/s² | 회전 가감속 제한 |
| `controller_frequency` | 20 Hz | 제어 루프 주기 |
