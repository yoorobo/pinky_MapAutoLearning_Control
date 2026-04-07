#!/usr/bin/env python3
"""
Pinky Robot 12-bit Self-Learning SLAM + Fleet Mission Dashboard
================================================================
Gemini 대화 기반 최종 통합 구현체

포함 기능:
  1. 12-bit (4096 단계) 점유 격자 지도 (64x64 = 4096칸)
  2. LiDAR 스캔 실시간 마킹 (미지 탐색 → 자가 학습)
  3. 장애물 팽창(Inflation) 버퍼 (원형 몸체 의식)
  4. 원점(Origin) 추적 및 복귀 로직
  5. Short/Long Lookahead 탐지봉 시각화
  6. 군집 주행 (2대 이상 로봇)
  7. 중앙 관제 대시보드 (통합 맵 + 상태 + 로그)
  8. LAYER/CODE/BIN/DEC/OCT/HEX 실시간 데이터 표시

실행 방법:
  - Jupyter: 첫 줄 주석 해제 후 실행 (%matplotlib widget)
  - 터미널: python3 pinky_12bit_slam.py
"""

# Jupyter에서 실행 시 아래 줄 주석 해제:
# %matplotlib widget

import matplotlib
# matplotlib.use('TkAgg')  # 터미널 실행 시 필요한 경우 주석 해제

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager, rc
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import matplotlib.patches as patches
import math
import os
from collections import deque

# ============================================================
# 1. 폰트 설정 (한글 지원)
# ============================================================
font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if os.path.exists(font_path):
    font_prop = font_manager.FontProperties(fname=font_path)
    rc('font', family=font_prop.get_name())
else:
    plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['axes.unicode_minus'] = False

# ============================================================
# 2. 12-bit 코드 시스템 (AA ~ HH, 64개)
#    6비트를 상위 3비트 + 하위 3비트로 나누어 각각 A-H 매핑
# ============================================================
CHARS = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
GUA_CODES = {i: f"{CHARS[(i >> 3) & 0x07]}{CHARS[i & 0x07]}" for i in range(64)}

def get_full_info(val_6bit: int) -> dict:
    """6비트 값을 CODE/BIN/DEC/OCT/HEX 딕셔너리로 변환"""
    v = max(0, min(63, int(val_6bit)))
    return {
        'code': GUA_CODES[v],
        'bin':  format(v, '06b'),
        'dec':  v,
        'oct':  format(v, '02o'),
        'hex':  format(v, '02X'),
    }

def angle_to_12bit(angle_deg: float) -> int:
    """각도(도)를 12비트 펄스값(0~4095)으로 변환 (90° 오프셋 포함)"""
    pulse = int((((90.0 - angle_deg) % 360.0) / 360.0) * 4096)
    return min(pulse, 4095)

def pos_to_grid(x: float, y: float, world_w: float = 500.0, world_h: float = 150.0) -> tuple:
    """월드 좌표 → 64×64 그리드 인덱스 변환"""
    gx = int(np.clip(x / world_w * 64, 0, 63))
    gy = int(np.clip(y / world_h * 64, 0, 63))
    return gx, gy

def grid_to_code(gx: int, gy: int) -> str:
    """그리드 인덱스 → 12-bit 주소 코드 문자열"""
    return f"[{GUA_CODES[gx]}]-[{GUA_CODES[gy]}]"

# ============================================================
# 3. 핵심 시뮬레이션 엔진: 12-bit SLAM + 탐지봉 + 회피
# ============================================================
class PinkySlamEngine:
    """
    Pinky 로봇 단일 에이전트 시뮬레이션 엔진
    - 12비트 점유 격자 자가 학습
    - LiDAR 레이캐스팅
    - 장애물 팽창 버퍼
    - 원점 기억 및 복귀
    """

    # 그리드 셀 상태값
    UNKNOWN   = 0  # 미탐색
    FREE      = 1  # 이동 가능
    WALL      = 2  # 벽/장애물
    BUFFER    = 3  # 팽창 버퍼 (접근 금지)
    ROBOT     = 4  # 다른 로봇 위치

    def __init__(self, start_x: float, start_y: float, robot_id: int = 1,
                 custom_obstacles: list = None, zone_x: tuple = (0, 63)):
        # 원점 및 위치 (월드 좌표)
        self.origin      = np.array([start_x, start_y])
        self.pos         = np.array([start_x, start_y, 0.5])  # z=0.5m
        self.yaw         = 0.0    # 라디안
        self.vel         = 3.5    # 이동 속도
        self.robot_id    = robot_id
        self.zone_x      = zone_x  # 담당 수색 구역 (X축 범위)

        # 로봇 안전 반경 설정
        self.robot_radius   = 5.0
        self.safety_buffer  = 3.0
        self.inflation_r    = self.robot_radius + self.safety_buffer  # 8.0

        # 경로 및 목표
        self.waypoints = [
            [10.0,  10.0], [150.0, 10.0], [150.0, 120.0],
            [300.0, 120.0], [300.0, 10.0], [450.0, 10.0]
        ]
        self.goal = np.array([450.0, 10.0, 0.5])

        # 도로 벽 생성
        self.road_width  = 40.0
        self.left_wall, self.right_wall = self._generate_walls(self.waypoints, self.road_width)
        self.wall_segments = self._get_wall_segments()

        # 외부 장애물
        self.obstacles = custom_obstacles if custom_obstacles else []

        # 12비트 점유 격자 지도 (64×64)
        self.grid = np.zeros((64, 64), dtype=np.int8)

        # 탐지봉 타겟
        self.short_target = np.array([start_x, start_y])
        self.long_target  = np.array([start_x, start_y])

        # 12비트 데이터
        self.pulse_val = 0
        self.gua_l1    = get_full_info(0)
        self.gua_l2    = get_full_info(0)

        # 주행 상태
        self.earth_hex = 0b000000
        self.man_hex   = 0b000000
        self.mode      = "READY"
        self.finished  = False
        self.path_log  = []  # 주행 경로 기록

    # ----------------------------------------------------------
    # 도로 생성 유틸
    # ----------------------------------------------------------
    def _generate_walls(self, waypoints, width):
        lw, rw = [], []
        w = width / 2.0
        for i, pt in enumerate(waypoints):
            p = np.array(pt)
            if i == 0:
                d = np.array(waypoints[1]) - p
                d /= np.linalg.norm(d); n = np.array([-d[1], d[0]])
                lw.append(p + n*w); rw.append(p - n*w)
            elif i == len(waypoints) - 1:
                d = p - np.array(waypoints[-2])
                d /= np.linalg.norm(d); n = np.array([-d[1], d[0]])
                lw.append(p + n*w); rw.append(p - n*w)
            else:
                d1 = p - np.array(waypoints[i-1]); d1 /= np.linalg.norm(d1)
                d2 = np.array(waypoints[i+1]) - p; d2 /= np.linalg.norm(d2)
                tang = d1 + d2
                if np.linalg.norm(tang) < 1e-5:
                    n = np.array([-d1[1], d1[0]]); mlen = w
                else:
                    tang /= np.linalg.norm(tang)
                    n = np.array([-tang[1], tang[0]])
                    cos_t = np.dot(np.array([-d1[1], d1[0]]), n)
                    mlen = w / cos_t if abs(cos_t) > 0.1 else w
                lw.append(p + n*mlen); rw.append(p - n*mlen)
        return lw, rw

    def _get_wall_segments(self):
        segs = []
        for i in range(len(self.left_wall) - 1):
            segs.append((self.left_wall[i],  self.left_wall[i+1]))
            segs.append((self.right_wall[i], self.right_wall[i+1]))
        segs.append((self.left_wall[0],  self.right_wall[0]))
        segs.append((self.left_wall[-1], self.right_wall[-1]))
        return segs

    # ----------------------------------------------------------
    # 레이캐스팅 (LiDAR 시뮬레이션)
    # ----------------------------------------------------------
    def _raycast(self, origin, direction, max_dist=30.0):
        min_t, hit = max_dist, False
        ox, oy = origin[0], origin[1]
        dx, dy = direction[0], direction[1]

        # 벽 세그먼트 검사
        for p1, p2 in self.wall_segments:
            v1 = origin[:2] - np.array(p1[:2])
            v2 = np.array(p2[:2]) - np.array(p1[:2])
            v3 = np.array([-dy, dx])
            dot = np.dot(v2, v3)
            if abs(dot) < 1e-6: continue
            t1 = np.cross(v2, v1) / dot
            t2 = np.dot(v1, v3) / dot
            if t1 > 1e-4 and 0.0 <= t2 <= 1.0:
                if t1 < min_t: min_t, hit = t1, True

        # 원형 장애물 검사
        for obs in self.obstacles:
            cx, cy, r = obs['pos'][0], obs['pos'][1], obs['r']
            A = dx*dx + dy*dy
            B = 2.0*(dx*(ox-cx) + dy*(oy-cy))
            C = (ox-cx)**2 + (oy-cy)**2 - r*r
            det = B*B - 4.0*A*C
            if det >= 0.0:
                t = (-B - math.sqrt(det)) / (2.0*A)
                if 1e-4 < t < min_t: min_t, hit = t, True

        return hit, min_t

    # ----------------------------------------------------------
    # LiDAR 스캔 → 격자 마킹 (자가 학습)
    # ----------------------------------------------------------
    def update_lidar_and_map(self):
        """LiDAR 스캔 → 12비트 그리드 자가 학습"""
        angles     = np.linspace(0, 2*np.pi, 36, endpoint=False)
        lidar_range = 60.0

        # 현재 위치 FREE 마킹
        gx, gy = pos_to_grid(self.pos[0], self.pos[1])
        if 0 <= gx < 64 and 0 <= gy < 64:
            self.grid[gx, gy] = self.FREE

        for a in angles:
            direction = np.array([math.cos(self.yaw + a), math.sin(self.yaw + a)])
            hit, dist = self._raycast(self.pos, direction, lidar_range)

            for step in np.arange(0.5, min(dist, lidar_range), 0.5):
                tx = self.pos[0] + step * direction[0]
                ty = self.pos[1] + step * direction[1]
                pgx, pgy = pos_to_grid(tx, ty)
                if 0 <= pgx < 64 and 0 <= pgy < 64:
                    if self.grid[pgx, pgy] not in (self.WALL, self.BUFFER):
                        self.grid[pgx, pgy] = self.FREE

            if hit:
                hx = self.pos[0] + dist * direction[0]
                hy = self.pos[1] + dist * direction[1]
                pgx, pgy = pos_to_grid(hx, hy)
                if 0 <= pgx < 64 and 0 <= pgy < 64:
                    self.grid[pgx, pgy] = self.WALL
                    self._apply_inflation(pgx, pgy)

    def _apply_inflation(self, gx: int, gy: int, radius: int = 2):
        """장애물 주변 팽창 버퍼 적용 (원형 몸체 보호)"""
        for dx in range(-radius, radius+1):
            for dy in range(-radius, radius+1):
                if dx*dx + dy*dy <= radius*radius:
                    nx, ny = gx+dx, gy+dy
                    if 0 <= nx < 64 and 0 <= ny < 64:
                        if self.grid[nx, ny] in (self.UNKNOWN, self.FREE):
                            self.grid[nx, ny] = self.BUFFER

    # ----------------------------------------------------------
    # Lookahead 타겟 계산 (탐지봉)
    # ----------------------------------------------------------
    def _get_lookahead_targets(self, pos, dist_short=20.0, dist_long=50.0):
        min_dist, best_i, best_proj = 1e9, 0, pos[:2]
        for i in range(len(self.waypoints) - 1):
            p1 = np.array(self.waypoints[i])
            p2 = np.array(self.waypoints[i+1])
            v = p2 - p1; l = np.linalg.norm(v)
            if l < 1e-6: continue
            u = v / l
            t = np.clip(np.dot(pos[:2] - p1, u), 0.0, l)
            proj = p1 + u * t
            d = np.linalg.norm(pos[:2] - proj)
            if d < min_dist:
                min_dist, best_i, best_proj = d, i, proj

        def find_pt(start_p, start_i, target_d):
            cp, ci, rem = start_p.copy(), start_i, target_d
            while ci < len(self.waypoints) - 1:
                p2 = np.array(self.waypoints[ci+1])
                dn = np.linalg.norm(p2 - cp)
                if rem <= dn:
                    u = (p2 - cp) / dn if dn > 0 else np.array([1.0, 0.0])
                    return cp + u * rem
                rem -= dn; ci += 1
                if ci < len(self.waypoints):
                    cp = np.array(self.waypoints[ci])
            return np.array(self.waypoints[-1])

        return (find_pt(best_proj, best_i, dist_short),
                find_pt(best_proj, best_i, dist_long))

    # ----------------------------------------------------------
    # 메인 업데이트 루프
    # ----------------------------------------------------------
    def update(self):
        if self.finished: return

        # 목표 도달 확인
        if np.linalg.norm(self.pos[:2] - self.goal[:2]) < 10.0:
            self.mode = "GOAL REACHED"; self.finished = True; return

        # LiDAR 학습
        self.update_lidar_and_map()

        # Lookahead 타겟 계산
        self.short_target, self.long_target = self._get_lookahead_targets(self.pos)

        # 12비트 펄스 계산
        target_yaw = math.atan2(
            self.long_target[1] - self.pos[1],
            self.long_target[0] - self.pos[0]
        )
        target_yaw_deg = math.degrees(target_yaw)
        self.pulse_val = angle_to_12bit(target_yaw_deg)
        self.gua_l1 = get_full_info((self.pulse_val >> 6) & 0x3F)  # 상위 6비트
        self.gua_l2 = get_full_info(self.pulse_val & 0x3F)          # 하위 6비트

        # 6방향 LiDAR 장애물 감지 (earth_hex)
        self.earth_hex = 0
        for i, a in enumerate(np.linspace(0, 2*np.pi, 6, endpoint=False)):
            hit, dist = self._raycast(
                self.pos,
                np.array([math.cos(self.yaw + a), math.sin(self.yaw + a)]),
                30.0
            )
            if hit: self.earth_hex |= (1 << i)

        # 인간 판단 (man_hex): 기억된 위험 누적
        self.man_hex = (self.man_hex | self.earth_hex)
        if np.random.rand() > 0.8:
            self.man_hex &= 0b111110

        # 조향 결정
        short_yaw = math.atan2(
            self.short_target[1] - self.pos[1],
            self.short_target[0] - self.pos[0]
        )
        if (self.earth_hex | self.man_hex) & 0b000001:
            self.mode = "AVOIDANCE"
            free_dirs = ~(self.earth_hex | self.man_hex) & 0x3F
            if free_dirs & 0b000110:
                steer_yaw = self.yaw + 0.4
            elif free_dirs & 0b110000:
                steer_yaw = self.yaw - 0.4
            else:
                steer_yaw = self.yaw + 0.5
        else:
            self.mode = "NAVIGATING"
            # Long + Short 혼합 조향
            cx = 0.6*math.cos(target_yaw) + 0.4*math.cos(short_yaw)
            cy = 0.6*math.sin(target_yaw) + 0.4*math.sin(short_yaw)
            steer_yaw = math.atan2(cy, cx)

        # 각도 업데이트
        diff = (steer_yaw - self.yaw + math.pi) % (2*math.pi) - math.pi
        self.yaw += np.clip(diff * 0.35, -0.25, 0.25)

        # 위치 업데이트 (충돌 전방 검사)
        fwd = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        hit, dist = self._raycast(self.pos, fwd, self.vel + 1.0)
        if hit and dist < self.vel:
            self.mode = "BRAKING"
            self.yaw += 0.4
        else:
            next_pos = self.pos[:2] + fwd * self.vel
            self.pos[:2] = next_pos
            self.path_log.append(tuple(self.pos[:2]))

# ============================================================
# 4. 군집 관리자 (Fleet Manager)
# ============================================================
class PinkyFleetManager:
    """
    2대 이상 Pinky 로봇 군집 관리
    - 공유 통합 지도 (global_grid)
    - 구역 분할 수색
    - 타겟 발견 로그
    """
    def __init__(self, num_robots: int = 2, custom_obstacles: list = None):
        self.num_robots = num_robots
        self.target_log = []
        self.mission_log = []

        # 공유 통합 12비트 지도
        self.global_grid = np.zeros((64, 64), dtype=np.int8)

        # 각 로봇 초기화 (구역 분할)
        obstacles = custom_obstacles or []
        zone_width = 64 // num_robots
        starts_x = [25.0 + i * 150.0 for i in range(num_robots)]

        self.engines = []
        for i in range(num_robots):
            zone = (i * zone_width, min((i+1)*zone_width - 1, 63))
            eng = PinkySlamEngine(
                start_x=starts_x[min(i, len(starts_x)-1)],
                start_y=10.0,
                robot_id=i+1,
                custom_obstacles=obstacles,
                zone_x=zone
            )
            self.engines.append(eng)

    def update_all(self):
        for eng in self.engines:
            if not eng.finished:
                eng.update()
                # 지도 병합 (OR 연산으로 학습 데이터 통합)
                mask = eng.grid > 0
                self.global_grid[mask] = np.maximum(
                    self.global_grid[mask], eng.grid[mask]
                )

                # 타겟 발견 시뮬레이션
                if np.random.rand() > 0.998:
                    gx, gy = pos_to_grid(eng.pos[0], eng.pos[1])
                    addr = grid_to_code(gx, gy)
                    target = {
                        'id':    len(self.target_log) + 1,
                        'robot': eng.robot_id,
                        'addr':  addr,
                        'pos':   eng.pos[:2].copy(),
                    }
                    self.target_log.append(target)
                    self.mission_log.append(
                        f"🚨 Pinky-{eng.robot_id}: Target at {addr}"
                    )
                    eng.mode = "TARGET SPOTTED"

    @property
    def all_finished(self):
        return all(e.finished for e in self.engines)

# ============================================================
# 5. 중앙 관제 대시보드 GUI
# ============================================================
class CommandCenterGUI:
    """
    Pinky 군집 주행 통합 관제 대시보드

    Layout:
      [좌] 통합 12비트 SLAM 지도 (탐지봉 포함)
      [우상] 로봇별 실시간 상태 + 12비트 데이터 테이블
      [우하] 미션 타겟 발견 로그
    """

    ROBOT_COLORS = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6']

    def __init__(self, num_robots: int = 2, custom_obstacles: list = None):
        self.fleet  = PinkyFleetManager(num_robots, custom_obstacles)
        self.frame  = 0

        # Figure 레이아웃
        self.fig = plt.figure(figsize=(15, 8))
        self.fig.patch.set_facecolor('#1a1a2e')
        self.fig.subplots_adjust(left=0.05, right=0.97, bottom=0.12,
                                  top=0.92, wspace=0.35, hspace=0.4)

        # 좌측: 통합 SLAM 지도
        self.ax_map = self.fig.add_subplot(1, 2, 1)

        # 우측 상단: 로봇 상태
        self.ax_status = self.fig.add_subplot(2, 2, 2)

        # 우측 하단: 미션 로그
        self.ax_log = self.fig.add_subplot(2, 2, 4)

        self._style_axes()

        # 버튼
        self.ax_btn_start = plt.axes([0.42, 0.02, 0.10, 0.05])
        self.ax_btn_stop  = plt.axes([0.54, 0.02, 0.10, 0.05])
        self.btn_start = Button(self.ax_btn_start, '수색 시작', color='#27ae60', hovercolor='#2ecc71')
        self.btn_stop  = Button(self.ax_btn_stop,  '긴급 정지', color='#c0392b', hovercolor='#e74c3c')
        self.btn_start.on_clicked(self._start)
        self.btn_stop.on_clicked(self._stop)

        self.running = False
        self.ani     = None

        # 2D 마커 초기화
        self._init_markers()

        # 초기 지도 그리기
        self._draw_initial_map()

    def _style_axes(self):
        for ax in [self.ax_map, self.ax_status, self.ax_log]:
            ax.set_facecolor('#16213e')
            for spine in ax.spines.values():
                spine.set_edgecolor('#0f3460')

    def _init_markers(self):
        """로봇 마커 및 탐지봉 초기화"""
        self.robot_markers  = []
        self.robot_buffers  = []
        self.short_lines    = []
        self.long_lines     = []

        for i, eng in enumerate(self.fleet.engines):
            color = self.ROBOT_COLORS[i % len(self.ROBOT_COLORS)]
            m, = self.ax_map.plot([], [], 'o', color=color, markersize=10,
                                   zorder=10, label=f'Pinky-{eng.robot_id}')
            buf = patches.Circle((0, 0), eng.inflation_r,
                                  fill=False, color=color, ls='--',
                                  alpha=0.4, linewidth=1.5, zorder=9)
            self.ax_map.add_patch(buf)
            sl, = self.ax_map.plot([], [], '-', color='#ff6b6b', lw=1.5,
                                    alpha=0.8, zorder=8)
            ll, = self.ax_map.plot([], [], '-', color='#51cf66', lw=1.5,
                                    alpha=0.8, zorder=8)
            self.robot_markers.append(m)
            self.robot_buffers.append(buf)
            self.short_lines.append(sl)
            self.long_lines.append(ll)

    def _draw_initial_map(self):
        """초기 2D 지도 베이스 그리기"""
        self.ax_map.clear()
        self.ax_map.set_facecolor('#16213e')
        self.ax_map.set_xlim(-20, 480)
        self.ax_map.set_ylim(-20, 160)
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, linestyle='--', alpha=0.15, color='white')

        # 도로
        wp = self.fleet.engines[0].waypoints
        px, py = zip(*wp)
        self.ax_map.plot(px, py, color='#555', lw=35, alpha=0.3, solid_capstyle='round')
        self.ax_map.plot(px, py, color='#888', ls='--', lw=1.0, alpha=0.6)

        # 출발지 / 목적지
        self.ax_map.scatter(25, 10, c='#3498db', s=120, zorder=11, label='출발지')
        self.ax_map.scatter(450, 10, c='#f1c40f', s=200, marker='*', zorder=11, label='목적지')

        self.ax_map.set_title("Pinky Fleet 12-bit SLAM Map", fontsize=12,
                               color='white', pad=8)
        self.ax_map.legend(loc='upper left', fontsize=8,
                            facecolor='#1a1a2e', edgecolor='gray',
                            labelcolor='white')

        # 마커 재배치
        self._init_markers()

    def _start(self, event):
        if self.running: return
        self.running = True
        self.ani = FuncAnimation(
            self.fig, self._update_frame,
            frames=2000, interval=60, repeat=False
        )
        self.fig.canvas.draw_idle()

    def _stop(self, event):
        self.running = False
        if self.ani:
            self.ani.event_source.stop()
        for eng in self.fleet.engines:
            eng.mode = "EMERGENCY STOP"

    def _update_frame(self, frame_idx):
        if not self.running or self.fleet.all_finished:
            return

        self.fleet.update_all()
        self.frame += 1

        # ----- 지도 업데이트 -----
        # 12비트 격자 오버레이 (imshow를 clear 없이 효율적으로 갱신)
        if not hasattr(self, '_map_img'):
            self._map_img = self.ax_map.imshow(
                self.fleet.global_grid.T, origin='lower',
                extent=[0, 500, 0, 150],
                cmap='Blues', alpha=0.45, vmin=0, vmax=4, zorder=1
            )
        else:
            self._map_img.set_data(self.fleet.global_grid.T)

        # 로봇 마커 + 탐지봉 업데이트
        for i, eng in enumerate(self.fleet.engines):
            if eng.finished: continue
            x, y = eng.pos[0], eng.pos[1]
            self.robot_markers[i].set_data([x], [y])
            self.robot_buffers[i].center = (x, y)
            self.short_lines[i].set_data(
                [x, eng.short_target[0]], [y, eng.short_target[1]]
            )
            self.long_lines[i].set_data(
                [x, eng.long_target[0]], [y, eng.long_target[1]]
            )

        # ----- 상태 패널 업데이트 -----
        self.ax_status.clear()
        self.ax_status.set_facecolor('#16213e')
        self.ax_status.axis('off')

        status_lines = ["【 PINKY FLEET 실시간 상태 】\n"]
        for i, eng in enumerate(self.fleet.engines):
            color = self.ROBOT_COLORS[i % len(self.ROBOT_COLORS)]
            gx, gy = pos_to_grid(eng.pos[0], eng.pos[1])
            l1, l2 = eng.gua_l1, eng.gua_l2
            txt = (
                f"Pinky-{eng.robot_id}  [{eng.mode}]\n"
                f"  Addr : {grid_to_code(gx, gy)}\n"
                f"  Pulse: {eng.pulse_val:04d} / 4095\n"
                f"  {'='*36}\n"
                f"  {'LAYER':<6}|{'CODE':<5}|{'BIN':<7}|{'DEC':<4}|{'OCT':<4}|{'HEX'}\n"
                f"  {'-'*36}\n"
                f"  L1(H)  |{l1['code']:<5}|{l1['bin']:<7}|{l1['dec']:<4}|{l1['oct']:<4}|{l1['hex']}\n"
                f"  L2(L)  |{l2['code']:<5}|{l2['bin']:<7}|{l2['dec']:<4}|{l2['oct']:<4}|{l2['hex']}\n"
            )
            status_lines.append(txt)

        full_status = "\n".join(status_lines)
        self.ax_status.text(
            0.02, 0.98, full_status,
            transform=self.ax_status.transAxes,
            fontsize=7.5, family='monospace', va='top', color='#ecf0f1',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#0d1b2a',
                      edgecolor='#3498db', alpha=0.9)
        )
        self.ax_status.set_title(f"Frame: {self.frame}", fontsize=9,
                                  color='#bdc3c7', pad=4)

        # ----- 미션 로그 패널 -----
        self.ax_log.clear()
        self.ax_log.set_facecolor('#16213e')
        self.ax_log.axis('off')

        log_header = "【 미션 타겟 발견 로그 】\n"
        recent_logs = self.fleet.mission_log[-8:] if self.fleet.mission_log else ["(탐색 중...)"]
        log_text = log_header + "\n".join(recent_logs)

        if self.fleet.target_log:
            log_text += f"\n\n총 발견: {len(self.fleet.target_log)}개"
            for t in self.fleet.target_log[-3:]:
                log_text += f"\n  #{t['id']} by Pinky-{t['robot']} @ {t['addr']}"

        self.ax_log.text(
            0.02, 0.98, log_text,
            transform=self.ax_log.transAxes,
            fontsize=8, family='monospace', va='top', color='#ff6b6b',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#0d1b2a',
                      edgecolor='#e74c3c', alpha=0.9)
        )
        self.ax_log.set_title(
            f"탐색률: {np.count_nonzero(self.fleet.global_grid > 0) / 40.96:.1f}%",
            fontsize=9, color='#bdc3c7', pad=4
        )

        # 타이틀 업데이트
        modes = [e.mode for e in self.fleet.engines]
        self.fig.suptitle(
            f"Pinky Fleet Command Center  |  Robots: {self.fleet.num_robots}  |  "
            f"Targets: {len(self.fleet.target_log)}",
            fontsize=13, color='white', y=0.97
        )


# ============================================================
# 6. 단독 SLAM 시뮬레이션 (1대용 상세 대시보드)
# ============================================================
class SingleRobotDashboard:
    """
    단일 로봇 상세 SLAM 대시보드
    - 마우스 클릭으로 장애물 배치
    - 2D 지도 + 3D 뷰
    - 코드/BIN/DEC/OCT/HEX 테이블
    """
    def __init__(self):
        self.fig = plt.figure(figsize=(14, 7))
        self.fig.patch.set_facecolor('#1a1a2e')
        self.fig.subplots_adjust(bottom=0.12, left=0.05, right=0.97,
                                  top=0.93, wspace=0.3)

        self.ax_2d = self.fig.add_subplot(1, 2, 1)
        self.ax_3d = self.fig.add_subplot(1, 2, 2, projection='3d')
        self.ax_2d.set_facecolor('#16213e')

        self.custom_obstacles = []
        self.sim  = None
        self.ani  = None
        self.running = False

        # 버튼
        self.ax_btn = plt.axes([0.44, 0.02, 0.12, 0.055])
        self.btn = Button(self.ax_btn, '주행 시작', color='#27ae60', hovercolor='#2ecc71')
        self.btn.on_clicked(self._start)

        # 마우스 클릭 이벤트
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self._on_click)

        self._draw_base()

    def _draw_base(self):
        self.ax_2d.clear()
        self.ax_2d.set_facecolor('#16213e')
        self.ax_2d.set_xlim(-20, 480); self.ax_2d.set_ylim(-20, 160)
        self.ax_2d.set_aspect('equal')
        self.ax_2d.grid(True, ls='--', alpha=0.15, color='white')

        wp = [[10,10],[150,10],[150,120],[300,120],[300,10],[450,10]]
        px, py = zip(*wp)
        self.ax_2d.plot(px, py, color='#555', lw=30, alpha=0.3, solid_capstyle='round')
        self.ax_2d.plot(px, py, color='#888', ls='--', lw=1)
        self.ax_2d.scatter(25, 10,  c='#3498db', s=80,  zorder=10, label='Origin')
        self.ax_2d.scatter(450, 10, c='#f1c40f', s=150, zorder=10, marker='*', label='Goal')
        self.ax_2d.legend(loc='upper left', fontsize=8,
                           facecolor='#1a1a2e', labelcolor='white')
        self.ax_2d.set_title("클릭으로 장애물 추가 → [주행 시작]",
                              fontsize=11, color='white')

        self.robot_dot, = self.ax_2d.plot([], [], 'o', color='#e74c3c',
                                            markersize=10, zorder=11)
        self.short_line, = self.ax_2d.plot([], [], '-', color='#ff6b6b', lw=2, alpha=0.8)
        self.long_line,  = self.ax_2d.plot([], [], '-', color='#51cf66', lw=2, alpha=0.8)

    def _on_click(self, event):
        if self.running or event.inaxes != self.ax_2d: return
        x, y = event.xdata, event.ydata
        if x is None or y is None: return
        r = 8.0
        self.custom_obstacles.append({'pos': [x, y], 'r': r})
        circ = patches.Circle((x, y), r, color='#e74c3c', alpha=0.55)
        self.ax_2d.add_patch(circ)
        buf  = patches.Circle((x, y), r + 3.0, fill=False,
                               color='#e74c3c', ls='--', alpha=0.3)
        self.ax_2d.add_patch(buf)
        self.fig.canvas.draw_idle()

    def _start(self, event):
        if self.running: return
        self.running = True
        self.fig.canvas.mpl_disconnect(self.cid)
        self.ax_btn.set_visible(False)
        self.sim = PinkySlamEngine(25.0, 10.0, 1, self.custom_obstacles)
        self.ani = FuncAnimation(
            self.fig, self._animate, frames=1500, interval=50, repeat=False
        )
        self.fig.canvas.draw_idle()

    def _animate(self, frame_idx):
        if self.sim is None or self.sim.finished: return
        self.sim.update()

        px, py = self.sim.pos[0], self.sim.pos[1]

        # 2D 업데이트
        self.robot_dot.set_data([px], [py])
        self.short_line.set_data([px, self.sim.short_target[0]],
                                   [py, self.sim.short_target[1]])
        self.long_line.set_data([px, self.sim.long_target[0]],
                                  [py, self.sim.long_target[1]])

        # 3D 씬 갱신
        self.ax_3d.clear()
        self.ax_3d.set_facecolor('#0d1b2a')

        # 벽
        lx, ly = zip(*self.sim.left_wall)
        rx, ry = zip(*self.sim.right_wall)
        self.ax_3d.plot(lx, ly, [0]*len(lx), 'w-', lw=1.2, alpha=0.6)
        self.ax_3d.plot(rx, ry, [0]*len(rx), 'w-', lw=1.2, alpha=0.6)

        # 도로 바닥
        for j in range(len(self.sim.waypoints)-1):
            p1, p2 = self.sim.waypoints[j], self.sim.waypoints[j+1]
            self.ax_3d.plot([p1[0],p2[0]], [p1[1],p2[1]], [0,0],
                             color='#555', lw=35, alpha=0.12)

        # 탐지봉
        self.ax_3d.plot([px, self.sim.short_target[0]],
                         [py, self.sim.short_target[1]], [0.5, 0.5],
                         'r-', lw=2.5, label='Short Stick')
        self.ax_3d.plot([px, self.sim.long_target[0]],
                         [py, self.sim.long_target[1]], [0.5, 0.5],
                         color='#51cf66', lw=2.5, label='Long Stick')

        # 장애물 (구형)
        for obs in self.sim.obstacles:
            u, v = np.mgrid[0:2*np.pi:8j, 0:np.pi:8j]
            ox = obs['r']*np.cos(u)*np.sin(v) + obs['pos'][0]
            oy = obs['r']*np.sin(u)*np.sin(v) + obs['pos'][1]
            oz = obs['r']*np.cos(v) + obs['r']
            self.ax_3d.plot_wireframe(ox, oy, oz, color='#e74c3c',
                                       alpha=0.25, lw=0.8)

        # 로봇 + 목표
        self.ax_3d.scatter(*self.sim.pos, color='#3498db', s=120,
                            edgecolors='white', linewidth=1.5, zorder=10)
        self.ax_3d.scatter(*self.sim.goal, color='#f1c40f',
                            s=300, marker='*', zorder=10)

        # 시야 범위 설정
        self.ax_3d.set_xlim(px-70, px+70)
        self.ax_3d.set_ylim(py-70, py+70)
        self.ax_3d.set_zlim(0, 15)
        self.ax_3d.view_init(elev=50, azim=-80)
        self.ax_3d.set_facecolor('#0d1b2a')

        # 12-bit 데이터 HUD
        l1, l2 = self.sim.gua_l1, self.sim.gua_l2
        gx, gy = pos_to_grid(px, py)
        hud = (
            f"【 PINKY 12-bit DASHBOARD 】\n"
            f"Grid Addr : {grid_to_code(gx, gy)}\n"
            f"Pulse     : {self.sim.pulse_val:04d} / 4095\n"
            f"{'='*40}\n"
            f"{'LAYER':<7}|{'CODE':<5}|{'BIN':<7}|{'DEC':<4}|{'OCT':<4}|{'HEX'}\n"
            f"{'-'*40}\n"
            f"L1(High) |{l1['code']:<5}|{l1['bin']:<7}|{l1['dec']:<4}|{l1['oct']:<4}|{l1['hex']}\n"
            f"L2(Low)  |{l2['code']:<5}|{l2['bin']:<7}|{l2['dec']:<4}|{l2['oct']:<4}|{l2['hex']}\n"
            f"{'='*40}\n"
            f"Body Buffer : {self.sim.inflation_r:.1f} units\n"
            f"MODE : {self.sim.mode}"
        )
        self.ax_3d.text2D(
            0.02, 0.98, hud,
            transform=self.ax_3d.transAxes,
            fontsize=8, family='monospace', va='top', color='#ecf0f1',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#0d1b2a',
                      edgecolor='#3498db', alpha=0.9)
        )
        self.ax_3d.set_title(f"STATUS: {self.sim.mode}", fontsize=10,
                              color='white', pad=6)


# ============================================================
# 메인 실행
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("  Pinky 12-bit SLAM Dashboard")
    print("  실행 모드 선택:")
    print("    1 → 단일 로봇 상세 대시보드 (장애물 직접 배치)")
    print("    2 → 2대 군집 관제 센터")
    print("    3 → 3대 군집 관제 센터")
    print("=" * 60)

    try:
        choice = input("선택 (1/2/3) [기본: 2]: ").strip() or "2"
    except Exception:
        choice = "2"

    if choice == "1":
        print("→ 단일 로봇 대시보드 실행")
        dashboard = SingleRobotDashboard()
        plt.show()
    else:
        n = int(choice) if choice in ("2", "3") else 2
        print(f"→ {n}대 군집 관제 센터 실행")
        obs = [
            {'pos': [150, 50], 'r': 12.0},
            {'pos': [300, 80], 'r': 10.0},
        ]
        gui = CommandCenterGUI(num_robots=n, custom_obstacles=obs)
        plt.show()
