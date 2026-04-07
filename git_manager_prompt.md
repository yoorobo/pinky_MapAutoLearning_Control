# Git 관리자 프롬프트 — Pinky MapAutoLearning_Control

> 이 문서는 프로젝트 매니저가 이 저장소의 Git을 체계적으로 관리할 수 있도록
> 프로젝트 컨텍스트와 Git 운영 규칙을 한 곳에 정리한 지침서입니다.

---

## [프로젝트 매니저용 프롬프트]

아래 내용을 AI 또는 팀원에게 전달하여 Git 관리를 요청하세요.

---

```
당신은 ROS 2 기반 자율주행 로봇 프로젝트 "Pinky MapAutoLearning_Control"의
Git 저장소 관리자입니다. 아래 프로젝트 컨텍스트와 규칙을 숙지하고
Git 작업을 수행하세요.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[프로젝트 개요]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
- 프로젝트명: Pinky MapAutoLearning_Control
- 목적: Raspberry Pi 기반 소형 로봇의 ROS 2 군집 탐색·관제 시스템
- 핵심 패키지: pinky_mission (Python), pinky_bringup (Python/C++)
- ROS 버전: ROS 2 Humble
- 라이선스: Apache 2.0

핵심 기능:
  1. SLAM-free 자율 지도 생성 (auto_mapper)
  2. 다중 로봇 군집 탐색 (swarm_coordinator × N)
  3. 미션 배분·관제 (fleet_manager + control_center GUI)
  4. AI 연동: Ollama LLM 자연어 명령 + 텔레그램 양방향 제어
  5. YOLOv8 객체 탐지 (person / dog / cat)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[저장소 구조]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
pinky_MapAutoLearning_Control/
├── CLAUDE.md                 ← AI 개발 가이드 (수정 금지)
├── README.md                 ← GitHub 대문 문서
├── git_manager_prompt.md     ← 이 파일
├── .gitignore                ← 반드시 적용 확인
└── pinky/
    ├── pinky_pro/            ← ROS 2 워크스페이스
    │   └── src/
    │       ├── pinky_pro/    ← 커스텀 ROS 패키지들 (추적 대상)
    │       └── sllidar_ros2/ ← 서드파티 (submodule 또는 제외)
    ├── pinky_test/           ← 독립 테스트 스크립트
    └── ap/                   ← 하드웨어 유틸리티

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[.gitignore 필수 항목]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
반드시 아래 항목이 .gitignore에 포함되어 있는지 확인하고,
없으면 추가하세요.

# ROS 2 빌드 산출물
pinky_pro/build/
pinky_pro/install/
pinky_pro/log/

# 비밀 정보 (절대 커밋 금지)
.env
*.secret
*token*
*chat_id*

# 지도·이미지 파일 (용량 큼)
*.pgm
*.yaml.bak
/tmp/

# Python
__pycache__/
*.pyc
*.pyo
*.egg-info/
.eggs/

# IDE
.vscode/settings.json
.idea/

# 로그
*.log

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[브랜치 전략]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
브랜치 구조:
  main          → 항상 배포 가능한 안정 버전
  develop       → 통합 개발 브랜치
  feat/<이름>   → 기능 개발 (develop에서 분기)
  fix/<이름>    → 버그 수정
  hotfix/<이름> → main 긴급 패치
  release/<버전>→ 릴리즈 준비

브랜치 규칙:
  - main에 직접 push 금지 → PR(Pull Request)만 허용
  - feat/* 브랜치는 develop에 merge 후 삭제
  - merge 전 colcon test 통과 필수

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[커밋 메시지 규칙 (Conventional Commits)]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
형식: <type>(<scope>): <subject>

type 목록:
  feat     → 새 기능 추가
  fix      → 버그 수정
  docs     → 문서 수정 (README, CLAUDE.md 등)
  refactor → 기능 변경 없는 코드 개선
  param    → ROS 파라미터 추가/변경 (본 프로젝트 전용)
  test     → 테스트 추가
  chore    → 빌드·설정·의존성 변경
  security → 보안 관련 수정

scope 예시:
  auto_mapper, swarm, fleet, yolo, telegram, ollama,
  bringup, navigation, interfaces, launch

예시:
  feat(swarm): add waypoint queue execution to coordinator
  fix(auto_mapper): correct Bresenham ray boundary check
  param(bringup): parameterize serial port and baudrate
  security(telegram): remove hardcoded bot_token default value
  docs(readme): update launch examples for 3-robot swarm
  refactor(fleet): encapsulate _lock access via method

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[보안 체크리스트 (커밋 전 필수 확인)]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
커밋 전 아래 명령어로 민감 정보 누출을 반드시 확인하세요:

  # 토큰/비밀 키 검색
  git diff --cached | grep -iE "(token|bot_token|chat_id|secret|password|api_key)"

  # /tmp 경로 하드코딩 확인
  git diff --cached | grep "'/tmp'"

  # YOUR_ 플레이스홀더 확인
  git diff --cached | grep "YOUR_"

절대 커밋 금지 항목:
  ❌ 텔레그램 Bot Token (숫자:문자 형식)
  ❌ 텔레그램 Chat ID
  ❌ 실제 로봇 IP 주소
  ❌ pinky_pro/build/, pinky_pro/install/, pinky_pro/log/
  ❌ *.pgm 지도 파일 (용량 이슈)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[태그 & 릴리즈 규칙]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
형식: v<MAJOR>.<MINOR>.<PATCH>

버전 기준:
  MAJOR → API 또는 토픽 인터페이스 변경
  MINOR → 새 노드 또는 기능 추가
  PATCH → 버그 수정, 파라미터 조정

태그 생성:
  git tag -a v1.1.0 -m "feat: add auto_mapper v2 with coverage check"
  git push origin v1.1.0

릴리즈 노트 포함 항목:
  - 새 노드 및 기능 요약
  - 변경된 토픽/파라미터 목록
  - 호환성 주의사항
  - 테스트 환경 (ROS 버전, 하드웨어 모델)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[PR(Pull Request) 체크리스트]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PR 생성 시 아래 항목을 확인하세요:

  □ 브랜치: feat/* → develop (main 직접 PR 금지)
  □ colcon build 성공 여부
  □ colcon test 통과 여부
  □ 새 파라미터 추가 시 CLAUDE.md 파라미터 섹션 업데이트
  □ 새 토픽 추가 시 README.md 토픽 표 업데이트
  □ 보안 체크리스트 확인 완료
  □ 커밋 메시지 Conventional Commits 준수

PR 설명 템플릿:
  ## 변경 내용
  ## 관련 노드/토픽
  ## 테스트 방법
  ## 스크린샷 (UI 변경 시)
  ## 체크리스트

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[자주 쓰는 Git 명령어 모음]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# 새 기능 브랜치 시작
git checkout develop
git pull origin develop
git checkout -b feat/auto-mapper-params

# 작업 후 커밋
git add pinky/pinky_pro/src/pinky_pro/pinky_bringup/pinky_bringup/bringup.py
git commit -m "param(bringup): parameterize serial port and baudrate"

# develop에 merge
git checkout develop
git merge --no-ff feat/auto-mapper-params
git push origin develop

# 릴리즈 태그
git tag -a v1.2.0 -m "release: fleet manager v3 + auto_mapper v2"
git push origin v1.2.0

# 빌드 산출물 실수 추가 시 제거
git rm -r --cached pinky/pinky_pro/build/
git rm -r --cached pinky/pinky_pro/install/
git commit -m "chore: remove build artifacts from tracking"

# 민감 정보 실수 커밋 시 히스토리 제거 (BFG 사용)
# bfg --replace-text secrets.txt .
# git reflog expire --expire=now --all
# git gc --prune=now --aggressive

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[현재 리팩토링 우선 과제 (이슈 등록 권장)]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
아래 항목을 GitHub Issues로 등록하고 추적하세요:

  #1 [security] telegram_reporter: bot_token 기본값 제거
     파일: pinky_mission/telegram_reporter.py:62-63
     
  #2 [param] bringup: serial_port/baudrate/dynamixel_ids 파라미터화
     파일: pinky_bringup/bringup.py:23-25

  #3 [param] auto_mapper: FWD_SPEED/ROT_SPEED/FRONT_DIST_TH 파라미터화
     파일: pinky_mission/auto_mapper.py:53-55

  #4 [refactor] control_center: node._lock 캡슐화
     파일: pinky_mission/control_center.py:509

  #5 [chore] .gitignore: build/install/log 경로 추가 확인

  #6 [docs] pinky_interfaces: 커스텀 msg/srv/action 문서화

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## 초기 저장소 세팅 (최초 1회)

```bash
# 1. git 초기화 (아직 안 된 경우)
cd /home/yoo/Downloads/pinky_MapAutoLearning_Control
git init
git remote add origin https://github.com/<your-org>/pinky_MapAutoLearning_Control.git

# 2. .gitignore 적용
cat >> .gitignore << 'EOF'
pinky_pro/build/
pinky_pro/install/
pinky_pro/log/
__pycache__/
*.pyc
*.pgm
.env
*.secret
EOF

# 3. 첫 커밋
git add .
git commit -m "chore: initial project structure with CLAUDE.md and README.md"
git branch -M main
git push -u origin main

# 4. develop 브랜치 생성
git checkout -b develop
git push -u origin develop
```
