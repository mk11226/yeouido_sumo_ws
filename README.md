# Artificial Intelligence and Control for Autonomous Driving

> **자율주행 인공지능 및 제어 — Course Project Materials**

수업 프로젝트에 필요한 자료를 순서대로 정리합니다.

---

## 📚 Project Materials

### 개인별 진행

### 0. Project Introduction

- [프로젝트 소개 및 전체 진행 방향](./project_materials/0_Project_Introduction.md)

### 1. SUMO Installation

- [Windows — SUMO 설치 및 환경 설정](./project_materials/1_SUMO_Windows_Setup.md)
- [macOS (Apple Silicon) — SUMO 설치 및 환경 설정](./project_materials/1_SUMO_macOS_AppleSilicon_Setup.md)

### 2. Road Environment Construction

- [도로 및 모델 POMDP 구조 이해](./project_materials/2_Road_Environment_Construction.md)

### 3. Model Training — Behavior Cloning

- [모방학습 기반 자율주행 의사결정 모델 학습](./project_materials/3_Model_Training_BC.md)

---

### 팀별 진행 (1-2명으로 구성, 자율 building)

### 4. Model Training — Reinforcement Learning

- [강화학습 기반 자율주행 의사결정 모델 학습](./project_materials/4_Model_Training_RL.md)

### 혜화로터리 실제 지도 기반 실험

- [환경 구축·PPO 학습·평가 결과 및 재실행 방법](./project_materials/5_Hyehwa_RL_구축_학습_결과.md)
- [혜화로터리 도로와 8개 주행 경로](./env/hyehwa/network.png)
- 도로 생성: `build_hyehwa.py`, 교차로 환경: `env/hyehwa_env.py`, 학습·평가: `train_hyehwa.py`
- OSM 도로 형상을 사용하며 신호·교통량은 실험용 설정입니다. 충돌 방지는 SUMO 안전 제어와 함께 동작합니다.

### 성균관대학교 서울 → 수원 캠퍼스 주행 환경

- [약 38.35km 도로 구축·검증 결과 및 실행 방법](./project_materials/6_SKKU_서울_수원_도로환경.md)
- [현재 환경: 실제 건물·공식 신호 위치 보완 및 검증](./project_materials/7_SKKU_건물_신호등_보완.md)
- [도심·자동차전용도로·수원 도착 경로 지도](./env/skku_commute/network.png)
- GUI 실행: `.venv-hyehwa/bin/python run_skku_commute.py --gui --background --viewport-m 180 --keep-open`
- 실제 OSM 도로·건물 윤곽과 공식 시설 자료의 신호 위치를 반영했습니다. 신호 시간표·교통량은 실험용이며 현장 전체 시설의 완전한 복제는 아닙니다. 이 경로의 PPO 학습은 아직 수행하지 않았습니다.

---
### 팀별 진행 (3-5명으로 구성, 자율 building)

### 5. Project Preparation

- [프로젝트 진행을 위한 코드 소개](./project_materials/5_Project_Preparation.md)

### 이후. 팀별 프로젝트 진행

---
> 수업 자료는 `project_materials/` 폴더에서도 확인 가능합니다.

## POMDP 차선변경 + 데이터 수집 (업데이트)

현재 환경 행동은 2차원입니다.

- `action[0]`: 가감속 raw `[-1, 1]`
- `action[1]`: 차선변경 raw `[-1, 1]`
  - `+1`: 왼쪽 1개 차선
  - `0`: 현재 차선 유지
  - `-1`: 오른쪽 1개 차선
  - PPO의 연속 Gaussian 출력을 유지하기 위해 환경 내부에서 threshold를 기준으로 `{-1,0,+1}`로 양자화합니다.

기존 observation은 ego 주변만 보는 19차원 부분관측이므로 POMDP의 `o_t`로 취급합니다.
`collect_pomdp_data.py`는 시간 순서를 유지한 trajectory를 저장하며, agent가 보지 않는 SUMO 내부 상태는
`state` / `next_state`라는 privileged field로 별도 저장합니다.

```bash
# 랜덤 behavior policy로 100 episode 수집 (headless)
python collect_pomdp_data.py --episodes 100

# GUI를 켜고 10 episode 확인
python collect_pomdp_data.py --episodes 10 --gui

# 차선 유지 위주로 파이프라인 확인
python collect_pomdp_data.py --policy keep-lane --episodes 10
```

출력은 `data/*.jsonl`(전체 POMDP transition + privileged state)과
`data/*.npz`(학습에 바로 쓰기 쉬운 고정 크기 배열) 두 형식입니다.

GUI에서는 ego 차량을 자동 추적하고 확대해서 보여주며, 차량 타입은 `passenger/sedan`으로 생성됩니다.
차선변경 action dimension이 1개 추가되었으므로 **기존 1차원 action으로 학습된 model.pt는 새 환경과 shape이 맞지 않습니다.**
새 설정에서는 `python train.py`로 다시 학습해야 합니다.

## 일반 차량

일반 차량은 단일 모델이 아니라 서로 다른 차량 추종 모델 4종을 확률적으로 섞어
투입한다 (SUMO vTypeDistribution). 단일 모델만 쓰면 에이전트가 그 모델의
버릇에 과적합되기 때문. GUI에서 색으로 구분된다:

| 컨트롤러 | 비율 | 색 | 성격 |
|---|---|---|---|
| Krauss | 35% | 노랑 | SUMO 기본. sigma로 무작위 감속이 섞인 산만한 인간 운전자 |
| IDM | 30% | 시안 | 부드럽고 예측 가능한 가감속. 교통류 연구 표준 인간 모델 |
| EIDM | 20% | 주황 | IDM + 반응지연·부주의. 더 현실적인 인간 거동 |
| ACC | 15% | 마젠타 | 기계처럼 일정 차간시간 유지. 자율주행/크루즈 차량 느낌 |

비율·파라미터는 env/road_config.py의 TRAFFIC["controllers"] 에서 수정.
항목을 추가/삭제하면 그대로 반영된다 (비율 합은 자동 정규화).
