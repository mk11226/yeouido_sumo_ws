# 여의도 SUMO 도로 환경과 자율주행 학습

여의도·영등포구 북부의 실제 도로망과 시간대별 교통량으로 SUMO 환경을 구성하고,
SUMO 선생님 정책·BC·PPO로 차량의 주행 의사결정을 실험한다.
도로는 국가표준노드링크, 교통량은 서울시 TOPIS, 주변 건물·공원은 OpenStreetMap 자료를 사용한다.

## 현재 구성

- **교통량**: 심야 03시·주간 12시·출근 08시 실측 대수(배율 1)를 도로망 전체에 고르게 퍼뜨린 "uniform" 교통이 기본값이다. 기존 경계 출입 교통은 `YEOUIDO_DEMAND=measured`.
- **경로**: 랜드마크 7곳과 외곽 게이트 6곳 사이의 최단거리 경로 156개.
- **주행 화면**: 출발·도착과 시간대를 고르는 조절 창, 경로 차로 강조, 승용차 형태의 차량 표시.
- **신호**: 229개 제어기 중 30곳에 T DATA 관측 주기 적용. 적용 범위와 한계는 [도로 환경 문서](yeouido/README.md#t-data-실측-신호) 참고.
- **여의2교 남단**: 국회의사당 방향 좌회전 2차로와 맞은편 직진 종료 후 보호 좌회전 순서 반영.
- **평가**: 모든 정책에 같은 조건을 주는 고정 시험지(150문제, 최종 성적표 60문제 `random_60_s1`)와 결과 비교 도구.
- **학습 결과**: 최종 모델 DAgger(도착 78 %, 충돌 0)와 추론 주행 영상. 아래 [진행 결과](#진행-결과-2026-11-학습--최종-평가--추론-주행) 참고.

## 설치

저장소에는 최종 모델·교통·워밍업 상태·학습 데이터·결과가 모두 들어 있어서, 받은 뒤 아래 설치만 하면 바로 주행·평가할 수 있다.
작업 환경은 Python 3.10, SUMO 1.27.1(pip), PyTorch 2.x에서 확인했다.

### 1. 저장소 받기

```bash
git clone <저장소 주소> bmil_ws      # 처음 (약 1.5 GB). 이미 있으면 git pull
cd bmil_ws
```

내 변경이 있는 상태에서 `git pull` 하면 같은 파일(`env/road_config.py`, `env/yeouido/states/` 등)에서 충돌할 수 있다. 먼저 commit 하거나 `git stash` 한다.

### 2. 가상환경과 패키지

macOS · Linux:

```bash
python3.10 -m venv .venv             # conda 사용 시: conda create -n sumo_rl python=3.10 && conda activate sumo_rl
source .venv/bin/activate
python -m pip install -r yeouido/requirements-drive.txt
```

Windows (PowerShell):

```powershell
py -3.10 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -r yeouido/requirements-drive.txt
```

`requirements-drive.txt`에는 SUMO(`eclipse-sumo`, `traci`, `libsumo`, 모두 1.27.1), gymnasium, 영상용 Pillow·imageio-ffmpeg가 들어 있다.

### 3. PyTorch (컴퓨터에 맞는 것 하나)

```bash
python -m pip install torch                                                       # macOS (Apple GPU 자동 사용)
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu      # Windows·Linux CPU
python -m pip install torch --index-url https://download.pytorch.org/whl/cu121    # NVIDIA GPU (CUDA 12.1)
```

주행·평가는 CPU만으로 충분하다. 학습(PPO 등)은 GPU가 있으면 빠르다.

### 4. SUMO_HOME 설정 (새 터미널마다, 또는 셸 설정 파일에 추가)

```bash
export SUMO_HOME="$(python -c 'import sumo; print(sumo.SUMO_HOME)')"            # macOS · Linux
```

```powershell
$env:SUMO_HOME = python -c "import sumo; print(sumo.SUMO_HOME)"                  # Windows PowerShell
```

기존에 따로 설치한 SUMO를 쓰려면 `SUMO_HOME`을 그 설치 폴더로 둔다. 버전이 1.27.1과 다르면 결과가 조금 달라질 수 있다.

### 5. 설치 확인

```bash
# 최종 모델로 3문제 평가 (화면 없음, 3개 병렬로 수 분). 원래 결과와 같으면 정상: #1 도착, #0·#2 는 정체로 시간초과
LIBSUMO_AS_TRACI=quiet python test.py results/dagger_v1/iter1/model.pt \
    --eval-set eval_set/random_60_s1.json --nogui --limit 3 --workers 3

# sumo-gui 창에서 최종 모델 주행 (대표 영상과 같은 주행)
python -m yeouido_cross.drive_model --problem 30 --level day --reroute
```

Windows PowerShell 에서는 `LIBSUMO_AS_TRACI=quiet` 대신 먼저 `$env:LIBSUMO_AS_TRACI="quiet"`를 실행한다 (GUI 실행 전에는 `Remove-Item Env:LIBSUMO_AS_TRACI`).

### 참고

- **GUI:** 운영체제의 Tk와 그래픽 실행 환경이 필요하다. 화면 없는 서버에서는 `--nogui` 평가만 된다.
- **한글 글꼴:** 지도 핀 이름과 영상 상태 띠에 쓴다. macOS는 기본 글꼴(AppleGothic), Windows는 맑은 고딕, Linux는 `fonts-nanum` 또는 `fonts-noto-cjk`를 설치한다.
- **LIBSUMO_AS_TRACI=quiet:** 화면 없는 평가·학습을 약 2배 빠르게 한다 (결과는 같음). sumo-gui 실행에는 붙이지 않는다.
- **원본 주행 기록:** 100 MB 넘는 원본 주행 기록 3개는 조각으로 올라가 있다. 학습·주행에는 필요 없고, 쓸 때만 [큰 파일 합치기](#큰-파일-합치기)를 한다.
- **설치 문서:** OS별 SUMO 설치 상세는 [실습 문서](#실습-문서)에 있다.

## 빠른 실행

모든 명령은 **저장소 최상위 폴더**에서 실행한다.

```bash
# 시간대·출발·도착을 고르는 경로 주행 창
python -m yeouido_cross.drive_panel --level rush

# 국회의사당(1) → 마포대교 경계(B)를 바로 주행
python -m yeouido_cross.drive_panel --level night --start 1 B

# 조절 창 없이 경로 하나만 주행
python -m yeouido_cross.drive_route 5 3 --level day
```

| 입력 | 랜드마크 | 입력 | 외곽 게이트 |
|---|---|---|---|
| 1 | 국회의사당 | A | 서강대교 |
| 2 | IFC 서울 | B | 마포대교 |
| 3 | 63빌딩 | C | 원효대교 |
| 4 | 여의도공원 | D | 양화대교 |
| 5 | 영등포역 | E | 오목교 |
| 6 | 당산역 | F | 거리공원 |
| 7 | 노량진수산시장 | | |

기본 시간대는 주간(`day`)이며 심야는 `night`, 출근은 `rush`다.
경로 주행 도구는 `SumoExpert` 선생님 정책으로 운전한다.
처음 실행하거나 도로·교통량·신호가 바뀌면 워밍업 상태를 재생성하므로 수 분 걸릴 수 있다.
옵션·화면 조작·경로 파일 설명은 [경로 도구 README](yeouido_cross/README.md)에 정리했다.

배경 교통만 관찰하려면 다음을 사용한다.

```bash
python -m yeouido.live                  # 시간대 전환 창
python -m yeouido.view --level night    # 한 시간대 고정
```

## 고정 시험지 평가

156개 경로 목록과 평가 시험지 150문제는 별도다.
경로 주행 도구는 저장된 최단거리 경로를 사용하고, 학습·평가 환경은 출발·도착에서 SUMO가 찾은 최단시간 경로를 사용한다.

```bash
# 선생님 정책으로 6문제 점검
python test.py --expert --eval-set eval_set/eval_set.json --nogui --limit 6

# 선생님 정책 전체 평가
python test.py --expert --eval-set eval_set/eval_set.json --nogui --workers 4

# 학습한 모델 평가: 실제 모델 파일 경로로 변경
python test.py results/<run>/model.pt --eval-set eval_set/eval_set.json --nogui --workers 4
```

결과 파일·조건·비교 방법은 [평가 도구 README](eval_set/README.md)를 참고한다.
원본 브랜치에 기록된 과거 점수는 현재 교통량·도로망·신호 조건의 성능 수치가 아니다.

## 학습과 데이터 수집

도로 설정은 `env/road_config.py`, 관측·행동·보상은 `env/mdp_config.py`, 학습 설정은 각 실행 파일에서 조정한다.

```bash
python collect_pomdp_data.py --policy sumo-expert --episodes 500 --workers 4
python train_bc.py
python train.py
```

학습 차량이 156개 경로를 자동으로 순회하도록 설정된 것은 아니다. 자세한 절차는 아래 실습 문서를 따른다.

## 도로·교통량 수정과 검수

```bash
python -m inspection.serve                   # 도로망 검수 도구
python -m yeouido.build_demand               # 교통량 배율 변경 후 차량 일정표 재생성
python -m yeouido.run_all --skip-validate     # 도로·교통·신호 등 전체 환경 재생성
python -m yeouido_cross.build_routes         # 도로 변경 후 경로 재생성
python -m yeouido.validate                   # 70분 시뮬레이션 검증
```

생성 명령은 해당 결과 파일을 갱신한다. 변경 후 열린 SUMO 창은 다시 실행한다.
원본 도로 자료·신호 적용 범위·검증 한계는 [도로 환경 README](yeouido/README.md),
차로·회전 연결 수정 방법은 [검수 도구 README](inspection/README.md)에 있다.
T DATA API 키는 선택 사항이며, 사용할 때는 Git에서 제외되는 `.env` 또는 환경변수에 설정한다.

## 프로젝트 구성

| 경로 | 내용 |
|---|---|
| [yeouido/](yeouido/README.md) | 도로망·교통량·신호 생성과 검증 |
| [yeouido_cross/](yeouido_cross/README.md) | 156개 경로 생성과 GUI 주행 |
| [eval_set/](eval_set/README.md) | 고정 평가 조건 150개 |
| `env/yeouido/`, `env/yeouido_env.py` | SUMO 환경 파일·워밍업 상태·학습 환경 |
| [inspection/](inspection/README.md) | 도로망 검수와 수정 기록 |
| `algorithms/`, `utils/` | BC·PPO·선생님 정책, 버퍼·로깅·평가 |
| `data/`, `results/` | 학습 데이터·모델·실험 기록 |
| `tests/` | 신호 순서·관측 데이터 처리 등 회귀 검사 |

## 실습 문서

- [프로젝트 소개](project_materials/0_Project_Introduction.md)
- [Windows 설치](project_materials/1_SUMO_Windows_Setup.md)
- [macOS 설치](project_materials/1_SUMO_macOS_AppleSilicon_Setup.md)
- [공용 도로 환경·POMDP 구성](project_materials/2_Road_Environment_Construction.md)
- [BC 학습](project_materials/3_Model_Training_BC.md)
- [강화학습](project_materials/4_Model_Training_RL.md)

도로·주변 환경: © OpenStreetMap contributors, [ODbL 및 저작권 안내](https://www.openstreetmap.org/copyright).
국가표준노드링크·TOPIS·T DATA의 출처와 적용 범위는 [도로 환경 문서](yeouido/README.md)에 기록한다.

---

## 진행 결과 (2026-11): 학습 → 최종 평가 → 추론 주행

학습(BC → DAgger → PPO), 최종 평가, 추론 주행 영상까지 모두 끝났다.
**최종 모델은 DAgger 1회차** (`results/dagger_v1/iter1/model.pt`)이고 경로 재탐색은 끈다.

### 최종 결과 (새 교통 "uniform", 고정 시험지 60문제 `eval_set/random_60_s1.json`)

| 과제 기준 | 결과 |
|---|---|
| 충돌률 < 5 % | **0 %** (0/60) |
| 목적지 도착 | **47/60 (78 %)**: 심야 20/20 · 주간 12/17 · 출근 15/23 (선생님 정책과 같음) |
| 빨간불 통과 | 0회 |
| 주변 일반 차량(HV) | 주행 중 도로망 평균 4,007대 (심야 465 · 주간 4,547 · 출근 6,688), ego 100 m 안 평균 34대 |
| 순간이동(300초 정체) | 1/60 (실패로 계산) |

실패 13문제는 모두 시간초과 12개와 순간이동 1개다. 원인은 운전 실수가 아니라 주간·출근 교통 정체다.
전체 표는 `results/final/final_eval.md`(`.csv`)에 있다.

| 모델 (새 교통) | 도착 | 충돌 | 순간이동 | 쉴드 개입/문제 |
|---|---|---|---|---|
| **DAgger (최종)** | **47** | 0 | 1 | 712 |
| PPO | 46 | 0 | 2 | 83 |
| 선생님 (SumoExpert) | 47 | 0 | 1 | 0 |
| DAgger + 재탐색 | 44 | 0 | 3 | 658 |
| PPO + 재탐색 | 44 | 0 | 3 | 84 |
| 선생님 + 재탐색 | 47 | 0 | 3 | 0 |

- **PPO:** 도착은 1문제 적지만 안전 쉴드 개입이 DAgger의 약 1/9이다. 모델 스스로 더 안전하게 운전한다.
- **경로 재탐색:** 새 교통에서 도착을 늘리지 못해 평가에서는 끈다. 추론 영상에서만 켠다.

### 단계별 흐름

| 단계 | 하는 일 | 결과 (기존 교통 기준, 60문제) | 위치 |
|---|---|---|---|
| 0. 점검 | 안전 쉴드·저크·보상 순서 확인 | 보상: 선생님 > 서 있기 > 무작위 | `eval_set/step0_check.py`, `eval_set/reward_audit.py` |
| 1. 선생님 | SUMO 운전자 모델(SumoExpert) 평가 | 도착 45, 충돌 0 | `results/sumo_expert/` |
| 2. BC | 선생님 데이터(DART 노이즈 200판)로 PPO와 같은 신경망 학습 | 도착 42, 충돌 0 | `train_bc_hybrid.py`, `results/bc_hybrid_v2/` |
| 3. DAgger | 모델이 운전, 선생님이 정답 라벨 (3회 반복, 1회차 최고) | 도착 44, 충돌 0 | `dagger.py`, `results/dagger_v1/` |
| 4. PPO | DAgger 모델에서 미세조정 (critic 워밍업, 8환경 병렬, 약 100만 스텝) | 도착 45, 충돌 0 | `train.py`, `results/ppo_v1/best/` |
| 5. 최종 평가 | 새 교통에서 6개 조합 재평가, 최종 모델 확정 | 위 표 | `eval_set/final_report.py`, `results/uniform/`, `results/final/` |
| 6. 추론 주행 | sumo-gui에서 최종 모델 주행, 영상·주행 기록 | 시나리오 6개 × 3시간대 중 17개 도착 | `yeouido_cross/drive_model.py`, `results/video/` |

### 이번에 바뀐 환경

- **교통 "uniform" (기본값):**
  - 시간대별 차량 대수는 실측 그대로(배율 1, 70분 동안 심야 4,780 · 주간 28,259 · 출근 36,752대)다.
  - 출발·도착은 도로망 전체에서 차로-km에 비례해 고르게 뽑는다. 기존 교통은 경계 간선도로에만 몰려 도로의 57 %가 비어 있었다.
  - 워밍업은 600초 시점 하나만 쓴다. 시험지의 다른 시점은 600초로 바꿔 쓴다.
  - 기존 교통을 쓰려면 `YEOUIDO_DEMAND=measured`를 준다.
- **일반 차량 4종:** Krauss 35 % · IDM 30 % · EIDM 20 % · ACC 15 %. 화면 색은 노랑·하늘·주황·자주다.
- **차로 변경 모드 257:** ego와 일반 차량이 같은 순간 같은 빈자리로 끼어들던 충돌(1건)을 막는다. 변경하는 순간 바로 충돌이면 SUMO가 차로 변경을 거부한다.
- **경로 재탐색 (선택, `YEOUIDO_REROUTE=1`):** 아래 조건을 모두 만족할 때만 같은 목적지로 가는 길을 바꾼다.
  - 시속 7 km(2 m/s) 미만으로 60초 넘게 막혔을 때
  - 지금 교통 기준으로 30 % 이상, 60초 이상 빨라질 때
  - 우회 거리가 남은 거리의 1.5배 + 500 m 이하일 때
  - 한 판에 3회까지
- **기록 항목 추가:** `info`에 `hv_total`(도로망 HV 수), `hv_near`(100 m 안 HV 수), `reroutes`가 있다.

### 사용 방법

모든 명령은 저장소 최상위에서, 화면 없는 실행은 `LIBSUMO_AS_TRACI=quiet`를 붙이면 약 2배 빠르다 (GUI 실행에는 붙이지 않는다).

```bash
# 최종 모델 평가 (60문제, 8개 병렬)
LIBSUMO_AS_TRACI=quiet python test.py results/dagger_v1/iter1/model.pt \
    --eval-set eval_set/random_60_s1.json --workers 8 --nogui --out results/uniform/dagger.json
YEOUIDO_REROUTE=1 ...        # 같은 명령에 붙이면 경로 재탐색 켬
python test.py --expert ...  # 선생님 정책

# 최종 평가표 (md + csv)
python -m eval_set.final_report --final "DAgger=results/uniform/dagger.json" \
    --compare "PPO=results/uniform/ppo.json" "선생님=results/uniform/expert.json"

# 추론 주행 (sumo-gui). 파란색 = 처음 경로, 주황색 = 재탐색으로 바뀐 경로
python -m yeouido_cross.drive_model --problem 30 --level day --reroute          # 시험지 문제를 다른 시간대로
python -m yeouido_cross.drive_model --origin 4 --destination 6 --level rush --reroute
python -m yeouido_cross.drive_model --problem 21 --check results/uniform/dagger.json   # 추론 = 평가 결과인지 확인
python -m yeouido_cross.drive_model --problem 30 --level day --reroute \
    --record results/video/demo.mp4                                               # 영상(10 fps = 5배속) + 주행 기록 CSV

# 다시 학습할 때
LIBSUMO_AS_TRACI=quiet python collect_pomdp_data.py --policy sumo-expert --episodes 200 --workers 8 --dart-sigma 0.15
python train_bc_hybrid.py --data data/expert_dart200_v2.npz
LIBSUMO_AS_TRACI=quiet python dagger.py --init results/bc_hybrid_v2/model.pt \
    --base-data data/expert_dart200_v2.npz --iters 3 --episodes 100 --betas 0.5,0.25,0
LIBSUMO_AS_TRACI=quiet python train.py --init results/dagger_v1/iter1/model.pt --workers 8   # PPO (--resume 로 이어서)

# 교통 다시 만들기 (배율 바꿀 때 --scale)
python -m yeouido.build_demand_uniform
```

영상 상태 띠에는 시간, 속도, 남은 거리, 주변·도로망 차량 수, 목적지, 재탐색 횟수가 나온다.
게이트는 실제 이름으로 표시한다 (B → 마포대교).
목적지는 출발할 때 정하고 바꾸지 않는다.

### 파일 위치

| 경로 | 내용 |
|---|---|
| `results/dagger_v1/iter1/model.pt` | **최종 모델** (DAgger 1회차) |
| `results/ppo_v1/best/model.pt` | PPO 최고 모델 (검증 시험지 기준 step 602,112) |
| `results/bc_hybrid_v2/model.pt` | BC 모델 (DAgger 출발점) |
| `data/expert_dart200_v2.npz`, `data/dagger_v1_iter1.npz` | 최종 모델 학습 데이터 (관측 42차원 + 선생님 행동). 다른 `data/*.npz`는 DAgger 2·3회차 등 실험 데이터 |
| `data/*.jsonl.gz` | NPZ 를 만든 원본 주행 기록 (스텝마다 전체 상태). 100 MB 넘는 3개는 `.part_*` 조각 → 아래 "큰 파일 합치기" |
| `results/final/final_eval.md`, `.csv` | 최종 평가표 (`measured_demand/` = 기존 교통) |
| `results/uniform/*.json` | 새 교통 6개 조합 평가 결과 (문제별 기록) |
| `results/video/final_demo.mp4` (+ `.csv`) | 대표 영상: 주간 여의도공원 → 당산역, 재탐색 2회 후 도착 (189초) |
| `results/video/reroute/` | 시나리오 6개 × 심야·주간·출근 추론 영상(mp4)과 주행 기록(CSV) |
| `results/demand_check/` | 교통 배율별 정체·순간이동 측정 |
| `eval_set/random_60_s1.json`, `random_30_s2.json` | 최종 시험지 60문제, PPO 검증용 30문제 |
| `env/yeouido/<시간대>_uniform.rou.xml`, `states/<시간대>_uniform_600.xml.gz` | 새 교통 일정표와 워밍업 상태 |
| `algorithms/ppo.py`, `utils/vec_env.py` | PPO (병렬 환경, critic 워밍업, 체크포인트 이어 학습) |
| `yeouido/build_demand_uniform.py` | 새 교통 생성 |
| `eval_set/reroute_*.py`, `demand_check.py`, `final_report.py` | 재탐색 효과 확인, 교통 측정, 최종 평가표 |

### 받은 뒤 바로 해 보기

저장소에는 최종 모델, 새 교통, 워밍업 상태, 시험지, 모든 학습 데이터·평가 결과·영상이 들어 있다.
`git pull`(또는 clone) 후 [설치](#설치)만 하면 아래가 바로 된다.
(이 저장소만 복사한 빈 폴더에서 최종 모델 12문제를 다시 평가해 원래 결과와 12/12 같음을 확인했다.)

```bash
python -m yeouido_cross.drive_model --problem 30 --level day --reroute      # 대표 영상과 같은 주행 (sumo-gui)
LIBSUMO_AS_TRACI=quiet python test.py results/dagger_v1/iter1/model.pt \
    --eval-set eval_set/random_60_s1.json --workers 8 --nogui --limit 12      # results/uniform/dagger.json 과 같은 결과
```

### 큰 파일 합치기

GitHub는 파일 하나가 100 MB를 넘으면 받지 않아서, 원본 주행 기록 3개는 90 MB 조각으로 나눠 올렸다.
학습·주행에는 필요 없고(NPZ 로 충분), 원본 기록이 필요할 때만 합친다.

```bash
for f in dagger_v1_iter3 expert_dart200_v2 expert_dart207; do
    cat data/$f.jsonl.gz.part_* > data/$f.jsonl.gz
done
```

### 알려진 한계

- **배율 1 교통은 이 SUMO 도로망에 과포화다.**
  - 출근 시간대에는 30분 동안 도로망 차량이 5,500대에서 10,000대로 계속 쌓이고, 교차로 꼬리물기가 늘어난다(교차로 안 정지 94대 → 381대).
  - 운전자·신호 설정을 바꿔도 13 % 이하밖에 줄지 않았다. 배율 0.4~0.5에서는 크게 줄었다(17~45대).
  - 실측 대수를 지키기 위해 배율 1을 유지했다.
  - 실패 문제의 대부분이 이 정체 때문이다. 영등포역·노량진 방향 장거리 주행은 주간·출근에 시간초과가 난다.
- **재탐색은 정체가 심하면 도움이 적다.** 막힌 차가 많아 다른 길도 막히거나 우회가 길어진다.
- **신호 시간:** 229개 중 T DATA 실측 주기는 주간 27개·출근 12개·심야 0개(전체 30곳)이고, 나머지는 SUMO 자동 신호다.
