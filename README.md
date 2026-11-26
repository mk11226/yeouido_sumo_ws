# 여의도 SUMO 자율주행: 실제 도로 환경 구축 → 학습 → 추론(주행)

여의도·영등포구 북부의 실제 도로망과 시간대별 실측 교통량으로 SUMO 환경을 만들고,
자율주행 차(ego)가 출발지에서 목적지까지 충돌 없이 가도록 학습했다.
규칙 기반 운전자(선생님)를 따라 배운 뒤(BC · DAgger), 보상으로 스스로 다듬었다(PPO).

- **도로:** 국토교통부 국가표준노드링크
- **교통량:** 서울시 TOPIS 평일 실측 (심야 03시 · 주간 12시 · 출근 08시)
- **신호:** 서울시 T DATA 실측 신호 주기 (일부 교차로)
- **주변 환경:** OpenStreetMap 건물 · 공원 · 한강

## 1. 결과 요약

최종 모델은 **PPO** (`results/ppo_v1/best/model.pt`)다. 고정 시험지 60문제(`inference/eval_set/random_60_s1.json`)로 평가했다.

| 과제 기준 | 결과 |
|---|---|
| 충돌률 < 5 % | **0 %** (0/60) |
| 목적지 도착 | **45/60 (75 %): 심야 20/20 · 주간 13/17 · 출근 12/23** |
| 빨간불 통과 | 0회 |
| 주변 일반 차량(HV) | 주행 중 도로망 평균 4,262대, ego 100 m 안 평균 32대 |
| 순간이동(300초 정체) | 0/60 |
| 30초 이상 주행 영상 | `results/video/final_demo.mp4`: 출근, 서강대교 → 영등포역, 216초에 막혀 재탐색(주황) 후 그 길로 도착 (260초 영상) |

| 모델 | 도착 | 충돌 | 순간이동 | 쉴드 개입/문제 |
|---|---|---|---|---|
| **PPO (최종)** | **45** (심야 20 · 주간 13 · 출근 12) | 0 | 0 | 43 |
| DAgger | 43 (심야 20 · 주간 12 · 출근 11) | 0 | 2 | 655 |
| 선생님 (SumoExpert) | 45 (심야 20 · 주간 13 · 출근 12) | 0 | 0 | 0 |

- 실패는 대부분 주간 · 출근 교통 정체로 인한 시간초과다. 선생님 정책도 비슷한 수준에서 막힌다.
- PPO는 선생님과 같은 도착률을 내면서 안전 쉴드 개입을 DAgger의 약 1/15로 줄였다 (스스로 안전 속도를 지킴).
- 전체 표와 도로망 변경 분석: `results/final/final_eval.md`. 이전 도로망 결과: `results/final/old_net/`.

## 2. 전체 흐름

```
[1] 도로 환경 구축          [2] 학습 환경            [3] 학습                     [4] 평가          [5] 추론(주행)
노드링크 → 도로망           YeouidoEnv              선생님 데이터(DART)          고정 시험지 60문제   sumo-gui 주행
TOPIS → 교통량       →      관측 42 · 행동 2   →    → BC → DAgger → PPO    →    충돌·도착·정체  →   영상·주행 기록
T DATA → 신호               안전 쉴드 · 보상
OSM → 배경
```

### 2-1. 도로 환경 구축 (`yeouido/`)

1. **다운로드:** TOPIS 교통량 엑셀, OSM 신호 위치 · 건물 · 공원 · 한강 (`download.py`)
2. **도로 추출:** 전국 표준노드링크에서 범위(경도 126.880~126.950, 위도 37.505~한강 중앙선) 안 도로만 잘라냄 (`nodelink.py`)
3. **도로망:** 교차로 합치기 · 골목길 제거 · 회전 제한 · 신호 → netconvert, 검수로 확인한 차로 · 연결 수정 반영 (`build_net.py`, `corrections.json`)
4. **교통량:** 측정 지점별 시간대 평균(`prepare_counts.py`) → 도로에 연결 · 출입 게이트 지정(`map_counts.py`) → 일반 차량 수요(`build_demand.py`)
5. **새 교통 "uniform" (현재 기본값):** 시간대별 대수는 실측 그대로(배율 1, 70분 동안 심야 4,780 · 주간 28,259 · 출근 36,752대), 출발 · 도착은 도로망 전체에 차로-km 비례로 고르게 (`build_demand_uniform.py`)
6. **신호:** T DATA 실측 주기를 일부 교차로에 적용 (`tdata_signals.py`), 나머지는 SUMO 자동 신호
7. **배경 · 랜드마크:** 건물 · 공원 · 한강 폴리곤(`build_context.py`), 랜드마크 7곳을 차도에 연결(`landmarks.py`)
8. **검증:** 70분 시뮬레이션으로 실측과 비교 · 순간이동 · 충돌 검사 (`validate.py`)

결과물은 `env/yeouido/`에 있다 (도로망 `yeouido.net.xml`, 교통 `<시간대>_uniform.rou.xml`, 신호 `signals_<시간대>.add.xml`, 워밍업 상태 `states/`).

### 2-2. 학습 환경 (`env/yeouido_env.py`)

- **한 판:** 시간대를 고르고 → 배경 교통이 이미 흐르는 워밍업 상태(600초 시점)를 불러온 뒤 → 출발지에서 목적지까지 SUMO 최단시간 경로로 주행. 목적지 도착이면 성공, 충돌 · 300초 정체(순간이동) · 30분 초과면 실패.
- **관측 42차원:** 기본 31(내 속도 · 차로, 주변 차) + 여의도 11(앞 신호 색 · 정지선 거리, 다음 교차로 · 경로 차로, 남은 경로, 제한속도, 교차로 상대 차, 먼 앞차). 정의는 `env/mdp_config.py`.
- **행동:** 가감속 연속값 1개 + 차로 유지/왼쪽/오른쪽.
- **안전 쉴드:** 정책이 낸 속도를 선생님 계산(앞차 · 신호 · 교차로 양보)의 안전 속도 이하로 자른다 → 충돌을 구조적으로 막는다.
- **보상:** 목적지 쪽 전진(+, 경로 전체 +10), 스텝 감점, 불필요한 정지 감점, 빨간불 −2, 충돌 −20, 도착 +10, 쉴드 개입 · 저크 감점. 값은 `env/mdp_config.py`, `training/train.py`.
- **경로 재탐색 (선택, `YEOUIDO_REROUTE=1` 또는 `--reroute`):** 60초 넘게 막히면 지금 교통 기준으로 같은 목적지까지 30 % · 60초 이상 빠른 길이 있을 때만 경로를 바꾼다. 도로 통과 시간에 더해, 꺾어야 하는 차로에 서 있는 차 수 × 2.5초(회전 대기 줄)를 두 경로 모두에 더해 비교한다. 우회는 남은 거리 × 1.5 + 500 m 이하, 한 판 3회까지. 바꾼 도로를 벗어나기 전에는 다시 바꾸지 않고, 버린 길로 되돌아가지 않는다. 주간 · 출근 32문제를 재탐색 켜고 다시 주행하면 시간초과 5문제가 도착으로 바뀌고 2문제가 시간초과가 됐다 (+3). 최종 평가는 재탐색을 끈 결과이고, 추론 영상에서 켠다.
- **일반 차량 4종:** Krauss 35 % · IDM 30 % · EIDM 20 % · ACC 15 % (화면 색 노랑 · 하늘 · 주황 · 자주).

### 2-3. 학습

| 단계 | 하는 일 | 코드 | 결과물 |
|---|---|---|---|
| 0. 점검 | 안전 쉴드 · 보상 순서 확인 (선생님 > 서 있기 > 무작위) | `inference/eval_set/step0_check.py`, `inference/eval_set/reward_audit.py` | `results/step0_check.json`, `results/reward_audit/` |
| 1. 선생님 데이터 | SUMO 운전자 모델(SumoExpert)이 운전, 실제 운전에만 작은 잡음(DART) | `training/collect_pomdp_data.py` | `training/data/expert_dart200_v2.npz` |
| 2. BC | PPO와 같은 신경망으로 선생님 행동 따라 하기 | `training/train_bc_hybrid.py` | `results/bc_hybrid_v2/` |
| 3. DAgger | 모델이 직접 운전하며 만난 상황에 선생님 정답을 붙여 보충 학습 (3회, 1회차 최고) | `training/dagger.py` | `results/dagger_v1/`, `training/data/dagger_v1_iter*.npz` |
| 4. PPO | DAgger 모델에서 강화학습 미세조정 (critic 워밍업, 8환경 병렬, 약 100만 스텝) | `training/train.py`, `training/algorithms/ppo.py` | `results/ppo_v1/best/model.pt` (**최종**) |

### 2-4. 평가

- 고정 시험지: 문제마다 시간대 · 출발 · 도착 · SUMO 시드가 고정이라 어떤 모델이든 같은 조건, 다시 돌려도 같은 결과.
  최종 성적표 `inference/eval_set/random_60_s1.json`(60문제), PPO 모델 고르기용 `inference/eval_set/random_30_s2.json`(30문제).
- 지표: 도착 · 충돌 · 순간이동 · 시간초과 · 빨간불 · 쉴드 개입 · 주변 차량 수.
- 결과: `results/uniform_newnet/` (현재 도로망), 평가표 `results/final/final_eval.md`.

### 2-5. 추론(주행)

학습한 모델이 잡음 없이 운전한다 (가감속은 평균값, 차로는 가장 확률 높은 선택, 안전 쉴드는 학습 때와 같음).
`inference/drive_model.py`로 sumo-gui에서 주행을 보고 영상(mp4)과 주행 기록(CSV)을 남긴다.
화면에는 처음 경로가 파란색, 재탐색으로 바뀐 경로가 주황색으로 표시된다. 실행 방법은 [8. 주행 실행 방법](#8-주행-실행-방법).

## 3. 폴더 구성

| 경로 | 내용 |
|---|---|
| `main.py` | **최종 모델 주행 화면 바로 실행** (선택 창 또는 옵션으로 시간대 · 출발 · 목적지 · 배속, [9절](#9-한-번에-주행-화면-보기-mainpy)) |
| `yeouido/` | 도로망 · 교통량 · 신호 · 배경 생성 파이프라인 ([4절](#4-yeouido-코드-의미와-실행)) |
| `training/` | **학습**: 데이터 수집 · BC · DAgger · PPO 실행 파일, `algorithms/`(PPO · BC · 선생님 정책/안전 쉴드), `data/`(학습 데이터). 설명은 [training/README.md](training/README.md) |
| `inference/` | **추론**: 시험지 평가 `test.py`, sumo-gui 주행 · 영상 `drive_model.py`, 교통량 고정 실험 창 `fixed_population_panel.py`, `eval_set/`(시험지 · 점검 · 평가표). 설명은 [inference/README.md](inference/README.md) |
| `env/` | 학습 환경: `yeouido_env.py`(여의도 환경), `sumo_env.py`(기본 환경), `road_config.py`(도로 · 교통 설정), `mdp_config.py`(관측 · 행동 · 보상), `view_road.py`(수업 기본 직선 도로를 sumo-gui로 확인: `python -m env.view_road`) |
| `env/yeouido/` | 생성된 도로망 · 교통 · 신호 · 배경 · 랜드마크 · 워밍업 상태 |
| `utils/` | 학습 · 추론 공용: `evaluator.py`(시험지 평가), `vec_env.py`(병렬 환경), `networks.py`, `buffer.py`, `logger.py`, `eval_report.py` |
| `yeouido_cross/` | 핀포인트 경로 생성과 선생님 정책 GUI 주행 도구 ([5절](#5-기타-도구)) |
| `inspection/` | 도로망 검수 도구 (SUMO 그림 · 항공사진 비교, 수정 기록) |
| `results/` | 모델 · 학습 기록 · 평가 결과 · 평가표 · 영상 (학습 · 추론 결과가 모이는 곳) |
| `tests/` | 신호 · 도로 수정 · PPO 경계 회귀 검사 |
| `project_materials/` | 수업 실습 문서 (설치, 도로 환경, BC, 강화학습) |

## 4. `yeouido/` 코드: 의미와 실행

모든 명령은 저장소 최상위에서 `python -m yeouido.<이름>`으로 실행한다.

| 파일 | 의미 | 실행 |
|---|---|---|
| `run_all.py` | 아래 단계를 순서대로 모두 실행 (다운로드 → 도로망 → 교통량 → 수요 → 배경 · 랜드마크 → 그림 → 검증) | `python -m yeouido.run_all` (검증 생략: `--skip-validate`) |
| `config.py` | 공통 설정: 범위, 측정 지점, 랜드마크, 시간대, 파일 경로 | (설정 파일, 직접 실행 안 함) |
| `download.py` | 1단계: TOPIS 교통량 엑셀 · OSM 도로 · OSM 주변 환경 다운로드 | `python -m yeouido.download` (전부 다시: `--refresh`) |
| `nodelink.py` | 전국 표준노드링크 shapefile에서 범위 안 노드 · 링크 · 회전 제한 추출 → `region.json` | `python -m yeouido.nodelink` (원본을 `env/yeouido/source/nodelink/raw/`에 받아 둬야 함) |
| `build_net.py` | 2단계: 노드링크 → SUMO 도로망 `yeouido.net.xml` (교차로 합치기 · 회전 제한 · 신호 · 수정 목록 반영) | `python -m yeouido.build_net` (수정 없이: `--no-corrections`, 다른 곳에: `--out <경로>`) |
| `corrections.py`, `corrections.json` | 검수로 확인한 도로망 수정 목록 (차로 수 · 회전 연결 등) | (`build_net.py`가 읽음) |
| `road_corrections.py`, `signal_corrections.py` | netconvert 뒤 확인된 도로 연결 · 신호 순서 보정 | (`build_net.py`가 읽음) |
| `prepare_counts.py` | 3단계: TOPIS 엑셀 → 측정 지점별 평일 시간대 평균 교통량 `counts.json` | `python -m yeouido.prepare_counts` |
| `map_counts.py` | 4단계: 측정 지점 → SUMO 도로 연결, 출입 게이트(A~F, ㄱ~ㄷ) 지정 | `python -m yeouido.map_counts` |
| `build_demand.py` | 5단계: 실측 교통량 → 시간대별 일반 차량 수요(경계 출입 방식, `YEOUIDO_DEMAND=measured`) | `python -m yeouido.build_demand` |
| `build_demand_uniform.py` | 현재 기본 교통 "uniform": 실측 대수 그대로, 출발 · 도착은 도로망 전체에 고르게 | `python -m yeouido.build_demand_uniform` (배율: `--scale 0.8`) |
| `tdata_signals.py` | T DATA 신호 기록 → SUMO 신호 프로그램 `signals_<시간대>.add.xml` | `python -m yeouido.tdata_signals --history <CSV/ZIP>` |
| `tdata_current.py` | T DATA 실시간 신호 상태 API 조회 · 기록 | `python -m yeouido.tdata_current --seconds 480 --interval 3` |
| `audit_signals.py` | 모델링된 신호 전체 목록 · 근거 점검 | `python -m yeouido.audit_signals` |
| `build_context.py` | 건물 · 공원 · 한강 · 토지 이용 폴리곤과 지명 | `python -m yeouido.build_context` |
| `landmarks.py` | 랜드마크 7곳 위치를 구해 가장 가까운 차도에 연결 | `python -m yeouido.landmarks` |
| `validate.py` | 6단계: 70분 시뮬레이션으로 실측 비교 · 안정성 검사 | `python -m yeouido.validate` (한 시간대: `--level rush`) |
| `plot.py` | 도로망 · 측정 도로 · 게이트 그림 `env/yeouido/network.png` | `python -m yeouido.plot` |
| `view.py` | sumo-gui로 도로 · 교통만 보기 (학습 없이) | `python -m yeouido.view --level night` (netedit: `--netedit`) |
| `live.py` | sumo-gui + 시간대 전환 버튼 (심야 / 주간 / 출근) | `python -m yeouido.live --level rush` |
| `visualization.py` | 지하차도를 밝은 회색으로 표시하는 화면용 도로망 복사본 생성 | (`view` · `live`가 사용) |
| `build_net_osm.py` | (이전 방식, 참고용) OSM → 도로망 | `python -m yeouido.build_net_osm` |
| `requirements.txt`, `requirements-drive.txt` | 도로 도구용 / 주행 · 학습용 패키지 목록 | `pip install -r yeouido/requirements-drive.txt` |

표준노드링크 원본(약 270 MB)은 저장소에 없다. 도로망을 처음부터 다시 만들 때만 국가교통정보센터(its.go.kr) → 표준노드링크에서 받아 `env/yeouido/source/nodelink/raw/`에 푼다. 없으면 `run_all`은 저장소의 `region.json`을 그대로 쓴다.

## 5. 기타 도구

| 파일 | 의미 | 실행 |
|---|---|---|
| `yeouido_cross/build_routes.py` | 핀포인트 13곳 사이 최단거리 경로 156개 생성 → `yeouido_cross/output/routes.json` | `python -m yeouido_cross.build_routes` |
| `yeouido_cross/drive_panel.py` | 시간대 · 출발 · 도착을 고르는 조절 창, 선생님 정책으로 반복 주행 | `python -m yeouido_cross.drive_panel --level rush` |
| `yeouido_cross/drive_route.py` | 경로 하나를 선생님 정책으로 주행 (조절 창 없이) | `python -m yeouido_cross.drive_route 1 B --level day` |
| `inference/drive_model.py` | **학습한 모델로 주행 · 영상 녹화** ([8절](#8-주행-실행-방법)) | `python -m inference.drive_model --problem 12 --reroute` |
| `inference/test.py` | 모델 · 선생님 평가 (시험지, 화면 없이 병렬) 또는 GUI 재생 ([8절](#8-주행-실행-방법)) | `python -m inference.test results/ppo_v1/best/model.pt --eval-set inference/eval_set/random_60_s1.json --nogui --workers 8` |
| `inference/eval_set/build_eval_set.py` | 고정 시험지 만들기 | `python -m inference.eval_set.build_eval_set` |
| `inference/eval_set/final_report.py` | 결과 JSON 여러 개 → 최종 평가표(md · csv) | `python -m inference.eval_set.final_report --final "PPO=results/uniform_newnet/ppo.json" --compare "DAgger=results/uniform_newnet/dagger.json"` |
| `inference/eval_set/demand_check.py` | 교통 수요의 시간별 차량 수 · 정체 · 순간이동 측정 | `python -m inference.eval_set.demand_check --demand uniform --level rush` |
| `inference/eval_set/reroute_check.py`, `reroute_verify.py`, `reroute_decide.py` | 경로 재탐색 효과 확인 · 재검증 · 판정 | `python -m inference.eval_set.reroute_check` |
| `inspection/serve.py` | 도로망 검수 도구 (브라우저에서 교차로 · 도로 카드 비교, 수정 기록) | `python -m inspection.serve` |
| `training/collect_pomdp_data.py` | 학습 데이터 수집 (선생님 · DART · DAgger) | `python -m training.collect_pomdp_data --policy sumo-expert --episodes 200 --workers 8 --dart-sigma 0.15` |
| `training/train_bc_hybrid.py` | BC 학습 (PPO와 같은 신경망) | `python -m training.train_bc_hybrid --data training/data/expert_dart200_v2.npz` |
| `training/dagger.py` | DAgger 반복 (수집 → 검증 → 학습 → 평가) | `python -m training.dagger --init results/bc_hybrid_v2/model.pt --base-data training/data/expert_dart200_v2.npz --iters 3 --episodes 100 --betas 0.5,0.25,0` |
| `training/train.py` | PPO 학습 (DAgger 모델에서 출발, 중단 시 `--resume`) | `python -m training.train --init results/dagger_v1/iter1/model.pt --workers 8` |

화면 없는 실행(평가 · 학습 · 수집)에는 앞에 `LIBSUMO_AS_TRACI=quiet`를 붙이면 약 2배 빠르다 (결과는 같음). sumo-gui 실행에는 붙이지 않는다.

## 6. 설치

저장소에는 최종 모델·교통·워밍업 상태·학습 데이터·결과가 모두 들어 있어서, 받은 뒤 아래 설치만 하면 바로 주행·평가할 수 있다.
작업 환경은 Python 3.10, SUMO 1.27.1(pip), PyTorch 2.x에서 확인했다.

### 6-1. 저장소 받기

```bash
git clone <저장소 주소> bmil_ws      # 처음 (약 1.5 GB). 이미 있으면 git pull
cd bmil_ws
```

내 변경이 있는 상태에서 `git pull` 하면 같은 파일(`env/road_config.py`, `env/yeouido/states/` 등)에서 충돌할 수 있다. 먼저 commit 하거나 `git stash` 한다.

### 6-2. 가상환경과 패키지

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

### 6-3. PyTorch (컴퓨터에 맞는 것 하나)

```bash
python -m pip install torch                                                       # macOS (Apple GPU 자동 사용)
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu      # Windows·Linux CPU
python -m pip install torch --index-url https://download.pytorch.org/whl/cu121    # NVIDIA GPU (CUDA 12.1)
```

주행·평가는 CPU만으로 충분하다. 학습(PPO 등)은 GPU가 있으면 빠르다.

### 6-4. SUMO_HOME 설정 (새 터미널마다, 또는 셸 설정 파일에 추가)

```bash
export SUMO_HOME="$(python -c 'import sumo; print(sumo.SUMO_HOME)')"            # macOS · Linux
```

```powershell
$env:SUMO_HOME = python -c "import sumo; print(sumo.SUMO_HOME)"                  # Windows PowerShell
```

기존에 따로 설치한 SUMO를 쓰려면 `SUMO_HOME`을 그 설치 폴더로 둔다. 버전이 1.27.1과 다르면 결과가 조금 달라질 수 있다.

### 6-5. 설치 확인

```bash
# 최종 모델로 3문제 평가 (화면 없음, 3개 병렬로 수 분). 결과가 이렇게 나오면 정상: #19 · #41 도착, #0 시간초과 (출근 정체)
LIBSUMO_AS_TRACI=quiet python -m inference.test results/ppo_v1/best/model.pt \
    --eval-set inference/eval_set/random_60_s1.json --nogui --limit 3 --workers 3

# sumo-gui 창에서 최종 모델 주행 (대표 영상과 같은 주행)
python -m inference.drive_model --problem 12 --reroute
```

Windows PowerShell 에서는 `LIBSUMO_AS_TRACI=quiet` 대신 먼저 `$env:LIBSUMO_AS_TRACI="quiet"`를 실행한다 (GUI 실행 전에는 `Remove-Item Env:LIBSUMO_AS_TRACI`).

### 6-6. 참고

- **GUI:** 운영체제의 Tk와 그래픽 실행 환경이 필요하다. 화면 없는 서버에서는 `--nogui` 평가만 된다.
- **한글 글꼴:** 지도 핀 이름과 영상 상태 띠에 쓴다. macOS는 기본 글꼴(AppleGothic), Windows는 맑은 고딕, Linux는 `fonts-nanum` 또는 `fonts-noto-cjk`를 설치한다.
- **LIBSUMO_AS_TRACI=quiet:** 화면 없는 평가·학습을 약 2배 빠르게 한다 (결과는 같음). sumo-gui 실행에는 붙이지 않는다.
- **원본 주행 기록:** 100 MB 넘는 원본 주행 기록 3개는 조각으로 올라가 있다. 학습·주행에는 필요 없고, 쓸 때만 [큰 파일 합치기](#7-2-큰-파일-합치기)를 한다.
- **설치 문서:** OS별 SUMO 설치 상세는 [실습 문서](project_materials/)에 있다.


## 7. 알려진 한계와 참고

### 7-1. 한계

- **교통 과포화:** 배율 1 교통은 이 SUMO 도로망에 과포화라, 주간 · 출근에는 주행 중 점점 막히고 교차로 꼬리물기가 생긴다 (출근 30분 동안 도로망 차량 약 5,500 → 10,000대). 운전자 설정으로는 13 % 이하만 줄었고, 실측 대수를 지키려고 배율 1을 유지했다. 실패 문제의 대부분이 이 정체 때문이다.
- **신호:** 229개 신호 중 T DATA 실측 주기는 일부(주간 51 · 출근 44곳)이고 나머지는 SUMO 자동 신호다.
- **경로 재탐색:** 회전 대기 줄을 넣은 뒤 주간 · 출근 32문제에서 +3(회복 5 · 손실 2)으로 효과가 보이지만, 60문제 전체 정식 평가는 재탐색을 끈 결과로 보고했다. 정체가 심하면 다른 길도 막혀 실패하는 문제가 남는다.
- **영상:** `results/video/reroute/`의 시나리오 18개는 이전 도로망 · DAgger로 녹화한 것이다.

### 7-2. 큰 파일 합치기

GitHub는 파일 하나가 100 MB를 넘으면 받지 않아서, 원본 주행 기록 3개는 90 MB 조각(`.part_*`)으로 나눠 올렸다.
학습 · 주행에는 필요 없고(NPZ 로 충분), 원본 기록이 필요할 때만 합친다.

```bash
for f in dagger_v1_iter3 expert_dart200_v2 expert_dart207; do
    cat training/data/$f.jsonl.gz.part_* > training/data/$f.jsonl.gz
done
```

도로 · 주변 환경: © OpenStreetMap contributors ([ODbL](https://www.openstreetmap.org/copyright)). 도로망: 국토교통부 국가표준노드링크. 교통량: 서울시 TOPIS. 신호: 서울시 T DATA.

## 8. 주행 실행 방법

모든 명령은 **저장소 최상위 폴더**(`bmil_ws/`)에서 실행한다. 설치는 [6절](#6-설치).

### 8-1. 학습한 모델로 주행 보기: `inference/drive_model.py`

sumo-gui 창에서 최종 모델(PPO)이 운전한다. 파란색은 처음 경로, 주황색은 재탐색으로 바뀐 경로다.

```bash
# 대표 영상과 같은 주행 (출근, 서강대교 → 영등포역, 경로 재탐색 켬)
python -m inference.drive_model --problem 12 --reroute

# 출발 · 도착 · 시간대를 직접 지정 (6 = 당산역 → B = 마포대교, 출근)
python -m inference.drive_model --origin 6 --destination B --level rush --reroute

# 영상(mp4)과 주행 기록(CSV) 저장
python -m inference.drive_model --problem 12 --reroute --record results/video/demo.mp4

# 다른 모델로 주행
python -m inference.drive_model --model results/dagger_v1/iter1/model.pt --problem 12
```

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `--model` | `results/ppo_v1/best/model.pt` | 운전할 모델 파일. 기본은 최종 모델 PPO |
| `--problem` | 없음 | 시험지 문제 번호. 그 문제의 시간대 · 출발 · 도착 · SUMO 시드를 그대로 재현 (평가와 같은 주행) |
| `--eval-set` | `inference/eval_set/random_60_s1.json` | `--problem` 이 가리키는 시험지 |
| `--origin` | 무작위 | 출발지: 랜드마크 번호 `1`~`7`, 게이트 글자 `A`~`F` · `ㄱ`~`ㄷ`, 또는 이름 (아래 표) |
| `--destination` | 무작위 | 도착지 (형식은 `--origin` 과 같음) |
| `--level` | `rush` | 교통 시간대: `night`(심야) · `day`(주간) · `rush`(출근). `--problem` 과 같이 쓰면 같은 출발 · 도착 · 시드로 시간대만 바꿈 |
| `--seed` | `0` | `--problem` 없이 주행할 때 SUMO 시드 |
| `--reroute` | 꺼짐 | 경로 재탐색 켜기. 막히면 같은 목적지로 더 빠른 길을 찾고 주황색으로 표시 |
| `--record` | 없음 | 영상 저장 경로(`.mp4`). 같은 이름 `.csv` 에 스텝별 주행 기록 |
| `--fps` | `10` | 영상 초당 프레임 (스텝 0.5초 → 10 fps = 5배속) |
| `--size` | `1280x720` | 녹화 해상도 |
| `--max-seconds` | 없음 | 출발 후 이 시간(초)이 지나면 멈춤 |
| `--zoom` | 설정값 | 화면 확대 배율 |
| `--delay` | `0` | sumo-gui 재생 지연(ms). 천천히 보려면 50~100 |
| `--hold` | 꺼짐 | 주행이 끝나도 창을 닫지 않음 (직접 닫으면 끝) |
| `--keep-frames` | 꺼짐 | 영상을 만든 뒤 PNG 프레임을 지우지 않음 |
| `--label` | 모델 경로에서 자동 | 화면 · 영상 제목의 모델 이름 (PPO / DAgger / BC) |
| `--check` | 없음 | 평가 결과 JSON. `--problem` 과 같이 쓰면 주행 결과가 평가와 같은지 확인 (추론 = 평가 검증) |
| `--change-dest`, `--at` | 없음, `120` | 주행 중 목적지 변경 시험용 (보통 쓰지 않음, 목적지는 출발 때 정함) |

출발지 · 도착지:

| 입력 | 랜드마크 | 입력 | 게이트 (실제 이름) |
|---|---|---|---|
| `1` | 국회의사당 | `A` | 서강대교 |
| `2` | IFC 서울 | `B` | 마포대교 |
| `3` | 63빌딩 | `C` | 원효대교 |
| `4` | 여의도공원 | `D` | 양화대교 |
| `5` | 영등포역 | `E` | 오목교 |
| `6` | 당산역 | `F` | 거리공원 |
| `7` | 노량진수산시장 | `ㄱ` · `ㄴ` · `ㄷ` | 여의교 · 서울교 · 여의2교 |

영상 위쪽 상태 띠에는 시간, 속도, 남은 거리, 주변(100 m) · 도로망 차량 수, 목적지, 재탐색 횟수가 나온다.

### 8-2. 시험지로 평가하기: `inference/test.py`

화면 없이 고정 시험지를 풀어 도착 · 충돌 등을 잰다. 모델 대신 `--expert` 를 주면 선생님 정책을 평가한다.

```bash
# 최종 모델 60문제 평가 (8개 병렬, 약 25분) → results/uniform_newnet/ppo.json 과 같은 결과
LIBSUMO_AS_TRACI=quiet python -m inference.test results/ppo_v1/best/model.pt \
    --eval-set inference/eval_set/random_60_s1.json --workers 8 --nogui --out results/my_eval.json

# 빠른 점검 (골고루 3문제)
LIBSUMO_AS_TRACI=quiet python -m inference.test results/ppo_v1/best/model.pt \
    --eval-set inference/eval_set/random_60_s1.json --nogui --limit 3 --workers 3

# 선생님 정책 기준선
LIBSUMO_AS_TRACI=quiet python -m inference.test --expert --eval-set inference/eval_set/random_60_s1.json --workers 8 --nogui

# 시험지 없이 무작위 출발 · 도착으로 GUI 재생 3판
python -m inference.test results/ppo_v1/best/model.pt --episodes 3
```

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `model` (맨 앞 위치 인자) | 가장 최근 `results/*/model.pt` | 평가할 모델 파일 |
| `--eval-set` | 없음 | 고정 시험지 JSON. 주면 `--episodes` 대신 시험지 문제를 모두 푼다 |
| `--episodes` | `3` | 시험지 없이 재생 · 평가할 판 수 |
| `--nogui` | 꺼짐 | 화면 없이 지표만 측정 (빠름). `--workers` 병렬은 이때만 |
| `--workers` | `1` | 시험지 평가를 SUMO 몇 개로 병렬 실행할지 |
| `--limit` | `0` (전부) | 시험지에서 골고루 건너뛰며 일부 문제만 |
| `--out` | 모델 폴더 | 결과 JSON 경로 (문제마다 결과 · 걸린 스텝 · 빨간불 · 쉴드 개입 · 주변 차량 수) |
| `--expert` | 꺼짐 | 모델 대신 선생님 정책 평가 |
| `--algorithm` | `auto` | 모델 종류 `ppo` / `bc` (보통 파일에서 자동 판별) |

환경변수: `LIBSUMO_AS_TRACI=quiet` (화면 없는 실행 약 2배 빠름, GUI에는 쓰지 않음), `YEOUIDO_REROUTE=1` (경로 재탐색 켬), `YEOUIDO_DEMAND=measured` (이전 경계 출입 교통).
Windows PowerShell 에서는 먼저 `$env:LIBSUMO_AS_TRACI="quiet"` 를 실행한다.

### 8-3. 선생님 정책으로 경로 주행 (학습 모델 없이)

```bash
python -m yeouido_cross.drive_panel --level rush            # 조절 창에서 시간대 · 출발 · 도착 고르기
python -m yeouido_cross.drive_route 1 B --level day         # 국회의사당 → 마포대교 하나만
python -m yeouido.live                                      # 배경 교통만 보기 (시간대 전환 버튼)
```

### 8-4. 교통량 고정 모드 (실험): `inference/fixed_population_panel.py`

시간대별 일반 차량 수(심야 462 · 주간 3,338 · 출근 4,417대)를 도로망 전체에 고르게 배치하고, 도착한 만큼 다시 채워 그 대수를 유지한다. 배율 1 교통이 점점 막히는 과포화 · 꼬리물기의 대안으로 시험 중이며, 지금은 선생님 정책 GUI 창에서만 쓴다 (학습 · 평가 환경에는 아직 연결 안 됨).

```bash
python -m inference.fixed_population_panel                       # Windows: 여의도_분산교통_실행.cmd
python -m inference.fixed_population_panel --validate --seconds 180
```

자세한 규칙과 주의점은 [inference/README.md](inference/README.md#교통량-고정-분산-배치--보충-fixed_population_panelpy).

## 9. 한 번에 주행 화면 보기: `main.py`

저장소 최상위의 `main.py`는 **최종 모델(PPO)의 자율주행을 sumo-gui로 바로 보여 주는 실행 파일**이다.
출발지 · 목적지를 주지 않으면 **선택 창**이 떠서 교통 시간대 · 출발지 · 목적지 · 경로 재탐색 · 재생 속도 · 영상 저장을 고르고 [주행 시작]을 누른다.
둘 다 주면 선택 창 없이 바로 주행한다. 운전은 `inference/drive_model.py`와 같다 (같은 환경 · 안전 쉴드 · 결정적 행동).

```bash
python main.py                                                    # 선택 창에서 고르고 주행
python main.py --level rush --origin 6 --destination B            # 출근, 당산역 → 마포대교 (2배속)
python main.py --level day --origin 국회의사당 --destination 당산역 --reroute --speed 4
python main.py --level night --origin 1 --destination 3 --record results/video/demo.mp4   # 영상 + 주행 기록 저장
python main.py --list                                             # 출발지 · 목적지 목록
```

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `--level` | `rush` | 교통 시간대: `night`(심야 03시) · `day`(주간 12시) · `rush`(출근 08시) |
| `--origin` | 선택 창 | 출발지: 랜드마크 `1`~`7`, 게이트 `A`~`F` · `ㄱ`~`ㄷ`, 또는 이름 (`국회의사당`, `마포대교` 등, [8-1절 표](#8-1-학습한-모델로-주행-보기-inferencedrive_modelpy)) |
| `--destination` | 선택 창 | 목적지 (형식은 `--origin`과 같음). 목적지는 출발 때 정하고 주행 중 바꾸지 않는다 |
| `--reroute` | 꺼짐 | 경로 재탐색: 60초 넘게 막히면 같은 목적지로 30 % · 60초 이상 빠른 길(회전 대기 줄 포함)을 찾아 주황색으로 표시 |
| `--speed` | `2` | 재생 배속: `1` · `2` · `4` · `8` (실제 시간 대비), `max`(지연 없이 최대). 차가 많은 출근 시간대는 계산 때문에 3~4배속 정도가 한계 |
| `--delay` | 없음 | sumo-gui 스텝 지연(ms)을 직접 지정 (주면 `--speed` 무시) |
| `--model` | `results/ppo_v1/best/model.pt` | 운전할 모델 (예: DAgger `results/dagger_v1/iter1/model.pt`) |
| `--seed` | `0` | SUMO 시드 (같은 값이면 같은 교통) |
| `--record` | 없음 | 영상 저장 경로(`.mp4`). 같은 이름 `.csv`에 스텝별 주행 기록. 녹화할 때는 지연 없이 돈다 |
| `--hold` | 꺼짐 | 주행이 끝나도 sumo-gui 창을 닫지 않음 |
| `--list` | — | 출발지 · 목적지 목록 출력 |

- 주행 중에도 sumo-gui 위쪽 **Delay** 칸 숫자를 바꾸면 속도가 바로 바뀐다 (작을수록 빠름).
- 화면: 파란색 = 처음 경로, 주황색 = 재탐색으로 바뀐 경로, 빨간 차 = ego. 핀은 랜드마크(노란 원) · 게이트(빨간 네모).
- `LIBSUMO_AS_TRACI`가 켜져 있으면 sumo-gui를 띄울 수 없으니 끄고 실행한다.
