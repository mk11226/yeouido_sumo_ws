# training/: 학습 코드

선생님 데이터 수집 → BC → DAgger → PPO 순서로 자율주행 정책을 학습한다.
모든 명령은 **저장소 최상위**(`bmil_ws/`)에서 `python -m training.<이름>`으로 실행한다 (`python training/<이름>.py`도 된다).
화면 없는 실행에는 앞에 `LIBSUMO_AS_TRACI=quiet`를 붙이면 약 2배 빠르다 (결과는 같음).
결과(모델 · 학습 기록)는 저장소 최상위 `results/`에, 데이터는 `training/data/`에 저장된다.

## 폴더 구성

| 경로 | 내용 |
|---|---|
| `collect_pomdp_data.py` | 주행 데이터 수집 (선생님 · 무작위 · 학습 모델, DART 잡음, DAgger 혼합 운전). JSONL(전체 관측 · 상태) + NPZ(학습용) |
| `train_bc_hybrid.py` | BC: PPO와 같은 신경망(256×256, 가감속 μ + 차로 3방향)으로 선생님 행동 따라 하기. 저장 형식이 PPO와 같아 바로 PPO 미세조정 가능 |
| `train_bc.py` | (수업 기본) BCPolicy로 두 행동을 MSE 회귀하는 BC |
| `dagger.py` | DAgger 반복: 모델이 운전하며 수집 → 데이터 검증 → BC 재학습 → 시험지 평가, 가장 좋은 모델 선택 |
| `train.py` | PPO 학습. 하이퍼파라미터 블록(`HPARAMS`), 보상 설정(`REWARD`), 검증 시험지로 최고 모델 저장, 중단 시 이어 학습 |
| `salvage_parts.py` | 수집을 중간에 멈췄을 때 워커별 부분 기록에서 끝난 에피소드만 모아 NPZ 복원 |
| `algorithms/ppo.py` | PPO (병렬 환경 rollout, critic 워밍업, KL 조기 종료, 체크포인트 저장 · 불러오기) |
| `algorithms/bc.py` | `train_bc.py`의 BCPolicy |
| `algorithms/sumo_expert.py` | 선생님 정책(SumoExpert): SUMO 운전자 모델 기반 목표 속도 · 차로 변경. 환경의 **안전 쉴드**도 이 계산을 쓴다 |
| `data/` | 학습 데이터. `expert_dart200_v2.npz`(선생님 DART 200판, BC · DAgger 기반), `dagger_v1_iter1~3.npz`(DAgger 수집), `*.jsonl.gz`(원본 주행 기록, 100 MB 넘는 것은 `.part_*` 조각) |

## 학습 순서와 명령

```bash
# 1) 선생님 데이터 (DART 잡음 0.15, 8개 병렬) → training/data/expert_dart200_v2.npz
LIBSUMO_AS_TRACI=quiet python -m training.collect_pomdp_data --policy sumo-expert --episodes 200 \
    --workers 8 --dart-sigma 0.15 --name expert_dart200_v2

# 2) BC → results/bc_hybrid_v2/
python -m training.train_bc_hybrid --data training/data/expert_dart200_v2.npz --out-dir results/bc_hybrid_v2

# 3) DAgger 3회 → results/dagger_v1/iter1~3/ (1회차가 최고)
LIBSUMO_AS_TRACI=quiet python -m training.dagger --init results/bc_hybrid_v2/model.pt \
    --init-result results/bc_hybrid_v2/random_60_s1_results.json \
    --base-data training/data/expert_dart200_v2.npz --iters 3 --episodes 100 --betas 0.5,0.25,0 --out dagger_v1

# 4) PPO 미세조정 (기본 출발 모델 = DAgger 1회차) → results/ppo_v1/best/model.pt
LIBSUMO_AS_TRACI=quiet python -m training.train --name ppo_v1 --workers 8
LIBSUMO_AS_TRACI=quiet python -m training.train --resume results/ppo_v1      # 중단된 학습 이어서
LIBSUMO_AS_TRACI=quiet python -m training.train --smoke                      # 몇 분짜리 동작 확인
```

학습이 끝나면 평가와 주행은 [inference/README.md](../inference/README.md)를 따른다.

## 주요 옵션

### `train.py` (PPO)

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `--init` | `results/dagger_v1/iter1/model.pt` | 이어 학습할 출발 모델 (BC · DAgger · PPO의 model.pt) |
| `--scratch` | 꺼짐 | 무작위 신경망부터 학습 (`--init` 무시) |
| `--resume` | 없음 | 이어서 학습할 run 폴더 (`ckpt.pt`, `state.json`이 있는 곳) |
| `--total` | 설정값 | 총 학습 스텝 수 |
| `--workers` | 설정값 | 병렬 환경 수 (SubprocVecEnv) |
| `--n-steps` | 512 | 환경 하나당 rollout 길이 |
| `--critic-warmup` | 3 | 처음 몇 번은 가치 함수(critic)만 학습 |
| `--eval-set` | `inference/eval_set/random_30_s2.json` | 학습 중 모델 고르기용 검증 시험지 (최종 성적표와 별개) |
| `--eval-every` | 200,000 | 이 스텝마다 검증 평가, 더 좋으면 `best/`에 저장 |
| `--eval-limit` | 0 | 검증 시험지에서 이 문제 수만 (0 = 전부) |
| `--no-init-eval` | 꺼짐 | 출발 모델 기준선 평가 생략 |
| `--name` | 날짜 시각 | 결과 폴더 이름 `results/<name>/` |
| `--smoke` | 꺼짐 | 동작 확인용: 환경 2개, 짧은 rollout, 검증 3문제 |

학습률 · 엔트로피 · KL 한도 등은 `train.py` 위쪽 `HPARAMS` · `FINETUNE` 블록에서 바꾼다. 보상 값은 `env/mdp_config.py`와 `train.py`의 `REWARD`.

### `collect_pomdp_data.py` (데이터 수집)

| 옵션 | 의미 |
|---|---|
| `--policy` | 행동 정책 `random` · `keep-lane` · `sumo-expert` · `ppo` · `bc` |
| `--episodes` | 수집할 에피소드 수 |
| `--workers` | 병렬 SUMO 수 (코어 수만큼 주면 거의 그 배수로 빨라짐) |
| `--dart-sigma` | DART: 실제 실행하는 가감속에만 N(0, σ) 잡음 (저장 라벨은 원래 행동). 0.1~0.2 권장 |
| `--dagger-model`, `--beta` | DAgger: 이 모델이 운전하고 라벨은 선생님 행동. 스텝마다 β 확률로 선생님이 대신 운전 |
| `--model`, `--stochastic` | 학습한 모델로 수집 (샘플링 행동) |
| `--out-dir`, `--name` | 저장 위치 (기본 `training/data/`)와 파일 이름 |
| `--gui` | sumo-gui로 보면서 수집 (1개만) |

저장 칼럼 설명은 저장소 밖 `8조_칼럼설명.csv` (필수 항목 + 여의도 전용 관측 · 상태).

### `train_bc_hybrid.py` (BC)

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `--data` | (필수) | 학습 NPZ, 여러 개면 합친다 |
| `--epochs`, `--batch-size`, `--lr` | 50, 512, 1e-3 | 학습 횟수 · 배치 · 학습률 (cosine 감쇠) |
| `--lane-weight` | 5.0 | 차로 변경 손실 가중치 |
| `--select` | `accel` | 모델을 고르는 검증 손실 (가감속 MSE 또는 합계) |
| `--patience` | 8 | 검증 손실이 이만큼 epoch 동안 안 좋아지면 멈춤 |
| `--init` | 없음 | 이어서 학습할 모델 (DAgger 반복용) |
| `--save-log-std` | −1.6 | 저장할 가감속 σ (PPO 시작 탐험 폭) |
| `--device` | `auto` | cuda > mps > cpu 자동 |

### `dagger.py`

| 옵션 | 의미 |
|---|---|
| `--init`, `--init-result` | 출발 BC 모델과 그 평가 결과 (없으면 처음에 평가) |
| `--base-data` | 처음 선생님 데이터 NPZ (반복마다 지금까지 데이터 전부와 합쳐 학습) |
| `--iters`, `--episodes`, `--betas` | 반복 횟수, 반복마다 수집 에피소드 수, 반복별 β |
| `--eval-set` | 반복마다 평가할 시험지 (기본 `inference/eval_set/random_60_s1.json`) |
| `--out` | 결과 폴더 `results/<out>/` (요약 `summary.json`) |
