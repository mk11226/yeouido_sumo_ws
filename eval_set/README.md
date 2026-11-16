# 평가 시험지 (여의도)

모든 모델(선생님 정책 · BC · DAgger · PPO)을 **똑같은 150문제**로 비교하기 위한 고정 문제 목록.
학습에는 쓰지 않고 평가에만 쓴다.

| 파일 | 내용 |
|---|---|
| `eval_set.json` | 시험지 150문제 |
| `build_eval_set.py` | 시험지를 만드는 코드 |

## 평가하기

```bash
source .venv/bin/activate

# 학습한 모델 평가
python test.py results/<run>/model.pt --eval-set eval_set/eval_set.json --nogui --workers 4

# 선생님 정책(기준선) 평가
python test.py --expert --eval-set eval_set/eval_set.json --nogui --workers 4

# 빠른 점검 (골고루 6문제만)
python test.py results/<run>/model.pt --eval-set eval_set/eval_set.json --nogui --limit 6

# 결과 여러 개 나란히 비교
python -m utils.eval_report results/sumo_expert/eval_set_results.json results/<run>/eval_set_results.json
```

결과는 모델 폴더의 `eval_set_results.json`에 문제마다 남는다 (도착 / 충돌 / 시간초과, 걸린 시간, 속도 등).
표는 시간대 · 경로 종류 · 지나는 다리별로 나눠 보여 준다.
`--limit 6`이면 파일명에 `_limit6`가 붙는다. `--out /path/result.json`으로 위치를 지정할 수 있다.
설치와 SUMO_HOME 설정은 [루트 README](../README.md#설치)를 참고한다.

## 시험지 구성

- **150문제 = 출발·도착 50쌍 × 시간대 3개** (심야 · 주간 · 출근)
- 한 문제는 `env.reset(options=...)`에 그대로 넣는 조건이다.

```json
{"id": 2, "level": "rush", "origin": "국회의사당", "destination": "당산역",
 "snapshot_time": 1800, "sumo_seed": 1002, "category": "섬↔본토", "bridges": ["여의2교"], "route_m": 3139}
```

| 칸 | 뜻 |
|---|---|
| `level` | 시간대 (`night` / `day` / `rush`) |
| `origin`, `destination` | 출발, 도착 (랜드마크 또는 게이트, [핀포인트 표](../yeouido_cross/README.md#핀포인트-13곳)) |
| `snapshot_time` | 배경 교통이 몇 초 흐른 상태에서 출발할지 (600 / 1200 / 1800 / 2400) |
| `sumo_seed` | 교통 난수 시드. **같은 문제는 몇 번을 돌려도 배경 차량까지 똑같이 움직인다** |
| `category`, `bridges`, `route_m` | 결과를 묶어 보기 위한 참고값 (경로 종류, 지나는 샛강 다리, 경로 길이) |

| 경로 종류 | 쌍 | 문제 |
|---|---|---|
| 섬↔본토 (샛강 건넘) | 20 | 60 |
| 여의도 안 / 영등포 안 | 9 / 6 | 27 / 18 |
| 게이트 출발·도착 | 15 | 45 |

## 원본 브랜치의 과거 점수 (선생님 정책, 2026-11-16)

아래는 `seonu` 브랜치에 기록된 과거 결과다. 현재 프로젝트의 교통량 `1.0`, 변경된 도로망·신호
조건에서 전체 150문제를 재평가한 결과가 아니므로 현재 성능으로 인용하지 않는다.

| 충돌 | 완주 | 시간초과 | 평균 속도 | 완주 소요 |
|---|---|---|---|---|
| 2.0% (3/150) | 98.0% | 0% | 35.7 km/h | 6.8분 |

교차로 양보 수정(`algorithms/sumo_expert.py`) 후 값. 수정 전은 충돌 3.3% (5/150).

## 시험지 다시 만들기

도로망이나 랜드마크·게이트가 바뀌었을 때만 다시 만든다. 다시 만들면 이전 결과와 점수를 비교할 수 없다.

```bash
python -m eval_set.build_eval_set --check   # 150문제 모두 출발이 되는지 확인한 뒤 저장
```

- `--check`를 붙여야 `route_m`이 실제 주행 환경 기준 길이로 기록된다.
- `env/road_config.py`의 `places`가 `"landmarks+gates"`여야 한다.
