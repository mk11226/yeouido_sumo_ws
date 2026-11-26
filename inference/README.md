# inference/: 추론 (평가 · 주행)

학습한 모델이 잡음 없이 운전한다 (가감속은 평균값, 차로는 가장 확률 높은 선택, 안전 쉴드는 학습 때와 같음).
화면 없이 고정 시험지를 풀어 숫자를 내는 **평가**와, sumo-gui로 운전을 보고 영상을 남기는 **주행**이 있다.
모든 명령은 **저장소 최상위**(`bmil_ws/`)에서 `python -m inference.<이름>`으로 실행한다 (`python inference/<이름>.py`도 된다).
평가처럼 화면 없는 실행에는 앞에 `LIBSUMO_AS_TRACI=quiet`를 붙이면 약 2배 빠르다. sumo-gui 주행에는 붙이지 않는다.

## 폴더 구성

| 경로 | 내용 |
|---|---|
| `drive_model.py` | 학습한 모델로 sumo-gui 주행. 파란색 = 처음 경로, 주황색 = 재탐색으로 바뀐 경로. 영상(mp4) · 주행 기록(CSV) 저장 |
| `test.py` | 고정 시험지 평가 (병렬, 화면 없이) 또는 GUI 재생. 모델 대신 선생님 정책도 평가 |
| `eval_set/random_60_s1.json` | 최종 성적표 시험지 60문제 (문제마다 시간대 · 출발 · 도착 · SUMO 시드 고정) |
| `eval_set/random_30_s2.json` | PPO 학습 중 모델 고르기용 30문제 (성적표와 겹치지 않게 별도) |
| `eval_set/eval_set.json` | 경로 156개 기반 시험지 150문제 |
| `eval_set/build_eval_set.py` | 시험지 만들기 |
| `eval_set/final_report.py` | 평가 결과 JSON 여러 개 → 최종 평가표 `results/final/final_eval.md` · `.csv` |
| `eval_set/demand_check.py` | 교통 수요 측정 (시간별 차량 수 · 정지 비율 · 순간이동) |
| `eval_set/reroute_check.py`, `reroute_verify.py`, `reroute_decide.py` | 경로 재탐색 효과 확인 · 재검증 · 켤지 판정 |
| `eval_set/step0_check.py`, `reward_audit.py` | 학습 전 점검: 안전 쉴드 · 저크, 보상 순서 (선생님 > 서 있기 > 무작위) |

## 주행: `drive_model.py`

새로 학습한 64차원 보조 없는 모델은 `python -m inference.drive_model --model results/ppo_unassisted_v1/model.pt --unassisted --problem 12 --hold`로 실행한다.
평가에는 `python -m inference.test results/ppo_unassisted_v1/model.pt --no-assist --nogui --eval-set inference/eval_set/random_60_s1.json`를 사용한다.
학습 방법과 입력 정의는 [training/README.md](../training/README.md#보조-없는-ppo-새-학습)를 따른다.

기존 PPO를 보조 없이 시험하려면 `python -m inference.drive_model --problem 12 --no-assist --hold`를 실행한다.
`--no-assist`는 신호 정지·제한속도 강제·안전 쉴드·자동 차로 변경·차로 변경 안전 검사·경로 재탐색을 끈다.
출발 차로도 경로 기준으로 고르지 않으며, 경로별 차로 안내 관측 `[34]`, `[35]`는 0으로 가린다.
기존 모델을 불러오도록 입력은 42차원으로 유지한다. 기존 모델은 보조가 있는 환경에서 학습됐으므로
이 실행은 조건을 바꾼 시험이며 기존 평가 결과와 직접 같다고 볼 수 없다. 옵션을 생략하면 기존 동작을 유지한다.

```bash
# 대표 영상과 같은 주행 (최종 모델 PPO, 출근, 서강대교 → 영등포역, 경로 재탐색 켬)
python -m inference.drive_model --problem 12 --reroute

# 출발 · 도착 · 시간대 직접 지정 (6 = 당산역 → B = 마포대교)
python -m inference.drive_model --origin 6 --destination B --level rush --reroute

# 영상 + 주행 기록 저장
python -m inference.drive_model --problem 12 --reroute --record results/video/demo.mp4

# 주행 결과가 평가와 같은지 확인 (추론 = 평가)
python -m inference.drive_model --problem 21 --check results/uniform_newnet/ppo.json
```

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `--model` | `results/ppo_v1/best/model.pt` | 운전할 모델 (최종 PPO) |
| `--problem` | 없음 | 시험지 문제 번호. 시간대 · 출발 · 도착 · SUMO 시드를 그대로 재현 |
| `--eval-set` | `inference/eval_set/random_60_s1.json` | `--problem`의 시험지 |
| `--origin`, `--destination` | 무작위 | 랜드마크 `1`~`7`, 게이트 `A`~`F` · `ㄱ`~`ㄷ`, 또는 이름 (1 국회의사당, 2 IFC 서울, 3 63빌딩, 4 여의도공원, 5 영등포역, 6 당산역, 7 노량진수산시장 / A 서강대교, B 마포대교, C 원효대교, D 양화대교, E 오목교, F 거리공원, ㄱ 여의교, ㄴ 서울교, ㄷ 여의2교) |
| `--level` | `rush` | `night` · `day` · `rush`. `--problem`과 쓰면 같은 문제를 다른 시간대 교통으로 |
| `--seed` | `0` | `--problem` 없이 주행할 때 SUMO 시드 |
| `--reroute` | 꺼짐 | 경로 재탐색 (60초 넘게 막히면 같은 목적지로 30 % · 60초 이상 빠른 길, 바꾼 길은 끝까지 지킴) |
| `--record` | 없음 | 영상 경로(`.mp4`), 같은 이름 `.csv`에 스텝별 기록 |
| `--fps`, `--size` | `10`, `1280x720` | 영상 프레임 (10 fps = 5배속) · 해상도 |
| `--max-seconds` | 없음 | 출발 후 이 시간이 지나면 멈춤 |
| `--zoom`, `--delay` | 설정값, `0` | 화면 확대 · 재생 지연(ms) |
| `--hold` | 꺼짐 | 끝난 뒤 창을 닫지 않음 |
| `--keep-frames` | 꺼짐 | 영상 만든 뒤 PNG 프레임 유지 |
| `--label` | 자동 | 제목의 모델 이름 |
| `--check` | 없음 | 평가 결과 JSON과 결과 · 스텝 수 비교 (`--problem` 필요) |
| `--change-dest`, `--at` | 없음, `120` | 주행 중 목적지 변경 시험용 (보통 쓰지 않음) |

## 평가: `test.py`

```bash
# 최종 모델 60문제 (8개 병렬, 약 25분)
LIBSUMO_AS_TRACI=quiet python -m inference.test results/ppo_v1/best/model.pt \
    --eval-set inference/eval_set/random_60_s1.json --workers 8 --nogui --out results/uniform_newnet/ppo.json

# 선생님 정책 기준선
LIBSUMO_AS_TRACI=quiet python -m inference.test --expert --eval-set inference/eval_set/random_60_s1.json --workers 8 --nogui

# 빠른 점검 3문제 (정상: #19 · #41 도착, #0 시간초과)
LIBSUMO_AS_TRACI=quiet python -m inference.test results/ppo_v1/best/model.pt \
    --eval-set inference/eval_set/random_60_s1.json --nogui --limit 3 --workers 3

# 평가표
python -m inference.eval_set.final_report --final "PPO=results/uniform_newnet/ppo.json" \
    --compare "DAgger=results/uniform_newnet/dagger.json" "선생님=results/uniform_newnet/expert.json"
```

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `model` (위치 인자) | 가장 최근 `results/*/model.pt` | 평가할 모델 |
| `--eval-set` | 없음 | 시험지 JSON. 주면 시험지 문제를 모두 푼다 |
| `--episodes` | `3` | 시험지 없이 재생할 판 수 |
| `--nogui` | 꺼짐 | 화면 없이 지표만 (병렬은 이때만) |
| `--workers` | `1` | 병렬 SUMO 수 |
| `--limit` | `0` | 시험지에서 골고루 일부만 |
| `--out` | 모델 폴더 | 결과 JSON (문제마다 결과 · 스텝 · 빨간불 · 쉴드 개입 · 주변 차량 수 · 재탐색 횟수) |
| `--expert` | 꺼짐 | 선생님 정책 평가 |
| `--algorithm` | `auto` | `ppo` / `bc` (보통 자동 판별) |

환경변수: `YEOUIDO_REROUTE=1` (평가에서도 재탐색 켬), `YEOUIDO_DEMAND=measured` (이전 경계 출입 교통).

## 현재 결과 (새 도로망, 60문제)

| 모델 | 도착 | 충돌 | 순간이동 | 쉴드 개입/문제 |
|---|---|---|---|---|
| **PPO (최종)** | **45** | 0 | 0 | 43 |
| 선생님 | 45 | 0 | 0 | 0 |
| DAgger | 43 | 0 | 2 | 655 |

전체 표: `results/final/final_eval.md`. 평가 결과: `results/uniform_newnet/`.

## 교통량 고정 (분산 배치 · 보충): `fixed_population_panel.py`

시간대별로 **도로망 위 일반 차량 수를 정해 두고 그 대수를 유지**하는 실험용 교통 모드다 (교통 과포화 · 꼬리물기 대안).
코드는 `env/fixed_population.py`, 실행 창은 `inference/fixed_population_panel.py`. 운전자는 선생님 정책이다 (학습 모델 평가와는 별개).

```bash
python -m inference.fixed_population_panel                       # 시간대 전환 · 주행 창 (Windows: 루트의 여의도_분산교통_실행.cmd)
python -m inference.fixed_population_panel --validate --seconds 180   # 세 시간대 검증: 목표 대수 초과 · 도착 후 보충 · 충돌
python -m inference.fixed_population_panel --smoke --delay 0      # GUI 동작 확인
python -m unittest discover -s tests -p test_fixed_population.py
```

- **목표 대수:** 기존 uniform 교통 600초 시점에 도로 위에 있던 일반 차량 수 = 심야 462 · 주간 3,338 · 출근 4,417대.
- **초기 배치:** 도로 길이 · 차로 수에 비례해 도로망 전체에 고르게 (승용차 통행 가능, 길이 50 m 이상, 안전 간격 15 m 확보한 차로만, 도로마다 최소 1대).
- **운행 · 보충:** 일반 차량은 연결된 도로 두 구간을 달리고 도착한다. 부족해진 대수는 배정량보다 비어 있는 차로에 다시 넣는다 (SUMO 삽입 안전 검사를 따름, 기존 차량을 옮기거나 지우지 않음). ego 경로는 그대로.
- **목표 대수에 포함:** 도로 위 · 삽입 대기 · 순간이동 복귀 중인 차량. 그래서 매 순간 도로 위 대수가 목표와 똑같지는 않다.
- **시간대 전환:** 주행 중 바꾸면 새 목표에 맞춰 보충하고, 목표가 낮아지면 도착에 따라 자연 감소한다.
- **주의:** 차량 대수와 공간 분포만 제어하며 실제 통근 OD나 시간당 유입량을 재현하지 않는다. 배경 차량이 자주 교체되므로 누적 생성 · 도착 대수를 실측 통행량과 직접 비교하면 안 된다.
- 초기 상태 · 설정 · 검증 결과는 `env/yeouido/fixed_population/`에 자동 생성된다 (Git 제외).
