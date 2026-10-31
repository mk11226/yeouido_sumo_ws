# 여의도 ↔ 영등포 넘나드는 경로

`yeouido/` 가 만든 도로망(`env/yeouido/yeouido.net.xml`)과 랜드마크(`landmarks.json`)를 그대로 쓰고,
섬(여의도)과 본토(영등포)를 오가는 AV 경로만 따로 만든다. 도로망·교통량은 건드리지 않는다.

## 실행

```bash
conda activate sumo-rl
python -m yeouido_cross.build_routes          # 경로 생성 → yeouido_cross/output/
python -m yeouido_cross.build_routes --gui    # 생성 후 sumo-gui로 경로 겹쳐 보기
```

## 샛강 연결로 (섬 ↔ 본토)

이 링크들을 빼면 섬과 본토가 완전히 끊어지는 것을 코드에서 확인한다 (`island_edges`).

| 연결로 | 도로 | 방향 | 링크 |
|---|---|---|---|
| 여의2교 | 국회대로 | 양방향 | 1180031402 / 1180031502 |
| 서울교 | 경인로 | 양방향 | 1180001501 / 1180001601 |
| 여의교 | 여의대방로 | 양방향 | 1190006903→1190006902 / 1190007002→1190007003 |
| 여의하류 램프 | "올림픽대로" (노들로 당산 쪽 → 여의서로) | 본토 → 섬만 | 1180103300 |

랜드마크 구분: 여의도 = 국회의사당, IFC 서울, 63빌딩, 여의도공원 / 영등포 = 영등포역, 당산역, 노량진수산시장.

## 만드는 경로

섬 랜드마크 4 × 본토 랜드마크 3 × 두 방향 = 24쌍. 쌍마다 **연결로별로 "그 연결로를 건너는 가장 빠른 경로"** 를
하나씩 만든다 (섬→본토 3개, 본토→섬 4개). 총 84개.

- 출발 → 연결로 입구, 연결로 출구 → 도착을 각각 가장 빠른 경로(제한속도 기준, 회전 제한 반영)로 잇는다.
- 다른 연결로를 한 번 더 건너는 경로는 뺀다.
- 출발·도착 위치와 길이 계산은 `YeouidoEnv._insert_ego` 와 같다 (출발은 edge 끝에서 20 m 이상 남김).

**한쪽 안 경로**: 샛강을 건너지 않고 섬 안 / 영등포 안에서만 다니는 가장 빠른 경로 하나씩. 쓸 만한 것만 남긴다.
- 1 km 이상 (학습 환경 `min_trip` 과 같음)
- 도로 길이가 직선거리의 2.5배 이내 (`MAX_DETOUR`). 예: IFC 서울 ↔ 여의도공원은 직선 313 m인데 도로로
  1.7~2.2 km를 돌아가서 뺐다 (랜드마크가 붙은 차로 방향 때문에 한 바퀴 돌아야 함).

| 종류 | id | 출발·도착 후보 | 경로 |
|---|---|---|---|
| 여의도 안 | `i000`~ | 섬 랜드마크 4곳 + 북쪽 다리 게이트 A 서강·B 마포·C 원효대교 | 37 (42쌍 중 5쌍 제외) |
| 영등포 안 | `m000`~ | 본토 랜드마크 3곳 + 경계 게이트 G 양화대교·H 오목교·I 거리공원 | 30 (30쌍 모두) |

샛강 다리 위 게이트(D·E·F)는 어느 한쪽 안도 아니라서 넣지 않았다.
`routes.json` 에서는 `direction: "여의도 안"/"영등포 안"`, `bridge: null`. GUI에서는 주황(섬 안) / 청록(영등포 안).

총 151개 (연결로 84 + 여의도 안 37 + 영등포 안 30).

결과 (`output/`):

| 파일 | 내용 |
|---|---|
| `routes.json` | `{id, origin, destination, direction, bridge, fastest, length_m, free_time_s, detour, edges}` |
| `routes.rou.xml` | SUMO `<route>` 정의 (id = `routes.json` 의 id) |
| `routes.poly.xml` | sumo-gui 겹쳐 보기용 선. 연결로별 색 (빨강 여의2교, 파랑 서울교, 초록 여의교, 보라 램프, 주황 섬 안, 청록 영등포 안), 가장 빠른 경로는 굵게 |

`detour` = 그 쌍에서 가장 빠른 연결로 대비 시간 비율 (1.0 = 가장 빠름).

## 평가 시험지 (`build_eval_set.py`)

모든 모델(선생님·BC·DAgger·PPO)을 같은 150문제로 평가하기 위한 고정 목록. 학습에는 쓰지 않는다.

```bash
python -m yeouido_cross.build_eval_set --check   # → output/eval_set.json (150문제 모두 reset 되는지 확인)
```

한 문제는 `env.reset(options=...)` 에 그대로 넣는 조건이다.

```json
{"id": 2, "level": "rush", "origin": "국회의사당", "destination": "당산역", "snapshot_time": 1800, "sumo_seed": 1002,
 "category": "섬↔본토", "bridges": ["여의2교"], "route_m": 3139}
```

- `level`, `origin`, `destination`, `snapshot_time`, `sumo_seed` 다섯 칸이 조건. `category`, `bridges`, `route_m` 는
  결과를 종류별로 묶어 볼 때 쓰는 참고값.
- `sumo_seed` 를 주면 `YeouidoEnv.reset` 이 그 시드로 SUMO를 새로 켠다. 같은 문제를 두 번 돌리면 배경 차량까지
  똑같이 움직인다 (선생님 정책으로 두 번 돌려 관측값이 같음을 확인).
- `places` 가 `"landmarks+gates"` 여야 한다 (`env/road_config.py`).

### 평가 실행

```bash
# 모델 평가 (결과: 모델 폴더/eval_set_results.json + 종류별 표)
python test.py results/bc_…/model.pt --eval-set yeouido_cross/output/eval_set.json --nogui --workers 4
# 선생님 기준선 (결과: results/sumo_expert/eval_set_results.json)
python test.py --expert --eval-set yeouido_cross/output/eval_set.json --nogui --workers 4
# 빠른 점검: 시간대·종류가 고르게 섞인 일부만
python test.py results/bc_…/model.pt --eval-set yeouido_cross/output/eval_set.json --nogui --limit 6
# 여러 결과 나란히 비교 (충돌률 / 완주율, 시간대·종류·연결로별)
python -m utils.eval_report results/sumo_expert/eval_set_results.json results/bc_…/eval_set_results.json
```

- 결과 JSON 에는 문제마다 끝난 이유(도착/충돌/시간초과), 스텝 수, 보상, 속도, 빨간불 위반 수가 남는다.
- `--workers` 로 병렬로 돌려도 결과는 순차 실행과 똑같다 (문제마다 SUMO 시드로 새로 켜므로).

구성: 50쌍 × 시간대 3개 = 150문제. 워밍업 시점 4개는 돌아가며 배정 (각 37~38문제).

| 종류 | 쌍 | 문제 |
|---|---|---|
| 섬↔본토 (랜드마크끼리, 샛강 건넘) | 20 (방향별 10) | 60 |
| 여의도 안 / 영등포 안 | 9 / 6 | 27 / 18 |
| 게이트 (A~I 모두 출발 1번, 6곳은 도착 1번 더) | 15 | 45 |

지나는 연결로는 여의2교 54, 서울교 36, 여의교 9문제. 여의교는 노량진 쪽 경로에서만 가장 빠른 길이라 적고,
여의하류 램프는 가장 빠른 길이 되는 쌍이 없어 0이다.

## 알아 둘 점

- **노량진수산시장 경로가 길다** (섬에서 6~8 km). 도로망 자료에서 여의교 남단 → 노들로 동쪽(노량진 방향)
  좌회전이 금지돼 있고, 랜드마크가 노들로 동쪽 방향 차로에 붙어 있어서, 여의교로 건너도 영등포로를 돌아
  노들로로 다시 들어온다. 실제 회전 제한이 맞는지는 아직 확인하지 않았다.
- 시간은 신호·정체 없이 제한속도로 달릴 때 값이다. 시간대별 실제 소요 시간은 아직 재지 않았다.
- `YeouidoEnv` 는 아직 이 경로를 쓰지 않는다 (지금은 `findRoute` 로 가장 빠른 경로 하나만 탐).
