# 여의도 ↔ 영등포 넘나드는 경로

`yeouido/` 가 만든 도로망(`env/yeouido/yeouido.net.xml`)과 랜드마크(`landmarks.json`)를 그대로 쓰고,
섬(여의도)·본토(영등포) 핀포인트 사이 AV 경로만 따로 만든다. 도로망·교통량은 건드리지 않는다.
만든 경로는 sumo-gui에서 직접 주행해 볼 수 있다 (아래 [경로 주행 보기](#경로-주행-보기)).

| 파일 | 내용 |
|---|---|
| `yeouido_cross/build_routes.py` | 핀포인트 13곳 모든 쌍의 최단거리 경로 → `output/routes.*` |
| `yeouido_cross/build_eval_set.py` | 평가 시험지 150문제 → `output/eval_set.json` |
| `drive_route.py` (저장소 루트) | 경로 하나를 sumo-gui에서 주행 (명령줄) |
| `drive_panel.py` (저장소 루트) | sumo-gui + 조절 창: 시간대 버튼, 출발·도착 입력해 주행 |

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

**핀포인트 13곳 사이 모든 쌍(13 × 12 = 156)에 최단거리 경로 하나씩.** 거르는 쌍 없음.

| 쪽 | 핀포인트 |
|---|---|
| 여의도 섬 (7) | 1 국회의사당, 2 IFC 서울, 3 63빌딩, 4 여의도공원, 게이트 A 서강·B 마포·C 원효대교 |
| 영등포 본토 (6) | 5 영등포역, 6 당산역, 7 노량진수산시장, 게이트 D 양화대교·E 오목교·F 거리공원 |

- 샛강 다리 위 게이트(ㄱ 여의교·ㄴ 서울교·ㄷ 여의2교)는 섬도 본토도 아니라서 넣지 않았다.
- 출발·도착 위치는 `YeouidoEnv._insert_ego` 와 같다. 랜드마크는 그 지점(출발은 edge 끝에서 20 m 이상 남김),
  게이트는 출발 = 들어오는 도로 시작, 도착 = 나가는 도로 끝.
- 최단거리 = 도로 길이 기준 (`getShortestPath`, 승용차 통행·회전 제한 반영).

| 종류 | id | 쌍 |
|---|---|---|
| 섬↔본토 | `x000`~ | 7 × 6 × 두 방향 = 84 |
| 여의도 안 | `i000`~ | 7 × 6 = 42 |
| 영등포 안 | `m000`~ | 6 × 5 = 30 |
| 합계 | | 156 |

`bridge` = 경로가 건너는 샛강 연결로 (안 건너면 null). 같은 쪽 안 경로가 샛강을 건너는 경우는 없다.
`detour` = 도로 길이 / 직선거리. 가까운데 크게 돌아가는 경로(예: IFC 서울 → 여의도공원 직선 313 m, 도로 2.2 km,
7.0배)를 평가·학습에서 거르고 싶으면 이 값으로 고른다.

결과 (`output/`):

| 파일 | 내용 |
|---|---|
| `routes.json` | `{id, origin, destination, direction, bridge, length_m, free_time_s, straight_m, detour, edges}` |
| `routes.rou.xml` | SUMO `<route>` 정의 (id = `routes.json` 의 id) |
| `routes.poly.xml` | sumo-gui 겹쳐 보기용 선. 연결로별 색 (빨강 여의2교, 파랑 서울교, 초록 여의교, 보라 램프, 주황 섬 안, 청록 영등포 안) |

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
| 게이트 (A~F·ㄱ~ㄷ 모두 출발 1번, 6곳은 도착 1번 더) | 15 | 45 |

지나는 연결로는 여의2교 48, 서울교 27, 여의교 9문제 (2026-11-16 최신 도로망 기준). 여의교는 노량진 쪽 경로에서만
가장 빠른 길이라 적고, 여의하류 램프는 가장 빠른 길이 되는 쌍이 없어 0이다.

선생님 정책 기준선 (2026-11-16, 교차로 양보 수정 후): 충돌 2.0% (3/150), 완주 98.0%, 시간초과 0%,
평균 속도 35.7 km/h, 완주 소요 6.8분. 수정 전은 충돌 3.3% (5/150). 남은 3건(#76, #91, #146)은 모두 교차로 안 충돌.

## 알아 둘 점

- **노량진수산시장 경로가 길다** (섬에서 6~8 km). 도로망 자료에서 여의교 남단 → 노들로 동쪽(노량진 방향)
  좌회전이 금지돼 있고, 랜드마크가 노들로 동쪽 방향 차로에 붙어 있어서, 여의교로 건너도 영등포로를 돌아
  노들로로 다시 들어온다. 실제 회전 제한이 맞는지는 아직 확인하지 않았다.
- 시간은 신호·정체 없이 제한속도로 달릴 때 값이다. 시간대별 실제 소요 시간은 아직 재지 않았다.
- `YeouidoEnv` 는 아직 이 경로를 쓰지 않는다 (학습·평가는 `findRoute` 로 가장 빠른 경로 하나만 탐).
  `drive_route.py` / `drive_panel.py` 만 출발 직후 `setRoute` 로 `routes.json` 경로로 바꿔 탄다.
- **한 바퀴 돌아가는 경로**: 도로망의 회전 제한 때문에 블록을 돌아오는 경로가 있다.
  - 2 → 7 (IFC 서울 → 노량진수산시장): 서울교 남단에서 노량진 쪽으로 꺾는 길이 실제로도 없어서 영등포 쪽을 돈다 (OSM 확인, 정상).
  - A → 7: 여의2교 서쪽 끝(국회대로×노들로)에서 노들로 남쪽 좌회전이 노드링크 회전 제한으로 막혀 있어 약 1.4 km를 돈다.
    실제 좌회전 가능 여부는 확인 전.
  - 서울교 남단에서 경인로(영등포 쪽) → 노들로 노량진 방향 연결로가 노드링크 회전 제한으로 빠져 있다 (OSM에는 있음).
    도로망 수정 요청으로 정리해 두었다.

## 경로 주행 보기

### 조절 창 (`drive_panel.py`)

sumo-gui 하나와 작은 조절 창을 띄운다. 조절 창에서 시간대를 고르고 출발·도착을 넣으면 그 경로를 주행한다.

```bash
python drive_panel.py                 # 주간 교통으로 시작
python drive_panel.py --level rush    # 출근 교통으로 시작
python drive_panel.py --start 1 B     # 켜자마자 1 → B 주행
python drive_panel.py --delay 50      # 재생을 느리게 (sumo-gui 지연 ms, 기본 20)
```

| 조절 창 | 동작 |
|---|---|
| `[심야 03시] [주간 12시] [출근 08시]` | 주행 중이 아니면 즉시 그 시간대의 워밍업 교통(시뮬 20분 시점)으로 바뀐다. 주행 중이면 그때부터 새로 들어오는 일반 차량만 그 시간대 일정표를 따른다 (`yeouido/live.py` 와 같은 방식) |
| `⏸ 일시정지` / `▶ 계속` | 시뮬레이션 멈춤·진행 |
| `출발 [ ] → 도착 [ ] [주행 시작]` | `1`~`7`, `A`~`F` 또는 이름. Enter로도 시작. 이전 경로 색칠은 지워진다 |
| 상태 표시 | 시간대, 시뮬 시각, 도로 위 차량(정지 대수), 평균 속도, 주행 상태(남은 거리 → 도착/충돌·걸린 시간) |

- 주행이 끝나도 교통은 계속 흐르고, 바로 다음 경로를 넣을 수 있다.
- **둘 중 아무 창이나 닫으면 둘 다 닫힌다** (sumo-gui 닫기 버튼 또는 ⌘Q, 조절 창 닫기 버튼).
- 조절 창은 sumo-gui에 가려지지 않게 항상 위에 뜬다.
- 동작 방식: SUMO는 도로망 + 차량 종류만 읽고(`output/panel.sumocfg`) 시작하고, 일반 차량은 조절 창이 선택된 시간대의
  `env/yeouido/<level>.rou.xml` 일정표에서 출발 시각에 맞춰 넣는다. 그래서 시간대를 바꿔도 sumo-gui가 다시 뜨지 않는다.
  주행은 `YeouidoEnv` + 선생님 정책(`SumoExpert`)이 한다.

### 명령줄 (`drive_route.py`)

경로 하나만 바로 주행할 때.

```bash
python drive_route.py 1 B                    # 랜드마크 1(국회의사당) → 게이트 B(마포대교), 주간
python drive_route.py 5 3 --level rush       # 출근 교통
python drive_route.py 영등포역 "게이트 A"     # 이름으로도 지정
python drive_route.py 1 B --overview         # ego를 따라가지 않고 도로망 전체 보기
python drive_route.py --list                 # 핀포인트 목록
python drive_route.py 1 B --auto-close       # 끝나면 바로 창 닫기 (기본: 끝 화면 유지, 창을 닫으면 종료)
```

터미널에 100스텝마다 남은 거리, 끝나면 결과(도착/충돌, 마지막 위치, 충돌 상대)를 출력한다.

### 화면 표시 (두 도구 공통)

- **핀포인트 13곳**: 랜드마크 1~7은 노란 동그라미, 게이트 A~F는 빨간 둥근 네모 (샛강 다리 ㄱ~ㄷ은 표시하지 않음).
  sumo-gui 글꼴은 한글이 안 나와서 핀·이름표를 그림(`output/pins/`)으로 만들어 POI에 붙인다.
- **핀 크기**: 지도에 고정 (전체 지도에서 동그라미 지름 약 150 m), 단 확대했을 때 화면 폭의 2.5%(`MAX_SCREEN`)보다
  커지지 않게 줄인다 → 멀리서는 잘 보이고 가까이서는 도로를 가리지 않는다.
- **경로 색칠**: 경로가 지나는 차로만 아스팔트 색을 파랗게 바꾼다 (차선·중앙선은 그대로 보임). 도로마다 다음 도로로
  이어지는 연결이 있는 차로만 칠한다 (예: 좌회전이 1·2차로만 되면 그 둘만). 교차로 안 연결 차로도 칠한다.
  sumo-gui의 "선택" 기능 + 차로 색칠 방식 `by selection` 으로 구현.
- 같은 시간대 일반 차량이 함께 달린다 (워밍업 상태를 불러오므로 출발 순간부터 그 시간대 교통).

실행할 때마다 만들어지는 파일 (커밋하지 않음, `.gitignore`): `output/pins/`, `output/pins.add.xml`,
`output/route.view.xml`, `output/panel.sumocfg`.
