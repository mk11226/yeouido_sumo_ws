# 여의도·영등포 경로와 주행 보기

여의도·영등포 지도에 **출발·도착 후보 13곳(핀포인트)**을 정해 두고,
그 사이 모든 경로를 만든다. 원하는 경로를 sumo-gui에서 바로 주행해 볼 수 있다.

## 핀포인트 13곳

| 표기 | 여의도 섬 | 영등포 본토 |
|---|---|---|
| 랜드마크 (숫자) | 1 국회의사당 · 2 IFC 서울 · 3 63빌딩 · 4 여의도공원 | 5 영등포역 · 6 당산역 · 7 노량진수산시장 |
| 게이트 (영어) | A 서강대교 · B 마포대교 · C 원효대교 | D 양화대교 · E 오목교 · F 거리공원 |

- 게이트 = 지도 경계에서 차가 들어오고 나가는 다리·도로.
- 섬과 본토를 잇는 샛강 다리는 ㄱ 여의교 · ㄴ 서울교 · ㄷ 여의2교. 출발·도착 후보는 아니다.

## 주행해 보기

```bash
cd bmil_ws                          # 저장소 맨 위 폴더에서 실행
conda activate sumo-rl
python -m yeouido_cross.drive_panel
```

> `yeouido_cross/` 안에 들어가서 `python drive_panel.py` 로 실행하면 다른 폴더의 코드를 못 찾아 오류가 난다.
> 항상 저장소 맨 위 폴더에서 `python -m yeouido_cross.…` 로 실행한다.

sumo-gui와 조절 창이 같이 뜬다.

1. **시간대**: `[심야 03시] [주간 12시] [출근 08시]` 중 하나를 누른다.
2. **경로**: 출발·도착 칸에 `1`~`7` 또는 `A`~`F`를 넣고 **주행 시작**을 누른다 (Enter도 됨).
3. 주행이 끝나면 다른 경로를 바로 다시 넣으면 된다.
4. 끝낼 때는 **아무 창이나 닫으면** 둘 다 닫힌다.

화면에서
- 노란 동그라미 = 랜드마크, 빨간 네모 = 게이트.
- **파랗게 칠해진 차로** = 지금 달리는 경로. 회전할 때는 그 회전이 가능한 차로만 칠한다.
- 같은 시간대의 일반 차량이 함께 달린다.

자주 쓰는 옵션:

```bash
python -m yeouido_cross.drive_panel --level rush     # 출근 교통으로 시작
python -m yeouido_cross.drive_panel --start 1 B      # 켜자마자 1 → B 주행
```

조절 창 없이 경로 하나만 볼 때: `python -m yeouido_cross.drive_route 1 B` (`--level rush`, `--list` 등은 `--help` 참고).

## 경로 (`routes.json`)

13곳 사이 모든 쌍(13 × 12 = **156개**)에 **최단거리 경로 하나씩**.

랜드마크는 **건물 쪽 도로와 반대 방향 도로 둘 다** 출발·도착 지점 후보이고, 그중 짧은 쪽을 쓴다
(`landmark_sides.py`). 예전에는 건물 쪽 한 방향에만 붙어 있어서 반대 방향에서 오면 블록을 크게 돌았다.
- 노량진수산시장 앞 노들로 반대 방향은 지도 밖에서만 들어오는 도로라 **출발만** 가능 (도착은 건물 쪽만).
- 지도에서 확인: `python -m yeouido_cross.landmark_sides` (랜드마크마다 고른 도로 출력)

```bash
python -m yeouido_cross.build_routes          # 다시 만들기 → output/
python -m yeouido_cross.build_routes --gui    # 만든 뒤 sumo-gui로 전체 경로 겹쳐 보기
```

| 칸 | 뜻 |
|---|---|
| `id` | `x…` 섬↔본토 84개, `i…` 여의도 안 42개, `m…` 영등포 안 30개 |
| `origin`, `destination` | 출발, 도착 |
| `bridge` | 건너는 샛강 다리 (안 건너면 `null`) |
| `length_m` | 도로 길이 (m) |
| `detour` | 도로 길이 ÷ 직선거리. 크면 가까운데 돌아가는 경로 |
| `origin_side`, `dest_side` | 실제 출발·도착 지점: `건물 쪽` / `반대쪽` / `게이트` (`origin_pos`, `dest_pos` = 그 도로 위 위치 m) |
| `edges` | 지나는 도로 목록 |

## 알아 둘 점

- **도로망을 고치면** `python -m yeouido_cross.build_routes` 로 경로를 다시 만든다.
- **블록을 한 바퀴 도는 경로가 있다.** 도로망의 회전 제한 때문이다.
  - 2 → 7: 서울교에서 노량진 쪽으로 바로 꺾는 길이 실제로도 없음 (정상).
  - A → 7: 여의2교 서쪽 끝 좌회전이 막혀 있음 (실제 가능 여부 확인 중).
- 학습·평가 환경(`YeouidoEnv`)은 이 경로 파일이 아니라 SUMO가 찾은 가장 빠른 길로 달린다.
  `drive_panel` / `drive_route`만 이 파일의 경로를 탄다.
- 실행할 때 생기는 `output/pins/`, `pins.add.xml`, `route.view.xml`, `panel.sumocfg`는 커밋하지 않는다.
