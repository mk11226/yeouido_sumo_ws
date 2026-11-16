# 여의도·영등포 경로와 주행 도구

13개 핀포인트 사이의 최단거리 경로 156개를 만들고, 선생님 정책으로 선택한 경로를 주행한다.
교통량 배율은 `1.0`이며, 차량은 승용차 차체와 창문이 보이는 형태로 표시한다.

## 실행 준비

항상 저장소 맨 위에서 실행한다. `yeouido_cross/` 폴더 안에서 파일을 직접 실행하지 않는다.

```bash
cd ~/bmil_ws
source .venv/bin/activate
python -m yeouido_cross.drive_panel
```

새 환경을 만드는 경우 다음을 설치한다. SUMO_HOME·Tk 등 준비 사항은 [루트 README](../README.md#설치)를 참고한다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r yeouido/requirements-drive.txt
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```

마지막 명령은 CPU용 PyTorch다. 경로 보기와 선생님 주행에는 GPU가 필요 없다.
GUI에는 운영체제의 Tk와 SUMO GUI 실행 환경이 필요하다. 한글 핀 이름은 NanumGothic 또는 Noto CJK 글꼴을 사용한다.
처음 실행하거나 도로·차량·신호를 바꾼 뒤에는 600/1200/1800/2400초 워밍업 상태를 다시 만든다.
현재 실측 100% 교통량에서는 수 분 걸릴 수 있으며, 터미널의 워밍업 생성 메시지가 끝나면 창이 열린다.

## 조절 창에서 반복 주행

```bash
python -m yeouido_cross.drive_panel --level rush
python -m yeouido_cross.drive_panel --level night --start 1 B
python -m yeouido_cross.drive_panel --level day --delay 20
```

- 기본 시간대는 주간(`day`)이다. 심야는 `night`, 출근은 `rush`다.
- 출발·도착에 아래 숫자·영어 또는 지점 이름을 넣고 `주행 시작`을 누른다.
- `--start 1 B`는 국회의사당에서 마포대교 경계로 바로 주행한다.
- 주행이 끝나면 같은 창에서 다른 경로를 입력할 수 있다.
- 주행하지 않을 때 시간대 버튼을 누르면 해당 시간대의 1200초 워밍업 교통으로 바뀐다.
- 주행 중 시간대 버튼을 누르면 새로 투입하는 일반 차량의 일정표가 바뀐다.
  이미 달리는 차량은 남아 있으므로 교통이 서서히 바뀐다.
- 일시정지 버튼으로 멈추거나 재개한다. 어느 창을 닫아도 두 창이 함께 종료된다.
- `--delay`는 SUMO GUI 지연(ms)이며 작을수록 빠르다. 화면 위 Delay로도 조절할 수 있다.
- 노란 원은 랜드마크, 빨간 네모는 게이트, 파란 차로는 선택한 주행 경로다.
  회전 직전에는 해당 회전으로 이어지는 차로와 교차로 내부 연결 차로만 강조한다.

## 핀포인트 13곳

| 입력 | 위치 |
|---|---|
| 1 | 국회의사당 |
| 2 | IFC 서울 |
| 3 | 63빌딩 |
| 4 | 여의도공원 |
| 5 | 영등포역 |
| 6 | 당산역 |
| 7 | 노량진수산시장 |
| A | 서강대교 경계 |
| B | 마포대교 경계 |
| C | 원효대교 경계 |
| D | 양화대교 경계 |
| E | 오목교 경계 |
| F | 거리공원 경계 |

이전 프로젝트와 게이트 이름이 달라졌다. 이전 G/H/I가 현재 D/E/F이며,
이전 D/E/F(여의교/서울교/여의2교)는 현재 ㄱ/ㄴ/ㄷ다.
주행 도구의 13개 지점에는 샛강 다리 ㄱ/ㄴ/ㄷ를 넣지 않는다.
학습·평가 환경은 `places="landmarks+gates"`로 설정해 이 다리 지점도 인식한다.

## 경로 하나만 실행

```bash
python -m yeouido_cross.drive_route --list
python -m yeouido_cross.drive_route 1 B --level night
python -m yeouido_cross.drive_route 5 3 --level rush
python -m yeouido_cross.drive_route 영등포역 "게이트 A" --level day
python -m yeouido_cross.drive_route 1 B --overview --delay 20
python -m yeouido_cross.drive_route 1 B --auto-close
```

`--overview`는 차량 추적 대신 전체 지도를 보여 준다. 기본은 ego를 따라간다.
기본적으로 주행이 끝난 화면을 유지하며, `--auto-close`를 주면 종료한다.
`--seed`로 난수를 지정하고, `--screenshot /absolute/path/image.png`로 출발 후 화면을 저장할 수 있다.
모든 옵션은 `python -m yeouido_cross.drive_route --help`에서 확인한다.

주행 주체는 학습한 모델이 아니라 `SumoExpert` 선생님 정책이다.
모델 파일을 불러오거나 BC/PPO 학습을 자동 시작하지 않는다.

## 156개 경로와 결과 파일

13개 출발지 각각에 다른 12개 도착지를 연결해 156개를 만든다. A→B와 B→A는 다른 경로다.
각 쌍에는 도로 길이 기준 최단거리 경로 하나를 저장한다.
섬↔본토 84개(`x…`), 여의도 안 42개(`i…`), 영등포 안 30개(`m…`)다.
이전 버전과 달리 우회가 크다는 이유로 지점 쌍을 제외하지 않는다.

| 파일 | 내용 |
|---|---|
| `yeouido_cross/output/routes.json` | 출발·도착, 도로 목록, 거리, 연결 다리 등 |
| `yeouido_cross/output/routes.rou.xml` | SUMO 경로 정의. 차량 투입 일정 자체는 아님 |
| `yeouido_cross/output/routes.poly.xml` | 전체 경로를 선으로 겹쳐 보여 주는 파일 |

`length_m`은 거리(m), `free_time_s`는 신호·정체를 제외한 제한속도 기준 시간,
`straight_m`은 직선거리, `detour`는 도로 거리/직선거리다.
`bridge`는 샛강 연결로 이름이며 안 건너면 `null`, 여러 개를 건너면 `+`로 이어진다.
출발·도착이 같은 쪽이어도 최단경로가 샛강을 건널 수 있다.

도로를 다시 고친 뒤에는 다음 명령으로 현재 도로망에 맞춰 결과 파일을 재생성한다.
이 명령은 가져온 원본 결과 데이터를 덮어쓴다. 파이썬 소스는 바뀌지 않는다.

```bash
python -m yeouido_cross.build_routes
python -m yeouido_cross.build_routes --gui
```

두 번째 명령은 생성 후 전체 경로를 겹쳐 보여 준다. 차량이 실제로 운전하는 화면은 `drive_panel`/`drive_route`다.
`yeouido.live`/`yeouido.view`는 기존 교통 관찰 도구이므로 출발·도착 경로 입력 창이 없다.

## 고정 시험지 평가

기존 `yeouido_cross/build_eval_set.py`와 `yeouido_cross/output/eval_set.json`은
이번 버전에서 `eval_set/build_eval_set.py`, `eval_set/eval_set.json`으로 이동했다.
이는 경로 파일 156개와 다른 목적이다. 50개 출발·도착 쌍에 시간대 3개를 적용한 고정 평가 조건 150개다.

```bash
# 선생님 정책으로 일부 조건만 빠르게 확인
python test.py --expert --eval-set eval_set/eval_set.json --nogui --limit 6

# 선생님 정책 전체 평가
python test.py --expert --eval-set eval_set/eval_set.json --nogui --workers 4

# 학습한 모델 평가: 실제 모델 파일 경로로 바꾼다
python test.py results/<run>/model.pt --eval-set eval_set/eval_set.json --nogui --workers 4

# 전체 결과 비교: 실제 생성된 결과 파일을 지정한다
python -m utils.eval_report results/sumo_expert/eval_set_results.json results/<run>/eval_set_results.json
```

일부만 평가하면 기본 결과 이름은 `eval_set_results_limit6.json`, 전체는 `eval_set_results.json`이다.
`--out /path/result.json`으로 결과 저장 위치를 지정할 수 있다.
도로·게이트 위치가 달라져 시험지를 새로 만들려면 `python -m eval_set.build_eval_set --check`를 쓴다.
비교 실험 중간에 시험지를 재생성하면 같은 시험지로 비교한 결과가 아니므로 구분해 관리한다.

`drive_panel`/`drive_route`는 저장된 최단거리 경로를 ego에 직접 적용한다.
`train.py`/`test.py`의 `YeouidoEnv`는 출발·도착 조건에서 SUMO가 찾은 가장 빠른 경로를 사용한다.
따라서 파일을 가져온 것만으로 학습 차량이 156개 고정 경로를 순회하도록 바뀌지는 않는다.
새 평가 도구의 난수 시드는 같은 환경 조건에서 반복 재현하기 위한 것이며,
도로망·신호·교통량을 바꾼 이전 실험과 같은 차량 궤적이나 점수를 보장하지 않는다.

## 가져온 버전

경로 데이터와 주행 도구는 `seonu`의 `5d447b7`(2026-11-17)을 기준으로 가져왔다.
현재 프로젝트의 신호 파일 연결, Linux 한글 글꼴 선택과 GUI 자동 종료를 보완했다.
