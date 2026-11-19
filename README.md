# 여의도 SUMO 도로 환경과 자율주행 학습

여의도·영등포구 북부의 실제 도로망과 시간대별 교통량으로 SUMO 환경을 구성하고,
SUMO 선생님 정책·BC·PPO로 차량의 주행 의사결정을 실험한다.
도로는 국가표준노드링크, 교통량은 서울시 TOPIS, 주변 건물·공원은 OpenStreetMap 자료를 사용한다.

## 현재 구성

- **교통량**: 심야 03시·주간 12시·출근 08시, 공통 배율 `DEMAND_SCALE = 1.0`.
- **경로**: 랜드마크 7곳과 외곽 게이트 6곳 사이의 최단거리 경로 156개.
- **주행 화면**: 출발·도착과 시간대를 고르는 조절 창, 경로 차로 강조, 승용차 형태의 차량 표시.
- **신호**: 228개 제어기 중 51곳에 2026-11-16 T DATA 관측 주기 적용 (심야 4·주간 51·출근 44곳). 적용 범위와 한계는 [도로 환경 문서](yeouido/README.md#t-data-실측-신호) 참고.
- **여의2교 남단**: 국회의사당 방향 좌회전 2차로와 맞은편 직진 종료 후 보호 좌회전 순서 반영.
- **평가**: 모든 정책에 같은 조건을 주는 고정 시험지 150문제와 결과 비교 도구.

## 설치

아래는 Linux에서 새 가상환경을 만드는 예시다. 기존 환경이 있으면 활성화 후 패키지만 설치한다.
Windows·macOS 설치는 [실습 문서](#실습-문서)를 참고한다.

```bash
cd ~/bmil_ws
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r yeouido/requirements-drive.txt

# 경로 보기·선생님 주행·CPU 학습용 PyTorch
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu

# pip로 설치한 SUMO 사용: 새 터미널에서도 설정한다
export SUMO_HOME="$(python -c 'import sumo; print(sumo.SUMO_HOME)')"
```

GUI 실행에는 운영체제의 Tk와 그래픽 실행 환경이 필요하다.
한글 핀 이름에는 NanumGothic 또는 Noto CJK 글꼴을 사용한다.
기존 SUMO 설치를 사용할 경우 `SUMO_HOME`을 해당 설치 폴더로 설정한다.

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
