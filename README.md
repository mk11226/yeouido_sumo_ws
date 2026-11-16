# 성균관대학교 서울 → 수원 SUMO 시뮬레이션

서울 인문사회과학캠퍼스에서 수원 자연과학캠퍼스까지 약 38km의 도로 환경이다. 건물·신호등, 한강 수면, 지하철역·주요 교량·톨게이트 이름표, 서울 시내 정체와 의왕톨게이트의 4→6→4차로 이용·합류를 포함한다. 차량 추종·안전 제어는 SUMO 기본 모델을 사용하고, 톨게이트 전 정체에서는 규칙에 따라 차로 변경을 요청한다. 이 경로의 강화학습 모델은 아직 없다.

## 실행

현재 PC는 시스템에 SUMO가 설치되어 있어 다음 명령으로 실행한다. 모든 명령은 저장소 루트에서 입력한다. 다른 위치에 저장소를 받았다면 `cd` 경로를 해당 위치로 바꾼다.

```bash
cd /home/mk/bmil_ws
python3 run_skku_commute.py --gui --background --viewport-m 400 --keep-open
```

### 실행이 안 될 때: 가상환경으로 실행

위 명령에서 `SUMO is unavailable` 또는 Python 모듈을 찾을 수 없다는 오류가 발생하면, 저장소 루트에서 가상환경을 만들고 필요한 패키지를 설치한 뒤 실행한다.

```bash
python3 -m venv .venv-skku
.venv-skku/bin/python -m pip install --upgrade pip
.venv-skku/bin/python -m pip install -r requirements-skku.txt

.venv-skku/bin/python run_skku_commute.py \
  --gui --background --viewport-m 400 --keep-open
```

설치를 마친 뒤에는 가상환경을 다시 만들 필요 없이 아래 명령으로 실행하면 된다. 가상환경의 Python을 직접 지정하므로 별도의 `activate`도 필요하지 않다.

```bash
.venv-skku/bin/python run_skku_commute.py \
  --gui --background --viewport-m 400 --keep-open
```

### 실행 동작과 옵션

빨간 ego 차량을 자동 추적한다. 지명을 넓게 보려면 `--viewport-m 800`을 사용한다. 배경 교통을 시뮬레이션 300초 동안 먼저 형성한 뒤 ego가 출발한다. 기본 검사 제한은 혼잡을 고려해 시뮬레이션 3시간이며 ego 도착 시 종료한다. 결과는 `results/skku_commute/`에 저장한다.

실행기는 의왕톨게이트 4→6차로 진입·6→4차로 합류의 내부 연결 차로 12개에서 모든 차량의 좌우 차로 변경을 제한한다. 갈라지거나 모이는 연결 구간 안에서의 측면 충돌을 방지하기 위한 것으로, 진입 전 본선과 요금소 본체에서는 차로 변경이 가능하다. 충돌 검사·3초 차로 변경·요금소 정차는 유지하며, 적용 차로는 실행 결과의 `protected_toll_connector_lanes`에 기록한다. 변경 사항은 실행기를 다시 시작하면 적용된다.

톨게이트 접근 시 ego는 본선 1차선(맨 왼쪽)을 유지하다 전방 120m 내 느린 차량을 감지하면 2→3→4차선으로 이동을 요청한다. 정체가 없으면 마지막 500m에서 요금소 진입을 위한 정렬을 시작한다. 차로 변경 의사를 먼저 SUMO에 전달하되 안전 간격 검사는 유지한다. 간격이 부족하면 같은 도로의 인접 차로 뒤 차량 한 대에 최대 8초간 간격 확보를 요청한다(협조 운전자 가정, 추가 감속 한도 2m/s²). 변경 완료·구간 이탈·기한 만료 시 양보 제어를 해제한다. ego의 현재 저속을 반복 고정하는 감속 명령은 사용하지 않는다. 실제 이동은 안전 조건에 따라 지연될 수 있다. 톨게이트 직전 약 755m와 요금소 약 541m 및 그 연결부의 제한속도는 30km/h이며, 요금소 6초 정차는 유지한다.

협조 합류 검증: 통과 경로 수요 2,430대/시간에서 [seed 42](results/skku_commute/toll_cooperative_bounded_seed42/summary.json)와 [seed 43](results/skku_commute/toll_cooperative_bounded_seed43/summary.json) 모두 ego 충돌·텔레포트 없이 약 38.34km를 완주했다. 정체 감지부터 본선 4차선 도달까지 각각 38초와 107.5초였다. 이는 합류 불능 개선 검증이며, 4차선 이동 후 톨게이트 대기시간 제거를 의미하지 않는다. 두 실행에서 다른 도심 교차로의 배경 차량 충돌은 각각 2건·4건 남아 있다.

```bash
# GUI 없이 배경 교통과 주행
python3 run_skku_commute.py --background --seed 42

# 실행 옵션 확인
python3 run_skku_commute.py --help
```

## 별도 Python 환경 설정

다른 PC에서 구축 도구까지 사용할 때 필요한 직접 의존성은 [requirements-skku.txt](requirements-skku.txt)에 정리했다. 아래 설치 명령은 이번 정리 작업에서 새로 실행하지 않았다. GUI에는 화면 연결과 해당 운영체제의 그래픽 라이브러리가 필요하고, 한글 이름표 생성에는 Noto Sans CJK 폰트가 필요하다.

```bash
python3 -m venv .venv-skku
.venv-skku/bin/python -m pip install -r requirements-skku.txt
source .venv-skku/bin/activate
python run_skku_commute.py --gui --background --viewport-m 400 --keep-open
```

## 정리된 도로 구축 도구 실행

일반 주행은 위의 `run_skku_commute.py` 명령을 사용한다. `Road_Environment/`로 옮긴 보조 도구는 저장소 루트에서 모듈 이름으로 실행한다.

```bash
# 구축 옵션 확인
python3 -m Road_Environment.build.build_skku_commute --help

# 보관한 지도 자료로 도로 환경 재생성 (현재 시나리오 파일 갱신)
python3 -m Road_Environment.build.build_skku_commute --skip-convert

# 주행 결과 검사 옵션 확인
python3 -m Road_Environment.validation.audit_skku_queue --help

# 도로 환경 테스트
python3 -m unittest discover -s tests -p 'test_skku*.py' -v
```

각 도구의 위치와 문서 6~14번은 [Road_Environment 안내](Road_Environment/README.md)에 모았다.

## 파일 구성

| 경로 | 내용 |
|---|---|
| [Road_Environment/](Road_Environment/README.md) | 서울→수원 구축 문서 6~14번과 도구 모음 |
| `Road_Environment/control/` | 정체 감지 및 차로 변경 제어 |
| `run_skku_commute.py` | 주행·GUI 실행, 차량 추적, 결과 기록 |
| `Road_Environment/download/` | 도로·건물·지명 원본 수집 |
| `Road_Environment/build/` | 도로·건물·신호·지명·차로 보정과 교통 수요 생성 |
| `Road_Environment/prepare/` | 서울·경기 신호 원본 정규화 |
| `Road_Environment/validation/` | 톨게이트 이용·합류 및 지명 검사 |
| `env/skku_commute/` | 원본 지도, 현재 SUMO 환경, 보정 기록 |
| `results/skku_commute/` | 주행 결과, 검증 자료, 단계별 환경·코드 스냅샷 |
| `tests/test_skku_buildings.py` | 건물 형상 처리 검사 |
| [yeouido_cross/](yeouido_cross/README.md) | 여의도·영등포 핀포인트 13곳 경로(156개)와 주행 도구 |
| [eval_set/](eval_set/README.md) | 여의도 평가 시험지 150문제 |
| `yeouido_cross/drive_panel.py` | 여의도 sumo-gui + 조절 창 (시간대 버튼, 출발·도착 입력해 경로 주행): `python -m yeouido_cross.drive_panel` |
| `yeouido_cross/drive_route.py` | 여의도 경로 하나를 sumo-gui에서 주행 (명령줄): `python -m yeouido_cross.drive_route 1 B` |

## 여의도 평가 시험지

모든 모델을 같은 150문제로 비교하는 고정 시험지. 사용법은 **[eval_set/](eval_set/README.md)**.

```bash
python test.py results/<run>/model.pt --eval-set eval_set/eval_set.json --nogui --workers 4
```

## 구축 기록

톨게이트 설정과 이전 변경 내역은 **[10번 통합 문서](Road_Environment/10_SKKU_톨게이트_차로이용_합류.md)**를 기준으로 확인한다. 6~9번은 단계별 기록이므로 과거 설정·검증 수치가 포함되어 있다.

- [6. 서울 → 수원 도로 환경](Road_Environment/6_SKKU_서울_수원_도로환경.md)
- [7. 건물·신호등 보완](Road_Environment/7_SKKU_건물_신호등_보완.md)
- [8. 한강·지명·정체](Road_Environment/8_SKKU_한강_지명_정체.md)
- [9. 본선 차로·교량명 보완](Road_Environment/9_SKKU_본선_차로_교량명_보완.md)
- [10. 톨게이트 차로 이용·합류 및 전체 변경 내역](Road_Environment/10_SKKU_톨게이트_차로이용_합류.md)

최신 배경 교통 확장은 [11. 양방향·교차로 주변 교통](Road_Environment/11_SKKU_양방향_교차로_교통.md)을 참고한다.

2026-09-28 정리 과정에서 함께 삭제했던 공용 BC/PPO 실습 코드·데이터·기존 도로 환경은 이전 커밋 `fdd19a3`에서 복구했다. 혜화로터리 전용 자료는 삭제 상태이며, 서울→수원 원본 지도와 주행·검증 결과는 유지했다. 과거 결과 폴더에 보존한 코드와 해시는 당시 실험의 기록이다.

도로 데이터: © OpenStreetMap contributors, [ODbL 및 저작권 안내](https://www.openstreetmap.org/copyright). 신호·건물 등 개별 자료의 출처는 구축 문서와 원본 manifest를 따른다.

역 주변 직진 신호 연장은 [12번 문서](Road_Environment/12_SKKU_역주변_직진신호_연장.md)를 참고한다.

교통·신호·1차로 정체 대응의 이전 설정은 [13번 문서](Road_Environment/13_SKKU_정체수준_주변교통_차로변경.md), 최신 협조 합류 수정과 검증은 [14번 문서](Road_Environment/14_SKKU_톨게이트_협조합류_0930.md)를 참고한다.


## 기존 공용 학습·실습 자료

`algorithms/`의 BC·PPO, `utils/`, `train.py`, `train_bc.py`, `collect_pomdp_data.py`, `test.py`, `view_road.py`와 기존 `env/sumo/` 환경을 복구했다. `data/dataset.npz`, `results/bc_0922_1358/`, `results/run_20260907_202220/`의 학습 데이터·모델·기록도 원본 그대로 복구했다. 이 모델은 기존 실습 환경의 결과이며 서울→수원 경로에서 학습한 모델이 아니다.

- [프로젝트 소개](project_materials/0_Project_Introduction.md)
- [Windows 설치](project_materials/1_SUMO_Windows_Setup.md), [macOS 설치](project_materials/1_SUMO_macOS_AppleSilicon_Setup.md)
- [기존 도로 환경·POMDP 구성](project_materials/2_Road_Environment_Construction.md)
- [BC 학습](project_materials/3_Model_Training_BC.md)
- [강화학습](project_materials/4_Model_Training_RL.md)

`requirements-skku.txt`는 서울→수원 시뮬레이션용 의존성이며 공용 학습 도구 전체의 의존성 목록은 아니다. 학습 도구의 설치·사용법은 위 실습 문서를 따른다.
