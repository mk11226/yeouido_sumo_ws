# 성균관대학교 서울 → 수원 SUMO 시뮬레이션

서울 인문사회과학캠퍼스에서 수원 자연과학캠퍼스까지 약 38km의 도로 환경이다. 건물·신호등, 한강 수면, 지하철역·주요 교량·톨게이트 이름표, 서울 시내 정체와 의왕톨게이트의 4→6→4차로 이용·합류를 포함한다. 차량 추종·안전 제어는 SUMO 기본 모델을 사용하고, 톨게이트 전 정체에서는 규칙에 따라 차로 변경을 요청한다. 이 경로의 강화학습 모델은 아직 없다.

## 실행

현재 PC는 시스템에 SUMO가 설치되어 있어 다음 명령으로 실행한다.

```bash
cd /home/mk/bmil_ws
python3 run_skku_commute.py --gui --background --viewport-m 400 --keep-open
```

빨간 ego 차량을 자동 추적한다. 지명을 넓게 보려면 `--viewport-m 800`을 사용한다. 배경 교통을 시뮬레이션 300초 동안 먼저 형성한 뒤 ego가 출발한다. 기본 검사 제한은 혼잡을 고려해 시뮬레이션 3시간이며 ego 도착 시 종료한다. 결과는 `results/skku_commute/`에 저장한다.

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

## 파일 구성

| 경로 | 내용 |
|---|---|
| `run_skku_commute.py` | 주행·GUI 실행, 차량 추적, 결과 기록 |
| `download_skku_*.py` | 도로·건물·지명 원본 수집 |
| `build_skku_*.py` | 도로·건물·신호·지명·차로 보정과 교통 수요 생성 |
| `prepare_seoul_signals.py`, `prepare_gyeonggi_signals.py` | 서울·경기 신호 원본 정규화 |
| `audit_skku_toll.py`, `audit_skku_queue.py`, `validate_skku_landmarks.py` | 톨게이트 이용·합류 및 지명 검사 |
| `env/skku_commute/` | 원본 지도, 현재 SUMO 환경, 보정 기록 |
| `results/skku_commute/` | 주행 결과, 검증 자료, 단계별 환경·코드 스냅샷 |
| `tests/test_skku_buildings.py` | 건물 형상 처리 검사 |

## 구축 기록

톨게이트 설정과 이전 변경 내역은 **[10번 통합 문서](project_materials/10_SKKU_톨게이트_차로이용_합류.md)**를 기준으로 확인한다. 6~9번은 단계별 기록이므로 과거 설정·검증 수치가 포함되어 있다.

- [6. 서울 → 수원 도로 환경](project_materials/6_SKKU_서울_수원_도로환경.md)
- [7. 건물·신호등 보완](project_materials/7_SKKU_건물_신호등_보완.md)
- [8. 한강·지명·정체](project_materials/8_SKKU_한강_지명_정체.md)
- [9. 본선 차로·교량명 보완](project_materials/9_SKKU_본선_차로_교량명_보완.md)
- [10. 톨게이트 차로 이용·합류 및 전체 변경 내역](project_materials/10_SKKU_톨게이트_차로이용_합류.md)

최신 배경 교통 확장은 [11. 양방향·교차로 주변 교통](project_materials/11_SKKU_양방향_교차로_교통.md)을 참고한다.

2026-09-28에 혜화로터리 전용 환경·문서·학습 결과와 이전 BC/PPO 실습 코드·데이터를 삭제했다. 서울→수원 원본 지도와 주행·검증 결과는 유지했다. 과거 결과 폴더에 보존한 코드와 해시는 당시 실험의 기록이다.

도로 데이터: © OpenStreetMap contributors, [ODbL 및 저작권 안내](https://www.openstreetmap.org/copyright). 신호·건물 등 개별 자료의 출처는 구축 문서와 원본 manifest를 따른다.

역 주변 직진 신호 연장은 [12번 문서](project_materials/12_SKKU_역주변_직진신호_연장.md)를 참고한다.

최신 교통·신호·1차로 정체 대응은 [13번 문서](project_materials/13_SKKU_정체수준_주변교통_차로변경.md)를 참고한다.
