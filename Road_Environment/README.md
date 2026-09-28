# 서울 → 수원 도로 환경

실습 문서 6~13번과 서울→수원 도로 구축 도구를 모았다. 기존 공용 학습 문서 0~4번은 `project_materials/`, BC/PPO 코드는 `algorithms/`에 있다.

## 문서

- [6. SKKU 서울 수원 도로환경](6_SKKU_서울_수원_도로환경.md)
- [7. SKKU 건물 신호등 보완](7_SKKU_건물_신호등_보완.md)
- [8. SKKU 한강 지명 정체](8_SKKU_한강_지명_정체.md)
- [9. SKKU 본선 차로 교량명 보완](9_SKKU_본선_차로_교량명_보완.md)
- [10. SKKU 톨게이트 차로이용 합류](10_SKKU_톨게이트_차로이용_합류.md)
- [11. SKKU 양방향 교차로 교통](11_SKKU_양방향_교차로_교통.md)
- [12. SKKU 역주변 직진신호 연장](12_SKKU_역주변_직진신호_연장.md)
- [13. SKKU 정체수준 주변교통 차로변경](13_SKKU_정체수준_주변교통_차로변경.md)

## 도구 구성

| 폴더 | 용도 |
|---|---|
| `download/` | 도로·건물·지명 원본 수집 |
| `prepare/` | 서울·경기 신호 원본 정규화 |
| `build/` | 네트워크·건물·신호·교통량·차로·이름표 생성 |
| `control/` | 주행 중 정체 감지와 차로 변경 제어 |
| `validation/` | 톨게이트 이용·합류·정체·지명 검증 |
| `paths.py`, `runtime.py` | 저장소 경로와 SUMO 실행 환경 설정 |

## 실행

아래 명령은 저장소 루트(`/home/mk/bmil_ws`)에서 실행한다. 보조 도구는 `python3 -m`으로 호출한다.

```bash
cd /home/mk/bmil_ws
# 기존 주행 명령 유지
python3 run_skku_commute.py --gui --background --viewport-m 400 --keep-open

# 구축 옵션 확인
python3 -m Road_Environment.build.build_skku_commute --help

# 보관한 지도 자료로 환경 재생성: 현재 시나리오 파일을 갱신함
python3 -m Road_Environment.build.build_skku_commute --skip-convert

# 기존 주행 결과의 정체·차로 변경 관측 검사
python3 -m Road_Environment.validation.audit_skku_queue --help
```

의존성 설치는 [루트 README](../README.md)를 따른다. 원본·생성된 도로는 `env/skku_commute/`, 주행 결과는 `results/skku_commute/`에 유지한다.

이번 정리는 파일 위치와 import·문서 경로를 변경한 것이다. 기존 신호 시간·교통량·차로 변경 설정은 유지했다. 과거 결과 폴더의 코드 스냅샷·경로·해시는 당시 실험 기록이므로 그대로 보존했다.
