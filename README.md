# 여의도 SUMO 도로 환경 및 자율주행 학습

영등포구 북부·여의도의 실제 도로망과 교통량을 바탕으로 SUMO 환경을 구성하고, BC·PPO로 자율주행 차량의 의사결정을 학습하는 프로젝트다. 국가표준노드링크 도로망, 서울시 TOPIS 교통량, OSM 주변 환경을 사용하며 심야·주간·출근 시간대 교통을 지원한다.

## 설치 및 실행

수업 기본 Python 환경에서 여의도 도로 구축 도구의 추가 의존성을 설치한다. 자세한 준비 사항과 원본 도로 자료 안내는 [여의도 문서](yeouido/README.md)를 참고한다.

```bash
pip install -r yeouido/requirements.txt

# SUMO GUI와 시간대 전환 창
python -m yeouido.live

# 한 시간대 고정 (night / day / rush)
python -m yeouido.view --level rush
```

도로망을 다시 생성하거나 시뮬레이션을 검증하려면 다음 명령을 사용한다. 재생성 명령은 현재 여의도 환경 파일을 갱신한다.

```bash
python -m yeouido.run_all --skip-validate
python -m yeouido.validate
```

## 학습 및 데이터 수집

`env/road_config.py`의 `ROAD_TYPE = "yeouido"` 설정으로 여의도 환경을 선택한다. 관측·행동·보상은 `env/mdp_config.py`, 학습 설정은 각 실행 파일에서 조정한다. 기본 학습 환경의 의존성과 사용법은 아래 실습 문서를 따른다.

```bash
# SUMO 운전자 모델로 모방학습 데이터 수집
python collect_pomdp_data.py --policy sumo-expert --episodes 500 --workers 4

# BC 학습
python train_bc.py

# PPO 학습
python train.py

# 저장한 모델 평가 (경로를 실제 모델 파일로 변경)
python test.py results/<run_name>/model.pt
```

## 도로망 검수

[검수 도구 안내](inspection/README.md)에 따라 항공사진과 SUMO 도로망을 비교하고, 차로·회전 연결 등 수정사항을 기록한다.

```bash
python -m inspection.serve
```

## 파일 구성

| 경로 | 내용 |
|---|---|
| [yeouido/](yeouido/README.md) | 여의도 도로망·교통량 구축, 시각화, 검증 도구 |
| `env/yeouido/` | 여의도 원본 자료, SUMO 환경, 워밍업 상태 |
| `env/yeouido_env.py` | 여의도 학습 환경 |
| [inspection/](inspection/README.md) | 여의도 도로망 검수 도구와 기록 |
| `algorithms/` | BC·PPO 및 SUMO 전문가 정책 |
| `utils/` | 경험 버퍼, 네트워크, 로깅, 평가 |
| `train.py`, `train_bc.py` | 강화학습·모방학습 실행 |
| `collect_pomdp_data.py`, `test.py` | 데이터 수집 및 모델 평가 |
| `data/`, `results/` | 학습 데이터·모델·실험 기록 |
| `env/sumo/`, `env/sumo_env.py` | 공용 SUMO 실습 환경 |
| `project_materials/` | 프로젝트 소개와 설치·학습 실습 문서 |

## 실습 문서

- [프로젝트 소개](project_materials/0_Project_Introduction.md)
- [Windows 설치](project_materials/1_SUMO_Windows_Setup.md)
- [macOS 설치](project_materials/1_SUMO_macOS_AppleSilicon_Setup.md)
- [공용 도로 환경·POMDP 구성](project_materials/2_Road_Environment_Construction.md)
- [BC 학습](project_materials/3_Model_Training_BC.md)
- [강화학습](project_materials/4_Model_Training_RL.md)

도로 데이터: © OpenStreetMap contributors, [ODbL 및 저작권 안내](https://www.openstreetmap.org/copyright). 개별 자료의 출처는 여의도 문서와 원본 자료 기록을 따른다.
