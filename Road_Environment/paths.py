"""저장소 및 시나리오의 공통 경로."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / "env/skku_commute"
