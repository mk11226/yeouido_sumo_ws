"""효과 확인(reroute_check) 결과로 '효과 있음'을 판정한다. exit 0 = 효과 있음, 1 = 없음.
기준: (풀린 실패 문제 − 망가진 대조군) ≥ 2  그리고  충돌 0, 빨간불 0.
(2문제 = 60문제 시험지에서 +3%p. 1문제는 우연 차이와 구분이 어려워 효과로 보지 않는다)"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))))  # 저장소 최상위
import json, sys
d = json.load(open("results/reroute_check/expert_reroute.json"))
base = {r["id"]: r for r in json.load(open("results/sumo_expert/random_60_s1_results.json"))["records"]}
recs = d["records"]
fixed = [r["id"] for r in recs if base[r["id"]]["ended_by"] != "arrival" and r["ended_by"] == "arrival"]
broke = [r["id"] for r in recs if base[r["id"]]["ended_by"] == "arrival" and r["ended_by"] != "arrival"]
coll = sum(r["ended_by"] == "collision" for r in recs)
red = sum(r.get("red_lights") or 0 for r in recs)
ok = len(fixed) - len(broke) >= 2 and coll == 0 and red == 0
print(f"풀림 {fixed} 망가짐 {broke} 충돌 {coll} 빨간불 {red} → {'효과 있음' if ok else '효과 없음'}")
sys.exit(0 if ok else 1)
