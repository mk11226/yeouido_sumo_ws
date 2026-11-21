"""중간에 멈춘 collect_pomdp_data.py 의 워커별 부분 파일(.<name>.w*.jsonl.gz)에서
끝까지 기록된 에피소드만 모아 정식 결과(<name>.jsonl.gz + <name>.npz)를 만든다.

    python -m training.salvage_parts expert_dart600 --out expert_dart204

collect_pomdp_data.py 는 에피소드를 다 돌아야 NPZ 를 쓰므로, 수집을 일찍 멈추면 NPZ 가 없다.
JSONL 은 스텝마다 쓰고 있었으므로 거기서 복원한다. 마지막에 돌던 미완성 에피소드와
gzip 이 덜 쓰인 끝부분은 버린다. NPZ 칼럼은 collect_pomdp_data.py 와 같다.
"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))  # 저장소 최상위 (env, utils 등)
import argparse
import glob
import gzip
import json
import os
import zlib

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 저장소 최상위


def read_lines(path):
    """덜 닫힌 gzip 도 읽을 수 있는 만큼 읽는다 (마지막 불완전한 줄은 버림)."""
    d = zlib.decompressobj(16 + zlib.MAX_WBITS)
    text = d.decompress(open(path, "rb").read()).decode("utf-8", "ignore")
    return text.split("\n")[:-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name")
    ap.add_argument("--out", required=True)
    ap.add_argument("--data-dir", default=os.path.join(BASE, "training", "data"))
    args = ap.parse_args()

    parts = sorted(glob.glob(os.path.join(args.data_dir, f".{args.name}.w*.jsonl.gz")))
    if not parts:
        raise SystemExit("부분 파일이 없습니다")
    episodes = []                                   # [[record, ...], ...] 끝난 에피소드만
    for p in parts:
        cur = []
        for ln in read_lines(p):
            try:
                r = json.loads(ln)
            except ValueError:
                break
            if cur and r["episode"] != cur[0]["episode"]:
                cur = []                            # (있을 수 없지만) 끝 표시 없이 바뀌면 버림
            cur.append(r)
            if r["terminated"] or r["truncated"]:
                episodes.append(cur)
                cur = []
        print(f"  {os.path.basename(p)}: 끝난 에피소드 누적 {len(episodes)} (미완성 {len(cur)} 스텝 버림)")

    cols = {k: [] for k in ("observation", "next_observation", "action_raw", "action_executed",
                            "lane_change", "reward", "terminated", "truncated", "episode", "t")}
    xy, offsets = [], [0]
    out_jsonl = os.path.join(args.data_dir, args.out + ".jsonl.gz")
    with gzip.open(out_jsonl, "wt", encoding="utf-8", compresslevel=6) as jf:
        for ep in episodes:
            for i, r in enumerate(ep):
                jf.write(json.dumps(r, ensure_ascii=False, separators=(",", ":")) + "\n")
                nxt = r["next_observation"] if i == len(ep) - 1 else ep[i + 1]["observation"]
                cols["observation"].append(r["observation"])
                cols["next_observation"].append(nxt)
                cols["action_raw"].append(r["action_raw"])
                cols["action_executed"].append([r["action"]["accel_raw"], r["action"]["lane_change_raw"]])
                cols["lane_change"].append(r["action"]["lane_change"])
                cols["reward"].append(r["reward"])
                cols["terminated"].append(r["terminated"])
                cols["truncated"].append(r["truncated"])
                cols["episode"].append(r["episode"])
                cols["t"].append(r["t"])
                xy.extend((d["x"], d["y"]) for d in r["detected_vehicles"])
                offsets.append(len(xy))
    dtypes = dict(observation=np.float32, next_observation=np.float32, action_raw=np.float32,
                  action_executed=np.float32, lane_change=np.int8, reward=np.float32,
                  terminated=np.bool_, truncated=np.bool_, episode=np.int32, t=np.int32)
    arrays = {k: np.asarray(v, dtype=dtypes[k]) for k, v in cols.items()}
    arrays["detected_xy"] = np.asarray(xy, dtype=np.float32).reshape(-1, 2)
    arrays["detected_offsets"] = np.asarray(offsets, dtype=np.int64)
    out_npz = os.path.join(args.data_dir, args.out + ".npz")
    np.savez_compressed(out_npz, **arrays)
    print(f"에피소드 {len(episodes)}, transitions {len(arrays['reward']):,}")
    print(f"  → {out_jsonl} ({os.path.getsize(out_jsonl) / 2**20:.1f} MB)")
    print(f"  → {out_npz} ({os.path.getsize(out_npz) / 2**20:.1f} MB)")


if __name__ == "__main__":
    main()
