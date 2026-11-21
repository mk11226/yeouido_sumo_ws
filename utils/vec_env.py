"""여러 SUMO 환경을 프로세스마다 하나씩 띄워 동시에 돌리는 벡터 환경 (PPO 병렬 수집용).

병목은 신경망이 아니라 SUMO 시뮬레이션이라 환경을 N개 동시에 돌리면 수집이 약 N배 빨라진다.
libsumo(LIBSUMO_AS_TRACI=quiet)는 한 프로세스에 시뮬레이션 하나만 가능하므로 프로세스로 나눈다.

    venv = SubprocVecEnv(env_kwargs, n_envs=8, seed=0)
    obs = venv.reset()                                  # (N, obs_dim)
    obs, rew, term, trunc, final_obs, episodes = venv.step(actions)   # actions (N, act_dim)

- 끝난 환경은 워커가 바로 reset 하고 새 에피소드의 첫 관측을 obs 로 돌려준다.
  끝나기 직전의 관측은 final_obs[i] (시간 초과 부트스트랩용), 에피소드 요약은 episodes[i] (끝나지 않았으면 None).
- 출발 지점이 막혀 reset 이 RuntimeError 를 내면 다른 문제로 다시 뽑는다 (학습이 멈추지 않게).
"""
import multiprocessing as mp

import numpy as np


def _ended_by(info, terminated):
    if info.get("collided"):
        return "collision"
    if info.get("teleported"):
        return "teleport"
    if info.get("arrived"):
        return "arrival"
    return "timeout" if not terminated else "other"


def _worker(remote, env_kwargs, seed):
    import torch
    torch.set_num_threads(1)
    np.random.seed(seed)                     # SUMO 시드는 전역 np.random 에서 뽑는다
    from env.yeouido_env import make_env
    env = make_env(**env_kwargs)
    first = [True]
    stats = {"ret": 0.0, "len": 0}

    def do_reset():
        for attempt in range(10):
            try:
                obs, _ = env.reset(seed=seed if first[0] else None)
                first[0] = False
                stats["ret"], stats["len"] = 0.0, 0
                return obs
            except RuntimeError as e:        # 출발 실패 → 다른 출발·도착으로 다시
                first[0] = False
                print(f"[vec_env {seed}] reset 실패, 다시 뽑음 ({attempt + 1}/10): {e}", flush=True)
        raise RuntimeError("reset 10번 연속 실패")

    try:
        while True:
            cmd, data = remote.recv()
            if cmd == "reset":
                remote.send(do_reset())
            elif cmd == "step":
                obs, reward, terminated, truncated, info = env.step(data)
                stats["ret"] += reward
                stats["len"] += 1
                final_obs, episode = None, None
                if terminated or truncated:
                    final_obs = obs
                    episode = dict(ended_by=_ended_by(info, terminated), ret=stats["ret"], len=stats["len"],
                                   level=getattr(env, "level", None),
                                   red_lights=info.get("red_lights"), shield_steps=info.get("shield_steps"))
                    obs = do_reset()
                remote.send((obs, reward, terminated, truncated, final_obs, episode))
            elif cmd == "close":
                break
    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        remote.close()


class SubprocVecEnv:
    def __init__(self, env_kwargs: dict, n_envs: int, seed: int = 0):
        ctx = mp.get_context("spawn")
        self.n_envs = n_envs
        self.remotes, self.procs = [], []
        for i in range(n_envs):
            parent, child = ctx.Pipe()
            p = ctx.Process(target=_worker, args=(child, env_kwargs, seed * 1000 + i), daemon=True)
            p.start()
            child.close()
            self.remotes.append(parent)
            self.procs.append(p)
        self.closed = False

    def reset(self):
        for r in self.remotes:
            r.send(("reset", None))
        return np.stack([r.recv() for r in self.remotes])

    def step(self, actions):
        for r, a in zip(self.remotes, actions):
            r.send(("step", a))
        results = [r.recv() for r in self.remotes]
        obs, rew, term, trunc, final_obs, episodes = zip(*results)
        return (np.stack(obs), np.asarray(rew, dtype=np.float32), np.asarray(term), np.asarray(trunc),
                list(final_obs), list(episodes))

    def close(self):
        if self.closed:
            return
        self.closed = True
        for r in self.remotes:
            try:
                r.send(("close", None))
            except (BrokenPipeError, EOFError, OSError):
                pass
        for p in self.procs:
            p.join(timeout=30)
            if p.is_alive():
                p.terminate()
