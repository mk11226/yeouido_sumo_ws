"""PPO(Proximal Policy Optimization) — 알고리즘 본체.

이 파일에는 "알고리즘"만 있다:
    네트워크 구조  → utils/networks.py
    경험 버퍼      → utils/buffer.py   (RolloutBuffer — 메인에서 주입받음)
    정책 평가      → utils/evaluator.py
    로깅          → utils/logger.py
덕분에 아래 코드는 PPO 수식과 1:1로 대응해서 읽을 수 있다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PPO 요약 (Schulman et al., 2017)

policy gradient의 기본:
    advantage A가 양수인 행동의 확률은 높이고, 음수면 낮춘다.

문제: 한 번의 업데이트로 정책이 너무 크게 변하면 학습이 붕괴한다.
해법: 새/옛 정책의 확률비를 클리핑해서 과도한 업데이트의 이득을 제거.

    r(θ) = π_θ(a|s) / π_θ_old(a|s)

    L^CLIP(θ) = E[ min( r(θ)·A ,  clip(r(θ), 1-ε, 1+ε)·A ) ]

전체 손실:
    L = -L^CLIP  +  c_v · L^VF  -  c_e · H[π]
        ────────     ─────────      ───────
        정책 개선     가치 회귀      탐험 유지
    (L^VF = (V(s) - R_target)², H = 정책 엔트로피)

advantage는 GAE(λ)로 추정:
    δ_t = r_t + γ·V(s_{t+1})·(1-done) - V(s_t)
    A_t = δ_t + (γλ)·(1-done)·A_{t+1}          ← 뒤에서 앞으로 재귀
    R_t = A_t + V(s_t)                          ← critic의 회귀 타깃
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

학습 루프:
    반복 {
      1) rollout 수집   : 현재 정책으로 n_steps만큼 환경과 상호작용
      2) advantage 계산 : GAE(λ)
      3) 정책 업데이트  : 같은 데이터로 n_epochs번 미니배치 경사하강
      4) (주기적) 평가  : 노이즈 없는 정책으로 주행 지표 측정
    }
"""
from collections import deque

import numpy as np
import torch
import torch.nn as nn

from utils.networks import GaussianActorCritic, HybridActorCritic
from utils.evaluator import evaluate_policy
from utils.buffer import RolloutBuffer


# ======================================================================
class PPO:
    """PPO 에이전트.

    하이퍼파라미터는 생성자 인자로 명시적으로 받는다
    (train.py에서 한눈에 보고 바꿀 수 있도록).
    """

    def __init__(
        self,
        obs_dim: int,
        act_dim: int,
        # ---- 최적화 ----
        lr: float = 3e-4,
        n_steps: int = 2048,          # rollout 길이 (업데이트 1회당 수집량)
        n_epochs: int = 10,           # 같은 데이터 재사용 횟수
        minibatch_size: int = 64,
        # ---- 할인 / advantage ----
        gamma: float = 0.99,          # 할인율
        gae_lambda: float = 0.95,     # GAE λ
        # ---- PPO 고유 ----
        clip_eps: float = 0.2,        # 클리핑 범위 ε
        value_coef: float = 0.5,      # 가치 손실 가중치 c_v
        entropy_coef: float = 0.0,    # 엔트로피 보너스 c_e (전체 행동)
        lane_entropy_coef: float = 0.0,  # hybrid 전용: "차선 head만"의
                                         # 엔트로피 보너스. 차선변경 확률이
                                         # 초기에 0으로 붕괴하는 것을 늦춘다.
                                         # (가감속 σ에는 영향 없음)
        max_grad_norm: float = 0.5,   # gradient clipping
        normalize_advantage: bool = True,
        target_kl: float = None,      # 한 업데이트에서 정책 변화(KL)가 1.5×이 값을 넘으면 남은 epoch 중단.
                                      # 잘 배운 모델(BC/DAgger)을 이어 학습할 때 한 번에 망가지는 것을 막는다.
        # ---- 네트워크 ----
        policy_type: str = "hybrid",  # "hybrid"(가감속 연속 + 차선 이산, 권장)
                                      # | "gaussian"(모든 축 연속 — dead zone 있음)
        lane_keep_bias: float = 1.0,  # hybrid 전용: 차선 "유지" 초기 편향
        hidden_sizes=(64, 64),
        activation: str = "tanh",
        init_log_std=-0.5,            # 초기 탐험 폭 σ = exp(·). 스칼라 또는
                                      # 차원별 튜플 (utils/networks.py 참고)
        # ---- 기타 ----
        seed: int = 0,
        device: str = "auto",
        buffer: RolloutBuffer = None,   # 경험 버퍼 (메인에서 주입).
                                        # None이면 내부에서 기본 생성.
    ):
        torch.manual_seed(seed)
        np.random.seed(seed)

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)

        # 네트워크는 utils/networks.py 에서 가져온다.
        # hybrid: 가감속(Normal) + 차선변경(Categorical 3-way).
        #   양자화 dead zone이 없어 deterministic 평가에서도 변경이 나온다.
        # gaussian: 모든 축 연속 — 비교 실험용으로 남겨둠.
        if policy_type == "hybrid":
            assert act_dim == 2, "hybrid 정책은 [가감속, 차선] 2차원 행동 전용"
            self.policy = HybridActorCritic(
                obs_dim,
                hidden_sizes=hidden_sizes,
                activation=activation,
                init_log_std=init_log_std,
                lane_keep_bias=lane_keep_bias,
            ).to(self.device)
        else:
            self.policy = GaussianActorCritic(
                obs_dim, act_dim,
                hidden_sizes=hidden_sizes,
                activation=activation,
                init_log_std=init_log_std,
            ).to(self.device)
        self.policy_type = policy_type
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=lr)

        self.n_steps = n_steps
        self.n_epochs = n_epochs
        self.minibatch_size = minibatch_size
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_eps = clip_eps
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.lane_entropy_coef = lane_entropy_coef
        self.max_grad_norm = max_grad_norm
        self.normalize_advantage = normalize_advantage
        self.target_kl = target_kl

        # 버퍼 주입(dependency injection):
        # train.py(메인)가 utils.buffer.RolloutBuffer를 만들어 넘겨준다.
        # 어떤 버퍼를 쓰는지가 메인에서 보이고, 나중에 다른 버퍼로 교체할 때
        # 이 파일을 수정할 필요가 없다.
        self.buffer = buffer if buffer is not None else RolloutBuffer()
        self.num_timesteps = 0

    # ==================================================================
    # 행동 선택
    # ==================================================================
    @torch.no_grad()
    def act(self, obs):
        """학습용 확률적 행동. Returns (action, log_prob, value).

        hybrid: 가감속은 Normal 샘플, 차선은 Categorical 샘플(-1/0/+1).
        gaussian: 기존과 동일한 전차원 Normal 샘플.
        """
        obs_t = torch.as_tensor(obs, dtype=torch.float32,
                                device=self.device).unsqueeze(0)
        if hasattr(self.policy, "sample"):          # hybrid 경로
            action, log_prob, value = self.policy.sample(obs_t)
            return (action.squeeze(0).cpu().numpy().astype(np.float32),
                    float(log_prob.item()), float(value.item()))
        mu, std, value = self.policy(obs_t)          # gaussian 경로
        dist = torch.distributions.Normal(mu, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(-1)
        return (action.squeeze(0).cpu().numpy().astype(np.float32),
                float(log_prob.item()), float(value.item()))

    @torch.no_grad()
    def predict(self, obs, deterministic: bool = True):
        """평가/실행용. hybrid의 deterministic = (μ, argmax 차선).

        utils/evaluator.py 가 요구하는 인터페이스이기도 하다.
        """
        obs_t = torch.as_tensor(obs, dtype=torch.float32,
                                device=self.device).unsqueeze(0)
        if hasattr(self.policy, "det_action"):       # hybrid 경로
            if deterministic:
                a = self.policy.det_action(obs_t)
            else:
                a, _, _ = self.policy.sample(obs_t)
            return a.squeeze(0).cpu().numpy().astype(np.float32)
        mu, std, _ = self.policy(obs_t)              # gaussian 경로
        if deterministic:
            return mu.squeeze(0).cpu().numpy().astype(np.float32)
        return (torch.normal(mu, std).squeeze(0)
                .cpu().numpy().astype(np.float32))

    @torch.no_grad()
    def get_value(self, obs) -> float:
        obs_t = torch.as_tensor(obs, dtype=torch.float32,
                                device=self.device).unsqueeze(0)
        value = self.policy(obs_t)[-1]   # 마지막 원소 = V(s) (정책 종류 무관)
        return float(value.item())

    # ==================================================================
    # 1단계) rollout 수집
    # ==================================================================
    def collect_rollout(self, env, state, logger=None, ep_stats=None):
        """현재 정책으로 n_steps만큼 경험을 모은다.

        Args:
            state: 직전 rollout이 끝난 시점의 관측 (에피소드를 이어서 수집)
            ep_stats: [ep_return, ep_length] 리스트 (호출 간 누적 유지)
        Returns:
            (last_obs, last_value, finished_returns)
        """
        obs = state
        finished_returns = []
        if ep_stats is None:
            ep_stats = [0.0, 0]
        self.buffer.clear()

        for _ in range(self.n_steps):
            action, log_prob, value = self.act(obs)
            next_obs, reward, terminated, truncated, _ = env.step(action)

            self.num_timesteps += 1
            ep_stats[0] += reward
            ep_stats[1] += 1

            # ★ truncation(시간 초과) 보정:
            #   MDP가 끝난 게 아니라 시간 제한으로 잘린 것이므로,
            #   이어졌다면 받았을 미래 가치를 V(s')로 근사해 보상에 더한다.
            #   생략하면 "시간초과 = 이후 가치 0"으로 잘못 학습된다.
            stored_reward = reward
            if truncated and not terminated:
                stored_reward += self.gamma * self.get_value(next_obs)

            # 시간 초과의 부트스트랩은 위 보상에 이미 반영됐다.
            # GAE는 reset 이후의 별도 에피소드 가치/advantage를 섞지 않도록
            # terminated와 truncated 모두에서 재귀를 끊어야 한다.
            self.buffer.add(obs, action, log_prob, stored_reward, value,
                            terminated or truncated)

            if terminated or truncated:
                finished_returns.append(ep_stats[0])
                if logger is not None:
                    logger.log_episode(self.num_timesteps, ep_stats[0],
                                       int(ep_stats[1]))
                ep_stats[0], ep_stats[1] = 0.0, 0
                obs, _ = env.reset()
            else:
                obs = next_obs

        # rollout이 에피소드 중간에 끊겼다면 이후 가치를 부트스트랩
        return obs, self.get_value(obs), finished_returns

    # ==================================================================
    # 3단계) 정책 업데이트 (clipped surrogate)
    # ==================================================================
    def update(self, advantages, returns, value_only: bool = False) -> dict:
        """value_only=True: 정책(trunk·mu·lane·log_std)은 그대로 두고 v_head 만 학습 (critic 워밍업)."""
        obs_t, act_t, old_log_prob_t = self.buffer.to_tensors(self.device)
        # 병렬 환경이면 (T, N, ·) → (T·N, ·) 로 펼친다 (단일 환경이면 그대로)
        obs_t = obs_t.reshape(-1, obs_t.shape[-1])
        act_t = act_t.reshape(-1, act_t.shape[-1])
        old_log_prob_t = old_log_prob_t.reshape(-1)
        adv_t = torch.as_tensor(np.asarray(advantages).reshape(-1), device=self.device)
        ret_t = torch.as_tensor(np.asarray(returns).reshape(-1), device=self.device)
        if value_only:
            return self._update_value_only(obs_t, ret_t)

        # advantage 정규화: 스케일을 평균0/표준편차1로 → 학습률 민감도 감소
        if self.normalize_advantage:
            adv_t = (adv_t - adv_t.mean()) / (adv_t.std() + 1e-8)

        T = len(adv_t)
        indices = np.arange(T)
        stats = {"policy_loss": 0.0, "value_loss": 0.0,
                 "entropy": 0.0, "approx_kl": 0.0, "clip_frac": 0.0}
        n_batches = 0
        stopped_early = False

        for _ in range(self.n_epochs):
            if stopped_early:
                break
            np.random.shuffle(indices)
            for start in range(0, T, self.minibatch_size):
                mb = indices[start:start + self.minibatch_size]

                # 현재 정책으로 "그때 그 행동"을 다시 평가
                log_prob, entropy, value = self.policy.evaluate_actions(
                    obs_t[mb], act_t[mb])

                # 확률비 r(θ) = exp(log π_new - log π_old)
                ratio = torch.exp(log_prob - old_log_prob_t[mb])

                # ---- PPO 핵심: clipped surrogate objective ----
                surrogate_1 = ratio * adv_t[mb]
                surrogate_2 = torch.clamp(ratio, 1 - self.clip_eps,
                                          1 + self.clip_eps) * adv_t[mb]
                policy_loss = -torch.min(surrogate_1, surrogate_2).mean()

                # ---- critic: V(s)를 GAE 타깃에 회귀 ----
                value_loss = ((value - ret_t[mb]) ** 2).mean()

                # ---- 엔트로피 보너스: 탐험 유지 ----
                entropy_loss = -entropy.mean()

                loss = (policy_loss
                        + self.value_coef * value_loss
                        + self.entropy_coef * entropy_loss)
                # 차선 head 전용 엔트로피 보너스 (hybrid에서만 동작)
                if self.lane_entropy_coef > 0 and \
                        hasattr(self.policy, "lane_entropy"):
                    loss = loss - (self.lane_entropy_coef
                                   * self.policy.lane_entropy(obs_t[mb]).mean())

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.policy.parameters(),
                                         self.max_grad_norm)
                self.optimizer.step()

                # 진단 지표
                with torch.no_grad():
                    # approx_kl: 정책이 얼마나 변했는지. 0.02를 크게 넘으면
                    #   업데이트가 과격하다는 신호 (lr을 낮출 것)
                    log_ratio = log_prob - old_log_prob_t[mb]
                    approx_kl = ((torch.exp(log_ratio) - 1) - log_ratio).mean()   # 항상 ≥0 인 추정 (Schulman)
                    # clip_frac: 클리핑에 걸린 샘플 비율. 0.1~0.3이 보통
                    clip_frac = ((ratio - 1).abs() > self.clip_eps).float().mean()

                stats["policy_loss"] += policy_loss.item()
                stats["value_loss"] += value_loss.item()
                stats["entropy"] += entropy.mean().item()
                stats["approx_kl"] += approx_kl.item()
                stats["clip_frac"] += clip_frac.item()
                n_batches += 1
                if self.target_kl is not None and approx_kl.item() > 1.5 * self.target_kl:
                    stopped_early = True
                    break

        out = {k: v / max(n_batches, 1) for k, v in stats.items()}
        out["n_minibatches"] = n_batches
        out["kl_stop"] = float(stopped_early)
        # hybrid 진단: 업데이트 "후" 정책의 평균 차선변경(좌+우) 확률.
        #   0으로 붕괴 → 차선유지 지역최적 경고 / 적당히 유지되며
        #   eval의 차선변경 횟수가 오르면 제대로 배우는 중.
        if hasattr(self.policy, "lane_change_prob"):
            with torch.no_grad():
                out["p_lane_change"] = float(
                    self.policy.lane_change_prob(obs_t).mean().item())
        return out

    # ==================================================================
    # 메인 학습 루프
    # ==================================================================
    def learn(self, env, total_timesteps: int, logger=None,
              log_interval: int = 1, eval_interval: int = 0,
              eval_episodes: int = 5):
        """학습 실행.

        Args:
            logger: utils.logger.RunLogger (없으면 콘솔 출력만)
            eval_interval: 몇 번의 업데이트마다 평가할지 (0이면 평가 안 함)
        """
        obs, _ = env.reset()
        ep_stats = [0.0, 0]                    # [현재 에피소드 보상, 길이]
        recent_returns = deque(maxlen=20)
        n_updates = 0

        while self.num_timesteps < total_timesteps:
            # 1) rollout
            obs, last_value, finished = self.collect_rollout(
                env, obs, logger=logger, ep_stats=ep_stats)
            recent_returns.extend(finished)

            # 2) advantage
            advantages, returns = self.buffer.compute_gae(
                last_value, self.gamma, self.gae_lambda)

            # 3) update
            stats = self.update(advantages, returns)
            n_updates += 1

            # 로깅
            metrics = {
                "update": n_updates,
                "ep_rew_mean": float(np.mean(recent_returns))
                               if recent_returns else float("nan"),
                "std": self.policy.current_std,
                **stats,
            }
            if logger is not None:
                logger.log_train(self.num_timesteps, metrics,
                                 verbose=(n_updates % log_interval == 0))

            # 4) 주기적 평가 (utils/evaluator.py 사용)
            if eval_interval > 0 and n_updates % eval_interval == 0:
                m = evaluate_policy(self, env, n_episodes=eval_episodes)
                if logger is not None:
                    logger.log_eval(self.num_timesteps, m.as_dict(),
                                    m.summary(), eval_episodes)
                # 평가가 학습 중이던 에피소드를 끊었으므로 리셋.
                # (끊긴 경험은 이미 버퍼에 반영됐고 GAE가 부트스트랩하므로 무해)
                obs, _ = env.reset()
                ep_stats[0], ep_stats[1] = 0.0, 0

    # ==================================================================
    # critic 워밍업 — BC/DAgger 모델은 v_head 를 학습하지 않았다 (무작위 그대로).
    # 그 상태로 PPO 를 돌리면 엉터리 advantage 로 잘 배운 정책을 망가뜨리므로,
    # 처음 몇 번은 정책을 고정하고 V(s) 만 실제 return 에 맞춘다.
    # trunk 는 정책과 공유하므로 건드리지 않고(특징만 꺼내 씀) v_head 만 따로 학습한다.
    # ==================================================================
    def _update_value_only(self, obs_t, ret_t, lr: float = 1e-3, epochs: int = 10) -> dict:
        if not hasattr(self, "_v_opt"):
            self._v_opt = torch.optim.Adam(self.policy.v_head.parameters(), lr=lr)
        with torch.no_grad():
            feats = self.policy.trunk(obs_t)
        T = len(ret_t)
        indices = np.arange(T)
        loss_sum, n = 0.0, 0
        for _ in range(epochs):
            np.random.shuffle(indices)
            for start in range(0, T, self.minibatch_size):
                mb = indices[start:start + self.minibatch_size]
                value = self.policy.v_head(feats[mb]).squeeze(-1)
                loss = ((value - ret_t[mb]) ** 2).mean()
                self._v_opt.zero_grad()
                loss.backward()
                self._v_opt.step()
                loss_sum += loss.item()
                n += 1
        with torch.no_grad():
            v = self.policy.v_head(feats).squeeze(-1)
            ev = 1 - torch.var(ret_t - v) / (torch.var(ret_t) + 1e-8)
        return {"value_loss": loss_sum / max(n, 1), "explained_var": float(ev.item()),
                "policy_loss": 0.0, "approx_kl": 0.0, "clip_frac": 0.0}

    # ==================================================================
    # 병렬 환경 (utils/vec_env.SubprocVecEnv) 수집 / 학습 루프
    # ==================================================================
    def collect_rollout_vec(self, venv, obs, episodes_out: list):
        """N개 환경에서 각각 n_steps 씩 (총 N·n_steps) 모은다. 끝난 에피소드 요약은 episodes_out 에 추가."""
        self.buffer.clear()
        for _ in range(self.n_steps):
            obs_t = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
            with torch.no_grad():
                action, log_prob, value = self.policy.sample(obs_t) if hasattr(self.policy, "sample") \
                    else self._gaussian_sample(obs_t)
            action = action.cpu().numpy().astype(np.float32)
            next_obs, reward, term, trunc, final_obs, episodes = venv.step(action)
            self.num_timesteps += venv.n_envs
            stored = reward.copy()
            # 시간 초과(잘린 것)는 이어졌다면 받았을 가치를 V(마지막 관측)로 더한다 (단일 환경 경로와 같음)
            for i in range(venv.n_envs):
                if trunc[i] and not term[i]:
                    stored[i] += self.gamma * self.get_value(final_obs[i])
                if episodes[i] is not None:
                    episodes_out.append(episodes[i])
            self.buffer.add(obs, action, log_prob.cpu().numpy(), stored, value.cpu().numpy(),
                            np.logical_or(term, trunc))
            obs = next_obs
        with torch.no_grad():
            last_value = self.policy(torch.as_tensor(obs, dtype=torch.float32, device=self.device))[-1]
        return obs, last_value.cpu().numpy()

    def _gaussian_sample(self, obs_t):
        mu, std, value = self.policy(obs_t)
        dist = torch.distributions.Normal(mu, std)
        a = dist.sample()
        return a, dist.log_prob(a).sum(-1), value

    def learn_vec(self, venv, total_timesteps: int, logger=None, critic_warmup: int = 0,
                  callback=None, n_updates: int = 0):
        """병렬 학습. 처음 critic_warmup 번은 v_head 만 학습. callback(n_updates, metrics) 가 False 면 멈춘다."""
        obs = venv.reset()
        recent = deque(maxlen=50)
        while self.num_timesteps < total_timesteps:
            episodes = []
            obs, last_value = self.collect_rollout_vec(venv, obs, episodes)
            recent.extend(episodes)
            if logger is not None:
                for e in episodes:
                    logger.log_episode(self.num_timesteps, e["ret"], int(e["len"]))
            advantages, returns = self.buffer.compute_gae(last_value, self.gamma, self.gae_lambda)
            warm = n_updates < critic_warmup
            stats = self.update(advantages, returns, value_only=warm)
            n_updates += 1
            ended = [e["ended_by"] for e in recent]
            metrics = {
                "update": n_updates,
                "phase": "critic" if warm else "ppo",
                "ep_rew_mean": float(np.mean([e["ret"] for e in recent])) if recent else float("nan"),
                "arrive": ended.count("arrival") / len(ended) if ended else float("nan"),
                "collide": ended.count("collision") / len(ended) if ended else float("nan"),
                "teleport": ended.count("teleport") / len(ended) if ended else float("nan"),
                "n_ep": len(ended),
                "std": self.policy.current_std,
                **stats,
            }
            if logger is not None:
                logger.log_train(self.num_timesteps, {k: v for k, v in metrics.items() if k != "phase"},
                                 verbose=False)
            if callback is not None and callback(n_updates, metrics) is False:
                break
        return n_updates

    def save_checkpoint(self, path: str, **extra):
        """이어 학습용: 가중치 + optimizer + 진행 상황."""
        state = {"policy": self.policy.state_dict(), "optimizer": self.optimizer.state_dict(),
                 "num_timesteps": self.num_timesteps, **extra}
        if hasattr(self, "_v_opt"):
            state["v_optimizer"] = self._v_opt.state_dict()
        torch.save(state, path + ".tmp")
        import os
        os.replace(path + ".tmp", path)

    def load_checkpoint(self, path: str) -> dict:
        state = torch.load(path, map_location=self.device, weights_only=False)
        self.policy.load_state_dict(state["policy"])
        self.optimizer.load_state_dict(state["optimizer"])
        self.num_timesteps = int(state["num_timesteps"])
        return state

    # ==================================================================
    def save(self, path: str):
        torch.save(self.policy.state_dict(), path)

    def load(self, path: str):
        self.policy.load_state_dict(
            torch.load(path, map_location=self.device))
