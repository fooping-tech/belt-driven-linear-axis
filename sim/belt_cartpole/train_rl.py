"""RL 学習スクリプト (Stable-Baselines3 / SAC)。

使い方:
    pip install "stable-baselines3>=2.0" gymnasium mujoco
    python train_rl.py --steps 400000 --envs 8

学習中はドメインランダマイゼーション・観測ノイズ・行動レイテンシを有効にし、
脱調 (推力上限超過による指令-実位置の乖離) は即終了 + 罰として
「実機で安全な方策」へ寄せる。
"""
import argparse

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.callbacks import EvalCallback

from env import BeltCartPoleEnv
from params import DEFAULT


def make_env(rank: int, randomize: bool = True):
    def _f():
        return BeltCartPoleEnv(DEFAULT, seed=1000 + rank,
                               randomize=randomize, delay_steps=1,
                               obs_noise=1.0)
    return _f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=400_000)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--out", type=str, default="sac_belt_cartpole")
    args = ap.parse_args()

    venv = VecMonitor(SubprocVecEnv([make_env(i) for i in range(args.envs)]))
    eval_env = VecMonitor(SubprocVecEnv([make_env(99, randomize=False)]))

    model = SAC(
        "MlpPolicy", venv,
        learning_rate=3e-4,
        buffer_size=400_000,
        batch_size=512,
        gamma=0.99,
        tau=0.01,
        train_freq=1,
        gradient_steps=1,
        policy_kwargs=dict(net_arch=[256, 256]),
        verbose=1,
        seed=0,
    )
    cb = EvalCallback(eval_env, eval_freq=10_000 // args.envs,
                      n_eval_episodes=5, best_model_save_path="./best")
    model.learn(total_timesteps=args.steps, callback=cb)
    model.save(args.out)
    print(f"saved -> {args.out}.zip (best model: ./best/best_model.zip)")


if __name__ == "__main__":
    main()
