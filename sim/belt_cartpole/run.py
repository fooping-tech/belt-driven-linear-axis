"""実行スクリプト: 古典制御 / RL を切り替えてスイングアップ + 倒立維持を可視化。

例:
    python run.py --controller classic
    python run.py --controller rl --rl-model sac_belt_cartpole.zip
    python run.py --controller classic --headless --duration 10

ビューア中のキー操作:
    R   : リセット (真下からやり直し)
    Tab : classic <-> rl 切替 (RL モデル読み込み時のみ)
"""
import argparse
import time

import numpy as np
import mujoco
import mujoco.viewer

from params import DEFAULT
from env import BeltCartPoleSim, make_obs
from classic import HybridController
from observer import FirmwareObserver


class RLController:
    def __init__(self, model_path: str):
        from stable_baselines3 import SAC
        self.policy = SAC.load(model_path, device="cpu")
        self.prev_a = 0.0

    def reset(self):
        self.prev_a = 0.0

    def __call__(self, s: dict) -> float:
        obs = make_obs(s, self.prev_a, DEFAULT)          # 推論時はノイズなし
        a_norm, _ = self.policy.predict(obs, deterministic=True)
        a_norm = float(np.clip(np.asarray(a_norm).reshape(-1)[0], -1.0, 1.0))
        self.prev_a = a_norm
        return a_norm * DEFAULT.a_max


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", choices=["classic", "rl"], default="classic")
    ap.add_argument("--rl-model", type=str, default="sac_belt_cartpole.zip")
    ap.add_argument("--duration", type=float, default=60.0, help="[s]")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--firmware-observer", action="store_true",
                    help="feed the controller AS5600/stepper-style observed state")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    p = DEFAULT
    sim = BeltCartPoleSim(p, seed=args.seed, randomize=False)

    controllers = {"classic": HybridController(p)}
    if args.controller == "rl":
        controllers["rl"] = RLController(args.rl_model)
    active = {"name": args.controller}

    observer = FirmwareObserver(p) if args.firmware_observer else None

    def controller_step(s):
        observed = observer.observe(s, p.ctrl_dt) if observer else s
        return controllers[active["name"]](observed), observed

    # ------------------------------------------------------------------
    if args.headless:
        s = sim.reset(theta0=0.0)
        observed = observer.reset(s) if observer else s
        n = int(args.duration * p.ctrl_hz)
        upright_time = 0.0
        for _ in range(n):
            accel, observed = controller_step(s)
            sim.set_accel(accel)
            s = sim.control_step()
            if abs(observed["phi"]) < 0.2:
                upright_time += p.ctrl_dt
            if s["stalled"]:
                print("!! stall detected (指令と実位置が乖離)")
                break
        print(f"controller={active['name']}  duration={args.duration:.1f}s  "
              f"upright={upright_time:.1f}s  "
              f"final phi={observed['phi']:+.3f} rad  x={observed['x']:+.3f} m")
        return

    # ------------------------------------------------------------------
    def key_cb(keycode):
        if keycode in (ord('R'), ord('r')):
            sim.reset(theta0=0.0)
            if isinstance(controllers.get("rl"), RLController):
                controllers["rl"].reset()
            if isinstance(controllers["classic"], HybridController):
                controllers["classic"].mode = "swingup"
            print("[reset]")
        elif keycode == 258 and "rl" in controllers:        # Tab
            active["name"] = "rl" if active["name"] == "classic" else "classic"
            print(f"[controller -> {active['name']}]")

    s = sim.reset(theta0=0.0)
    observed = observer.reset(s) if observer else s
    t_end = time.time() + args.duration
    with mujoco.viewer.launch_passive(sim.model, sim.data,
                                      key_callback=key_cb) as viewer:
        last_print = 0.0
        while viewer.is_running() and time.time() < t_end:
            step_start = time.time()
            accel, observed = controller_step(s)
            sim.set_accel(accel)
            s = sim.control_step()
            viewer.sync()

            if time.time() - last_print > 1.0:
                last_print = time.time()
                mode = (controllers["classic"].mode
                        if active["name"] == "classic" else "policy")
                print(f"[{active['name']}/{mode}] phi={observed['phi']:+.2f} rad  "
                      f"x={observed['x']:+.3f} m  v_cmd={s['cmd_vel']:+.2f} m/s"
                      + ("  STALL!" if s["stalled"] else ""))

            # 実時間同期
            dt_sleep = p.ctrl_dt - (time.time() - step_start)
            if dt_sleep > 0:
                time.sleep(dt_sleep)


if __name__ == "__main__":
    main()
