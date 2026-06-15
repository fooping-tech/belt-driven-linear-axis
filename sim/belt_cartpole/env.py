"""MuJoCo シミュレーション本体と Gymnasium 環境。

- BeltCartPoleSim : gym 非依存のコアシミュレータ (古典制御・可視化用)
- BeltCartPoleEnv : RL 用 Gymnasium ラッパ (gymnasium が無くても import 可能)

sim2real 向けオプション:
  - 行動レイテンシ (delay_steps)
  - 観測ノイズ (エンコーダ/IMU 量子化相当のガウスノイズ)
  - ドメインランダマイゼーション (振子質量・ヒンジ摩擦・サーボ剛性)
  - トルク-速度特性による推力上限の動的変化
  - 脱調 (指令位置と実位置の乖離) 検出
"""
from collections import deque

import numpy as np
import mujoco

from params import Params, DEFAULT
from model_xml import build_model_xml
from stepper import StepperDriver


def wrap_pi(a: float) -> float:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


# ----------------------------------------------------------------------------
class BeltCartPoleSim:
    """gym 非依存のコア。制御周期ごとに set_accel() → control_step() を呼ぶ。"""

    def __init__(self, p: Params = DEFAULT, seed: int | None = None,
                 randomize: bool = False):
        self.p = p
        self.rng = np.random.default_rng(seed)
        self.randomize = randomize

        self.model = mujoco.MjModel.from_xml_string(build_model_xml(p))
        self.data = mujoco.MjData(self.model)
        self.stepper = StepperDriver(p)

        self._aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor")
        self._hinge_dof = self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "hinge")]
        self._pole_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pole")

        # ランダマイズで書き換えるノミナル値を保存
        self._nom_body_mass = self.model.body_mass.copy()
        self._nom_body_inertia = self.model.body_inertia.copy()
        self._nom_damping = self.model.dof_damping.copy()
        self._nom_friction = self.model.dof_frictionloss.copy()
        self._nom_gainprm = self.model.actuator_gainprm.copy()
        self._nom_biasprm = self.model.actuator_biasprm.copy()

        self.accel_cmd = 0.0
        self.stalled = False
        self.reset()

    # ------------------------------------------------------------------
    def reset(self, theta0: float | None = None):
        if self.randomize:
            self._apply_randomization()
        mujoco.mj_resetData(self.model, self.data)
        if theta0 is None:
            theta0 = self.rng.uniform(-0.08, 0.08)      # ほぼ真下から
        self.data.qpos[0] = self.rng.uniform(-0.02, 0.02)
        self.data.qpos[1] = theta0
        self.data.qvel[:] = 0.0
        self.stepper.reset(self.data.qpos[0])
        self.data.ctrl[self._aid] = self.stepper.cmd_pos
        self.accel_cmd = 0.0
        self.stalled = False
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def _apply_randomization(self):
        m = self.model
        s_mass = self.rng.uniform(0.85, 1.15)
        m.body_mass[:] = self._nom_body_mass
        m.body_inertia[:] = self._nom_body_inertia
        m.body_mass[self._pole_bid] *= s_mass
        m.body_inertia[self._pole_bid] *= s_mass
        m.dof_damping[:] = self._nom_damping * self.rng.uniform(0.5, 2.0)
        m.dof_frictionloss[:] = self._nom_friction * self.rng.uniform(0.5, 2.0)
        s_kp = self.rng.uniform(0.8, 1.2)
        m.actuator_gainprm[:] = self._nom_gainprm
        m.actuator_biasprm[:] = self._nom_biasprm
        m.actuator_gainprm[self._aid, 0] *= s_kp     # kp
        m.actuator_biasprm[self._aid, 1] *= s_kp     # -kp 側

    # ------------------------------------------------------------------
    def set_accel(self, a: float):
        """制御器からの指令加速度 [m/s^2] (ステッパモデルが制限を適用)"""
        self.accel_cmd = float(a)

    def control_step(self):
        """制御 1 周期 (= substeps 回の物理ステップ) を進める。"""
        p = self.p
        for _ in range(p.substeps):
            x_cmd = self.stepper.step(self.accel_cmd, p.timestep)
            # トルク-速度特性: 推力上限を速度に応じて更新
            f = self.stepper.available_force()
            self.model.actuator_forcerange[self._aid] = (-f, f)
            # 速度フィードフォワード: position actuator の kv は絶対速度に効くため
            # ctrl = x_cmd + (kv/kp)*v_cmd として F = kp(x_cmd-x) + kv(v_cmd-xd) を実現。
            # これが無いと定速走行時に kv*v/kp (~10 mm @0.7 m/s) の偽の追従遅れが出る。
            self.data.ctrl[self._aid] = (
                x_cmd + (p.servo_kv / p.servo_kp) * self.stepper.cmd_vel)
            mujoco.mj_step(self.model, self.data)
        # 脱調判定: 指令と実位置の乖離 (実機ならステップ抜け)
        if abs(self.stepper.cmd_pos - self.data.qpos[0]) > p.stall_threshold:
            self.stalled = True
        return self.state()

    # ------------------------------------------------------------------
    def state(self) -> dict:
        x = float(self.data.qpos[0])
        xd = float(self.data.qvel[0])
        th = float(self.data.qpos[1])
        thd = float(self.data.qvel[1])
        return {
            "x": x, "xdot": xd,
            "theta": th,
            "phi": wrap_pi(th - np.pi),      # 倒立で 0
            "phidot": thd,
            "cmd_pos": self.stepper.cmd_pos,
            "cmd_vel": self.stepper.cmd_vel,
            "stalled": self.stalled,
        }


# ----------------------------------------------------------------------------
def make_obs(s: dict, prev_action: float, p: Params,
             rng: np.random.Generator | None = None,
             noise: float = 0.0) -> np.ndarray:
    """RL の観測ベクトル (古典制御の run でも RL 推論時に共用)。"""
    phi, phid = s["phi"], s["phidot"]
    if rng is not None and noise > 0.0:
        phi = phi + rng.normal(0, noise * 0.01)
        phid = phid + rng.normal(0, noise * 0.1)
    obs = np.array([
        s["x"] / p.x_lim,
        s["xdot"] / p.v_max,
        np.sin(phi),
        np.cos(phi),
        phid / 10.0,
        s["cmd_vel"] / p.v_max,
        prev_action,
    ], dtype=np.float32)
    return obs


OBS_DIM = 7

# ---- gymnasium はオプション依存 (古典制御だけなら不要) --------------------
try:
    import gymnasium as gym
    from gymnasium import spaces
    _GYM = True
except ImportError:                      # pragma: no cover
    _GYM = False

    class _Stub:                          # 最低限の互換スタブ
        class Env:  # noqa: D401
            pass
    gym = _Stub()


class BeltCartPoleEnv(gym.Env):
    """action: [-1, 1] → 指令加速度 a = action * a_max
    reward: 倒立度 - 中央からのずれ - 角速度 - 行動 の各ペナルティ
    脱調で大きな罰 + 終了 (実機でやってはいけない操作を学習で排除)。
    """
    metadata = {"render_modes": []}

    def __init__(self, p: Params = DEFAULT, seed: int | None = None,
                 randomize: bool = True, delay_steps: int = 1,
                 obs_noise: float = 1.0, max_steps: int = 600,
                 terminate_on_stall: bool = True):
        assert _GYM, "gymnasium がインストールされていません (pip install gymnasium)"
        super().__init__()
        self.p = p
        self.sim = BeltCartPoleSim(p, seed=seed, randomize=randomize)
        self.rng = np.random.default_rng(seed)
        self.delay_steps = delay_steps
        self.obs_noise = obs_noise
        self.max_steps = max_steps
        self.terminate_on_stall = terminate_on_stall

        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(OBS_DIM,),
                                            dtype=np.float32)
        self._action_buf: deque = deque()
        self._prev_a = 0.0
        self._t = 0

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
            self.sim.rng = np.random.default_rng(seed)
        s = self.sim.reset()
        # レイテンシ: 行動が delay_steps 周期遅れて反映される
        delay = self.delay_steps
        if self.sim.randomize and delay > 0:
            delay = int(self.rng.integers(0, self.delay_steps + 1))
        self._action_buf = deque([0.0] * delay)
        self._delay = delay
        self._prev_a = 0.0
        self._t = 0
        return make_obs(s, 0.0, self.p, self.rng, self.obs_noise), {}

    def step(self, action):
        a_norm = float(np.clip(np.asarray(action).reshape(-1)[0], -1.0, 1.0))
        if self._delay > 0:
            self._action_buf.append(a_norm)
            applied = self._action_buf.popleft()
        else:
            applied = a_norm

        self.sim.set_accel(applied * self.p.a_max)
        s = self.sim.control_step()
        self._t += 1

        phi = s["phi"]
        upright = 0.5 * (1.0 + np.cos(phi))                       # 0..1
        r = (upright
             - 0.10 * (s["x"] / self.p.x_lim) ** 2
             - 0.001 * s["phidot"] ** 2
             - 0.01 * a_norm ** 2
             - 0.02 * (a_norm - self._prev_a) ** 2)               # 滑らかさ

        terminated = False
        if s["stalled"] and self.terminate_on_stall:
            r -= 5.0
            terminated = True
        truncated = self._t >= self.max_steps
        self._prev_a = a_norm

        obs = make_obs(s, a_norm, self.p, self.rng, self.obs_noise)
        info = {"phi": phi, "stalled": s["stalled"]}
        return obs, float(r), terminated, truncated, info
