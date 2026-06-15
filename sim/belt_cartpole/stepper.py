"""ステッパドライバの簡易モデル (sim2real の中核)。

実機の「ステッパ + ドライバ (例: TMC2209) + ファームの台形/加速度制限プロファイル」
を次のように抽象化する:

  - 制御入力は「指令加速度 a_cmd」(RL/古典制御の出力)
  - 内部で指令速度 v_cmd を積分: |v_cmd| <= v_max, |dv/dt| <= a_max
  - 指令位置 x_cmd を積分: レール可動域でクランプ
  - 端に最大減速度で止まれない速度では自動的にフルブレーキ
    (実機ファームのソフトリミットに相当)
  - x_cmd は MuJoCo の高剛性位置サーボ (推力上限つき) に渡される。
    推力上限を超える負荷では実位置が指令から乖離 → 「脱調」とみなす。

つまりエージェントは理想的な力を出せず、必ず
  速度制限・加速度制限・位置制限・推力(トルク)制限
を通した上でしか台車を動かせない。
"""
import numpy as np
from params import Params, DEFAULT


class StepperDriver:
    def __init__(self, p: Params = DEFAULT):
        self.p = p
        self.reset()

    def reset(self, pos: float = 0.0):
        self.cmd_pos = float(pos)
        self.cmd_vel = 0.0
        self.saturated_v = False   # 直前ステップで速度制限に当たったか
        self.braking = False       # 端ブレーキ中か

    def step(self, accel_cmd: float, dt: float) -> float:
        """指令加速度を 1 物理ステップぶん積分し、新しい指令位置を返す。"""
        p = self.p
        a = float(np.clip(accel_cmd, -p.a_max, p.a_max))

        # --- レール端保護: いま全力で減速しても止まれないなら強制ブレーキ ---
        self.braking = False
        v = self.cmd_vel
        if abs(v) > 1e-9:
            stop_dist = v * v / (2.0 * p.a_max)
            limit = p.x_lim - p.soft_margin
            if v > 0 and self.cmd_pos + stop_dist >= limit:
                a = -p.a_max
                self.braking = True
            elif v < 0 and self.cmd_pos - stop_dist <= -limit:
                a = p.a_max
                self.braking = True

        # --- 速度積分 + 速度制限 ---
        v_new = v + a * dt
        self.saturated_v = abs(v_new) >= p.v_max
        v_new = float(np.clip(v_new, -p.v_max, p.v_max))

        # --- 位置積分 + ハードクランプ ---
        x_new = self.cmd_pos + v_new * dt
        if x_new >= p.x_lim:
            x_new, v_new = p.x_lim, 0.0
        elif x_new <= -p.x_lim:
            x_new, v_new = -p.x_lim, 0.0

        self.cmd_vel = v_new
        self.cmd_pos = x_new
        return x_new

    def available_force(self) -> float:
        """トルク-速度特性: 速度が上がるほど実効推力が落ちる (直線近似)。"""
        p = self.p
        f = p.force_lowspeed * (1.0 - abs(self.cmd_vel) / p.v_knee)
        return float(max(p.force_min, f))
