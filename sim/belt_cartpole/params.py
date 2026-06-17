"""実機 (NEMA17 + GT2ベルト + 400mmリニアレール) を想定した共有パラメータ。

ここを変更すれば MJCF モデル・ステッパモデル・LQR 線形化・RL 環境の
すべてに一貫して反映される。
"""
import json
import math
from dataclasses import dataclass, field
from pathlib import Path


def _load_shared_config() -> dict:
    path = Path(__file__).resolve().parents[2] / "sim_config" / "belt_cartpole.json"
    return json.loads(path.read_text())


@dataclass
class Params:
    # ---------- 機構 ----------
    # GT2 ベルト: ピッチ 2 mm, プーリ 20T → 40 mm/rev
    pulley_teeth: int = 20
    belt_pitch: float = 0.002                      # [m]
    # 400 mm レールからキャリッジ長・エンド余裕を引いた可動域 (±)
    x_lim: float = 0.16                            # [m]
    soft_margin: float = 0.005                     # [m] 端手前の減速マージン

    # ---------- モータ (NEMA17, 例: 42-40 系) ----------
    # 保持トルク ~0.45 N·m。実用域では電圧/速度で目減りするため控えめに。
    torque_lowspeed: float = 0.30                  # [N·m] 低速時の実効トルク
    torque_min: float = 0.10                       # [N·m] 高速時の下限トルク
    v_knee: float = 1.0                            # [m/s] トルク低下の傾き基準
    rotor_inertia: float = 5.7e-6                  # [kg·m^2] ロータ慣性 (57 g·cm^2)

    # ---------- ステッパ駆動の運動制限 (sim2real の核) ----------
    v_max: float = 0.7                             # [m/s]  (~1050 rpm 相当)
    a_max: float = 15.0                            # [m/s^2]
    issued_v_max: float = 0.7                      # [m/s] 発行済みSTEP位置の実効追従速度
    issued_step_m: float = 0.000025                # [m] 1 step相当 (1/8 microstep, 40 steps/mm)
    stall_threshold: float = 0.004                 # [m] 指令-実位置乖離→脱調と判定

    # ---------- ステージ / 振子 ----------
    cart_mass: float = 0.20                        # [kg] キャリッジ+マウント
    pole_len: float = 0.30                         # [m] 棒長
    rod_mass: float = 0.045                        # [kg] カーボン棒など
    tip_mass: float = 0.020                        # [kg] 先端 (マグネット等)
    hinge_damping: float = 1.5e-4                  # [N·m·s] ベアリング粘性
    hinge_frictionloss: float = 5e-4               # [N·m] クーロン摩擦

    # ---------- サーボ (ステッパの位置剛性の近似) ----------
    servo_kp: float = 6000.0                       # [N/m]
    servo_kv: float = 90.0                         # [N·s/m]

    # ---------- 時間 ----------
    timestep: float = 0.0005                       # [s] 物理ステップ
    ctrl_hz: float = 50.0                          # [Hz] 制御 (エージェント) 周期

    # ---------- 派生量 ----------
    @property
    def pulley_radius(self) -> float:
        return self.pulley_teeth * self.belt_pitch / (2.0 * math.pi)

    @property
    def force_lowspeed(self) -> float:
        """低速時のベルト推力上限 [N]"""
        return self.torque_lowspeed / self.pulley_radius

    @property
    def force_min(self) -> float:
        return self.torque_min / self.pulley_radius

    @property
    def reflected_rotor_mass(self) -> float:
        """ロータ慣性の並進換算質量 J/r^2 [kg]。NEMA17+GT2では無視できない。"""
        return self.rotor_inertia / self.pulley_radius ** 2

    @property
    def total_pole_mass(self) -> float:
        return self.rod_mass + self.tip_mass

    @property
    def pole_com(self) -> float:
        """ヒンジから重心までの距離 l_c [m]"""
        m = self.total_pole_mass
        return (self.rod_mass * self.pole_len / 2.0 + self.tip_mass * self.pole_len) / m

    @property
    def pole_inertia(self) -> float:
        """ヒンジ回り慣性 J [kg·m^2] (一様棒 + 先端質点)"""
        return self.rod_mass * self.pole_len ** 2 / 3.0 + self.tip_mass * self.pole_len ** 2

    @property
    def substeps(self) -> int:
        return max(1, round(1.0 / (self.ctrl_hz * self.timestep)))

    @property
    def ctrl_dt(self) -> float:
        return self.substeps * self.timestep


def params_from_shared_config() -> Params:
    data = _load_shared_config()
    mechanics = data["mechanics"]
    motor = data["motor"]
    limits = data["stepper_limits"]
    stage = data["stage_pole"]
    servo = data["servo"]
    timing = data["timing"]
    return Params(
        pulley_teeth=int(mechanics["pulley_teeth"]),
        belt_pitch=float(mechanics["belt_pitch_m"]),
        x_lim=float(mechanics["x_lim_m"]),
        soft_margin=float(mechanics["soft_margin_m"]),
        torque_lowspeed=float(motor["torque_lowspeed_nm"]),
        torque_min=float(motor["torque_min_nm"]),
        v_knee=float(motor["v_knee_mps"]),
        rotor_inertia=float(motor["rotor_inertia_kg_m2"]),
        v_max=float(limits["v_max_mps"]),
        a_max=float(limits["a_max_mps2"]),
        issued_v_max=float(limits.get("issued_v_max_mps", limits["v_max_mps"])),
        issued_step_m=float(limits.get("issued_step_m", 0.000025)),
        stall_threshold=float(limits["stall_threshold_m"]),
        cart_mass=float(stage["cart_mass_kg"]),
        pole_len=float(stage["pole_len_m"]),
        rod_mass=float(stage["rod_mass_kg"]),
        tip_mass=float(stage["tip_mass_kg"]),
        hinge_damping=float(stage["hinge_damping_nms"]),
        hinge_frictionloss=float(stage["hinge_frictionloss_nm"]),
        servo_kp=float(servo["kp_n_per_m"]),
        servo_kv=float(servo["kv_ns_per_m"]),
        timestep=float(timing["timestep_s"]),
        ctrl_hz=float(timing["ctrl_hz"]),
    )


DEFAULT = params_from_shared_config()
