"""古典制御: エネルギー法スイングアップ (Åström–Furuta) + LQR 倒立維持。

角度は倒立から測った phi (= wrap(theta - pi)) を使う。台車加速度 a を入力とする
カートポールの式:
    J*phi'' = m*g*lc*sin(phi) - m*lc*a*cos(phi) - b*phi'

エネルギー (倒立静止で 0):
    E = 1/2*J*phi'^2 + m*g*lc*(cos(phi) - 1)
    dE/dt = -m*lc*a*phi'*cos(phi)
よって a = k_e * E * phi' * cos(phi) とすると E<0 (倒立未満) のとき dE/dt >= 0
となりエネルギーが汲み上げられる。a_max で飽和させるので、これはそのまま
ステッパの加速度制限と整合する。

倒立近傍では LQR に切り替える。線形化 (phi ≈ 0):
    x'' = a
    phi'' = alpha*(g*phi - a) - (b/J)*phi',   alpha = m*lc/J
"""
import numpy as np
from scipy.linalg import solve_continuous_are

from params import Params, DEFAULT, _load_shared_config

G = 9.81


class HybridController:
    """mode: 'swingup' / 'lqr'。__call__(state) -> 指令加速度 [m/s^2]"""

    # 切替条件 (ヒステリシス付き)
    CATCH_PHI = 0.30        # [rad] これより近づいたら LQR
    CATCH_PHID = 4.0        # [rad/s]
    RELEASE_PHI = 0.80      # [rad] これより外れたらスイングアップに戻る

    def __init__(self, p: Params = DEFAULT,
                 k_energy: float | None = None, swing_sat: float | None = None,
                 q=None, r: float | None = None):
        self.p = p
        cfg = _load_shared_config()["controller"]
        use_configured_gains = q is None and r is None
        if k_energy is None:
            k_energy = float(cfg["k_energy"])
        configured_gains = cfg.get("lqr_gains")
        if q is None:
            q = tuple(float(v) for v in cfg["lqr_q"])
        if r is None:
            r = float(cfg["lqr_r"])
        m, lc, J = p.total_pole_mass, p.pole_com, p.pole_inertia
        b = p.hinge_damping
        alpha = m * lc / J

        # ---- LQR ゲイン ----
        if configured_gains and use_configured_gains:
            self.K = np.array([float(v) for v in configured_gains])
        else:
            A = np.array([[0, 1, 0, 0],
                          [0, 0, 0, 0],
                          [0, 0, 0, 1],
                          [0, 0, G * alpha, -b / J]])
            B = np.array([[0.0], [1.0], [0.0], [-alpha]])
            Q = np.diag(q)
            R = np.array([[r]])
            P = solve_continuous_are(A, B, Q, R)
            self.K = (np.linalg.solve(R, B.T @ P)).ravel()   # a = -K @ [x,xd,phi,phid]

        # ---- スイングアップ ----
        self.k_energy = k_energy
        self.swing_sat = swing_sat if swing_sat is not None else float(cfg.get("swing_sat_ratio", 0.9)) * p.a_max
        self.deadlock_phidot = float(cfg.get("deadlock_phidot_rad_s", 0.08))
        self.deadlock_accel = float(cfg.get("deadlock_accel_mps2", 3.0))
        self.center_hold_kx = float(cfg.get("center_hold_kx", 8.0))
        self.center_hold_kd = float(cfg.get("center_hold_kd", 4.0))
        self.swing_rail_guard_x = float(cfg.get("swing_rail_guard_x_m", 0.0))
        self.swing_top_brake_phi = float(cfg.get("swing_top_brake_phi_rad", 0.0))
        self.swing_top_brake_phid = float(cfg.get("swing_top_brake_phidot_rad_s", 0.0))
        self.swing_top_brake_ratio = float(cfg.get("swing_top_brake_ratio", 0.0))
        self.lqr_center_kx = float(cfg.get("lqr_center_kx", 0.0))
        self.lqr_center_kd = float(cfg.get("lqr_center_kd", 0.0))
        self.catch_phi = float(cfg.get("catch_phi_rad", self.CATCH_PHI))
        self.catch_phid = float(cfg.get("catch_phidot_rad_s", self.CATCH_PHID))
        self.release_phi = float(cfg.get("release_phi_rad", self.RELEASE_PHI))
        self.E_bottom = -2.0 * m * G * lc                # 真下静止のエネルギー
        self._m, self._lc, self._J = m, lc, J
        self.mode = "swingup"

    # ------------------------------------------------------------------
    def energy(self, phi: float, phid: float) -> float:
        return 0.5 * self._J * phid ** 2 + self._m * G * self._lc * (np.cos(phi) - 1.0)

    def __call__(self, s: dict) -> float:
        phi, phid = s["phi"], s["phidot"]
        x, xd = s["x"], s["xdot"]

        # ---- モード切替 (ヒステリシス) ----
        if self.mode == "swingup":
            if abs(phi) < self.catch_phi and abs(phid) < self.catch_phid:
                self.mode = "lqr"
        else:
            if abs(phi) > self.release_phi:
                self.mode = "swingup"

        if self.mode == "lqr":
            a = -float(self.K @ np.array([x, xd, phi, phid]))
            a += -self.lqr_center_kx * x - self.lqr_center_kd * xd
            return float(np.clip(a, -self.p.a_max, self.p.a_max))

        # ---- エネルギー法スイングアップ ----
        E = self.energy(phi, phid)
        pump = phid * np.cos(phi)
        if abs(E - self.E_bottom) < 0.02 * abs(self.E_bottom) and abs(phid) < 0.2:
            a = self.swing_sat                       # 静止デッドロック脱出キック
        else:
            a = self.k_energy * E * pump             # E<0 なので dE/dt >= 0
            if abs(phid) < self.deadlock_phidot and abs(a) < self.deadlock_accel:
                a = -self.deadlock_accel if phi >= 0.0 else self.deadlock_accel
            if (self.swing_top_brake_phi > 0.0
                    and abs(phi) < self.swing_top_brake_phi
                    and abs(phid) > self.swing_top_brake_phid
                    and abs(pump) > 1.0e-6):
                a = self.swing_top_brake_ratio * self.swing_sat * np.sign(pump)
        # レール中央維持
        a += -self.center_hold_kx * x - self.center_hold_kd * xd
        if self.swing_rail_guard_x > 0.0:
            if x > self.swing_rail_guard_x and a > 0.0:
                a = -self.swing_sat
            elif x < -self.swing_rail_guard_x and a < 0.0:
                a = self.swing_sat
        return float(np.clip(a, -self.swing_sat, self.swing_sat))
