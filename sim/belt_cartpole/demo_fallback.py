"""MuJoCo なし環境向けの検証デモ。

MJCF が表現している物理 (位置サーボ力で駆動される台車 + ヒンジ振子の連成系、
推力上限・トルク-速度特性・ヒンジ粘性/クーロン摩擦) を同じ式・同じ dt で
数値積分し、stepper.py / classic.py を「本番のコードそのまま」通して
スイングアップ → 倒立維持を実行する。

連成運動方程式 (phi: 倒立からの角度, COM = (x + lc sin phi, lc cos phi)):
    [ M_t        m lc cos]) [x'' ]   [ F + m lc phi'^2 sin       ]
    [ m lc cos   J       ]  [phi''] = [ m g lc sin - b phi' - tau_f ]
F = clip(kp (x_cmd - x) - kv x', +-F_avail(v))   <- MuJoCo の position actuator と同じ

出力: demo_timeseries.png, demo_anim.gif
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation

from params import DEFAULT as p
from stepper import StepperDriver
from classic import HybridController

G = 9.81


class FallbackSim:
    """BeltCartPoleSim と同じインターフェイス (state()/set_accel()/control_step())"""

    def __init__(self):
        self.stepper = StepperDriver(p)
        self.m = p.total_pole_mass
        self.lc = p.pole_com
        self.J = p.pole_inertia
        self.Mt = p.cart_mass + p.reflected_rotor_mass + self.m
        self.b = p.hinge_damping
        self.tau_c = p.hinge_frictionloss
        self.reset()

    def reset(self, phi0=np.pi - 0.03):
        self.x = 0.0
        self.xd = 0.0
        self.phi = phi0          # ほぼ真下 (倒立から pi)
        self.phid = 0.0
        self.stepper.reset(0.0)
        self.accel_cmd = 0.0
        self.F = 0.0
        self.F_avail = self.stepper.available_force()
        self.stalled = False
        return self.state()

    def set_accel(self, a):
        self.accel_cmd = float(a)

    def _substep(self, dt):
        x_cmd = self.stepper.step(self.accel_cmd, dt)
        self.F_avail = self.stepper.available_force()
        # 位置+速度追従サーボ (MuJoCo 側は ctrl への速度フィードフォワードで等価)
        F = (p.servo_kp * (x_cmd - self.x)
             + p.servo_kv * (self.stepper.cmd_vel - self.xd))
        F = float(np.clip(F, -self.F_avail, self.F_avail))
        self.F = F

        m, lc, J = self.m, self.lc, self.J
        c, s = np.cos(self.phi), np.sin(self.phi)
        tau_f = self.tau_c * np.tanh(self.phid / 0.01)        # クーロン摩擦の平滑化
        M = np.array([[self.Mt, m * lc * c],
                      [m * lc * c, J]])
        rhs = np.array([F + m * lc * self.phid ** 2 * s,
                        m * G * lc * s - self.b * self.phid - tau_f])
        xdd, phidd = np.linalg.solve(M, rhs)

        # 半陰的オイラー (MuJoCo と同じ dt=0.5 ms)
        self.xd += xdd * dt
        self.phid += phidd * dt
        self.x += self.xd * dt
        self.phi += self.phid * dt
        # レール物理リミット (柔らかい当て: 通常は端ブレーキで到達しない)
        if abs(self.x) > p.x_lim:
            self.x = float(np.clip(self.x, -p.x_lim, p.x_lim))
            self.xd = 0.0

    def control_step(self):
        for _ in range(p.substeps):
            self._substep(p.timestep)
        if abs(self.stepper.cmd_pos - self.x) > p.stall_threshold:
            self.stalled = True
        return self.state()

    def state(self):
        return {"x": self.x, "xdot": self.xd,
                "phi": (self.phi + np.pi) % (2 * np.pi) - np.pi,
                "phidot": self.phid,
                "cmd_pos": self.stepper.cmd_pos,
                "cmd_vel": self.stepper.cmd_vel,
                "stalled": self.stalled}


# ---------------------------------------------------------------- run
def main():
    sim = FallbackSim()
    ctrl = HybridController(p)
    s = sim.reset()

    T = 12.0
    n = int(T * p.ctrl_hz)
    log = {k: [] for k in ["t", "x", "cmd", "phi", "vcmd", "a", "mode", "F", "Favail"]}
    for i in range(n):
        a = ctrl(s)
        sim.set_accel(a)
        s = sim.control_step()
        log["t"].append(i * p.ctrl_dt)
        log["x"].append(s["x"]); log["cmd"].append(s["cmd_pos"])
        log["phi"].append(s["phi"]); log["vcmd"].append(s["cmd_vel"])
        log["a"].append(a); log["mode"].append(1 if ctrl.mode == "lqr" else 0)
        log["F"].append(sim.F); log["Favail"].append(sim.F_avail)

    t = np.array(log["t"]); phi = np.array(log["phi"])
    upright = np.sum(np.abs(phi) < 0.2) * p.ctrl_dt
    catch_i = next((i for i, m in enumerate(log["mode"]) if m), None)
    print(f"swing-up -> LQR 切替: t = {t[catch_i]:.2f} s" if catch_i is not None
          else "LQR 未到達")
    print(f"倒立維持時間: {upright:.1f} / {T:.0f} s   "
          f"最終 phi = {phi[-1]:+.4f} rad,  x = {log['x'][-1]:+.4f} m,  "
          f"stall = {s['stalled']}")

    # ---------------- time series ----------------
    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(t, np.degrees(np.abs(phi)), lw=1.2)
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].set_ylabel("|phi| [deg]\n(0 = upright)")
    if catch_i is not None:
        for a_ in ax:
            a_.axvline(t[catch_i], color="g", ls="--", lw=1, alpha=0.7)
        ax[0].text(t[catch_i] + 0.05, 150, "swing-up -> LQR", color="g")

    ax[1].plot(t, log["x"], label="actual x")
    ax[1].plot(t, log["cmd"], "--", label="stepper cmd_pos", lw=1)
    for y in (p.x_lim, -p.x_lim):
        ax[1].axhline(y, color="r", ls=":", lw=1)
    ax[1].set_ylabel("cart [m]"); ax[1].legend(loc="upper right", fontsize=8)

    ax[2].plot(t, log["vcmd"], label="cmd_vel")
    for y in (p.v_max, -p.v_max):
        ax[2].axhline(y, color="r", ls=":", lw=1)
    ax[2].set_ylabel("v_cmd [m/s]\n(limit ±0.7)")

    ax[3].plot(t, log["a"], lw=1, label="accel cmd")
    for y in (p.a_max, -p.a_max):
        ax[3].axhline(y, color="r", ls=":", lw=1)
    ax[3].set_ylabel("a_cmd [m/s²]\n(limit ±15)")
    ax[3].set_xlabel("time [s]")
    fig.suptitle("Belt CartPole — classic controller through stepper model "
                 "(fallback physics, same EOM as MJCF)")
    fig.tight_layout()
    fig.savefig("demo_timeseries.png", dpi=110)

    # ---------------- animation ----------------
    fps = 25
    skip = max(1, int(p.ctrl_hz / fps))
    frames = range(0, n, skip)
    L = p.pole_len

    figa, axa = plt.subplots(figsize=(6.4, 4.6))
    axa.set_xlim(-0.33, 0.33); axa.set_ylim(-0.36, 0.40)
    axa.set_aspect("equal"); axa.set_xticks([]); axa.set_yticks([])
    figa.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01)
    axa.axhline(0, color="0.6", lw=4, zorder=0)                       # レール
    for xl in (p.x_lim + 0.025, -p.x_lim - 0.025):
        axa.plot([xl, xl], [-0.03, 0.03], color="0.4", lw=6)          # エンド
    cart_w, cart_h = 0.044, 0.024
    cart = plt.Rectangle((0, 0), cart_w, cart_h, fc="#d94f33", zorder=3)
    axa.add_patch(cart)
    pole, = axa.plot([], [], lw=3.5, color="#3380e6", zorder=2)
    tip, = axa.plot([], [], "o", ms=8, color="#f2d335", zorder=4)
    ghost, = axa.plot([], [], "|", ms=14, color="g", zorder=1)        # 指令位置
    txt = axa.text(0.02, 0.95, "", transform=axa.transAxes, va="top",
                   fontsize=9, family="monospace")

    X = np.array(log["x"]); PH = np.array(log["phi"]); MD = log["mode"]

    def draw(k):
        i = list(frames)[k]
        x, ph = X[i], PH[i]
        tipx, tipz = x + L * np.sin(ph), L * np.cos(ph)
        cart.set_xy((x - cart_w / 2, -cart_h / 2))
        pole.set_data([x, tipx], [0, tipz])
        tip.set_data([tipx], [tipz])
        ghost.set_data([log["cmd"][i]], [0.0])
        txt.set_text(f"t={t[i]:4.1f}s  mode={'LQR     ' if MD[i] else 'swing-up'}"
                     f"  phi={np.degrees(abs(ph)):5.1f} deg")
        return cart, pole, tip, ghost, txt

    ani = animation.FuncAnimation(figa, draw, frames=len(list(frames)),
                                  interval=1000 / fps, blit=True)
    ani.save("demo_anim.gif", writer=animation.PillowWriter(fps=fps))
    print("saved: demo_timeseries.png, demo_anim.gif")


if __name__ == "__main__":
    main()
