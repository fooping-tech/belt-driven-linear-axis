"""params.Params から MJCF (MuJoCo XML) を生成する。

XML を別ファイルで持たずコードで生成することで、LQR の線形化に使う
質量・長さと物理モデルが必ず一致するようにしている。

座標系 / 角度の定義:
  - スライド軸: x。qpos[0] = キャリッジ位置 [m]
  - ヒンジ軸: y。qpos[1] = theta。theta = 0 で棒は真下 (吊り下げ)、
    theta = ±pi で倒立。倒立からの角度 phi = wrap(theta - pi) を制御で使う。
  - キャリッジ +x 加速時、棒先端は (phi 表現で) 標準的なカートポール
    の式 J*phi'' = m*g*lc*sin(phi) - m*lc*a*cos(phi) - b*phi' に従う。
"""
from params import Params, DEFAULT


def build_model_xml(p: Params = DEFAULT) -> str:
    rail_half = p.x_lim + 0.045          # 見た目用レール長
    cart_total = p.cart_mass + p.reflected_rotor_mass
    return f"""
<mujoco model="belt_cartpole">
  <option timestep="{p.timestep}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <map znear="0.01"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.35 0.45 0.6" rgb2="0.1 0.12 0.18" width="64" height="64"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.18 0.2 0.22" rgb2="0.24 0.26 0.28" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.1"/>
    <material name="rail"  rgba="0.55 0.57 0.62 1"/>
    <material name="cart"  rgba="0.85 0.3 0.2 1"/>
    <material name="pole"  rgba="0.2 0.5 0.9 1"/>
    <material name="tip"   rgba="0.95 0.85 0.2 1"/>
  </asset>

  <worldbody>
    <light pos="0 -1 2" dir="0 0.4 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="2 2 0.05" material="grid"/>

    <!-- 400 mm リニアレール (見た目のみ) -->
    <geom name="rail" type="box" size="{rail_half:.4f} 0.012 0.006"
          pos="0 0 0.55" material="rail"/>
    <geom name="endL" type="box" size="0.006 0.015 0.02" pos="{-rail_half:.4f} 0 0.55" material="rail"/>
    <geom name="endR" type="box" size="0.006 0.015 0.02" pos="{rail_half:.4f} 0 0.55" material="rail"/>

    <body name="cart" pos="0 0 0.55">
      <!-- ロータ慣性をベルト換算質量として含める (sim2real) -->
      <inertial pos="0 0 0" mass="{cart_total:.4f}" diaginertia="1e-5 1e-5 1e-5"/>
      <joint name="slide" type="slide" axis="1 0 0"
             range="{-p.x_lim} {p.x_lim}" limited="true"
             solreflimit="0.005 1" damping="0.0"/>
      <geom name="cart_geom" type="box" size="0.022 0.022 0.012" material="cart"
            contype="0" conaffinity="0"/>

      <body name="pole" pos="0 0.03 0">
        <joint name="hinge" type="hinge" axis="0 1 0" limited="false"
               damping="{p.hinge_damping}" frictionloss="{p.hinge_frictionloss}"/>
        <geom name="rod" type="capsule" fromto="0 0 0 0 0 {-p.pole_len}"
              size="0.005" mass="{p.rod_mass}" material="pole"
              contype="0" conaffinity="0"/>
        <geom name="tip" type="sphere" pos="0 0 {-p.pole_len}" size="0.009"
              mass="{p.tip_mass}" material="tip" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- ステッパの「指令位置に剛に追従、ただし推力上限あり」を位置サーボで近似。
         forcerange は env 側で速度依存に動的更新される (トルク-速度特性)。 -->
    <position name="motor" joint="slide"
              kp="{p.servo_kp}" kv="{p.servo_kv}"
              ctrlrange="{-(p.x_lim + p.servo_kv / p.servo_kp * p.v_max + 0.005):.4f} {(p.x_lim + p.servo_kv / p.servo_kp * p.v_max + 0.005):.4f}"
              forcerange="{-p.force_lowspeed:.2f} {p.force_lowspeed:.2f}"/>
  </actuator>
</mujoco>
"""


if __name__ == "__main__":
    print(build_model_xml())
