# Belt CartPole — NEMA17 + GT2 + 400mm リニアレール倒立振子 (MuJoCo)

1軸ベルト駆動キャリッジにヒンジ付きの棒をぶら下げ、**スイングアップ + 倒立維持**を
**古典制御 (エネルギー法 + LQR)** と **強化学習 (SAC)** の両方で行うシミュレーションです。
sim2real を意識し、制御入力は理想的な力ではなく
**速度制限・加速度制限・位置制限・トルク(推力)制限つきのステッパ駆動モデル**を必ず通ります。

## セットアップ

```bash
pip install mujoco scipy numpy            # 古典制御のみならこれで十分
pip install gymnasium "stable-baselines3>=2.0"   # RL を使う場合
```

## 使い方

```bash
# 古典制御 (エネルギー法スイングアップ → LQR)
python run.py --controller classic

# RL の学習 (~40万ステップ, CPU で数十分目安)
python train_rl.py --steps 400000 --envs 8

# 学習済み方策で実行
python run.py --controller rl --rl-model sac_belt_cartpole.zip

# GUI なし評価
python run.py --controller classic --headless --duration 10
```

ビューア中: `R` でリセット、`Tab` で classic ↔ rl 切替 (RL モデル読込時)。

## ファイル構成

| ファイル | 内容 |
|---|---|
| `../../sim_config/belt_cartpole.json` | SIM/実機共通の設定値 (プーリ径, v/a 上限, トルク, 質量, 制御周期, 制御ゲイン…) |
| `params.py` | 共通JSONを読み込んで Python 用 `Params` に変換 |
| `model_xml.py` | params から MJCF を生成 (LQR の線形化と物理モデルが常に一致) |
| `stepper.py` | ステッパドライバモデル: 加速度指令→台形プロファイル積分, 端ブレーキ, トルク-速度特性 |
| `env.py` | コアシミュレータ + Gymnasium 環境 (ノイズ / レイテンシ / ドメインランダマイゼーション / 脱調終了) |
| `classic.py` | エネルギー法スイングアップ + LQR (ヒステリシス切替) |
| `train_rl.py` | SB3 SAC 学習スクリプト |
| `run.py` | 可視化・制御切替・ヘッドレス評価 |

## モデル化のポイント (sim2real)

**制御入力 = 指令加速度。** RL も LQR もスイングアップも、出力は台車の
「指令加速度」1 自由度です。実機ではこれをそのままステップパルスの
加減速プロファイル (例: TMC2209 + ステップ生成ファーム) に対応させられます。

`stepper.py` が実機ファーム相当の処理を行います:

1. `|a| <= a_max` にクリップし指令速度を積分、`|v| <= v_max` にクリップ
2. 指令位置を積分、レール可動域 (±0.16 m) でクランプ
3. **端ブレーキ**: 最大減速度でも止まれない速度なら強制フルブレーキ (ソフトリミット相当)
4. 指令位置は MuJoCo の高剛性位置サーボ (kp=6000) で追従。ただし
   **推力上限 = モータトルク / プーリ半径** を超える負荷では追従できない
5. 推力上限は**トルク-速度特性**で速度に応じて直線的に低下
6. 指令位置と実位置が 4 mm 以上乖離したら**脱調**と判定
   (RL では即終了 + 罰 → 実機で危険な方策を学習段階で排除)

その他:

- **ロータ慣性の反映**: NEMA17 のロータ慣性 (~57 g·cm²) はGT2-20T (r≈6.4 mm) では
  並進換算で **約 0.14 kg** に相当し無視できないため、台車質量に加算しています。
- **RL の堅牢化**: 行動レイテンシ 1 制御周期、観測ノイズ、振子質量 ±15% /
  摩擦 ×0.5–2 / サーボ剛性 ±20% のドメインランダマイゼーション。
- 制御周期 50 Hz / 物理 2 kHz。実機の制御ループ周波数に合わせて `ctrl_hz` を変更可。

## 古典制御の中身

倒立からの角度 `phi` に対しエネルギー `E = ½Jφ̇² + mglc(cosφ − 1)`(倒立静止で 0)を定義し、

```
a = k_e · E · φ̇ · cosφ   (dE/dt = −m·lc·a·φ̇·cosφ ≥ 0 を保証)
```

で汲み上げ、`|phi| < 0.3 rad` かつ `|φ̇| < 4 rad/s` で LQR に引き込みます
(外れたらヒステリシス付きでスイングアップに復帰)。LQR は
`ẍ = a, φ̈ = α(gφ − a) − (b/J)φ̇` (α = m·lc/J) を `params.py` の値から自動線形化して
CARE を解くため、棒の長さや質量を変えても再調整不要です。

## 実機パラメータの合わせ込み

`../../sim_config/belt_cartpole.json` で最低限合わせるべき項目:

- `v_max` / `a_max`: 実機ファームの設定値そのまま
- `torque_lowspeed` / `torque_min` / `v_knee`: モータのトルク-速度曲線
  (電源電圧・ドライバ電流設定に依存。データシートのプルアウトトルク参照)
- `pole_len` / `rod_mass` / `tip_mass`: 実測
- `hinge_damping` / `hinge_frictionloss`: 棒を手で振って減衰の様子を合わせる
- `x_lim`: キャリッジ可動域の実測値

実機ファームは PlatformIO の pre-build script (`tools/generate_belt_cartpole_config.py`) で
同じJSONから `include/generated/BeltCartpoleConfig.h` を生成して読みます。

## 既知の簡略化

- マイクロステップの離散性 (実機既定は 1 ステップ ≈ 0.025 mm @ 8 microstep) は連続近似
- ベルトの伸び・バックラッシュは未モデル化 (必要なら slide ジョイントを
  ばね付き 2 自由度に分割して表現可能)
- 脱調は「乖離検出」のみで、脱調後の磁極引き込み挙動は再現していない
