# 開発計画

## 1. プロジェクトの目的

- ベルト駆動リニア 1 軸の制御ソフトを作る。
- 将来的にペンプロッタの X/Y/Z 軸に展開する。
- ホーミング処理、リミットスイッチ処理、移動制御を再利用可能にする。

## 2. ソフトウェア構成

```text
src/
  main.cpp
  StepDirDriver.cpp
  LimitSwitch.cpp
  Axis.cpp
  HomingController.cpp
  MotionController.cpp
  AppController.cpp

include/
  config.h
  StepDirDriver.h
  LimitSwitch.h
  Axis.h
  HomingController.h
  MotionController.h
  AppController.h
```

`main.cpp` はアプリケーション入口だけにし、実処理は各クラスに分離する。

## 3. 各クラスの責務

### StepDirDriver

- STEP/DIR/EN ピンを扱う。
- A4988/TMC2209 の STEP/DIR 駆動で共通利用する。

### LimitSwitch

- リミットスイッチ入力を扱う。
- activeLow、INPUT_PULLUP、デバウンスを担当する。

### Axis

- 1 軸分の現在位置、steps/mm、homed 状態、ソフトリミットを管理する。
- mm 単位の指令を step 数へ変換する。

### HomingController

- 二段階ホーミングを状態機械として実装する。
- GPIO を直接触らず、Axis と LimitSwitch の抽象化を使う。

### MotionController

- 通常移動を担当する。
- `moveToMm`、`moveRelativeMm`、速度指定移動、ソフトリミット確認を担当する。
- 将来、台形加減速を入れられるようにする。

### AppController

- アプリ全体の状態管理を担当する。
- Boot、NotHomed、Homing、Ready、Moving、Error を管理する。

## 4. 状態遷移

AppState:

```text
Boot
  -> NotHomed
  -> Homing
  -> Ready
  -> Moving
  -> Ready
  -> Error
```

HomingState:

```text
Idle
  -> SeekFast
  -> Backoff
  -> SeekSlow
  -> SetZero
  -> Done
  -> Error
```

## 5. ホーミング仕様

- 二段階ホーミングを行う。
- 最初は速めにリミット方向へ移動する。
- リミット ON で停止する。
- リミットが OFF になるまで戻る。
- `backoffMm` 戻ってもリミットが OFF にならなければ Error にする。
- 低速でもう一度リミットへ当てる。
- その位置を 0 mm にする。
- home 完了前は通常移動を禁止する。
- `maxTravelMm` 以内にリミットへ当たらなければ Error にする。

## 6. 安全設計

- home 完了前の通常移動は禁止。
- ソフトリミットを超える移動は禁止。
- 任意距離・任意速度のテスト移動も通常移動と同じ安全制約を使う。
- リミット ON 時は安全停止する。
- 通電中にモータ線を抜かない。
- GND 共通を必須とする。

## 7. 機械パラメータ

- GT2 ベルト。
- 20T プーリー。
- 1 回転 40 mm。
- NEMA17 1.8 deg。
- TMC2209 UARTで 1/16 microstepへ設定する。
- steps/mm = 80。
- `STEPS_PER_MM` は `MOTOR_FULL_STEPS_PER_REV * MICROSTEPS / PULLEY_TRAVEL_MM_PER_REV` から導出する。
- X_MIN_MM = 0.0。
- RAIL_LENGTH_MM = 100.0。
- CARRIAGE_LENGTH_MM = 40.0。
- END_MARGIN_MM = 5.0。
- X_MAX_TRAVEL_MM = 100.0 - 40.0 - 5.0 = 55.0。
- X_MAX_MM = 55.0 初期値。
- TEST_MOVE_MIN_SPEED_MM_S = 0.1。
- TEST_MOVE_MAX_SPEED_MM_S = 50.0。

## 8. 今後の実装計画

- [x] config.h へピン定義と機械定数を集約する。
- [x] StepDirDriver を実装する。
- [x] LimitSwitch を実装する。
- [x] Axis を実装する。
- [x] HomingController を実装する。
- [x] MotionController を実装する。
- [x] AppController を実装する。
- [x] main.cpp を薄くする。
- [x] 二段階ホーミングを検証する。
- [x] 10mm 移動テストを行う。
- [ ] 50mm 移動テストを行う。
- [ ] ソフトリミットを検証する。
- [x] 任意距離・任意速度のテスト移動コマンドを実装する。
- [ ] `m 10 5`、`m -5 2.5`、範囲外速度、home前拒否を検証する。
- [x] TMC2209 UARTで1/16 microstep設定を実装する。
- [ ] UART接続OK時に10mm指令が10mmになることを検証する。
- [x] 台形加減速プロファイルを実装する。
- [x] 一定周期STEPの direct profile を比較用に残す。
- [x] 加速度をSerialコマンドで変更できるようにする。
- [x] TMC2209の電流、stealthChop/spreadCycle、microstepを実行時に切り替えられるようにする。
- [x] README を更新する。
- [x] TMC2209 UART 対応を統合する。

## 9. 脱調原因切り分けフェーズ

60mm/s付近で位置が飛ぶ問題は、ベルトやプーリーのズレがない前提で、ロータが指令STEPに追従できていない脱調として扱う。
次の順で検証する。

- [ ] ベルトテンションを少し弱め、50mm/sが維持できるか確認する。
- [ ] `profile trap`、`a 100`、`m 50 60` で60mm/s・100mm/s2を確認する。
- [ ] 100mm/s2でNGなら `i 600`、`i 700`、`i 800` の順に電流を段階的に上げる。
- [ ] 電流を上げてもNGなら `mode spread` でspreadCycleを試す。
- [ ] 60mm/sがOKになったら `a 200`、`a 300`、`a 500` へ加速度を上げる。
- [ ] 速度境界を50、52、55、58、60、65mm/sで記録する。
- [ ] `microstep 8` を診断用に試し、STEP周波数や高周波側トルク低下の影響を切り分ける。
- [ ] レール、ベルトテンション、アイドラー、プーリー偏心、キャリッジ固定、可動配線の負荷を再確認する。
- [ ] 12Vで高速側が不足する場合だけ、24V化またはモータ変更を検討する。

記録する項目:

| 速度 | 加速度 | 電流 | モード | microstep | 結果 |
| --- | --- | --- | --- | --- | --- |
| 50 | 300 | 500mA | stealthChop | 1/16 | OK/NG |
| 60 | 100 | 500mA | stealthChop | 1/16 | OK/NG |
| 60 | 100 | 600mA | stealthChop | 1/16 | OK/NG |
| 60 | 100 | 600mA | spreadCycle | 1/16 | OK/NG |
| 60 | 100 | 600mA | spreadCycle | 1/8 | OK/NG |

## 10. 設計方針

- main.cpp に処理を集めない。
- GPIO 直接操作を各クラスに分離する。
- HomingController はピン番号を知らない。
- Axis は mm と step の変換を担当する。
- 各クラスを X 軸/Y 軸/Z 軸で再利用できるようにする。
- TMC2209 化しても StepDirDriver より上の設計は変えない。

## 11. 今回はやらないこと

- G-code 解釈。
- XY 同時制御。
- ペン上下 Z 軸制御。
- 24V化やモータ変更。

## 12. Sim2real 同定計画

目的:

- 実機で倒立しない原因を、実機ログに合う MuJoCo/sim パラメータへ修正して切り分ける。
- まず「実機で失敗する挙動を再現できる sim」を作り、その sim で古典制御条件を再調整する。
- 速度/加速度は、AS5600 脱調探索で確認済みの `300 mm/s`, `3000 mm/s^2` を初期上限として扱う。

実行手順:

- [x] 実機 `BALANCE_STATUS` をCSVとして保存するツールを作る。
- [x] 取得CSVには `t_s`, `mode`, `x_m`, `xdot_mps`, `phi_rad`, `phidot_rad_s`, `cmd_x_m`, `cmd_v_mps`, `accel_mps2`, `cmd_lag_m` を入れる。
- [x] 実機ログ取得時は、原点復帰、中央移動、RAWANGLEによる `balzero`、`balstart`、終了時 `balstop` まで自動化する。
- [x] 実機ログを `reports/` 配下にCSVとMarkdownで保存する。
- [x] 取得した実機ログの特徴量を計算する。
  - 最小 `abs(phi_rad)`
  - 最大 `abs(x_m)`
  - 最大/平均 `abs(cmd_lag_m)`
  - LQR突入回数
  - 推定周期
- [x] 同じ古典制御器で sim を走らせ、実機特徴量に近いパラメータを探索する。
  - `hinge_damping_nms`
  - `hinge_frictionloss_nm`
  - `servo.kp_n_per_m`
  - `servo.kv_ns_per_m`
  - 必要なら `rod_mass_kg`, `pole_len_m`
- [x] `cmd_lag_m` を再現できるように、simのステッパモデルへファーム実装相当のステップ発行遅れ/量子化を追加する。
- [x] ステップ発行遅れ込みで再度パラメータ探索する。
- [x] もっとも近いパラメータを候補として `sim_config/belt_cartpole.json` に反映する。
- [x] 候補パラメータで MuJoCo 古典制御を再実行し、倒立可否を確認する。
- [x] 結果を `reports/real_balance_attempt_20260617.md` または新規レポートに追記する。

判断基準:

- まず実機失敗ログの `min_abs_phi_rad` と `max_abs_x_m` を sim が近く再現できること。
- 再現できた sim で倒立する条件が見つかった場合だけ、実機へ反映する。
- 実機へ反映するときは、AS5600 脱調探索済みの速度/加速度範囲を超えない。
- 範囲を超える必要が出た場合は、先に AS5600 脱調探索を追加で実行する。

2026-06-17 実行結果:

- 実機ログ: `reports/real_balance_log_20260617_164045.csv`
- 実機ログレポート: `reports/real_balance_log_20260617_164045.md`
- 初回fit結果: `reports/sim_param_fit_20260617_164045.csv`
- 実機ログは `min_abs_phi_rad=0.0015`, `lqr_count=16`, `max_abs_x_m=0.15377`, `max_abs_cmd_lag_m=0.04827`。
- 粗い摩擦/サーボ剛性探索の最良simは `sim_min_abs_phi_rad=1.4646`, `sim_lqr_count=0`, `sim_max_abs_cmd_lag_m=0.00037` で、実機を再現できなかった。
- 次は摩擦係数ではなく、ファームの連続指令位置と発行済みステップ位置の差である `cmd_lag_m` をsimへ入れる。

2026-06-17 追加実行結果:

- simのステッパモデルを `cmd_pos` と `issued_pos` に分け、`cmd_lag = cmd_pos - issued_pos` を状態へ追加した。
- 最初の探索 `reports/sim_param_fit_step_lag_20260617_164045.csv` では最良 `cost=8.318`、`sim_max_abs_cmd_lag_m=0.01589`、`sim_lqr_count=0` で、まだ実機遅れを過小評価した。
- 低速側再探索 `reports/sim_param_fit_step_lag_lowv_20260617_164045.csv` では最良 `cost=2.200`、`issued_v_max=0.14`, `hinge_damping=0.0004`, `hinge_frictionloss=0.001`, `servo_kp=1000`, `servo_kv=10` だったが、通常実行では脱調した。
- 脱調しない上位候補として `issued_v_max=0.14`, `hinge_damping=0.0004`, `hinge_frictionloss=0.001`, `servo_kp=1000`, `servo_kv=20` を採用した。
- この候補は `sim_mean_abs_cmd_lag_m=0.01698` で、実機の `mean_abs_cmd_lag_m=0.01608` に近い。`sim_max_abs_cmd_lag_m=0.07711` は実機の `0.04827` より大きい。
- 20秒 headless MuJoCo 実行は脱調なしで完走したが、保持倒立は未達。結果は `upright=0.0s`, `final phi=+0.576 rad`, `x=-0.005 m`。
- 現時点では「実機ログへ寄せたsim」には前進したが、「古典制御で安定保持できる実機条件」には未達。次は保持失敗時の LQR ゲイン/キャッチ条件を、同定済み遅れモデル上で再設計する。

2026-06-17 再リトライ結果:

- `CartPoleBalanceController` の制御角度を AS5600 `rawDegrees` に統一した。`balzero` は元から `rawDegrees` を使っていたため、制御開始時だけ `angleDegrees` を使う不整合を解消した。
- MuJoCo は現行古典制御で倒立可能。`a_max=3.0` では `upright=18.8s`、`a_max=4.0`, `catch_phi=0.50` では `upright=6.3s`。
- 実機では `angle_direction=-1` は `min_abs_phi_rad=2.9006` で棄却し、`angle_direction=1` に戻した。
- 実機 `a_max=3.0` は `min_abs_phi_rad` が約 `1.26..1.29 rad` で、LQR突入なし。
- 実機 `a_max=4.0`, `catch_phi=0.35` は `reports/real_balance_log_20260617_200028.csv` で `min_abs_phi_rad=0.5473` まで改善したが、`catch_phi=0.35` が狭く LQR突入なし。
- 実機 `a_max=4.0`, `catch_phi=0.50` は `reports/real_balance_log_20260617_200223.csv` で `min_abs_phi_rad=0.9170`、LQR突入なし。20秒試行では最良パスは再現しなかった。
- 現在の設定は `a_max=4.0`, `v_max=0.3`, `catch_phi=0.50`, `swing_rail_guard_x=0.10`, `rawDegrees` 基準。
- まだ実機倒立は未達。次は、LQRゲイン単体ではなく、スイングアップの位相則または安全な速度/加速度上限の追加探索を優先する。

2026-06-17 sim/実機差分の再同定:

- 追加した内容:
  - `sim/belt_cartpole/observer.py` を追加し、AS5600相当の角度量子化、ファーム同等の角速度差分/LPF、ステッパ発行済み位置を古典制御器の観測値として使えるようにした。
  - `sim/belt_cartpole/run.py --firmware-observer` で、理想状態ではなく実機相当観測で MuJoCo を回せるようにした。
  - `tools/fit_balance_sim_params.py` を実機相当観測対応にし、`--match-initial-row` と `--k-energy` 探索を追加した。
- 実機ログ `reports/real_balance_log_20260617_200223.csv` に対する最良候補:
  - `hinge_damping_nms=0.00015`
  - `hinge_frictionloss_nm=0.0005`
  - `issued_v_max_mps=0.22`
  - `servo.kv_ns_per_m=60`
  - `rod_mass_kg=0.045`
  - `k_energy=420`
- 反映:
  - `sim_config/belt_cartpole.json` に上記の sim 同定値を反映。
  - `tools/generate_belt_cartpole_config.py` を実行し、`include/generated/BeltCartpoleConfig.h` を更新。
- 検証:
  - `run.py --firmware-observer` の20秒 MuJoCo 実行は `upright=0.0s`, final `phi=-1.506 rad`, `x=+0.022 m`。
  - `pio run` は成功。
- 解釈:
  - 従来の理想観測simが実機より楽観的だったため、実機で倒立しない条件を sim でも再現する方向へ寄せた。
  - まだ 200028 の一回だけ良かった `min_abs_phi_rad=0.5473` は再現できていない。200223 の再現性あるログでは `min_abs_phi_rad=0.9170` に対し、同定simは `1.2272` でやや保守的。
  - 次はこの実機相当観測sim上で、スイングアップ位相則またはキャッチ条件を再設計してから実機へ戻す。

2026-06-17 firmware-observer 上の古典制御再調整:

- `v_max=0.30`, `a_max=4.0` のままでは、キャッチ条件、中央保持、制御周期を調整しても倒立近傍に安定して入らなかった。
- `v_max=0.45`, `a_max=4.0` では、実機相当観測simで倒立近傍に入る候補が出たが、60秒安定保持は未達。
- 現在の暫定候補:
  - `v_max_mps=0.45`
  - `catch_phi_rad=0.25`
  - `catch_phidot_rad_s=2.5`
  - `release_phi_rad=1.0`
  - `lqr_gains=[-11.180339885, 0.0, -147.19697301, -21.884681952]`
  - `lqr_center_kx=0.0`, `lqr_center_kd=5.0`
- 検証:
  - `--firmware-observer` 60秒sim: `upright=5.6s`, final `phi=-1.193 rad`。安定保持ではない。
  - `pio run`: 成功。
  - `pio run -t upload`: 成功。
- 実機ログ取得:
  - `tools/collect_balance_log.py` の権限付き実行は使用量制限により承認レビューで拒否された。
  - このため、このステップでは新規実機CSVは取得できていない。
  - ユーザー指示後に 8秒の短時間 `balstart` ログ取得を再試行したが、同じ使用量制限で拒否された。Codexからの実機動作は実行されていない。
- 次回:
  - まず `450 mm/s`, `4000 mm/s^2` の短い脱調確認または短時間 `balstart` ログを取る。
  - `v_max=0.45` は未検証なので、長時間倒立試験より先に安全確認を行う。

2026-06-17 脱調後ロールバック:

- ユーザーから `v_max=0.45` 暫定候補で脱調報告あり。
- `v_max=0.45` は実機使用不可として棄却。
- 設定を安全側へ戻した:
  - `v_max_mps=0.30`
  - `a_max_mps2=4.0`
  - `catch_phi_rad=0.50`
  - `catch_phidot_rad_s=6.0`
  - `release_phi_rad=0.80`
  - LQRゲインを前回値へ戻し、`lqr_center_kx=20.0`, `lqr_center_kd=5.0`
- `tools/generate_belt_cartpole_config.py` 実行済み。
- `pio run`: 成功。
- `pio run -t upload`: 成功。安全側ファームを実機へ再書き込み済み。

2026-06-17 安全速度内のスイングアップ則変更:

- 方針:
  - `v_max=0.30` は維持する。
  - 速度上限を上げず、スイングアップ則だけを変更する。
- 実装:
  - スイングアップ中に上付近で角速度が大きいときだけ、エネルギーを抜くトップブレーキを追加。
  - `sim/belt_cartpole/classic.py`
  - `src/CartPoleBalanceController.cpp`
  - `tools/generate_belt_cartpole_config.py`
- 採用設定:
  - `swing_top_brake_phi_rad=1.3`
  - `swing_top_brake_phidot_rad_s=1.5`
  - `swing_top_brake_ratio=0.6`
  - `v_max_mps=0.30`, `a_max_mps2=4.0` は維持。
- sim検証:
  - `--firmware-observer` 30秒: `upright=1.4s`, final `phi=-0.101 rad`, `x=+0.113 m`
  - 60秒単独評価: `upright_s=3.68`, `lqr_count=510`, `stall_count=0`
- ビルド/アップロード:
  - `tools/generate_belt_cartpole_config.py`: 成功。
  - `pio run`: 成功。
  - `pio run -t upload`: 成功。
- 状態:
  - 安全速度内トップブレーキ版ファームを実機へアップロード済み。
  - このステップでは新規実機ログは未取得。
