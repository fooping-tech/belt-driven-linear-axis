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
- `backoffMm` だけ戻る。
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
- [x] README を更新する。
- [ ] TMC2209 UART 対応を次フェーズで検討する。

## 9. 設計方針

- main.cpp に処理を集めない。
- GPIO 直接操作を各クラスに分離する。
- HomingController はピン番号を知らない。
- Axis は mm と step の変換を担当する。
- 各クラスを X 軸/Y 軸/Z 軸で再利用できるようにする。
- TMC2209 化しても StepDirDriver より上の設計は変えない。

## 10. 今回はやらないこと

- G-code 解釈。
- XY 同時制御。
- 加速度制御の本格実装。
- TMC2209 UART の本格統合。
- ペン上下 Z 軸制御。
