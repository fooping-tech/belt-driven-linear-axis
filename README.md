# Belt Driven Linear Axis

M5Atom S3 と A4988 STEP/DIR ドライバで、GT2 ベルト駆動の 1 軸リニア機構を動かす PlatformIO プロジェクトです。
起動直後は `NotHomed` になり、ホーミング完了後だけ通常移動できます。

## ソフトウェア構成

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

`main.cpp` は `app.begin()` と `app.update()` だけを呼びます。
GPIO 操作は `StepDirDriver` と `LimitSwitch` に閉じ込め、ホーミングや移動制御は GPIO ピン番号を知らない構造にしています。

## 各クラスの責務

- `StepDirDriver`: STEP/DIR/EN ピンを扱います。A4988 でも TMC2209 でも STEP/DIR 駆動なら上位層を変えずに使い回せます。
- `LimitSwitch`: active-low、`INPUT_PULLUP`、デバウンス付きのリミット入力を扱います。
- `Axis`: 位置、`steps/mm`、homed 状態、ソフトリミットを管理します。
- `HomingController`: 二段階ホーミングを状態機械として実行します。GPIO は直接触りません。
- `MotionController`: ホーミング後の通常移動とソフトリミット確認を担当します。
- `AppController`: アプリ全体の状態、ボタン、Serial コマンド、画面表示を担当します。

## 配線

```text
ATOM S3                         A4988

3.3V  ------------------------  VDD
GND   ------------------------  GND
G5    ------------------------  STEP
G6    ------------------------  DIR

                                ENABLE ---- GND
                                SLEEP  ---- 3.3V
                                RESET  ---- 3.3V

12V電源 + --------------------  VBB / VMOT
12V電源 - --------------------  GND

モータ コイルA ---------------  OUT1A / OUT1B
モータ コイルB ---------------  OUT2A / OUT2B
```

| ATOM S3 | A4988 |
| --- | --- |
| G5 / GPIO5 | STEP |
| G6 / GPIO6 | DIR |
| GND | GND |

ATOM S3 と A4988 の GND は必ず共通にしてください。
通電中にモータ線を抜かないでください。

この個体では `include/config.h` の `DIR_INVERTED = true` で、`HOMING_DIRECTION = -1` が X-min リミットへ向かう方向になります。
ホーミング時に逆方向へ動く場合は、座標系の `HOMING_DIRECTION` ではなく `DIR_INVERTED` を切り替えてください。

## リミットスイッチ

標準設定では X-min リミットスイッチを `GPIO8` に接続します。
`include/config.h` の `PIN_LIMIT_X_MIN` で変更できます。

```text
ATOM S3 GPIO8 ---- リミットスイッチ ---- GND
```

入力は active-low + `INPUT_PULLUP` です。
スイッチが押されると GPIO が LOW になり、`LimitSwitch` が 30 ms デバウンスして ON と判定します。

## 状態遷移

App:

```text
Boot
  -> NotHomed
  -> Homing
  -> Ready
  -> Moving
  -> Ready
  -> Error
```

Homing:

```text
Idle
  -> SeekFast
  -> Backoff
  -> SeekSlow
  -> SetZero
  -> Done
  -> Error
```

## ホーミング

長押し、または Serial の `h` でホーミングを開始します。

1. `HOMING_FAST_MM_S` でリミット方向へ移動します。
2. リミット ON で停止します。
3. `HOMING_BACKOFF_MM` だけ反対方向へ戻ります。
4. `HOMING_SLOW_MM_S` でもう一度リミット方向へ移動します。
5. リミット ON 位置を `X=0.00 mm` にします。
6. `Ready` になり、通常移動が許可されます。

`HOMING_MAX_TRAVEL_MM` 以内にリミットへ当たらない場合、または backoff 後もスイッチが離れない場合は `Error` になります。

実機確認では次の状態遷移を Serial ログで確認しています。

```text
Homing state: Backoff ... limit=ON
Homing state: SeekSlow ... limit=OFF
Homing state: SetZero ... limit=ON
Homing state: Done pos=0.00mm steps=0 limit=ON
```

## ソフトリミット

`Axis` は `X_MIN_MM` から `X_MAX_MM` までを移動可能範囲として管理します。
`X_MAX_MM` はレール長、キャリッジ長、終端マージンから計算します。

```text
RAIL_LENGTH_MM = 100.0
CARRIAGE_LENGTH_MM = 40.0
END_MARGIN_MM = 5.0
X_MAX_TRAVEL_MM = 100.0 - 40.0 - 5.0 = 55.0
X_MIN_MM = 0.0
X_MAX_MM = 55.0
```

home 完了前の通常移動は禁止です。
home 完了後も、ソフトリミットを超える `10mm` / `50mm` 移動は拒否されます。
通常移動中にリミット方向へ進んでリミット ON を検出した場合は安全停止して `Error` になります。

## 操作

- 長押し: homing 開始
- 短押し: `Ready` のとき +10 mm 移動
- 移動中の短押し: 停止

Serial コマンド:

| コマンド | 動作 |
| --- | --- |
| `h` | homing 開始 |
| `1` | +10 mm 移動 |
| `5` | +50 mm 移動 |
| `b` | -10 mm 移動 |
| `s` | status 表示 |

10 mm 往復テストは `h` の後に `1` と `b` を送ります。
50 mm テストは `h` の後に `5` を送ります。
現在状態、位置、リミット状態、homed 状態は Serial ログと本体画面で確認できます。

## 機械パラメータ

GT2 20T 前提です。

```text
GT2 pitch: 2 mm
Pulley: 20 teeth
Travel per rev: 40 mm
Motor: NEMA17 1.8 deg, 200 full steps/rev
Microstep: 1/16, 3200 steps/rev
steps/mm: 3200 / 40 = 80
```

## ビルドとアップロード

```sh
pio run
pio run -t upload
pio device monitor
```

Serial monitor は `115200 bps` です。
