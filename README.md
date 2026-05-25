# Belt Driven Linear Axis

M5Atom S3 と TMC2209 STEP/DIR ドライバで、GT2 ベルト駆動の 1 軸リニア機構を動かす PlatformIO プロジェクトです。
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
ATOM S3                         TMC2209

3.3V  ------------------------  VDD
GND   ------------------------  GND
G5    ------------------------  STEP
G6    ------------------------  DIR
G1 / TX -- 1kΩ ---------------  PDN_UART
G2 / RX ----------------------  PDN_UART

                                ENABLE ---- GND

12V電源 + --------------------  VBB / VMOT
12V電源 - --------------------  GND

モータ コイルA ---------------  OUT1A / OUT1B
モータ コイルB ---------------  OUT2A / OUT2B
```

| ATOM S3 | TMC2209 |
| --- | --- |
| G5 / GPIO5 | STEP |
| G6 / GPIO6 | DIR |
| G1 / GPIO1 | UART TX -> 1kΩ -> PDN_UART |
| G2 / GPIO2 | UART RX <- PDN_UART |
| GND | GND |

ATOM S3 と TMC2209 の GND は必ず共通にしてください。
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
3. リミットがOFFになるまで反対方向へ戻ります。
4. `HOMING_SLOW_MM_S` でもう一度リミット方向へ移動します。
5. リミット ON 位置を `X=0.00 mm` にします。
6. `Ready` になり、通常移動が許可されます。

`HOMING_MAX_TRAVEL_MM` 以内にリミットへ当たらない場合、または `HOMING_BACKOFF_MM` 戻ってもスイッチが離れない場合は `Error` になります。
起動時点でリミットスイッチがONでも、まずbackoffしてスイッチを離してから低速seekへ進みます。

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
home 完了後も、ソフトリミットを超える移動は拒否されます。
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
| `off` | モータ通電OFF。TMC2209は `toff(0)` で出力段を無効化 |
| `on` | モータ通電ON。TMC2209設定を再適用 |
| `v <mm/s>` | 通常移動の既定速度を変更。`speed <mm/s>` も使用可能 |
| `a <mm/s2>` | 台形加減速の加速度を変更。`accel <mm/s2>` も使用可能 |
| `profile trap` | 通常移動を台形加減速で実行 |
| `profile direct` | 通常移動を従来同等の一定周期STEPで実行 |
| `i <mA>` | TMC2209の実行時RMS電流を変更。`current <mA>` も使用可能 |
| `mode stealth` | TMC2209をstealthChopへ切り替え。`chop stealth` も使用可能 |
| `mode spread` | TMC2209をspreadCycleへ切り替え。`chop spread` も使用可能 |
| `microstep 16` | TMC2209を1/16 microstepへ切り替え。再ホーミング必須 |
| `microstep 8` | TMC2209を1/8 microstepへ切り替え。診断用、再ホーミング必須 |
| `m <mm>` | 任意距離を現在の既定速度で相対移動 |
| `m <mm> <mm/s>` | 任意距離を指定速度で相対移動。この速度はその移動だけに適用 |

10 mm 往復テストは `h` の後に `1` と `b` を送ります。
50 mm テストは `h` の後に `5` を送ります。
移動速度を変更する場合は `h` の後に `v 5` を送り、その後 `1`、`b`、`m 20` などを送ります。
任意距離・任意速度のテストは `h` の後に `m 10 5` や `m -5 2.5` を送ります。
`m` コマンドは home 完了後だけ使えます。`v` は次回以降の通常移動に適用され、実行中の移動速度は変更しません。
速度範囲は `TEST_MOVE_MIN_SPEED_MM_S` から `TEST_MOVE_MAX_SPEED_MM_S` までで、初期値は `0.1..200.0 mm/s`、既定速度の初期値は `DEFAULT_MOVE_SPEED_MM_S` です。
加速度範囲は `TEST_ACCEL_MIN_MM_S2` から `TEST_ACCEL_MAX_MM_S2` までで、初期値は `DEFAULT_ACCELERATION_MM_S2 = 100 mm/s2` です。
`profile trap` が既定です。`profile direct` は脱調切り分けの比較用で、移動開始直後から指定速度のSTEP周期を出します。
`i`、`mode`、`microstep` は TMC2209 UART 接続が OK のときだけ反映されます。
`microstep` を切り替えると `steps/mm` が変わるため、位置保持を信用せず `homed=false` に戻ります。再度 `h` でホーミングしてください。
`off` 後は位置保持が信用できないため `homed=false` に戻ります。`on` 後に再度 `h` でホーミングしてください。
現在状態、位置、リミット状態、homed 状態、既定速度、加速度、速度プロファイル、TMC2209の電流、chop mode、microstep、steps/mm は Serial ログで確認できます。

## 脱調切り分け検証

指令STEPにモータロータが追従できず位置が飛ぶ場合、まず最高速度より加速、減速、反転時を疑います。
ベルトやプーリーがズレていない前提では、加速度、TMC2209モード、電流、メカ負荷の順に切り分けます。

最初の推奨手順:

1. ベルトテンションを少し弱め、レールを端から端まで手で軽く動かせるか確認します。
2. `h` でホーミングします。
3. `profile trap`、`a 100`、`m 50 60` を実行します。
4. NGなら `i 600` で電流を600mAへ上げ、再度 `m -50 60` または `m 50 60` を試します。
5. まだNGなら `mode spread` でspreadCycleへ切り替えて再試験します。
6. 60mm/sがOKになったら `a 200`、`a 300` と上げます。
7. 60mm/sが安定したら `m 50 65`、`m -50 70` のように空走速度を上げます。

判定の目安:

| 結果 | 判断 |
| --- | --- |
| 100mm/s2ならOK | 加速度が高すぎた可能性が高い |
| 100mm/s2でもNG | 電流、ドライバモード、メカ負荷側が濃厚 |
| 低速でも反転時だけNG | 加減速処理または機械の反転負荷を疑う |
| spreadCycleで改善 | stealthChopのトルク不足が濃厚 |
| spreadCycleでもNG | 加速度、電流、メカ負荷が主因 |
| 1/8 microstepで改善 | パルス生成周期または高周波側トルク低下の影響を疑う |

速度境界は50mm/sから細かく見ます。
`s` コマンドで各条件の `profile`、`accel`、`runtimeCurrent`、`chopMode`、`microsteps`、`stepsPerMm` を記録してください。

| 速度 mm/s | 加速度 mm/s2 | 電流 | モード | 結果 |
| --- | --- | --- | --- | --- |
| 50 | 300 | 500mA | stealthChop | OK/NG |
| 52 | 300 | 500mA | stealthChop | OK/NG |
| 55 | 300 | 500mA | stealthChop | OK/NG |
| 58 | 300 | 500mA | stealthChop | OK/NG |
| 60 | 300 | 500mA | stealthChop | OK/NG |
| 60 | 100 | 500mA | stealthChop | OK/NG |
| 60 | 100 | 600mA | stealthChop | OK/NG |
| 60 | 100 | 600mA | spreadCycle | OK/NG |
| 65 | 100 | 600mA | spreadCycle | OK/NG |

電流は一気に上げず、`i 500`、`i 600`、`i 700`、`i 800` の順に試します。

| 状態 | 判断 |
| --- | --- |
| ほんのり温かい | OK |
| 触れるが熱い | 要観察 |
| 触れない | 電流を下げる |
| ドライバが熱停止 | 電流を下げ、放熱を追加する |

1/8 microstepは診断用です。
1/16では `steps/mm = 80`、60mm/s時は4800step/sです。
1/8では `steps/mm = 40`、60mm/s時は2400step/sです。
最終的なペンプロッタ用途では1/16を本命にし、1/8は切り分けとして使います。

24V化やモータ変更は最後の検討項目です。
12Vで高速側のトルクが足りない場合は24V化が効くことがありますが、TMC2209モジュール、電解コンデンサ耐圧、電源容量、放熱を確認してから行ってください。

## 機械パラメータ

GT2 20T 前提です。TMC2209 は起動時にUARTで 1/16 microstepへ設定します。
脱調切り分け時は `microstep 8` で 1/8 に切り替えられます。

```text
GT2 pitch: 2 mm
Pulley: 20 teeth
Travel per rev: 40 mm
Motor: NEMA17 1.8 deg, 200 full steps/rev
Microstep: 1/16, 3200 steps/rev
steps/mm: 3200 / 40 = 80
```

UART接続が失敗するとTMC2209側の既定マイクロステップのまま動くため、Serialログまたは `s` コマンドで `tmc2209_uart=OK` を確認してください。
`tmc2209_uart=FAIL` の場合、microstep設定が検証できないため homing と通常移動は拒否されます。
電流設定は起動時に `TMC_RMS_CURRENT_MA` と `TMC_HOLD_MULTIPLIER` から `IRUN` / `IHOLD` を計算し、UARTで直接レジスタへ書き込みます。
脱調切り分け時は `i <mA>` でRMS電流を実行時に変更できます。
`TMC_TPOWERDOWN` 後に `IHOLD` 側へ移行します。
起動直後はTMC2209がUART設定を取りこぼす場合があるため、`TMC_STARTUP_REAPPLY_DELAY_MS` 待ってから同じ設定を再適用します。
さらに `StepDirDriver` 有効化後にも再適用し、起動直後から `on` コマンド後と同じ設定状態にします。
`s` コマンドで `targetHold`, `irun`, `ihold`, `vsense`, `estimatedRun`, `estimatedHold`, `reportedRms`, `csActual`, `ifcnt` を確認できます。
`ifcnt` はTMC2209が受信したUART書き込み数を示すカウンタです。
実測で校正する場合は `new_steps_per_mm = old_steps_per_mm * commanded_mm / measured_mm` で計算します。

## ビルドとアップロード

```sh
pio run
pio run -t upload
pio device monitor
```

Serial monitor は `115200 bps` です。
