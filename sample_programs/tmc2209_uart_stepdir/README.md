# ATOM S3 TMC2209 UART STEP/DIR Stepper Test

M5Stack ATOM S3からステッピングモータドライバをSTEP/DIRで駆動するサンプルです。
標準設定ではBIGTREETECH TMC2209 V1.2互換ドライバをUARTで初期化し、STEP/DIRでNEMA17を動かします。
`include/config.h` の `ACTIVE_DRIVER` を `DRIVER_A4988` に変更すると、UART初期化を行わないA4988互換のSTEP/DIRテストとしても使えます。

## 前提

- MCU: M5Stack ATOM S3
- Motor driver: BIGTREETECH TMC2209 V1.2互換、またはA4988
- Motor: NEMA17 17HS3401S系 1.8deg / 1A / 34mm
- Belt: GT2
- Pulley: 20T
- Microstep: 1/16
- GT2 20Tは1回転40mmです。
- 200 steps/rev x 16 microsteps / 40mm = `80 steps/mm`

## TMC2209 STEP/DIR + UART配線

```text
M5Stack ATOM S3                 TMC2209

G5    ------------------------  STEP
G6    ------------------------  DIR
G1 / ESP32 TX ---- 1kΩ ------  PDN_UART
G2 / ESP32 RX ----------------  PDN_UART
3.3V  ------------------------  VDD
GND   ------------------------  GND
G39   ------------------------  LIMIT SW

                                EN ---- GND
                                MS1 --- GND
                                MS2 --- GND
                                CLK --- 未接続

12V電源 + --------------------  VS / VMOT
12V電源 - --------------------  GND

モータ コイルA ---------------  A1 / A2
モータ コイルB ---------------  B1 / B2
```

12V電源GND、TMC2209 GND、M5Stack ATOM S3 GNDは必ず共通にしてください。
通電中にモータ線を抜かないでください。ドライバを破損する可能性があります。

TMC2209モジュールに `PDN_UART` が1ピンだけ出ている場合は、M5Stack ATOM S3のG1/TXを1kΩ程度の抵抗経由で `PDN_UART` へ、G2/RXも同じ `PDN_UART` へ接続してください。RX/TXが別端子で出ているモジュールでは、M5のTXをTMCのRXへ、M5のRXをTMCのTXへ接続します。

## A4988配線

```text
M5Stack ATOM S3                 A4988

3.3V  ------------------------  VDD
GND   ------------------------  GND
G5    ------------------------  STEP
G6    ------------------------  DIR
G39   ------------------------  LIMIT SW

                                ENABLE ---- GND
                                SLEEP  ---- 3.3V
                                RESET  ---- 3.3V

12V電源 + --------------------  VBB / VMOT
12V電源 - --------------------  GND

モータ コイルA ---------------  OUT1A / OUT1B
モータ コイルB ---------------  OUT2A / OUT2B
```

## 端子対応表

| M5Stack ATOM S3 | A4988 | TMC2209 |
| --- | --- | --- |
| G5 / GPIO5 | STEP | STEP |
| G6 / GPIO6 | DIR | DIR |
| G1 / GPIO1 | 未使用 | ESP32 TX -> 1kΩ -> TMC PDN_UART |
| G2 / GPIO2 | 未使用 | ESP32 RX <- TMC PDN_UART |
| G39 / GPIO39 | LIMIT SW | LIMIT SW |
| 3.3V | VDD | VDD |
| GND | GND | GND |
| 12V+ | VBB / VMOT | VS / VMOT |
| 12V- | GND | GND |

## TMC2209設定

- UARTは `HardwareSerial(1)` を使います。
- ESP32 RX pin = GPIO2
- ESP32 TX pin = GPIO1
- Baudrate = `115200`
- `R_SENSE = 0.11f`
- `DRIVER_ADDRESS = 0b00`
- `rms_current(500)` で初期電流を500mAに設定します。
- モータやドライバが熱い場合は `include/config.h` の `RMS_CURRENT_MA` を下げてください。
- `include/config.h` のTMC2209 UART configurationで、起動時にUART経由で書き込む値を指定します。

MS1=GND、MS2=GNDの場合、UART addressは `0b00` として使います。
TMC2209ではUART使用時、microstepはソフト側で `driver.microsteps(16)` として設定します。
CLKは未接続で問題ありません。
ENはGND固定で常時有効です。後でGPIO制御化してもかまいません。

`config.h` から指定できるTMC2209 UART設定:

| 分類 | 設定値 |
| --- | --- |
| UART/接続 | `TMC_UART_BAUDRATE`, `DRIVER_ADDRESS`, `R_SENSE` |
| 電流/保持 | `RMS_CURRENT_MA`, `HOLD_MULTIPLIER`, `TMC_USE_IHOLD_IRUN_DIRECT`, `TMC_IHOLD`, `TMC_IRUN`, `TMC_IHOLDDELAY` |
| 共通 | `TMC_GSTAT_CLEAR`, `TMC_TPOWERDOWN`, `TMC_TPWMTHRS` |
| GCONF | `TMC_I_SCALE_ANALOG`, `TMC_INTERNAL_RSENSE`, `TMC_EN_SPREADCYCLE`, `TMC_SHAFT`, `TMC_INDEX_OTPW`, `TMC_INDEX_STEP`, `TMC_PDN_DISABLE`, `TMC_MSTEP_REG_SELECT`, `TMC_MULTISTEP_FILT` |
| SLAVECONF | `TMC_SENDDELAY` |
| FACTORY_CONF | `TMC_WRITE_FACTORY_CONF`, `TMC_FCLKTRIM`, `TMC_OTTRIM` |
| OTP_PROG | `TMC_ENABLE_OTP_PROG`, `TMC_OTP_PROG` |
| VACTUAL | `TMC_VACTUAL` |
| CHOPCONF | `TMC_TOFF`, `TMC_HSTRT`, `TMC_HEND`, `TMC_TBL`, `TMC_VSENSE`, `TMC_USE_MRES_DIRECT`, `TMC_MRES`, `TMC_INTPOL`, `TMC_DEDGE`, `TMC_DISS2G`, `TMC_DISS2VS` |
| PWMCONF | `TMC_PWM_OFS`, `TMC_PWM_GRAD`, `TMC_PWM_FREQ`, `TMC_PWM_AUTOSCALE`, `TMC_PWM_AUTOGRAD`, `TMC_FREEWHEEL`, `TMC_PWM_REG`, `TMC_PWM_LIM` |
| StallGuard/CoolStep | `TMC_TCOOLTHRS`, `TMC_SGTHRS`, `TMC_SEMIN`, `TMC_SEUP`, `TMC_SEMAX`, `TMC_SEDN`, `TMC_SEIMIN` |

`TMC_WRITE_FACTORY_CONF` と `TMC_ENABLE_OTP_PROG` は通常 `false` のままにしてください。
特にOTP programmingは一度書き込むと戻せないため、内容を理解している場合だけ有効化してください。

## リミットスイッチ

GPIO39にリミットスイッチを接続します。
このサンプルではGPIO39を `INPUT_PULLUP` に設定します。
リミットスイッチONでGPIO39がGNDに落ちる配線にしてください。
スイッチONは「原点に到達した」状態として扱います。
起動すると低速でリミットスイッチ方向へ移動し、スイッチONになった位置を0mmとして記録します。
原点を記録した瞬間にモータを停止し、ボタンを押すまでその場で待機します。
原点停止後にボタンを押すと、`include/config.h` の `TEST_TRAVEL_MM` で指定した距離だけ正方向へ移動して停止します。
このサンプルでは原点方向をリバース方向として扱います。

標準値:

```text
DIR_REVERSE_LEVEL = LOW
DIR_FORWARD_LEVEL = HIGH
INVERT_DIR_PIN = true
HOMING_SPEED_MM_PER_SEC = 2.0 mm/s
TEST_TRAVEL_MM = 10.0 mm
```

## 操作

- 起動直後: リバース方向へ原点出しを開始します。
- リミットスイッチON: その位置を0mmとして記録し、モータを停止します。
- 原点停止後のボタン短押し: `TEST_TRAVEL_MM` へ移動します。
- 移動完了後のボタン短押し: 原点出しから同じテストを再実行します。

移動速度プリセットはmm/s基準です。現在は起動時の初期値 `1 mm/s` を使います。

```text
1, 5, 10, 30, 50 mm/s
```

移動テストは安全確認しやすいように `1 mm/s` から始まります。
`80 steps/mm` 前提なので、`1 mm/s` は `80 step/s` です。

## ビルドとアップロード

このディレクトリで実行してください。

```sh
pio run
pio run -t upload
pio device monitor
```

TMC2209 UART設定を使わず、A4988向けSTEP/DIR確認としてビルドする場合:

```sh
pio run -e m5stack-atoms3-a4988
pio run -e m5stack-atoms3-a4988 -t upload
pio device monitor -e m5stack-atoms3-a4988
```

シリアルモニタは `115200 bps` です。
