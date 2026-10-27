# Pico Copter — Pico / Pico 2 共通基板用ファームウェア

**同じSN74LVC245A回路でPico（RP2040）とPico 2（RP2350）を使用できるよう設計しています。** 両機種とも標準モードは双方向DShot300、DIR=GP16、/OE=GP18、ESC専用TX受信=GP17です。2026-10-27に実機通信を確認できたのは**Pico 2のみ**です。初代Picoの同回路版はまだ実機未検証であり、飛行試験済みではありません。

[リポジトリTOP](../../README.md) ｜ [ESC UART・配線の詳細](ESC_SERIAL_TELEMETRY.md) ｜ [UF2配布](https://temesotejam.github.io/pico-copter-fc-esc-telemetry/)

## ハードウェアと主な動作

| 機能 | 設定 |
|---|---|
| MCU | **Raspberry Pi Pico（RP2040）／Pico 2（RP2350）** |
| DShot | **双方向DShot300**、4基を400 Hzで制御、eRPM返信は診断用（Pico 2で実機検証済み） |
| DShot予備回線 | Pico 2：GP20/21からDShot 0。初代Pico：GP20/21は入力＋弱プルダウンで、DShotは送信しない |
| 姿勢推定 | LSM9DS1、EKF（100 Hz姿勢ループ） |
| モータ制御 | PID、P/Q/Rミキサ、4モータの推力範囲を従来duty 0.10〜0.95からDShot値48〜2047に対応付け |
| RC | SBUS 100000 bps、8E2、UART0 GP1 |
| 専用TXテレメトリ | GP17でPIO UART RX（RP2040: PIO1／RP2350: PIO2、115200 bps、8N1）、温度・電圧・電流・mAh・eRPM |
| サーボ | GP6、従来PWM |
| USB | DBG/IMU/MAG/ATT/EKF/MIX、DShot返信統計、ESC_UART |
| 記録 | CH5操作による100 Hz・38項目のRAMリングログ（直近1000行／約10秒） |

### 共通基板のGPIO一覧

| 用途 | Pico GPIO | 物理ピン |
|---|---:|---:|
| RL（後左） | GP2 | 4 |
| RR（後右） | GP3 | 5 |
| FL（前左） | GP14 | 19 |
| FR（前右） | GP15 | 20 |
| 予備DShot 5、6（STOP固定） | GP20、GP21 | 26、27 |
| ESC専用TXパッドUART RX | **GP17** | **22** |
| SN74LVC245A IC 1番ピン（DIR） | **GP16** | **21** |
| SN74LVC245A IC 19番ピン（/OE） | **GP18** | **24** |
| SBUS RX | GP1 | 2 |
| サーボPWM | GP6 | 9 |

SN74LVC245AのA側＝Pico 2、B側＝ESCを想定します。DIR=HighならA→B、DIR=LowならB→A、/OE=HighならHi-Zです。DIRと/OEは隣接GPIOではないため、`pico2_shifter.pio`はSET（GP16）とside-set（GP18）を使います。**/OEは3.3 Vへ外付けでプルアップしてください。** 起動時に不意にレベルシフタが有効にならないための重要な条件です。GPIOは5 V耐性がありません。ESC専用TXパッドはSN74LVC245Aとは別の、電圧に適合した保護入力を介してGP17に接続します。

**GP20/21は6基飛行用の制御回線ではありません。** Pico 2ではDShot 0（STOP）を送信しますが、初代PicoはDShotを送信せず入力＋弱プルダウンです。したがって初代Picoで5/6番目のESCは動作させないでください。現在のPID・ミキサ・フェイルセーフは4基用です。

## ESCの2種類のテレメトリ

1. **DShot eRPM返信:** DShot信号線を送信後に入力に切り替えて取得します。4基の`#DShot300`表示に`eRPM`と`valid`が出ます。返信がないときも次回のモータ指令は継続します。
2. **ESCの専用TXパッド:** GP17（PIO UART）で受信します。DShot指令のテレメトリ要求ビットを非ゼロ推力のESCに順次設定し、**40 ms以上の間隔、8 ms以内の応答**を対応付けます。初回のUART値は2フレームの整合性を確認し、古い値は500 msで無効です。これはDShotのeRPMが取得できない場合も要求可能です。Auto TelemetryをOFFにして要求応答で識別する前提です。

UARTのKISSテレメトリに送信元ESCのIDは含まれません。**複数ESCのpush-pull TX出力を直接束ねないでください。** 要求を1基ずつ出していても、電気的に安全とは限りません。ESCのTX電気方式を確認し、必要なら別受信回路またはマルチプレクサを使います。詳細は[ESC_SERIAL_TELEMETRY.md](ESC_SERIAL_TELEMETRY.md)。

いずれもPID・EKFの入力には使用しません。専用UARTの温度・電圧・電流はESC内部計測値であり、独立した校正値ではありません。

## ビルド

**対応ターゲットは`pico`および`pico2`です。** GitHub Actionsと配布ページは両方のUF2を生成します。CMakeはPico SDK 2.3.0、ARM GCC、CMake 3.13以上、Eigen 3.4.0を使用します。配布用ソースZIPにはEigenヘッダを同梱し、SDKとコンパイラは含まれません。

```bash
cmake -S firmware/pico_copter -B build-pico2 -G Ninja \
  -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_BOARD=pico2 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-pico2 --parallel
# build-pico2/pico_copter.uf2
```

初代Pico向けは`-DPICO_BOARD=pico`と別の`build-pico`フォルダを使ってください。生成物は初代Pico用UF2です。Pico 2用UF2と取り違えないようにしてください。

CMakeをこのディレクトリで実行する場合は`cmake -S . -B build ...`でも構いません。同じビルドディレクトリを再利用すればゲイン等を変更後に増分ビルドできます。

両機種とも標準設定は`dshot_config.hpp`の`DSHOT_RATE_KBPS=300`、`DSHOT_BIDIRECTIONAL=1`です。`-DDSHOT_BIDIRECTIONAL=0`は実験用の単方向ビルド指定ですが、**今回実機成功した保存版ではありません**。Pico 2および初代Picoの共有レベルシフタ版双方向PIOは**DShot300のみ**をサポートします。150/600/1200への双方向変更は静的アサーションで拒否されます。

### ファイルの役割

| ファイル | 役割 |
|---|---|
| `pwm_pico2.cpp` | RP2350用6回線DShotドライバ、400 Hz制御、DIR/OE |
| `pwm_pico_rp2040.cpp` | RP2040用4回線DShotドライバ、PIO0、PIO1のDIR/OE制御（未実機検証） |
| `dshot_tx_pico2.pio` | 双方向DShotの送受信 |
| `pico2_shifter.pio` | Pico 2でGP16（DIR）・GP18（/OE）の同期切り替え |
| `pico_shifter_rp2040.pio` | 初代Pico用のPIO1方向切替、PIO0との起動時差を考慮（要波形検証） |
| `pico2_uart_rx.cpp/.pio` | GP17の専用UART受信。RP2040: PIO1、RP2350: PIO2 |
| `esc_serial_uart.cpp/.hpp` | UARTテレメトリ解析・要求管理 |
| `control.cpp` | 4基PID/ミキサ、CH5記録・フェイルセーフ |
| `radio.cpp`、`sbus_parser.hpp` | SBUSの受信とリンクの安全判定 |
| `sensor.cpp`、`ekf.cpp` | LSM9DS1取得と姿勢推定 |
| `debug_usb.cpp/.hpp` | USB表示形式と表示カテゴリのON/OFF |
| `CMakeLists.txt` | Pico 2用ファームウェアをビルド |

従来のGP5 UARTやRP2040向け旧ドライバ`pwm.cpp`は本配線の標準ビルドには使用しません。ソースとしては過去版との比較のため残しています。

## 送信機の安全処理・ログ

- **SBUS通信断:** 正常フレームが100 ms届かなければアーム解除し、DShot 0を送信します。フェイルセーフフラグ受信時も停止します。再受信後はスロットル低・アーム操作解除を200 ms継続し、再アーム操作が必要です。**STOPはモータ電源遮断ではありません。**
- **CH5記録:** アーム後にCH5をOFF→ONで記録開始。RAMに1000行×38個のfloat（152 kB、約10秒）を保持。CH5 OFF／ディスアーム／通信断で凍結し、USB出力後も保持します。再記録や電源OFFでは旧データが失われます。
- **USB診断:** 非アーム時は約0.5秒ごと、アーム中は約1秒ごと。`#DBG STATE`、`#DBG SBUS`、`#DBG IMU`、`#DBG GYRO`、`#DBG MAG`、`#DBG ATT`、`#DBG EKF`、`#DBG MIX`、`#DShot300`、`#ESC_UART`。詳細は`debug_usb.cpp`。`valid=0`は最新データなしを意味し、ゼロ値ではありません。

## 保存済みPico 2実機成功版と安全上の制限

[**2026-10-27 Pico 2実機動作成功版**](https://github.com/temesotejam/pico-copter-fc-esc-telemetry/tree/archive/pico2-bidir-dshot300-uart-gp17-working-20261027)／コミット`70ef5e610d017eea354c6ea3d0f266ea5b14f5d2`。この保存版は`main`を編集しても変わりません。

**Pico 2では**4基同時のDShot eRPM返信とGP17のUART受信を確認しましたが、これは**ベンチでの通信検証**です。**初代Picoの同回路対応は未実機検証です。**飛行安定性、6基制御、電源・信号の電気的安全性については未検証です。必ずプロペラを外し、/OEの外付けプルアップ、端子電圧、GP16・GP18の波形、信号線の競合を確認してください。

### Pico（RP2040）の追加の制限

初代PicoにはPIO2がないため、PIO0の4ステートマシンでモータを駆動し、PIO1の2ステートマシンをGP17 UARTとGP16/18の方向制御へ使います。RP2350のような**PIOブロック間のハードウェア同時起動は使用できません**。PIO0を開始して直後にPIO1の方向制御を開始するため、信号タイミングをPico実機で測定する必要があります。**ビルド成功だけでESC接続時の安全性を判断しないでください。**
