# Pico Copter FC — 共通基板対応・双方向DShot300・ESC UART

**このリポジトリは同一回路を Raspberry Pi Pico（RP2040）と Pico 2（RP2350）の両方で使用するためのファームウェアです。** ピン配置は共通で、GP16がSN74LVC245AのDIR、GP18が/OE、GP17がESC専用TXテレメトリです。両機種ともデフォルトは**双方向DShot300**です。

**[UF2・ソースZIP配布ページ](https://temesotejam.github.io/pico-copter-fc-esc-telemetry/)** ｜ [ファームウェア／ビルド詳細](firmware/pico_copter/README.md) ｜ [配線・テレメトリ仕様](firmware/pico_copter/ESC_SERIAL_TELEMETRY.md)

## 対応状況

| 項目 | Pico（RP2040） | Pico 2（RP2350） |
|---|---|---|
| 共通基板／同じ接続先 | 対応するように実装（**実機未確認**） | **2026-10-27実機通信確認済み** |
| DShotモード | 双方向DShot300 | 双方向DShot300 |
| 4基のFCモータ指令 | GP15 / GP14 / GP3 / GP2 | GP15 / GP14 / GP3 / GP2 |
| 追加GP20/21 | **入力＋弱プルダウン（DShotなし）** | **DShot 0（STOP）送信** |
| ESC専用TXパッド受信 | GP17・PIO1 | GP17・PIO2 |
| SN74LVC245A DIR、/OE | GP16、GP18・PIO1 | GP16、GP18・PIO2 |
| DShotのPIO | PIO0：4チャンネル | PIO0/PIO1：6チャンネル |
| PIO間同期 | **CPU順次起動・要波形検証** | PIOハードウェア同期 |
| 制御／姿勢ループ | 400 Hz／100 Hz | 400 Hz／100 Hz |

**両機種とも姿勢制御・モータミキサは4基用です。** Pico 2ではGP20/GP21へSTOP指令を出しますが、初代PicoはこれらのピンからDShotを送信しません。追加2基のESCを使用する場合、初代Picoでの動作は保証されません。6基飛行制御は未実装です。

### 共通GPIO配置

| 役割 | GPIO | 物理ピン |
|---|---:|---:|
| Rear Left（RL）DShot | GP2 | 4 |
| Rear Right（RR）DShot | GP3 | 5 |
| Front Left（FL）DShot | GP14 | 19 |
| Front Right（FR）DShot | GP15 | 20 |
| 追加DShot 5/6 | GP20 / GP21 | 26 / 27 |
| **SN74LVC245A 1番ピン DIR** | **GP16** | **21** |
| **ESC専用TXテレメトリ RX** | **GP17** | **22** |
| **SN74LVC245A 19番ピン /OE** | **GP18** | **24** |
| SBUS受信（UART0） | GP1 | 2 |
| サーボPWM | GP6 | 9 |

SN74LVC245Aは**A側＝マイコン、B側＝ESC**として使用します。**/OE（GP18）に3.3 Vへの外付けプルアップを設け、リセット中は必ず出力無効（Hi-Z）**にしてください。DIRと/OEの固定配線があれば取り外します。

ESC専用TX（GP17）はSN74LVC245Aの共通DIR/OE回路ではなく、**電圧が3.3 V GPIOに適合する別の受信回路**を介します。複数ESCのTX出力を無条件に直結しないでください。必要ならバッファやマルチプレクサを使います。

## DShotとUARTテレメトリ

**DShotの返信**は制御線を送信後に受信へ切り替えて、各モータのeRPMを取得します。一方、**ESC専用TXテレメトリ**はGP17で115200 bps／8N1の10バイトKISSパケットを取得します（温度、電圧、電流、mAh、eRPM）。両機種とも**DShotテレメトリ要求ビットを順番に送信する方式**です。要求間隔40 ms以上、受信待ち8 ms、古い測定値は500 msで無効になります。ESC側のAuto TelemetryはOFFが前提です。

いずれもUSB診断用であり、PID/EKFやモータ出力フィードバックには使いません。`#DShot300`と`#ESC_UART`ログを区別してください。電源の抜き差しを行うと累積受信成功率は下がるので、接続中のカウンタ差分で評価してください。

## 実機試験と保存版

- **2026-10-27：Pico 2（RP2350）＋SN74LVC245Aで、4基同時の双方向DShot300返信とGP17のUARTテレメトリを実機確認。** ベンチ試験であり、飛行安定性や全電気条件の安全性の検証ではありません。
- **Pico（RP2040）＋同じレベルシフタ回路：本ブランチで実装した対応は、ビルド確認と実機波形検証が必要です。** PIO0とPIO1間で完全なハードウェア同時起動はできず、タイミング余裕を調整した新しい`pico_shifter_rp2040.pio`を使用します。ESC接続前にGP16/18とDShotの切り替え波形・バス競合をオシロスコープで確認してください。
- **[Pico 2動作成功版を固定保存したブランチ](https://github.com/temesotejam/pico-copter-fc-esc-telemetry/tree/archive/pico2-bidir-dshot300-uart-gp17-working-20261027)**（コミット`70ef5e610d017eea354c6ea3d0f266ea5b14f5d2`）は変更していません。

## UF2配布・書き込み

[配布ページ](https://temesotejam.github.io/pico-copter-fc-esc-telemetry/)で**PicoとPico 2を選択**し、それぞれのUF2を取得します。

1. プロペラを外し、対象ボードの**BOOTSEL**を押しながらUSB接続します。
2. 対応するUF2をUSBドライブへコピーします。
3. 再起動後、USBシリアルで`#DShot CONFIG rate=300 kbps mode=bidir control=400Hz`を確認します。
4. 最初はESC 1基でSTOP、モータ対応、双方向返信、GP17 UARTおよびSBUS通信断を確認します。

## 開発とローカルビルド

`firmware/pico_copter/`がビルド可能なCMakeプロジェクトです。Pico SDK 2.3.0、ARM GCC、CMake、Eigen 3.4.0を使用します。配布ZIPにはEigenヘッダが含まれます。

```bash
# Pico（RP2040）
cmake -S firmware/pico_copter -B build-pico -G Ninja -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_BOARD=pico
cmake --build build-pico --parallel

# Pico 2（RP2350）
cmake -S firmware/pico_copter -B build-pico2 -G Ninja -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_BOARD=pico2
cmake --build build-pico2 --parallel
```

標準は双方向DShot300。機種ごとのPIO実装は`pwm_pico_rp2040.cpp`／`pwm_pico2.cpp`で分岐します。USB出力・姿勢制御・SBUSの安全処理・100 Hz／38項目のCH5 RAMログは共通です。

[GPIO・プログラム内部の詳細](firmware/pico_copter/README.md) ｜ [電気的な注意事項](firmware/pico_copter/ESC_SERIAL_TELEMETRY.md) ｜ [任意：Arduino UNO R3のBLHeli設定用インターフェース書き込み](https://temesotejam.github.io/pico-copter-fc-esc-telemetry/uno-flasher.html)
