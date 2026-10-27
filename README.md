# Pico 2 Copter FC — 双方向DShot300・ESCテレメトリ

**このリポジトリは、2026-10-27に実機通信を確認した Raspberry Pi Pico 2（RP2350）＋SN74LVC245A構成専用です。** 初代Pico（RP2040）用UF2は配布しません。ファームウェアの標準設定は**双方向DShot300**です。

[**Pico 2用UF2／ソースZIP配布ページ**](https://temesotejam.github.io/pico-copter-fc-esc-telemetry/) ｜ [ファームウェア／ビルド詳細](firmware/pico_copter/README.md) ｜ [ESCテレメトリ・配線仕様](firmware/pico_copter/ESC_SERIAL_TELEMETRY.md)

## 今回の構成

| 機能 | 実装 |
|---|---|
| 対象ボード | **Raspberry Pi Pico 2（RP2350）** |
| モータ制御 | **双方向DShot300、400 Hzで更新** |
| 姿勢推定・制御 | LSM9DS1＋EKF／PID、姿勢制御100 Hz |
| 受信機 | SBUS（UART0 RX: GP1、100000 bps、8E2） |
| eRPM | 各DShot信号線でESC返信を受信 |
| 追加テレメトリ | **ESC専用TXパッド → GP17**（PIO2 UART RX、115200 bps／8N1） |
| レベルシフタ | **SN74LVC245A、GP16=DIR（IC 1番ピン）、GP18=/OE（IC 19番ピン）** |
| 飛行制御のモータ数 | **4基**（FR/FL/RR/RL） |
| 追加DShot 2回線 | GP20/21は常時**DShot 0（STOP）**。6基飛行制御は未実装 |
| 診断 | USBシリアル（`#DShot300`、`#ESC_UART` など）、CH5の100 Hz・38項目RAMログ |

### Pico 2のGPIO配線

| 接続先 | GPIO | Pico 2の物理ピン |
|---|---:|---:|
| Rear Left（RL）、DShot | 2 | 4 |
| Rear Right（RR）、DShot | 3 | 5 |
| Front Left（FL）、DShot | 14 | 19 |
| Front Right（FR）、DShot | 15 | 20 |
| 追加DShot（STOP固定） | 20、21 | 26、27 |
| サーボ（PWM） | 6 | 9 |
| SBUS受信 | 1 | 2 |
| ESC専用TXテレメトリ受信 | **17** | **22** |
| SN74LVC245A DIR（IC 1番ピン） | **16** | **21** |
| SN74LVC245A /OE（IC 19番ピン） | **18** | **24** |

SN74LVC245Aは**A側＝Pico 2、B側＝ESC**を想定します。GP16とGP18は隣接しないため、PIOのSET／side-setを使って送信と受信の方向を切り替えます。**/OEに3.3 Vへの外付けプルアップが必要**です（起動・リセット中はHi-Zに保つ）。固定DIR・/OE配線を残したままGPIOから駆動しないでください。

**ESCの専用TXパッドはGP17の別の保護入力へ接続**し、共有DIR/OEレベルシフタを通しません。GPIOは5 V非対応です。**複数のESCのTX出力を仕様未確認のまま直結しないでください。** 必要なら個別バッファやマルチプレクサを使ってください。

## DShot返信と専用UARTの違い

- **双方向DShot：** 各モータの制御線からeRPMを取得（同じ信号線を送受信に使用）。返信が欠けてもDShot指令の更新は継続します。
- **専用UART：** ESCのTXパッドから温度・電圧・電流・消費mAh・eRPMを取得。Pico 2では推力指令が0でないESCへDShot要求ビットを順番に付け、**40 ms以上の要求間隔、8 msの返信受付窓**でフレームをそのESCに対応付けます。最初の値は整合性確認を行い、古い値は無効にします。ESCの自動送信（Auto Telemetry）は**OFF**を想定しています。

いずれのテレメトリも**診断用**で、PID/EKFやフェイルセーフの制御入力としては使用していません。USB出力の`#DShot300`と`#ESC_UART`を区別してください。電源の抜き差しを含む累積`rx/tx`の比率は通信中の成功率ではありません。接続中のカウンタ差分を比較してください。

## 実機確認・固定保存版

**2026-10-27：Pico 2＋SN74LVC245Aで4系統同時の双方向DShot300返信、およびGP17のESC専用UARTテレメトリをベンチ試験で取得しました。** ESC電源を切った際には返信が途絶え、再接続後に取得できることを確認しています。今回の結果は通信機能の検証であり、**飛行安定性・6基飛行・信号の電気的安全性を保証するものではありません**。

[**実機動作成功版（固定ブランチ）**](https://github.com/temesotejam/pico-copter-fc-esc-telemetry/tree/archive/pico2-bidir-dshot300-uart-gp17-working-20261027) （コミット `70ef5e610d017eea354c6ea3d0f266ea5b14f5d2`）。このブランチは`main`の更新では変わりません。

## UF2の書き込み

1. [配布ページ](https://temesotejam.github.io/pico-copter-fc-esc-telemetry/)から**Pico 2用**UF2を取得します。
2. プロペラを取り外し、Pico 2の**BOOTSEL**を押しながらUSB接続します。
3. 表示されたUSBドライブへUF2をコピーします。再起動後にUSBシリアルで`#DShot CONFIG rate=300 kbps mode=bidir control=400Hz`を確認します。
4. まずESC 1基でSTOP、DShot返信、GP17 UART、SBUSフェイルセーフを確認してください。信号レベルとGP16・GP18の切替はオシロスコープでの検証を推奨します。

**これは飛行試験済みのリリースではありません。** /OEの初期無効化、電圧・信号方向の確認、プロペラなしの試験が必要です。

## 開発・ビルド

ファームウェアは`firmware/pico_copter/`にあります。GitHub Actionsは**Pico 2用UF2と持ち出し用のソースZIPのみを配布**します。別環境でコンパイルする場合は Pico SDK 2.3.0、ARM GCC、CMake、Eigen 3.4.0 が必要です（配布ZIPにはEigenヘッダ同梱）。

```bash
cmake -S firmware/pico_copter -B build-pico2 -G Ninja \
  -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_BOARD=pico2 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-pico2 --parallel
# build-pico2/pico_copter.uf2
```

- [ファームウェアの構造・設定方法](firmware/pico_copter/README.md)
- [配線・テレメトリ解析・安全上の注意](firmware/pico_copter/ESC_SERIAL_TELEMETRY.md)
- [GitHub Actions](.github/workflows/build-and-deploy.yml)
- [任意：Arduino UNO R3向けBLHeli設定用インターフェース準備](https://temesotejam.github.io/pico-copter-fc-esc-telemetry/uno-flasher.html)

ベースは`temesotejam/pico-copter-fc`および`pico-copter-uf2-builder`で、PID・配線・SBUS等は`i-26`を参考にしています。元の4基の姿勢制御、SBUS復帰処理、CH5リングログは維持しています。
