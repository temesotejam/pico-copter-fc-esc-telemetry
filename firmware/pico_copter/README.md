# Pico Copter firmware (BLHeli_32 serial TX telemetry variant)

This branch defaults to **bidirectional DShot300**. It detects a connected
ESC only after a valid eRPM reply on that specific motor DShot GPIO,
and removes the detected status after 250 ms without valid replies.
It adds a separate ESC TX-pad 115200 baud receive path at
**GPIO5/UART1 RX**. When a detected motor runs, serial telemetry is
requested using the DShot telemetry bit. Unconnected channels are skipped
for UART requests, but their DShot control commands are still sent. New `#ESC_UART` USB diagnostics report voltage,
current, consumption, temperature and electrical RPM with CRC and age.
Read [ESC_SERIAL_TELEMETRY.md](ESC_SERIAL_TELEMETRY.md) before wiring.
The 38-field CH5 flight log and all original motor/RC control are untouched.


このフォルダが、Raspberry Pi Picoへ書き込むファームウェアの独立したプロジェクトです。共有するときは`firmware/pico_copter`フォルダをそのまま取り出せます。

`.cpp`、`.hpp`、`.c`、`.h`は`github.dev`または通常のローカル開発環境で編集できます。変更履歴はGitで管理され、GitHub Actionsが変更後のファームウェアをコンパイルします。

## 必要なもの

- Raspberry Pi Pico SDK 2.3.0
- ARM GCC
- CMake 3.13以上
- Eigen 3.4.0

Eigenの場所を指定しない場合、CMakeが公式Eigenの固定コミットを自動取得します。

## 別環境への持ち出し用ソースZIP

[配布ページ](https://temesotejam.github.io/pico-copter-fc/)から`pico_copter_source.zip`を取得すると、このフォルダの**実行に必要なソース・CMakeLists.txt・PIO・ライセンス・ビルド手順**と、**Eigen 3.4.0のヘッダ**を入手できます。リポジトリの`web/`、`.github/`、旧テストファイルやGit履歴は含めません。

展開後の`pico_copter`がCMakeのトップディレクトリです。同梱の`third_party/eigen/Eigen`をCMakeが自動認識します。**Pico SDK 2.3.0、ARM GCC、CMake（Ninja推奨）**は別途インストールが必要です。詳細はZIP内の`PORTABLE_BUILD.md`を参照してください。

何度もゲイン調整するときは、初回のビルド後に`build`フォルダを消さず、変更後に`cmake --build build --parallel`だけを実行してください。EigenやPico SDKを毎回取得・構築し直す必要はありません。

## ビルド例

```bash
cmake -S . -B build \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DPICO_BOARD=pico \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --parallel
```

生成物は`build/pico_copter.uf2`です。

既にEigenを持っている場合は、次を追加できます。

```bash
-DEIGEN3_INCLUDE_DIR=/path/to/eigen
```

## i-26 からの移植について

このファームウェアは `pico-copter-uf2-builder` を土台に、`i-26` の制御・PWM・SBUS等の変更を移植しています。**モータミキシングでは `R_com`（Yaw補正）の符号だけ元のPico Copterを維持しています。現在はスロットルの使用可能範囲を再スケーリングし、姿勢補正項に `0.9 / BATTERY_VOLTAGE` を掛けています。**

### GPIO配置（DShot300・PWM）

| Output | GPIO | Pico physical pin |
|---|---:|---:|
| RL | 2 | 4 |
| RR | 3 | 5 |
| FL | **14** | **19** |
| FR | **15** | **20** |
| Servo | 6 | 9 |

モータ信号はPIOによるDShot300であり、サーボGPIO6のみPWMです。ピンを変更する場合は `pwm.hpp` の `PWM_PIN_*` を確認します。メインループは `PWM_MAINLOOP_PIN` のPWMラップ割り込み（400Hz）に同期します。

### 実機に書き込む前の注意

- 現行基板では FL=GPIO14 / FR=GPIO15 を使用します。i-26 のREADMEの GPIO4 / GPIO5 は旧配線を示しており、現在の `pwm.hpp` には適合しません。
- i-26 に合わせてPIDパラメータとSBUS範囲が変更されているため、送信機のキャリブレーションやゲインの再確認が必要です。
- モータミキシングの符号は i-26 に準じ、`R_com` の符号だけ Pico Copter のオリジナルを維持しています。スロットル基準値は使用可能範囲に合わせて換算します。
- プロペラを外して各軸の応答・回転方向・フェイルセーフを確認してください。

## CH5操作による100 Hzリングバッファ記録

飛行中のログは**CH5をONにした間だけ**記録します。アームするだけでは記録せず、CH5をOFFにするとその時点の直近約10秒分をRAMに保持します。

| 操作・状況 | 記録の動作 |
|---|---|
| 電源投入・アーム | 記録しない。アーム後にCH5のOFF状態を確認してからONを受け付ける |
| 飛行中にCH5をON | 新しい記録として開始（以前のデータを置き換える） |
| CH5をONのまま | 100 Hzで38項目を1行として連続記録し、満杯なら最古の行を上書き |
| CH5をOFF | 次の100 Hz処理で記録を凍結し、データを消去しない（停止遅延は通常10 ms以内） |
| ディスアーム・通信断 | 記録していた場合は凍結。自動再記録しない |
| 停止後に再びCH5をON | 前の記録を消去して新しい記録を開始 |
| ディスアーム中の既存ログ出力操作 | RAMの実データだけ古い順にUSBシリアル出力。出力後も保持 |
| 再アーム（CH5はOFF） | 以前の記録を保持。新たにCH5をONにするまで上書きしない |

- **記録周期:** 公称100 Hz。リアルタイムの制御・推定を優先し、リング記録以外の処理周期は変更しません。
- **記録内容:** 既存の38項目をそのまま維持します。
- **容量:** 1,000行×38個のfloat＝**152,000バイトの固定RAM**。公称10秒分。
- **CH5の判定:** 現行のCH5中間値より上でON、下側でOFF。
- **出力時刻:** 記録開始からの公称時間で、リングが一周しても出力行の時刻は連続します。実測タイムスタンプではありません。
- **負荷対策:** CH5 OFF時は38項目の行生成・RAM書込みを行わず、CH5 ON時だけ実行します。RAM上で完結し、記録中のシリアル出力やフラッシュ書込みは行いません。

**注意:** RAMのため電源OFFでデータは消失します。新しいCH5 ONでも前の記録を上書きするため、必要なデータは再記録の前にPCへ取り出してください。実機での動作確認は必ずプロペラを外してください。

## 双方向・単方向DShotの切換え

[`dshot_config.hpp`](dshot_config.hpp) の次の2つの設定で変更できます。

```cpp
#define DSHOT_RATE_KBPS 300      // 150/300/600/1200
#define DSHOT_BIDIRECTIONAL 1    // 1=双方向 / 0=単方向
```

実際のファイルではいずれも`#ifndef`ガード内に記載されています。**値を変更して再ビルド・書込みすると切り替わります。実行中の自動切換えはありません。**

- **双方向（1、デフォルト）**：`dshot_tx.pio`を使用。反転波形・反転CRC、同じGPIOでESC返信を受信しeRPMを復号。返信が欠けても指令を継続します。今回の改修でも、以前eRPM取得に成功した双方向PIOの命令列は維持しています。
- **単方向（0）**：`dshot_oneway.pio`を使用。非反転波形・通常CRC、送信だけ行い、eRPMは取得しません。USB診断は`mode=oneway`、`valid=0`、`eRPM=-1`となります。この派生版では、DShot応答による接続確認ができないため、単方向設定時は専用TXテレメトリ要求も無効になります。

両モードともモータ出力は400 Hzで更新し、既存のスロットル変換・PID・EKF・SBUSフェイルセーフ・CH5ログは共通です。CMakeの`-DDSHOT_BIDIRECTIONAL=0/1`と`-DDSHOT_RATE_KBPS=150/300/600/1200`による一時的な上書きも可能です。

GitHub Actionsでは通常ビルドとして**ヘッダで選んだモード**を配布し、もう一方のモードも検証用にコンパイルします。Webの配布ページには選択UIを追加せず、実際のビルドモードをマニフェストの説明文に反映します。

## DShot速度の設定（150 / 300 / 600 / 1200）

[`dshot_config.hpp`](dshot_config.hpp) にある以下の**1行**が通常の変更箇所です。

```cpp
#ifndef DSHOT_RATE_KBPS
#define DSHOT_RATE_KBPS 300
#endif
```

`150`、`300`（実機検証済み）、`600`、`1200` のいずれかを設定し、Pico用UF2を再ビルドします。双方向時は1ビット40サイクルでPIOクロックを6/12/24/48 MHzに変更し、約25 µsの折り返し待ちと約43 µsの返信監視を調整します。単方向時は1ビット8サイクルでPIOクロックを変更し、受信処理は使用しません。受信待ち回数は、元のPIOの`set y, 7`命令を読み取り、**起動前にだけ**指定速度用の`set y, 3/7/15/31`へ差し替えます。300の場合は命令を変更しません。

ビルド時に一時的な設定変更をする方法：

```bash
cmake -S . -B build-dshot600 \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DPICO_BOARD=pico \
  -DDSHOT_RATE_KBPS=600
cmake --build build-dshot600 --parallel
```

`-DDSHOT_RATE_KBPS`を指定した場合はファイル内の値より優先されます。別速度への切替時は、**専用のビルドフォルダを使用するかCMakeキャッシュを更新**してください。速度の設定を変更しても、制御側の400 Hz出力周期／100 Hz姿勢制御、DShot 0停止、回転指令48〜2047、PID/EKF、CH5記録、受信欠落時の非ブロッキング制御は変更されません。

起動時の`#DShot CONFIG rate=600 kbps mode=bidir control=400Hz`と`#DShot600 RL mode=bidir ...`で適用した速度を確認できます。300以外のeRPM動作は**未検証**なので、最初はプロペラを外して波形・返信・受信率を測定してください。

## USBデバッグの管理場所

通常のUSB CDC表示は **[`debug_usb.cpp`](debug_usb.cpp)** にまとめています。表示文字列と表示周期（非アーム時約500 ms、アーム時約1秒）はこのファイルで変更できます。**[`debug_usb.hpp`](debug_usb.hpp)** に以下の表示スイッチをまとめています。

| `debug_usb.hpp` の設定 | 表示内容 | USB出力 |
|---|---|---|
| `kEnablePeriodicDiagnostics` | 周期診断全体 | 全て一括ON/OFF |
| `kEnableRcStateMessages` | RC受信状態の変化 | `#RC` |
| `kEnableState` | アーム・フェイルセーフなど | `#DBG STATE` |
| `kEnableSbus` | SBUS CH1〜6・通信鮮度 | `#DBG SBUS` |
| `kEnableImu` | 加速度とジャイロ角速度 | `#DBG IMU`、`#DBG GYRO` |
| `kEnableMag` | 補正・正規化した地磁気ベクトル | `#DBG MAG` |
| `kEnableAttitude` | 推定ロール・ピッチ・ヨー | `#DBG ATT` |
| `kEnableEkf` | 推定クォータニオン・ジャイロバイアス | `#DBG EKF` |
| `kEnablePidMixer` | T、P/Q/R、FR/FL/RR/RL | `#DBG MIX` |
| `kEnableDShot` | DShot指令とeRPM・通信エラー | `#DShot<速度>` |

`true` / `false` に変更し、ビルドし直すと表示の有無を切り替えられます。これは**表示のみ**の設定で、IMU読み込み・EKF計算・PID・DShot送受信・CH5記録には影響しません。

### IMUと姿勢推定の単位・読出し方法

- `#DBG IMU ax/ay/az`：コントローラの座標系での加速度、**m/s²**
- `#DBG GYRO p/q/r`：コントローラの座標系での角速度、**rad/s**（生センサのmdpsから変換済み）
- `#DBG MAG mx/my/mz`：地磁気を校正・正規化した**無次元の方向ベクトル**（mGではない）
- `#DBG ATT roll_deg/pitch_deg/yaw_deg`：`Phi-Phi_bias`など**基準角を差し引いた角度、度（°）**。CH5の姿勢角記録と同じ値を度換算したもの
- `#DBG ATT raw_*_deg`：基準角を差し引く前の推定角度、度（°）
- `#DBG EKF q0..q3`：スカラー先頭の推定クォータニオン、`bias_p/bias_q/bias_r`：EKF内部のバイアス状態
- `age_ms`：最後のIMU／EKF更新からの経過時間。ディスアーム時はIMU更新が止まる場合があるため、**値の新しさの判断**に使う
- `valid=0`：起動直後などで、まだ対応するセンサ／推定結果が初回取得されていない状態

IMU情報は400 Hz割り込み側の既存の値を短時間割り込み保護してコピーし、別コアで動く100 HzのEKF／姿勢推定は専用mutexで保護した**診断専用スナップショット**として受け渡します。USB表示のためにセンサ通信やEKF計算を追加したり、割り込み内で`printf`することはありません。

`pico_copter.cpp` は起動時メッセージと `debug_usb::poll()` を呼ぶだけで、`sensor.cpp` の「Device not found !」も`debug_usb::sensor_not_found()`で表示します。各機能はスナップショットを提供するだけなので、メインループでまとめて文字列化します。**USB出力を制御割り込みやコア1では実行しません。** USB未接続時とCH5ログ出力中には周期診断を止めます。

CH5の100 Hz・38項目データ、P/Q/R行列、PIDゲインはCSV風実験ログの一部なので、`control.cpp`の`log_output()`を変更せず別扱いとしています。不要になっていた未使用の単発デバッグ関数は削除しました。

## 現行ベースライン：初代Picoの双方向DShot300

**2026-10-16、初代Pico（RP2040）とレベルシフタ未実装の別基板で、ESC・モータ1組によるeRPM取得を実機確認しました。** スロットル操作へのeRPMの追従も良好との報告です。4基同時・実飛行・Pico 2の双方向受信の動作保証はありません。

- [動作確認済みの変更前コミットを固定した保存ブランチ](https://github.com/temesotejam/pico-copter-fc/tree/archive/pico-rp2040-bidir-dshot300-erpm-working-20261016)（`8c8dc12eccf2c3aface2deb3d233d23d1701714d`）
- [実験条件・プロトコル・復元手順の記録](DSHOT300_BASELINE.md)
- [以前の一方向DShot300実装](https://github.com/temesotejam/pico-copter-fc/commit/2ed49dcd35af65c0a592df5d8b10def4ff508090)

### ピンとハードウェア

| 出力 | Pico GPIO | 物理ピン |
|---|---:|---:|
| FR | 15 | 20 |
| FL | 14 | 19 |
| RR | 3 | 5 |
| RL（今回の実機確認） | **2** | **4** |
| サーボPWM | 6 | 9 |

新基板は一方向レベルシフタを挟まず、Pico GPIOとESC Signalを接続します。GNDは共通です。ESCの駆動用電源電圧とDShot信号の電圧は区別し、**GPIOに5 Vを印加しないでください**。別回路の出力ドライバが同じ信号線を駆動すると故障の恐れがあります。

### 実装上の構成と維持事項

- `dshot_tx.pio`：40 PIOクロック/ビットの反転DShot300送信、約25 µsの折り返し待ち、GPIO方向切替、返信先頭を約43 µs待機、21ビットの返信サンプリング。**実機で成功したPIO命令列・待ち時間は維持**。
- `dshot_protocol.hpp`：停止`0`・回転`48〜2047`、反転CRCと送信用の反転16ビット列。ここに残る`frame()`と`pio_word()`は**旧一方向版の互換・試験用**で、現在のモータ送信には使っていません。
- `pwm.cpp`/`pwm.hpp`：PIO0を4モータ分使用、400 Hz割り込みで制御後に送信し、前回返信を**待機なし**で取り出します。速度の設定は `dshot_config.hpp` で変更します。GPIO6サーボPWM、PWMスライス7による400 Hzタイマーは維持。
- `dshot_bidir_decode.hpp`：NRZI/GCR・CRC・eRPM復号。eRPMは**診断専用**でPID・EKF・モータ指令のフィードバックには使用していません。
- `control.cpp`：スロットル停止判定は従来の`T_ref/BATTERY_VOLTAGE < Disable_duty`、停止時はDShot`0`を継続送信。走行時`duty=0.10〜0.95`を`48〜2047`に割り当てます。各軸ミキシング・ゲイン・SBUSフェイルセーフ・CH5リングログは維持。
- `tests/test_dshot*.cpp`と`tests/test_motor_mix_scaling.cpp`：プロトコルのパケット・CRC、テレメトリ復号、スロットル指令の全域テスト。GitHub ActionsでPico／Pico 2をそれぞれビルドしますが、実機確認済みは初代Picoのみです。

### 双方向モードでESC返信を受信できない場合

**双方向モードでは返信テレメトリは任意です。** 4モータのDShot指令は400 Hzの制御計算から送られ、eRPMの有無・数値はPID、EKF、アーム判定、モータミキシング、SBUSフェイルセーフに使っていません。

- ESC未接続・停止中の返信なし・一時的なCRC/GCRエラー・通信線のノイズの場合でも、**返信完了を待つブロッキング処理をせず、次の400 Hz指令送信を続けます**。
- 正常なeRPMが250 ms以上来なければ、USBログの`valid=0`、`eRPM=-1`となるだけです。最後のeRPMを飛行制御に使用したり、返信がないだけで停止やアーム解除をしたりしません。
- RX FIFOに古い返信が残っても、毎周期すべてのチャンネルを最大4語ずつ空にし、送信を不必要に停止させないようにしています。`stray`は余分・遅延返信の破棄累計です。
- `timeout`、`gcr`、`crc`等は診断のみです。`busy`は実際に送信用FIFOが空でなく新しいDShot指令を投入できなかった回数で、**返信欠落とは区別**します。
- **通常の一方向DShotへは自動切替しません。** 送信する波形・CRCは常に双方向DShot300形式です。そのため、通常DShotのみ対応のESCがモータ指令を理解できるとは限りません。今回の改善は「双方向DShot対応ESCの返信が一時的・継続的に欠落しても制御を妨げない」ものです。
- ESCの電源故障やモータ停止などハードウェア自体の不具合をソフトウェアで解決するものではありません。送信FIFOの本当のハング、GPIO故障、電源断を含む全故障時の動作保証ではありません。

### USB CDC診断

USBが接続されていて、CH5ログ出力中でなければ、通常のメインループから表示します（非アーム時約0.5秒間隔、アーム時約1秒間隔）。制御割り込み内では`printf`しません。

| ログ | 主な内容 |
|---|---|
| `#DBG STATE` | arm、lock、フェイルセーフ、CH5状態 |
| `#DBG SBUS` | CH1〜6、生フレーム受信数、受信鮮度、フェイルセーフ |
| `#DBG IMU` / `#DBG GYRO` / `#DBG MAG` | IMUと地磁気の状態・経過時間 |
| `#DBG ATT` / `#DBG EKF` | 姿勢角（°）、クォータニオン・バイアス・更新鮮度 |
| `#DBG MIX` | `T`、`P/Q/R`、FR/FL/RR/RLの制御値 |
| `#DShot150` / `#DShot300` / `#DShot600` / `#DShot1200` | `mode=bidir`、要求`duty`、送信`cmd`と論理`pkt`、eRPM、通信カウンタ |

`#DShot300 RL mode=bidir duty=... cmd=... pkt=... eRPM=... valid=... rx=... timeout=... gcr=... crc=... other=... stray=... busy=...`（標準設定の例。速度変更時は接頭辞が変わります）

`cmd`/`pkt`はPIOへ投入された最新の指令であり、ESCが受信した証明ではありません。`valid=1`は**直近250 ms以内に成功したeRPM**があること、`eRPM=-1 valid=0`は受信値が未取得または古いことを表します。`rx`/`timeout`/`gcr`/`crc`/`other`/`stray`/`busy`は起動後の累積カウンタです。ESC未接続の3チャンネルで`timeout`が増えても、それは異常ではありません。

### 動作確認と復元

成功版を保護した`archive/...`ブランチは、`main`を変更しても残ります。自動配布のUF2は**mainの最新版**なので、同一バイナリとして固定されたものではありません。将来の変更時は保存ブランチのコミットから再ビルドして比較できます。試験はプロペラを外した状態で行い、停止指令、受信成功率、回転方向、フェイルセーフを順に確認してください。

## SBUS communication-loss protection

- `sbus_parser.hpp`: 完全な25バイトフレームのみ解析し、フレームロスト/フェイルセーフフラグを確認。
- `radio.cpp`: 正常なフレームの最終受信時刻を記録し、100 msの通信監視を実施。
- `control.cpp`: 異常検出時にアームを解除し、モータにDShot 0（STOP）を送信。通信復帰後はスロットル低・アーム操作解除を200 ms維持し、通常の再アーム操作を必須にする。
- `tests/test_sbus.cpp`: フレーム解析、通信タイムアウト、復帰条件の単体テスト。

**注意:** DShot 0の停止指令は電源遮断ではありません。受信機がフェイルセーフフラグなしで古いチャンネル値を送信し続ける場合は通信断を検出できません。プロペラを外して実機確認してください。


## Pico 2 hardware variant (experimental, October 2026)

The RP2040/Pico driver and its UART1 GP5 configuration remain unchanged.
For `PICO_BOARD=pico2`, a separate `pwm_pico2.cpp` driver is selected:

- SN74LVC245APWR A side=Pico2, B side=ESC; **GP18=DIR**, **GP19=/OE**.
  An external pull-up from /OE to 3.3 V is mandatory for safe boot/reset.
  Remove previous hardwired DIR and /OE straps before connecting MCU outputs.
- DShot channels: GP15, GP14, GP3, GP2, GP20, GP21. Existing flight mixer
  controls only the first four. **GP20/GP21 transmit STOP only**, not flight
  throttle, until a six-motor mixer and failsafe are developed.
- ESC dedicated TX-pad telemetry goes through its **own protected input**
  to GP17 (PIO2 UART RX at 115200 8N1), **not** through the shared DIR/OE
  transceiver. This input requires voltage compatibility / buffering as needed.
- Pico2 defaults to one-way DShot300; unlike the RP2040 build, it can request
  dedicated ESC TX-pad telemetry **without** eRPM/presence confirmation. It
  requests only for nonzero motor commands, serializes requests, and never
  gates motor output on telemetry.
- To test the shared-SN74LVC245A **bidirectional DShot300** path, build Pico2
  with `-DDSHOT_BIDIRECTIONAL=1`. PIO0+PIO1 motor streams and the PIO2
  DIR/OE controller are synchronously restarted. The PIO UART receiver remains
  continuously active. **Do not operate this mode before verifying DIR/OE
  waveforms, signal voltage, and bus contention on a scope without ESCs.**
  Rates other than 300 kbit/s are deliberately rejected for this experimental
  Pico2 bidirectional controller.
- UART TX-pad frames have no source ID. When multiple ESCs auto-transmit on
  the same wire, frame attribution is impossible and push-pull TX connections
  can electrically conflict. Do **not** tie unrelated TX outputs together
  without a suitable mux/isolation and an explicit source-identification
  scheme. `kFixedUartSourceMotor` may be configured only for a physically
  single-source telemetry connection. Request-scoped attribution assumes
  Auto Telemetry is off on all shared-line ESCs.

These changes are **source/build checks, not hardware validation**. No flight
with Pico2 should be attempted until protected single-ESC tests and
six-channel timing verification have passed.


### Pico 2: request-driven ESC TX telemetry (no bidirectional DShot required)

The target ESC sends a dedicated TX-pad KISS UART reply when the DShot
telemetry-request bit is set. Pico 2 therefore uses **request-only**
telemetry: the default one-way DShot300 sends requests to one active motor
(FR/FL/RR/RL) at a time, at least 40 ms apart. Only a complete reply
received within the 8 ms request window may be assigned to that motor.
Two consistent CRC-valid replies are needed before the UART data is shown
as fresh and the ESC is marked detected. RP2350 does not use bidirectional
DShot eRPM for this presence decision. The RP2040 path stays unchanged.

GP17 receives the UART TX-pad signal through its own protected input; it
does not pass through the common DIR/OE-controlled SN74LVC245A. GPIO20/21
remain STOP-only until a six-motor flight mixer is implemented.

**Electrical caution:** one-at-a-time DShot requests do not automatically
make several push-pull ESC TX pins safe to wire together. Verify the ESC
TX electrical output topology first; add per-ESC input isolation or a
controlled multiplexer if needed. Disable unsolicited/auto ESC telemetry
for request-window-based motor identification.
