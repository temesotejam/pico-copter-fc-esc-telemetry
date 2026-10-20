# Pico Copter FC — ESC TX telemetry variant

This repository branches from `temesotejam/pico-copter-fc` and adds
**BLHeli_32 ESC dedicated TX-pad telemetry** to GPIO5 (UART1 RX), separate
from the existing DShot motor controls. This RP2040 build selects
**bidirectional DShot300**, automatically recognizes each ESC from a
recent, checksum-valid eRPM reply on the corresponding motor GPIO, and
requests separate TX-pad data **only from recognized ESCs**. All four DShot
motor outputs remain active, whether an ESC is recognized or not.
Data are shown on USB as
`#ESC_UART`: temperature, voltage, current, mAh, and electrical RPM per ESC.

**See [wiring, voltage limits and no-propeller test procedure](firmware/pico_copter/ESC_SERIAL_TELEMETRY.md).**
The new serial feedback is diagnostic only. The connected status is
based on DShot replies, not stray UART bytes, and expires after 250 ms
without valid eRPM. UART telemetry is not requested while the corresponding
DShot channel is unrecognized or commanded to stop.
Hardware reception from the
specific MAD ESC TX pad has not yet been validated; this is not a
flight-tested firmware release. Original motor mixer, PID/EKF, SBUS failsafe,
and 38-column CH5 logger remain unchanged.


Raspberry Pi PicoおよびPico 2用のPico Copterファームウェアを、GitHub Actionsでコンパイルし、最新のUF2をGitHub Pagesからダウンロードするためのリポジトリです。


## 派生元と移植範囲

- ベース: [pico-copter-uf2-builder](https://github.com/temesotejam/pico-copter-uf2-builder)
- 変更の移植元: [i-26](https://github.com/temesotejam/i-26)
- **Yaw補正 `R_com` の符号は元のPico Copter、P/Qの補正符号とPIDゲインはi-26準拠です。** 従来の `T_ref * 0.9` による早期飽和は修正し、停止解除後のスロットル範囲をモータdutyへ適切にスケーリングしています。
- 移植する変更: PIDパラメータ、EKF呼び出し、PWM GPIO設定・IRQ処理・出力競合検査、SBUS入力範囲など。
- `i26-pico-firmware` は今回の移植元ではありません。ToF、RGB LEDなどの追加機能は含みません。

### 現行基板の配線（i-26 準拠）

| 出力 | 元のPico Copter | このリポジトリ（i-26準拠） |
|---|---:|---:|
| Rear Left | GPIO 2 | GPIO 2 |
| Rear Right | GPIO 3 | GPIO 3 |
| Front Left | GPIO 4 | **GPIO 14** |
| Front Right | GPIO 5 | **GPIO 15** |
| Servo | GPIO 6 | GPIO 6 |

**現行基板は i-26 の実装ピン配置です。** FL/FR は GPIO 14/15 を使用し、旧基板の GPIO 4/5 には戻しません。i-26 側の README に書かれた FL/FR=GPIO 4/5 は実装と一致していません。

ミキシングの `P_com` / `Q_com` の符号・構造は i-26 と同じで、`R_com` の符号のみ Pico Copter のものです。補正項には **`0.9 / 11.1`** を適用し、スロットル基準出力は停止解除後から全開まで **`0.10〜0.95`** に割り当てます。旧版の `T_ref * 0.9` は飽和のため使いません。

プロペラを外して初期動作を確認してください。コードの移植は飛行安全性やビルドの成功を保証するものではありません。

## RC link failsafe

SBUS受信安全処理（`firmware/pico_copter/sbus_parser.hpp`）を追加しています。

- 有効なSBUSフレームが100 ms届かない場合、最小ESC指令へ移行してアーム解除
- SBUSのフェイルセーフフラグを検出した場合、直ちに安全状態に移行
- 単発のフレームロストは有効受信に数えず、100 msタイムアウトで判定
- 通信復帰後はスロットル低・アーム操作解除を200 ms継続し、改めて通常のアーム操作を必要とする
- 起動直後も受信確認まではアームできない
- CIでSBUSフレーム解析と復帰判定のテストを実施

**制限:** 停止指令はESCへのDShot 0であり、モータの電源遮断ではありません。受信機が古いスティック値を正常なSBUSフレームとして送信し続け、フェイルセーフを通知しない設定の場合、通信断を判別できません。受信機の設定を確認し、プロペラを外して検証してください。

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

## DShot通信方式の選択（プログラム内で指定）

ファームウェアの [`firmware/pico_copter/dshot_config.hpp`](firmware/pico_copter/dshot_config.hpp) にある**2か所の設定**で、通信方式と速度を選びます。選択は**ビルド時に固定**され、実行中の自動切換えはありません。

```cpp
#ifndef DSHOT_RATE_KBPS
#define DSHOT_RATE_KBPS 300      // 150 / 300 / 600 / 1200
#endif

#ifndef DSHOT_BIDIRECTIONAL
#define DSHOT_BIDIRECTIONAL 1    // 1=双方向、0=単方向
#endif
```

| 選択値 | ESCへの指令 | ESC返信とeRPM |
|---|---|---|
| `DSHOT_BIDIRECTIONAL=1`（初期値） | 反転信号・反転CRCの双方向DShot | 受信する。返信が欠けてもモータ指令は続ける |
| `DSHOT_BIDIRECTIONAL=0` | 通常極性・通常CRCの単方向DShot | 受信しない。eRPMは常に無効 |

単方向用の送信PIOは従来版を [`dshot_oneway.pio`](firmware/pico_copter/dshot_oneway.pio) に分離しました。双方向用の [`dshot_tx.pio`](firmware/pico_copter/dshot_tx.pio) は変更していません。モードの切換えでPID・EKF・SBUS・ミキサ・CH5ログ・サーボPWM・400 Hz制御周期は変わりません。

ヘッダを編集せずにローカルビルドで一時的に選択する場合は、`-DDSHOT_BIDIRECTIONAL=0` または `1` をCMakeへ渡すこともできます。通常はヘッダを編集してGitHubへ反映するだけで、GitHub Actionsが**選択された1方式のUF2**を再ビルド・公開します。配布ページに通信方式を選ぶ操作はありません。既存のUF2のダウンロードURLは維持します。

**検証状況：** 実機でeRPM取得を確認した構成は、初代Pico・双方向DShot300です。単方向PIOは過去のDShot300版の実装をもとにしていますが、今回統合した新しいファームウェアは**実機で再検証が必要**です。変更後は必ずプロペラを外して動作を確認してください。単方向時はeRPMを取得できず、USBログは`mode=oneway`、`valid=0`、`eRPM=-1`となります。

## DShot通信速度の切り替え（150 / 300 / 600 / 1200）

**速度設定は1か所だけです。** [`firmware/pico_copter/dshot_config.hpp`](firmware/pico_copter/dshot_config.hpp) の次の行を書き換え、再ビルドしてください。

```cpp
#ifndef DSHOT_RATE_KBPS
#define DSHOT_RATE_KBPS 300  // ここを150 / 300 / 600 / 1200に変更
#endif
```

| 設定 | 送信ビット速度 | 16ビットの送信時間 | PIOクロック |
|---|---:|---:|---:|
| `150` | 150 kbit/s | 約106.7 µs | 6 MHz |
| `300` | 300 kbit/s | 約53.3 µs | 12 MHz |
| `600` | 600 kbit/s | 約26.7 µs | 24 MHz |
| `1200` | 1200 kbit/s | 約13.3 µs | 48 MHz |

双方向モードではこの設定に連動して、PIOクロック分周比、約25 µsの送受信折り返し待ち、ESC返信開始を検出する約43 µsの待ち時間、USBシリアルの`#DShot150`/`#DShot300`等の表示が変わります。`dshot_tx.pio`の実機動作確認済み命令列は維持し、返信待ちの`SET Y`命令のみ（PIO起動前に）ビット速度に応じて調整します。

**どの速度でも、上記で選んだ通信方式が適用されます。** 400 Hzの制御・モータ指令更新周期、100 Hz姿勢制御、PID/EKF、スロットル範囲、フェイルセーフは変わりません。従来のeRPM取得実績があるのは**初代PicoのDShot300のみ**で、150/600/1200の実機通信・受信成功率は未確認です。ESCの対応速度を確認し、プロペラなしで波形・スロットル・eRPMを試してください。

独立したビルドフォルダで速度を試す場合はソースを変更せず、`cmake -S firmware/pico_copter -B build-600 -DPICO_SDK_PATH=... -DPICO_BOARD=pico -DDSHOT_RATE_KBPS=600` でも指定できます。CMakeの指定値はヘッダ設定より優先します。通常のGitHub Actions/Pages公開ビルドはヘッダの値を使用し、配布ページの説明にもその速度を表示します。

## RP2040で動作確認した双方向DShot300（保存版）

**2026-10-16に初代Raspberry Pi Pico（RP2040）、レベルシフタを実装しない別基板、ESC＋モータ1組で、DShot300指令・eRPM返信の取得に成功しました。** スロットル操作に対するeRPM変化も良好との報告を受けています。4基同時のeRPM、飛行安定性、Pico 2での双方向受信は未検証です。

- **動作確認済みの元版（変更前）:** [保存ブランチ](https://github.com/temesotejam/pico-copter-fc/tree/archive/pico-rp2040-bidir-dshot300-erpm-working-20261016)（コミット`8c8dc12`）
- **再現に必要な情報:** [DShot300ベースライン記録](firmware/pico_copter/DSHOT300_BASELINE.md)
- **現在の実装:** 通信速度を変更できる双方向DShot（標準300、反転波形・反転CRC）、GPIOを送信後に入力へ切替、ESC返信をPIOで受信
- **以前の一方向版:** コミット`2ed49dc`（比較用。現在のmainは双方向です）
- **Pico用UF2:** [pico_copter.uf2](https://temesotejam.github.io/pico-copter-fc/firmware/pico_copter.uf2)（`main`更新時に再ビルドされる最新版。固定版のダウンロード先ではありません）

### 基板・信号の対応

| モータ | GPIO | Pico物理ピン |
|---|---:|---:|
| FR（前右） | 15 | 20 |
| FL（前左） | 14 | 19 |
| RR（後右） | 3 | 5 |
| RL（後左・1組の動作確認で使用） | **2** | **4** |
| サーボ（従来PWM） | 6 | 9 |

PIO0の4ステートマシンがモータ信号を扱い、PWMスライス7は出力ピンとは独立に400 Hzの制御周期を生成します。100 Hzの姿勢制御、PID・EKF・モータミキシング・SBUSフェイルセーフ・CH5の38項目リングログは維持しています。eRPMはUSB診断のみに使用します。

### 双方向モードでESC返信を受信できない場合

**双方向モードでは返信テレメトリは任意です。** 4モータのDShot指令は400 Hzの制御計算から送られ、eRPMの有無・数値はPID、EKF、アーム判定、モータミキシング、SBUSフェイルセーフに使っていません。

- ESC未接続・停止中の返信なし・一時的なCRC/GCRエラー・通信線のノイズの場合でも、**返信完了を待つブロッキング処理をせず、次の400 Hz指令送信を続けます**。
- 正常なeRPMが250 ms以上来なければ、USBログの`valid=0`、`eRPM=-1`となるだけです。最後のeRPMを飛行制御に使用したり、返信がないだけで停止やアーム解除をしたりしません。
- RX FIFOに古い返信が残っても、毎周期すべてのチャンネルを最大4語ずつ空にし、送信を不必要に停止させないようにしています。`stray`は余分・遅延返信の破棄累計です。
- `timeout`、`gcr`、`crc`等は診断のみです。`busy`は実際に送信用FIFOが空でなく新しいDShot指令を投入できなかった回数で、**返信欠落とは区別**します。
- **通常の一方向DShotへは自動切替しません。** 送信する波形・CRCは常に双方向DShot300形式です。そのため、通常DShotのみ対応のESCがモータ指令を理解できるとは限りません。今回の改善は「双方向DShot対応ESCの返信が一時的・継続的に欠落しても制御を妨げない」ものです。
- ESCの電源故障やモータ停止などハードウェア自体の不具合をソフトウェアで解決するものではありません。送信FIFOの本当のハング、GPIO故障、電源断を含む全故障時の動作保証ではありません。

### スロットルとUSB診断

制御側の走行時の`duty=0.10〜0.95`をDShotの`48〜2047`へ対応させ、停止は常に`0`です。既存の停止境界（CH3約587）は変更していません。DShot指令は回転数を直接指定するRPM制御ではありません。

USB CDCに`#DBG STATE`（アーム状態）、`#DBG SBUS`（CH1〜6と受信鮮度）、`#DBG MIX`（推力と各軸補正）、`#DShot<速度>`（各モータの指令値・eRPM・受信状況）を出力します。eRPM返信の検証では`RL mode=bidir`、`valid`、`eRPM`、`rx`、`timeout`、`gcr`、`crc`、`stray`を確認してください。詳しい項目は[ファームウェアREADME](firmware/pico_copter/README.md)にあります。

**安全上の条件:** レベルシフタや別の出力ドライバが同じESC信号線を駆動しない配線で使用してください。PicoのGPIOは5 V耐性ではないため、ESC側の信号線が3.3 V系であることを確認します。プロペラを外し、まずモータ1組で検証してください。

## USB診断の整理

通常のUSB CDCデバッグ表示を **`firmware/pico_copter/debug_usb.cpp`** に集約しています。`debug_usb.hpp` がその呼び出し口と表示のON/OFF設定です。`pico_copter.cpp` は起動時に `debug_usb::startup_countdown()`/`startup_complete()` を呼び、メインループで `debug_usb::poll()` を呼ぶだけです。IMU初期化に失敗した場合も `sensor.cpp` から `debug_usb::sensor_not_found()` を呼びます。

| 変更したいこと | 修正箇所 |
|---|---|
| SBUS・IMU・推定姿勢・EKF・PID・DShotの表示項目や書式 | `debug_usb.cpp` |
| 通常の周期的USB診断をまとめてOFF | `debug_usb.hpp` の `kEnablePeriodicDiagnostics` |
| IMU・地磁気・姿勢角・EKFの表示を個別にON/OFF | `debug_usb.hpp` の `kEnableImu`、`kEnableMag`、`kEnableAttitude`、`kEnableEkf` |
| SBUS・PID／ミキサ・DShotの表示を個別にON/OFF | `debug_usb.hpp` の `kEnableSbus`、`kEnablePidMixer`、`kEnableDShot` |
| RC状態変化メッセージをOFF | `debug_usb.hpp` の `kEnableRcStateMessages` |
| 送受信・制御データの取得処理 | 各機能の `*_debug_snapshot()`（出力自体は担当しない） |

ディスアーム時は約0.5秒ごと、アーム時は約1秒ごとの表示を維持します。`#DBG IMU`はm/s²、`#DBG GYRO`はrad/s、`#DBG MAG`は正規化ベクトル、`#DBG ATT`は基準角差し引き後と差し引き前の姿勢角（°）、`#DBG EKF`はクォータニオンと内部バイアスを出力します。各行の`age_ms`で取得からの経過時間が分かります。**デバッグの`printf`は400 Hz割り込みやコア1の制御タスクには置いていません。** USB未接続時、およびCH5記録データ出力時は通常診断を表示しません。

**例外：CH5の38項目リングログ・P/Q/R共分散・PIDゲインのエクスポートだけは `control.cpp` の`log_output()`に残しています。** これは実験用データであって通常のデバッグ表示ではなく、既存の行形式と書き出し手順を変えないためです。過去の呼び出されていない単発`output_data()`、`output_sensor_raw_data()`、`printData()`と無効なサンプルの`printf`は整理しました。

## ファームウェアとビルダーの分離

マイコンへ書き込むプログラムは、次の1フォルダへまとめています。

```text
firmware/pico_copter/
```

他人と共有するときは、このフォルダをそのまま取り出せます。フォルダ内には、プログラム本体、独立ビルド用`CMakeLists.txt`、ビルド手順が入っています。

```text
firmware/pico_copter/
├─ pico_copter.cpp / .hpp, debug_usb.cpp / .hpp
├─ control.cpp / .hpp
├─ ekf.cpp / .hpp
├─ sensor.cpp / .hpp
├─ pwm.cpp / .hpp, dshot_tx.pio, dshot_protocol.hpp, dshot_bidir_decode.hpp
├─ radio.cpp / .hpp
├─ lsm9ds1_reg.c / .h
├─ CMakeLists.txt
└─ README.md
```

GitHub Actions、キャッシュ、Web配布画面などは、ファームウェアフォルダの外側に分離しています。

## 飛行制御ソースZIP（ローカル開発用）

[UF2配布ページ](https://temesotejam.github.io/pico-copter-fc/) に「**飛行制御ソース一式（ZIP）**」のダウンロードを追加しています。Gitリポジトリ全体のZIPとは異なり、`firmware/pico_copter` にある**実際の飛行制御コード・ヘッダ・PIO・CMake設定・DShotライセンス**に加え、**ビルドで使用しているEigen 3.4.0のヘッダとライセンス**、`PORTABLE_BUILD.md` をまとめます。Webアプリ、GitHub Actions、履歴資料、過去のビルド生成物は含みません。

ZIPには、ビルド用ツールチェーン（**Pico SDK 2.3.0、ARM GCC、CMake、任意でNinja**）は含まれません。これらを既にインストールしたPCなら、ZIPを展開した`pico_copter`フォルダを別の場所へ移してビルドできます。**Eigenを改めてネットから取得する必要はありません。**

```bash
cd pico_copter
cmake -S . -B build -G Ninja -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_BOARD=pico -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
# → build/pico_copter.uf2
```

次回以降、PIDゲインやDShot設定を変更した際は、`build`フォルダを削除せず、**`cmake --build build --parallel`のみ**を実行すれば増分ビルドできます。Pico 2用は別のビルドフォルダと`-DPICO_BOARD=pico2`を指定します。

GitHub ActionsはZIPを生成した後、**リポジトリ外の一時ディレクトリへZIPを展開し、同梱のEigenを利用してPico用UF2を再ビルド**します。ZIPのダウンロードファイル、サイズ、SHA-256はサイト上に表示され、配布UF2と同じソースコミットに基づきます。

## github.devでファームウェアを編集する

通常のGitHubで編集したいファイルを開き、キーボードの`.`キーを押すと、同じリポジトリを`github.dev`で開けます。

主な編集対象は次のフォルダです。

```text
firmware/pico_copter/
```

編集後は、左側のソース管理から変更内容を入力して`Commit & Push`を実行します。`main`へ反映されるとGitHub Actionsが自動的にコンパイルし、成功したUF2をPagesへ公開します。Pull Requestではコンパイル確認だけを行います。

ファームウェアの変更履歴はGitで管理されるため、以前の状態はGitHubのコミット履歴から確認・復元できます。

## リポジトリ構成

```text
pico-copter-fc/
├─ firmware/
│  └─ pico_copter/              共有・持ち出し可能なファームウェア
├─ web/                         UF2配布ページ
├─ .github/workflows/
│  └─ build-and-deploy.yml      クラウドビルドとPages公開
├─ .gitignore
└─ README.md
```

元のPico Copterリポジトリとのサブモジュール連携や自動同期はありません。このリポジトリ内のファイルだけで管理します。

## GitHubでUF2を生成する

1. `Settings`を開く
2. `Pages` → `Build and deployment`を開く
3. `Source`を`GitHub Actions`に設定する
4. `Actions`を開く
5. `Build Pico Copter UF2 and deploy Pages`を選ぶ
6. `Run workflow`を押す
7. 成功後、`Settings` → `Pages`に表示される公開先を開く

`main`のファームウェアやWorkflowを変更した場合も自動実行されます。Pull Requestではコンパイル確認だけを行い、Pagesへの公開は行いません。

## ファームウェアフォルダ単体でビルドする

`firmware/pico_copter`へ移動し、Pico SDKを指定してビルドします。

```bash
cd firmware/pico_copter

cmake -S . -B build \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DPICO_BOARD=pico \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --parallel
```

生成物は次です。

```text
firmware/pico_copter/build/pico_copter.uf2（設定した基板向け）
```

## 依存関係

| 項目 | 固定値 |
|---|---|
| 対象ボード | `pico` / `pico2` |
| Pico SDK | `2.3.0` |
| Eigen | `3.4.0` |
| Eigenコミット | `3147391d946bb4b6c68edd901f2add6ac1f31f8c` |
| C++ | C++17 |

ファームウェアフォルダのCMakeは、`EIGEN3_INCLUDE_DIR`が指定されていない場合、公式Eigenの固定コミットを自動取得します。既にEigenを持っている場合は、次のように指定できます。

```bash
-DEIGEN3_INCLUDE_DIR=/path/to/eigen
```

GitHub ActionsではEigenを別に取得してキャッシュし、そのパスをファームウェア側のCMakeへ渡します。

## 高速化

GitHub Actionsでは次をキャッシュします。

```text
Pico SDK
Eigen 3.4.0
ccacheによるC/C++のコンパイル結果
```

初回、キャッシュ削除後、依存バージョン変更後は通常どおり時間がかかります。

## ビルド結果

成功時は次を生成します。

```text
pico_copter.uf2
pico_copter.uf2.sha256
size.txt
manifest.json
configure.log
build.log
```

通常Artifactと診断ログの保持期間は1日です。GitHub Pagesには最後に成功したUF2が公開されます。

## コンパイルエラーについて

編集した`.cpp/.hpp/.c/.h`にコンパイルエラーやリンクエラーがある場合、Workflowのログへそのまま表示されます。Workflowがプログラム本体を自動修正することはありません。

## UF2の書き込み

1. Pagesで初代Pico（RP2040）を選び、`pico_copter-pico.uf2`をダウンロードする
2. Picoの`BOOTSEL`ボタンを押したままUSB接続する
3. 表示された`RPI-RP2`ドライブへUF2をコピーする
4. コピー後、Picoが自動的に再起動する
