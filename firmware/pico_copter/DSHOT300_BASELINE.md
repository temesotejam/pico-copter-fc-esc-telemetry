# 双方向DShot300 / eRPM：2026-10-16 動作確認ベースライン

## 保存対象と確認範囲

- **動作確認日：** 2026-10-16
- **マイコン：** 初代Raspberry Pi Pico（RP2040）。Pico 2（RP2350）ではない
- **基板：** 一方向レベルシフタ（SN74LVC245APWR）を実装せず、ジャンパで信号を接続した別基板
- **対象ESC：** MAD MOTOR BLHeli_32 70A 2–8S Single ESC（会話で使用を確認）。ファームウェアの詳細バージョンは未確認
- **モータ：** 当初の低KVモータではなく、交換後の高KVモータ。型番・KV値は未記録
- **実際に確認したチャンネル：** GPIO2 = RL、ESC＋モータ1組、プロペラなし
- **動作結果：** ユーザーから**eRPM取得に成功**し、スロットル操作への追従も良好との報告あり
- **未検証：** 4モータ同時、飛行、長時間のeRPM成功率、送受信ジッタ、Pico 2でのeRPM取得

### 変更前の動作確認済みソース

- **固定コミット：** `8c8dc12eccf2c3aface2deb3d233d23d1701714d`
- **保存ブランチ：** [archive/pico-rp2040-bidir-dshot300-erpm-working-20261016](https://github.com/temesotejam/pico-copter-fc/tree/archive/pico-rp2040-bidir-dshot300-erpm-working-20261016)
- **GitHub Actions成功実行：** [38854374959](https://github.com/temesotejam/pico-copter-fc/actions/runs/38854374959)
- **保存上の注意：** ブランチが保存するのは**ソースコードと履歴**です。GitHub PagesのUF2は`main`の最新ビルドで上書きされ、Actionsのアーティファクトも保持期限があります。後日この版に戻すときは上記コミットをチェックアウトし、同じSDKとEigenで再ビルドしてください

## DShotの送受信仕様

| 項目 | 現行値・意味 |
|---|---|
| モータGPIO | FR=15、FL=14、RR=3、RL=2 |
| サーボ | GPIO6、PWMのまま |
| DShot形式 | **双方向DShot300**（反転信号・反転CRC） |
| スロットルデータ | STOP=0、回転指令=48〜2047、予約値1〜47は使用しない |
| DShot命令周期 | 400 Hz（2.5 msごと） |
| DShotビット速度 | 300 kbit/s |
| PIOクロック | 12 MHz（40 PIOクロック/ビット） |
| 送信時間 | 約53.3 µs（16ビット） |
| 送信後待機 | 約25 µs |
| 受信開始検出 | 最大約43 µs（従来の約8 µsから延長） |
| 返信データ | 約21ビット、NRZI/GCR形式でeRPMを復号 |
| telemetry bit | 0（双方向は同じ信号線上で返信を受け取る） |
| CPU側の受信 | 400 Hz割り込みで**前周期のRX FIFOのみ確認**。返信待ちのブロッキングなし |
| eRPMの利用 | USBシリアル診断のみ。PID・EKF・モータ制御・CH5ログには入力しない |

`dshot_tx.pio`の命令列・タイミング、`pwm.cpp`のPIO構成、`dshot_bidir_decode.hpp`の復号器は、eRPMが取得できた構成の中核です。試験根拠を失わないよう、保存版の整理作業では**実行部分を変更しません**。制御周期も同じです。

### ピン配線

| モータ | PicoのGPIO | Pico物理ピン |
|---|---:|---:|
| RL（検証済み） | 2 | 4 |
| RR | 3 | 5 |
| FL | 14 | 19 |
| FR | 15 | 20 |
| サーボ | 6 | 9 |

ESCの信号電圧が3.3 V系の範囲内であること、GNDを共通にすること、他の出力ドライバが信号線に残っていないことを確認します。**PicoのGPIOは5 V耐性ではありません。** 双方向ではESCとPicoの双方が同じ信号線を駆動するため、回路変更時の競合に注意してください。

## ソフトウェアの動作とログ

`control.cpp`の停止判定は変更しません。`T_ref/BATTERY_VOLTAGE < Disable_duty`の場合、全モータのDShot指令を0にします。運転時のモータ出力`duty=0.10〜0.95`をDShot指令`48〜2047`へ変換します。

USBログの例（**項目名の説明用。数値は実測ログではありません**）：

```text
#DBG STATE t_ms=... arm=2 lock=2 rc_latch=0 ...
#DBG SBUS ch1=... ch2=... ch3=... healthy=1 ...
#DBG MIX active=1 T=... P=... Q=... R=... RL=...
#DShot300 RL mode=bidir duty=... cmd=... pkt=... eRPM=... valid=1 rx=.../... (...) timeout=... gcr=... crc=... other=... busy=...
```

- `cmd`：PIO TX FIFOへ最後に投入したDShot指令。物理信号を測定した値ではありません
- `pkt`：反転CRCを含む**論理フレーム**。実際にはこのフレームをビット反転して送信します
- `eRPM`：受信した電気的回転速度（電気的rpm）。機械的rpmへの変換にはモータ極対数が必要です
- `valid`：最後の有効なeRPMが250 ms以内なら1。0の場合`eRPM=-1`と表示
- `rx`：累積成功回数と累積送信回数。`rx=0`が続く場合は波形の返信レベルや復号状態を確認
- `timeout`：返信を取得できなかった回数、`gcr`/`crc`：復号エラー数、`other`：その他の形式のテレメトリ、`busy`：送信前にTX/RX FIFOが残っていた回数

## 変更を加える場合の順番

1. 保存ブランチと比較し、まずは`dshot_tx.pio`・`pwm.cpp`・`dshot_bidir_decode.hpp`の実行内容を変更しない
2. GitHub Actionsで`test_dshot.cpp`、`test_dshot_bidir.cpp`、`test_motor_mix_scaling.cpp`とPico向けビルドを実行する
3. **プロペラなし**でGPIO2のSTOP、スロットル変化、`valid=1`、`rx`増加とeRPM変化を確認する
4. 4基での検証や飛行は別の段階で行い、ベースライン実績として混同しない

ビルドに用いるPico SDKは2.3.0、Eigenは3.4.0（コミット`3147391d946bb4b6c68edd901f2add6ac1f31f8c`）、C++は17です。初代Pico用UF2は`pico_copter.uf2`です。
