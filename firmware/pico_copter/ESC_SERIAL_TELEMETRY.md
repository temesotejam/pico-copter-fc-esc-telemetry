# 共通基板：Pico / Pico 2の双方向DShot・ESC専用TXテレメトリ

この資料は**GP16＝SN74LVC245AのDIR（IC 1番ピン）、GP18＝/OE（IC 19番ピン）、GP17＝ESC TXパッドのUART RX**という同じ配線で、初代Pico（RP2040）とPico 2（RP2350）を使うためのものです。

**検証状況：** 2026-10-27、**Pico 2**で4基同時の双方向DShot300・専用UART受信をベンチ確認。**初代Picoをこの回路で使用する実機試験は未完了**です。実飛行の保証はありません。

## GPIOとハードウェア

| 接続信号 | GPIO | 物理ピン | 備考 |
|---|---:|---:|---|
| FR / FL（DShot） | GP15 / GP14 | 20 / 19 | FCが制御 |
| RR / RL（DShot） | GP3 / GP2 | 5 / 4 | FCが制御 |
| 予備2回線 | GP20 / GP21 | 26 / 27 | Pico 2のみDShot 0送信。初代Picoは弱プルダウン入力 |
| **DIR（SN74LVC245A 1番ピン）** | **GP16** | **21** | 1＝Pico→ESC、0＝ESC→Pico |
| **ESC TX UART RX** | **GP17** | **22** | 115200 bps / 8N1、別の保護入力 |
| **/OE（SN74LVC245A 19番ピン）** | **GP18** | **24** | 1＝High-Z、0＝出力有効 |
| SBUS RX | GP1 | 2 | UART0受信 |
| サーボ PWM | GP6 | 9 | 従来PWM |

SN74LVC245AはA側をマイコン、B側をESCに接続してください。**/OEは外付け抵抗で3.3 Vへプルアップ**し、電源投入時・プログラム未起動時にレベルシフタの全出力が無効になるようにします。DIR/OEの固定配線があればマイコン制御と競合しないよう除去します。共通GNDも必要です。

**ESCの専用TXパッドは、上記の双方向DShot用SN74LVC245Aを通りません。** 入力電圧を測定し、3.3 V互換の別の保護回路からGP17へ入力します。Pico GPIOは5 V耐性ではありません。**複数ESCのTXパッドを仕様確認なしに直結しないでください。** 要求を順番に出してもpush-pull出力は電気的に競合するおそれがあります。

## 2つの情報取得経路

**双方向DShot300:** 各モータのDShotピンがまず指令を送信し、方向切り替え後にESCからeRPMを受信します。成功／失敗はUSBの`#DShot300 FR/FL/RR/RL`に表示されます。返信がなくてもモータ指令は継続します。

**専用UART:** GP17で10バイトのKISSテレメトリを受け取ります。情報はESC温度（°C）、電圧（0.01 V単位）、電流（0.01 A単位）、消費容量（mAh）、電気回転数（eRPM、プロトコル値に係数100）。UARTとDShot eRPMは独立した計測値です。

- DShotのテレメトリ要求ビットを、推力指令がゼロでないESCへ1基ずつ付けます。**要求間隔40 ms以上、返答は8 ms以内**に限定します。
- KISS UARTパケットには送信元IDがないため、**要求中のESCのみ**へパケットを割り当てます。CRC8（多項式0x07）と妥当な数値範囲・継続したフレームの整合性を検証します。最初の一致候補は`confirm`として保留されます。
- 500 msより古いテレメトリは`valid=0`になり、値は無効として扱います。ESC側の**Auto TelemetryをOFF**にすることが、この要求応答方式で正しく識別する前提です。
- 双方向DShotの返信がない場合でも、UART専用TXの要求は送れます。どちらの取得値も飛行制御には使用しません。

## PicoとPico 2の違い（共通回路のまま使用）

| 項目 | 初代Pico / RP2040 | Pico 2 / RP2350 |
|---|---|---|
| DShotのPIO | PIO0に4ステートマシン | PIO0に4＋PIO1に2ステートマシン |
| 専用UART RX | PIO1の1ステートマシン | PIO2の1ステートマシン |
| DIR/OEのPIO | PIO1の1ステートマシン | PIO2の1ステートマシン |
| GP20/GP21 | GPIO入力＋弱プルダウン。**STOPパケットなし** | 双方向DShotのSTOPパケット |
| 方向制御のタイミング | **PIO0→PIO1のCPU順次起動。実機波形確認必須** | PIO間ハード同期。2026-10-27通信実機確認済み |

初代PicoのPIO1命令メモリは、UART受信8命令＋方向制御24命令＝**32命令**で上限です。RP2040はRP2350のPIO間ハードウェア同時起動を持たないため、Pico用の`pico_shifter_rp2040.pio`は送受信切り替えのタイミングを調整しています。**コンパイル成功だけではタイミングの正しさも電気的安全性も保証できません。**

Pico 2の成功版は[保存ブランチ](https://github.com/temesotejam/pico-copter-fc-esc-telemetry/tree/archive/pico2-bidir-dshot300-uart-gp17-working-20261027)に固定されています。

## USBログの読み方

- `#DShot300 FR/FL/RR/RL`：`mode=bidir`、`cmd`、`pkt`、`eRPM`、`valid`、`rx`、`timeout`、`gcr`、`crc`、`busy`
- `#ESC_UART FR/FL/RR/RL`：`requests`、`rx`、`req_rx`、`confirm`、`timeout`、`age_ms`、`valid`、温度・電圧・電流・mAh・eRPM
- `#ESC_UART SUM`：共有UARTの合計`bytes`、`plausible`、`crc_windows`、`range_reject`、`unassigned`、`gaps`

`crc_windows`はスライディング窓のCRC不一致回数で、要求数に対する失敗数ではありません。`valid=0`／`eRPM=-1`は未取得や期限切れを意味し、0 eRPMの有効計測とは異なります。ESC電源の抜き差し中はタイムアウトが当然増えるため、**実際に接続されている区間のカウンタ差分**で通信を評価します。

## 初回試験（必ずプロペラなし）

1. ESCをまだ接続せず、/OE外付けプルアップ、GPIO電圧、GP16/GP18の方向制御波形とDShot送信をオシロスコープで確認します。**特に初代PicoのPIO間タイミングは未実機検証です。**
2. ESCのTX電圧を測り、必要な入力保護を組んでからESCをまず1基だけ接続します。
3. 起動ログ`#DShot CONFIG rate=300 kbps mode=bidir control=400Hz`、DShot STOP、UART、eRPM返信を確認します。
4. アーム後は小さな推力で`cmd`、`eRPM`、`#ESC_UART`の変化を確認します。SBUS通信断時はDShot 0とアーム解除を確認します。
5. 複数ESCでは専用TXの電気的な接続方法を個別に検証します。飛行試験の前に各軸、回転方向、電源条件を検証してください。

この仕様は電気的／飛行的な安全性を保証しません。
