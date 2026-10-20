import { parseIntelHex, Stk500v1, flashAndVerify } from "./uno-flasher-core.js";

const $ = id => document.getElementById(id);
const hexInput = $("hex-file");
const safety = $("safety-check");
const button = $("flash-button");
const status = $("upload-status");
const progress = $("upload-progress");
const info = $("hex-info");
let selectedFirmware = null;
let running = false;

function display(message, error = false) {
  status.textContent = message;
  status.className = error ? "error" : "";
}
function refresh() {
  button.disabled = running || !selectedFirmware || !safety.checked || !("serial" in navigator);
}
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));

class WebSerialTransport {
  constructor(port) {
    this.port = port;
    this.queue = [];
    this.waiters = [];
    this.error = null;
    this.reader = null;
    this.writer = null;
    this.pumpPromise = null;
  }

  start() {
    this.reader = this.port.readable.getReader();
    this.writer = this.port.writable.getWriter();
    this.pumpPromise = this.pump();
  }

  async pump() {
    try {
      while (true) {
        const { value, done } = await this.reader.read();
        if (done) break;
        for (const byte of value) {
          if (this.waiters.length) this.waiters.shift().resolve(byte);
          else {
            this.queue.push(byte);
            if (this.queue.length > 4096) this.queue.shift();
          }
        }
      }
    } catch (error) {
      this.error = error;
    } finally {
      const error = this.error || new Error("シリアル接続が終了しました。");
      for (const pending of this.waiters.splice(0)) pending.reject(error);
    }
  }

  clearInput() { this.queue.length = 0; }
  async send(bytes) {
    if (this.error) throw this.error;
    await this.writer.write(bytes);
  }
  readByte(timeoutMs = 600) {
    if (this.queue.length) return Promise.resolve(this.queue.shift());
    if (this.error) return Promise.reject(this.error);
    return new Promise((resolve, reject) => {
      const pending = {
        resolve: byte => { clearTimeout(timer); resolve(byte); },
        reject: error => { clearTimeout(timer); reject(error); }
      };
      const timer = setTimeout(() => {
        this.waiters = this.waiters.filter(item => item !== pending);
        reject(new Error("UNOからの応答がタイムアウトしました。COMポートと接続を確認してください。"));
      }, timeoutMs);
      this.waiters.push(pending);
    });
  }
  async close() {
    try { if (this.reader) await this.reader.cancel(); } catch { /* disconnected */ }
    try { if (this.pumpPromise) await this.pumpPromise; } catch { /* disconnected */ }
    try { if (this.reader) this.reader.releaseLock(); } catch { /* disconnected */ }
    try { if (this.writer) this.writer.releaseLock(); } catch { /* disconnected */ }
    try { await this.port.close(); } catch { /* disconnected */ }
  }
}

async function pulseReset(port, io) {
  // The ATmega16U2 USB bridge on an original UNO R3 drives the reset
  // capacitor via DTR. False -> true causes an active-low DTR edge.
  await port.setSignals({ dataTerminalReady: false, requestToSend: false });
  await sleep(90);
  io.clearInput();
  await port.setSignals({ dataTerminalReady: true, requestToSend: false });
  await sleep(75);
}

async function connectBootloader(port, io, stk) {
  for (let attempt = 0; attempt < 9; ++attempt) {
    if (attempt % 3 === 0) await pulseReset(port, io);
    io.clearInput();
    try {
      await stk.sync();
      return;
    } catch {
      await sleep(65);
    }
  }
  throw new Error("UNO R3のOptibootに同期できません。UNO R3、COMポート、ブートローダー、115200bpsを確認してください。");
}

async function loadSelectedFile() {
  selectedFirmware = null;
  progress.value = 0;
  const file = hexInput.files?.[0];
  if (!file) {
    info.textContent = "ファイル未選択";
    refresh();
    return;
  }
  try {
    if (!file.name.toLowerCase().endsWith(".hex"))
      throw new Error("拡張子 .hex のファイルを選んでください。");
    if (file.size > 1_000_000)
      throw new Error("HEXファイルが大きすぎます（1MB上限）。");
    const parsed = parseIntelHex(await file.text());
    selectedFirmware = parsed;
    info.textContent = `${file.name}：${parsed.bytes.toLocaleString()} バイト、${parsed.pages.length} ページ（検証済みのHEX形式）`;
    display("HEXの形式を確認できました。UNOへの書き込みはまだ行っていません。");
  } catch (error) {
    info.textContent = `読み込み失敗：${error.message}`;
    display("正しいUNO用4-way HEXを選んでください。", true);
  }
  refresh();
}

async function handleFlash() {
  if (!selectedFirmware || running || !safety.checked) return;
  if (!navigator.serial) {
    display("Web Serial非対応ブラウザです。WindowsのChromeかEdgeから開いてください。", true);
    return;
  }
  running = true;
  hexInput.disabled = true;
  safety.disabled = true;
  progress.value = 0;
  refresh();
  let port;
  let io;
  try {
    display("UNO R3のCOMポートを選んでください。");
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: "none", flowControl: "none" });
    io = new WebSerialTransport(port);
    io.start();
    const stk = new Stk500v1(io);
    display("UNOをリセットしてOptibootと同期しています…");
    await connectBootloader(port, io, stk);
    display("UNOの型番を確認し、書き込みと読み戻し検証を開始します…");
    await flashAndVerify(stk, selectedFirmware, (done, total) => {
      progress.value = Math.round(done * 100 / total);
      display(`書き込み・読戻し検証：${done} / ${total} ページ（${progress.value}%）`);
    });
    display("書き込み・読み戻し検証とも成功しました。USBを挿し直してからBLHeliSuite32で設定を読み出してください。");
  } catch (error) {
    if (error.name === "NotFoundError")
      display("COMポートの選択がキャンセルされました。書き込みは開始されていません。");
    else display(`書き込み失敗：${error.message}`, true);
  } finally {
    if (io) await io.close();
    else if (port) { try { await port.close(); } catch { /* unopened */ } }
    running = false;
    hexInput.disabled = false;
    safety.disabled = false;
    refresh();
  }
}

hexInput.addEventListener("change", () => { void loadSelectedFile(); });
safety.addEventListener("change", refresh);
button.addEventListener("click", () => { void handleFlash(); });
if (!("serial" in navigator)) {
  display("Web Serial APIを利用できません。WindowsのGoogle ChromeまたはMicrosoft EdgeでHTTPSページを開いてください。", true);
}
refresh();
