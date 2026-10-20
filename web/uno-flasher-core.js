/* Arduino UNO R3 (ATmega328P/Optiboot) Intel HEX uploader helpers.
 * No third-party dependencies; runs locally in the user's browser.
 * Does not implement ESC programming or BLHeli proprietary functions.
 */
export const UNO_SIGNATURE = [0x1e, 0x95, 0x0f];
export const UNO_PAGE_SIZE = 128;
export const UNO_FLASH_LIMIT = 0x7e00; // Preserve 512-byte Optiboot section.

export function parseIntelHex(text) {
  if (typeof text !== "string" || text.length > 1_000_000)
    throw new Error("HEXファイルが大きすぎるか、文字列ではありません。");
  const memory = new Map();
  let base = 0;
  let eof = false;
  const lines = text.replace(/^\uFEFF/, "").split(/\r?\n/);
  for (let index = 0; index < lines.length; ++index) {
    const line = lines[index].trim();
    if (!line) continue;
    if (eof) throw new Error(`HEXのEOF以降にデータがあります（行${index + 1}）。`);
    if (!/^:[0-9a-f]+$/i.test(line) || (line.length - 1) % 2 !== 0)
      throw new Error(`HEXの形式が不正です（行${index + 1}）。`);
    const bytes = [];
    for (let p = 1; p < line.length; p += 2)
      bytes.push(parseInt(line.slice(p, p + 2), 16));
    if (bytes.length < 5 || bytes.length !== bytes[0] + 5)
      throw new Error(`HEXのレコード長が不正です（行${index + 1}）。`);
    if ((bytes.reduce((s, x) => s + x, 0) & 0xff) !== 0)
      throw new Error(`HEXのチェックサムが不正です（行${index + 1}）。`);
    const len = bytes[0];
    const offset = (bytes[1] << 8) | bytes[2];
    const type = bytes[3];
    const data = bytes.slice(4, 4 + len);
    if (type === 0x00) {
      for (let i = 0; i < len; ++i) {
        const address = base + offset + i;
        if (address < 0 || address >= UNO_FLASH_LIMIT)
          throw new Error("Arduino UNO R3のアプリ領域外です。ブートローダーは上書きしません。");
        if (memory.has(address) && memory.get(address) !== data[i])
          throw new Error(`重複アドレスのデータが矛盾しています: 0x${address.toString(16)}`);
        memory.set(address, data[i]);
      }
    } else if (type === 0x01) {
      if (len !== 0 || offset !== 0) throw new Error("EOFレコードが不正です。");
      eof = true;
    } else if (type === 0x04) {
      if (len !== 2 || offset !== 0) throw new Error("拡張リニアアドレスが不正です。");
      base = ((data[0] << 8) | data[1]) * 0x10000;
    } else if (type === 0x02) {
      if (len !== 2 || offset !== 0) throw new Error("拡張セグメントアドレスが不正です。");
      base = (((data[0] << 8) | data[1]) << 4);
    } else if (type === 0x03 || type === 0x05) {
      if (len !== 4) throw new Error("開始アドレスレコードが不正です。");
    } else {
      throw new Error(`未対応のIntel HEXレコード種別: ${type}`);
    }
  }
  if (!eof || memory.size === 0 || !memory.has(0))
    throw new Error("UNO用の完全なHEXファイルではありません（EOFまたは先頭アドレスがありません）。");
  const pageNumbers = [...new Set([...memory.keys()].map(a => Math.floor(a / UNO_PAGE_SIZE)))]
    .sort((a, b) => a - b);
  const pages = pageNumbers.map(pageNumber => {
    const address = pageNumber * UNO_PAGE_SIZE;
    const data = new Uint8Array(UNO_PAGE_SIZE).fill(0xff);
    for (let i = 0; i < UNO_PAGE_SIZE; ++i)
      if (memory.has(address + i)) data[i] = memory.get(address + i);
    return { address, data };
  });
  return { pages, bytes: memory.size, highestAddress: Math.max(...memory.keys()) };
}

// Transport must expose async send(Uint8Array) / readByte(timeoutMs).
export class Stk500v1 {
  constructor(transport) { this.io = transport; }

  async command(body, replyLength = 0, timeoutMs = 600) {
    await this.io.send(Uint8Array.from([...body, 0x20])); // CRC_EOP
    const prefix = await this.io.readByte(timeoutMs);
    if (prefix !== 0x14) throw new Error(`STK500: 応答開始が不正です (0x${prefix.toString(16)})`);
    const result = new Uint8Array(replyLength);
    for (let i = 0; i < replyLength; ++i)
      result[i] = await this.io.readByte(timeoutMs);
    const suffix = await this.io.readByte(timeoutMs);
    if (suffix !== 0x10) throw new Error("STK500: 応答終了が不正です。");
    return result;
  }

  async sync() { await this.command([0x30]); }
  async signature() { return this.command([0x75], 3); }
  async loadAddress(byteAddress) {
    if ((byteAddress & 1) !== 0 || byteAddress < 0 || byteAddress >= UNO_FLASH_LIMIT)
      throw new Error("UNOのアドレスが不正です。");
    const word = byteAddress >>> 1;
    await this.command([0x55, word & 0xff, (word >> 8) & 0xff]);
  }
  async writePage(data) {
    if (!(data instanceof Uint8Array) || data.length !== UNO_PAGE_SIZE)
      throw new Error("UNOのページサイズが不正です。");
    await this.command([0x64, 0x00, UNO_PAGE_SIZE, 0x46, ...data], 0, 1500);
  }
  async readPage() {
    return this.command([0x74, 0x00, UNO_PAGE_SIZE, 0x46], UNO_PAGE_SIZE, 1500);
  }
  async leave() { await this.command([0x51], 0, 400); }
}

export async function flashAndVerify(stk, parsed, report = () => {}) {
  if (!parsed.pages.length) throw new Error("書き込むデータがありません。");
  const sig = await stk.signature();
  if (!UNO_SIGNATURE.every((value, i) => sig[i] === value))
    throw new Error(`対象がUNO R3（ATmega328P）ではありません: signature=${[...sig].map(x => x.toString(16).padStart(2, "0")).join(" ")}`);
  for (const [i, page] of parsed.pages.entries()) {
    await stk.loadAddress(page.address);
    await stk.writePage(page.data);
    await stk.loadAddress(page.address);
    const actual = await stk.readPage();
    for (let j = 0; j < UNO_PAGE_SIZE; ++j) {
      if (actual[j] !== page.data[j])
        throw new Error(`書込検証失敗: 0x${(page.address + j).toString(16)}。再書込前に接続を確認してください。`);
    }
    report(i + 1, parsed.pages.length);
  }
  await stk.leave();
}
