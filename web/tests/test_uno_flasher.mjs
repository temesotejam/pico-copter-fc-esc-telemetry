import assert from "node:assert/strict";
import { parseIntelHex, Stk500v1, flashAndVerify, UNO_PAGE_SIZE, UNO_FLASH_LIMIT } from "../uno-flasher-core.js";

function hexRecord(address, type, payload) {
  const raw = [payload.length, (address >> 8) & 255, address & 255, type, ...payload];
  const checksum = (-raw.reduce((a, b) => a + b, 0)) & 255;
  return ":" + [...raw, checksum].map(x => x.toString(16).padStart(2, "0")).join("").toUpperCase();
}

const input = [
  hexRecord(0, 0, [0x0c, 0x94, 0x00, 0x00]),
  hexRecord(0x80, 0, [1, 2, 3, 4]),
  hexRecord(0, 1, [])
].join("\n");
const parsed = parseIntelHex(input);
assert.equal(parsed.bytes, 8);
assert.equal(parsed.pages.length, 2);
assert.equal(parsed.pages[0].address, 0);
assert.equal(parsed.pages[0].data[0], 0x0c);
assert.equal(parsed.pages[0].data[5], 0xff);
assert.equal(parsed.pages[1].address, 128);
assert.equal(parsed.pages[1].data[3], 4);
assert.throws(() => parseIntelHex(input.replace(":04000000", ":04000001")), /チェックサム/);
assert.throws(() => parseIntelHex(input.split("\n").slice(0, 2).join("\n")), /EOF/);
assert.throws(() => parseIntelHex(input + "\n" + hexRecord(0, 0, [1])), /EOF以降/);
assert.throws(() => parseIntelHex([
  hexRecord(0x7e00, 0, [1]), hexRecord(0, 0, [1]), hexRecord(0, 1, [])
].join("\n")), /ブートローダー/);
assert.equal(UNO_PAGE_SIZE, 128);
assert.equal(UNO_FLASH_LIMIT, 32256);

class FakeTransport {
  constructor(signature = [0x1e, 0x95, 0x0f], corruptRead = false) {
    this.signature = signature;
    this.corruptRead = corruptRead;
    this.memory = new Uint8Array(UNO_FLASH_LIMIT).fill(0xff);
    this.word = 0;
    this.responses = [];
    this.commands = [];
  }
  async send(bytes) {
    assert.equal(bytes.at(-1), 0x20);
    const cmd = bytes[0];
    this.commands.push(cmd);
    let response = [];
    if (cmd === 0x75) response = this.signature;
    else if (cmd === 0x55) this.word = bytes[1] + (bytes[2] << 8);
    else if (cmd === 0x64) {
      assert.equal(bytes[1] << 8 | bytes[2], UNO_PAGE_SIZE);
      assert.equal(bytes[3], 0x46);
      this.memory.set(bytes.slice(4, 4 + UNO_PAGE_SIZE), this.word * 2);
    } else if (cmd === 0x74) {
      assert.equal(bytes[3], 0x46);
      response = [...this.memory.slice(this.word * 2, this.word * 2 + UNO_PAGE_SIZE)];
      if (this.corruptRead) response[0] ^= 0x01;
    } else if (cmd !== 0x30 && cmd !== 0x51) throw Error("unknown STK500 command");
    this.responses.push(0x14, ...response, 0x10);
  }
  async readByte() {
    if (!this.responses.length) throw new Error("read buffer empty");
    return this.responses.shift();
  }
}

const io = new FakeTransport();
const stk = new Stk500v1(io);
await stk.sync();
const progress = [];
await flashAndVerify(stk, parsed, (done, total) => progress.push([done, total]));
assert.deepEqual(progress, [[1, 2], [2, 2]]);
assert.equal(io.memory[0], 0x0c);
assert.equal(io.memory[0x80], 1);
assert.deepEqual(io.commands.filter(x => x === 0x64), [0x64, 0x64]);
assert.equal(io.commands.at(-1), 0x51);
await assert.rejects(flashAndVerify(new Stk500v1(new FakeTransport([0, 0, 0])), parsed), /対象がUNO R3/);
await assert.rejects(flashAndVerify(new Stk500v1(new FakeTransport(undefined, true)), parsed), /検証失敗/);
console.log("UNO HEX parsing, bounds, STK500v1 write/readback verify passed");
