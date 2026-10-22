# V.92 交接（2026-10-22）

给接手 V.92 的下一位。读完这一页就能开工：现状、下一步、踩过的坑、台架事实、命令。

---

## 1. 一句话现状

**Phase 2（能力交换、Table 18）与 Phase 3（数字端全程）已经跑通并有测试**；`v92::analogue`
仍是空桩，Phase 4 与"PCM 上行承载字节"未做。分支 `v92-dev`，HEAD `42010f2`。

```bash
cd BinModem && git log --oneline -6
42010f2 V.92: a call, and Phase 3 through it
146a726 V.92: the TRN1u fit solves for the equaliser, and waits where TRN1u is
cec0e41 V.92: trace the bits the Ja reader is making
f443f79 V.92: the hunt finds Ru on a real line
81c7c76 V.90: TRN1d long enough for the far end to turn round, and the last XID
bb51f32 V.92: the digital modem's Phase 3, and the server that offers it
```

## 2. 已经能跑的东西

| 东西 | 命令 | 结果 |
|---|---|---|
| Phase 3 全程（数字端） | `cargo test -p datapump --test v92_call -- --nocapture` | 通过：Ja(7ffff) / Su / Jp / 107 / CPt(16) / E1u → Phase 4 |
| PCM 上行链路（设计+编码+解码） | `cargo test -p datapump --test v92_upstream` | 通过：drn 19 = 48000 bit/s，13392 bit 0 误码 |
| 数字端 Phase 3 单测 | `cargo test -p datapump --lib v92::digital` | 4 条：Ru→Phase 4、Ja 只读一次、Jd 自读回、DIL 段边界 |
| V.92 能力交换（实线） | 见 §6 HSF | HSF CONNECT，V.90 数据态 21600/56000，2000 B 0 错 |

`v92_call.rs` 打印的证据（这就是 Phase 3 的验收输出）：

```
TRN1u trained at 179.8 dB
Ja read, 0 DIL segments, upstream rates 7ffff
Su at 6737
Jp with epsilon 0
second S-bar-u, 107 on
CPt drn 16, 1 set(s)
E1u
```

## 3. 下一步（按顺序，每步都有验收）

### 3.1 `v92::analogue` 的 Phase 3（V92-36）

**文件**：`crates/datapump/src/v92/analogue.rs`（现在是 6 行注释的桩）。

**蓝图已经有了**：`crates/datapump/tests/v92_call.rs` 里的 `Script` 就是模拟端的 Phase 3，
只是写成测试内的脚本。把它搬进 `analogue.rs`，加上状态机与错误路径即可：

- 发送（全部现成，见 §9）：`ru(n, bar)`、`su(n, bar)`、`Trn1u::with_mode(Mode::Answer)`、
  `TwoPointSender::after(Trn1u::new())` + `preamble()` + `push(bits)`、`e1u()`。
- 接收：`Receiver` + `set_reference_mode(Mode::Call)`（数字端的 TRN1d 是 GPC）、
  `train_near(...)`、`TwoPointReader::differential_with(Mode::Call)` 读 Jd/Jp。
- 时序：Ru 384T → R̄u 24T →（MD = `md_units * 276`）→ Ru → R̄u → TRN1u ≥2040T → Ja 重复；
  收到 Sd→S̄d 反转后静音；训练 TRN1d（起点 = 反转 + 48）；读 Jd → Su 144T → S̄u 24T → Su；
  收到 Jp → 107 ON → S̄u (24+ε)T → TRN1u 覆盖 DIL → CPt 重复；收到 R̄i → E1u → Phase 4。
- 验收：把 `v92_call.rs` 的 `Script` 换成 `analogue::Modem`，同一个测试仍然通过；
  再加两条：`phase_3_completes_with_md`、`a_digital_end_that_never_sends_sd_fails_analogue`.

### 3.2 Phase 4（V92-35 下游源 + V92-39 模拟端 + V92-41 数据）

- **`v92::down_source`**（5 行桩）：数字端 Phase 4 的下游信号 TRN2d → SUVd/CPd → Ed → B1d → 数据。
  现在的 `v92::digital::Out::Trn2d` 分支只会**循环重发 TRN2d**，是留给这一步的钩子。
- **CPd 怎么算**：`v92::design::Channel::estimate(history)`（用 TRN1u 的 history，
  `Receiver::history_since`）+ `v92::design::design(...)` + `v92::upchoice::choose(...)`，
  再用 `v92::exchange::Exchange` 做 SUVd/CPd 的交换。`v92_upstream.rs` 已经把这条链
  在一段仿真的环路信道上跑通了（0 误码），**照抄它的调用顺序**。
- **ε（采样相位）**：`v92::epsilon` 仍是 5 行桩，`Jp` 现在固定发 0。
  直线/仿真网络上 0 是对的；要接真实模拟端才需要它。
- 验收：`v92_call.rs` 加 `a_v92_call_connects_with_pcm_both_ways_and_carries_data`
  （两端 `Status::Connected`、下行 ≥48000、上行在 PCM 阶梯上、双向数据、无重训），
  即 plan 的 V92-41/M2。

### 3.3 之后再谈

短 Phase 1（QTS/ANSpcm，V92-31/49）、Quick Connect（V92-50）、MOH（V92-43）、
速率重协商（V92-40）、掉线重建。**HSF 不需要 QC**（见 §6），别先去写它。

## 4. 九个坑（都真踩过，别重犯）

1. **多项式跟角色走**（6.3/V.92）：模拟端 TRN1u/Ja/CPt = **GPA(`Mode::Answer`)**；
   数字端 TRN1d/Jd/Jp = **GPC(`Mode::Call`)**。用错就是"对着错误序列训练"，
   干净链路上也只得到噪声。接口：`Receiver::set_reference_mode`、
   `TwoPointReader::differential_with`、`Trn1u::with_mode`。
2. **均衡器拟合方向**：解 `reference = Σ taps·received`（均衡器），
   **不是** `received = Σ taps·reference`（那是信道，作用两次后输出只剩 4%）。
3. **`train_near` 很贵**（129 次 48×48 最小二乘）：绝不每样点调用；失败把锚点前移
   （digital +650，脚本 +600）。否则测试直接挂死（我挂过一次，180 s 超时）。
4. **拟合索引是 history 相对下标**：`from - (symbols - history_len)`。
   history 是滑动窗口，训练时早已滑过反转点。
5. **拟合窗口固定 `TRAIN_WINDOW = 1200` 符号**，不要用"history 全部"：会伸进 Ja/静音。
6. **`AUDIBLE` 量纲**：真实线路到本端约 0.25 满度（均方 0.06）；旧值 0.05 均方 = 0.22 LU，
   hunt 一次都不触发，Phase 3 干等 Ru 到 DR1 失败。现 1e-4（后面是归一化相关）。
7. **形状要连续保持 8 个周期**才算 Ru/Su，并**原谅翻转那一周期**（`Anchor::stray`）；
   2 个周期会被 phase 2 的音骗得每秒"发现 Ru"。
8. **MD 只等一次**（INFO1a 的 MD 在第一次 Ru→R̄u 之后）；每次反转都等会把
   "该起 TRN1u 的那次"吞掉。
9. **`TRAINED_SNR_DB = 12 dB`**：8-bit 码字 + 48 前馈抽头实测约 16 dB，
   原来的 20 dB 是合成 burst 的自评。

## 5. 已知红（开工前就有，别当成你的回归）

- `cargo test -p dsp --test echo_drift`：plan 15.2 记的既有失败。
- `cargo test -p datapump --test v90_call`：**13 条**。已用 `git stash` 对照 HEAD 确认，
  与本分支无关（V.90 的速率重协商/整形用例）。

## 6. 三个模拟对端的事实

### HSF（Conexant SoftV92）—— 要 PCM 上行，但台架带限
```bash
cd interop/hsf && BINMODEM_V92_PCM_UP=1 \
  python3 run_hsf_v90.py --bridge-mod V92 --seconds 20 --connect-timeout 40
```
- `BINMODEM_V92_PCM_UP=1`（服务端 INFO1d bit70=1）时，HSF 回 **Table 18**：
  `sections=0, ltot=3, lmax=0, md_length=20, uinfo=78`。
- 不打开时回 Table 10，V.90 数据态 **21600/56000**、2000 B 0 错。
- **HSF 不要 QC**：早前 V.8 卡死是我们自己的缺陷——HSF 把 V.21 载波停在 CJ 第三
  八位组中间并立刻起 phase 2 音，线上变成 `00 00 80`。已在 `v8::Decoder` 修
  （常量 `DAMAGED_ZERO`，只允许 CJ 的**最后一个**八位组这样）。
  `framing.rs` 的 `RING_COAST` 保持 **false**，不要打开。
- **台架不适合验 PCM 上行**：桥里 16 kHz→8 kHz 的 `Polyphase` 是 3.4 kHz 带限，
  而 V.92 上行是 8000 符号/秒；拟合最高 15.7 dB 且无对齐尖峰，Ja 读不出来。
  用 HSF 只验 Phase 2 与 V.90 数据态。

### Agere SV92PP —— 8 kHz 无 SRC，但 Phase 3 不领跑
```bash
python3 interop/agrsm/run_agr_v90.py --ms-args "V92" --pre "AT+PIG=0" \
        --connect-timeout 45 --hold 4
```
- **必须 `AT+PIG=0`**（默认 `+PIG:1` = 忽略 PCM 上行），否则它回 Table 10。
  设了之后回 Table 18，我们的 V.92 调制器会被建立。
- 8 kHz、无 SRC，Ru/RuReversal 能识别；但它 **Phase 3 起始只发静音、不发 Ru**
  （约 3.5 s 后 `V34 exception: 60095213` 重训）。**不能做 Phase 3 领跑者**。
- 日志很富（`--verbosity 3/5`）：`V34 Phase 2 finished, modulation v.92`、
  `PCM phase 3 started`、`PCM Mu-law coding`、`CPt sent ...`、`Rx Data Mode Active!`。
- **ATI11**（引擎自己的连接报告，判断它到底选了 V.92 还是回落 V.90）：
  脚本已实现——`ATH` 后等 ~6 s，先发 `ATI6` 顶掉 ATH 的 NO CARRIER，再
  `ATI11` + 多一个回车。输出含 `Last Connection`、收发速率、SNR、RTD、回声、重训计数。
  （`+++` 在这颗 build 上无法进入 online-command，试过，没 OK。）

### D-Modem golden —— 没有 V.92
`GOLDEN_DP=V92` 会打印 `change dp: --> 92 error`：`modem.c` 表里有 `DP_V92`，
但 dsplibs 只有 `V92Modulator/V92Precoder` 等类，没有 driver init。
**只能做 V.90/V.34 对端。**

## 7. 构建、调试、回放

```bash
export PATH=$HOME/.cargo/bin:$PATH
export PKG_CONFIG_PATH=/tmp/alsa-shim/lib/pkgconfig   # 指向 ~/local/alsa，libdir 必须是真实目录
cargo test  --release -p datapump --test v92_call
cargo build --release -p modem --bin v90-bridge
```

- `V92_TRACE=1`：Phase 3 阶段/hunt 事件/`train_near` 候选 SNR/Ja bit 计数。
- `PHASE2_TRACE=1`：INFO0/INFO1a/INFO1c，含 **bit70 是否真的上线**。
- `V8_TRACE=1`：逐八位组。
- **回放（比实线快 20 倍，是首选）**：
  ```bash
  python3 tools/binmodem_replay.py <caller.raw> --rate 16000 --start 0 --seconds 14 \
          --mod V92 --realtime 0
  ```
  它复刻桥的 SRC + G.711；`BINMODEM_MS` 可覆盖整个 +MS 列表。

## 8. 文件地图

| 文件 | 状态 | 说明 |
|---|---|---|
| `datapump/src/v92/receiver.rs` | 成熟（907 行） | Ru/Su hunt、TRN1u 拟合、切片、history、相位 |
| `datapump/src/v92/digital.rs` | **Phase 3 完成**（1024 行） | 数字端 Ru→R-bar-i→TRN2d；Phase 4 是钩子 |
| `datapump/src/v92/analogue.rs` | **空桩（6 行）** | 下一步 §3.1 |
| `datapump/src/v92/down_source.rs` | 空桩 | §3.2 |
| `datapump/src/v92/epsilon.rs` | 空桩（Jp 固定 0） | 真实模拟端才需要 |
| `datapump/src/v92/{transmit,upstream,decoder,design,upchoice,modulus,precoder,exchange}.rs` | 完成 | PCM 上行链路，已被 `v92_upstream.rs` 验证 |
| `datapump/src/v92/{sequences,up_signals,anspcm}.rs` | 完成 | Ja/CPt/CPd 帧、Ru/Su/TRN1u、QTS/ANSpcm |
| `datapump/tests/v92_call.rs` | 新增（309 行） | Phase 3 呼叫；`Script` 是 analogue 的蓝图 |
| `datapump/tests/v92_upstream.rs` | 完成 | 链路 0 误码 |
| `datapump/src/v90/startup.rs` | 已接 | `Digital::with_v92`；Table 18 → 建 V.92 调制器 |
| `modem/src/lib.rs` | 已接 | `+MS=V92`、`V92S` 泵、`BINMODEM_V92_PCM_UP` |

## 9. 关键 API 速查

```rust
// 接收
let mut rx = Receiver::new();
rx.set_reference_mode(Mode::Call);      // 读数字端的 TRN1d（GPC）
rx.feed(sample);                         // 每样点
while let Some(h) = rx.heard() { /* Ru/Su/RuReversal/SuReversal/Trained/Symbol/Lost/Found */ }
rx.train_near(at, 64)                    // at 是 history 相对下标；从 Ru 反转 +24 搜
rx.history_since(n) / rx.taps()          // 给 design::Channel::estimate

// 发送
ru(n, bar); su(n, bar);                  // 6 符号序列
let mut trn = Trn1u::with_mode(Mode::Answer);
let mut sender = TwoPointSender::after(Trn1u::new());
sender.preamble();                       // 24 个 1
sender.push(&Descriptor::none().to_bits_in(Some(0x7ffff)));  // Ja
sender.push(&cp.to_bits_in(Layout::V92, 1));                 // CPt
sender.e1u();
sender.next_symbol();                    // Option<f64>，耗尽后按需重填

// 读取对方的帧
let mut reader = TwoPointReader::differential_with(Mode::Call);  // Jd/Jp
reader.feed(level) -> Option<bool>;
J::from_bits(&window72) -> Option<J>;    // 72 位滑窗，CRC 校验
```
