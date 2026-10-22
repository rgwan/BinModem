# V.92 交接（2026-10-23）

给接手 V.92 的下一位。读完这一页就能开工：现状、下一步、踩过的坑、台架事实、命令。

---

## 1. 一句话现状

**Phase 2（能力交换、Table 18）与 Phase 3（模拟端 + 数字端全程）都已跑通并有测试**。
`v92::analogue` 现在是真状态机，做到 E1u 与 `Status::Phase3Done`；Phase 4 与"PCM 上行承载字节"未做。
真机台架（Agere SV92PP）已把 Phase 3 跑到 DIL、对端发出 CPt，**我们仍解不出 CPt**（§5.1）。
分支 `v92-dev`。

```bash
cd BinModem && git log --oneline -6
e2effb3 V.92: what an Agere SV92PP and an HSF engine found in phase 3
8b8c7ed V.92: the analogue modem's phase 3, Su to E1u
3544a0c V.92: a handover page
42010f2 V.92: a call, and Phase 3 through it
146a726 V.92: the TRN1u fit solves for the equaliser, and waits where TRN1u is
cec0e41 V.92: trace the bits the Ja reader is making
f443f79 V.92: the hunt finds Ru on a real line
```

## 2. 已经能跑的东西

| 东西 | 命令 | 结果 |
|---|---|---|
| **Phase 3 全程，两端都是调制解调器** | `cargo test -p datapump --test v92_call -- --nocapture` | 3 条：双向 Phase 3、带 MD、无 Sd 时 AR1 报警 |
| PCM 上行链路（设计+编码+解码） | `cargo test -p datapump --test v92_upstream` | 通过：drn 19 = 48000 bit/s，13392 bit 0 误码 |
| 模拟端单测 | `cargo test -p datapump --lib v92::analogue` | 4 条：A1 长度、DIL 上的 TRN1u、ε 是分数、TRN1u 够拟合 |
| 数字端 Phase 3 单测 | `cargo test -p datapump --lib v92::digital` | 4 条：Ru→Phase 4、Ja 只读一次、Jd 自读回、DIL 段边界 |
| 全量回归 | `cargo test -p datapump --release --no-fail-fast` | 只有 `v90_call` 的 13 条已知红 |
| V.92 能力交换（实线） | 见 §5.1/§6 | HSF 与 Agere 都回 Table 18，V.92 调制器建立 |
| **V.92 Phase 3（真机对端）** | 见 §5.1 Agere | Ru/TRN1u/Ja/Sd/TRN1d/Jd/Su/Jp/Jp'/DIL 全过，停在解 CPt |

`v92_call.rs` 打印的证据（`--nocapture`，两端各自的笔记）：

```
--- digital ---
V.92: TRN1u expected from symbol 973        V.92: TRN1u trained at 179.8 dB
V.92: Ja read, 0 DIL segments, upstream rates 7ffff
V.92: Su at 6424       V.92: Jp with epsilon 0     V.92: second S-bar-u, 107 on
V.92: CPt drn 16, 1 set(s)                  V.92: E1u
--- analogue ---
Ja  Sd at 3401  Sd reversal at 3743, silent for TRN1d
TRN1d trained at 179.4 dB  Jd read, lookahead 1  Jp read, epsilon 0 and 107 on
TRN1u over the DIL, from upstream interval 0   Jp' read, DIL of 0 symbols
CPt for drn 16   Ri at 9144, CPt ends   CPt ends, E1u   E1u sent, phase 3 done
```

## 3. 下一步（按顺序，每步都有验收）

顺序是：先把 Phase 4 做完（3.1，主线），再用 3.2 把模拟端"记录而非应用"的三样接上；
台架（§6）等 Phase 4 有 `Status::Connected` 之后再碰——三个对端现在都不能领跑 Phase 3。

### 3.1 Phase 4（V92-35 下游源 + V92-39 模拟端 + V92-41 数据）

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

### 3.2 `v92::analogue` 记录而非应用的三样（V92-36 的剩余项）

状态机已在，但这三件事接 Phase 4 时才会被消费，别以为已经做完了：

1. **ε 与半符号**：`Modem::delays()` 是队列，`v92::transmit::PcmTransmitter::delay`
   是消费者，现在没人消费它——所以 `v92_call` 里 Jp 的 ε 恒为 0（数字端 `v92::epsilon`
   仍是空桩）。真机接上时先接这条，再谈 `epsilon_from_jp_is_applied_to_the_second_s_bar_u`。
2. **DIL 的内容**：`Modem::dil()` 存的是均衡器输出的原始值，还没解析成 route。
   V92-39 要用它选下游速率。`v90::dil` 是现成读法，但 code-analogue §11.1 的
   `v90::downstream::DilReader` 重构没做，所以要么先做重构，要么在 V92-39 里重写一遍。
3. **A10 的顺序**：`analogue::ZERO_DIL_A10_ORDER = false`（坑 11 末尾有解释）。
   真机数字端若真发 SCR，把常量翻成 true。

### 3.3 之后再谈

短 Phase 1（QTS/ANSpcm，V92-31/49）、Quick Connect（V92-50）、MOH（V92-43）、
速率重协商（V92-40）、掉线重建。**HSF 不需要 QC**（见 §6），别先去写它。

## 4. 十一个坑（都真踩过，别重犯）

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
10. **E1u 不是"连续 12 个零"，是"12 个零出现在该有同步的位置"**。CPt 里本来就有
    16 连零（Table 23 的保留位和整形系数是整字零），数字端旧代码用"任意 12 连零"
    判 E1u，于是**每收到一个 CPt 就把自己的 CPt 序列判成结束**：Ri 只发了 1 个符号
    就被 R̄i 的 `change` 覆盖掉，模拟端永远看不到 Ri 的反转，双方僵在 CPt。
    现在按已读到的 CPt 求一次重复长度、取模跟踪位置，只数同步位那 12 个比特
    （`digital::Modem::cpt_bit`）。
11. **CPt 的填充单位是"12 比特"，不是 1**：`cp.to_bits_in(Layout::V92, 1)`
    等于不填充，CPt 就不再是 12 符号的整数倍，Ja 建立的上行数据帧栅格当场丢掉。
    正确写法是 `to_bits_in(Layout::V92, UP_INTERVALS)`（8.5.1 + 8.5.7）。
    附带一条：**Ri/R̄i 不需要新检测器**——它们就是 Ru/R̄u 的形状（8.6.5 + 8.5.5）
    打在 UINFO 上，`v92::receiver` 的 hunt 会直接报 `Heard::Ru`/`RuReversal`；
    Sd 同理报成 `Heard::Su`。别为它写第二个形状识别器。
    再附带一条：N = 0 时 9.5.2.1.10 要求"先 Ri 再 CPt、R̄i 收尾"，与我们数字端
    （不发 SCR、看到 CPt 才发 Ri）互相死等；`analogue::ZERO_DIL_A10_ORDER`
    就是这条分叉，默认 false 走 A11 的顺序。

## 5. 已知红（开工前就有，别当成你的回归）

- `cargo test -p dsp --test echo_drift`：plan 15.2 记的既有失败。
- `cargo test -p datapump --test v90_call`：**13 条**。已用 `git stash` 对照 HEAD 确认，
  与本分支无关（V.90 的速率重协商/整形用例）。
- `cargo test -p modem --release --test v90_call when_one_end_hangs_up_the_other_notices`：
  **1 条**，挂在 `v90_call.rs:430`（两端没进数据态）。已用 `git worktree` 在 HEAD~1 上
  复现，同样红；V.90 的挂断路径，与 V.92 无关。

## 5.1 台架现状（2026-10-23，真机）

命令（Agere，8 kHz 原生、日志最全，**调试首选**）：

```bash
cd /home/hikari/comm/softmodem && export PKG_CONFIG_PATH=/tmp/alsa-shim/lib/pkgconfig
export BRIDGE_MOD=V92 BINMODEM_V92_PCM_UP=1 V92_TRACE=1
python3 interop/agrsm/run_agr_v90.py --ms-args "V92" --pre "AT+PIG=0" \
        --connect-timeout 45 --hold 4 --verbosity 5 --capture
```

- **`BRIDGE_MOD=V92` 必须显式给**：Agere 脚本没有 `--bridge-mod`，默认把我们服务端起成
  `+MS=V90`，那样永远协商不到 V.92（会看到 `AT+MS=V90` 在 binmodem 日志里）。
- 采集方向别搞反：`caller.raw` 是**我们自己的 TX**（bridge 写的是 `from_binmodem`），
  对端音频是 `agr-tx.raw`。回放对端要 `tools/binmodem_replay.py <agr-tx.raw> --rate 8000`。

**Agere SV92PP 现在走到哪**（`e2effb3` 之后）：

```
Table 18 双向（P2TRACE: table18 true / table10 false）
V.92 phase 3: Ru(679) → TRN1u(15.4 dB) → Ja → Sd/S̄d → TRN1d → Jd → Su → Jp → Jp' → DIL
Agere 侧：Jd detected / Jp detected / Jd' detected / "CPt sent" / PCM phase 3 finished
卡点：**我们解不出 Agere 的 CPt**（`CPt reader` 有 bit，但 CRC 从不通过），于是不发 Ri，
      Agere 在 Phase 4 等 4 s 后 `V34 exception: 6009521a Cleardown`。
```

**HSF（SoftV92）**：Phase 2 正常（bit70 → Table 18），Phase 3 到 **Jp** 就停——
桥里 16k→8k 是 3.4 kHz 带限，Su 的 hunt 一次都不匹配（`V92TRACE` 里没有 Su 事件）。
另外要注意：`AwaitSuBar` 阶段没有专门超时，只有 `start_up_by`（20 s + 6·RTD）兜底，
而**泵在 `Status::Failed` 时不会真的重训**，所以 HSF 这条路现在会干等到窗口结束，
不会再自动回落到 V.90 数据态（`agr92b` 那种 `CONNECT 31200/52000` 的回落看不到了）。

**卡点是上行符号定时，不是 CPt 解码器**（`c4b4081` 已定量）：

| 分数符号偏移 | 同一套拟合 |
|---|---|
| 0 | 166 dB |
| 1/4 符号 | 156 dB |
| **1/2 符号** | **19 dB** |
| 3/4 符号 | 157 dB |

- CPt 解码器**本身是对的**：`a_cpt_reads_back_through_a_band_limited_line`
  用带限信道 + TRN1u 训练后能读回 Table 23 的 CPt。
- 长窗重拟合（1200 → 4800 符号）只把 15.4 提到 **15.6 dB**：方程数不是瓶颈。
- 所以瓶颈是**接收机在网络采样时刻切片，不管对端的符号钟**。半符号偏就是
  "两个相邻符号之和"，任何均衡器都补不回来；CPt 的 CRC 需要约 30 dB。

**下一步（按收益排序）**

1. **上行符号定时恢复**（唯一真正解锁 CPt 的事）：在 `v92::receiver` 的切片前
   加插值/定时环，或让拟合同时解一个分数延迟（Fractionally-spaced equaliser：
   2 样点/符号降采样）。验证口径：Agere 的 TRN1u 拟合从 15.6 dB 上到 30 dB 以上，
   然后 `CPt drn` 出现在 binmodem 日志里、对端不再 clear down。
2. **失败重训已经接好的部分**：`v90/startup.rs:617` 的 `v92` 分支在
   `pcm::Status::Failed` 且 `failed_starts < V90_RETRAINS` 时会 `restart_phase2()`——
   但重训后对端仍回 Table 18，于是又进 V.92 再失败一次。真机要的是
   **第 N 次失败后拒绝 PCM 上行**（`decline_pcm()`，V.90 分支已有这个动作，V.92 没有）。
3. **重训保住 V.92 旗标**（`v34/phase2.rs::again()`，code-analogue §11.2 的老 TODO）：
   现在重训后会退成 Table 10（P2TRACE 第二次 `table10 true`）。
4. 采集文件不可全信：`agr-tx.raw` 上用接收机链连 TRN1u 都拟合不过 0 dB（在线是 15.4），
   说明这份 raw 与线路上真正听到的不是同一个信号（或采样率/格式不同）。
   `tests/v92_capture.rs` 的两个诊断就是为这件事留的。

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

### Agere SV92PP —— 8 kHz 原生，Phase 3 **能**领跑（首选台架）
```bash
python3 interop/agrsm/run_agr_v90.py --ms-args "V92" --pre "AT+PIG=0" \
        --connect-timeout 45 --hold 4
```
- **必须 `AT+PIG=0`**（默认 `+PIG:1` = 忽略 PCM 上行），否则它回 Table 10。
  设了之后回 Table 18，我们的 V.92 调制器会被建立。
- 8 kHz、无 SRC。**它确实领跑 Phase 3**：`+PIG=0` + 服务端 `BRIDGE_MOD=V92`
  + `BINMODEM_V92_PCM_UP=1` 之后，Ru/TRN1u/Ja/Su/Jp'/DIL/CPt 全都发（见 §5.1）。
  旧记录里"只发静音、不发 Ru"是**服务端被起成 V90** 时的现象，别照抄。
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
| `datapump/src/v92/digital.rs` | **Phase 3 完成**（1100 行） | 数字端 Ru→R-bar-i→TRN2d；Phase 4 是钩子；E1u 检测见 §4 坑 10 |
| `datapump/src/v92/analogue.rs` | **Phase 3 完成**（1050 行） | 模拟端 A1-A11 状态机，到 E1u → `Status::Phase3Done`；ε/半符号在 `delays()`，DIL 在 `dil()` |
| `datapump/src/v92/down_source.rs` | 空桩 | §3.3 |
| `datapump/src/v92/epsilon.rs` | 空桩（Jp 固定 0） | 真实模拟端才需要 |
| `datapump/src/v92/{transmit,upstream,decoder,design,upchoice,modulus,precoder,exchange}.rs` | 完成 | PCM 上行链路，已被 `v92_upstream.rs` 验证 |
| `datapump/src/v92/{sequences,up_signals,anspcm}.rs` | 完成 | Ja/CPt/CPd 帧、Ru/Su/TRN1u、QTS/ANSpcm |
| `datapump/tests/v92_call.rs` | 完成（190 行） | 两端都是调制解调器的 Phase 3 呼叫：全程、带 MD、无 Sd |
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
let mut trn = Trn1u::with_mode(Mode::Answer);   // 模拟端就是 Answer（默认）
let mut sender = TwoPointSender::after(Trn1u::new());
sender.preamble();                       // 24 个 1
sender.push(&Descriptor::none().to_bits_in(Some(0x7ffff)));  // Ja
sender.push(&cp.to_bits_in(Layout::V92, UP_INTERVALS));      // CPt：12 比特为单位，别传 1
sender.e1u();
sender.next_symbol();                    // Option<f64>，耗尽后按需重填

// 用一个模拟端跑 Phase 3（v92_call.rs 里的做法）
let mut modem = analogue::Modem::new(analogue::Settings {
    md_length: 0,                        // INFO1a 的 MD，单位 276 符号
    ..analogue::Settings::new(Descriptor::none(), 0x7ffff, cp)
});
let out = modem.step(from_digital);      // 一个符号进、一个符号出
modem.status() == analogue::Status::Phase3Done
modem.far_jd() / modem.jp() / modem.epsilon() / modem.circuit107()
modem.delays()                           // [0.5, ε]：给 PcmTransmitter::delay 吃
modem.dil()                              // DIL 的原始样点，等 V92-39 解析

// 读取对方的帧
let mut reader = TwoPointReader::differential_with(Mode::Call);  // Jd/Jp
reader.feed(level) -> Option<bool>;
J::from_bits(&window72) -> Option<J>;    // 72 位滑窗，CRC 校验
```
