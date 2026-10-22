# V.92 交接（2026-10-23）

给接手 V.92 的下一位。读完这一页就能开工：现状、下一步、踩过的坑、台架事实、命令、文件地图。
**所有数字都是实测的**；不确定的地方都标了"未验证"。

---

## 1. 一句话现状

- **Phase 2（能力交换、Table 18）与 Phase 3（模拟端 + 数字端全程）都已跑通并有测试**。
- `v92::analogue` 是真状态机：9.5.2.1 的 A1–A11，做到 E1u → `Status::Phase3Done`。
- `v92::digital` 的 Phase 3 完成，Phase 4 只到 TRN2d 的钩子。
- **真机台架（Agere SV92PP）能把 Phase 3 走完**：Ru/TRN1u/Ja/Sd/TRN1d/Jd/Su/Jp/Jp'/DIL 全过，
  Jp 的 ε 已送达对端（对端日志 `Jp detected: aa57 1000`），**卡在我们解不出对端的 CPt**。
- Phase 4（`down_source`）与"PCM 上行承载字节"未做。
- 分支 `v92-dev`，HEAD `ac09682`。

```bash
cd BinModem && git log --oneline -13
ac09682 V.92: drop a must_use the compiler does not want on fitted()
7587c73 V.92: the digital modem's header stops calling epsilon a zero
431570b V.92: plan.md says ZERO_DIL_A10_ORDER is on, because it is
d1ea3d8 V.92: the handover page, rewritten for the state the bench left it in
6066ce6 V.92: epsilon reaches the far end now; what it still has to get right
7814625 V.92: measure epsilon on Su, and stop telling the far end to shift by nothing
d9f31a2 V.92: refit the equaliser in the second TRN1u, where the phase has moved
c8d6804 V.92: the bench blocker is upstream timing, and the numbers that say so
c4b4081 V.92: the fit has a timing ceiling, and a longer window does not move it
87c3a0d V.92: what a capture says about the CPt, and the test that says the decoder is right
bd12994 V.92: the bench, as it stands on both engines
e2effb3 V.92: what an Agere SV92PP and an HSF engine found in phase 3
3eb5061 V.92: the handover page, after phase 3 both ways
8b8c7ed V.92: the analogue modem's phase 3, Su to E1u
```

**开工第一件事**：读 §3.1。台架卡在"上行符号定时"，`v92::epsilon` 刚接上但符号/量纲未核对，
这是唯一还没做过闭环的地方。

## 2. 已经能跑的东西

| 东西 | 命令 | 结果 |
|---|---|---|
| **Phase 3 全程，两端都是本项目的调制解调器** | `cargo test -p datapump --test v92_call -- --nocapture` | 3 条：双向 Phase 3、带 MD、无 Sd 时 AR1 报警 |
| V.92 各模块单测 | `cargo test -p datapump --lib v92::` | **151 条**全绿（receiver/digital/analogue/epsilon/upstream/…） |
| PCM 上行链路（设计+编码+解码） | `cargo test -p datapump --test v92_upstream` | 2 条：drn 19 = 48000 bit/s，13392 bit 0 误码 |
| 采集诊断（要文件，`#[ignore]`） | 见 §5.1 | 2 条 |
| 全量回归 | `cargo test -p datapump --release --no-fail-fast` | 只有 `v90_call` 的 13 条已知红（§5） |
| V.92 能力交换（真机） | 见 §5.1/§6 | HSF 与 Agere 都回 Table 18，V.92 调制器建立 |
| **V.92 Phase 3（真机对端）** | 见 §5.1 Agere | 全过，停在解 CPt |

`v92_call` 的真实输出（`--nocapture`；注意 **Jp 的 ε 不再是 0**，且 N=0 走 A10 的顺序）：

```
--- digital ---
V.92: TRN1u expected from symbol 973       V.92: TRN1u trained at 179.8 dB
V.92: Ja read, 0 DIL segments, upstream rates 7ffff
V.92: Su at 6424                           V.92: Jp with epsilon 17588
V.92: second S-bar-u, 107 on               V.92: CPt drn 16, 1 set(s)
--- analogue ---
Ja   Sd at 3401   Sd reversal at 3743, silent for TRN1d
TRN1d trained at 179.4 dB   Jd read, lookahead 1
Jp read, epsilon 17588 and 107 on
TRN1u over the DIL, from upstream interval 0   Jp' read, DIL of 0 symbols
CPt for drn 16   Ri at 9148, CPt   R-bar-i at 9526, CPt ends   E1u sent, phase 3 done
```

## 3. 下一步（按优先级，每步都有验收口径）

### 3.1 解锁台架：上行符号定时（**主线**）

**问题**：接收机在网络的 8 kHz 采样时刻切片，不管对端的符号钟。半符号偏等于"相邻两个符号
相加"，信息真的丢了，任何均衡器都补不回来（§4 坑 6 的表：166 dB → 19 dB）。CPt 的 CRC 要
约 30 dB，所以 CPt 解不出，对端 4 s 后 `6009521a` 清线。

规范给的两条路，两条都还没走完：

1. **Jp 的 ε（9.5.1.1.6-7）——已接上但未验证**（`7814625`）。
   现在在**均衡器输出**上测 Su 的相位，而均衡器本身已经被拟合成"把收到的 TRN1u 映射到我们
   采样点上的 ±1"，它自己的残余延迟混进了测量。台架实测 ε = 0xaa57 ≈ 0.665 符号，
   而 0.665 与"拟合 15.4 dB"隐含的偏移并不自洽，所以**符号很可能反了**。
   改法（按代价排序）：
   - 在**接收机的原始样点**上测 Su 的相位（`Receiver` 的 history 里就有），
     并用抽头算信道在 f0 = 8000/6 Hz 的相位去校正；`epsilon::offset` 的输入换成原始样点。
   - 或者用实验定符号：跑一次台架，读 Agere 的 `Timing Offset` / `Signal/Noise Ratio`
     （它日志里的 Phase 3 报告），把 ε 取反再跑一次，看哪一个让它上升。
   **验收**：Agere 的 Phase 3 SNR 上升、我们这边的 TRN1u 拟合离开 15.6 dB、
   `CPt drn` 出现在 binmodem 日志里、对端不再 clear down。
2. **第二 TRN1u 的重拟合——代码有了，真机上没生效**（`d9f31a2`）。
   规范（`spec-phase3-procedures.md` §4 第 4 条）："the second TRN1u also lets the digital
   modem refine its equalizer at the new phase"。`digital::Modem::refit_from` 已实现，
   但锚点是猜的（`Su 反转 + 24 + 1300`）、搜索只有 ±64，真机对端的第二 TRN1u
   起点落在范围外，台架日志里**从来没有** `V.92: refit at the new phase`
   （注意与 `train_near` 自己的 `V92TRACE: refit at N over 4800: ...` 区分，
   那是长窗重拟合打的，说明不了第二 TRN1u 有没有被拟合到）。
   **改法**：先用相关（而不是逐点最小二乘）把那段 TRN1u 找出来——TRN1u 是 GPA 抖动序列，
   对 ±UINFO 的 2 点流做相关就能定位；拿到起点后再做一次拟合。
   **验收**：日志出现 refit 且 SNR 高于第一次；CPt 仍要单独验证。

**顺带**：`c4b4081` 的长窗重拟合（1200 → 4800 符号）只值 0.2 dB，已经在内，
说明"方程数"这条路走完了，别再往这个方向加。

### 3.2 Phase 4（V92-35 下游源 + V92-39 模拟端 + V92-41 数据）

- **`v92::down_source`**（5 行桩）：数字端 Phase 4 的下游 TRN2d → SUVd/CPd → Ed → B1d → 数据。
  现在的 `digital::Out::Trn2d` 分支只会**循环重发 TRN2d**，是留给这一步的钩子。
- **CPd 怎么算**：`v92::design::Channel::estimate(rx.history_since(..))` +
  `design::design(..)` + `upchoice::choose(..)`，再用 `v92::exchange::Exchange` 做
  SUVd/CPd 的交换。`v92_upstream.rs` 已经把这条链在一段仿真环路信道上跑通（0 误码），
  **照抄它的调用顺序**。
- **验收**：`v92_call.rs` 加 `a_v92_call_connects_with_pcm_both_ways_and_carries_data`
  （两端 `Status::Connected`、下行 ≥48000、上行在 PCM 阶梯上、双向数据、无重训），
  即 plan 的 V92-41/M2。

### 3.3 `v92::analogue` 记录而非应用的三样（V92-36 的剩余项）

1. **ε 与半符号**：`Modem::delays()` 是队列，`v92::transmit::PcmTransmitter::delay` 是消费者，
   现在没人消费它。8.5.6 要求 Su/S̄u 是 12 符号的整数倍，所以它们**不能**变成额外符号。
2. **DIL 的内容**：`Modem::dil()` 存的是均衡器输出的原始值，还没解析成 route。
   V92-39 要用它选下游速率。`v90::dil` 是现成读法，但 code-analogue §11.1 的
   `v90::downstream::DilReader` 重构没做，所以要么先做重构，要么在 V92-39 里重写一遍。
3. **A10 的顺序**：`analogue::ZERO_DIL_A10_ORDER` 现在是 **true**（配 9.5.2.1.10，
   数字端 N=0 时发 SCR→Ri）。只有在对端"不看到 CPt 就不发 Ri"的相反实现下才需要翻回 false。

### 3.4 重训与协商（真机上会碰到）

- **失败重训已经接线**：`v90/startup.rs:617` 的 `v92` 分支在 `pcm::Status::Failed` 且
  `failed_starts < V90_RETRAINS` 时会 `restart_phase2()`。缺的是
  **第 N 次失败后拒绝 PCM 上行**（V.90 分支有 `decline_pcm()`，V.92 没有），
  否则每次重训又回 Table 18、又进 V.92、又失败。
- **重训保住 V.92 旗标**（`v34/phase2.rs::again()`，code-analogue §11.2 的老 TODO）：
  现在重训后会退成 Table 10（P2TRACE 第二次 `table10 true`）。
- **`AwaitSuBar` 阶段没有专门超时**：只有 `start_up_by`（20 s + 6·RTD）兜底，
  HSF 那种"Su 永远不来"的台架会一直等到那时。

### 3.5 之后再谈

短 Phase 1（QTS/ANSpcm，V92-31/49）、Quick Connect（V92-50）、MOH（V92-43）、
速率重协商（V92-40）、掉线重建。**HSF 不需要 QC**（见 §6），别先去写它。

## 4. 十三个坑（都真踩过，别重犯）

1. **多项式跟角色走**（6.3/V.92）：模拟端 TRN1u/Ja/CPt = **GPA（`Mode::Answer`，默认）**；
   数字端 TRN1d/Jd/Jp = **GPC（`Mode::Call`）**。接口：`Receiver::set_reference_mode`、
   `TwoPointReader::differential_with`、`Trn1u::with_mode`。
   **是"跟角色走"而不是"跟谁发走"**：数字端训练的是**模拟端的 TRN1u → GPA**，别弄反。
2. **均衡器拟合方向**：解 `reference = Σ taps·received`（均衡器），
   **不是** `received = Σ taps·reference`（那是信道，作用两次后输出只剩 4%）。
3. **`train_near` 很贵**（129 次 48×48 最小二乘）：绝不每样点调用；失败把锚点前移
   （digital +650，脚本 +600）。否则测试直接挂死（挂过一次，180 s 超时）。
4. **拟合索引是 history 相对下标**：`from - (symbols - history_len)`。history 是滑动窗口，
   训练时早已滑过反转点。
5. **拟合窗口固定 `TRAIN_WINDOW = 1200` 符号**，不要用"history 全部"（会伸进 Ja/静音）。
   长窗重拟合（`TRAIN_WINDOW_LONG = 4800`）只值 0.2 dB，别再指望它。
6. **拟合有定时上限**（实测，同一个合成链路）：

   | 对端符号相对采样点的偏移 | 拟合 SNR |
   |---|---|
   | 0 | 166 dB |
   | 1/4 符号 | 156 dB |
   | **1/2 符号** | **19 dB** |
   | 3/4 符号 | 157 dB |

   半符号不是插值的假象：采样晚半个符号就是"相邻两个符号相加"，没有均衡器能还原。
   这是整条台架卡点的根（§3.1）。
7. **`train_near` 失败会把 `trained` 关掉**（不是 no-op）。试第二次拟合（比如 3.1 的重拟合）
   必须用 `Receiver::fitted()` / `restore()` 把 taps + snr + trained 一起回滚，
   否则一次倒霉的窗口就让接收机失锁、CPt 解码器全瞎。
8. **`AUDIBLE` 量纲**：真实线路到本端约 0.25 满度（均方 0.06）；旧值 0.05 均方 = 0.22 LU，
   hunt 一次都不触发，Phase 3 干等 Ru 到 DR1 失败。现 1e-4（后面是归一化相关）。
9. **形状要连续保持 8 个周期**才算 Ru/Su，并**原谅翻转那一周期**（`Anchor::stray`）；
   2 个周期会被 phase 2 的音骗得每秒"发现 Ru"。
10. **Ru 和 Su 靠形状分不开，得靠时刻表**。真机把 Su 的两个零抹平后，一个周期与 **Ru** 模板
    相关 **0.911**（门槛 0.90）、与 Su 模板只有 0.644——Agere 的 Ru 和 Su 甚至几乎是同一个
    1333 Hz 波形。所以：Su 有两套模板（8.5.6 印的形状 + 带限后近似正弦，`SU_FILTERED`），
    并且用 `Receiver::expect_ru()` / `expect_su()` 按顺序挑（Ru 领起 Phase 3、Su 答 Jd、
    Ri 在 CPt 之后）。
11. **E1u 不是"连续 12 个零"，是"12 个零出现在该有同步位的地方"**。CPt 里本来就有 16 连零
    （Table 23 的保留位与整形系数是整字零），旧代码用"任意 12 连零"判 E1u，于是每个 CPt
    都把自己的序列判成结束：Ri 只发了 1 个符号就被 R̄i 覆盖，模拟端永远看不到反转。
    现在按已读到的 CPt 求一次重复长度、取模跟踪，只数同步位那 12 个比特（`cpt_bit`）。
12. **CPt 的填充单位是"12 比特"，不是 1**：`to_bits_in(Layout::V92, 1)` 等于不填充，
    CPt 就不再是 12 符号的整数倍，Ja 建立的上行数据帧栅格当场丢掉。
    正确写法：`to_bits_in(Layout::V92, UP_INTERVALS)`（8.5.1 + 8.5.7）。
13. **`RU_PERIOD = 6`，`UP_INTERVALS = 12`**：Su/Ru 的形状周期是 6，上行**数据帧**是 12。
    ε 的相位测量用错了这个（按 12 取 DFT 得到的是别的分量），症状是偏移恒为 0 或乱跳。

附带两条不是坑但容易搞错：

- **Ri/R̄i 不需要新检测器**：它们是 Ru/R̄u 的形状打在 UINFO 上（8.6.5 + 8.5.5），
  hunt 直接报 `Heard::Ru` / `RuReversal`；Sd 同理报成 `Heard::Su`。
- **`JpPrime` 之后必须走 `Out::Dil`**（描述符要了 DIL）或 `Out::Scr`（没要），
  以前这条出口只去 `Silence`，DIL 从来没上过线。

## 5. 已知红（开工前就有，别当成你的回归）

- `cargo test -p dsp --test echo_drift`：plan 15.2 记的既有失败。
- `cargo test -p datapump --test v90_call`：**13 条**。已用 `git stash` 对照 HEAD 确认，
  与本分支无关（V.90 的速率重协商/整形用例）。
- `cargo test -p modem --release --test v90_call when_one_end_hangs_up_the_other_notices`：
  **1 条**，挂在 `v90_call.rs:430`（两端没进数据态）。已用 `git worktree` 在 HEAD~1 复现。

## 5.1 台架现状（2026-10-23，真机）

**Agere SV92PP 是首选台架**（8 kHz 原生、无 SRC、日志最全）：

```bash
cd /home/hikari/comm/softmodem && export PKG_CONFIG_PATH=/tmp/alsa-shim/lib/pkgconfig
export BRIDGE_MOD=V92 BINMODEM_V92_PCM_UP=1 V92_TRACE=1
python3 interop/agrsm/run_agr_v90.py --ms-args "V92" --pre "AT+PIG=0" \
        --connect-timeout 40 --hold 2 --verbosity 5 --capture
```

- **`BRIDGE_MOD=V92` 必须显式给**：Agere 脚本没有 `--bridge-mod`，默认把服务端起成
  `+MS=V90`（binmodem 日志里能看到 `AT+MS=V90`），那样永远到不了 V.92。
  §6 里"Agere 不发 Ru"的旧结论就是这个假象。
- 每轮台架约 2 分钟；改完 `datapump` 必须 `cargo build --release -p modem --bin v90-bridge`。
- 采集方向别搞反：`caller.raw` 是**我们自己的 TX**（bridge 写的是 `from_binmodem`），
  对端音频是 `agr-tx.raw`。但对端那份 raw **不可全信**：用它跑接收机链连 TRN1u 都拟合
  不过 0 dB，而在线的同一段是 15.4 dB，说明这份文件与线路上真正听到的不是同一个信号
  （或采样率/格式不同）。`tests/v92_capture.rs` 两个 `#[ignore]` 诊断就是为这件事留的：

  ```bash
  V92_CAPTURE=<run>/agr-tx.raw cargo test --release -p datapump --test v92_capture \
      -- --ignored --nocapture
  ```

**Agere 现在走到哪**：

```
Table 18 双向（P2TRACE: table18 true / table10 false，INFO1d bit70 = true）
V.92 phase 3: Ru(679) → TRN1u(15.4 dB) → Ja → Sd/S̄d → TRN1d(3.6 s) → Jd → Su → Jp
              → Jp' → DIL →（对端）CPt sent →（我们）解不出 → 对端 4 s 后 6009521a Cleardown
对端侧：Jd detected / Jp detected: aa57 1000（ε 已收到）/ Jd' detected / PCM phase 3 finished
```

**HSF（SoftV92）**：Phase 2 正常（bit70 → Table 18），Phase 3 到 **Jp** 就停——
桥里 16k→8k 是 3.4 kHz 带限，Su 的 hunt 一次都不匹配（`V92TRACE` 里没有 Su 事件）。
HSF 只适合验 Phase 2。

**为什么 CPt 解不出（已定量，`c4b4081`）**：

- CPt 解码链**本身是对的**：`a_cpt_reads_back_through_a_band_limited_line`（带限信道 +
  TRN1u 训练）能读回 Table 23 的 CPt，`CpFinder` 的 CRC 通过。
- 长窗重拟合只把 15.4 提到 15.6 dB → 方程数不是瓶颈。
- 瓶颈是 §4 坑 6 那张表：0.5 符号偏就是 19 dB，而 CPt 的 CRC 要 ~30 dB。

## 6. 三个模拟对端的事实

### HSF（Conexant SoftV92）—— 要 PCM 上行，但台架带限
```bash
cd interop/hsf && BINMODEM_V92_PCM_UP=1 \
  python3 run_hsf_v90.py --bridge-mod V92 --seconds 20 --connect-timeout 40
```
- `BINMODEM_V92_PCM_UP=1`（服务端 INFO1d bit70=1）时，HSF 回 **Table 18**：
  `sections=0, ltot=3, lmax=0, md_length=20, uinfo=78`。
- 不打开时回 Table 10，V.90 数据态 **21600/56000**、2000 B 0 错。
- **HSF 不要 QC**：早前 V.8 卡死是我们自己的缺陷——HSF 把 V.21 载波停在 CJ 第三八位组
  中间并立刻起 phase 2 音，线上变成 `00 00 80`。已在 `v8::Decoder` 修（常量 `DAMAGED_ZERO`，
  只允许 CJ 的**最后一个**八位组这样）。`framing.rs` 的 `RING_COAST` 保持 **false**。
- **台架不适合验 PCM 上行**：桥里 16 kHz→8 kHz 的 `Polyphase` 是 3.4 kHz 带限，
  而 V.92 上行 8000 符号/秒。

### Agere SV92PP —— 8 kHz 原生，Phase 3 **能**领跑（首选台架）
```bash
python3 interop/agrsm/run_agr_v90.py --ms-args "V92" --pre "AT+PIG=0" \
        --connect-timeout 40 --hold 2 --verbosity 5
```
- **必须 `AT+PIG=0`**（默认 `+PIG:1` = 忽略 PCM 上行），否则它回 Table 10。
- 8 kHz、无 SRC。`+PIG=0` + 服务端 `BRIDGE_MOD=V92` + `BINMODEM_V92_PCM_UP=1` 之后，
  Ru/TRN1u/Ja/Su/Jp'/DIL/CPt 全都发（§5.1）。
- 日志很富（`--verbosity 3/5`）：`V34 Phase 2 finished, modulation v.92`、
  `PCM phase 3 started`、`PCM Mu-law coding`、`Signal/Noise Ratio`、`Timing Offset`、
  `Jp detected: <ε> <bits>`、`CPt sent ...`、`Rx Data Mode Active!`。
- **ATI11**（引擎自己的连接报告）：脚本已实现——`ATH` 后等 ~6 s，先发 `ATI6` 顶掉
  ATH 的 NO CARRIER，再 `ATI11` + 多一个回车。输出含 `Last Connection`、收发速率、SNR、
  RTD、回声、重训计数。（`+++` 在这颗 build 上无法进入 online-command，试过，没 OK。）

### D-Modem golden —— 没有 V.92
`GOLDEN_DP=V92` 会打印 `change dp: --> 92 error`：`modem.c` 表里有 `DP_V92`，
但 dsplibs 只有 `V92Modulator/V92Precoder` 等类，没有 driver init。**只能做 V.90/V.34 对端。**

## 7. 构建、调试、回放

```bash
export PATH=$HOME/.cargo/bin:$PATH
export PKG_CONFIG_PATH=/tmp/alsa-shim/lib/pkgconfig   # 指向 ~/local/alsa，libdir 必须是真实目录
cargo test  --release -p datapump --test v92_call     # release 比 debug 快约 10 倍
cargo build --release -p modem --bin v90-bridge       # 台架必须重建它
```

- `V92_TRACE=1`：Phase 3 阶段 / hunt 事件 / `train_near` 候选 SNR / refit /
  Ja 与 CPt 读取器的位计数（`CPt reader N bits, M ones`）。
- `PHASE2_TRACE=1`：INFO0/INFO1a/INFO1c，含 **bit70 是否真的上线**、`table18/table10`。
- `V8_TRACE=1`：逐八位组。
- **回放（比实线快 20 倍，首选调试手段）**：
  ```bash
  python3 tools/binmodem_replay.py <对端.raw> --rate 8000 --start 4 --seconds 16 \
          --mod V92 --realtime 0
  ```
  `<对端.raw>` 是**对端发的东西**（Agere 是 `agr-tx.raw`）。喂自己的 TX 会得到"听到自己的
  ANSam"这种怪结果——踩过。

## 8. 文件地图

| 文件 | 状态 | 说明 |
|---|---|---|
| `datapump/src/v92/receiver.rs` | 成熟（1211 行） | Ru/Su hunt（含 `expect_ru/expect_su` 与 `SU_FILTERED`）、TRN1u 拟合（长窗重拟合、`fitted/restore`）、切片、history |
| `datapump/src/v92/digital.rs` | **Phase 3 完成**（1267 行） | 数字端 Ru→R̄i→TRN2d；Phase 4 是钩子；E1u 判定（坑 11）、SCR/DIL 出口、`refit_from`、Jp 的 ε |
| `datapump/src/v92/analogue.rs` | **Phase 3 完成**（1023 行） | 模拟端 A1–A11 状态机，到 E1u → `Status::Phase3Done`；`delays()`、`dil()`、`ZERO_DIL_A10_ORDER` |
| `datapump/src/v92/epsilon.rs` | **已实现**（127 行） | Su 的 6 点 DFT 相位 → ε；`to_field`/`offset`/`measure`；符号与量纲待核（§3.1） |
| `datapump/src/v92/down_source.rs` | 空桩（5 行） | §3.2 |
| `datapump/src/v92/{transmit,upstream,decoder,design,upchoice,modulus,precoder,exchange}.rs` | 完成 | PCM 上行链路，已被 `v92_upstream.rs` 验证 |
| `datapump/src/v92/{sequences,up_signals,anspcm}.rs` | 完成 | Ja/CPt/CPd 帧、Ru/Su/TRN1u、QTS/ANSpcm |
| `datapump/tests/v92_call.rs` | 完成（190 行） | 两端都是调制解调器的 Phase 3 呼叫：全程、带 MD、无 Sd |
| `datapump/tests/v92_upstream.rs` | 完成（189 行） | 链路 0 误码 |
| `datapump/tests/v92_capture.rs` | 诊断（153 行，`#[ignore]`） | 采集里的 CPt / TRN1u 拟合诊断 |
| `datapump/src/v90/startup.rs` | 已接 | `Digital::with_v92`；Table 18 → 建 V.92 调制器；`:617` 是失败重训 |
| `modem/src/lib.rs` | 已接 | `+MS=V92`、`V92S` 泵、`BINMODEM_V92_PCM_UP` |

## 9. 关键 API 速查

```rust
// 接收
let mut rx = Receiver::new();
rx.set_reference_mode(Mode::Call);      // 读数字端的 TRN1d（GPC）；读模拟端 TRN1u 用默认 Answer
rx.expect_ru();                         // 现在只等 Ru（Phase 3 开头）
rx.expect_su();                         // 只等 Su（答 Jd、Sd 的形状）
rx.feed(sample);                        // 每样点
while let Some(h) = rx.heard() { /* Ru/Su/RuReversal/SuReversal/Trained/Symbol/Lost/Found */ }
let saved = rx.fitted();                // taps + snr + trained
rx.train_near(at, 64)                   // at 是 history 相对下标；失败会关掉 trained
rx.restore(&saved);                     // 试第二次拟合后回滚
rx.history_since(n) / rx.taps()         // 给 design::Channel::estimate

// 发送
ru(n, bar); su(n, bar);                 // 6 符号序列（period = RU_PERIOD = 6）
let mut trn = Trn1u::with_mode(Mode::Answer);   // 模拟端就是 Answer（默认）
let mut sender = TwoPointSender::after(Trn1u::new());
sender.preamble();                      // 24 个 1
sender.push(&Descriptor::none().to_bits_in(Some(0x7ffff)));  // Ja
sender.push(&cp.to_bits_in(Layout::V92, UP_INTERVALS));      // CPt：12 比特为单位，别传 1
sender.e1u();
sender.next_symbol();                   // Option<f64>，耗尽后按需重填

// ε
epsilon::measure(&su_symbols) -> Option<u16>;   // su_symbols 是 Su 那一段（每符号一个值）
epsilon::to_field(0.25) == 0x4000;

// 用一个模拟端跑 Phase 3（v92_call.rs 的做法）
let mut modem = analogue::Modem::new(analogue::Settings {
    md_length: 0,                       // INFO1a 的 MD，单位 276 符号
    ..analogue::Settings::new(Descriptor::none(), 0x7ffff, cp)
});
let out = modem.step(from_digital);     // 一个符号进、一个符号出
modem.status() == analogue::Status::Phase3Done
modem.far_jd() / modem.jp() / modem.epsilon() / modem.circuit107()
modem.delays()                          // [0.5, ε]：给 PcmTransmitter::delay 吃
modem.dil()                             // DIL 的原始样点，等 V92-39 解析

// 读取对方的帧
let mut reader = TwoPointReader::differential_with(Mode::Call);  // Jd/Jp
reader.feed(level) -> Option<bool>;
J::from_bits(&window72) -> Option<J>;   // 72 位滑窗，CRC 校验
```

## 10. 这一轮（2026-10-23）改了什么

台架第一次接真机：四处堵点、六处修复。

| 提交 | 内容 |
|---|---|
| `e2effb3` | DIL 从未上线（`JpPrime` 出口不接 `Out::Dil`）；Su 被判成 Ru（`SU_FILTERED` + `expect_*`）；Su 超时少算 TRN1d（从 TRN1d 开始算 5100 ms）；E1u 判定改成"同步位上的 12 个零"；CPt 填充单位 12 比特；N=0 走 SCR→Ri |
| `87c3a0d` | `a_cpt_reads_back_through_a_band_limited_line`（证明解码链正确）+ `tests/v92_capture.rs` 两个诊断 |
| `c4b4081` | 长窗重拟合（1200→4800）+ 定时上限的表（0/¼/½/¾ 符号 → 166/156/**19**/157 dB） |
| `d9f31a2` | 第二 TRN1u 的重拟合（规范 #4）+ `Receiver::fitted/restore`（失败的 fit 会关训练） |
| `7814625` | `v92::epsilon` 真正实现：Su 的 6 点 DFT 相位 → Jp 的 ε（台架从 `0 1000` 变成 `aa57 1000`） |
| `6066ce6` | 台架现状与下一步（本页的上一版） |
| `d1ea3d8` / `431570b` | 本页整体重写；`plan.md` 的 `ZERO_DIL_A10_ORDER` 改成"on" |

**留下的东西**：§3.1 的两条路（ε 的符号/量纲核对、第二 TRN1u 的定位）是唯一还没闭环的；
再往后是 Phase 4。
