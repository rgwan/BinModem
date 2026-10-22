# V.92 交接（2026-10-23）

给接手 V.92 的下一位。读完这一页就能开工：现状、下一步、踩过的坑、台架事实、命令、文件地图。
**所有数字都是实测的**；不确定的地方都标了"未验证"。

---

## 0.0 第三轮取证（结论：管道是好的，第二轮的上行误码判定成立）

加了两个一手探针（都已在仓库里）：
- `V92_RX_TRACE=<file>`：`rx.feed()` 收到的每个样点（`样点序号 电平`）；
- `V92_UP_TRACE=<file>`：每个上行符号（`样点序号 电平 解扰后的比特`）。

**先纠一个中途的错判**：`rx` 从本端 16 s 起确实被冻在一段 **周期 40 样点、1800+2400 Hz、
mean|x|=0.28** 的循环音上，但那**不是管道坏**，是**对端清线之后的残响**：
`0x962140` 清线（对端 27.68 s）之后，对端进程仍在往线上推同一块 5 ms 的缓冲。
证据：给 `agrsm-softmodem` 的 `line_flush_out` 加了一路 `AGRSM_SOCK_TX_CAPTURE`（socket 上
**实际写出**的字节），用**包络**（40 样点平滑 + 40 倍抽取）做互相关，与本端 `rx` 在
**恒定偏移 12.39 s** 上 **0.997–0.999**——管道是直连、不丢不重、没有速率差。

于是时间轴对上了（本端 = 对端 − 12.39 s）：

| 本端 | 对端 | 线上是什么 |
|---|---|---|
| 11.8–16 s | 24.2–28.4 s | 对端真的在发：TRN2u、`CPu`（其 25.998 s）、`SUVu`/`SUVu'` |
| 13.86–15.5 s | 26.2–27.9 s | 我们 CRC 通过的那 137 条 **SUVu**（`ack` 从 14.76 s 起为 1） |
| 16–25 s | 28.4–37.4 s | 清线后的冻结循环音（假信号，别拿它算误码） |

⇒ **第二轮的上行误码判定成立**：`CPu` 在本端 ~13.6 s 就**到达**了，是**我们解不出来**。
冻结音那段是清线的后果，别混进统计。

## 0.0b 第四轮：上行解调的**拟合**没问题，喂给它的信号不是模型里的样子

按要求检查 BinModem 的上行解调（`v92/receiver.rs`）。结论分两半：

**（一）代码是对的，而且已经调到它的最优。** 把同一段 `V92_RX_TRACE` 数据用 numpy
**独立重解**一遍最小二乘（48 抽头、滞后全域搜索）：

| 模型 | 最佳 SNR |
|---|---|
| **GPA(5,23) 正极性、1 样点/符号** | **15.63 dB**（与 `Receiver` 报的 15.6 dB 完全一致） |
| GPC(18,23) | 0.24 dB |
| GPA 反极性 | 0.03 dB |
| 2 样点/符号（零阶保持） | 0.22 dB |
| 抽头 8/16/24/32/48 | 10.9 / 13.2 / 14.2 / 14.8 / 15.6 dB |

⇒ `fit_window`/`equalise_and_slice` 的抽头顺序、归一化（`LU = 1.0`）、参考序列、
滞后搜索都是对的；15.6 dB 是这套模型的**上限**，不是代码 bug。

**（二）但输入不是 ±LU 两点信号。** 我们训练用的那一段（`rx` 0.2–2.8 s，白、前后不相关）：

- `mean|x| = 0.0739` 而 `rms = 0.1023`（比值 1.38；纯 ±LU 时二者相等）；
- `|x|` 分位 0.00 / 0.014 / 0.060 / 0.107 / 0.180 —— **幅度连续分布，含大量接近 0 的点**；
- 拟合残差被 **4 kHz（隔样点交变）成分**主导：`corr(e[n],e[n+2]) = 0.97`，而信号本身
  `corr(x[n],x[n+2]) = −0.07`（即交变只出现在**误差**里）。

⇒ 我们的解调器是"在线性模型下已最优"，缺的是**模型里没有的东西**：要么这段不是 TRN1u
（时间轴 12.39 s 的映射还不够硬），要么对端上行发出来的就已经是这种形态。

**下一步（两条，都很便宜）**：
1. 给 `Receiver` 加一路 `V92_EQ_TRACE`：训练时落盘 `reference[]` 与均衡器输出 `model[]`、
   以及 48 个抽头。这样"残差长什么样"可以直接看，不用再靠推断。
2. 两端时间轴**一次钉死**：`V92_RX_TRACE` 与 `AGRSM_SOCK_TX_CAPTURE` 都写
   `CLOCK_MONOTONIC` 时间戳（现在一个写样点序号、一个写裸字节），再对齐就不再靠猜。

## 0.0c 第五轮：把"15 dB"追到了一眼 4 kHz

加了两个探针（都已提交）：
- `V92_EQ_TRACE=<file>`：训练胜出的那次拟合的 `from`/`snr`/**48 个抽头**，以及逐样点的
  `reference / model / error`（`Receiver::dump_fit`）；
- **墙钟时间锚点**：`V92_RX_TRACE`/`V92_UP_TRACE` 写 `<file>.t0`（epoch 秒），
  `AGRSM_SOCK_TX_CAPTURE` 写 `<file>.t0`。两端第一次能按**同一时钟**对齐，不再靠猜。

**结论一：bridge 是透明的（用户的判断对）**。对端 socket 流与本端 `rx` 在对齐位置上
`mean|x|` 0.0983 vs 0.0987（比值 **0.996**），同一个窗口的最小二乘拟合两边都是
**15.43 / 15.44 dB**，残差形状一致 ⇒ 没有线模拟器、没有丢样、没有速率差，问题不在这段管道。

**结论二：我们的拟合已经是符号间隔线性模型的最优**。独立重解 + 分数延迟扫描（0/±0.1…±0.5 样点）
最好点就在 **d = 0，15.43 dB**；参考假设里 GPA(5,23)/正极性/1 样点/符号 比其它选项好 15 dB 以上。

**结论三（新，也是下一轮的入口）：拟合在追一眼 4 kHz。**
- 48 个抽头是**交替符号、幅度单调增长**的（0.21, 0.005, 0.275, −0.007, 0.315, −0.032, 0.348,
  −0.057, 0.398, −0.095, 0.449, −0.134, 0.516, −0.168, 0.614, −0.194 …）——符号间隔滤波器
  只有在输入里有一个**它想挖掉、接近正交的窄带分量**时才会长成这样（ridge 1e-9 压不住）。
- 残差 `corr(e[n], e[n+2]) = 0.97`，而信号本身 `corr(x[n],x[n+2]) = −0.07`
  ⇒ 交变（**正好 4 kHz = Nyquist**）只存在于误差里。
- 分数延迟救不了它（d 扫描无改善），所以不是"半符号定时"错。

**对端上行信号地图**（`AGRSM_SOCK_TX_CAPTURE`，逐 0.5 s）：
`19.0 s` = **Su**（x 的 6 周期自相关 +0.98）；`20–27.5 s` 白、多电平
（`|x|` std/mean **0.72**，理想的四点只有 0.50）—— 才是 TRN2u/CPu/SUVu；
`28–37.5 s` = 清线后的冻结块（假信号）。

**下一步（唯一一条）**：找出那眼 4 kHz 的来源。三个候选，按代价排序：
1. 对端 socket 流的**频谱**（看 4000 Hz 上有没有线；顺手看 0 Hz 有没有 DC）；
2. 对端自己的接收（`agr-rx.raw`，即我们下行）频谱对照 —— 若 4 kHz 是我们下行漏进来的，
   就是它的回声消除器在本移植里没配好；
3. `hw.c` 的 `hw_dma_tick` 里 RX/TX 环有没有串路（一次插值/取整造成的隔样点结构）。

## 0.0d 第六轮：根因是一道 **3.8 kHz 低通**，在**对端的上行发送**里

把上一轮那眼 4 kHz 追到底了（数据：`AGRSM_SOCK_TX_CAPTURE`，对端 20.5–27.0 s 的上行）：

**对端上行频谱（相对 0.5–2 kHz 均值）**

| 0–200 | 0.5–1k | 1–2k | 2–3k | 3.25–3.5k | 3.5–3.75k | 3.75–3.95k | 3.95–4k |
|---|---|---|---|---|---|---|---|
| 0.65 | 0.99 | 1.01 | 1.02 | 0.98 | 0.89 | **0.27** | **0.00** |

即：**平到 3.5 kHz，3.75 kHz 以上被砍掉**。两点 TRN1u 是白谱，3.75–4 kHz 里约有一成的能量，
被砍掉后最小二乘必须把它抬回来——我们 `V92_EQ_TRACE` 里那 48 个抽头的频响正是这样：

```
|H(0)| 2.95   |H(500)| 7.96   |H(1k)| 8.44   |H(2k)| 8.30   |H(3.5k)| 8.26   |H(3.9k)| 16.7   |H(4k)| 22.3
```

Nyquist 上 **+9 dB** 的增益就是"把被砍的 4 kHz 抬回来"，代价是把那里的噪声一起放大
⇒ 拟合封顶 15.4 dB、残差在隔样点交变、上行误码 ~8% ⇒ 1377 bit 的 CPu 永远过不了 CRC。

**这是对端上行独有的**：下行没有这个限制（对端解我们的 CPd/SUVd 报 SNR 44 dB，4 bit/符号照解）。
也确认不是我们的移植代码里的滤波器：`hw.c` 的 DMA 只是环读写（`hw_dma_tick` 一进一出加中断），
`line.c` 只做 FIFO/捕获；所以低通在 **`agrsm_core.o` 的上行发送 DSP 里**。

**下一步（唯一一条）**：在 `agrsm_core.o` 里找这道上行发送滤波器的**选择开关**。
做法：`.rodata` 里找对称 FIR 系数表（~30–60 抽头、截止 3.4–3.8 kHz），再找引用它的代码与
决定"窄带/V.92 宽带"的状态字；真实驱动会在进 V.92 上行时把它换掉（我们的移植很可能没写那个
参数，于是引擎一直用窄带）。找到后由 `glue.c`/`hw.c` 在进 V.92 时设上，上行频谱应平到 4 kHz，
`CPu` 的 CRC 就能过，`SUVd′ → Ed/E2u → B1d/B1u` 才有意义。

## 0. 下一步（新 session 从这里开始）

**目标没变**：让 BinModem（数字端）与 Agere SV92PP 走完 V.92 建链并传数据。

### 0.1 这一轮解决掉的（根因，真机验证过）

上一版判定的根因——**Phase 4 下游用两点信号，对端按 CPt 星座切 24 bit/帧**——已经修了
（`f76e9c0`）：TRN2d/SUVd/CPd/Ed 现在全部走 `v90::encoder::Encoder` +
`Mapping::from_cp(CPt)`，一段数据帧 6 个码字、`drn+8` 比特，序列按 **整数据帧** 填充，
Ed 是 **2 个数据帧**的扰码零。真机结果（`005855-run`）：

| 对端日志 | 之前 | 现在 |
|---|---|---|
| 检测器 `acq` | 恒 0 | **从 25.3 s 起恒 1** |
| `SUVd detected` | 从未 | **25.956 s** |
| 对端回 `CPu sent` | 从未 | **25.956 s** |
| 对端回 `SUVu' sent a001`（ack=1） | 从未 | **26.628 s** |
| `CPd detected` | 从未 | **26.628 s**（`1e 6f12`） |
| 结果 | `60096212`/`60096211` | **`60962140`（27.63 s，新的、更靠后的堵点）** |

我们自己的线上信号：`caller.raw` 相位 3 的 TRN1d 仍是 1 个幅度（两点，正确），
相位 4 变成 **8 个幅度**（CPt 星座）。

### 0.2 现在的堵点：上游 CPu 收不到 → 我们的 ack 永远是 0

`0x962140` 抛在 `FUN_0006d550`（对端在收到 SUVd 之后进入的 Rx 状态）：

```c
if ([0x11698] <= [0x11694]) {          // 8000 样点 = 1 s
    if ([0x116b8] == 0) V92ClientControlException(0x962140);
    else                V92ClientControlClearDown(0x22);
}
if (*(short *)([0x11754]) == 0) return;   // 还没收到 ack=1 的 SUVd，继续等
```

`[0x11754]` 只在 `FUN_0006a010` 里被置：**收到的 SUVd 的 ack 位（Table 31 bit 33 = 已收到
CPu）为 1**。也就是：它给了我们 1 s 去回一个 **SUVd'**，而我们回不出来 ——
因为我们的 ack 位来自 `Exchange`，`Exchange` 要**先解出它的 CPu**，而它 25.956 s 就发了
`CPu sent 7a 0 2000 0 0 0 100 2000`，我们这边日志里 **`V.92: CPu drn …` 一次都没有**。

（对比：它的 `SUVu' ack=1` 我们**解到了** —— `V.92: SUVu, ack true` 从第 357 行起连成串，
说明上游的 `Trn2uReader` + `UpFinder` 通路、解扰、定时都基本对；缺的是 CPu 那一档。）

### 0.3 下一步（按顺序）

1. **查 CPu 为什么解不出来**：`Trn2uReader`（4 点）+ `UpFinder` 在 `v92_upstream.rs` 里自环是绿的，
   真机上 `SUVu` 能出、`CPu` 不出。先看 `AGRSM_EVT_TRACE`/我们自己 `V92TRACE` 的时机：
   对端的 `CPu sent` 在我们发完第一组 SUVd 之后立刻开始，我们的上游 reader 在
   `begin_upstream()`（TRN2d 起点）就装好了，所以嫌疑按顺序是：
   - **CPu 的长度/可选段**（Table 23 type 1）与我们 `UpFinder` 的窗口不一致（CPu 是我们这里
     最长的上游序列）；
   - **TRN2u 的点数**（`Jp` bit 48：四点 vs 八点）与它实际发的不同（我们现在硬编码 `Four`）；
   - 上游 reader 的**符号定时**（它从 TRN2u 起点算，`begin_upstream` 用的是 `trn2_from`）。
   验收：我们日志里出现 `V.92: CPu drn …`，并且随后出现 `V.92: SUVu, ack true`（我们自己的）。
2. 之后：对端 `CPd detected` 已经成立，接下来是 **Ed → E2u → B1d/B1u → 数据态**。

**别再重复做的**：下游帧结构/长度/CRC/字宽、解扰多项式（`x18x23`）、检测器 arm、能力字、
`[0x8654]` 门闩、TRN2d 长度、V.90 当对照、"`[0x8614]` 在变"、"相位 4 里 `[0x897c]` 没被调"、
"用 `agr-rx.raw` 读自己的读出 SUVd 就是对端收到了"（坑 29/31）。

**每轮台架基线**：这一轮 `CPt sent … 100 …`、`Jd detected: ffff 403f`、异常码 `0x962140`；
上一轮是 `0x96211`、再上一轮 `0x96212`。**异常码每轮不一样，先读当轮 `AGRSM_ENGINE_TRACE` 尾部。**

---

## 1. 一句话现状（2026-10-23 夜）

- **Phase 2 / Phase 3（双向全程）/ Phase 4 握手代码全部已实现并有测试**；
  `--lib` **657 条**、`v92_call` **5 条**、`v92_upstream` **2 条**。
- **真机 Agere SV92PP 上 Phase 3 双向走完**，双方进入 Phase 4，对端 `PCM phase 4 started`。
- **Phase 4 下游信号已确认正确上线且对端能干净收到**：`caller.raw` 里 SUVd 每 0.5 s 74 个帧同步
  （= 8000/54）、CPd 每 0.5 s 约 1 个；`agr-rx.raw`（对端收到的）里同样可读。
- **堵点**：对端的序列检测器在相位 4 从不被喂 → 10 s 超时 → `60096212` → V.34 phase 2 冷重启。
  细节与已排除项见 §3.2。
- 未做：B1d 的 DTE 位流仍是常量；上游只出比特、无字节出口；从未到 CONNECT。

**这一页的读法**：§0 是下一步；§3.2 是堵点的全部证据链；§3.2.1 是探针工具；
§5.1 是台架命令与采集口径；§6 是坑清单（**20/21 是本轮新加的**）。

## 2. 已经能跑的东西

| 东西 | 命令 | 结果 |
|---|---|---|
| **Phase 3 全程，两端都是本项目的调制解调器** | `cargo test -p datapump --test v92_call -- --test-threads=1` | **5 条**：双向 Phase 3、带 MD、无 Sd 时 AR1 报警、**分数符号抖动的线路（0.2/0.3/0.45/0.7）仍然建到 Phase 4 并读出 CPt**、**对端不应用 ε 时靠第二段 TRN1u 重拟合救回** |
| V.92 各模块单测 | `cargo test -p datapump --lib v92::` | **158 条**全绿（receiver/digital/analogue/epsilon/exchange/upstream/…） |
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

### 3.1 这次解决了什么（已闭环，勿重做）

1. **ε 的测量位置与量纲**。原来在均衡器输出上测、而且测的是 **TRN1d 的尾巴**（`su` 缓冲从
   Ja 就开始收，收满 144 个就走）。现在按 9.5.1.1.6-7 在**线路原始样点**上测 1333.3 Hz 基波，
   窗口取"首个 S̄u **之后**那段 Su"的 96 个样点（`SU_AFTER_S_BAR`/`SU_MEASURE_SYMBOLS`），
   并且带质量门（`epsilon::COHERENT`：非 Su 的窗口直接返回 None，原来那些 TRN1d 符号正是
   所以 ε = 0xaa57 的来源）。台架实测 lag = 0.2559 符号，重复性好到 0.001。
2. **ε 的方向是约定，不是测量**。`epsilon::EXTENDS_LATE = false`：Agere 把 ε 当作**提前**。
   证据是两次台架：ε = 0x4180（正是 lag 本身）→ CPt CRC 过、Phase 3 双向走完；
   ε = 0xbe80（同一个 lag 取反）→ CPt 一次没过、对端 4 s 后清线。这一条同时说明了
   `Receiver::expect_step` 要拿 `epsilon::step_direction() * …`。
3. **第二段 TRN1u 的重拟合**。锚点原来多加了 `TRAIN_AFTER`(1300)（拟合的是 TRN1u 中间某段，
   参考序列从 0 开始 → 永远对不上），搜索只有 ±64。现在：锚点 = 第二段 TRN1u 的起点估计，
   用**相关定位**（`Receiver::align_trn1u`，±192，质量 = 归一化相关，实测 0.70–0.94），
   然后用 `train_verified` 直接拟合（不走 12 dB 门：4 kHz 处线路没有内容，两点流的拟合上限
   本来就只有 13–15 dB）。同时 `set_frame_origin` 落在这里（9.5.1.1.10 的模 12 计数）。
4. **换向检测必须匹配"已知时步"**。S̄u 的 24.5T 是**半符号时步**，而 Su 近似单音：
   "反相 + 半个符号"对单音等于转 30°，原地相关永远看不到（老代码就是这么漏判的）。
   `turned_share` 同时试**原地取反**和**按 `expect_step` 的时步 re-read**（`reread` 是 6 点 DFT
   的带限移位，负频 bin 的符号是坑，见 §4）。
5. **Ri 的帧边界**（V.90 8.4.1）：Ri→R̄i 现在对齐 6 符号帧。对端靠数 R̄i 的 24 个符号确定
   **TRN2d 的扰码器复位点**（V.90 8.6.5），差 1 个符号，它后续解出来的就全是噪声。
6. **Agere 的 `Jd' detected` 读法**：它是 V.90 8.6.2 的**重训练请求**，不是 Progress 事件。
   反编译 `StatusParser`（Ghidra，`/tmp/DecompileAgere.java`）确认：状态码 0x4c，属于
   "training progress" 组（`*param_1 == 0x42`）。

### 3.2 当前唯一的堵点：对端收得到 SUVd/CPd，但一个都没认出来

**先说不堵的那一半。** Phase 4 的序列现在已经整条整条地上了线（`c1b05d3`）：

`Source::send` 过去会在交出序列时 `bits.clear()`，把**正在发送的那条序列**清空，于是下一个
符号就发现队列空了、报"序列结束"，交换层立刻要下一条，又清空一次 —— 每条序列只播得出
一个符号。SUVd 和 CPd 的第 0 位都是帧同步的第一个 `1`，所以线上实际是**一段扰码全 1**，
里面一个帧同步都没有。修掉之后（`send` 只写 `last` 和 `pending`，队列交给 `start`）：

```
caller.raw（我们发的）  22.0–24.0 s  每 0.5 s 74 个帧同步 = 8000/54，SUVd 背靠背
                        24.5–29.5 s  密度 0.15–0.19、每 0.5 s 1 个同步 = 3732 bit 的 CPd 重复
agr-rx.raw（对端收到）  25.0–26.5 s  67–74 个帧同步  ← SUVd 在对端是**干净可读**的
                        27.5–33.0 s  每 0.5 s 1 个同步  ← CPd 同样可读
```

两份 capture 的时间基 ≈ +3.0 s（`agr-rx` 晚），与 §5.2 的 24192 样点一致。读取工具
`crates/datapump/examples/v92_capture.rs`（`--levels` / `--sync FROM TO` / `--down` / `--up`）。

**堵点。** 对端照旧：`SUVu sent 2001`（ack=0）循环，10 s 后 `V34 exception: 60096212`。
`StatusParser` 里有 `SUVd detected` / `SUVd' detected` / `CPd detected` 三条串，日志里
**一条都没出现**，所以是它的检测器不认，不是我们发得不对。

`60096212` 的完整含义（Ghidra + objdump，已闭环，别再追）：

- `V92ClientControlException(0x96212)` → `V34Exception(0x96212 | 0x60000000)` →
  `V34ReportStatus(0x21, …)` → StatusParser 状态码 `0x21` = `"V34 exception:  %8lx"`。
  **`0x60000000` 是 V.92 client 控制层加的类位**，不是对端数据。
- 低 28 位 `0x96212`：`9`=V.92、`6`=phase 4、`21x`=状态序。抛出点只有两处：
  `V92ClientPhase4RxWaitSUVState`（0x5d242）与它的孪生（0x5d514）。
- 判据就一条超时：`if ([0x11698] <= [0x11694])` → `[0x116b8]==0` 时抛 0x96212，否则
  `ClearDown(0x22)`。deadline 在 `V92ClientPhase4TxSendTRN2uState` 里装（0x5d05f）：
  `[0x11698] = 当前样点位置 + 0x13880` = **80000 样点 = 10 s @ 8 kHz**（`AGR_FS=8000`）。
  同时清 `[0x11753]`、`[0x11752]`，arm `V34RxDataSetCPdDetector`，装 RxWaitSUV。
  实测：`SUVu sent`(25.9) → 异常(34.76) = 8.86 s，落在 24.75 起算的 10 s 上。
- 该状态唯一出口是 `[0x11753] != 0`，而全区只有一处把 `0x11753` 置 1：0x5a077，在
  "把收到的序列信息字存到 `0x11ade`"的助手（`type==4` 即 SUV）里 —— 也就是**必须解出一个
  SUVd**。`0x11753` 一直是 0，所以它连 CPu 都没回（9.6.2.1.2 要求收到 SUVd 才回 CPu）。
- 后果：`V34P2ControlExceptionHandler` 把 0x96212 归一成动作码 **0xb521**（0x22150
  `cmp $0x96212` → `je 0x22080` → `mov $0xb521`）→ 走 0x21a60：Reset Xmt/Rcv 延迟线、
  DPSK 重初始化、Xmt/RcvMode=0 → **V.34 phase 2 冷重启** = 日志里的 `V34 phase 2 started`
  与 `Training failed`。对比 `0xb522`（0x9522/0x9722）走 `...AnsRxWaitForRetrainState`，
  `0xfff21` 走 WaitForRetrain —— 0x96212 没有这条路。

**对端的检测器已经被反编译清楚了（`/tmp/agere-cpddet.c`、`/tmp/agere-mpdet.c`）。**

`V34RxDataSetCPdDetector(buf)` 只是 arm 共用的 MP 检测器：`V34RxDataSetMPDetector()` →
`[0x8664]=1`（CPd 模式）、`[0x8660]=buf`、`[0x8668]=4`、`[0x864c]=4`、`[0x8648]=0`。
`V34RxDataMPDetector` 干的事，和我们的 `Cpd::to_bits`/`from_bits` 一一对应：

1. **捕获**：只认一个窗口——`(shift >> 15) & 0x1ff == 0x1ff`（移位寄存器里 **9 个连 1**），
   再数连续 1 的长度 `iVar10 = iVar17 + 9 + local_7c`，`iVar10 > 16` 且当前字节不是 0xff
   时置 `[0x863c]=1`。注意它要求**中间那个字节整字节是 0xff**，所以 17 连 1 只有在
   9 种对齐里的 8 种能被捕获——够用，但说明"17 个 1 一定被认"是不成立的直觉。
2. **读头部**：从缓冲头开始看 bit19/20/21（`**buf & 2/4/8`），据此设 `[0x866c]`（哪种可选段）
   与 `[0x864c]`（总长）。
3. **算总长**（单位是**去掉 start bit 的数据字节**）：
   `4 + 12·[bit19] + (8 + 2·ΣL)·[bit20] + (10 + 2·ΣLC)·[bit21]` 字节。
4. **收齐 + CRC**：`BlockCRC([0x8648], buf, &crc)`，`crc==0` 才报事件 9（"序列收到"）并置
   `[0x865d]=1`；否则丢弃重来。
5. `[0x8654]` 是**一次性**门闩，只有 `V34RxDataSetJdDetector` 置 1，CPd/MP 的 setter 清零
   —— 不是阻塞点。

**这一条把一整类假设排掉了。** 拿台架那条 CPd（12 个 moduli、p1=64、z2=128、一个 8 点星座）
代入上面的公式：

```
4 (头部) + 12 (6 个模数字 x 16b) + 8 (4 个长度字 x 16b) + 2x192 (系数) + 10 (5 个星座字) + 2x8 (点)
= 434 字节 = 3472 bit
我们 Cpd::to_bits 的信息段            = 3472 bit        ← 逐字节相同
```

所以**帧结构、字宽（16 数据位 + 1 起始位）、可选段顺序、长度、CRC 覆盖范围都是对的**，
"我们发的 CPd 不符合 Table 30" 这条不成立。

### 3.2.1 探针与钩子（关键工具）：对端内部状态，无扰动

gdb watchpoint 会单步整个进程，8 kHz + DSP 就不是调制解调器了。改成在用户态移植里
**每块采一次**（`agrsm-softmodem/src/glue.c`）：

```bash
export AGRSM_PHASE4_PROBE=/tmp/p4.txt   # 检测器/接收机状态，一行一个变化
export AGRSM_EVT_TRACE=/tmp/evt.txt     # "收到序列"事件（含内容）
export AGRSM_SCR_TRACE=/tmp/scr.txt     # 进解扰器的切片字节 + 出来的比特
export AGRSM_BOX_TRACE=/tmp/box.txt     # 引擎内部邮箱 0x70 的 (code,arg,arg)
```

- 探针字段：`acq`(0x863c) `n`(0x8648) `len`(0x864c) `pos`(0x8640) `shift`(0x8614)
  `caps`(0x11718) `cfg`(0xba27/30/34/38) `dmod`(0xba2a/b) `det`(0x862c) `tick/pwr`(0xba50/60/64)
  `b25/b26/b27`(0xba25/26/27)。
- **安全前提**：`gGlobalPtr` 前 3 秒不指向引擎状态；三个钩子都先验 `[0x8628]` 是否为
  `DescramblerX18X23/X5X23`（引擎起来后才有的签名）才读写。不校验会写坏引擎（踩过）。
- `[0x8610]` = `V92ClientControlEventHandler`：**case 9 = 收到序列** → `FUN_0006a010(len, buf)`；
  `FUN_0006a010` 用 **`len == 4` 判 SUV**（存 0x11ade，置 `[0x11753]`）、否则判 CP
  （存 0x11ad8，置 `[0x11752]`，并调 `V92DataModeEncoderParseCPData`）。
  我们 SUVd 去掉 start bit 正好 16+16 = **4 字节** ✓（第三次独立印证帧正确）。

### 3.2.2 对端 Phase 4 的完整画像（上一版，**读之前先看 §3.2.3**）

> 这一节的"`V34RxData` 相位 4 = 0 次""检测器从不被喂""我们发的都对"三条已被
> §3.2.3 推翻（探针过滤造成的假象）。保留它是因为里面的解调器/U.90 对照仍然有效。


| 观测 | V.92（卡住） | V.90（能通） |
|---|---|---|
| `V34RxData` 调用（喂检测器的唯一入口） | 相位 3 每秒 ~170 次，**相位 4 = 0** | 相位 3 ~160 次，相位 4 只有 24/26 s 各 1 次 |
| 解调器模式（`[0xba1c]/[0xba20]` 回调对） | 12 s Idle → 相位 3 **RcvOnly** → 相位 4 **FullDuplex** | **PCM 解调器完全没用**（callbacks = 0） |
| 检测器 | `det=MP`、`mode=1`、`len=4`、`acq/n` 恒 0 | — |
| 解扰多项式 | `x18x23` ✓ | — |
| 能力字 `[0x11718]` | `0xa01f9fb5`（对端要检的 bit 0/2/5/8/12 全置） | 0（不用） |
| 切片段前置条件 | `[0xba25]=1`、`[0xba30]=2` → 条件成立，块处理在跑（`[0xba50]` 24 s 起循环 0..3） | — |
| `[0x11a50]`（V.34 指针分发的解码器槽） | **全程 0**（指针路径没人用） | — |

**结论一：对端在 V.92 相位 4 走的是它的 PCM client 解调器（FullDuplex），不是 V.34 数据泵；
而唯一那个序列检测器（`V34RxDataMPDetector`）只有 `V34RxData` 能喂，`V34RxData` 的调用者
只有 `V34TrainingDecoder`(指针槽=0)、`V34DataAssembler`、`V90TrainingDecoderGetRcvdMaps`
（只被两个 **相位 3** 状态直接调）、`V90DataModeDecoder`。所以相位 4 里检测器拿不到比特。**

**结论二（新，很关键）：我们的 CPd 确实被对端"看见"了。** 把下游改成只发 SUVd
（`V92_NO_CP` 实验，已回滚）后，对端**不再能退出 `TxSendTRN2uState`**，改报
**`60096211`**（`V92ClientPhase4TxSendTRN2uState` 的异常），且提前到 25.88 s；
正常轮它 24.75 s 就退出 TRN2u 并 arm 了 10 s 的 SUV-wait 窗口。⇒ 相位 4 里我们发的 CPd
对它能"往下走"是有作用的，说明它的下游接收**至少能解出 CPd**，缺的是 **SUV 那一档**。

**结论三（本轮最后一条，链条已指到具体函数）：对端的解码器入口是 `[0x897c]`。**

```
PCMClientDataPumpInit 装：  [0x8974]=V92ClientControlRcv  [0x8978]=V92ClientControlXmt
                            [0x897c]=V90Decoder
V90Decoder(0x43dc0) 每块分发 →  call *[0x13a7c]
V90DecoderSetDataMode 装：  [0x13a7c] = V90DataModeDecoder
V90DataModeDecoder 的两处调用 → V34RxData → DescramblerX18X23 → V34RxDataMPDetector
```

探针实测 `[0x13a7c]`：12–22 s 是另一个 `V90...` 局部函数，**24 s 起（相位 4）切换成
`V90DataModeDecoder`（0x0809c950）** —— 也就是**对端确实把那条"会喂检测器"的解码器装上了**。
可 `V34RxData` 仍然 0 次调用、`acq/n` 恒 0 ⇒ **装了但没被调**，缺的是"谁每块调 `[0x897c]`"。

**下一步（唯一一条）**：查 `PCMClientDataPumpInit` 装的三项里谁每块被调，以及调用
`[0x897c]`（`V90Decoder`）的那个函数**门条件**是什么（探针加 `[0x8974]/[0x8978]/[0x897c]`，
再对 `V90Decoder` 的地址下断点或在 glue 里包一层计数）。
**验收**：说出相位 4 里 `[0x897c]` 被调用与否、若没被调是因为哪个标志。

老的下一步（若上面通了再看）：
`len==4` 只是进 SUV 分支的门票；它还要把 4 字节信息字存到 `0x11ade` 并置 `[0x11753]`，
而 `RxWaitSUV` 的推进还要求 `[0x8800]==0`。两个具体怀疑：
1. **4 字节内容**：我们对端那个窗口里 `[0x11adf] & 0x40`（= SUVd bit 32 静默位）与
   `param_2[1] < 0`（= bit 33 ack 位）的判读是否与我们发的 ack=0/静默=0 一致；
2. **`[0x8800]`**：RxWaitSUV 推进的第二条件，至今没人读过它是什么。

### 3.2.3 2026-10-23 第二轮：上一版的三条结论被推翻，根因是下游调制（已闭环）

上一版的 §3.2.2 有三条是**探针假象**，本轮用新钩子逐条推翻。新工具：`AGRSM_DET_TRACE`（见 §5.1）。

**推翻 1："相位 4 里 `V34RxData` 0 次调用"。** 把 `[0x862c]`（检测器槽）包一层，打印
`(count, buf, 返回地址, eafc, eb00)`，实测：

| 时间 | 每块字节数 `n` | 说明 |
|---|---|---|
| 14.9–19.4 s | **6** | 相位 3，`V90TrainingDecoder` 读我们的 TRN1d：48 样本 / 48 bit = **1 bit/样本** |
| 19.9–24.4 s | 6 | 相位 4 头（TRN2d 还没上线） |
| **24.5–25.9 s** | **24** | 相位 4 数据模式解码器接管：48 样本解 24 字节 = **4 bit/样本 = 24 bit/帧** |

每秒 **167 次**，也就是说 `V34RxData`/检测器**一直在被喂**。旧的"0 次"来自
`agr_scr_shim` 的 `count <= 16` 过滤（数据模式每块 24 字节，直接被滤掉，见坑 27）。

**推翻 2："`[0x13a7c]` 24 s 起是 `V90DataModeDecoder`、没人调"。** 邮箱链（`AGRSM_BOX_TRACE`）
在相位 4 每秒 167 次出现 `0x3b/0x3d`（`PCMClientDataPumpRcv` 交块）、`0x28`（`V90DataModeDecoder`
的 MSE 上报）、`0x49/0x4a/0x4b`（切点/解映射/整形输出）、`5`（交给 `V34RxData` 的字节数）。
反编译的调用链因此全部成立：

```
PCMClientDataPumpRcv  while (PCMClientDemodulator(0x30))     被调 167/s
    -> [0x897c] V90Decoder(0x30,...)                         被调 167/s
    -> [0x13a7c] V90DataModeDecoder(0x30,...)                被调 167/s
    -> V34RxData(24, buf) -> [0x862c] V34RxDataMPDetector    被调 167/s
```

**推翻 3："对端能解出 CPd（`V92_NO_CP` 实验）"。** 那轮 `60096211` 来自
`V92ClientPhase4TxSendTRN2uState` 里**速率掩码为空**（`uVar2 == 0 || iVar6 < 1`），
和"听没听到 CPd"无关；本轮正常发 CPd 也报了同一个码（25.88 s）。

**根因（本轮新）**：对端在相位 4 一开局就把解码器换成 `V90DataModeDecoder`，并按**它自己 CPt
的星座**切 24 bit/帧；而我们的 TRN2d/SUVd/CPd/Ed 是 `Source::next_level` 里的
`level(self.uinfo, self.sign)` —— **两点 ±U_INFO，1 bit/样本**。两边速率不同，切出来必然是垃圾，
17 个 1 的帧同步当然永远不出现（`acq=0` 一整段）。

三条独立印证：

1. **规范**：V.92 8.8.5「SUVd … transmitted using the **corresponding TRN2d modulation**」；
   8.8.6 → V.90 8.6.5「TRN2d is generated by applying scrambled binary ones to **the encoder of
   5.4**. The constellation set to be used is **passed in CPt**」，速率 `(drn+8)·8000/6`。
   8.8.3（CPd）、8.6.2（Ed）同。
2. **对端的切点宽度**：24 bit/帧 = `training_bits(drn=16)` = `16+8`，而 `drn 16` 正是它 CPt 里
   的 drn（我们同轮日志 `V92TRACE: CPt drn 16`）。相位 3 它是 1 bit/样本（对应 TRN1d 的两点），
   相位 4 变 4 bit/样本 —— 它自己的行为就把"该用哪个星座"说清楚了。
3. **我们的线上信号**：`caller.raw` 在 TRN2d/SUVd 段（21–24 s）幅度直方图只有 ±3772 两个簇。

**为什么一直没发现**：我们唯一的"下游对不对"的离线证据是拿 `agr-rx.raw` 喂**我们自己的两点
读法**，那等于用同一个错误假设验证自己（坑 29）。而对端的 `acq/n` 我们读到了，却把它解释成
"没被喂"，而不是"喂了但内容不对"。

**修法**（下一轮要做）：`v92::digital::Source` 的 Phase 4 出口改成 V.90 5.4 编码器
（`v90::encoder::Encoder`）：`Mapping::from_cp(cpt)` 已经是 TRN2d 的全部参数（星座来自 CPt、
`K = D − S`、`S` 来自 CPt 的 Sr、`ld/a1..b2` 也来自 CPt），RR 那一档用
`Mapping::for_renegotiation(cpt, data_mode)`。`Out::Trn2d` 每帧推 D 个"扰码后的 1"，
`Out::Suvd/Cpd/Ed` 推各自位流（扰码后），差分编码器沿用上一个序列的最后一个符号位
（GEN-8/GEN-9）；出来的 6 个 ucode/帧按 `Out::B1d` 的做法出电平。

**修法与结果（已做，`f76e9c0`）**。`v92::digital::Source` 的 Phase 4 出口换成了 V.90 5.4 的
`Encoder`，映射来自 `Mapping::from_cp(CPt)`（RR 那一档是 `for_renegotiation`）；`Out::Trn2d` 每帧
推 D 个扰码后的 1，`Out::Suvd/Cpd/Ed` 推各自位流，扰码器与差分编码器跨序列连续；序列按整数据帧
（D 比特）填充，Ed 是 2 个数据帧。三个读回测试也换成了**对端的读法**（CPt 星座切片 + 5.4 差分
解码 + 解扰 + 序列查找），另加一条 `phase_4_carries_cpt_rate_and_not_two_points` 钉住速率。

真机（`005855-run`）：对端 `acq` 从 25.3 s 起为 1、`SUVd detected`、`CPu sent`、`SUVu' sent a001`、
`CPd detected 1e 6f12` 全部出现（这些之前一次都没有）；异常从 `0x96212/0x96211` 变成
**`0x962140`**（27.63 s），是更靠后的一档：它在 `FUN_0006d550` 里给了我们 1 s 去回 **SUVd'**
（Table 31 bit 33 = 已收到 CPu），而我们回不出来 —— **我们没解出它的 CPu**（`V.92: CPu drn` 一次没有，
而 `V.92: SUVu, ack true` 是一串，说明上游通路基本是通的）。见 §0.2/§0.3。

### 3.3 Phase 4 之后（本轮已接线，未验证）

`v92::digital` 现在有一条完整的 Phase 4 路径，全部有单测，但**只有第一段在台架见过**：

| 段 | 代码 | 依据 | 台架 |
|---|---|---|---|
| TRN2d ≥ 2040T（本端发 4800T） | `Out::Trn2d` + `trn2d_symbols` | 9.6.1.1.1 | 见过 |
| SUVd 反复 | `Out::Suvd` + `Exchange::next_sequence` | 9.6.1.1.1-2 | 见过 |
| 读 SUVu/CPu | `Trn2uReader`(4 点) + `UpFinder` | 8.7.2-8.7.6 | 未见到 |
| CPd | `Cpd::from(&Parameters)`，参数由 `design`/`choose` 从第二段 TRN1u 估出 | 9.6.1.1.2，Table 30 | 未发出 |
| Ed | `Out::Ed`（一帧零） | 8.8.2 | 未发出 |
| B1d | `v90::encoder::Encoder`，`Mapping::from_cp(CPu)` | 9.6.1.1.5 | 未发出 |
| B1u 收 | `v92::Decoder::new(&本端 CPd 的参数)` | 6.4 | 未接出 |

两个自检工具（都在，别重写）：`digital::tests::the_phase_4_sequences_read_back_as_themselves`
（把本端发的 SUVd 用对端同样的 2 点读法读回来，抓扰码器/差分/填充错位）和
`epsilon::*`（ε 的整条闭环）。

上游速率现在是**最低一档 drn 1 = 24 kbit/s**（`design_upstream` 里的 `SLOWEST`）：选择它靠的
噪声只有两点流的拟合残差可读，而它在 4 kHz 的零点上被抬高到 13 dB —— 台架上"没有一档能承载"
就是这个原因，所以先用最慢一档把链路打通，再往上调。

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

## 4. 三十二个坑（都真踩过，别重犯）

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

14. **一段信号只发一次，别循环**（DIL、TRN2d）：`Source` 里凡是"一次"的信号，循环会让对端的
    时间表整体漂移（V.90 8.4.1）。DIL 发完就静音等 CPt，TRN2d 发够 2040T 等 SUVd。
15. **所有信号切换都对齐 6 符号帧**（V.90 8.4.1）：`Source::at_frame_boundary()`。对端靠数
    符号定位下一段（尤其 R̄i 的 24 个符号 → TRN2d 扰码器复位点），差 1 个符号就是全盘噪声。
16. **`rem_euclid(1.0)` 会吃掉时步的符号**：`-0.5` 变成 `+0.5`，"早半符号"就成了"晚半符号"，
    换向检测从此只认对一半。时步要按**形状周期**归一（`RU_PERIOD`），不是 1。
17. **带限移位的负频 bin**：`reread`（6 点 DFT 做分数符号移位）必须把 k>N/2 当**负频**处理
    （`k - N`），否则每次移位都多转整数圈——整符号时看不出来，分数符号时全错。
18. **SUVd/CPd 的读回自检要用对端的多项式**（`Mode::Call`）：用解自己上行的那只 reader
    （`Mode::Answer`）去读本端下游，解出来是噪声，而这是"我们发的对不对"唯一的离线证据。
19. **4 点调制只认 L_U 单位**（`Trn2uSize::group_for`）：`level·√points` 与 1、3 比。均衡器输出
    已经把 TRN1u 的 ±L_U 归一成 ±1，所以 TRN2u 的 ±L_U/√5 正好是 0.447/1.342，别再乘一次 L_U。
20. **交付一条 Phase 4 序列时不许动正在播的队列**：`Source::send` 只写 `last` + `pending`，
    填充交给 `start`。曾经在 `send` 里 `bits.clear()`，于是下一个符号就"序列结束"，交换层
    每符号重发一次，每条序列只播得出**第 0 位**——而 SUVd/CPd 的第 0 位都是帧同步的 `1`，
    线上就成了一段扰码全 1（`caller.raw` 里 22–24 s 的密度正好是 1.000）。离线判据：正常时
    每 0.5 s 应看到 `4000/序列长度` 个帧同步（SUVd 74 个，CPd 约 1 个）。
21. **读 capture 要按序列长度换算同步数**：CPd 是 3732 bit，0.2 s 窗口里最多 1 个帧同步，
    "syncs = 0/1" 是**对的**，不是读不出来。先算 `8000·窗口/序列长度` 再判。
22. **改 `datapump` 后必须确认真的重编了**：`cargo build --release -p modem --bin v90-bridge`
    有时只打印 `Compiling modem`（datapump 没动），台架跑的仍是旧码。稳妥做法：
    `touch crates/datapump/src/v92/digital.rs` 再 build，或看 run 的时间线是否按预期变了。
23. **`gGlobalPtr` 前 ~3 s 不指向引擎状态**：探针/钩子读写它之前必须先验
    `[0x8628]` 是不是 `DescramblerX18X23`/`X5X23`（引擎起来后才有的签名）。
    **不验就写 `[0x8610]`/`[0x8628]` 会把引擎写坏**（踩过：进程在 0 s 就死）。
24. **台架偶发 `FileNotFoundError: /dev/pts/N`**：脚本从 `agrsmodemd.log` 里读 PTY 路径，
    若进程启动即崩（或 devpts 抖动）就取不到。先看 `agrsmodemd.log` 尾部有没有
    引擎日志；崩了多半是自己的钩子写坏了内存。重跑一次通常就好，别急着改测试台。
25. **`[0x8614]`（shift 寄存器）不是"被喂了"的证据**：相位 4 里它被共享 scratch 改写，
    值照样在变，但检测器一次也没被调用。判"有没有被喂"要看 `AGRSM_SCR_TRACE`
    （解扰器调用次数）和 `acq/n`。
26. **`scr=x18x23` 之外的 "other" 有两种含义**：装的是别的解扰器，**或者是我们自己的钩子**
    （`agr_scr_shim`）。加钩子后别再按 "other" 判断装错了。

27. **`agr_scr_shim` 的 `count <= 16` 过滤掉了数据模式**：相位 3 每块 6 字节，相位 4 每块
    **24 字节**。按 16 过滤后，`AGRSM_SCR_TRACE` 在相位 4 一行都没有，于是"解扰器在相位 4 没被调"
    成了一个假结论，并顺着它推出"`V34RxData` 没被调、检测器没被喂"。现在上限是
    `AGR_SCR_MAX_LOG = 64`。**判"有没有被调"要看 `AGRSM_DET_TRACE`（检测器槽的实测），不要看解扰器。**
28. **对端每块解几个字节 = 它在用哪个星座，这是本轮最好用的一个探针**：48 个样本，
    6 字节 = 1 bit/样本（TRN1d 的两点），24 字节 = 4 bit/样本 = 24 bit/帧（CPt 的 `drn 16`）。
    对端换星座的时刻（24.5 s）和我们 TRN2d 上线的时刻对得上。以后凡是"它到底按什么参数在收"，
    先看这个数，别猜。
29. **拿 `agr-rx.raw` 喂我们自己的接收机是循环论证**：`v92_capture --sync` 能在 `agr-rx.raw`
    里读出 SUVd/CPd，只证明"我们发的波形符合我们自己的读法"。对端的读法不同（星座不同），
    所以这句话不能当作"对端收到了"的证据。要证明对端收到了，只能用对端的钩子
    （`AGRSM_DET_TRACE` 的 `n/acq`、`AGRSM_EVT_TRACE` 的 `evt 9/10`）。
30. **同一轮里的异常码不要混**：`0x96211` = `V92ClientPhase4TxSendTRN2uState` 里推不出可用下游速率
    （掩码为空或 `iVar6 < 1`）；`0x96212` = 等 SUVd 超时；`0x96221` = 后段。本轮（00:48）报的是
    **0x96211**（25.88 s），上一轮是 0x96212（34.76 s）——**台架每轮的 CPt/SNR 都会让结果落到不同的
    分支**，看判据前先读当轮 `AGRSM_ENGINE_TRACE` 的最后 20 行。

31. **Phase 4 序列的填充单位是 D（每数据帧的比特数），不是 6**：Table 30/31 的 "next multiple of
    6 symbols" 是六个符号 = **一个数据帧** = D 比特，而 D 随 CPt 变（drn 16 → 24）。按 6 填充的话，
    最后一个数据帧只有一部分比特，它之后的每一个数据帧都会错位 4 比特——而序列的 CRC 依然是对的，
    所以这个错只会在对端切出来是噪声时才暴露。`Source::padded` 现在按 `frame_bits()` 补。
32. **对端的 `eafc=0` 不等于"它没解扰"**：相位 4 里 `AGRSM_PHASE4_PROBE` 的 `summary` 一直
    显示 `eafc=0`（`V90DataModeDecoder` 的那条分支不调 `DescramblerX18X23`），可它照样
    `SUVd detected` / `CPd detected`。解扰在那条路上是由 Viterbi / 解映射那一侧做的，
    **别把 `eafc` 当"它有没有解扰"的判据**；判据只能是它有没有报 `… detected`、`acq`、
    以及 `AGRSM_EVT_TRACE` 的 `evt 9`。

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
# 对端内部观测（四路，可任意组合；都不影响时序）
export AGRSM_PHASE4_PROBE=/tmp/p4.txt      # 检测器/接收机/解码器槽，一行一个变化
export AGRSM_EVT_TRACE=/tmp/evt.txt        # "收到序列"事件（含内容与长度）
export AGRSM_SCR_TRACE=/tmp/scr.txt        # 解扰器调用：进/出的字节（判"有没有被喂"）
export AGRSM_BOX_TRACE=/tmp/box.txt        # 引擎内部邮箱 0x70 的 (code,arg,arg)
export AGRSM_DET_TRACE=/tmp/det.txt        # **检测器槽**：每块 (count, 字节, 返回地址, eafc, eb00, eb10/14)
export AGRSM_ENGINE_TRACE=/tmp/eng.txt     # 引擎的文本状态日志（SUVu sent / exception）
python3 interop/agrsm/run_agr_v90.py --ms-args "V92" --pre "AT+PIG=0" \
        --connect-timeout 40 --hold 2 --verbosity 5 --capture
```

探针字段速查（`AGRSM_PHASE4_PROBE`）：`acq`(0x863c 已捕获) `n`(0x8648 已收字节)
`len`(0x864c 期望长度) `pos`(0x8640 比特位置) `shift`(0x8614) `caps`(0x11718 能力字)
`cfg`(0xba27/30/34/38) `dmod`(0xba2a/b 数据模式) `det`(0x862c 装的是哪个检测器)
`dec34`(0x11a50 V.34 解码器槽) `dec90`(0x13a7c V.90 解码器槽)，
外加每 0.5 s 一行 `summary eafc= eb00= rate= slicer= det= n= acq= cp= suv=`
（`eafc` = 对端数据模式解码器有没有开解扰；`eb00` = 它还要吞掉多少样本才收第一块数据；
`rate` = 0xeb10/0xeb14/0xeb18；`det` = 检测器调用次数/本区间）。

**`AGRSM_DET_TRACE` 读法（本轮新增，最重要的一路）**：一行是 `V34RxData` 的一次调用，
`n` 就是它交给检测器的字节数。

| `n` | 含义 |
|---|---|
| 6 | `V90TrainingDecoder`：48 样本解 6 字节 = **1 bit/样本**（两点信号，TRN1d 正确时） |
| 24 | `V90DataModeDecoder`：48 样本解 24 字节 = **4 bit/样本 = 24 bit/帧**（CPt 的 `drn+8`） |

`ra` 是返回地址（原始值，减掉装载基址后就是调用者）。`eafc=0` 表示对端**没开解扰**
（`V90DataModeDecoderSetRateRenegotiationMode` 也是这个值），这时它交给检测器的是**扰码后**的字节。

**邮箱链（`AGRSM_BOX_TRACE`）速查**：`59/61`(0x3b/0x3d `PCMClientDataPumpRcv` 交块)
`40`(0x28 数据模式解码器的 MSE) `73/74/75`(0x49/0x4a/0x4b 切点/解映射/整形)
`5`(交给 `V34RxData` 的字节数) `60`(0x3c)/`4`(解调器回调上报)。相位 4 里这些都在 167/s 就是
"解码链在跑"，缺的只可能是内容。

**基线判据**（每轮先确认这三条，否则先查基线）：
`SUVu sent` 一直 `2001`（ack=0）；`V34 exception 60096212` 在 ~34.8 s；
`agr-rx.raw` 里 SUVd 每 0.5 s 有 67–74 个帧同步、CPd 每 0.5 s 约 1 个。

- **`BRIDGE_MOD=V92` 必须显式给**：Agere 脚本没有 `--bridge-mod`，默认把服务端起成
  `+MS=V90`（binmodem 日志里能看到 `AT+MS=V90`），那样永远到不了 V.92。
  §6 里"Agere 不发 Ru"的旧结论就是这个假象。
- 每轮台架约 2 分钟；改完 `datapump` 必须 `cargo build --release -p modem --bin v90-bridge`。
- 采集方向别搞反：`caller.raw` 是**我们自己的 TX**（bridge 写的是 `from_binmodem`），
  对端音频是 `agr-tx.raw`，而 `agr-rx.raw` 是**对端收到的**（= 我们的下游，经 G.711）。
  `agr-rx.raw` 是可信的：用 `v92_capture --sync` 在里面能干净地读出我们的 SUVd/CPd
  （每 0.5 s 67–74 个帧同步）——这是"对端到底听没听到"的第一手证据。但对端那份 raw **不可全信**：用它跑接收机链连 TRN1u 都拟合
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
