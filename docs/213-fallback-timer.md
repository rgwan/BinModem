# 213 — **Agere.** The 0.82 s is a **receive** deadline: the far end waits 500 ms + its own round-trip
delay for us to open the V.92 rate renegotiation, and we never answer

Marks: **[quoted]** = the Recommendation's words; **[verified]** = printed by a run on disk, read out
of `agrsm/agrmodemlib.o` (sha256 `87eff539…16a15`) here, or read out of this worktree's code;
**[inferred]** = follows from those, with the inference shown; **[our choice]** = ours.

Base: `fallback-213` off `sdwait-211` @ `368b10e` (= `v92-line` @ `2ed6d28` + docs/211's
`SD_AFTER_JA`). Docs-only: no code change, so the 739/0/11 suite is untouched. **Zero bench calls**
(§5). Run dirs: `/home/hikari/runs-211/{154542-sd108ab170, 154931-sd81ab170, 154703-sd108ab171}`.
Addresses are section-relative in `.text` (starts at 0), as in docs/212.

---

## 0. The answer

The far end is waiting for **us to start the rate renegotiation** — for a receive event from the
digital modem. It stops its own data transmitter, switches its transmitter to a pattern generator,
zeroes a **received-sample** counter and arms a deadline of

> `gGlobalPtr+0x116a0` (round-trip delay estimate, `RTDEd`) **+ `0xfa0` (4000 samples = 500.0 ms)**

on that counter. If the expected receive event has not arrived when the counter reaches the deadline,
it throws **`0x9521a`** and retrains. Our 0.82 s is exactly that: 321.75 ms of RTD plus 500 ms =
**821.75 ms**, against 0.822 / 0.828 / 0.828 s measured [verified §1, §2]. **It is ours to give, and we
give nothing** — our receiver stops looking for upstream control sequences the moment data mode starts
and our transmitter has no §9.8 responder at all [verified §3].

---

## 1. The timer: `.text+0x5e556`–`0x5e56e`, in `V92ClientDataModeRxState` (`0x5dea0`)

The entry that arms it, `.text+0x5e530`–`0x5e56e` [verified, `objdump -dr`]:

```
5e530  mov $0x5e790,%ecx
5e535  mov $0x5e750,%ebx
5e53a  mov %ecx,0x1168c(%eax)      ; per-tick sub-state := 0x5e790
5e540  call PCMClientDemodulatorSetFullDuplexMode
5e545  call V92ClientControlDisableUpdate
5e54b  call V34RxDataReset
5e554  xor  %edx,%edx
5e556  mov  %edx,0x11694(%eax)     ; RECEIVED-SAMPLE COUNTER := 0
5e55c  mov  0x116a0(%eax),%edx     ; RTDEd (round-trip delay estimate), samples
5e562  mov  %ebx,0x11688(%eax)     ; main state := 0x5e750
5e568  add  $0xfa0,%edx            ; + 4000 samples = +500.0 ms
5e56e  mov  %edx,0x11698(%eax)     ; DEADLINE := RTDEd + 500 ms
```

and the state it arms, `.text+0x5e750` [verified]:

```
5e75c  mov  0x11698(%edx),%eax
5e762  cmp  %eax,0x11694(%edx)     ; received samples >= deadline?
5e768  jge  5e780                  ; YES -> THROW
5e76a  cmpb $0x0,0x1174d(%edx)     ; the awaited receive event?
5e771  jne  5e777                  ; YES -> V92ClientPhase4RxWaitRiBarState, carry on
5e773  ret                        ; else keep waiting
5e780  movl $0x9521a,(%esp) ; call V92ClientControlException
```

* `+0x11694` is the **receive** sample counter: `V92ClientControlRcv(n)` does
  `add %edx,0x11694(%eax)` at `0x596c1`; `+0x11690` is the transmit twin, `add` at `0x59875` in
  `V92ClientControlXmt`, which is also what dispatches `+0x1168c` per tick [verified].
* `+0x116a0` is the far end's round-trip delay estimate. The far end **prints it**:
  `Round Trip Delay : 321.7500 ms` / `Round trip delay: 322 ms` at `[12.150]` in every run
  [verified, `runs-211/*/agrsmodemd.log:175-176`]. 321.75 ms = **2574 samples**.
* Deadline therefore `2574 + 4000 = 6574` samples = **821.75 ms**. Measured announcement→throw:
  **0.828 / 0.828 / 0.822 s** [verified]. The 0.822 s run is exact; the two 0.828 s runs carry one
  extra log tick. This also fixes the *unit* of these counters at 8 kHz — independently of docs/212's
  guesswork about the `0x300`/`0x80` margins.
* The sub-state installed at the same time, `0x5e790` [verified], is
  `movb $0,0x116bc(%eax) ; V34TxDataReset ; V34TxDataDisableScrambler ; V34TxDataSetPatternGenerator`
  — i.e. **it has stopped sending data and started a pattern generator** (the Ru/R̄u of §9.8.2.1.1
  [quoted `v92.txt:2994`]). It is transmitting, and it is waiting to *receive*.

Sibling sites, same shape, same run [verified]: `0x5e428/0x5e43a` (also `RTDEd + 0xfa0`, arming
`0x5e940`), `0x5e0e8/0x5e0f5` (`+0x11698 := 0x30` = 48 samples = 6 ms, tail-calling
`V92ClientPhase4RxWaitRiBarState`), `0x5c863/0x5c873` (`+0x1169c := 0x3e80` = 16000 transmit samples
= §9.8.2.1.2's "TRN2u for up to 16008T" [quoted `v92.txt:2997`]).

### 1.1 Where the event comes from, and what `0x9521a` really is here

`+0x1174d` is set by **`V92ClientControlEventHandler`** (`0x5a350`, an event jump table at
`.rodata+0x9a80`) at `.text+0x5a4c5`, for **event 22**, and it stores that event's parameter to
`+0x11788` [verified]. The phase-4 state `V92ClientPhase4RxWaitRiBarState` (`0x5c5b0`) tests the same
flag at `0x5c5d1` and throws the same `0x9521a` at `0x5c790` on deadline expiry [verified]. In other
words the data-mode state `0x5e750` is *"wait for the digital modem's renegotiation-start signal, give
up after 500 ms + RTD"* — the analogue modem's §9.8.2.1.2 wait for the exchange to be opened.

**Correction to docs/212.** `0x9521a` is thrown from **four** sites, not one: `0x5c394` (the CP-level
guard of docs/212), `0x5c590` and `0x5c790` (`V92ClientPhase4RxWaitRiState` `0x5c530` and
`V92ClientPhase4RxWaitRiBarState` `0x5c5b0`, **both on deadline expiry, no level test**), and `0x5e780`
(above, deadline expiry in data mode). The runs here fired a **deadline** site — the interval is
`RTDEd + 500 ms` to the sample, which no level search could produce — so docs/212's "the allocator
cannot build a CP" reading is the wrong site for these runs [inferred from §1, and it is why §1 of
docs/212 and §8 of docs/212 disagreed about which state the throw belongs to].

---

## 2. What the far end expects, in protocol terms

Not "a CP from us" and not "nothing". The V.92 renegotiation is opened by the **digital** modem:

* **`v92.txt:2833`–`2841`** [quoted] — rate renegotiation "can be initiated at any time during data
  mode … shall be initiated only on the boundary of a data frame".
* **`v92.txt:2994`–`2999`** [quoted] — §9.8.2.1.1/9.8.2.1.2, the analogue modem (our far end):
  transmit Ru for 384T and R̄u for 24T, then "condition its receiver to receive an SUVd sequence".
* **`v92.txt:2985`–`2990`** [quoted] — §9.8.1.2.1/9.8.1.2.2, the digital modem (us): "After detecting
  Ru … condition its receiver to detect the Ru-to-R̄u transition … shall transmit signal Rd for 384T
  and then R̄d for 24T", then per `9.8.1.1.2` (`v92.txt:2963`) TRN2d for up to 16008T followed by SUVd.
* The 0.82 s has no V.92 clause of its own: it is 500 ms + RTDEd, the same "plus a round-trip delay"
  idiom the Recommendation uses for every recovery timer (`v92.txt:2571`, `v92.txt:2623`-`2624`,
  `v92.txt:2770`, `v92.txt:2784`, `v92.txt:3127`) [quoted for the idiom, inferred for this instance].
  It is the far end's own grace period for the exchange to open [inferred].

So: **we owe it Rd/R̄d — equivalently the Ri that re-enters the phase-4 CP exchange in this engine's
implementation — then TRN2d and SUVd.** We send none of them; we keep sending data frames.

---

## 3. Our side in that window

### 3.1 Logs

`runs-211/154542-sd108ab170`, the window 22.8 → 33.7 s [verified]: `binmodem-dte.log`'s last line
before the gap is `22.820 note: V.92: B1u confirmed at 576 bits …`, then
`DTE: "\r\nCONNECT 24000\r\n…"`, and the **next** line with a timestamp is
`33.740 Data V.90 V.92 phase 4: B1d … down 24000 / up 53333` (the post-retrain connection). The far
end's `Fallback initiated!` is at `[27.228]` and `V34 exception: 6009521a` at `[28.056]`
(`agrsmodemd.log:253-254`), i.e. inside that gap, and **nothing in between**. The far end also logs
nothing between the two lines.

The silence is not evidence of absence by itself, and here it is *predicted* by the code: in data mode
we emit one line per connection change and nothing per symbol, so a received Ru, TRN2u, CPu or CPus
would not be logged — see the paths that would have to fire and cannot (§3.2).

### 3.2 Code — we have no renegotiation path at all

1. **The receiver stops listening for control sequences the moment data mode starts.**
   `crates/datapump/src/v92/digital.rs:4647` — after the `b1u_search` branch,
   `if self.upstream_data.is_some() { … self.b1u_symbol(symbol); return; }` — so the sequence finder
   (`self.up_finder`, `digital.rs:4684`) is never fed again. Any Ru, TRN2u, CPu or CPus the far end
   sends after B1u is decoded as payload. [verified]
2. **CPus is parsed and dropped.** `digital.rs:5157`-`5159`: `// CPus is a rate renegotiation's short form
   (Table 24); training never sees one.` / `CpFamily::Cpus(_) | CpFamily::Cpt(_) => {}` — the one
   message this layer could recognise exists in our parser and does nothing. [verified code, quoted
   comment; its claim about Table 24 is loose — §9.8 carries full CPu/CPd and CPus is the
   fast-parameter-exchange form, `v92.txt:3060`-region]
3. **The §9.8 sequencer exists and is never wired to a live renegotiation.** `exchange.rs:135` defines
   `Context::{Training, Renegotiation, FastExchange}` and the machine handles all three
   (`exchange.rs:163`–`183`, `exchange.rs:480`); `digital.rs:3721` constructs it exactly once, with
   `Context::Training`; the only `Context::Renegotiation` users are unit tests (`exchange.rs:1029`,
   `:1039`). [verified]
4. **We have no transmitter for the opening.** `digital.rs` sends Ri once, in phase 3, in answer to
   CPt (`digital.rs:1997`); there is no Ri/Rd/TRN2d/SUVd emitter reachable from data mode, and nothing
   reads the far end's Ru/R̄u transition after B1u. [verified]

So: **the defect is ours and it is in the phase-4 data-mode exchange** — not in the `Sd` wait, not in
the CPd `drn`, and not in the far end's allocator.

---

## 4. (C) — is there a downstream-rate bound on our side, and is 53333 inside it?

**No bound exists on our side, and 53333 is well inside the Recommendation's.** [verified]

* The downstream rate is the far end's to choose and it says so in CPu: `drn 20` → `20+20 = 40`,
  `40 × 8000/6 = 53 333` bit/s (`v92.txt:1492`–`1495` [quoted]: "Selected digital modem to analogue
  modem data signalling rate … drn, between 0 and 22 … = (drn + 20) × 8000/6 in CPu"). drn 22 = 56 000
  is the top of the table, so **53333 is two rungs below the maximum** and is not a rate the far end
  should not have chosen. The working call's 45333 is `drn 14`.
* Everything we bound is **upstream**: `rung.rs:446 ceiling_rung`, `rung.rs:475 apply_ceiling`, applied
  at `digital.rs:4204`–`4205` to **Ja's** upstream mask (the rung we grant in CPd, `drn 1` = 24000 in
  every run). docs/101's `Initial Receive 45333/46666` is an acceptance observation about the *working*
  calls, not a cap in code. **We assert no downstream cap, because we have none.**

So (C) is not the shape of this failure: the same throw appears at 45333 (the working call's rate), at
40000 and at 53333, in both directions of change, at three SNRs [verified, parent's §5] — a bound on
one rate cannot explain that; a timer can, and does.

---

## 5. Verdict: **(A)**, with the timer measured — and no bench call

**(A). In those 0.82 s the far end is waiting for us**: it has stopped its data transmitter, started
the Ru pattern, zeroed its receive-sample counter and armed `500 ms + RTDEd` (821.75 ms) for the
digital modem to open the V.92 rate renegotiation (Rd/R̄d and TRN2d+SUVd, `v92.txt:2985`–`2990`,
`v92.txt:2963`; in this engine's implementation, the Ri that re-enters `V92ClientPhase4RxWaitRiBarState`).
We answer with data frames, because our receiver stopped parsing upstream control sequences at B1u
(`digital.rs:4647`) and our `Context::Renegotiation` sequencer has no live caller
(`digital.rs:3721`). On expiry it throws `0x9521a` at `0x5e780` and retrains 6 ms later
(`V34 Phase 2 started`, `agrsmodemd.log:255`).

**No bench call was spent, and that is a deliberate choice, not an omission:**

* The (A)/(B) discriminating measurement needs no call — it is now arithmetic on a quantity the far end
  *prints*: `RTDEd` (321.75 ms) `+ 500 ms` = 821.75 ms against 822/828/828 ms measured, invariant over
  wait, rate, SNR and direction. No bench output would separate these further short of an engine trace.
* The gate the parent asked for at (A) requires a **code change that does not exist yet** — a §9.8
  responder: detect the far end's Ru/R̄u after B1u, send Rd/R̄d, TRN2d for up to 16008T, SUVd repeat,
  and drive the existing `Context::Renegotiation` exchange through a full CPu/CPd re-negotiation while
  keeping the data-frame alignment. That is a build ticket of its own (the Ru detector must live
  outside the payload decoder; the downstream must re-enter data mode at the new rate), not a one-line
  gate. Spending a call on an unbuilt change would spend it on nothing.
* Cheapest next call, once that change exists: one call at `--answer-blocks 170` with 108 ms and one at
  `171` with 0 ms; acceptance = far end `Signal/Noise Ratio ≥ 19.9375` at its check, `Jd detected`,
  `Su`, `CPd` acked, `ATI11 Initial Transmit 24000`, payload `2000/2000 mismatched 0`, **no `9521a`**,
  and the new line — **no `Fallback`/`Fallforward` retrain** inside the call.

**Priority stays Agere link stability: the cheapest way to stop the crash today is to stop provoking
the renegotiation (a rate/`drn` policy that keeps the far end's rate control quiet); the way to make it
correct is to answer it, which is §3.2's missing responder.**

---

**What is the far end waiting for in those 0.82 s, and is it ours to give?** It is waiting for the
digital modem to open the rate renegotiation — for a **receive** event, deadline `RTDEd + 500 ms`
(`+0x11698 := +0x116a0 + 0xfa0` at `.text+0x5e56e`, checked at `0x5e762`, thrown at `0x5e780`) — and it
is ours to give: we answer with data instead, because we have no §9.8 responder at all
(`digital.rs:4647`, `digital.rs:3721`).
