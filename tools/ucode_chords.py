import sys, array, math, bisect
from collections import defaultdict
def dec(u):
    b=u^0xFF; e=(b>>4)&7; m=b&0xF
    mag=(((m<<3)+0x84)<<e)-0x84
    return -mag if (b&0x80) else mag
LEVELS=sorted((abs(dec(u)),u) for u in range(256)); MAGS=[m for m,_ in LEVELS]
TAB={}
for u in range(256):
    TAB.setdefault((u&0x7F)>>4,[]).append(abs(dec(u)))
STEP={c:min(b-a for a,b in zip(sorted(set(v)),sorted(set(v))[1:])) for c,v in TAB.items()}
def load(p):
    a=array.array('h'); a.frombytes(open(p,'rb').read())
    if sys.byteorder=='big': a.byteswap()
    return a
def scan(xs,lo,hi):
    per=defaultdict(lambda: dict(n=0,s=0.0,rs=[],out=0,codes=set()))
    for k in range(lo,hi):
        x=xs[k]
        if x==0: continue
        a=min(abs(x),32124); i=bisect.bisect_left(MAGS,a)
        cand=[i]+([i-1] if i>0 else [])
        best=min(cand,key=lambda j:abs(MAGS[j]-a)); lvl=LEVELS[best][0]
        c=(LEVELS[best][1]&0x7F)>>4
        e=abs(abs(x)-lvl); d=per[c]
        d['n']+=1; d['s']+=e*e; d['rs'].append(e); d['codes'].add(lvl)
    return per
def table(per,label,gaps):
    n=sum(d['n'] for d in per.values())
    print(f"\n### {label}  n={n}")
    hdr="chord step    n    pop%   rmsLSB rms/half  max/LSB support(hs) codes minGapLSB " + " ".join(f"cross%g={g}" for g in gaps)
    print(hdr)
    tot={g:0.0 for g in gaps}
    for c in sorted(per):
        d=per[c]; st=STEP[c]; half=st/2.0; rms=math.sqrt(d['s']/d['n']); mx=max(d['rs'])
        codes=sorted(d['codes']); g=min([b-a for a,b in zip(codes,codes[1:])] or [0])
        line=f"{c:5d} {st:5d} {d['n']:6d} {100*d['n']/n:5.1f} {rms:7.2f} {rms/half:8.3f} {mx:9.0f} {mx/half:11.2f} {len(codes):5d} {g:9.0f} "
        for gg in gaps:
            hs=max(gg,st)/2.0
            cr=sum(1 for r in d['rs'] if r>hs)/d['n']
            tot[gg]+=cr*d['n']
            line+=f"{100*cr:9.3f} "
        print(line)
    print("weighted crossing rate per rung gap: " + " ".join(f"gap {g} -> {100*tot[g]/n:.4f}% of symbols" for g in gaps))
for p in sys.argv[1:]:
    xs=load(p); print(f"\n=== {p}: {len(xs)} samples")
    table(scan(xs,0,len(xs)),"whole capture",[320,128])
