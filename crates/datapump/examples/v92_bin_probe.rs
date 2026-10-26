//! Temp instrument: where does an ideal-case symbol decision stop being exact?
use datapump::v90::ucode::{self, Law};
use datapump::v92::decoder::Decoder;
use datapump::v92::upstream::UpEncoder;
use datapump::v92::{CONSTELLATION_FRAME, Filters, LevelSet, Parameters, Trellis, UP_INTERVALS, up_bits};

fn parameters(drn: u8, levelset: LevelSet) -> Parameters {
    let bits = up_bits(drn);
    let each = bits / UP_INTERVALS as u32;
    let spare = bits % UP_INTERVALS as u32;
    let mut moduli = [0u8; UP_INTERVALS];
    let mut levels = 1usize;
    for i in 0..UP_INTERVALS {
        let width = each + u32::from((i as u32) < spare);
        let modulus = 1u8 << width;
        moduli[i] = modulus;
        let need = if i % 4 == 3 { usize::from(modulus) } else { usize::from(modulus).div_ceil(2) };
        levels = levels.max(need);
    }
    let set = levelset.points(levels).expect("a level set");
    let gain = levelset.gain(&set);
    Parameters {
        drn,
        trellis: Trellis::Sixteen,
        extend_e2u: false,
        gain,
        moduli,
        filters: Filters { z2: vec![1.0], ..Filters::default() },
        sets: vec![set],
        indices: [0; CONSTELLATION_FRAME],
        levelset,
    }
}

fn quantise(law: Law, lsb: f64) -> i32 {
    let full = lsb.clamp(-32768.0, 32767.0).round() as i32;
    let (code, negative) = ucode::nearest(law, full.abs().min(32767));
    let level = ucode::linear(law, code).abs();
    if negative || full < 0 { -level } else { level }
}

fn main() {
    for drn in [1u8, 2, 3, 9, 13, 14, 15, 17, 19] {
        let p = parameters(drn, LevelSet::Code);
        let gain = p.gain;
        let ideal_lu = 1.0 / gain; // the far end at the set's own rms: k = 1
        let mut enc = UpEncoder::new(&p).expect("feasible");
        let tx: Vec<f64> = enc.b1u().iter().map(|s| s.out).collect();
        let mut line = String::new();
        for lu in [900.0f64, 2100.0, ideal_lu] {
            let cw: Vec<i32> = tx.iter().map(|&v| quantise(Law::Mu, v * lu)).collect();
            let mut dec = Decoder::new(&p);
            let mut ones = 0;
            let mut bits = 0;
            for &c in cw.iter() {
                dec.feed(f64::from(c) / lu);
                for b in dec.take_bits() {
                    bits += 1;
                    ones += usize::from(b);
                }
            }
            // symbol-level: nearest design point to the level the decoder sees
            let conj = datapump::v92::precoder::Constellation::new(&p.sets[0]);
            let mut wrong = 0;
            for &c in cw.iter() {
                let lvl = f64::from(c) / lu / gain;
                let mut best = (f64::INFINITY, 0i32);
                for e in conj.lowest()..=conj.highest() {
                    let d = (lvl - conj.level(e).unwrap()).abs();
                    if d < best.0 { best = (d, e); }
                }
                if best.0 > 0.0 { wrong += 1; }
            }
            line.push_str(&format!(" lu={lu:.0} b1u={ones}/{bits}={:.1}% offlevel={wrong} ", 100.0 * ones as f64 / bits as f64));
        }
        println!("drn {drn} lc {} k(lu=1/G)=1 ideal_lu={ideal_lu:.1} G={gain:.6} :{line}", p.sets[0].len());
    }
}
