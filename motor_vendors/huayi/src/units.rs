//! Unit conversions: unified SI ↔ Huayi wire units, plus the compressed-MIT
//! bit packing.
//!
//! Huayi native units (manual §2.2):
//!
//! - position / velocity / acceleration: `Float32` in Rev / Rev/s / Rev/s²
//!   (Rev = output-shaft revolution, user-confirmed),
//! - torque: `Int16` in ‰ of the 6076h peak torque (`Float32`, Nm),
//! - temperatures (2204h): `Int16` in 0.1 °C.
//!
//! The unified external interface is SI (rad, rad/s, Nm, °C), so every
//! crossing goes through this module — never inline a conversion constant
//! in motor code.

use motor_core::error::{MotorError, Result};

/// Radians per output-shaft revolution (2π).
pub const TWO_PI: f32 = std::f32::consts::TAU;

// ---------------------------------------------------------------------------
// position / velocity / acceleration: rad ↔ Rev (output shaft)
// ---------------------------------------------------------------------------

/// rad → Rev.
pub fn rad_to_rev(rad: f32) -> f32 {
    rad / TWO_PI
}

/// Rev → rad.
pub fn rev_to_rad(rev: f32) -> f32 {
    rev * TWO_PI
}

/// rad/s → Rev/s.
pub fn rad_s_to_rev_s(rad_s: f32) -> f32 {
    rad_s / TWO_PI
}

/// Rev/s → rad/s.
pub fn rev_s_to_rad_s(rev_s: f32) -> f32 {
    rev_s * TWO_PI
}

/// rad/s² → Rev/s².
pub fn rad_s2_to_rev_s2(rad_s2: f32) -> f32 {
    rad_s2 / TWO_PI
}

/// Rev/s² → rad/s².
pub fn rev_s2_to_rad_s2(rev_s2: f32) -> f32 {
    rev_s2 * TWO_PI
}

// ---------------------------------------------------------------------------
// torque: Nm ↔ ‰ of the 6076h peak torque
// ---------------------------------------------------------------------------

/// Nm → Int16 permille of `peak_nm`, clamped to ±1000 (±peak).
pub fn torque_nm_to_permille(nm: f32, peak_nm: f32) -> Result<i16> {
    if !(nm.is_finite() && peak_nm.is_finite() && peak_nm > 0.0) {
        return Err(MotorError::InvalidArgument(format!(
            "invalid torque {nm} Nm or peak torque {peak_nm} Nm"
        )));
    }
    let permille = (nm / peak_nm * 1000.0).round();
    let clamped = permille.clamp(-1000.0, 1000.0);
    Ok(clamped as i16)
}

/// Int16 permille of `peak_nm` → Nm.
pub fn permille_to_torque_nm(permille: i16, peak_nm: f32) -> f32 {
    f32::from(permille) / 1000.0 * peak_nm
}

// ---------------------------------------------------------------------------
// temperature: 2204h Int16 0.1 °C → °C
// ---------------------------------------------------------------------------

/// 0.1 °C raw → °C.
pub fn raw_temp_to_celsius(raw: i16) -> f32 {
    f32::from(raw) / 10.0
}

// ---------------------------------------------------------------------------
// MIT gains: unified Nm/rad → Huayi Nm/Rev and the dimensionless slow path
// ---------------------------------------------------------------------------

/// Unified `Nm/rad` → Huayi `Nm/Rev` (both MIT paths quantify gains per
/// output revolution).
pub fn gain_nm_rad_to_nm_rev(k_nm_per_rad: f32) -> f32 {
    k_nm_per_rad * TWO_PI
}

/// Huayi `Nm/Rev` → unified `Nm/rad`.
pub fn gain_nm_rev_to_nm_rad(k_nm_per_rev: f32) -> f32 {
    k_nm_per_rev / TWO_PI
}

/// Slow-path (2003h) Kp/Kd: the u16 entries are dimensionless counts where
/// `count × scale = Nm/Rev`, so `count = k_SI · 2π / scale`, clamped to u16.
pub fn gain_to_dimensionless(k_nm_per_rad: f32, scale: f32) -> Result<u16> {
    if !(k_nm_per_rad.is_finite() && scale.is_finite() && scale > 0.0) {
        return Err(MotorError::InvalidArgument(format!(
            "invalid gain {k_nm_per_rad} or MIT scale factor {scale}"
        )));
    }
    let raw = (gain_nm_rad_to_nm_rev(k_nm_per_rad) / scale).round();
    Ok(raw.clamp(0.0, f32::from(u16::MAX)) as u16)
}

/// Inverse of [`gain_to_dimensionless`]: `count × scale / 2π` in Nm/rad.
pub fn dimensionless_to_gain(raw: u16, scale: f32) -> f32 {
    gain_nm_rev_to_nm_rad(f32::from(raw) * scale)
}

// ---------------------------------------------------------------------------
// compressed MIT (2004h): field ranges and 64-bit packing
// ---------------------------------------------------------------------------

/// Per-field min/max ranges of the compressed MIT word (2004h_4.._D,
/// `Float32`, writable). All values are in Huayi wire units: Rev, Rev/s,
/// Nm/Rev, Nm·s/Rev, Nm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MitRanges {
    /// Pdes range (Rev), default ±0.5.
    pub pos: (f32, f32),
    /// Vdes range (Rev/s), default ±10.
    pub vel: (f32, f32),
    /// Kp range (Nm/Rev), default 0..100.
    pub kp: (f32, f32),
    /// Kd range (Nm·s/Rev), default 0..20.
    pub kd: (f32, f32),
    /// Tff range (Nm), default ±10.
    pub tff: (f32, f32),
}

/// Manual-default 2004h range registers (§5.2).
pub const DEFAULT_MIT_RANGES: MitRanges = MitRanges {
    pos: (-0.5, 0.5),
    vel: (-10.0, 10.0),
    kp: (0.0, 100.0),
    kd: (0.0, 20.0),
    tff: (-10.0, 10.0),
};

/// Sub-indices of the 2004h range registers, in field order.
pub const MIT_RANGE_SUBS: [u8; 10] = [0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D];

/// Compressed-MIT field bit widths: Pdes 16, Vdes 12, Kp 12, Kd 12, Tff 12
/// (total 64).
pub const MIT_FIELD_BITS: [u32; 5] = [16, 12, 12, 12, 12];

impl MitRanges {
    /// Ordered `(min, max)` pairs matching [`MIT_RANGE_SUBS`]:
    /// pos, vel, kp, kd, tff.
    pub fn pairs(&self) -> [(f32, f32); 5] {
        [self.pos, self.vel, self.kp, self.kd, self.tff]
    }
}

/// Normalize `value` into an unsigned field of `bits` width: the value is
/// clamped to `[min, max]` and linearly mapped onto `0..=2^bits - 1`.
/// Returns the raw field as u64 (already at bit 0).
pub fn pack_field(value: f32, min: f32, max: f32, bits: u32) -> Result<u64> {
    if !value.is_finite() || !min.is_finite() || !max.is_finite() || max <= min {
        return Err(MotorError::InvalidArgument(format!(
            "invalid MIT field value {value} for range [{min}, {max}]"
        )));
    }
    let max_raw = (1u64 << bits) - 1;
    let clamped = value.clamp(min, max);
    let ratio = (clamped - min) / (max - min);
    Ok((ratio * max_raw as f32).round() as u64)
}

/// Pack a compressed MIT control word from Huayi-unit values:
///
/// - `pos_rev` — Pdes (Rev),
/// - `vel_rev_s` — Vdes (Rev/s),
/// - `kp_nm_rev` / `kd_nm_rev` — gains in Nm/Rev, Nm·s/Rev,
/// - `tff_nm` — feedforward torque (Nm).
///
/// Layout (manual §5.2): `[63:48] Pdes | [47:36] Vdes | [35:24] Kp |
/// [23:12] Kd | [11:0] Tff`. The returned u64 splits into
/// `2003h`-style halves for the 2004h_2 (bits 31:0) / 2004h_3 (bits 63:32)
/// objects — note the Kp field straddles the 32-bit boundary, so always
/// split via `(word & 0xFFFF_FFFF)` / `(word >> 32)`, never pack the halves
/// independently.
pub fn pack_mit_word(
    pos_rev: f32,
    vel_rev_s: f32,
    kp_nm_rev: f32,
    kd_nm_rev: f32,
    tff_nm: f32,
    ranges: &MitRanges,
) -> Result<u64> {
    let p = pack_field(pos_rev, ranges.pos.0, ranges.pos.1, MIT_FIELD_BITS[0])?;
    let v = pack_field(vel_rev_s, ranges.vel.0, ranges.vel.1, MIT_FIELD_BITS[1])?;
    let kp = pack_field(kp_nm_rev, ranges.kp.0, ranges.kp.1, MIT_FIELD_BITS[2])?;
    let kd = pack_field(kd_nm_rev, ranges.kd.0, ranges.kd.1, MIT_FIELD_BITS[3])?;
    let t = pack_field(tff_nm, ranges.tff.0, ranges.tff.1, MIT_FIELD_BITS[4])?;
    Ok((p << 48) | (v << 36) | (kp << 24) | (kd << 12) | t)
}

/// Split the 64-bit MIT word into the (low, high) u32 halves written to
/// 2004h_2 (bits 31:0) and 2004h_3 (bits 63:32).
pub fn split_mit_word(word: u64) -> (u32, u32) {
    ((word & 0xFFFF_FFFF) as u32, (word >> 32) as u32)
}

/// Decode the raw fields back out of a packed word (tests / diagnostics).
pub fn unpack_mit_word(word: u64) -> (u64, u64, u64, u64, u64) {
    let mask = |shift: u32, bits: u32| -> u64 { (word >> shift) & ((1 << bits) - 1) };
    (
        mask(48, 16),
        mask(36, 12),
        mask(24, 12),
        mask(12, 12),
        mask(0, 12),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-4, "{a} != {b}");
    }

    #[test]
    fn rad_rev_roundtrip() {
        approx(rev_to_rad(1.0), TWO_PI);
        approx(rad_to_rev(TWO_PI), 1.0);
        approx(rad_s_to_rev_s(TWO_PI), 1.0);
        approx(rev_s_to_rad_s(1.0), TWO_PI);
        approx(rad_s2_to_rev_s2(TWO_PI), 1.0);
        approx(rev_s2_to_rad_s2(1.0), TWO_PI);
    }

    #[test]
    fn torque_permille() {
        let p = torque_nm_to_permille(5.0, 10.0).expect("permille");
        assert_eq!(p, 500);
        // negative torque
        assert_eq!(torque_nm_to_permille(-2.5, 10.0).expect("neg"), -250);
        // clamped at ±peak
        assert_eq!(torque_nm_to_permille(50.0, 10.0).expect("clamp"), 1000);
        assert_eq!(torque_nm_to_permille(-50.0, 10.0).expect("clamp"), -1000);
        // inverse
        approx(permille_to_torque_nm(500, 10.0), 5.0);
        // invalid inputs
        assert!(torque_nm_to_permille(f32::NAN, 10.0).is_err());
        assert!(torque_nm_to_permille(1.0, 0.0).is_err());
    }

    #[test]
    fn temperature_conversion() {
        approx(raw_temp_to_celsius(255), 25.5);
        approx(raw_temp_to_celsius(-100), -10.0);
    }

    #[test]
    fn mit_kp_kd_scale() {
        // k_SI × 2π / scale, clamped to u16
        let scale = 0.01;
        let k = 1.0 / TWO_PI; // 1 Nm/Rev expressed in Nm/rad
        assert_eq!(gain_to_dimensionless(k, scale).expect("dim"), 100);
        // inverse round trip
        approx(dimensionless_to_gain(100, scale), 1.0 / TWO_PI);
        // clamp at u16 max
        let big = gain_to_dimensionless(1e9, 0.01).expect("clamp");
        assert_eq!(big, u16::MAX);
        assert!(gain_to_dimensionless(1.0, 0.0).is_err());
        assert!(gain_to_dimensionless(f32::NAN, 1.0).is_err());
    }

    #[test]
    fn pack_field_maps_and_clamps() {
        // midpoint → round(ratio × max_raw): 0.5 × 65535 = 32767.5 → 32768
        assert_eq!(pack_field(0.0, -0.5, 0.5, 16).expect("mid"), 0x8000);
        // min → 0, max → max raw
        assert_eq!(pack_field(-0.5, -0.5, 0.5, 16).expect("min"), 0);
        assert_eq!(pack_field(0.5, -0.5, 0.5, 16).expect("max"), 0xFFFF);
        // clamping beyond range
        assert_eq!(pack_field(9.0, -0.5, 0.5, 16).expect("over"), 0xFFFF);
        assert_eq!(pack_field(-9.0, -0.5, 0.5, 16).expect("under"), 0);
        // 12-bit field
        assert_eq!(pack_field(10.0, -10.0, 10.0, 12).expect("12b"), 0xFFF);
        // invalid range
        assert!(pack_field(0.0, 1.0, 1.0, 12).is_err());
        assert!(pack_field(f32::NAN, 0.0, 1.0, 12).is_err());
    }

    #[test]
    fn mit_word_bit_layout() {
        let ranges = DEFAULT_MIT_RANGES;
        // max out every field
        let word = pack_mit_word(0.5, 10.0, 100.0, 20.0, 10.0, &ranges).expect("word");
        let (p, v, kp, kd, t) = unpack_mit_word(word);
        assert_eq!(p, 0xFFFF);
        assert_eq!(v, 0xFFF);
        assert_eq!(kp, 0xFFF);
        assert_eq!(kd, 0xFFF);
        assert_eq!(t, 0xFFF);

        // zeros
        let word = pack_mit_word(-0.5, -10.0, 0.0, 0.0, -10.0, &ranges).expect("zero");
        assert_eq!(word, 0);

        // midpoint of every field (round-half-away-from-zero)
        let word = pack_mit_word(0.0, 0.0, 50.0, 10.0, 0.0, &ranges).expect("mid");
        let (p, v, kp, kd, t) = unpack_mit_word(word);
        assert_eq!(p, 0x8000);
        assert_eq!(v, 0x800);
        assert_eq!(kp, 0x800);
        assert_eq!(kd, 0x800);
        assert_eq!(t, 0x800);

        // known single-field encodings
        let word = pack_mit_word(0.5, -10.0, 0.0, 0.0, -10.0, &ranges).expect("p");
        assert_eq!(unpack_mit_word(word).0, 0xFFFF);
        let word = pack_mit_word(-0.5, 10.0, 0.0, 0.0, -10.0, &ranges).expect("v");
        assert_eq!(unpack_mit_word(word).1, 0xFFF);
    }

    #[test]
    fn mit_word_split_matches_boundary() {
        // Kp straddles the 32-bit boundary: bits 35:24. Choose kp raw = 0xFFF
        // and everything else minimal so the high half is fully determined.
        let ranges = DEFAULT_MIT_RANGES;
        let word = pack_mit_word(-0.5, -10.0, 100.0, 0.0, -10.0, &ranges).expect("kp only");
        // v = 0, p = 0, so high32 = kp bits 3..0 = 0xF
        let (low, high) = split_mit_word(word);
        assert_eq!(high, 0x0000_000F);
        // low32 bits 31..24 = kp bits 11..4 = 0xFF, kd = t = 0
        assert_eq!(low, 0xFF00_0000);

        // and reassembling the halves recovers the word
        let reassembled = u64::from(high) << 32 | u64::from(low);
        assert_eq!(reassembled, word);
    }

    #[test]
    fn default_ranges_match_manual() {
        assert_eq!(DEFAULT_MIT_RANGES.pos, (-0.5, 0.5));
        assert_eq!(DEFAULT_MIT_RANGES.vel, (-10.0, 10.0));
        assert_eq!(DEFAULT_MIT_RANGES.kp, (0.0, 100.0));
        assert_eq!(DEFAULT_MIT_RANGES.kd, (0.0, 20.0));
        assert_eq!(DEFAULT_MIT_RANGES.tff, (-10.0, 10.0));
        // range sub-index order: pos, vel, kp, kd, tff min/max interleaved
        assert_eq!(MIT_RANGE_SUBS.len(), 10);
        // total field bits = 64
        assert_eq!(MIT_FIELD_BITS.iter().sum::<u32>(), 64);
    }
}
