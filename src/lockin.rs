use core::f32::consts::PI;
use core::ops::SubAssign;
use core::cmp::Ord;
extern crate libm;

#[path = "trig/trig.rs"]
mod trig;
use trig::{sin, cos, atan2};

use super::iir;
use stm32h7xx_hal as hal;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use super::SAMPLE_BUFFER_SIZE;
use super::TSTAMP_BUFFER_SIZE;
use super::OUTPUT_BUFFER_SIZE;

/// Slow external reference edge timestamps.
#[derive(Copy, Clone)]
pub struct TimeStamp {
    // Timestamp value.
    pub count: u16,
    // Number of sequences before the current one that the timestamp
    // occurred. A sequence is a set of `SAMPLE_BUFFER_SIZE` ADC samples. E.g., a
    // timestamp from the current sequence has this set to 0, a
    // timestamp from the previous sequence has this set to 1, etc. A
    // value of -1 indicates an invalid timestamp (i.e., one that has
    // not yet been set).
    pub sequences_old: i16,
}

impl TimeStamp {
    /// Increments `sequences_old` if TimeStamp is valid. This is
    /// called at the end of processing a sequence of ADC samples.
    pub fn new_sequence(&mut self) {
        if self.sequences_old != -1 {
            self.sequences_old += 1;
        }
    }

    /// Returns true if TimeStamp is valid.
    pub fn is_valid(&self) -> bool {
        self.sequences_old != -1
    }

    /// Set a new count value for TimeStamp. This also changes
    /// `sequences_old` to 0 to indicate the TimeStamp belongs to the
    /// current processing sequence.
    ///
    /// # Arguments
    ///
    /// * `newval` - New count value.
    pub fn new_count(&mut self, newval: u16) {
        self.count = newval;
        self.sequences_old = 0;
    }
}

macro_rules! prefilt_no_decimate {
    ( $x:expr, $t:expr, $r:expr, $phi:expr, $ffast:expr, $fadc:expr, $fscale:expr, $tstamps_mem:expr, $toggle:expr) => {{
        record_new_tstamps($t, $r, $tstamps_mem);
        let ts_valid_count = tstamps_valid_count($tstamps_mem);

        if ts_valid_count < 2 {
            return ([0.; OUTPUT_BUFFER_SIZE], [0.; OUTPUT_BUFFER_SIZE]);
        }

        let tadc = tadc($ffast, $fadc);
        let thetas = adc_phases($t[0], $tstamps_mem, $phi, $fscale, tadc, $toggle);
        // $toggle.set_high();
        let mut sines: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
        let mut cosines: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
        for i in 0..SAMPLE_BUFFER_SIZE {
            sines[i] = sin(thetas[i]);
            cosines[i] = cos(thetas[i]);
        }
        // $toggle.set_low();

        increment_tstamp_sequence($tstamps_mem);
        demod($x, sines, cosines)
    }};
}

/// Unfiltered in-phase and quadrature signals.
///
/// # Generics
///
/// * `N` - Number of ADC samples in each processing sequence.
/// This must be a power of 2.
/// * `M` - Maximum number of external reference edge timestamps.
/// The highest this should ever be set is SAMPLE_BUFFER_SIZE>>1.
/// * `K` - Number of output samples. This must be a power of 2 and
/// between 1 and `SAMPLE_BUFFER_SIZE` (inclusive).
///
/// # Arguments
///
/// * `x` - ADC samples.
/// * `t` - Counter values indicate the timestamps of the slow external
/// reference clock edges.
/// * `r` - Number of valid timestamps in `t`.
/// * `phi` - Demodulation phase offset specified as a number of counts
/// relative the fast clock period.
/// * `ffast` - Fast clock frequency (Hz). The fast clock increments
/// timestamp counter values used to record the edges of the external
/// reference.
/// * `fadc` - ADC sampling frequency (in Hz).
/// * `fscale` - Scaling factor for the demodulation frequency. For
/// instance, 2 would demodulate with the first harmonic of the reference
/// frequency.
/// * `tstamps_mem` - Last two external reference timestamps (i.e., recorded
/// values of `t`.)
///
/// # Latency (ADC batch size = 16): 5.6us
///
/// * 2.0us to compute ADC phase values
/// * 3.4us to compute sin and cos
pub fn prefilt(
    x: [i16; SAMPLE_BUFFER_SIZE],
    t: [u16; TSTAMP_BUFFER_SIZE],
    r: usize,
    phi: u16,
    ffast: u32,
    fadc: u32,
    fscale: u16,
    tstamps_mem: &mut [TimeStamp; 2],
    toggle: &mut hal::gpio::gpiod::PD0<hal::gpio::Output<hal::gpio::PushPull>>,
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    let (i, q) =
        prefilt_no_decimate!(x, t, r, phi, ffast, fadc, fscale, tstamps_mem, toggle);
    decimate(i, q)
}

/// Filtered in-phase and quadrature signals.
///
/// # Arguments
///
/// See `prefilt`.
/// * `iir` - IIR biquad for in-phase and quadrature components.
/// * `iirstate` - IIR biquad state for in-phase and quadrature
/// components.
///
/// # Latency (ADC batch size = 16, DAC size = 1, filter then decimate): 9.4us
/// # Latency (ADC batch size = 16, DAC size = 1, boxcar then filter): 6.2us
///
/// * 5.6us from `prefilt`
/// * 3.5us from `filter`
pub fn postfilt_iq(
    x: [i16; SAMPLE_BUFFER_SIZE],
    t: [u16; TSTAMP_BUFFER_SIZE],
    r: usize,
    phi: u16,
    ffast: u32,
    fadc: u32,
    fscale: u16,
    iir: [iir::IIR; 2],
    iirstate: &mut [iir::IIRState; 2],
    tstamps_mem: &mut [TimeStamp; 2],
    toggle: &mut hal::gpio::gpiod::PD0<hal::gpio::Output<hal::gpio::PushPull>>,
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    let (i, q) =
        prefilt_no_decimate!(x, t, r, phi, ffast, fadc, fscale, tstamps_mem, toggle);

    // filter then decimate
    let (ifilt, qfilt) = filter(i, q, iir, iirstate);
    decimate(ifilt, qfilt)

    // // average then filter
    // let iavg = avg(i);
    // let qavg = avg(q);
    // let ifilt = iir[0].update(&mut iirstate[0], iavg);
    // let qfilt = iir[1].update(&mut iirstate[1], qavg);
    // // TODO averaging only works for output size of 1.
    // ([ifilt], [qfilt])
}

/// Filtered magnitude and angle signals.
///
/// # Arguments
///
/// See `postfilt_iq`.
///
/// # Latency (ADC batch size = 16, DAC size = 1, filter then decimate): 9.6us
/// # Latency (ADC batch size = 16, DAC size = 1, boxcar then filter): 6.4us
pub fn postfilt_at(
    x: [i16; SAMPLE_BUFFER_SIZE],
    t: [u16; TSTAMP_BUFFER_SIZE],
    r: usize,
    phi: u16,
    ffast: u32,
    fadc: u32,
    fscale: u16,
    iir: [iir::IIR; 2],
    iirstate: &mut [iir::IIRState; 2],
    tstamps_mem: &mut [TimeStamp; 2],
    toggle: &mut hal::gpio::gpiod::PD0<hal::gpio::Output<hal::gpio::PushPull>>,
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    let (i, q) = {
        let (ipre, qpre) = prefilt_no_decimate!(
            x,
            t,
            r,
            phi,
            ffast,
            fadc,
            fscale,
            tstamps_mem,
            toggle
        );
        // filter then decimate
        let (ifilt, qfilt) = filter(ipre, qpre, iir, iirstate);
        decimate(ifilt, qfilt)

        // // average then filter
        // let iavg = avg(ipre);
        // let qavg = avg(qpre);
        // let ifilt = iir[0].update(&mut iirstate[0], iavg);
        // let qfilt = iir[1].update(&mut iirstate[1], qavg);
        // // TODO averaging only works for output size of 1.
        // ([ifilt], [qfilt])
    };

    // 198ns for size of 1.
    iq_to_at_map(i, q)
}

/// ARR (counter overflow value).
///
/// # Arguments
///
/// * `ffast` - Fast clock frequency (Hz). The fast clock increments
/// timestamp counter values used to record the edges of the external
/// reference.
/// * `fadc` - ADC sampling frequency (in Hz).
/// * `n` - Number of ADC samples in each processing block.
pub const fn arr(ffast: u32, fadc: u32, n: u16) -> u16 {
    tadc(ffast, fadc) * n
}

/// Simple average.
///
/// # Arguments
///
/// * `x` - Array of samples to average.
fn avg(x: [f32; SAMPLE_BUFFER_SIZE]) -> f32 {
    let mut total: f32 = 0.;
    for val in x.iter() {
        total += val;
    }
    total
}

/// Count number of valid TimeStamps from `tstamps`.
fn tstamps_valid_count(tstamps: &[TimeStamp; 2]) -> usize {
    let mut valid_count: usize = 0;
    for i in 0..2 {
        if tstamps[i].is_valid() {
            valid_count += 1;
        }
    }
    valid_count
}

/// Add new timestamps to the TimeStamp memory.
///
/// # Arguments
///
/// * `t` - New timestamp values.
/// * `r` - Number of valid timestamps.
/// * `tstamps_mem` - Last 2 recorded timestamps.
fn record_new_tstamps(
    t: [u16; TSTAMP_BUFFER_SIZE],
    r: usize,
    tstamps_mem: &mut [TimeStamp; 2],
) {
    if r > 1 {
        tstamps_mem[1].new_count(t[r - 2]);
        tstamps_mem[0].new_count(t[r - 1]);
    } else if r == 1 {
        tstamps_mem[1].count = tstamps_mem[0].count;
        tstamps_mem[1].sequences_old = tstamps_mem[0].sequences_old;
        tstamps_mem[0].new_count(t[r - 1]);
    }
}

/// ADC period in number of fast clock counts. Assumes `ffast` and
/// `fadc` chosen to yield an integer ratio.
///
/// # Arguments
///
/// * `ffast` - Fast clock frequency.
/// * `fadc` - ADC sampling frequency.
const fn tadc(ffast: u32, fadc: u32) -> u16 {
    (ffast / fadc) as u16
}

/// Map `iq_to_a` and `iq_to_t` to each pair of `i` and `q`.
fn iq_to_at_map(
    i: [f32; OUTPUT_BUFFER_SIZE],
    q: [f32; OUTPUT_BUFFER_SIZE],
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    let mut a: [f32; OUTPUT_BUFFER_SIZE] = [0.; OUTPUT_BUFFER_SIZE];
    let mut t: [f32; OUTPUT_BUFFER_SIZE] = [0.; OUTPUT_BUFFER_SIZE];
    for k in 0..OUTPUT_BUFFER_SIZE {
        a[k] = iq_to_a(i[k], q[k]);
        t[k] = iq_to_t(i[k], q[k]);
    }
    (a, t)
}

/// Returns magnitude from in-phase and quadrature signals.
///
/// # Arguments
///
/// `i` - In-phase signal.
/// `q` - Quadrature signal.
fn iq_to_a(i: f32, q: f32) -> f32 {
    2. * sqrt(pow2(i) + pow2(q))
}

/// Returns angle from in-phase and quadrature signals.
///
/// # Arguments
///
/// `i` - In-phase signal.
/// `q` - Quadrature signal.
fn iq_to_t(i: f32, q: f32) -> f32 {
    atan2(q, i)
}

/// Demodulation phase values corresponding to each ADC sample.
///
/// # Generics
///
/// * `N` - Number of ADC samples.
///
/// # Arguments
///
/// * `first_t` - First timestamp value from the current processing
/// period. The value provided here doesn't matter if there were no
/// timestamps in the current processing period.
/// * `tstamps` - Recorded TimeStamps.
/// * `phi` - Reference phase offset.
/// * `fscale` - Frequency scaling factor for the demodulation signal.
/// * `tadc` - ADC sampling period.
///
/// # Latency (ADC batch size = 16): 3.27us
///
/// 2.9us from computing `real_phase`.
fn adc_phases(
    first_t: u16,
    tstamps: &mut [TimeStamp; 2],
    phi: u16,
    fscale: u16,
    tadc: u16,
    toggle: &mut hal::gpio::gpiod::PD0<hal::gpio::Output<hal::gpio::PushPull>>,
) -> [f32; SAMPLE_BUFFER_SIZE] {
    let overflow_count: u16 = tadc * SAMPLE_BUFFER_SIZE as u16;
    let tref_count: u16 = tstamps_diff(tstamps, overflow_count);
    let mut thetas: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
    let mut theta_count: u16;

    // 68ns
    if tstamps[0].sequences_old == 0 {
        theta_count = tref_count - first_t;
    } else {
        theta_count = tstamps_diff(
            &[
                TimeStamp {
                    count: 0,
                    sequences_old: 0,
                },
                tstamps[0],
            ],
            overflow_count,
        );
    }

    // toggle.set_high();
    thetas[0] = real_phase(theta_count, fscale, tref_count, phi);
    for i in 1..SAMPLE_BUFFER_SIZE {
        theta_count += tadc;
        thetas[i] = real_phase(theta_count, fscale, tref_count, phi);
    }
    // toggle.set_low();

    thetas
}

/// Number of fast clock counts between two consecutive
/// TimeStamps. This requires that `tstamps[0]` is more recent than
/// `tstamps[1]` but otherwise imposes no restrictions on them. For
/// instance, they can be from different processing periods and this
/// will still count the number of counts between them, accounting for
/// overflow wrapping.
///
/// # Arguments
///
/// * `tstamps` - TimeStamp values.
/// * `overflow_count` - Max timestamp value.
fn tstamps_diff(tstamps: &[TimeStamp; 2], overflow_count: u16) -> u16 {
    if tstamps[0].sequences_old == tstamps[1].sequences_old {
        return tstamps[0].count - tstamps[1].count;
    }

    let rem0: u16 = tstamps[0].count;
    let rem1: u16 = overflow_count - tstamps[1].count;
    let empty_sequences =
        tstamps[1].sequences_old - tstamps[0].sequences_old - 1;

    rem0 + rem1 + overflow_count * empty_sequences as u16
}

/// Increment `sequences_old` in each TimeStamp of `tstamps`.
fn increment_tstamp_sequence(tstamps: &mut [TimeStamp; 2]) {
    tstamps[0].new_sequence();
    tstamps[1].new_sequence();
}

/// Compute the phase (in radians) for a integral phase given in
/// counts relative to some period in counts.
///
/// # Arguments
///
/// * `theta_count` - Phase in counts. This can be greater than the
/// period in counts.
/// * `fscale` - Frequency scaling factor for harmonic demodulation.
/// * `period_count` - Number of counts in 1 period.
/// * `phase_count` - Phase offset. In the same units as `period_count`.
///
/// # Latency (ADC batch size = 16): 138ns
fn real_phase(theta_count: u16, fscale: u16, period_count: u16, phase_count: u16) -> f32 {
    let total_angle = modulo::<u16>(theta_count * fscale + phase_count, period_count);
    2. * PI * (total_angle as f32 / period_count as f32)
}

// Arithmetic modulo operation. This assumes the dividend and divisor are
/// positive, although it also works in some other cases.
fn modulo<T: SubAssign + Ord + Copy>(dividend: T, divisor: T) -> T {
    let mut rem: T = dividend;
    while rem > divisor {
        rem -= divisor;
    }
    rem
}

/// Filter in-phase and quadrature signals with the IIR biquad filter.
///
/// TODO this currently does not offer enough filtering flexibility. For
/// instance, we might want to filter with two consecutive biquads.
///
/// # Arguments
///
/// `i` - In-phase signals.
/// `q` - Quadrature signals.
/// `iir` - IIR filters for the in phase (element 0) and quadrature
/// (element 1) signals.
/// `iirstate` - State of each IIR filter.
///
/// Latency (ADC batch size = 16): 3.5us
fn filter(
    i: [f32; SAMPLE_BUFFER_SIZE],
    q: [f32; SAMPLE_BUFFER_SIZE],
    iir: [iir::IIR; 2],
    iirstate: &mut [iir::IIRState; 2],
) -> ([f32; SAMPLE_BUFFER_SIZE], [f32; SAMPLE_BUFFER_SIZE]) {
    let mut filt_i: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
    let mut filt_q: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];

    for n in 0..SAMPLE_BUFFER_SIZE {
        filt_i[n] = iir[0].update(&mut iirstate[0], i[n]);
        filt_q[n] = iir[1].update(&mut iirstate[1], q[n]);
    }

    (filt_i, filt_q)
}

/// Decimate (downsample) from `SAMPLE_BUFFER_SIZE` to
/// `OUTPUT_BUFFER_SIZE` samples. SAMPLE_BUFFER_SIZE/OUTPUT_BUFFER_SIZE
/// is assumed to be equal to 2**n, where n is some non-negative
/// integer. Decimates the in-phase and quadrature signals separately
/// and returns the result as (i, q).
///
/// # Arguments
///
/// `i` - In-phase signal.
/// `q` - Quadrature signal.
fn decimate(
    i: [f32; SAMPLE_BUFFER_SIZE],
    q: [f32; SAMPLE_BUFFER_SIZE],
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    let n_sub_k: usize = SAMPLE_BUFFER_SIZE / OUTPUT_BUFFER_SIZE;
    let mut res_i: [f32; OUTPUT_BUFFER_SIZE] = [0.; OUTPUT_BUFFER_SIZE];
    let mut res_q: [f32; OUTPUT_BUFFER_SIZE] = [0.; OUTPUT_BUFFER_SIZE];

    let mut n: usize = 0;
    let mut k: usize = 0;
    while n < SAMPLE_BUFFER_SIZE {
        res_i[k] = i[n];
        res_q[k] = q[n];
        k += 1;
        n += n_sub_k;
    }

    (res_i, res_q)
}

/// Demodulate ADC inputs with in-phase and quadrature demodulation
/// signals.
///
/// # Arguments
///
/// * `x` - ADC samples.
/// * `sines` - Reference sine signal.
/// * `cosines` - Reference cosine signal.
fn demod(
    x: [i16; SAMPLE_BUFFER_SIZE],
    sines: [f32; SAMPLE_BUFFER_SIZE],
    cosines: [f32; SAMPLE_BUFFER_SIZE],
) -> ([f32; SAMPLE_BUFFER_SIZE], [f32; SAMPLE_BUFFER_SIZE]) {
    let mut i: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
    let mut q: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];

    for n in 0..SAMPLE_BUFFER_SIZE {
        let xf_n: f32 = x[n] as f32;
        i[n] = xf_n * sines[n];
        q[n] = xf_n * cosines[n];
    }
    (i, q)
}

fn sqrt(x: f32) -> f32 {
    libm::sqrtf(x)
}

fn pow2(x: f32) -> f32 {
    x * x
}

#[cfg(test)]
mod tests {
    // use std::test;
    use super::*;

    fn abs(x: f32) -> f32 {
        if x >= 0. {
            x
        } else {
            -x
        }
    }

    fn max(x: f32, y: f32) -> f32 {
        if x > y {
            x
        } else {
            y
        }
    }

    fn f32_is_close(a: f32, b: f32) -> bool {
        abs(a - b) <= (max(a.abs(), b.abs()) * f32::EPSILON)
    }

    #[test]
    fn arr_n_16_ffast_1e6_fadc_5e5() {
        let ffast: u32 = 100_000_000;
        let fadc: u32 = 500_000;
        let n: u16 = 16;
        assert_eq!(arr(ffast, fadc, n), 3200);
    }

    // TODO tests need to be modified to no longer use const generics
    // #[test]
    // fn decimate_n8_k1() {
    //     let i_in: [f32; 8] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8];
    //     let q_in: [f32; 8] = [0.9, 0.10, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16];
    //     assert!(decimate::<8, 1>(i_in, q_in) == ([0.1], [0.9]));
    // }

    // #[test]
    // fn decimate_n8_k2() {
    //     let i_in: [f32; 8] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8];
    //     let q_in: [f32; 8] = [0.9, 0.10, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16];
    //     assert!(decimate::<8, 2>(i_in, q_in) == ([0.1, 0.5], [0.9, 0.13]));
    // }

    // #[test]
    // fn decimate_n8_k4() {
    //     let i_in: [f32; 8] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8];
    //     let q_in: [f32; 8] = [0.9, 0.10, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16];
    //     assert!(
    //         decimate::<8, 4>(i_in, q_in)
    //             == ([0.1, 0.3, 0.5, 0.7], [0.9, 0.11, 0.13, 0.15])
    //     );
    // }

    // #[test]
    // fn real_phase_per_1000_phi_0() {
    //     let period_count: f32 = 1000.;
    //     let phi: f32 = 0.;
    //     // PI/4 increments
    //     assert!(f32_is_close(real_phase(0, period_count, phi), 0.));
    //     assert!(f32_is_close(real_phase(125, period_count, phi), PI / 4.));
    //     assert!(f32_is_close(real_phase(250, period_count, phi), PI / 2.));
    //     assert!(f32_is_close(
    //         real_phase(375, period_count, phi),
    //         3. * PI / 4.
    //     ));
    //     assert!(f32_is_close(real_phase(500, period_count, phi), PI));
    //     assert!(f32_is_close(
    //         real_phase(625, period_count, phi),
    //         5. * PI / 4.
    //     ));
    //     assert!(f32_is_close(
    //         real_phase(750, period_count, phi),
    //         3. * PI / 2.
    //     ));
    //     assert!(f32_is_close(
    //         real_phase(875, period_count, phi),
    //         7. * PI / 4.
    //     ));
    //     assert!(f32_is_close(real_phase(1000, period_count, phi), 0.));
    //     // other, < 1000
    //     assert!(f32_is_close(
    //         real_phase(1, period_count, phi),
    //         6.28318530718e-3
    //     ));
    //     assert!(f32_is_close(
    //         real_phase(7, period_count, phi),
    //         0.0439822971503
    //     ));
    //     assert!(f32_is_close(
    //         real_phase(763, period_count, phi),
    //         4.79407038938
    //     ));
    //     // > 1000
    //     for angle_count in 0..period_count as usize - 1 {
    //         for p in 0..3 {
    //             assert!(f32_is_close(
    //                 real_phase(
    //                     (angle_count as f32 + p as f32 * period_count) as u16,
    //                     period_count,
    //                     phi
    //                 ),
    //                 real_phase(angle_count as u16, period_count, phi)
    //             ));
    //         }
    //     }
    // }

    // #[test]
    // fn real_phase_per_1000_phi_adjust() {
    //     let period_count: f32 = 1000.;
    //     for theta in [0, 20, 611, 987].iter() {
    //         for phi in 0..period_count as usize - 1 {
    //             assert!(f32_is_close(
    //                 real_phase(*theta, period_count, phi as f32),
    //                 real_phase(*theta + phi as u16, period_count, 0.)
    //             ))
    //         }
    //     }
    // }

    // #[test]
    // fn increment_tstamp_sequence_valid_invalid() {
    //     let mut tstamps = [
    //         TimeStamp {
    //             count: 0,
    //             sequences_old: 0,
    //         },
    //         TimeStamp {
    //             count: 0,
    //             sequences_old: -1,
    //         },
    //     ];
    //     increment_tstamp_sequence(&mut tstamps);
    //     assert_eq!(tstamps[0].sequences_old, 1);
    //     assert_eq!(tstamps[1].sequences_old, -1);
    // }

    // #[test]
    // fn tstamps_valid_count_test() {
    //     for (valid_num, old1, old2) in
    //         [(0, -1, -1), (1, 1, -1), (1, -1, 1), (2, 1, 1)].iter()
    //     {
    //         assert_eq!(
    //             tstamps_valid_count(&[
    //                 TimeStamp {
    //                     count: 5,
    //                     sequences_old: *old1 as i16,
    //                 },
    //                 TimeStamp {
    //                     count: 0,
    //                     sequences_old: *old2 as i16,
    //                 }
    //             ]),
    //             *valid_num as usize
    //         );
    //     }
    // }

    // #[test]
    // fn tadc_test() {
    //     assert_eq!(tadc(100_000_000, 500_000), 200);
    //     assert_eq!(tadc(125_000_000, 500_000), 250);
    //     assert_eq!(tadc(100_000_000, 300_000), 333);
    // }

    // #[test]
    // fn record_new_tstamps_m4_r1() {
    //     let mut tstamps = [
    //         TimeStamp {
    //             count: 0,
    //             sequences_old: -1,
    //         },
    //         TimeStamp {
    //             count: 0,
    //             sequences_old: -1,
    //         },
    //     ];
    //     record_new_tstamps::<4>([1, 0, 0, 0], 1, &mut tstamps);
    //     assert_eq!(tstamps[0].count, 1);
    //     assert_eq!(tstamps[0].sequences_old, 0);
    //     assert_eq!(tstamps[1].count, 0);
    //     assert_eq!(tstamps[1].sequences_old, -1);
    // }

    // #[test]
    // fn record_new_tstamps_m4_r2() {
    //     let mut tstamps = [
    //         TimeStamp {
    //             count: 0,
    //             sequences_old: -1,
    //         },
    //         TimeStamp {
    //             count: 0,
    //             sequences_old: -1,
    //         },
    //     ];
    //     record_new_tstamps::<4>([1, 2, 0, 0], 2, &mut tstamps);
    //     assert_eq!(tstamps[0].count, 2);
    //     assert_eq!(tstamps[0].sequences_old, 0);
    //     assert_eq!(tstamps[1].count, 1);
    //     assert_eq!(tstamps[1].sequences_old, 0);
    // }

    // #[test]
    // fn record_new_tstamps_m4_r3() {
    //     let mut tstamps = [
    //         TimeStamp {
    //             count: 0,
    //             sequences_old: -1,
    //         },
    //         TimeStamp {
    //             count: 0,
    //             sequences_old: -1,
    //         },
    //     ];
    //     record_new_tstamps::<4>([1, 2, 3, 0], 3, &mut tstamps);
    //     assert_eq!(tstamps[0].count, 3);
    //     assert_eq!(tstamps[0].sequences_old, 0);
    //     assert_eq!(tstamps[1].count, 2);
    //     assert_eq!(tstamps[1].sequences_old, 0);
    // }

    // #[test]
    // fn iq_to_a_test() {
    //     assert!(f32_is_close(
    //         iq_to_a(1. / 2f32.sqrt(), 1. / 2f32.sqrt()),
    //         2.
    //     ));
    //     assert!(f32_is_close(iq_to_a(0.1, 1.6), 3.20624390838));
    //     assert!(f32_is_close(iq_to_a(-0.1, 1.6), 3.20624390838));
    //     assert!(f32_is_close(iq_to_a(0.1, -1.6), 3.20624390838));
    // }

    // #[test]
    // fn tstamps_diff_ofcount_1000() {
    //     let overflow_count: u16 = 1000;
    //     // same sequence
    //     let tstamps = [
    //         TimeStamp {
    //             count: 126,
    //             sequences_old: 0,
    //         },
    //         TimeStamp {
    //             count: 33,
    //             sequences_old: 0,
    //         },
    //     ];
    //     assert_eq!(tstamps_diff(&tstamps, overflow_count), 93);
    //     // adjacent sequences
    //     let tstamps = [
    //         TimeStamp {
    //             count: 9,
    //             sequences_old: 0,
    //         },
    //         TimeStamp {
    //             count: 33,
    //             sequences_old: 1,
    //         },
    //     ];
    //     assert_eq!(tstamps_diff(&tstamps, overflow_count), 976);
    //     let tstamps = [
    //         TimeStamp {
    //             count: 35,
    //             sequences_old: 0,
    //         },
    //         TimeStamp {
    //             count: 33,
    //             sequences_old: 1,
    //         },
    //     ];
    //     assert_eq!(tstamps_diff(&tstamps, overflow_count), 1002);
    //     // non-adjacent sequences
    //     let tstamps = [
    //         TimeStamp {
    //             count: 9,
    //             sequences_old: 0,
    //         },
    //         TimeStamp {
    //             count: 33,
    //             sequences_old: 2,
    //         },
    //     ];
    //     assert_eq!(tstamps_diff(&tstamps, overflow_count), 1976);
    //     let tstamps = [
    //         TimeStamp {
    //             count: 35,
    //             sequences_old: 0,
    //         },
    //         TimeStamp {
    //             count: 33,
    //             sequences_old: 2,
    //         },
    //     ];
    //     assert_eq!(tstamps_diff(&tstamps, overflow_count), 2002);
    // }

    // #[test]
    // fn demod_n4() {
    //     const SAMPLE_BUFFER_SIZE: usize = 4;
    //     let x: [i16; SAMPLE_BUFFER_SIZE] = [-1, 0, 1, -18];
    //     let sines: [f32; SAMPLE_BUFFER_SIZE] = [0.1, -0.3, 18.76, -33.1];
    //     let cosines: [f32; SAMPLE_BUFFER_SIZE] = [-0.2389, 0.1823, 0.123, -0.5839];
    //     let (i, q) = demod::<N>(x, sines, cosines);
    //     for n in 0..SAMPLE_BUFFER_SIZE {
    //         assert!(f32_is_close(i[n], x[n] as f32 * sines[n]));
    //         assert!(f32_is_close(q[n], x[n] as f32 * cosines[n]));
    //     }
    // }
}
