use core::f32::consts::PI;
use super::sinf::sinf;
use super::cosf::cosf;

const TRIG_N: usize = 1_000;
static SIN_TABLE: [f32; TRIG_N] = include!("sin_table.txt");
static ATAN_TABLE: [f32; TRIG_N] = include!("atan_table.txt");

/// Computes the sine of a value `theta` in the range [0, 2*PI).
///
/// # Arguments
///
/// * `theta` - Angle for which sine is computed. Must be in range
/// [0,2*PI).
pub fn sin(theta: f32) -> f32 {
    // TODO remove for custom implementation
    return sinf(theta);

    // let (factor, theta) = restrict_angle(theta);
    // let pos = table_pos(theta, PI / 2.);
    // let floor = pos as usize;
    // let ceil = floor + 1;

    // if ceil > TRIG_N - 1 {
    //     return factor * SIN_TABLE[TRIG_N - 1];
    // }

    // let rem = pos - floor as f32;
    // let floor_sin = SIN_TABLE[floor];
    // let ceil_sin = SIN_TABLE[ceil];

    // factor * lin_interp(rem, floor_sin, ceil_sin)
}

/// Computes the cosine of a value `theta` in the range [0, 2*PI).
///
/// # Arguments
///
/// * `theta` - Angle for which cosine is computed. Must be in range
/// [0,2*PI).
pub fn cos(theta: f32) -> f32 {
    // TODO remove for custom implementation
    return cosf(theta);

    // let mut theta = theta + (PI / 2.);
    // if theta >= 2. * PI {
    //     theta -= 2. * PI;
    // }
    // sin(theta)
}

/// Computes the four quadrant arctangent.
///
/// # Arguments
///
/// * `y` - Y coordinate value. Can be any real value.
/// * `x` - X coordinate value. Can be any real value.
pub fn atan2(y: f32, x: f32) -> f32 {
    // TODO remove for custom implementation
    return libm::atan2f(y, x);

    // // I believe atan2(0,0) is technically undefined, not =0, but the
    // // rust standard library defines it this way.
    // if x == 0. && y == 0. {
    //     return 0.;
    // }
    // if x < 0. {
    //     if y < 0. {
    //         return atan(y / x) - PI;
    //     }
    //     return atan(y / x) + PI;
    // }
    // atan(y / x)
}

fn atan(ratio: f32) -> f32 {
    let (factor, invert, ratio) = restrict_ratio(ratio);
    let pos = table_pos(ratio, 1.);
    let floor = pos as usize;
    let ceil = floor + 1;

    if ceil > TRIG_N - 1 {
        if invert {
            return factor * (PI / 2. - ATAN_TABLE[TRIG_N - 1]);
        } else {
            return factor * ATAN_TABLE[TRIG_N - 1];
        }
    }

    let rem = pos - floor as f32;
    let floor = ATAN_TABLE[floor];
    let ceil = ATAN_TABLE[ceil];

    if invert {
        return factor * (PI / 2. - lin_interp(rem, floor, ceil));
    }
    factor * lin_interp(rem, floor, ceil)
}

/// Restricts ratio passed to atan to a value between 0 and 1 (both
/// inclusive).
fn restrict_ratio(ratio: f32) -> (f32, bool, f32) {
    let mut factor: f32 = 1.;
    let mut ratio: f32 = ratio;
    let mut invert: bool = false;

    if ratio < 0. {
        factor = -1.;
        ratio *= -1.;
    }

    if ratio > 1. {
        invert = true;
        ratio = 1. / ratio;
    }

    (factor, invert, ratio)
}

/// Returns a factor, $f\in\{-1,1\}$ and equivalent angle,
/// $\theta'\in[0, \pi/2]$ such that $\sin(\theta)=f\sin(\theta')$,
/// where $\theta\in[0, 2*pi)$ is the input angle.
///
/// # Arguments
///
/// * `theta` - The angle in the range [0, 2*pi) to be restricted.
fn restrict_angle(theta: f32) -> (f32, f32) {
    #[cfg(debug_assertions)]
    if theta < 0. || theta >= 2. * PI {
        panic!("theta must be between 0 (inclusive) and 2pi (exclusive)");
    }

    if theta < PI / 2. {
        return (1., theta);
    } else if theta < PI {
        return (1., PI - theta);
    } else if theta < 3. * PI / 2. {
        return (-1., theta - PI);
    }
    (-1., 2. * PI - theta)
}

// Relative position of `val` with respect to a lookup table's
// indices. This returns a floating-point value so that we can use
// this position to interpolate between two existing table values.
fn table_pos(val: f32, max: f32) -> f32 {
    (TRIG_N as f32) * val / max
}

// Perform a linear interpolation. pos is the percentage distance from
// lval (the lower bound) relative to the total distance between lval
// and uval (the upper bound).
fn lin_interp(pos: f32, lval: f32, uval: f32) -> f32 {
    lval + (pos * (uval - lval))
}

#[cfg(test)]
mod tests {
    // extern crate std;
    // use std::test;
    use super::*;

    #[test]
    fn sin_tol_0_001() {
        const N: usize = 100_000;
        let delta = 2. * std::f64::consts::PI / N as f64;
        let tol: f32 = 1e-3;

        for i in 0..N {
            let theta = (i as f64 * delta) as f32;
            let res = sin(theta);
            let act = theta.sin();
            assert!(
                (res - act).abs() < tol,
                "theta: {}, res: {}, act: {}",
                theta,
                res,
                act
            );
        }

        // explicitly test quadrand boundaries
        let bounds: [f32; 4] = [0., PI / 2., PI, 3. * PI / 2.];
        for theta in bounds.iter() {
            let res = sin(*theta);
            let act = theta.sin();
            assert!(
                (res - act).abs() < tol,
                "theta: {}, res: {}, act: {}",
                theta,
                res,
                act
            );
        }
    }

    #[test]
    fn cos_tol_0_001() {
        const N: usize = 100_000;
        let delta = 2. * std::f64::consts::PI / N as f64;
        let tol: f32 = 1e-3;

        for i in 0..N {
            let theta = (i as f64 * delta) as f32;
            let res = cos(theta);
            let act = theta.cos();
            assert!(
                (res - act).abs() < tol,
                "theta: {}, res: {}, act: {}",
                theta,
                res,
                act
            );
        }

        let bounds: [f32; 4] = [0., PI / 2., PI, 3. * PI / 2.];
        for theta in bounds.iter() {
            let res = cos(*theta);
            let act = theta.cos();
            assert!(
                (res - act).abs() < tol,
                "theta: {}, res: {}, act: {}",
                theta,
                res,
                act
            );
        }
    }

    #[test]
    fn atan_tol_0_001_range_100() {
        const N: usize = 1_000_000;
        let range: f64 = 100.;
        let delta = range / N as f64;
        let ratio_start = -range / 2.;
        let tol: f32 = 1e-3;

        for i in 0..N {
            let ratio = (ratio_start + i as f64 * delta) as f32;
            let res = atan(ratio);
            let act = ratio.atan();
            assert!(
                (res - act).abs() < tol,
                "ratio: {}, res: {}, act: {}",
                ratio,
                res,
                act
            );
        }

        let bounds: [f32; 5] = [-f32::INFINITY, -1., 0., 1., f32::INFINITY];
        for ratio in bounds.iter() {
            let res = atan(*ratio);
            let act = ratio.atan();
            assert!(
                (res - act).abs() < tol,
                "ratio: {}, res: {}, act: {}",
                ratio,
                res,
                act
            );
        }
    }

    #[test]
    fn atan2_tol_0_001_range_100() {
        const N: usize = 1_000;
        let range: f64 = 10.;
        let delta = range / N as f64;
        let val_start = -range / 2.;
        let tol: f32 = 1e-3;

        let mut vals = [0f32; N];
        for i in 0..N {
            vals[i] = (val_start + i as f64 * delta) as f32;
        }

        for x in vals.iter() {
            for y in vals.iter() {
                let res = atan2(*y, *x);
                let act = y.atan2(*x);
                assert!(
                    (res - act).abs() < tol,
                    "x: {}, y: {}, res: {}, act: {}",
                    *x,
                    *y,
                    res,
                    act
                );
            }
        }
    }
}
