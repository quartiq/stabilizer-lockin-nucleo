mod sinf;
mod k_sinf;
mod cosf;
mod k_cosf;
mod rem_pio2f;

use super::trig::sinf::sinf;
use super::trig::cosf::cosf;

/// Computes the sine of a value `theta` in the range [0, 2*PI).
///
/// # Arguments
///
/// * `theta` - Angle for which sine is computed. Must be in range
/// [0,2*PI).
pub fn sin(theta: f32) -> f32 {
    return sinf(theta);
}

/// Computes the cosine of a value `theta` in the range [0, 2*PI).
///
/// # Arguments
///
/// * `theta` - Angle for which cosine is computed. Must be in range
/// [0,2*PI).
pub fn cos(theta: f32) -> f32 {
    return cosf(theta);
}

/// Computes the four quadrant arctangent.
///
/// # Arguments
///
/// * `y` - Y coordinate value. Can be any real value.
/// * `x` - X coordinate value. Can be any real value.
pub fn atan2(y: f32, x: f32) -> f32 {
    return libm::atan2f(y, x);
}
