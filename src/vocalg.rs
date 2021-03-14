/// This work is a port of Sensirion VOC indexing algorithm from
/// https://github.com/Sensirion/embedded-sgp/tree/master/sgp40_voc_index
/*
 * Copyright (c) 2020, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
use fixed::{traits::FromFixed, types::I16F16};
//use fixed_exp::FixedPowF;
use fixed_macro::fixed;
use fixed_sqrt::FixedSqrt;

type Fix = I16F16;

macro_rules! alg_fixed {
    ($a:expr) => {{
        fixed!($a: I16F16)
    }};
}

const SAMPLING_INTERVAL: Fix = fixed!(1: I16F16);
const INITIAL_BLACKOUT: Fix = fixed!(45: I16F16);
const VOC_INDEX_GAIN: Fix = fixed!(230: I16F16);
const SRAW_STD_INITIAL: Fix = fixed!(50: I16F16);
const SRAW_STD_BONUS: Fix = fixed!(220: I16F16);
const TAU_MEAN_VARIANCE_HOURS: Fix = fixed!(12: I16F16);
const TAU_INITIAL_MEAN: Fix = fixed!(20: I16F16);
const INIT_DURATION_MEAN: Fix = fixed!(2700: I16F16); // 3600. * 0.75
const INIT_TRANSITION_MEAN: Fix = fixed!(0.01: I16F16);
const TAU_INITIAL_VARIANCE: Fix = fixed!(2500: I16F16);
const INIT_DURATION_VARIANCE: Fix = fixed!(5200: I16F16); // 3600. * 1.45
const INIT_TRANSITION_VARIANCE: Fix = fixed!(0.01: I16F16);
const GATING_THRESHOLD: Fix = fixed!(340: I16F16);
const GATING_THRESHOLD_INITIAL: Fix = fixed!(510: I16F16);
const GATING_THRESHOLD_TRANSITION: Fix = fixed!(0.09: I16F16);
const GATING_MAX_DURATION_MINUTES: Fix = fixed!(180: I16F16); // 60.0 * 3.0
const GATING_MAX_RATIO: Fix = fixed!(0.3: I16F16);
const SIGMOID_L: Fix = fixed!(500: I16F16);
const SIGMOID_K: Fix = fixed!(-0.0065: I16F16);
const SIGMOID_X0: Fix = fixed!(213: I16F16);
const VOC_INDEX_OFFSET_DEFAULT: Fix = fixed!(100: I16F16);
const LP_TAU_FAST: Fix = fixed!(20: I16F16);
const LP_TAU_SLOW: Fix = fixed!(500: I16F16);
const LP_ALPHA: Fix = fixed!(-0.2: I16F16);
const PERSISTENCE_UPTIME_GAMMA: Fix = fixed!(10800: I16F16); // 3 * 3600
const MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING: Fix = fixed!(64: I16F16);
const MEAN_VARIANCE_ESTIMATOR__FIX16_MAX: Fix = fixed!(32767: I16F16);

// Stores VOC algorithm states
pub struct VocAlgorithm {
    voc_index_offset: Fix,
    tau_mean_variance_hours: Fix,
    gating_max_duration_minutes: Fix,
    sraw_std_initial: Fix,
    uptime: Fix,
    sraw: Fix,
    voc_index: Fix,
    mean_variance_estimator: MeanVarianceEstimator,
    mox_model: MoxModel,
    sigmoid_scaled: SigmoidScaledInit,
    adaptive_lowpass: AdaptiveLowpass,
}

impl VocAlgorithm {
    pub fn new() -> Self {
        let (mean_variance_estimator, mox_model, sigmoid_scaled, adaptive_lowpass) = VocAlgorithm::new_instances(
            SRAW_STD_INITIAL,
            TAU_MEAN_VARIANCE_HOURS,
            GATING_MAX_DURATION_MINUTES,
            VOC_INDEX_OFFSET_DEFAULT,
        );

        VocAlgorithm {
            voc_index_offset: VOC_INDEX_OFFSET_DEFAULT,
            tau_mean_variance_hours: TAU_MEAN_VARIANCE_HOURS,
            gating_max_duration_minutes: GATING_MAX_DURATION_MINUTES,
            sraw_std_initial: SRAW_STD_INITIAL,
            uptime: alg_fixed!(0),
            sraw: alg_fixed!(0),
            voc_index: alg_fixed!(0),
            mean_variance_estimator,
            mox_model,
            sigmoid_scaled,
            adaptive_lowpass,
        }
    }

    fn new_instances(
        sraw_std_initial: Fix,
        tau_mean_variance_hours: Fix,
        gating_max_duration_minutes: Fix,
        voc_index_offset: Fix,
    ) -> (MeanVarianceEstimator, MoxModel, SigmoidScaledInit, AdaptiveLowpass) {
        let mut mean_variance_estimator = MeanVarianceEstimator::new();
        mean_variance_estimator.set_parameters(sraw_std_initial, tau_mean_variance_hours, gating_max_duration_minutes);

        let mox_model = MoxModel::new(mean_variance_estimator.get_std(), mean_variance_estimator.get_mean());

        let sigmoid_scaled = SigmoidScaledInit::new(voc_index_offset);

        let adaptive_lowpass = AdaptiveLowpass::new();

        (mean_variance_estimator, mox_model, sigmoid_scaled, adaptive_lowpass)
    }

    /// Returns state0, state1
    pub fn get_states(&self) -> (i32, i32) {
        (
            i32::saturating_from_fixed(self.mean_variance_estimator.get_mean()),
            i32::saturating_from_fixed(self.mean_variance_estimator.get_std()),
        )
    }

    /// Used for setting state0 and state1 to return to the previously calibrated states.
    pub fn set_states(&mut self, state0: i32, state1: i32) {
        self.mean_variance_estimator
            .set_states(Fix::from_num(state0), Fix::from_num(state1), PERSISTENCE_UPTIME_GAMMA);

        self.sraw = Fix::from_num(state0);
    }

    pub fn set_tuning_parameters(
        &mut self,
        voc_index_offset: i32,
        learning_time_hours: i32,
        gating_duration_minutes: i32,
        std_initial: i32,
    ) {
        let (mean_variance_estimator, mox_model, sigmoid_scaled, adaptive_lowpass) = VocAlgorithm::new_instances(
            Fix::from_num(std_initial),
            Fix::from_num(learning_time_hours),
            Fix::from_num(gating_duration_minutes),
            Fix::from_num(voc_index_offset),
        );

        self.mean_variance_estimator = mean_variance_estimator;
        self.mox_model = mox_model;
        self.sigmoid_scaled = sigmoid_scaled;
        self.adaptive_lowpass = adaptive_lowpass;
    }

    pub fn process(&mut self, sraw: i32) -> i32 {
        if self.uptime <= INITIAL_BLACKOUT {
            self.uptime += SAMPLING_INTERVAL;
        } else {
            assert!(sraw > 0 && sraw < 65000);

            // -20000 from all the numbers
            let sraw = if sraw < 20001 {
                1
            } else if sraw > 52767 {
                32767
            } else {
                sraw - 20000
            };

            self.sraw = Fix::from_num(sraw);

            //println!("SRAW: {}", self.sraw);

            self.voc_index = self.mox_model.process(self.sraw);
            //println!("After Mox: {}", self.voc_index);

            self.voc_index = self.sigmoid_scaled.process(self.voc_index);
            //println!("After Sigmoid scaled: {}", self.voc_index);

            self.voc_index = self.adaptive_lowpass.process(self.voc_index);
            //println!("After Adaptive lowpass: {}", self.voc_index);

            if self.voc_index < alg_fixed!(0.5) {
                self.voc_index = alg_fixed!(0.5);
            }

            if self.sraw > alg_fixed!(0) {
                self.mean_variance_estimator.process(self.sraw, self.voc_index);
                //println!("Est std:{} mean:{}", self.mean_variance_estimator.get_std(),self.mean_variance_estimator.get_mean());
                self.mox_model = MoxModel::new(
                    self.mean_variance_estimator.get_std(),
                    self.mean_variance_estimator.get_mean(),
                );
            }
        }
        i32::saturating_from_fixed(self.voc_index + alg_fixed!(0.5))
    }
}

struct MeanVarianceEstimator {
    gating_max_duration_minutes: Fix,
    mean: Fix,
    sraw_offset: Fix,
    std: Fix,
    gamma: Fix,
    gamma_initial_mean: Fix,
    gamma_initial_variance: Fix,
    gamma_mean: Fix,
    gamma_variance: Fix,
    uptime_gamma: Fix,
    uptime_gating: Fix,
    gating_duration_minutes: Fix,
    sigmoid: MeanVarianceEstimatorSigmoid,
    initialized: bool,
}

impl MeanVarianceEstimator {
    fn new() -> Self {
        MeanVarianceEstimator {
            gating_max_duration_minutes: alg_fixed!(0),
            mean: alg_fixed!(0),
            sraw_offset: alg_fixed!(0),
            std: alg_fixed!(0),
            gamma: alg_fixed!(0),
            gamma_initial_mean: alg_fixed!(0),
            gamma_initial_variance: alg_fixed!(0),
            gamma_mean: alg_fixed!(0),
            gamma_variance: alg_fixed!(0),
            uptime_gamma: alg_fixed!(0),
            uptime_gating: alg_fixed!(0),
            gating_duration_minutes: alg_fixed!(0),
            sigmoid: MeanVarianceEstimatorSigmoid::new(),
            initialized: false,
        }
    }

    fn set_parameters(&mut self, std_initial: Fix, tau_mean_variance_hours: Fix, gating_max_duration_minutes: Fix) {
        self.gating_max_duration_minutes = gating_max_duration_minutes;
        self.initialized = false;
        self.mean = alg_fixed!(0);
        self.sraw_offset = alg_fixed!(0);
        self.std = std_initial;
        self.gamma = (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * (SAMPLING_INTERVAL / alg_fixed!(3600)))
            / (tau_mean_variance_hours + SAMPLING_INTERVAL / alg_fixed!(3600));
        self.gamma_initial_mean =
            (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * SAMPLING_INTERVAL) / (TAU_INITIAL_MEAN + SAMPLING_INTERVAL);
        self.gamma_initial_variance =
            (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * SAMPLING_INTERVAL) / (TAU_INITIAL_VARIANCE + SAMPLING_INTERVAL);
        self.gamma_mean = alg_fixed!(0);
        self.gamma_variance = alg_fixed!(0);
        self.uptime_gamma = alg_fixed!(0);
        self.uptime_gating = alg_fixed!(0);
        self.gating_duration_minutes = alg_fixed!(0);
    }

    fn set_states(&mut self, mean: Fix, std: Fix, uptime_gamma: Fix) {
        self.mean = mean;
        self.std = std;
        self.uptime_gamma = uptime_gamma;
        self.initialized = true;
    }

    fn get_std(&self) -> Fix {
        self.std
    }

    fn get_mean(&self) -> Fix {
        self.mean + self.sraw_offset
    }

    fn calculate_gamma(&mut self, voc_index_from_prior: Fix) {
        // Check this as we are likely running in 32-bit environment
        let uptime_limit = MEAN_VARIANCE_ESTIMATOR__FIX16_MAX - SAMPLING_INTERVAL;

        //println!("Updatime gamma:{}", self.uptime_gamma);

        if self.uptime_gamma < uptime_limit {
            self.uptime_gamma += SAMPLING_INTERVAL;
        }

        if self.uptime_gating < uptime_limit {
            self.uptime_gating += SAMPLING_INTERVAL;
        }

        self.sigmoid
            .set_parameters(alg_fixed!(1), INIT_DURATION_MEAN, INIT_TRANSITION_MEAN);

        let sigmoid_gamma_mean = self.sigmoid.process(self.uptime_gamma);

        let gamma_mean = self.gamma + ((self.gamma_initial_mean - self.gamma) * sigmoid_gamma_mean);

        let gating_threshold_mean =
            GATING_THRESHOLD + (GATING_THRESHOLD_INITIAL - GATING_THRESHOLD) * self.sigmoid.process(self.uptime_gating);

        self.sigmoid
            .set_parameters(alg_fixed!(1), gating_threshold_mean, GATING_THRESHOLD_TRANSITION);

        let sigmoid_gating_mean = self.sigmoid.process(voc_index_from_prior);

        self.gamma_mean = sigmoid_gating_mean * gamma_mean;

        self.sigmoid
            .set_parameters(alg_fixed!(1), INIT_DURATION_VARIANCE, INIT_TRANSITION_VARIANCE);

        let sigmoid_gamma_variance = self.sigmoid.process(self.uptime_gamma);

        let gamma_variance =
            self.gamma + (self.gamma_initial_variance - self.gamma) * (sigmoid_gamma_variance - sigmoid_gamma_mean);

        let gating_threshold_variance =
            GATING_THRESHOLD + (GATING_THRESHOLD_INITIAL - GATING_THRESHOLD) * self.sigmoid.process(self.uptime_gating);

        self.sigmoid
            .set_parameters(alg_fixed!(1), gating_threshold_variance, GATING_THRESHOLD_TRANSITION);

        let sigmoid_gating_variance = self.sigmoid.process(voc_index_from_prior);

        self.gamma_variance = sigmoid_gating_variance * gamma_variance;

        self.gating_duration_minutes += (SAMPLING_INTERVAL / alg_fixed!(60))
            * (((alg_fixed!(1) - sigmoid_gating_mean) * (alg_fixed!(1) + GATING_MAX_RATIO)) - GATING_MAX_RATIO);

        if self.gating_duration_minutes < alg_fixed!(0) {
            self.gating_duration_minutes = alg_fixed!(0);
        }

        if self.gating_duration_minutes > self.gating_max_duration_minutes {
            self.uptime_gating = alg_fixed!(0);
        }
    }

    fn process(&mut self, sraw: Fix, voc_index_from_prior: Fix) {
        if self.initialized == false {
            self.initialized = true;
            self.sraw_offset = sraw;
            self.mean = alg_fixed!(0);
        } else {
            if self.mean >= alg_fixed!(100) || self.mean <= alg_fixed!(-100) {
                self.sraw_offset += self.mean;
                self.mean = alg_fixed!(0);
                //println!("Mean reset");
            }

            let sraw = sraw - self.sraw_offset;

            self.calculate_gamma(voc_index_from_prior);

            //println!("Gamma variance:{}", self.gamma_variance);

            let delta_sgp = (sraw - self.mean) / MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING;

            let c = self.std + delta_sgp.abs();

            let additional_scaling = if c > alg_fixed!(1440) {
                alg_fixed!(4)
            } else {
                alg_fixed!(1)
            };

            self.std = (additional_scaling * (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING - self.gamma_variance)).sqrt()
                * ((self.std * (self.std / (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * additional_scaling)))
                    + (((self.gamma_variance * delta_sgp) / additional_scaling) * delta_sgp))
                    .sqrt();

            self.mean += self.gamma_mean * delta_sgp;

            //println!("Final mean:{} std:{}", self.mean, self.std);
        }
    }
}

struct MeanVarianceEstimatorSigmoid {
    L: Fix,
    K: Fix,
    X0: Fix,
}

impl MeanVarianceEstimatorSigmoid {
    fn new() -> Self {
        MeanVarianceEstimatorSigmoid {
            L: alg_fixed!(0),
            K: alg_fixed!(0),
            X0: alg_fixed!(0),
        }
    }

    fn process(&self, sample: Fix) -> Fix {
        let (x, b) = self.K.overflowing_mul(sample - self.X0);

        //println!("Sigmoid: sample:{} x:{}", sample, x);
        if b {
            return alg_fixed!(0);
        }

        if x < alg_fixed!(-50) {
            return self.L;
        } else if x > alg_fixed!(50) {
            return alg_fixed!(0);
        } else {
            let result = self.L / (alg_fixed!(1) + fixed_exp(x));
            //println!("Sigmoid:{}", result);
            return result;
        }
    }

    fn set_parameters(&mut self, L: Fix, K: Fix, X0: Fix) {
        self.L = L;
        self.K = K;
        self.X0 = X0;
    }
}

// Needs new implementation as the original code prevents going above 16-bit ranges
fn fixed_exp(x: Fix) -> Fix {
    let exp_pos_values = [
        alg_fixed!(2.7182818),
        alg_fixed!(1.1331485),
        alg_fixed!(1.0157477),
        alg_fixed!(1.0019550),
    ];
    let exp_neg_values = [
        alg_fixed!(0.3678794),
        alg_fixed!(0.8824969),
        alg_fixed!(0.9844964),
        alg_fixed!(0.9980488),
    ];

    if x >= alg_fixed!(10.3972) {
        // The maximum value is often used in the context of adding one so it is the best
        // dealt here (won't have significant impact to the formulas)
        MEAN_VARIANCE_ESTIMATOR__FIX16_MAX - alg_fixed!(1)
    } else if x <= alg_fixed!(-11.7835) {
        alg_fixed!(0)
    } else {
        // I guess we need to calculate (read:approximate this)
        let mut x = x;

        let val = if x < alg_fixed!(0) {
            x = -x;
            exp_neg_values
        } else {
            exp_pos_values
        };

        let mut res = alg_fixed!(1);
        let mut arg = alg_fixed!(1);

        for &v in &val {
            while x >= arg {
                res *= v;
                x -= arg;
            }
            arg >>= 3;
        }
        res
    }
}

struct SigmoidScaledInit {
    offset: Fix,
}

impl SigmoidScaledInit {
    fn new(offset: Fix) -> Self {
        SigmoidScaledInit { offset }
    }

    fn process(&self, sample: Fix) -> Fix {
        let x = SIGMOID_K * (sample - SIGMOID_X0);

        if x < alg_fixed!(-50) {
            return SIGMOID_L;
        } else if x > alg_fixed!(50) {
            return alg_fixed!(0);
        } else {
            //println!("Sample {}, offset:{} X:{}", sample, self.offset, x);
            if sample >= alg_fixed!(0) {
                let shift = (SIGMOID_L - (alg_fixed!(5) * self.offset)) / alg_fixed!(4);
                return ((SIGMOID_L + shift) / (alg_fixed!(1) + fixed_exp(x))) - shift;
            } else {
                //println!("X^{}", SigmoidScaledInit::exp(x));
                return (self.offset / VOC_INDEX_OFFSET_DEFAULT) * (SIGMOID_L / (alg_fixed!(1) + fixed_exp(x)));
            }
        }
    }
}

struct MoxModel {
    sraw_std: Fix,
    sraw_mean: Fix,
}

impl MoxModel {
    fn new(sraw_std: Fix, sraw_mean: Fix) -> Self {
        MoxModel { sraw_std, sraw_mean }
    }

    fn process(&self, sraw: Fix) -> Fix {
        ((sraw - self.sraw_mean) / (-(self.sraw_std + SRAW_STD_BONUS))) * VOC_INDEX_GAIN
    }
}

struct AdaptiveLowpass {
    A1: Fix,
    A2: Fix,
    X1: Fix,
    X2: Fix,
    X3: Fix,
    initialized: bool,
}

impl AdaptiveLowpass {
    fn new() -> Self {
        AdaptiveLowpass {
            A1: SAMPLING_INTERVAL / (LP_TAU_FAST + SAMPLING_INTERVAL),
            A2: SAMPLING_INTERVAL / (LP_TAU_SLOW + SAMPLING_INTERVAL),
            initialized: false,
            X1: alg_fixed!(0),
            X2: alg_fixed!(0),
            X3: alg_fixed!(0),
        }
    }

    fn process(&mut self, sample: Fix) -> Fix {
        if self.initialized == false {
            self.X1 = sample;
            self.X2 = sample;
            self.X3 = sample;
            self.initialized = true;
        }

        self.X1 = (alg_fixed!(1) - self.A1) * self.X1 + self.A1 * sample;
        self.X2 = (alg_fixed!(1) - self.A2) * self.X2 + self.A2 * sample;

        let abs_delta = (self.X1 - self.X2).abs();
        let F1 = fixed_exp(LP_ALPHA * abs_delta);
        let tau_a = ((LP_TAU_SLOW - LP_TAU_FAST) * F1) + LP_TAU_FAST;
        let a3 = SAMPLING_INTERVAL / (SAMPLING_INTERVAL + tau_a);
        self.X3 = (alg_fixed!(1) - a3) * self.X3 + a3 * sample;
        self.X3
    }
}

// Overall integration tests are needed as
#[cfg(all(test, not(no_std)))]
#[path = "test_vocalg.rs"]
mod test_vocalg;
mod tests {
    use super::*;

    #[test]
    fn test_basic() {
        let epsilon = 1;

        let mut voc = VocAlgorithm::new();
        let mut ref_voc = test_vocalg::VocAlgorithm::new();

        for _ in 1..1000 {
            value = voc.process(28_000);
            let ref_value = ref_voc.process(28_000);

            assert!((value - ref_value).abs() <= epsilon);
        }
    }

    #[test]
    fn whole_scale() {
        let epsilon = 1;

        let mut voc = VocAlgorithm::new();
        let mut ref_voc = test_vocalg::VocAlgorithm::new();

        // First 45 samples are effectively no-op so they are taken out prior starting
        // real testing
        for _ in 1..45 {
            let value = voc.process(20000);
            let ref_value = ref_voc.process(20000);

            assert!((value - ref_value).abs() <= epsilon);
        }

        for i in 20_000..52_768 {
            // We'll take 20 samples with the same value
            for _ in 1..20 {
                let value = voc.process(i);
                let ref_value = ref_voc.process(i);

                assert!((value - ref_value).abs() <= epsilon);
            }
        }
    }

    /* There is something wrong with VOC index algorithm. Not enabled yet
    #[test]
    fn min_value() {
        let epsilon = 1;

        let mut voc = VocAlgorithm::new();
        let mut ref_voc = test_vocalg::VocAlgorithm::new();

        let mut value = 0;

        for _ in 1..10000 {
            value = voc.process(52000);
            let ref_value = ref_voc.process(52000);

            assert!((value - ref_value).abs() <= epsilon);
        }

        assert_eq!(0, value);
    }

    #[test]
    fn max_value() {
        let epsilon = 1;

        let mut voc = VocAlgorithm::new();
        let mut ref_voc = test_vocalg::VocAlgorithm::new();

        let mut value = 0;

        for _ in 1..10000 {
            value = voc.process(20_010);
            let ref_value = ref_voc.process(20_010);

            assert!((value - ref_value).abs() <= epsilon);
        }

        assert_eq!(500, value);
    }
    */
}
