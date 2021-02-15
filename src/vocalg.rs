const SAMPLING_INTERVAL:f32 = 1.;
const INITIAL_BLACKOUT:f32 = 1.;
//const INITIAL_BLACKOUT:f32 = 45.0;
const VOC_INDEX_GAIN:f32 = 230.;
const SRAW_STD_INITIAL:f32 = 50.;
const SRAW_STD_BONUS:f32 = 220.;
const TAU_MEAN_VARIANCE_HOURS:f32 = 12.;
const TAU_INITIAL_MEAN:f32 = 20.;
const INIT_DURATION_MEAN:f32 = 3600. * 0.75;
const INIT_TRANSITION_MEAN:f32 = 0.01;
const TAU_INITIAL_VARIANCE:f32 = 2500.;
const INIT_DURATION_VARIANCE:f32 = 3600. * 1.45;
const INIT_TRANSITION_VARIANCE:f32 = 0.01;
const GATING_THRESHOLD:f32 = 340.;
const GATING_THRESHOLD_INITIAL:f32 = 510.;
const GATING_THRESHOLD_TRANSITION:f32 = 0.09;
const GATING_MAX_DURATION_MINUTES:f32 = 60.0 * 3.0;
const GATING_MAX_RATIO:f32 = 0.3;
const SIGMOID_L:f32 = 500.;
const SIGMOID_K:f32 = -0.0065;
const SIGMOID_X0:f32 = 213.;
const VOC_INDEX_OFFSET_DEFAULT:f32 = 100.;
const LP_TAU_FAST:f32 = 20.;
const LP_TAU_SLOW: f32 = 500.;
const LP_ALPHA: f32 = -0.2;
const PERSISTENCE_UPTIME_GAMMA: f32 = 3.0 * 3600.0;
const MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING: f32 = 64.;
const MEAN_VARIANCE_ESTIMATOR__FIX16_MAX: f32 = 32767.;

/**
 * Struct to hold all the states of the VOC algorithm.
 */
#[derive(Debug)]
pub struct VocAlgorithm {
    voc_index_offset: f32,
    tau_mean_variance_hours: f32,
    gating_max_duration_minutes: f32,
    sraw_std_initial: f32,
    uptime: f32,
    sraw: f32,
    voc_index: f32,

    // Initialize substructures
    mean_variance_estimator: MeanVarianceEstimator,
    mox_model: MoxModel,
    sigmoid_scaled: SigmoidScaledInit,
    adaptive_lowpass: AdaptiveLowpass
}

impl VocAlgorithm {
    pub fn new() -> Self {

        let (mean_variance_estimator, mox_model, sigmoid_scaled, adaptive_lowpass) =
            VocAlgorithm::new_instances(
                SRAW_STD_INITIAL,
                TAU_MEAN_VARIANCE_HOURS,
                GATING_MAX_DURATION_MINUTES,
                VOC_INDEX_OFFSET_DEFAULT);

        let alg = VocAlgorithm {
            voc_index_offset: VOC_INDEX_OFFSET_DEFAULT,
            tau_mean_variance_hours: TAU_MEAN_VARIANCE_HOURS,
            gating_max_duration_minutes: GATING_MAX_DURATION_MINUTES,
            sraw_std_initial: SRAW_STD_INITIAL,
            uptime: 0.0,
            sraw: 0.0,
            voc_index: 0.0,

            mean_variance_estimator,
            mox_model,
            sigmoid_scaled,
            adaptive_lowpass
        };

        println!("Alg {:?}", alg);
        alg
    }

    fn new_instances(sraw_std_initial: f32, tau_mean_variance_hours: f32, gating_max_duration_minutes: f32, voc_index_offset: f32) -> (MeanVarianceEstimator, MoxModel, SigmoidScaledInit, AdaptiveLowpass) {
        let mut mean_variance_estimator = MeanVarianceEstimator::new();
        mean_variance_estimator.set_parameters(sraw_std_initial, tau_mean_variance_hours, gating_max_duration_minutes);

        let mox_model = MoxModel::new(mean_variance_estimator.get_std(), mean_variance_estimator.get_mean());
        //mox_model.set_parameters();

        let sigmoid_scaled = SigmoidScaledInit::new(voc_index_offset);

        let adaptive_lowpass = AdaptiveLowpass::new();

        (mean_variance_estimator, mox_model, sigmoid_scaled, adaptive_lowpass)
    }

    /// Returns state0, state1
    pub fn get_states(&self) -> (i32, i32) {
        (self.mean_variance_estimator.get_mean() as i32, self.mean_variance_estimator.get_std() as i32)
    }

    pub fn set_states(&mut self, state0: i32, state1: i32) {
        self.mean_variance_estimator.set_states(state0 as f32, state1 as f32, PERSISTENCE_UPTIME_GAMMA);
        self.sraw = state0 as f32;
    }

    pub fn set_tuning_parameters(&mut self, voc_index_offset: i32, learning_time_hours: i32, gating_duration_minutes: i32, std_initial: i32) {
        let (mean_variance_estimator, mox_model, sigmoid_scaled, adaptive_lowpass) =
            VocAlgorithm::new_instances(
                std_initial as f32,
                learning_time_hours as f32,
                gating_duration_minutes as f32,
                voc_index_offset as f32
            );

            self.mean_variance_estimator = mean_variance_estimator;
            self.mox_model = mox_model;
            self.sigmoid_scaled = sigmoid_scaled;
            self.adaptive_lowpass = adaptive_lowpass;
    }

    pub fn process(&mut self, sraw: i32) -> i32 {
        if  self.uptime <= INITIAL_BLACKOUT {
            self.uptime += SAMPLING_INTERVAL;
            println!("Initial blackout");
        }
        else {
            assert!(sraw > 0 && sraw < 65000);

            let sraw = if sraw < 20001 {
                20001
            }
            else if sraw > 52767 {
                52767
            }
            else {
                sraw
            };

            self.sraw = (sraw - 20000) as f32;

            println!("SRAW: {}", self.sraw);

            self.voc_index = self.mox_model.process(self.sraw);
            println!("After Mox: {}", self.voc_index);

            self.voc_index = self.sigmoid_scaled.process(self.voc_index);
            println!("After Sigmoid scaled: {}", self.voc_index);

            self.voc_index = self.adaptive_lowpass.process(self.voc_index);
            println!("After Adaptive lowpass: {}", self.voc_index);

            if self.voc_index < 0.5 {
                self.voc_index = 0.5;
            }

            if self.sraw > 0.0 {
                self.mean_variance_estimator.process(self.sraw, self.voc_index);
                println!("Est std:{} mean:{}", self.mean_variance_estimator.get_std(),self.mean_variance_estimator.get_mean());
                self.mox_model = MoxModel::new(self.mean_variance_estimator.get_std(), self.mean_variance_estimator.get_mean());
            }
        }
        (self.voc_index + 0.5) as i32
    }
}

#[derive(Debug)]
struct MeanVarianceEstimator {
    gating_max_duration_minutes: f32,
    mean: f32,
    sraw_offset: f32,
    std: f32,
    gamma: f32,
    gamma_initial_mean: f32,
    gamma_initial_variance: f32,
    gamma_mean: f32,
    gamma_variance: f32,
    uptime_gamma: f32,
    uptime_gating: f32,
    gating_duration_minutes: f32,
    sigmoid: MeanVarianceEstimatorSigmoid,
    initialized: bool
}

impl MeanVarianceEstimator {
    fn new() -> Self {
        MeanVarianceEstimator {
            gating_max_duration_minutes: 0.,
            mean: 0.,
            sraw_offset: 0.,
            std: 0.,
            gamma: 0.,
            gamma_initial_mean: 0.,
            gamma_initial_variance: 0.,
            gamma_mean: 0.,
            gamma_variance: 0.,
            uptime_gamma: 0.,
            uptime_gating: 0.,
            gating_duration_minutes: 0.,
            sigmoid: MeanVarianceEstimatorSigmoid::new(),
            initialized: false
        }
    }

    fn set_parameters(&mut self, std_initial: f32, tau_mean_variance_hours: f32, gating_max_duration_minutes: f32) {
        self.gating_max_duration_minutes = gating_max_duration_minutes;
        self.initialized = false;
        self.mean = 0.;
        self.sraw_offset = 0.;
        self.std = std_initial;
        self.gamma = (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * (SAMPLING_INTERVAL / 3600.)) /
            (tau_mean_variance_hours + SAMPLING_INTERVAL / 3600.);
        self.gamma_initial_mean = (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * SAMPLING_INTERVAL) /
            (TAU_INITIAL_MEAN + SAMPLING_INTERVAL);
        self.gamma_initial_variance = (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * SAMPLING_INTERVAL) /
            (TAU_INITIAL_VARIANCE + SAMPLING_INTERVAL);
        self.gamma_mean     = 0.;
        self.gamma_variance = 0.;
        self.uptime_gamma   = 0.;
        self.uptime_gating  = 0.;
        self.gating_duration_minutes = 0.;
    }

    fn set_states(&mut self, mean: f32, std: f32, uptime_gamma: f32) {
        self.mean = mean;
        self.std = std;
        self.uptime_gamma = uptime_gamma;
        self.initialized = true;
    }

    fn get_std(&self) -> f32 {
        self.std
    }

    fn get_mean(&self) -> f32 {
        self.mean + self.sraw_offset
    }

    fn calculate_gamma(&mut self, voc_index_from_prior: f32) {
        // Check this as we are likely running in 32-bit environment
        const uptime_limit:f32 = MEAN_VARIANCE_ESTIMATOR__FIX16_MAX - SAMPLING_INTERVAL;

        //println!("Updatime gamma:{}", self.uptime_gamma);

        if self.uptime_gamma < uptime_limit {
            self.uptime_gamma += SAMPLING_INTERVAL;
        }

        if self.uptime_gating < uptime_limit {
            self.uptime_gating += SAMPLING_INTERVAL;
        }

        self.sigmoid.set_parameters(1., INIT_DURATION_MEAN, INIT_TRANSITION_MEAN);

        let sigmoid_gamma_mean = self.sigmoid.process(self.uptime_gamma);

        let gamma_mean = self.gamma + ((self.gamma_initial_mean - self.gamma) * sigmoid_gamma_mean);

        let gating_threshold_mean = GATING_THRESHOLD + (GATING_THRESHOLD_INITIAL - GATING_THRESHOLD) *
            self.sigmoid.process(self.uptime_gating);

        self.sigmoid.set_parameters(1., gating_threshold_mean, GATING_THRESHOLD_TRANSITION);

        let sigmoid_gating_mean = self.sigmoid.process(voc_index_from_prior);

        self.gamma_mean = sigmoid_gating_mean * gamma_mean;

        self.sigmoid.set_parameters(1.,  INIT_DURATION_VARIANCE, INIT_TRANSITION_VARIANCE);

        let sigmoid_gamma_variance = self.sigmoid.process(self.uptime_gamma);

        let gamma_variance = self.gamma + (self.gamma_initial_variance - self.gamma) * (sigmoid_gamma_variance - sigmoid_gamma_mean);

        let gating_threshold_variance = GATING_THRESHOLD + (GATING_THRESHOLD_INITIAL - GATING_THRESHOLD) * self.sigmoid.process(self.uptime_gating);

        self.sigmoid.set_parameters(1., gating_threshold_variance, GATING_THRESHOLD_TRANSITION);

        let sigmoid_gating_variance = self.sigmoid.process(voc_index_from_prior);

        self.gamma_variance = sigmoid_gating_variance * gamma_variance;

        self.gating_duration_minutes += (SAMPLING_INTERVAL / 60.) * (((1. - sigmoid_gating_mean) * (1. + GATING_MAX_RATIO)) - GATING_MAX_RATIO);

        if self.gating_duration_minutes < 0. {
            self.gating_duration_minutes = 0.;
        }

        if self.gating_duration_minutes > self.gating_max_duration_minutes {
            self.uptime_gating = 0.0;
        }
    }

    fn process(&mut self, sraw: f32, voc_index_from_prior: f32) {
        if self.initialized == false {
            self.initialized = true;
            self.sraw_offset = sraw;
            self.mean = 0.;
        }
        else {
            if self.mean >= 100. || self.mean <= -100. {
                self.sraw_offset += self.mean;
                self.mean = 0.;
                println!("Mean reset");
            }

            let sraw = sraw - self.sraw_offset;

            self.calculate_gamma(voc_index_from_prior);

            println!("Gamma variance:{}", self.gamma_variance);

            let delta_sgp = (sraw - self.mean) / MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING;

            let c = self.std + delta_sgp.abs();

            let additional_scaling = if c > 1440. {
                4.
            }
            else {
                1.
            };

            self.std = (additional_scaling * (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING -  self.gamma_variance)).sqrt() *
                (
                    (self.std * (self.std / (MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * additional_scaling))) +
                    (((self.gamma_variance * delta_sgp) / additional_scaling) * delta_sgp)
                ).sqrt();

            self.mean += self.gamma_mean * delta_sgp;

            println!("Final mean:{} std:{}", self.mean, self.std);
        }
    }

}

#[derive(Debug)]
struct MeanVarianceEstimatorSigmoid {
    L: f32,
    K: f32,
    X0: f32,
}


impl MeanVarianceEstimatorSigmoid {
    fn new() -> Self {
        MeanVarianceEstimatorSigmoid {
            L: 0.,
            K: 0.,
            X0: 0.,
        }
    }

    fn process(&self, sample: f32) -> f32 {
        let x = self.K * (sample - self.X0);

        //println!("Sigmoid: sample:{} x:{}", sample, x);

        if x < -50. {
            return self.L;
        }
        else if x > 50. {
            return 0.
        }
        else {
            let result = self.L / (1. + fixed_exp(x));
            //println!("Sigmoid:{}", result);
            return result;
        }
    }


    fn set_parameters(&mut self, L: f32, K: f32, X0: f32) {
        self.L = L;
        self.K = K;
        self.X0 = X0;
    }
}


// Needs new implementation as the original code prevents going above 16-bit ranges
fn fixed_exp(x: f32) -> f32 {
    let result = x.exp();
    if result > 32768.0 {
        32768.0
    }
    else {
        result
    }
}


#[derive(Debug)]
struct SigmoidScaledInit {
    offset: f32
}

impl SigmoidScaledInit {
    fn new(offset: f32) -> Self {
        SigmoidScaledInit {
            offset
        }
    }


    fn process(&self, sample: f32) -> f32 {
        let x = SIGMOID_K * (sample - SIGMOID_X0);

        if x < -50. {
            return SIGMOID_L;
        }
        else if x > 50. {
            return 0.
        }
        else {
            //println!("Sample {}, offset:{} X:{}", sample, self.offset, x);
            if sample >= 0. {
                let shift = (SIGMOID_L - (5. * self.offset)) / 4.;
                return ((SIGMOID_L + shift) / (1. + fixed_exp(x))) - shift;
            }
            else {
                //println!("X^{}", SigmoidScaledInit::exp(x));
                return (self.offset / VOC_INDEX_OFFSET_DEFAULT) * (SIGMOID_L / (1. + fixed_exp(x)));
            }
        }
    }

}

#[derive(Debug)]
struct MoxModel {
    sraw_std: f32,
    sraw_mean: f32
}

impl MoxModel {
    fn new(sraw_std: f32, sraw_mean: f32) -> Self {
        MoxModel {
            sraw_std,
            sraw_mean,
        }
    }

    fn process(&self, sraw: f32) -> f32 {
        ((sraw - self.sraw_mean) / (-(self.sraw_std + SRAW_STD_BONUS))) * VOC_INDEX_GAIN
    }
}

#[derive(Debug)]
struct AdaptiveLowpass {
    A1: f32,
    A2: f32,
    X1: f32,
    X2: f32,
    X3: f32,
    initialized: bool
}

impl AdaptiveLowpass {
    fn new() -> Self {
        AdaptiveLowpass {
            A1: SAMPLING_INTERVAL / (LP_TAU_FAST + SAMPLING_INTERVAL),
            A2: SAMPLING_INTERVAL / (LP_TAU_SLOW + SAMPLING_INTERVAL),
            initialized: false,
            X1: 0.,
            X2: 0.,
            X3: 0.
        }
    }

    fn process(&mut self, sample: f32) -> f32 {
        if self.initialized == false {
            self.X1 = sample;
            self.X2 = sample;
            self.X3 = sample;
            self.initialized = true;
        }

        self.X1 = (1. - self.A1) * self.X1 + self.A1*sample;
        self.X2 = (1. - self.A2) * self.X2 + self.A2*sample;

        let abs_delta = (self.X1 - self.X2).abs();
        let F1 = fixed_exp(LP_ALPHA * abs_delta);
        let tau_a = ((LP_TAU_SLOW - LP_TAU_FAST) * F1) + LP_TAU_FAST;
        let a3 = SAMPLING_INTERVAL / (SAMPLING_INTERVAL + tau_a);
        self.X3 = (1. - a3) * self.X3 + a3 * sample;
        self.X3
    }
}