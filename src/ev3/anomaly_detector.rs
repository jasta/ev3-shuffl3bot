use std::ops::Range;

pub struct AnomalyDetector<T> {
    pub sane_range: Range<T>,
    pub max_per_sample_delta: T,
    pub last_sane_value: Option<T>,
    pub recent_outliers: usize,
}

impl AnomalyDetector<i32> {
    pub fn update(&mut self, value: i32) -> Result<(), ()> {
        if !self.is_outlier(value) {
            self.last_sane_value = Some(value);
            self.recent_outliers = 0;
            Ok(())
        } else {
            self.recent_outliers += 1;
            Err(())
        }
    }

    fn is_outlier(&self, value: i32) -> bool {
        if let Some(last_sane_value) = self.last_sane_value {
            let delta = (value - last_sane_value).abs();
            if delta > self.max_per_sample_delta {
                return true;
            }
        }
        !self.sane_range.contains(&value)
    }
}