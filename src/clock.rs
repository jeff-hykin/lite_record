//! Putting a device's own hardware clock onto the host's wall clock.
//!
//! Every sensor here stamps in its own time base, and every one of those bases
//! is worth keeping: the device's spacing is exact where arrival times are
//! jittered by USB, ethernet and scheduling. So a stamp is the hardware value
//! plus one offset onto the host clock, never the arrival time and never the
//! raw device value.
//!
//! The offset cannot be measured once and kept. A Raspberry Pi has no clock of
//! its own across a power cycle: it boots at whatever time the last shutdown
//! left behind, and NTP steps it — by half an hour, in the recording that
//! exposed this — as soon as the network is up. An offset locked before that
//! step leaves every stamp behind by its size for the life of the process,
//! which is exactly what happened to the Livox stream in the grocery recording
//! while the RealSense streams, which re-estimate, followed the step.

const CLOCK_WINDOW_SECONDS: u64 = 30;

/// How far the observed offset must jump above the estimate before a step is
/// even considered. Delivery jitter on these links is milliseconds; a clock
/// correction is seconds. Nothing in between happens.
const STEP_THRESHOLD_NANOS: i128 = 1_000_000_000;

/// How many consecutive samples must agree that the clock moved. A burst of
/// late deliveries can lift one reading; it cannot hold every reading up.
const STEP_CONFIRM_SAMPLES: u32 = 8;

/// Puts a device's hardware stamp onto the host clock.
///
/// Delivery latency can only ever make a sample arrive later than it was taken,
/// so the smallest offset seen recently is the closest estimate of the true one.
/// Keeping one minimum per second lets the estimate follow the device's slow
/// drift without rescanning every sample.
#[derive(Default)]
pub struct HostClock {
    per_second: std::collections::VecDeque<(u64, i128)>,
    offset: i128,
    /// Consecutive samples that have come in far above the estimate, and the
    /// smallest of them. A window minimum follows a clock that jumps *back*
    /// immediately, because the new readings are smaller and win; a clock that
    /// jumps *forward* would otherwise be ignored until every pre-step sample
    /// had aged out, leaving up to a window of stamps behind.
    stepped_samples: u32,
    stepped_offset: i128,
}

impl HostClock {
    pub fn host_nanos(&mut self, device_nanos: u64) -> u64 {
        self.map(device_nanos, crate::record::now_nanos())
    }

    /// Split out from `host_nanos` so a test can drive the host clock instead of
    /// racing the real one across a second boundary.
    /// The offset the estimate currently stands at, for a caller that has to
    /// apply it to values it holds itself rather than one at a time.
    pub fn offset_nanos(&self) -> i64 {
        self.offset as i64
    }

    /// The device stamp on the host clock, using the estimate as it stands.
    /// `map` is what advances the estimate; this only reads it.
    ///
    /// The device's own spacing is preserved exactly, because the same offset
    /// is added to every stamp: two samples 33.333333 ms apart on the device
    /// come out 33.333333 ms apart in their headers.
    pub fn epoch_for(&self, device_nanos: u64) -> u64 {
        (device_nanos as i128 + self.offset).max(0) as u64
    }

    pub fn map(&mut self, device_nanos: u64, now: u64) -> u64 {
        let offset = now as i128 - device_nanos as i128;
        let second = now / 1_000_000_000;

        // A sustained jump means the host clock moved, and everything measured
        // before it describes a clock that no longer exists.
        if !self.per_second.is_empty() && offset > self.offset + STEP_THRESHOLD_NANOS {
            self.stepped_offset = match self.stepped_samples {
                0 => offset,
                _ => self.stepped_offset.min(offset),
            };
            self.stepped_samples += 1;
            if self.stepped_samples >= STEP_CONFIRM_SAMPLES {
                self.per_second.clear();
                self.per_second.push_back((second, self.stepped_offset));
                self.offset = self.stepped_offset;
                self.stepped_samples = 0;
                return (device_nanos as i128 + self.offset).max(0) as u64;
            }
        } else {
            self.stepped_samples = 0;
        }

        match self.per_second.back_mut() {
            // A better estimate is taken immediately rather than at the next
            // rollover, so a slow first sample cannot hold every stamp late for
            // a whole second. The window minimum can only fall when a bucket's
            // does, so this stays a comparison rather than a rescan.
            Some(newest) if newest.0 == second => {
                if offset < newest.1 {
                    newest.1 = offset;
                    self.offset = self.offset.min(offset);
                }
            }
            _ => {
                self.per_second.push_back((second, offset));
                while self
                    .per_second
                    .front()
                    .is_some_and(|(stamp, _)| stamp + CLOCK_WINDOW_SECONDS < second)
                {
                    self.per_second.pop_front();
                }
                self.offset = self
                    .per_second
                    .iter()
                    .map(|(_, offset)| *offset)
                    .min()
                    .unwrap_or(offset);
            }
        }
        (device_nanos as i128 + self.offset).max(0) as u64
    }
}

#[cfg(test)]
mod clock_tests {
    use super::{HostClock, STEP_CONFIRM_SAMPLES};

    const EPOCH: u64 = 1_788_000_000_000_000_000;
    const STEP: u64 = 5_000_000;

    /// The whole reason for keeping the hardware clock is that its spacing is
    /// exact. Ten seconds of a perfectly even 200 Hz has to come back out as a
    /// perfectly even 200 Hz however jittery the arrivals were — this is what
    /// librealsense's own fit got wrong, reporting the gyro as 191 Hz.
    #[test]
    fn even_device_spacing_survives_jittery_arrival() {
        let mut clock = HostClock::default();
        let stamps: Vec<u64> = (0..2000)
            .map(|index| {
                // A burst of four arriving together, then a 20 ms stall, which
                // is roughly what a busy USB bus does.
                let jitter = if index % 4 == 0 { 20_000_000 } else { 0 };
                clock.map(index * STEP, EPOCH + index * STEP + jitter)
            })
            .collect();

        // The first sample is the jittered one, so it is the offset estimate's
        // warm-up; from the second onwards the estimate has the true minimum and
        // never moves again, including across the ten second boundaries here.
        let gaps: Vec<u64> = stamps[1..].windows(2).map(|pair| pair[1] - pair[0]).collect();
        assert_eq!(gaps.len(), 1998);
        assert!(
            gaps.iter().all(|gap| *gap == STEP),
            "spacing was not preserved: {:?}",
            &gaps[..8]
        );
    }

    /// A device stamp counts from the camera powering on, so without
    /// re-anchoring every message would claim to be from 1970.
    #[test]
    fn device_uptime_lands_on_the_host_epoch() {
        let mut clock = HostClock::default();
        let stamp = clock.map(90_000_000_000, EPOCH);
        assert_eq!(stamp, EPOCH);
    }

    /// Arrival latency only ever pushes a sample later, so one sample that took
    /// an unusually long time to arrive must not drag every later stamp with it.
    #[test]
    fn a_single_late_arrival_does_not_shift_the_series() {
        let mut clock = HostClock::default();
        clock.map(0, EPOCH + 500_000_000);
        let after_a_stall = clock.map(STEP, EPOCH + STEP);
        assert_eq!(after_a_stall, EPOCH + STEP);
    }

    /// The bug this module exists for. The host clock is half an hour slow when
    /// the lidar's first packet arrives, and NTP corrects it a few seconds
    /// later. An offset measured once would leave every later stamp half an
    /// hour behind; the window has to follow the step.
    #[test]
    fn a_wall_clock_step_is_followed_rather_than_carried_for_ever() {
        let mut clock = HostClock::default();
        let wrong_by = 2_005_000_000_000u64;

        // Five seconds of 10 Hz packets while the host clock is still wrong.
        for index in 0..50u64 {
            let device = index * 100_000_000;
            clock.map(device, EPOCH - wrong_by + device);
        }
        let before = clock.map(50 * 100_000_000, EPOCH - wrong_by + 50 * 100_000_000);
        assert_eq!(before, EPOCH - wrong_by + 50 * 100_000_000);

        // NTP steps the clock forward. Every packet after it must land on the
        // corrected clock, not the one the first packet happened to see.
        let mut last = 0;
        for index in 51..400u64 {
            let device = index * 100_000_000;
            last = clock.map(device, EPOCH + device);
        }
        assert_eq!(
            last,
            EPOCH + 399 * 100_000_000,
            "the estimate is still {} s behind",
            (EPOCH + 399 * 100_000_000 - last) as f64 / 1e9
        );
    }

    /// The device clock need not start anywhere in particular — on some hosts
    /// the monotonic clock is an uptime counter that has already passed the
    /// epoch value — and the spacing has to survive the offset either way.
    #[test]
    fn an_arbitrary_device_base_still_lands_on_the_epoch_with_its_spacing_intact() {
        let mut clock = HostClock::default();
        let device = 2_000_000_000_000_000_000u64;
        clock.map(device, EPOCH);
        assert_eq!(clock.epoch_for(device + 1_000_000_000), EPOCH + 1_000_000_000);
        assert_eq!(
            clock.epoch_for(device + 133_333_333) - clock.epoch_for(device + 100_000_000),
            33_333_333
        );
    }

    /// A forward step is the slow case: every reading after it is *larger*
    /// than the estimate, so a window minimum keeps the pre-step value until it
    /// ages out. That left up to 30 s of stamps behind, which a mid-recording
    /// correction on the Pi reproduced exactly. A sustained jump is adopted
    /// within a handful of samples instead.
    #[test]
    fn a_forward_step_is_adopted_in_samples_rather_than_a_whole_window() {
        let mut clock = HostClock::default();
        let jump = 120 * 1_000_000_000u64;
        for index in 0..200u64 {
            clock.map(index * STEP, EPOCH + index * STEP);
        }
        let mut behind = Vec::new();
        for index in 200..260u64 {
            let device = index * STEP;
            let mapped = clock.map(device, EPOCH + jump + device);
            behind.push((EPOCH + jump + device) as i64 - mapped as i64);
        }
        let settled = behind.iter().position(|error| error.abs() < 1_000_000).unwrap_or(usize::MAX);
        assert!(
            settled <= STEP_CONFIRM_SAMPLES as usize + 1,
            "took {settled} samples to follow the step, first errors {:?}",
            &behind[..8]
        );
        assert_eq!(behind.last().copied(), Some(0));
    }

    /// And the guard has to hold: a burst of late arrivals, however long, is
    /// not a clock step and must not be adopted as one.
    #[test]
    fn a_run_of_late_arrivals_is_not_mistaken_for_a_step() {
        let mut clock = HostClock::default();
        for index in 0..100u64 {
            clock.map(index * STEP, EPOCH + index * STEP);
        }
        // 40 ms late, every sample, for far longer than the confirmation count.
        let mut mapped = 0;
        for index in 100..160u64 {
            let device = index * STEP;
            mapped = clock.map(device, EPOCH + device + 40_000_000);
        }
        assert_eq!(mapped, EPOCH + 159 * STEP, "latency leaked into the stamps");
    }
}
