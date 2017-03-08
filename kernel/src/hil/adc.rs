use returncode::ReturnCode;

/// Trait for handling callbacks from ADC module.
pub trait Client {
    /// Called when a sample is ready. Used for single sampling
    fn sample_done(&self, sample: u16);

    ///// Called when the buffer is full. Used for continuous sampling
    ///// Expects another buffer to be provided
    //fn buffer_full(&self, buf &'static [u8]) -> &'static [u8];

    ///// Called after sampling has been canceled. Used for continuous sampling
    //fn sampling_complete(&self, buf &'static [u8]);
}

/// Simple interface for reading a single ADC sample on any channel.
pub trait AdcSingle {
    /// Initialize must be called before taking a sample.
    /// Returns true on success.
    fn initialize(&self) -> ReturnCode;

    /// Request a single ADC sample on a particular channel.
    /// Returns true on success.
    fn sample(&self, channel: u8) -> ReturnCode;
    fn cancel_sample(&self) -> ReturnCode;
}

/// Interface for continuously sampling at a given frequency on a channel.
pub trait AdcContinuous {

    /// Start sampling continuously.
    /// Samples are collected into the given buffer
    fn sample_continuous(&self, channel: u8, frequency: u32, buf: &'static [u8]) -> ReturnCode;

    /// Stop continuous sampling.
    fn cancel_sampling(&self) -> ReturnCode;
}
