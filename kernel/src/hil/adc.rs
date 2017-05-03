use returncode::ReturnCode;

/// Trait for handling callbacks from ADC module.
pub trait Client {
    /// Called when a sample is ready. Used for single sampling
    fn sample_done(&self, sample: u16);

    /// Called when the buffer is full. Used for continuous sampling
    /// Expects an additional call to either continue sampling or stop
    fn buffer_ready(&self, buf: &'static mut [u16], length: usize);
}

/// Trait representing a channel on the ADC.
/// No functions needed. The chip driver will know how to handle this.
pub trait ADCChannel {
}

/// Simple interface for reading a single ADC sample on any channel.
pub trait ADCSingle {
    type Channel: ADCChannel;

    /// Initialize must be called before taking a sample.
    /// Returns true on success.
    fn initialize(&self) -> ReturnCode;

    /// Request a single ADC sample on a particular channel.
    /// Returns true on success.
    fn sample(&self, channel: &Self::Channel) -> ReturnCode;
}

/// Interface for continuously sampling at a given frequency on a channel.
pub trait ADCContinuous {
    type Channel: ADCChannel;

    /// Start sampling continuously.
    /// Samples are collected into the given buffer.
    fn sample_continuous(&self, channel: &Self::Channel, frequency: u32, buf: &'static mut [u16]) -> ReturnCode;

    /// Continue previous `sample_continuous` configuration with a new buffer.
    /// Expected to be called after a `buffer_ready` callback.
    /// Note that if this is not called quickly enough, it is possible to drop samples.
    fn continue_sampling(&self, buf: &'static mut [u16]) -> ReturnCode;

    /// Stop continuous sampling.
    /// Can be called at any time to cancel sampling, triggering a `buffer_ready` callback.
    /// However, the normal expectation is to call this during a `buffer_ready` callback in order
    /// to stop the ADC, which will not trigger a second `buffer_ready` callback.
    fn stop_sampling(&self) -> ReturnCode;
}
