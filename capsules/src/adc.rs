//! ADC Capsule
//!
//! Provides userspace applications with the ability to sample
//! analog signals.

use core::cell::Cell;
use kernel::{AppId, AppSlice, Callback, Container, Driver, ReturnCode, Shared};
use kernel::common::take_cell::{MapCell, TakeCell};
use kernel::hil;

pub struct ADC<'a, A: hil::adc::ADCSingle + hil::adc::ADCContinuous + 'a> {
    // ADC driver
    adc: &'a A,
    channels: &'a [&'a <A as hil::adc::ADCSingle>::Channel],

    // ADC state
    app_state: MapCell<AppState>,
    active: Cell<bool>,

    // ADC buffers
    adc_buf1: TakeCell<'static, [u16]>,
    adc_buf2: TakeCell<'static, [u16]>,
}

enum CallbackType {
    SingleSample = 0,
    SampleBuffer = 1,
    ContinuousSample = 2,
}

pub struct AppState {
    channel: Option<usize>,
    callback: Option<Callback>,
    sample_buffer1: Option<AppSlice<Shared, u8>>,
    sample_buffer2: Option<AppSlice<Shared, u8>>,
}

pub static mut ADC_BUFFER1: [u16; 128] = [0; 128];
pub static mut ADC_BUFFER2: [u16; 128] = [0; 128];

impl<'a, A: hil::adc::ADCSingle + hil::adc::ADCContinuous + 'a> ADC<'a, A> {
    pub fn new(adc: &'a A, channels: &'a [&'a <A as hil::adc::ADCSingle>::Channel], adc_buf1: &'static mut [u16; 128], adc_buf2: &'static mut [u16; 128]) -> ADC<'a, A> {
        ADC {
            // ADC driver
            adc: adc,
            channels: channels,

            // ADC state
            app_state: MapCell::new(AppState {
                channel: None,
                callback: None,
                sample_buffer1: None,
                sample_buffer2: None,
            }),
            active: Cell::new(false),

            // ADC buffers
            adc_buf1: TakeCell::new(adc_buf1),
            adc_buf2: TakeCell::new(adc_buf2),
        }
    }

    fn initialize(&self) -> ReturnCode {
        self.adc.initialize()
    }

    fn sample(&self, channel: usize, appid: AppId) -> ReturnCode {

        // only one sample at a time
        if self.active.get() {
            return ReturnCode::EBUSY;
        }

        // always initialize. Initialization will be skipped if already complete
        let res = self.initialize();
        if res != ReturnCode::SUCCESS {
            return res;
        }

        // convert channel index
        if channel > self.channels.len() {
            return ReturnCode::EINVAL;
        }
        let chan = self.channels[channel];

        // start a single sample
        let res = self.adc.sample(chan);
        if res != ReturnCode::SUCCESS {
            return res;
        }

        // save state for callback
        self.active.set(true);
        self.app_state.map(|state| {
            state.channel = Some(channel);
        });

        ReturnCode::SUCCESS
    }

    fn sample_buffer (&self, channel: usize, frequency: u32, appid: AppId) -> ReturnCode {

        // only one sample at a time
        if self.active.get() {
            return ReturnCode::EBUSY;
        }

        // always initialize. Initialization will be skipped if already complete
        let res = self.initialize();
        if res != ReturnCode::SUCCESS {
            return res;
        }

        // convert channel index
        if channel > self.channels.len() {
            return ReturnCode::EINVAL;
        }
        let chan = self.channels[channel];

        // cannot do sample a buffer without a buffer to sample into
        let exists = self.app_state.map_or(false, |state| {
            state.sample_buffer1.is_some()
        });
        if !exists {
            return ReturnCode::ENOMEM;
        }

        // start a continuous sample
        let res = self.adc_buf1.take().map_or(ReturnCode::EBUSY, |buf| {
            self.adc.sample_continuous(chan, frequency, buf)
        });
        if res != ReturnCode::SUCCESS {
            return res;
        }

        // save state for callback
        self.active.set(true);
        self.app_state.map(|state| {
            state.channel = Some(channel);
        });

        ReturnCode::SUCCESS
    }

    fn sample_continuous (&self, channel: usize, frequency: u32, appid: AppId) -> ReturnCode {

        panic!("NOPE");

        // only one sample at a time
        if self.active.get() {
            return ReturnCode::EBUSY;
        }

        // always initialize. Initialization will be skipped if already complete
        let res = self.initialize();
        if res != ReturnCode::SUCCESS {
            return res;
        }

        // convert channel index
        if channel > self.channels.len() {
            return ReturnCode::EINVAL;
        }
        let chan = self.channels[channel];

        // start a continuous sample
        // demand that both appslices have already been set

        // when the appslice is full, signal the user
        // type - continuous, channel, buffer pointer

        // switch to filling the second appslice

        ReturnCode::SUCCESS
    }
}

impl<'a, A: hil::adc::ADCSingle + hil::adc::ADCContinuous + 'a> hil::adc::Client for ADC<'a, A> {
    fn sample_done(&self, sample: u16) {
        self.active.set(false);

        self.app_state.map(|state| {
            state.callback.map(|mut callback| {
                let channel = state.channel.unwrap_or(0);
                callback.schedule(CallbackType::SingleSample as usize, channel, sample as usize);
            });
        });
    }

    fn buffer_ready(&self, buf: &'static mut [u16], length: usize) {
        panic!("not yet {:?}", buf);
        //debug!("Got {} bytes of data: \n{:?}", length, buf);
        //self.adc.continue_sampling(buf);

    }
}

impl<'a, A: hil::adc::ADCSingle + hil::adc::ADCContinuous + 'a> Driver for ADC<'a, A> {
    fn allow(&self, _appid: AppId, allow_num: usize, slice: AppSlice<Shared, u8>) -> ReturnCode {
        match allow_num {
            // Pass buffer for samples to go into
            0 => {
                // set first buffer
                self.app_state.map(|state| {
                    state.sample_buffer1 = Some(slice);
                });

                ReturnCode::SUCCESS
            }

            // Pass a second buffer to be used for double-buffered continuous sampling
            1 => {
                // set second buffer
                self.app_state.map(|state| {
                    state.sample_buffer2 = Some(slice);
                });

                ReturnCode::SUCCESS
            }

            // default
            _ => ReturnCode::ENOSUPPORT,
        }
    }

    fn subscribe(&self, subscribe_num: usize, callback: Callback) -> ReturnCode {
        match subscribe_num {
            // subscribe to ADC sample done (from all types of sampling)
            0 => {
                // set callback
                self.app_state.map(|state| {
                    state.callback = Some(callback);
                });

                ReturnCode::SUCCESS
            }

            // default
            _ => ReturnCode::ENOSUPPORT,
        }
    }

    fn command(&self, command_num: usize, data: usize, appid: AppId) -> ReturnCode {
        match command_num {
            // check if present
            0 => {
                ReturnCode::SuccessWithValue { value: self.channels.len() as usize }
            },

            // Single sample on channel
            1 => {
                let channel = (data & 0xFF) as usize;
                self.sample(channel, appid)
            },

            // Multiple sample on a channel
            2 => {
                let channel = (data & 0xFF) as usize;
                let frequency = (data >> 8) as u32;
                self.sample_buffer(channel, frequency, appid)
            },

            // Continuous sample on a channel
            3 => {
                let channel = (data & 0xFF) as usize;
                let frequency = (data >> 8) as u32;
                self.sample_continuous(channel, frequency, appid)
            },

            // default
            _ => ReturnCode::ENOSUPPORT,
        }
    }
}
