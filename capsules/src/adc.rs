//! ADC Capsule
//!
//! Provides userspace applications with the ability to sample
//! analog signals.

use core::cell::Cell;
use core::cmp;
use kernel::{AppId, AppSlice, Callback, Driver, ReturnCode, Shared};
use kernel::common::take_cell::{MapCell, TakeCell};
use kernel::hil;

pub struct ADC<'a, A: hil::adc::ADCSingle + hil::adc::ADCContinuous + 'a> {
    // ADC driver
    adc: &'a A,
    channels: &'a [&'a <A as hil::adc::ADCSingle>::Channel],

    // ADC state
    app_state: MapCell<AppState>,
    active: Cell<bool>,
    mode: Cell<ADCMode>,

    // ADC buffers
    adc_buf1: TakeCell<'static, [u16]>,
    adc_buf2: TakeCell<'static, [u16]>,
}

#[derive(Copy,Clone,PartialEq)]
enum ADCMode {
    NoMode = -1,
    SingleSample = 0,
    MultipleSample = 1,
    ContinuousSample = 2,
}

pub struct AppState {
    channel: Option<usize>,
    callback: Option<Callback>,
    app_buf1: Option<AppSlice<Shared, u8>>,
    app_buf2: Option<AppSlice<Shared, u8>>,
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
                app_buf1: None,
                app_buf2: None,
            }),
            active: Cell::new(false),
            mode: Cell::new(ADCMode::NoMode),

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
        self.mode.set(ADCMode::SingleSample);
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
            state.app_buf1.is_some()
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
        self.mode.set(ADCMode::MultipleSample);
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
        self.mode.set(ADCMode::NoMode);

        self.app_state.map(|state| {
            state.callback.map(|mut callback| {
                let channel = state.channel.unwrap_or(0);
                callback.schedule(ADCMode::SingleSample as usize, channel, sample as usize);
            });
        });
    }

    fn buffer_ready(&self, buf: &'static mut [u16], length: usize) {
        let prev_mode = self.mode.get();
        if prev_mode == ADCMode::MultipleSample {
            // sampled single buffer
            self.active.set(false);
            self.mode.set(ADCMode::NoMode);

            // replace buffer
            self.adc_buf1.replace(buf);

            // stop adc
            self.adc.stop_sampling();

            // handle buffers
            let mut buf_ptr = 0;
            let mut read_len = length;
            self.adc_buf1.map(|adc_buf| {
                self.app_state.map(|state| {
                    state.app_buf1.as_mut().map(|app_buf| {

                        // copy buffer, limiting to buffer sizes and checking for Nones.
                        // Also need to do chunking because appslices are inherently [u8]s
                        for (chunk, &sample) in app_buf.chunks_mut(2).zip(adc_buf.iter()).take(length) {
                            let mut val = sample;
                            for byte in chunk.iter_mut() {
                                *byte = (val & 0xFF) as u8;
                                val = val >> 8;
                            }
                        }

                        // save data to provide to app. The length is number of samples, which
                        // should be a u16
                        read_len = cmp::min(app_buf.len()/2, cmp::min(adc_buf.len(), length));
                        buf_ptr = app_buf.ptr() as usize;
                    });
                });
            });

            // perform callback
            self.app_state.map(|state| {
                state.callback.map(|mut callback| {
                    let channel = state.channel.unwrap_or(0);
                    let len_chan = (read_len << 8) | (channel & 0xFF);
                    callback.schedule(ADCMode::MultipleSample as usize, len_chan, buf_ptr);
                });
            });
        } else {
            // continuously sampling
            panic!("No impl sry");
        }
    }
}

impl<'a, A: hil::adc::ADCSingle + hil::adc::ADCContinuous + 'a> Driver for ADC<'a, A> {
    fn allow(&self, _appid: AppId, allow_num: usize, slice: AppSlice<Shared, u8>) -> ReturnCode {
        match allow_num {
            // Pass buffer for samples to go into
            0 => {
                // set first buffer
                self.app_state.map(|state| {
                    state.app_buf1 = Some(slice);
                });

                ReturnCode::SUCCESS
            }

            // Pass a second buffer to be used for double-buffered continuous sampling
            1 => {
                // set second buffer
                self.app_state.map(|state| {
                    state.app_buf2 = Some(slice);
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
