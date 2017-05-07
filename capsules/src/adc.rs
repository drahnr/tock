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
    active: Cell<bool>,
    mode: Cell<ADCMode>,

    // App state
    app_state: MapCell<AppState>,
    channel: Cell<usize>,
    callback: Cell<Option<Callback>>,
    app_buf_offset: Cell<usize>,
    using_app_buf1: Cell<bool>,

    // ADC buffers
    adc_buf1: TakeCell<'static, [u16]>,
    adc_buf2: TakeCell<'static, [u16]>,
    using_adc_buf1: Cell<bool>,
}

#[derive(Copy,Clone,PartialEq)]
enum ADCMode {
    NoMode = -1,
    SingleSample = 0,
    MultipleSample = 1,
    ContinuousSample = 2,
}

pub struct AppState {
    app_buf1: Option<AppSlice<Shared, u8>>,
    app_buf2: Option<AppSlice<Shared, u8>>,
}

/// Buffers to use for DMA transfers
/// The size is chosen somewhat arbitrarily, but has been tested. At 175000 Hz, buffers need to be
/// swapped every 70 us and copied over before the next swap. In testing, it seems to keep up fine.
pub static mut ADC_BUFFER1: [u16; 128] = [0; 128];
pub static mut ADC_BUFFER2: [u16; 128] = [0; 128];

impl<'a, A: hil::adc::ADCSingle + hil::adc::ADCContinuous + 'a> ADC<'a, A> {
    pub fn new(adc: &'a A, channels: &'a [&'a <A as hil::adc::ADCSingle>::Channel], adc_buf1: &'static mut [u16; 128], adc_buf2: &'static mut [u16; 128]) -> ADC<'a, A> {
        ADC {
            // ADC driver
            adc: adc,
            channels: channels,

            // ADC state
            active: Cell::new(false),
            mode: Cell::new(ADCMode::NoMode),

            // App state
            app_state: MapCell::new(AppState {
                app_buf1: None,
                app_buf2: None,
            }),
            channel: Cell::new(0),
            callback: Cell::new(None),
            app_buf_offset: Cell::new(0),
            using_app_buf1: Cell::new(true),

            // ADC buffers
            adc_buf1: TakeCell::new(adc_buf1),
            adc_buf2: TakeCell::new(adc_buf2),
            using_adc_buf1: Cell::new(true),
        }
    }

    fn initialize(&self) -> ReturnCode {
        self.adc.initialize()
    }

    fn sample(&self, channel: usize) -> ReturnCode {

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

        // save state for callback
        self.active.set(true);
        self.mode.set(ADCMode::SingleSample);
        self.app_buf_offset.set(0);
        self.channel.set(channel);

        // start a single sample
        let res = self.adc.sample(chan);
        if res != ReturnCode::SUCCESS {
            // failure, clear state
            self.active.set(false);
            self.mode.set(ADCMode::NoMode);

            return res;
        }

        ReturnCode::SUCCESS
    }

    fn sample_buffer (&self, channel: usize, frequency: u32) -> ReturnCode {

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

        // cannot sample a buffer without a buffer to sample into
        let mut app_buf_length = 0;
        let exists = self.app_state.map_or(false, |state| {
            app_buf_length = state.app_buf1.as_mut().map_or(0, |buf| buf.len());
            state.app_buf1.is_some()
        });
        if !exists {
            return ReturnCode::ENOMEM;
        }

        // save state for callback
        self.active.set(true);
        self.mode.set(ADCMode::MultipleSample);
        self.app_buf_offset.set(0);
        self.channel.set(channel);

        // start a continuous sample
        let res = self.adc_buf1.take().map_or(ReturnCode::EBUSY, |buf| {
            // determine request length
            let req_len = cmp::min(app_buf_length/2, buf.len());

            self.using_app_buf1.set(true);
            self.using_adc_buf1.set(true);
            self.adc.sample_continuous(chan, frequency, buf, req_len)
        });
        if res != ReturnCode::SUCCESS {
            // failure, clear state
            self.active.set(false);
            self.mode.set(ADCMode::NoMode);

            return res;
        }

        ReturnCode::SUCCESS
    }

    fn sample_continuous (&self, channel: usize, frequency: u32) -> ReturnCode {

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

        // cannot continuously sample without two buffers
        let mut app_buf_length = 0;
        let exists = self.app_state.map_or(false, |state| {
            app_buf_length = state.app_buf1.as_mut().map_or(0, |buf| buf.len());
            state.app_buf1.is_some() && state.app_buf2.is_some()
        });
        if !exists {
            return ReturnCode::ENOMEM;
        }

        // save state for callback
        self.active.set(true);
        self.mode.set(ADCMode::ContinuousSample);
        self.app_buf_offset.set(0);
        self.channel.set(channel);

        // start a continuous sample
        let res = self.adc_buf1.take().map_or(ReturnCode::EBUSY, |buf| {
            // determine request length
            let req_len = cmp::min(app_buf_length/2, buf.len());

            self.using_app_buf1.set(true);
            self.using_adc_buf1.set(true);
            self.adc.sample_continuous(chan, frequency, buf, req_len)
        });
        if res != ReturnCode::SUCCESS {
            // failure, clear state
            self.active.set(false);
            self.mode.set(ADCMode::NoMode);

            return res;
        }

        ReturnCode::SUCCESS
    }

    fn stop_sampling (&self) -> ReturnCode {
        if !self.active.get() || self.mode.get() == ADCMode::NoMode {
            // already inactive!
            return ReturnCode::SUCCESS;
        }

        if self.mode.get() == ADCMode::SingleSample {
            // set state for callback
            self.active.set(false);
            self.mode.set(ADCMode::NoMode);
            self.app_buf_offset.set(0);

            // cannot cancel the single sample, but we'll just not send a callback
            ReturnCode::SUCCESS
        } else {

            // set state for callback
            self.active.set(false);
            self.mode.set(ADCMode::NoMode);
            self.app_buf_offset.set(0);

            // actually cancel the operation
            self.adc.stop_sampling()
        }
    }
}

impl<'a, A: hil::adc::ADCSingle + hil::adc::ADCContinuous + 'a> hil::adc::Client for ADC<'a, A> {
    fn sample_done(&self, sample: u16) {
        if self.active.get() && self.mode.get() == ADCMode::SingleSample {
            self.active.set(false);
            self.mode.set(ADCMode::NoMode);
            self.app_buf_offset.set(0);

            self.callback.get().map(|mut callback| {
                callback.schedule(ADCMode::SingleSample as usize, self.channel.get(), sample as usize);
            });
        } else {
            // operation probably canceled. Make sure state is consistent. No callback
            self.active.set(false);
            self.mode.set(ADCMode::NoMode);
            self.app_buf_offset.set(0);
        }
    }

    fn buffer_ready(&self, buf: &'static mut [u16], length: usize) {

        // determine which adc buffer is which
        let curr_adc_buf;
        let next_adc_buf;
        if self.using_adc_buf1.get() {
            curr_adc_buf = &self.adc_buf1;
            next_adc_buf = &self.adc_buf2;
        } else {
            curr_adc_buf = &self.adc_buf2;
            next_adc_buf = &self.adc_buf1;
        }

        // replace buffer
        curr_adc_buf.replace(buf);

        // is this an expected buffer?
        if self.active.get() && (self.mode.get() == ADCMode::MultipleSample || self.mode.get() == ADCMode::ContinuousSample) {
            self.app_state.map(|state| {

                // determine which app buffer we should use
                // also save the length of the next buffer, in case we need to start a new request
                let app_buf;
                let next_app_buf;
                if self.using_app_buf1.get() {
                    app_buf = state.app_buf1.as_mut();
                    next_app_buf = state.app_buf2.as_ref();
                } else {
                    app_buf = state.app_buf2.as_mut();
                    next_app_buf = state.app_buf1.as_ref();
                }
                app_buf.map(move |app_buf| {

                    // check if we have received enough samples
                    let samples_remaining = (app_buf.len()-self.app_buf_offset.get())/2;
                    if length < samples_remaining {
                        // we need to receive more samples

                        // continue transfer with next buffer
                        next_adc_buf.take().map(|adc_buf| {
                            self.using_adc_buf1.set(!self.using_adc_buf1.get());
                            self.adc.continue_sampling(adc_buf, samples_remaining-length);
                        });

                        // copy bytes to app buffer
                        // In order, the `for` commands:
                        //  * `chunks_mut`: get sets of two bytes from the app buffer
                        //  * `skip`: skips the already written bytes from the app buffer
                        //  * `zip`: ties that iterator to an iterator on the adc buffer, limiting
                        //    iteration length to the minimum of each of their lengths
                        //  * `take`: limits us to the minimum of buffer lengths or sample length
                        // We then split each sample into its two bytes and copy them to the app
                        // buffer
                        curr_adc_buf.map(|adc_buf| {
                            for (chunk, &sample) in app_buf.chunks_mut(2).skip(self.app_buf_offset.get()/2).zip(adc_buf.iter()).take(length) {
                                let mut val = sample;
                                for byte in chunk.iter_mut() {
                                    *byte = (val & 0xFF) as u8;
                                    val = val >> 8;
                                }
                            }
                        });

                        // update our offset based on how many samples we copied
                        self.app_buf_offset.set(self.app_buf_offset.get() + length*2);

                    } else {
                        // we have received the full app buffer

                        // save original values for later use
                        let prev_app_buf_offset = self.app_buf_offset.get();
                        let prev_mode = self.mode.get();

                        // do we need to start another transfer?
                        if self.mode.get() == ADCMode::MultipleSample {
                            // we are finished!
                            self.active.set(false);
                            self.mode.set(ADCMode::NoMode);
                            self.app_buf_offset.set(0);
                            self.adc.stop_sampling();

                        } else {
                            // we need to start another sample!
                            self.app_buf_offset.set(0);
                            self.using_app_buf1.set(!self.using_app_buf1.get());

                            // continue sampling
                            next_adc_buf.take().map(|adc_buf| {
                                self.using_adc_buf1.set(!self.using_adc_buf1.get());
                                let next_buf_len = next_app_buf.map_or(0, |buf| buf.len());
                                self.adc.continue_sampling(adc_buf, next_buf_len/2);
                            });
                        }

                        // copy bytes to app buffer
                        // In order, the `for` commands:
                        //  * `chunks_mut`: get sets of two bytes from the app buffer
                        //  * `skip`: skips the already written bytes from the app buffer
                        //  * `zip`: ties that iterator to an iterator on the adc buffer, limiting
                        //    iteration length to the minimum of each of their lengths
                        //  * `take`: limits us to the minimum of buffer lengths or sample length
                        // We then split each sample into its two bytes and copy them to the app
                        // buffer
                        curr_adc_buf.map(|adc_buf| {
                            for (chunk, &sample) in app_buf.chunks_mut(2).skip(prev_app_buf_offset/2).zip(adc_buf.iter()).take(length) {
                                let mut val = sample;
                                for byte in chunk.iter_mut() {
                                    *byte = (val & 0xFF) as u8;
                                    val = val >> 8;
                                }
                            }
                        });

                        // perform callback
                        self.callback.get().map(|mut callback| {
                            let len_chan = (app_buf.len()/2 << 8) | (self.channel.get() & 0xFF);
                            callback.schedule(prev_mode as usize, len_chan, app_buf.ptr() as usize);
                        });
                    }
                });
            });
        } else {
            // operation was likely canceled. Make sure state is consistent. No callback
            self.active.set(false);
            self.mode.set(ADCMode::NoMode);
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
                self.callback.set(Some(callback));
                ReturnCode::SUCCESS
            }

            // default
            _ => ReturnCode::ENOSUPPORT,
        }
    }

    fn command(&self, command_num: usize, data: usize, _appid: AppId) -> ReturnCode {
        match command_num {
            // check if present
            0 => {
                ReturnCode::SuccessWithValue { value: self.channels.len() as usize }
            },

            // Single sample on channel
            1 => {
                let channel = (data & 0xFF) as usize;
                self.sample(channel)
            },

            // Multiple sample on a channel
            2 => {
                let channel = (data & 0xFF) as usize;
                let frequency = (data >> 8) as u32;
                self.sample_buffer(channel, frequency)
            },

            // Continuous sample on a channel
            3 => {
                let channel = (data & 0xFF) as usize;
                let frequency = (data >> 8) as u32;
                self.sample_continuous(channel, frequency)
            },

            // Stop sampling
            4 => {
                self.stop_sampling()
            },

            // default
            _ => ReturnCode::ENOSUPPORT,
        }
    }
}
