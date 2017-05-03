//! ADC Capsule
//!
//! Provides userspace applications with the ability to sample
//! ADC channels.

use core::cell::Cell;
use kernel::{AppId, AppSlice, Callback, Container, Driver, ReturnCode, Shared};
use kernel::common::take_cell::{MapCell};
use kernel::hil::adc::{Client, ADCChannel, ADCSingle, ADCContinuous};

pub struct ADC<'a, A: ADCSingle + ADCContinuous + 'a> {
    // ADC driver
    adc: &'a A,
    channels: &'a [&'a ADCChannel],

    // ADC state
    app_state: MapCell<AppState>,
    active: Cell<bool>
}

#[derive(Default)]
pub struct AppState {
    channel: Option<usize>,
    callback: Option<Callback>,
    sample_buffer1: Option<AppSlice<Shared, u8>>,
    sample_buffer2: Option<AppSlice<Shared, u8>>,
}

impl<'a, A: ADCSingle + ADCContinuous + 'a> ADC<'a, A> {
    pub fn new(adc: &'a A, channels: &'a [&'a ADCChannel]) -> ADC<'a, A> {
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

        // start a continuous sample
        // demand that the first appslice has been set

        // run until the appslice is full

        // when done perform callback
        // type - buffer, channel, buffer pointer

        ReturnCode::SUCCESS
    }

    fn sample_continuous (&self, channel: usize, frequency: u32, appid: AppId) -> ReturnCode {

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

impl<'a, A: ADCSingle + ADCContinuous + 'a> Client for ADC<'a, A> {
    fn sample_done(&self, sample: u16) {
        panic!("Got a sample: {}", sample);
        /*
        self.channel.get().map(|cur_channel| {
            self.channel.set(None);
            self.app.each(|app| if app.channel == Some(cur_channel) {
                app.channel = None;
                app.callback.map(|mut cb| cb.schedule(0, cur_channel as usize, sample as usize));
            } else if app.channel.is_some() {
                self.channel.set(app.channel);
            });
        });
        self.channel.get().map(|next_channel| { self.adc.sample(next_channel); });
        */
    }

    fn buffer_ready(&self, buf: &'static mut [u16], length: usize) {
        //debug!("Got {} bytes of data: \n{:?}", length, buf);
        //self.adc.continue_sampling(buf);

    }
}

impl<'a, A: ADCSingle + ADCContinuous + 'a> Driver for ADC<'a, A> {
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
