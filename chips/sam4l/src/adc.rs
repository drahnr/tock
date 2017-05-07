// adc.rs -- Implementation of SAM4L ADCIFE.
//
// This is an implementation of the SAM4L analog to digital converter. It is bare-bones
// because it provides little flexibility on how samples are taken. Currently,
// all samples
//   - are 12 bits
//   - use the ground pad as the negative reference
//   - use a VCC/2 positive reference
//   - are right justified
//
// Samples can either be collected individually or continuously at a specified
// frequency
//
// Author: Philip Levis <pal@cs.stanford.edu>, Branden Ghena <brghena@umich.edu>
// Updated: May 1, 2017
//

use core::cell::Cell;
use core::cmp;
use core::mem;
use core::slice;
use dma;
use kernel::common::math;
use kernel::common::volatile_cell::VolatileCell;
use kernel::hil;
use kernel::returncode::ReturnCode;
use nvic;
use pm::{self, Clock, PBAClock};
use scif;

/// Representation of an ADC channel on the SAM4L.
pub struct ADCChannel {
    chan_num: u32,
    internal: u32,
}

/// SAM4L ADC channels.
#[derive(Copy,Clone)]
#[repr(u8)]
enum Channel {
    AD0 = 0x00,
    AD1 = 0x01,
    AD2 = 0x02,
    AD3 = 0x03,
    AD4 = 0x04,
    AD5 = 0x05,
    AD6 = 0x06,
    AD7 = 0x07,
    AD8 = 0x08,
    AD9 = 0x09,
    AD10 = 0x0A,
    AD11 = 0x0B,
    AD12 = 0x0C,
    AD13 = 0x0D,
    AD14 = 0x0E,
    Bandgap = 0x0F,
    ScaledVCC = 0x12,
    DAC = 0x13,
    Vsingle = 0x16,
    ReferenceGround = 0x17,
}

/// Initialization of an ADC channel.
impl ADCChannel {
    /// Create a new ADC channel.
    /// channel - Channel enum representing the channel number and whether it is internal
    const fn new(channel: Channel) -> ADCChannel {
        ADCChannel {
            chan_num: ((channel as u8) & 0x0F) as u32,
            internal: (((channel as u8) >> 4) & 0x01) as u32,
        }
    }
}

/// Statically allocated ADC channels. Used in board configurations to specify which channels are
/// used on the platform.
pub static mut CHANNEL_AD0: ADCChannel = ADCChannel::new(Channel::AD0);
pub static mut CHANNEL_AD1: ADCChannel = ADCChannel::new(Channel::AD1);
pub static mut CHANNEL_AD2: ADCChannel = ADCChannel::new(Channel::AD2);
pub static mut CHANNEL_AD3: ADCChannel = ADCChannel::new(Channel::AD3);
pub static mut CHANNEL_AD4: ADCChannel = ADCChannel::new(Channel::AD4);
pub static mut CHANNEL_AD5: ADCChannel = ADCChannel::new(Channel::AD5);
pub static mut CHANNEL_AD6: ADCChannel = ADCChannel::new(Channel::AD6);
pub static mut CHANNEL_AD7: ADCChannel = ADCChannel::new(Channel::AD7);
pub static mut CHANNEL_AD8: ADCChannel = ADCChannel::new(Channel::AD8);
pub static mut CHANNEL_AD9: ADCChannel = ADCChannel::new(Channel::AD9);
pub static mut CHANNEL_AD10: ADCChannel = ADCChannel::new(Channel::AD10);
pub static mut CHANNEL_AD11: ADCChannel = ADCChannel::new(Channel::AD11);
pub static mut CHANNEL_AD12: ADCChannel = ADCChannel::new(Channel::AD12);
pub static mut CHANNEL_AD13: ADCChannel = ADCChannel::new(Channel::AD13);
pub static mut CHANNEL_AD14: ADCChannel = ADCChannel::new(Channel::AD14);
pub static mut CHANNEL_BANDGAP: ADCChannel = ADCChannel::new(Channel::Bandgap);
pub static mut CHANNEL_SCALED_VCC: ADCChannel = ADCChannel::new(Channel::ScaledVCC);
pub static mut CHANNEL_DAC: ADCChannel = ADCChannel::new(Channel::DAC);
pub static mut CHANNEL_VSINGLE: ADCChannel = ADCChannel::new(Channel::Vsingle);
pub static mut CHANNEL_REFERENCE_GROUND: ADCChannel = ADCChannel::new(Channel::ReferenceGround);


/// ADC driver code for the SAM4L.
pub struct ADC {
    registers: *mut ADCRegisters,

    enabled: Cell<bool>,
    adc_clk_freq: Cell<u32>,
    active: Cell<bool>,
    continuous: Cell<bool>,
    dma_running: Cell<bool>,

    rx_dma: Cell<Option<&'static dma::DMAChannel>>,
    rx_dma_peripheral: dma::DMAPeripheral,
    rx_len: Cell<usize>,

    client: Cell<Option<&'static hil::adc::Client>>,
}

/// Memory mapped registers for the ADC.
#[repr(C, packed)]
pub struct ADCRegisters {
    // From page 1005 of SAM4L manual
    pub cr: VolatileCell<u32>, // Control               (0x00)
    pub cfg: VolatileCell<u32>, // Configuration        (0x04)
    pub sr: VolatileCell<u32>, // Status                (0x08)
    pub scr: VolatileCell<u32>, // Status clear         (0x0c)
    pub pad: VolatileCell<u32>, // padding/reserved
    pub seqcfg: VolatileCell<u32>, // Sequencer config  (0x14)
    pub cdma: VolatileCell<u32>, // Config DMA          (0x18)
    pub tim: VolatileCell<u32>, // Timing config        (0x1c)
    pub itimer: VolatileCell<u32>, // Internal timer    (0x20)
    pub wcfg: VolatileCell<u32>, // Window config       (0x24)
    pub wth: VolatileCell<u32>, // Window threshold     (0x28)
    pub lcv: VolatileCell<u32>, // Last converted value (0x2c)
    pub ier: VolatileCell<u32>, // Interrupt enable     (0x30)
    pub idr: VolatileCell<u32>, // Interrupt disable    (0x34)
    pub imr: VolatileCell<u32>, // Interrupt mask       (0x38)
    pub calib: VolatileCell<u32>, // Calibration        (0x3c)
    pub version: VolatileCell<u32>, // Version          (0x40)
    pub parameter: VolatileCell<u32>, // Parameter      (0x44)
}
// Page 59 of SAM4L data sheet
pub const BASE_ADDRESS: *mut ADCRegisters = 0x40038000 as *mut ADCRegisters;

/// Statically allocated ADC driver. Used in board configurations to connect to various capsules.
pub static mut ADC0: ADC = ADC::new(BASE_ADDRESS, dma::DMAPeripheral::ADCIFE_RX);

/// Functions for initializing the ADC.
impl ADC {
    /// Create a new ADC driver.
    /// base_address - pointer to the ADC's memory mapped I/O registers
    /// rx_dma_peripheral - type used for DMA transactions
    const fn new(base_address: *mut ADCRegisters, rx_dma_peripheral: dma::DMAPeripheral) -> ADC {
        ADC {
            // pointer to memory mapped I/O registers
            registers: base_address,

            // status of the ADC peripheral
            enabled: Cell::new(false),
            adc_clk_freq: Cell::new(0),
            active: Cell::new(false),
            continuous: Cell::new(false),
            dma_running: Cell::new(false),

            // DMA status
            rx_dma: Cell::new(None),
            rx_dma_peripheral: rx_dma_peripheral,
            rx_len: Cell::new(0),

            // higher layer to send responses to
            client: Cell::new(None),
        }
    }

    /// Sets the client for this driver.
    /// client - reference to capsule which handles responses
    pub fn set_client<C: hil::adc::Client>(&self, client: &'static C) {
        self.client.set(Some(client));
    }

    /// Sets the DMA channel for this driver.
    /// rx_dma - reference to the DMA channel the ADC should use
    pub fn set_dma(&self, rx_dma: &'static dma::DMAChannel) {
        self.rx_dma.set(Some(rx_dma));
    }

    /// Interrupt handler for the ADC.
    pub fn handle_interrupt(&mut self) {
        let regs: &mut ADCRegisters = unsafe { mem::transmute(self.registers) };
        let status = regs.sr.get();

        // sequencer end of conversion (sample complete)
        if status & 0x01 == 0x01 {
            if self.enabled.get() && self.active.get() && !self.continuous.get() {
                self.active.set(false);

                // disable interrupt
                regs.idr.set(1);

                // single sample complete. Send value to client
                let val = (regs.lcv.get() & 0xffff) as u16;
                self.client.get().map(|client| { client.sample_done(val); });
            }

            // clear status
            regs.scr.set(0x00000001);
        }
    }
}

/// Implements an ADC capable of single samples.
impl hil::adc::ADCSingle for ADC {
    type Channel = ADCChannel;

    /// Enable and configure the ADC.
    /// This can be called multiple times with no side effects.
    fn initialize(&self) -> ReturnCode {
        let regs: &mut ADCRegisters = unsafe { mem::transmute(self.registers) };

        if !self.enabled.get() {
            self.enabled.set(true);

            // First, enable the clocks
            // Both the ADCIFE clock and GCLK10 are needed
            let mut clock_divisor;
            unsafe {
                // turn on ADCIFE bus clock. Already set to the same frequency
                // as the CPU clock
                pm::enable_clock(Clock::PBA(PBAClock::ADCIFE));
                nvic::enable(nvic::NvicIdx::ADCIFE);

                // turn on GCLK10. The Generic Clock is needed for the ADC to
                // work, although I have no idea what it is needed for. Its
                // speed does not seem to matter, so let's just peg it to the
                // CPU clock
                scif::generic_clock_enable(scif::GenericClock::GCLK10, scif::ClockSource::CLK_CPU);

                // determine clock divider
                // we need the ADC_CLK to be a maximum of 1.5 MHz in frequency, so we need to find
                // the PRESCAL value that will make this happen.
                // Formula: f(ADC_CLK) = f(CLK_CPU)/2^(N+2) <= 1.5 MHz and we solve for N
                // becomes: N <= ceil(log_2(f(CLK_CPU)/1500000)) - 2
                let cpu_frequency = pm::get_system_frequency();
                let divisor = (cpu_frequency + (1500000 - 1)) / 1500000; // ceiling of division
                clock_divisor = math::log_base_two(math::closest_power_of_two(divisor)) - 2;
                clock_divisor = cmp::min(cmp::max(clock_divisor, 0), 7); // keep in bounds
                self.adc_clk_freq.set(cpu_frequency / (1 << (clock_divisor + 2)));
            }

            // configure the ADC
            let cfg_val = (clock_divisor << 8) | // PRESCAL: clock divider
                          (0x1 << 6) | // CLKSEL: use ADCIFE clock
                          (0x0 << 4) | // SPEED: maximum 300 ksps
                          (0x4 << 1); // REFSEL: VCC/2 reference
            regs.cfg.set(cfg_val);

            // software reset (does not clear registers)
            regs.cr.set(1);

            // enable ADC
            regs.cr.set(1 << 8);

            // wait until status is enabled
            let mut timeout = 10000;
            while regs.sr.get() & (0x1 << 24) != (0x1 << 24) {
                timeout -= 1;
                if timeout == 0 {
                    // ADC never enabled
                    return ReturnCode::FAIL;
                }
            }

            // enable Bandgap buffer and Reference buffer. I don't actually
            // know what these do, but you need to turn them on
            let cr_val = (0x1 << 10) | // BGREQEN: Enable bandgap buffer request
                         (0x1 <<  4); // REFBUFEN: Enable reference buffer
            regs.cr.set(cr_val);

            // wait until buffers are enabled
            timeout = 100000;
            while regs.sr.get() & (0x51000000) != 0x51000000 {
                timeout -= 1;
                if timeout == 0 {
                    // ADC buffers never enabled
                    return ReturnCode::FAIL;
                }
            }
        }

        ReturnCode::SUCCESS
    }

    /// Capture a single analog sample, calling the client when complete.
    /// Returns an error if the ADC is already sampling.
    /// channel - the ADC channel to sample
    fn sample(&self, channel: &Self::Channel) -> ReturnCode {
        let regs: &mut ADCRegisters = unsafe { mem::transmute(self.registers) };

        if !self.enabled.get() {
            ReturnCode::EOFF

        } else if self.active.get() {
            // only one sample at a time
            ReturnCode::EBUSY

        } else {
            self.active.set(true);
            self.continuous.set(false);

            let cfg = (0x7 << 20) | // MUXNEG: ground pad
                      (channel.chan_num << 16) | // MUXPOS: selection
                      (0x1 << 15) | // INTERNAL: internal neg
                      (channel.internal << 14) | // INTERNAL: pos selection
                      (0x0 << 12) | // RES: 12-bit resolution
                      (0x0 <<  8) | // TRGSEL: software trigger
                      (0x0 <<  7) | // GCOMP: no gain compensation
                      (0x7 <<  4) | // GAIN: 0.5x gain
                      (0x0 <<  2) | // BIPOLAR: unipolar mode
                      (0x0 <<  0); // HWLA: right justify value
            regs.seqcfg.set(cfg);

            // clear any current status
            regs.scr.set(0x2F);

            // enable end of conversion interrupt
            regs.ier.set(1);

            // initiate conversion
            regs.cr.set(8);

            ReturnCode::SUCCESS
        }
    }
}

/// Implements an ADC capable of continuous sampling
impl hil::adc::ADCContinuous for ADC {
    /// Capture samples from the ADC continuously at a given frequency until buffer is full,
    /// calling the client when complete.
    /// Note that due to hardware constraints the maximum frequency range of the ADC is from
    /// 187 kHz to 23 Hz (although its precision is limited at higher frequencies due to aliasing).
    /// channel - the ADC channel to sample
    /// frequency - frequency to sample at
    /// buf - buffer to fill with samples
    /// length - number of samples to collect (up to buffer length)
    fn sample_continuous(&self,
                         channel: &Self::Channel,
                         frequency: u32,
                         buf: &'static mut [u16],
                         length: usize)
                         -> ReturnCode {
        let regs: &mut ADCRegisters = unsafe { mem::transmute(self.registers) };

        if !self.enabled.get() {
            ReturnCode::EOFF

        } else if self.active.get() {
            // only one sample at a time
            ReturnCode::EBUSY

        } else if frequency == 0 || frequency > 250000 {
            // can't sample faster than the max sampling frequency
            ReturnCode::EINVAL

        } else {
            self.active.set(true);
            self.continuous.set(true);

            let cfg = (0x7 << 20) | // MUXNEG: ground pad
                      (channel.chan_num << 16) | // MUXPOS: selection
                      (0x1 << 15) | // INTERNAL: internal neg
                      (channel.internal << 14) | // INTERNAL: pos selection
                      (0x0 << 12) | // RES: 12-bit resolution
                      (0x1 <<  8) | // TRGSEL: internal timer trigger
                      (0x0 <<  7) | // GCOMP: no gain compensation
                      (0x7 <<  4) | // GAIN: 0.5x gain
                      (0x0 <<  2) | // BIPOLAR: unipolar mode
                      (0x0 <<  0); // HWLA: right justify value
            regs.seqcfg.set(cfg);

            // stop timer if running
            regs.cr.set(0x02);

            // set timer, limit to bounds
            // f(timer) = f(adc) / (counter + 1)
            let mut counter = (self.adc_clk_freq.get() / frequency) - 1;
            counter = cmp::max(cmp::min(counter, 0xFFFF), 0);
            regs.itimer.set(counter);

            // clear any current status
            regs.scr.set(0x2F);

            // receive up to the buffer's length samples
            let dma_len = cmp::min(buf.len(), length);

            // change buffer into a [u8]
            // this is unsafe but acceptable for the following reasons
            //  * the buffer is aligned based on 16-bit boundary, so the 8-bit
            //    alignment is fine
            //  * the DMA is doing checking based on our expected data width to
            //    make sure we don't go past dma_buf.len()/width
            //  * we will transmute the array back to a [u16] after the DMA
            //    transfer is complete
            let dma_buf_ptr = unsafe { mem::transmute::<*mut u16, *mut u8>(buf.as_mut_ptr()) };
            let dma_buf = unsafe { slice::from_raw_parts_mut(dma_buf_ptr, buf.len() * 2) };

            // set up the DMA
            self.rx_dma.get().map(move |dma| {
                self.dma_running.set(true);
                dma.enable();
                self.rx_len.set(dma_len);
                dma.do_xfer(self.rx_dma_peripheral, dma_buf, dma_len);
            });

            // start timer
            regs.cr.set(0x04);

            ReturnCode::SUCCESS
        }
    }

    /// Provide a new buffer to send on-going continuous samples to.
    /// This is expected to be called after the `buffer_ready` callback.
    /// buf - buffer to fill with samples
    /// length - number of samples to collect (up to buffer length)
    fn continue_sampling(&self, buf: &'static mut [u16], length: usize) -> ReturnCode {
        if !self.enabled.get() {
            ReturnCode::EOFF

        } else if !self.active.get() {
            // cannot continue sampling that isn't running
            ReturnCode::EINVAL

        } else if !self.continuous.get() {
            // cannot continue a single sample operation
            ReturnCode::EINVAL

        } else if self.dma_running.get() {
            // cannot change buffer while DMA is running
            ReturnCode::EBUSY

        } else {
            // give a new buffer to the DMA and start it

            // receive up to the buffer's length samples
            let dma_len = cmp::min(buf.len(), length);

            // change buffer into a [u8]
            // this is unsafe but acceptable for the following reasons
            //  * the buffer is aligned based on 16-bit boundary, so the 8-bit
            //    alignment is fine
            //  * the DMA is doing checking based on our expected data width to
            //    make sure we don't go past dma_buf.len()/width
            //  * we will transmute the array back to a [u16] after the DMA
            //    transfer is complete
            let dma_buf_ptr = unsafe { mem::transmute::<*mut u16, *mut u8>(buf.as_mut_ptr()) };
            let dma_buf = unsafe { slice::from_raw_parts_mut(dma_buf_ptr, buf.len() * 2) };

            // set up the DMA
            self.rx_dma.get().map(move |dma| {
                self.dma_running.set(true);
                dma.enable();
                self.rx_len.set(dma_len);
                dma.do_xfer(self.rx_dma_peripheral, dma_buf, dma_len);
            });

            ReturnCode::SUCCESS
        }
    }

    /// Stop continuously sampling the ADC.
    /// This is expected to be called after the `buffer_ready` callback, but can be called at any
    /// time to abort the currently running operation. The buffer, if any, will be returned via the
    /// `buffer_ready` callback.
    fn stop_sampling(&self) -> ReturnCode {
        let regs: &mut ADCRegisters = unsafe { mem::transmute(self.registers) };

        if !self.enabled.get() {
            ReturnCode::EOFF

        } else if !self.active.get() {
            // cannot cancel sampling that isn't running
            ReturnCode::EINVAL

        } else if !self.continuous.get() {
            // cannot cancel a single sample operation
            ReturnCode::EINVAL

        } else {
            // stopping a continuous sample operation
            self.active.set(false);
            self.continuous.set(false);

            // stop internal timer
            regs.cr.set(0x02);

            // stop DMA transfer if going
            let dma_buffer = self.rx_dma.get().map_or(None, |rx_dma| {
                self.dma_running.set(false);
                let dma_buf = rx_dma.abort_xfer();
                rx_dma.disable();
                dma_buf
            });

            // if there was a buffer, send it to the client. It will be
            // partially invalid if it exists, but they were the ones who
            // wanted to stop in a hurry. In the common case, this is only
            // called after a `buffer_ready` call, in which case this map will
            // fizzle
            self.client.get().map(|client| {
                dma_buffer.map(|dma_buf| {
                    let length = self.rx_len.get();

                    // change buffer back into a [u16]
                    // the buffer was originally a [u16] so this should be okay
                    let buf_ptr =
                        unsafe { mem::transmute::<*mut u8, *mut u16>(dma_buf.as_mut_ptr()) };
                    let buf = unsafe { slice::from_raw_parts_mut(buf_ptr, dma_buf.len() / 2) };

                    // pass the buffer up to the next layer. It will then either send down another
                    // buffer to continue sampling, or stop sampling
                    client.buffer_ready(buf, length);
                });
            });
            self.rx_len.set(0);

            ReturnCode::SUCCESS
        }
    }
}

/// Implements a client of a DMA.
impl dma::DMAClient for ADC {
    /// Handler for DMA transfer completion.
    /// pid - the DMA peripheral that is complete
    fn xfer_done(&self, pid: dma::DMAPeripheral) {
        // check if this was an RX transfer
        if pid == self.rx_dma_peripheral {
            // RX transfer was completed

            // get buffer
            let dma_buffer = self.rx_dma.get().map_or(None, |rx_dma| {
                self.dma_running.set(false);
                let dma_buf = rx_dma.abort_xfer();
                rx_dma.disable();
                dma_buf
            });

            // get length
            let length = self.rx_len.get();
            self.rx_len.set(0);

            // alert client
            self.client.get().map(|client| {
                dma_buffer.map(|dma_buf| {

                    // change buffer back into a [u16]
                    // the buffer was originally a [u16] so this should be okay
                    let buf_ptr =
                        unsafe { mem::transmute::<*mut u8, *mut u16>(dma_buf.as_mut_ptr()) };
                    let buf = unsafe { slice::from_raw_parts_mut(buf_ptr, dma_buf.len() / 2) };

                    // pass the buffer up to the next layer. It will then either send down another
                    // buffer to continue sampling, or stop sampling
                    client.buffer_ready(buf, length);
                });
            });
        }
    }
}

/// Handles ADCIFE interrupts.
interrupt_handler!(adcife_handler, ADCIFE);
