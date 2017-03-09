// adc.rs -- Implementation of SAM4L ADCIFE.
//
// This is a bare-bones implementation of the SAM4L ADC. It is bare-bones
// because it provides little flexibility on how samples are taken. Currently,
// all samples
//   - are 12 bits
//   - use the ground pad as the negative reference
//   - use a VCC/2 positive reference
//   - are right justified
//
// Author: Philip Levis <pal@cs.stanford.edu>
// Date: August 5, 2015
//

use core::cell::Cell;
use core::mem;
use kernel::common::math;
use kernel::common::volatile_cell::VolatileCell;
use kernel::hil;
use kernel::hil::adc;
use kernel::returncode::ReturnCode;
use nvic;
use pm::{self, Clock, PBAClock};
use scif;

#[repr(C, packed)]
pub struct AdcRegisters {
    // From page 1005 of SAM4L manual
    cr: VolatileCell<u32>, // Control               (0x00)
    cfg: VolatileCell<u32>, // Configuration        (0x04)
    sr: VolatileCell<u32>, // Status                (0x08)
    scr: VolatileCell<u32>, // Status clear         (0x0c)
    pad: VolatileCell<u32>, // padding/reserved
    seqcfg: VolatileCell<u32>, // Sequencer config  (0x14)
    cdma: VolatileCell<u32>, // Config DMA          (0x18)
    tim: VolatileCell<u32>, // Timing config        (0x1c)
    itimer: VolatileCell<u32>, // Internal timer    (0x20)
    wcfg: VolatileCell<u32>, // Window config       (0x24)
    wth: VolatileCell<u32>, // Window threshold     (0x28)
    lcv: VolatileCell<u32>, // Last converted value (0x2c)
    ier: VolatileCell<u32>, // Interrupt enable     (0x30)
    idr: VolatileCell<u32>, // Interrupt disable    (0x34)
    imr: VolatileCell<u32>, // Interrupt mask       (0x38)
    calib: VolatileCell<u32>, // Calibration        (0x3c)
    version: VolatileCell<u32>, // Version          (0x40)
    parameter: VolatileCell<u32>, // Parameter      (0x44)
}

// Page 59 of SAM4L data sheet
const BASE_ADDRESS: *mut AdcRegisters = 0x40038000 as *mut AdcRegisters;

pub struct Adc {
    registers: *mut AdcRegisters,
    enabled: Cell<bool>,
    converting: Cell<bool>,
    client: Cell<Option<&'static hil::adc::Client>>,
}

pub static mut ADC: Adc = Adc::new(BASE_ADDRESS);

impl Adc {
    const fn new(base_address: *mut AdcRegisters) -> Adc {
        Adc {
            registers: base_address,
            enabled: Cell::new(false),
            converting: Cell::new(false),
            client: Cell::new(None),
        }
    }

    pub fn set_client<C: hil::adc::Client>(&self, client: &'static C) {
        self.client.set(Some(client));
    }

    pub fn handle_interrupt(&mut self) {
        let regs: &mut AdcRegisters = unsafe { mem::transmute(self.registers) };

        // Make sure this is the SEOC (Sequencer end-of-conversion) interrupt
        let status = regs.sr.get();
        if status & 0x01 == 0x01 {
            // conversion complete
            self.converting.set(false);

            // Clear SEOC interrupt
            regs.scr.set(0x00000001);

            // Disable SEOC interrupt
            regs.idr.set(0x00000001);

            // Read the value from the LCV register.
            // The sample is 16 bits wide
            let val = (regs.lcv.get() & 0xffff) as u16;
            self.client.get().map(|client| { client.sample_done(val); });
        } else if status & 0x20 == 0x20 {
            //XXX: TESTING
            // this is the timer time-out interrupt. Toggle a GPIO to check that we're setting the
            // frequency right
        }
    }
}

impl adc::AdcSingle for Adc {
    fn initialize(&self) -> ReturnCode {
        let regs: &mut AdcRegisters = unsafe { mem::transmute(self.registers) };

        if !self.enabled.get() {
            self.enabled.set(true);

            // This logic is from 38.6.1 "Initializing the ADCIFE" of
            // the SAM4L data sheet
            // 1. Start the clocks, ADC uses GCLK10, choose to
            // source it from CLK_CPU (whichever clock the CPU is using)
            unsafe {
                pm::enable_clock(Clock::PBA(PBAClock::ADCIFE));
                nvic::enable(nvic::NvicIdx::ADCIFE);

                // the clock used for the ADC must conform to 300000 >= F(clk)/6 so, in practice
                // F(clk) < 1.8 MHz.
                // To pick the correct clock divider, we solve F(clk)/2^(N+1) <= 1800000 for N,
                // resulting in the smallest divider that will make the ADC work.
                // This comes out to log_2(F(clk)/1800000) - 1 <= N, which is not enough.
                // But getting the closest power of two above F(clk)/1800000 works out such that
                // our result is >= N, which is what we want.
                // Also the biggest divider we can use is 8-bit, so cap it to that
                let cpu_frequency = pm::get_system_frequency();
                let divisor = (cpu_frequency + (1800000 - 1)) / 1800000; // ceiling of division
                let clock_divisor = (math::log_base_two(math::closest_power_of_two(divisor)) -
                                     1) as u8;
                scif::generic_clock_enable_divided(scif::GenericClock::GCLK10,
                                                   scif::ClockSource::CLK_CPU,
                                                   clock_divisor as u16);
            }

            // 2. Insert a fixed delay
            for _ in 0..100000 {
                let _ = regs.cr.get();
            }

            // 3, Enable the ADC
            let mut cr: u32 = regs.cr.get();
            cr |= 1 << 8;
            regs.cr.set(cr);

            // 4. Wait until ADC ready
            while regs.sr.get() & (1 << 24) == 0 {}

            // 5. Turn on bandgap and reference buffer
            let cr2: u32 = (1 << 10) | (1 << 8) | (1 << 4);
            regs.cr.set(cr2);

            // 6. Configure the ADCIFE
            // Setting below in the configuration register sets
            //   - the clock divider to be 4,
            //   - the source to be the Generic clock,
            //   - the max speed to be 300 ksps, and
            //   - the reference voltage to be VCC/2
            regs.cfg.set(0x00000008);
            while regs.sr.get() & (0x51000000) != 0x51000000 {}
        }

        ReturnCode::SUCCESS
    }

    fn sample(&self, channel: u8) -> ReturnCode {
        let regs: &mut AdcRegisters = unsafe { mem::transmute(self.registers) };

        if !self.enabled.get() {
            ReturnCode::EOFF

        } else if channel > 14 {
            // valid channels are 0-14 only
            ReturnCode::EINVAL

        } else if self.converting.get() {
            // only one sample at a time
            ReturnCode::EBUSY

        } else {
            self.converting.set(true);

            // This configuration sets the ADC to use Pad Ground as the
            // negative input, and the ADC channel as the positive. Since
            // this is a single-ended sample, the bipolar bit is set to zero.
            // Trigger select is set to zero because this denotes a software
            // sample. Gain is 0.5x (set to 111). Resolution is set to 12 bits
            // (set to 0).

            let chan_field: u32 = (channel as u32) << 16; // MUXPOS
            let mut cfg: u32 = chan_field;
            cfg |= 0x00700000; // MUXNEG   = 111 (ground pad)
            cfg |= 0x00008000; // INTERNAL =  10 (int neg, ext pos)
            cfg |= 0x00000000; // RES      =   0 (12-bit)
            cfg |= 0x00000000; // TRGSEL   =   0 (software)
            cfg |= 0x00000000; // GCOMP    =   0 (no gain error corr)
            cfg |= 0x00000070; // GAIN     = 111 (0.5x gain)
            cfg |= 0x00000000; // BIPOLAR  =   0 (not bipolar)
            cfg |= 0x00000000; // HWLA     =   0 (no left justify value)
            regs.seqcfg.set(cfg);

            // Enable end of conversion interrupt
            regs.ier.set(1);

            // Initiate conversion
            regs.cr.set(8);

            ReturnCode::SUCCESS
        }
    }

    fn cancel_sample(&self) -> ReturnCode {
        ReturnCode::FAIL
    }
}

impl adc::AdcContinuous for Adc {
    fn sample_continuous(&self, channel: u8, frequency: u32, buf: &'static [u8]) -> ReturnCode {
        // configure ADC

        // configure ADC timer

        // configure DMA transfer

        // begin sampling

        ReturnCode::FAIL
    }

    fn cancel_sampling(&self) -> ReturnCode {
        // check if running

        // stop ADC

        // cancel DMA transfer

        // call buffer_full as if dma completed with proper size

        ReturnCode::FAIL
    }
}

interrupt_handler!(adcife_handler, ADCIFE);
