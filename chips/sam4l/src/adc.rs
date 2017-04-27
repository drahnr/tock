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
use core::cmp;
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
pub const BASE_ADDRESS: *mut AdcRegisters = 0x40038000 as *mut AdcRegisters;

//XXX: TESTING
pub static mut TEST_BUFFER: [u8; 64] = [0; 64];

pub struct Adc {
    registers: *mut AdcRegisters,
    enabled: Cell<bool>,
    converting: Cell<bool>,
    adc_clk_freq: Cell<u32>,
    client: Cell<Option<&'static hil::adc::Client>>,
}

pub static mut ADC: Adc = Adc::new(BASE_ADDRESS);

impl Adc {
    const fn new(base_address: *mut AdcRegisters) -> Adc {
        Adc {
            registers: base_address,
            enabled: Cell::new(false),
            converting: Cell::new(false),
            adc_clk_freq: Cell::new(0),
            client: Cell::new(None),
        }
    }

    pub fn set_client<C: hil::adc::Client>(&self, client: &'static C) {
        self.client.set(Some(client));
    }

    pub fn handle_interrupt(&mut self) {
        let regs: &mut AdcRegisters = unsafe { mem::transmute(self.registers) };
        let status = regs.sr.get();

        if status & 0x0E != 0x00 {
            panic!("Other ADC interrupt: {:#X}", status);
            //regs.scr.set(0x0E);
        }

        if status & 0x51000000 != 0x51000000 {
            panic!("Bad status? {:#X}", status);
        }

        // sequencer end of conversion (sample complete)
        if status & 0x01 == 0x01 {
            let val = (regs.lcv.get() & 0xffff) as u16;

            // trigger GPIO pin
            unsafe{
                use gpio;
                gpio::PB[14].toggle();
                gpio::PB[14].toggle();
            }

            regs.scr.set(0x00000001);
        }
    }
}

impl adc::AdcSingle for Adc {
    fn initialize(&self) -> ReturnCode {
        let regs: &mut AdcRegisters = unsafe { mem::transmute(self.registers) };

        if !self.enabled.get() {
            self.enabled.set(true);

            // First, enable the clocks
            // Both the ADCIFE clock and GCLK10 are needed
            let mut clock_divisor = 0;
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
                self.adc_clk_freq.set(cpu_frequency / (1 << (clock_divisor+2)));
                debug!("{} {} {} {}", cpu_frequency, divisor, clock_divisor, self.adc_clk_freq.get());
            }

            // configure the ADC
            let cfg_val = (clock_divisor << 8) | // PRESCAL: clock divider
                          (0x1 << 6) | // CLKSEL: use ADCIFE clock
                          (0x0 << 4) | // SPEED: maximum 300 ksps
                          (0x4 << 1);  // REFSEL: VCC/2 reference
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
                    //XXX: replace with a returncode
                    panic!("ADC never enabled!");
                }
            }

            // enable Bandgap buffer and Reference buffer. I don't actually
            // know what these do, but you need to turn them on
            let cr_val = (0x1 << 10) | // BGREQEN: Enable bandgap buffer request
                         (0x1 <<  4);  // REFBUFEN: Enable reference buffer
            regs.cr.set(cr_val);

            // wait until buffers are enabled
            timeout = 100000;
            while regs.sr.get() & (0x51000000) != 0x51000000 {
                timeout -= 1;
                if timeout == 0 {
                    //XXX: replace with a returncode
                    panic!("ADC buffers never enabled!");
                }
            }
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

            let cfg = (0x7 << 20) | // MUXNEG: ground pad
                      ((channel as u32) << 16) | // MUXPOS: ADC channel
                      (0x2 << 14) | // INTERNAL: internal neg, external pos
                      (0x0 << 12) | // RES: 12-bit resolution
                      (0x0 <<  8) | // TRGSEL: software trigger
                      (0x0 <<  7) | // GCOMP: no gain compensation
                      (0x7 <<  4) | // GAIN: 0.5x gain
                      (0x0 <<  2) | // BIPOLAR: unipolar mode
                      (0x0 <<  0);  // HWLA: right justify value
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

impl adc::AdcContinuous for Adc {
    fn sample_continuous(&self, channel: u8, frequency: u32, buf: &'static [u8]) -> ReturnCode {
        let regs: &mut AdcRegisters = unsafe { mem::transmute(self.registers) };

        if !self.enabled.get() {
            ReturnCode::EOFF

        } else if channel > 14 {
            // valid channels are 0-14 only
            ReturnCode::EINVAL

        } else if self.converting.get() {
            // only one sample at a time
            ReturnCode::EBUSY

        } else if frequency == 0 || frequency > 250000 {
            // can't sample faster than the max sampling frequency
            ReturnCode::EINVAL

        } else {
            self.converting.set(true);

            //XXX: testing, remove
            unsafe{
                use gpio;
                gpio::PB[14].clear();
            }

            let cfg = (0x7 << 20) | // MUXNEG: ground pad
                      ((channel as u32) << 16) | // MUXPOS: ADC channel
                      (0x2 << 14) | // INTERNAL: internal neg, external pos
                      (0x0 << 12) | // RES: 12-bit resolution
                      (0x1 <<  8) | // TRGSEL: internal timer trigger
                      (0x0 <<  7) | // GCOMP: no gain compensation
                      (0x7 <<  4) | // GAIN: 0.5x gain
                      (0x0 <<  2) | // BIPOLAR: unipolar mode
                      (0x0 <<  0);  // HWLA: right justify value
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

            // enable timer interrupt
            //regs.ier.set(0x20);

            //XXX: testing, enable all interrupts
            regs.ier.set(0x0F);

            // start timer
            regs.cr.set(0x04);

            ReturnCode::SUCCESS
        }
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
/*
#[no_mangle]
#[allow(non_snake_case)]
#[allow(unused_imports)]
pub unsafe extern fn adcife_handler() {
    use kernel::common::Queue;
    use chip;

    let regs: &mut AdcRegisters = unsafe { mem::transmute(BASE_ADDRESS) };
    let status = regs.sr.get();

    if status & 0x02 == 0x02 || status & 0x04 == 0x04 || status & 0x08 == 0x08 {
        panic!("Other ADC interrupt: {:#X}", status);
        //regs.scr.set(0x0E);
    }

    if status & 0x53000000 != 0x53000000 {
        panic!("Bad status? {:#X}", status);
    }

    // timer sampling
    if status & 0x20 == 0x20 {
        //let val = (regs.lcv.get() & 0xffff) as u16;

        //// trigger GPIO pin
        //unsafe{
        //    use gpio;
        //    gpio::PB[14].toggle();
        //}

        // looks like we don't need to restart the timer manually
        //regs.cr.set(4);

        regs.scr.set(0x00000020);

    }

    if status & 0x01 == 0x01 {
        let val = (regs.lcv.get() & 0xffff) as u16;
        //panic!("ADC Val: {:#X}", val);

        // trigger GPIO pin
        unsafe{
            use gpio;
            gpio::PB[14].toggle();
        }

        regs.scr.set(0x00000001);
    }

    // // multi sampling
    // // Make sure this is the SEOC (Sequencer end-of-conversion) interrupt
    // let status = regs.sr.get();
    // if status & 0x01 == 0x01 {
    //     let val = (regs.lcv.get() & 0xffff) as u16;
    //
    //        // trigger GPIO pin
    //        unsafe{
    //            use gpio;
    //            gpio::PB[14].toggle();
    //        }
    //
    //        regs.scr.set(0xFFFFFFFF);
    //    }
    //    //} else {
    //
    //        //let nvic = nvic::NvicIdx::ADCIFE;
    //        //nvic::disable(nvic);
    //        //chip::INTERRUPT_QUEUE.as_mut().unwrap().enqueue(nvic);
    //    //}
}
*/
