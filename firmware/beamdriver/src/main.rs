#![no_std]
#![no_main]

// Pins in use:
//  gp6: PWM for pan servo
//  gp7: PWM for tilt servo
//  gp8: connected to led driver, high = beam on
//  gp18: cam0 line1 (not used but connected)
//  gp19: cam1 line1 (not used but connected)
//  gp20: cam0 line2, triggers the cam
//  gp21: cam1 line2, triggers the cam
//  gp22: diode-OR'ed line3 from cam0 and cam1, should be configured as ExposureActive output (active high) on the cams.

// Reserved Pins (do not use except for these assignments):
//  Pico (but not Pico W) LED: gpio25
//  I2C0 SDA: gpio4
//  I2C0 SCL: gpio5
//  Pico W wireless: gpio23
//  Pico W wireless: gpio24
//  Pico W wireless: gpio25
//  Pico W wireless: gpio29

use defmt_rtt as _;
use panic_reset as _; //it is important to reset asap so that the beam driver/led don't burn out

use embedded_hal::{
    digital::v2::InputPin,
    digital::v2::OutputPin,
    timer::{Cancel, CountDown},
    PwmPin,
};
use hal::pac;
use rp2040_hal::{self as hal, pwm::Slices as PWMSlices, Sio};
use rp_pico::XOSC_CRYSTAL_FREQ;

use fugit::MicrosDurationU32;

const MAX_BEAM_DURATION_US: u32 = 500;
const FRAMERATE: f64 = 150.0;
const FRAMERATE_CLOCK_DIVIDER: u8 = 20; //divider should be high enough to allow 16-bit counting. If 125.0e6/(divider * framerate) is < 65536 then we're good.

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let _core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let _clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut beam = pins.gpio8.into_push_pull_output();
    beam.set_low().unwrap();

    let exposure_active = pins.gpio22.into_pull_down_input();

    //set up trigger generator
    let pwm_slices = PWMSlices::new(pac.PWM, &mut pac.RESETS);
    let mut trigpwm = pwm_slices.pwm2;
    let divider = FRAMERATE_CLOCK_DIVIDER; //divider should be high enough to not let mytop exceed 65535
    let freq = FRAMERATE; //flicker frequency in hz
    let mytop = (125.0e6 / (divider as f64) / freq) as u16 - 1;
    let myduty = mytop / 2;
    trigpwm.default_config();
    trigpwm.set_div_int(divider);
    trigpwm.set_top(mytop);
    trigpwm.channel_a.set_duty(myduty);
    trigpwm.channel_a.output_to(pins.gpio20);
    trigpwm.channel_b.set_duty(myduty);
    trigpwm.channel_b.output_to(pins.gpio21);
    trigpwm.enable();

    //beam duration limiter timer
    let tt = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut timer = tt.count_down();

    //state machine
    enum MyState {
        Idle,     // exposure low, beam off
        Exposure, // exposure high, beam on
        Limit,    // exposure high and has been high for too long, beam off
    }
    let mut state = MyState::Idle;

    loop {
        if exposure_active.is_high().unwrap() {
            match state {
                MyState::Idle => {
                    beam.set_high().unwrap();
                    state = MyState::Exposure;
                    timer.start(MicrosDurationU32::micros(MAX_BEAM_DURATION_US));
                }
                MyState::Exposure => {
                    match timer.wait() {
                        Ok(()) => {
                            //timeout. Abort exposure
                            beam.set_low().unwrap();
                            //timer.cancel();
                            state = MyState::Limit;
                        }
                        _ => {}
                    }
                }
                MyState::Limit => {
                    //do nothing, wait for exposure_active to go low
                }
            }
        } else {
            match state {
                MyState::Idle => {
                    //do nothing, remain idle
                }
                MyState::Exposure => {
                    //exposure ended without reaching the limit.
                    beam.set_low().unwrap();
                    timer.cancel().unwrap();
                    state = MyState::Idle;
                }
                MyState::Limit => {
                    //exposure ended, but was too long.
                    state = MyState::Idle;
                }
            }
        }
    }
}
