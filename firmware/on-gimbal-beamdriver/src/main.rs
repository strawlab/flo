#![no_std]
#![no_main]

// Pins in use:
//  gp26: -> camera trigger
//  gp27: -> beam driver enable (not actual beam signal, only inhibits beam if low)

use defmt_rtt as _;
use panic_reset as _; //it is important to reset asap so that the beam driver/led don't burn out

use embedded_hal::{
    //digital::v2::InputPin,
    digital::v2::OutputPin,
    //timer::{Cancel, CountDown},
    PwmPin,
};
use hal::pac;
use rp2040_hal::{self as hal, pwm::Slices as PWMSlices, Sio};
use rp_pico::XOSC_CRYSTAL_FREQ;

//use fugit::MicrosDurationU32;

const FRAMERATE: f64 = 100.0;
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

    let mut beam_en = pins.gpio27.into_push_pull_output();
    beam_en.set_high().unwrap();

    //set up trigger generator
    let pwm_slices = PWMSlices::new(pac.PWM, &mut pac.RESETS);
    let mut trigpwm = pwm_slices.pwm5;
    let divider = FRAMERATE_CLOCK_DIVIDER; //divider should be high enough to not let mytop exceed 65535
    let freq = FRAMERATE; //flicker frequency in hz
    let mytop = (125.0e6 / (divider as f64) / freq) as u16 - 1;
    let myduty = mytop / 2;
    trigpwm.default_config();
    trigpwm.set_div_int(divider);
    trigpwm.set_top(mytop);
    trigpwm.channel_a.set_duty(myduty);
    trigpwm.channel_a.output_to(pins.gpio26);
    trigpwm.enable();

    loop {}
}
