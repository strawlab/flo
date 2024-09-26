#![no_std]
#![no_main]

// Pins in use:
//  PWM yaw: gpio6
//  PWM pitch: gpio7
//  square wave for illuminator flicker: gpio8

// Reserved Pins (do not use except for these assignments):
//  Pico (but not Pico W) LED: gpio25
//  I2C0 SDA: gpio4
//  I2C0 SCL: gpio5
//  Pico W wireless: gpio23
//  Pico W wireless: gpio24
//  Pico W wireless: gpio25
//  Pico W wireless: gpio29

use defmt_rtt as _;
use panic_probe as _;

fn hello_debugger() {
    use defmt::{debug, error, info, trace, warn};
    error!("error");
    warn!("warn");
    info!("info");
    debug!("debug");
    trace!("trace");
}

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [I2C0_IRQ])]
mod app {
    use super::*;
    use rp_pico::XOSC_CRYSTAL_FREQ;

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;

    use bbqueue::{BBBuffer, Consumer, Producer};
    use embedded_hal::{digital::v2::OutputPin, PwmPin};
    use fugit::ExtU64;
    use rp2040_hal::{
        self as hal,
        clocks::init_clocks_and_plls,
        pwm::{self, Slices as PWMSlices},
        timer::{monotonic::Monotonic, Alarm0},
        usb::UsbBus,
        watchdog::Watchdog,
        Sio,
    };

    use pwm_motor_types::{PwmDuration, PwmSerial, PwmState};

    const RX_BUF_SZ: usize = 512;

    pub struct PwmData {
        pwm_slice: pwm::Slice<pwm::Pwm3, pwm::FreeRunning>,
        pwm_clock_tick_period: f32,
    }

    #[shared]
    struct Shared {
        led: hal::gpio::Pin<
            hal::gpio::bank0::Gpio25,
            hal::gpio::FunctionSioOutput,
            hal::gpio::PullNone,
        >,
        serial: SerialPort<'static, UsbBus>,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[local]
    struct Local {
        pwms: PwmData,
        usb_dev: UsbDevice<'static, UsbBus>,
        rx_prod: Producer<'static, RX_BUF_SZ>,
        rx_cons: Consumer<'static, RX_BUF_SZ>,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        hello_debugger();
        let divider = 20;
        let top = 0xffff;
        let pwm_freq = 125e6 / (top as f64 * divider as f64); // 95.4 Hz
        defmt::info!(
            "Hello from {}. (DATATYPES_VERSION: {}, pwm_freq: {})",
            env!["CARGO_PKG_NAME"],
            pwm_motor_types::DATATYPES_VERSION,
            pwm_freq
        );
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();
        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets, &clocks);

        let usb_bus = c.local.usb_bus;
        usb_bus.replace(UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));
        let serial = SerialPort::new(usb_bus.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(usb_bus.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Straw Lab")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(2) // USB_CLASS_CDC
            .build();

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut led = pins.led.reconfigure();
        led.set_low().unwrap();

        //set up PWM
        let pwm_slices = PWMSlices::new(c.device.PWM, &mut resets);
        let mut pwm_slice = pwm_slices.pwm3;
        pwm_slice.default_config();
        pwm_slice.set_div_int(divider as u8);
        let pwm_clock_tick_period: f32 = divider as f32 / 125e6;
        pwm_slice.set_top(top);

        let mut pwms = PwmData {
            pwm_slice,
            pwm_clock_tick_period,
        };
        set_servos(
            &mut pwms,
            PwmState {
                pan: PwmDuration::default(),
                tilt: PwmDuration::default(),
                enabled: true,
            },
        );
        pwms.pwm_slice.channel_a.output_to(pins.gpio6);
        pwms.pwm_slice.channel_b.output_to(pins.gpio7);
        pwms.pwm_slice.enable();

        //flicker beam for event cam
        //(1500Hz square wave output on gpio8)
        {
            let mut mypwm = pwm_slices.pwm4;
            let divider = 2;
            let freq = 1500.0; //flicker frequency in hz
            let mytop: f64 = 125.0e6 / (f64::try_from(divider).unwrap()) / freq;
            assert!(mytop <= u16::MAX as f64);
            let mytop: u16 = mytop as u16;
            let myduty = mytop / 2;
            mypwm.default_config();
            mypwm.set_div_int(divider);
            mypwm.set_top(mytop);
            mypwm.channel_a.set_duty(myduty);
            mypwm.channel_a.output_to(pins.gpio8);
            mypwm.enable();
        }

        static BB: BBBuffer<RX_BUF_SZ> = BBBuffer::new();
        let (rx_prod, rx_cons) = BB.try_split().unwrap();

        let alarm = timer.alarm_0().unwrap();
        blink_led::spawn_after(500.millis()).unwrap();

        (
            Shared { led, serial },
            Local {
                pwms,
                usb_dev,
                rx_prod,
                rx_cons,
            },
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[idle(shared = [serial], local = [pwms,rx_cons])]
    fn idle(mut ctx: idle::Context) -> ! {
        let mut accum: heapless::String<RX_BUF_SZ> = heapless::String::new();

        loop {
            let grant = match ctx.local.rx_cons.read() {
                Ok(grant) => grant,
                Err(bbqueue::Error::InsufficientSize) => continue,
                Err(bbqueue::Error::GrantInProgress) => {
                    defmt::error!("GrantInProgress");
                    continue;
                }
                Err(bbqueue::Error::AlreadySplit) => {
                    defmt::error!("AlreadySplit");
                    continue;
                }
            };
            let mut used = 0;
            for ch in grant.buf().iter() {
                used += 1;
                match ch {
                    b'\n' | b'\r' => {
                        if !accum.is_empty() {
                            let (msg, consumed) =
                                match serde_json_core::from_str::<PwmSerial>(&accum) {
                                    Ok((msg, consumed)) => (msg, consumed),
                                    Err(e) => {
                                        use core::fmt::Write;

                                        let mut err_str: heapless::String<100> =
                                            heapless::String::new();
                                        write!(err_str, "{e}").unwrap();

                                        defmt::error!(
                                            "error decoding JSON: \"{=str}\" {=str}",
                                            accum,
                                            err_str
                                        );
                                        accum.clear();
                                        break;
                                    }
                                };
                            defmt::debug!("parsed message {=str}", accum);
                            assert_eq!(consumed, accum.len());
                            accum.clear();
                            match msg {
                                PwmSerial::Set(pt) => {
                                    set_servos(ctx.local.pwms, pt);
                                }
                                PwmSerial::VersionRequest => ctx.shared.serial.lock(|serial| {
                                    defmt::info!(
                                        "VersionRequest received. Reponding with version."
                                    );
                                    serial
                                        .write(pwm_motor_types::VERSION_RESPONSE_JSON_NEWLINE)
                                        .unwrap();
                                }),
                                PwmSerial::VersionResponse(_version) => {
                                    defmt::error!("ignoring reponding VersionResponse message");
                                }
                            }
                        }
                    }
                    ch => {
                        let chu8: u8 = *ch;
                        let mychar: char = chu8.into();
                        accum.push(mychar).unwrap();
                    }
                }
            }
            defmt::trace!("releasing {} bytes", used);
            grant.release(used);
        }
    }

    /// Set the servo positions via setting appropriate PWM pulse width.
    ///
    /// ## servos (HS-422):
    /// - full range of motion = 0.5 to 2.7 ms pulse width
    /// - +1 ms is approximately +90 degrees of rotation
    fn set_servos(pwms: &mut PwmData, command: PwmState) {
        defmt::debug!(
            "setting servos: pan {}, tilt {}, enabled {}",
            command.pan.duration_usec,
            command.tilt.duration_usec,
            command.enabled
        );
        if command.enabled {
            let pulse_dur_s_yaw: //pulse duration in seconds
            f32 = command.pan.duration_usec as f32*1e-6;
            let pulse_dur_s_pitch: //pulse duration in seconds
            f32 = command.tilt.duration_usec as f32*1e-6;
            pwms.pwm_slice
                .channel_a
                .set_duty((pulse_dur_s_yaw / pwms.pwm_clock_tick_period) as u16);
            pwms.pwm_slice
                .channel_b
                .set_duty((pulse_dur_s_pitch / pwms.pwm_clock_tick_period) as u16);
        }
    }

    #[task(binds=USBCTRL_IRQ, shared = [serial], local=[usb_dev, rx_prod])]
    fn on_usb(ctx: on_usb::Context) {
        let mut serial = ctx.shared.serial;
        let usb_dev = ctx.local.usb_dev;
        let rx_prod = ctx.local.rx_prod;
        serial.lock(|serial| {
            if !usb_dev.poll(&mut [&mut *serial]) {
                return;
            }
            let mut grant = rx_prod.grant_exact(128).unwrap();
            match serial.read(grant.buf()) {
                Ok(sz) => {
                    grant.commit(sz);
                    defmt::trace!("committed {} bytes", sz);
                }
                Err(usb_device::UsbError::WouldBlock) => {}
                Err(e) => {
                    panic!("usb error: {:?}", e);
                }
            }
        })
    }

    #[task(
        shared = [led],
        local = [tog: bool = true],
    )]
    fn blink_led(mut c: blink_led::Context) {
        if *c.local.tog {
            c.shared.led.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.led.lock(|l| l.set_low().unwrap());
        }
        *c.local.tog = !*c.local.tog;

        blink_led::spawn_after(500.millis()).unwrap();
    }
}
