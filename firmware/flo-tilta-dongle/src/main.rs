#![no_std]
#![no_main]

// useful links
// https://docs.embassy.dev/embassy-nrf/git/nrf52840/index.html

const MAX_USB_PACKET_SIZE: u16 = 64; //has to be one of 8, 16, 32 or 64.
const RX_BUF_SZ: usize = 512;
const RF_SEND_PERIOD: f32 = 0.001; //[seconds] how often to send positions to tilta during ramps
const RAMP_RATE: f32 = 4096.0 / 0.4; //[pos/s] where pos is tilta position - 0 to 4095 for full range
const SLEEP_AFTER: u64 = 2; //stop radio emissions after this many seconds of not updating the value

use tilta_dongle_comms::TitlaFocusDongleMessage as UsbMessage;

use heapless;
use num_traits::float::FloatCore;
use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::radio::ieee802154::{self, Packet as RfPacket};
use embassy_nrf::usb;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder as UsbBuilder, Config as UsbConfig, UsbDevice};

use embassy_nrf::bind_interrupts;
bind_interrupts!(struct Irqs{
    RADIO => embassy_nrf::radio::InterruptHandler<embassy_nrf::peripherals::RADIO>;
    USBD => usb::InterruptHandler<embassy_nrf::peripherals::USBD>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
});

type MyDriver =
    usb::Driver<'static, embassy_nrf::peripherals::USBD, usb::vbus_detect::HardwareVbusDetect>;
type MyRadio = ieee802154::Radio<'static, embassy_nrf::peripherals::RADIO>;

struct RadioLoopData {
    rf: MyRadio,
    ramp_led: Output<'static>,
    radio_led: Output<'static>,
}

#[derive(Debug, Clone)]
enum InternalMsg {
    //sender = usb loop, receiver = radio loop
    SetPos { pos: i32 },
}

static INTERNAL_CHANNEL: Signal<ThreadModeRawMutex, InternalMsg> = Signal::new(); //sender = usb loop, receiver = radio loop

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut cfg: embassy_nrf::config::Config = Default::default();
    cfg.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    cfg.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;
    cfg.dcdc.reg0 = true;
    cfg.dcdc.reg1 = true;

    let p = embassy_nrf::init(cfg);

    {
        //set dcdc voltage (otherwise,  after erase+reset, the dongle will default to 1.8V-supply which is not suitable for programming
        const REGOUT0: *mut u32 = (0x10001000 + 0x304) as *mut u32;
        unsafe {
            let r = uicr_write_masked(REGOUT0, 5, 0b00000000_00000000_00000000_00000111);
            if r == WriteResult::Failed {
                defmt::error!("setting buck to 3.3v failed to write REGOUT register");
            }
        }
    }

    let mut rf: MyRadio = ieee802154::Radio::new(p.RADIO, Irqs);
    rf.set_channel(12);
    rf.set_transmission_power(2); //+2dbm?

    let (usb, cdc_class) = {
        let usb_driver = usb::Driver::new(
            p.USBD,
            Irqs,
            usb::vbus_detect::HardwareVbusDetect::new(Irqs),
        );

        // Create embassy-usb Config
        let mut config = UsbConfig::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Strawlab");
        config.product = Some("flo-tilta-dongle");
        config.serial_number = Some("001");
        config.max_power = 100;
        config.max_packet_size_0 = 64;

        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;

        static STATE: StaticCell<CdcState> = StaticCell::new();
        let cdc_state = STATE.init(CdcState::new());

        // Create embassy-usb DeviceBuilder using the driver and config.
        static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
        static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
        static MSOS_DESC: StaticCell<[u8; 128]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
        let mut builder = UsbBuilder::new(
            usb_driver,
            config,
            &mut DEVICE_DESC.init([0; 256])[..],
            &mut CONFIG_DESC.init([0; 256])[..],
            &mut BOS_DESC.init([0; 256])[..],
            &mut MSOS_DESC.init([0; 128])[..],
            &mut CONTROL_BUF.init([0; 128])[..],
        );

        // Create classes on the builder.
        let cdc_class = CdcAcmClass::new(&mut builder, cdc_state, MAX_USB_PACKET_SIZE);

        // Build the builder.
        let usb = builder.build();
        (usb, cdc_class)
    };

    let ramp_led = Output::new(p.P1_09, Level::High, OutputDrive::Standard);
    let radio_led = Output::new(p.P0_12, Level::High, OutputDrive::Standard);

    spawner.spawn(usb_task(usb)).expect("usb task");
    spawner
        .spawn(usb_connect_task(cdc_class))
        .expect("usb parse task");
    spawner
        .spawn(radio_loop(RadioLoopData {
            rf,
            ramp_led,
            radio_led,
        }))
        .expect("radio task");

    let mut blinker = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);
    loop {
        blinker.set_low();
        //INTERNAL_CHANNEL.try_send(InternalMsg::SetPos { pos: (0) }).unwrap();
        Timer::after(Duration::from_millis(1000)).await;

        blinker.set_high();
        //INTERNAL_CHANNEL.try_send(InternalMsg::SetPos { pos: (1000) }).unwrap();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, MyDriver>) {
    device.run().await;
}

#[embassy_executor::task]
async fn usb_connect_task(mut class: CdcAcmClass<'static, MyDriver>) {
    loop {
        class.wait_connection().await;
        defmt::info!("Connected");
        let _ = usb_parse_loop(&mut class).await;
        defmt::info!("Disconnected");
    }
}

async fn usb_parse_loop(class: &mut CdcAcmClass<'static, MyDriver>) -> Result<(), Disconnected> {
    let mut buf = [0; MAX_USB_PACKET_SIZE as usize];
    let mut accum: heapless::String<RX_BUF_SZ> = heapless::String::new();
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        for ch in data.iter() {
            match ch {
                b'\n' | b'\r' => {
                    if !accum.is_empty() {
                        let (msg, consumed) = match serde_json_core::from_str::<UsbMessage>(&accum)
                        {
                            Ok((msg, consumed)) => (msg, consumed),
                            Err(e) => {
                                use core::fmt::Write;

                                let mut err_str: heapless::String<100> = heapless::String::new();
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
                            UsbMessage::SetPos(pos) => {
                                if pos >= 0 && pos <= 4095 {
                                    INTERNAL_CHANNEL.signal(InternalMsg::SetPos { pos });
                                } else {
                                    defmt::error!("position out of range: {}", pos);
                                }
                            }
                            UsbMessage::VersionRequest => {
                                defmt::info!("VersionRequest received. Reponding with version.");
                                class
                                    .write_packet(UsbMessage::VERSION_RESPONSE_JSON_NEWLINE)
                                    .await
                                    .unwrap_or_else(|e| {
                                        defmt::error!("failed to send version response: {:?}", e);
                                    });
                            }
                            UsbMessage::VersionResponse(_version) => {
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
    }
}

#[embassy_executor::task]
async fn radio_loop(mut ctx: RadioLoopData) {
    let mut p = RfPacket::new();
    p.copy_from_slice(b"\x61\x88\x00\xE7\x01\xFF\xFF\x96\xF0\x44\x05\x00\xB7");

    let update_packet = |val: i32, p: &mut RfPacket| {
        assert!(val >= 0);
        assert!(val <= 0xFFF);
        let val = val as u32;
        p[2] = p[2].wrapping_add(1); //increment packet number
        p[9] = 0x40 | ((val >> 8) as u8 & 0x0F);
        p[10] = val as u8 & 0xFF;
        p[12] = 0u8
            .wrapping_sub(p[9])
            .wrapping_sub(p[10])
            .wrapping_sub(p[11]);
    };

    let mut last_pos: i32 = 0;
    let mut target_pos: i32 = 0;
    let mut t_last_val = embassy_time::Instant::now();

    loop {
        if let Some(InternalMsg::SetPos { pos }) = {
            if t_last_val.elapsed().as_secs() > SLEEP_AFTER {
                defmt::info!("suspending rf");
                let msg = INTERNAL_CHANNEL.wait().await;
                defmt::info!("waking rf");
                Some(msg)
            } else {
                INTERNAL_CHANNEL.try_take()
            }
        } {
            target_pos = pos;
            t_last_val = embassy_time::Instant::now();
        }

        //ramp
        let dx_lim = (RAMP_RATE * RF_SEND_PERIOD).ceil() as i32;
        let dx = (target_pos - last_pos).clamp(-dx_lim, dx_lim);
        ctx.ramp_led.set_level((dx == 0).into());

        last_pos += dx;

        //transmit
        update_packet(last_pos, &mut p);
        ctx.radio_led.set_low();
        match ctx.rf.try_send(&mut p).await {
            Ok(()) => {}
            Err(e) => {
                defmt::error!("transmission failed: {:?}", e);
            }
        }
        ctx.radio_led.set_high();

        Timer::after(duration_secs(RF_SEND_PERIOD)).await;
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"), //don't panic?
            EndpointError::Disabled => Disconnected {},
        }
    }
}

//uicr_write_masked copied from embassy/embassy-nrf/src/lib.rs

#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
enum WriteResult {
    /// Word was written successfully, needs reset.
    Written,
    /// Word was already set to the value we wanted to write, nothing was done.
    Noop,
    /// Word is already set to something else, we couldn't write the desired value.
    Failed,
}

unsafe fn uicr_write_masked(address: *mut u32, value: u32, mask: u32) -> WriteResult {
    let curr_val = address.read_volatile();
    if curr_val & mask == value & mask {
        return WriteResult::Noop;
    }

    // We can only change `1` bits to `0` bits.
    if curr_val & value & mask != value & mask {
        return WriteResult::Failed;
    }

    use embassy_nrf::pac;
    let nvmc = &*pac::NVMC::ptr();
    nvmc.config.write(|w| w.wen().wen());
    while nvmc.ready.read().ready().is_busy() {}
    address.write_volatile(value | !mask);
    while nvmc.ready.read().ready().is_busy() {}
    nvmc.config.reset();
    while nvmc.ready.read().ready().is_busy() {}

    WriteResult::Written
}

///secs must be positive!! (Duration is unsigned, pfft!)
fn duration_secs(secs: f32) -> Duration {
    if secs > 0.0 {
        Duration::from_nanos((secs * 1e9) as u64)
    } else {
        Duration::from_ticks(0)
    }
}
