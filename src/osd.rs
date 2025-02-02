use color_eyre::eyre::{self, Result, WrapErr};
use futures::{SinkExt, StreamExt};
use std::{
    pin::Pin,
    sync::{Arc, Mutex},
};
use tokio_serial::SerialPortBuilderExt;

use flo_core::{
    drone_structs::{DroneEvent, FlightMode},
    elapsed,
    osd_structs::Align,
    Broadway, DeviceMode as TrackingMode, DroneStatus, OsdConfig, OsdState,
};
use multiwii_serial_protocol_v2::MspPacket;
use osd_displayport::{displayport_messages, msp_codec};
use osd_utils::OsdCache;

pub(crate) async fn run_osd_loop(
    mut flo: tokio::sync::watch::Receiver<OsdState>,
    broadway: Broadway,
    config: OsdConfig,
    canvas_arc: Arc<Mutex<OsdCache>>,
) -> Result<()> {
    let cal: flo_core::osd_structs::FpvCameraOSDCalibration = config
        .cal
        .ok_or_else(|| eyre::eyre!("OSD calibration required"))?;
    let cal: flo_core::osd_structs::LoadedFpvCameraOSDCalibration = cal.try_into()?;
    let mut drone_events = broadway.drone_events.subscribe();
    let (mut osd_tx, mut osd_rx): (_, Pin<Box<dyn futures::Stream<Item = _> + Send>>) =
        match config.port_path {
            Some(ser_path) => {
                const BAUD_RATE: u32 = 115_200;
                let serial_device = tokio_serial::new(&ser_path, BAUD_RATE)
                    .open_native_async()
                    .with_context(|| format!("Failed to open OSD serial device {}", ser_path))?;

                let (osd_tx, osd_rx) =
                    tokio_util::codec::Framed::new(serial_device, msp_codec::MspCodec::new_v1())
                        .split();
                (Some(osd_tx), Box::pin(osd_rx))
            }
            None => (None, Box::pin(futures::stream::pending())),
        };
    let mut heartbeat_tick = tokio::time::interval(std::time::Duration::from_secs_f64(1.0));
    let mut update_tick = tokio::time::interval(std::time::Duration::from_secs_f64(0.05));
    heartbeat_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    update_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    let mut drone_status: DroneStatus = Default::default();

    loop {
        #[allow(dead_code)]
        enum MyAction {
            Render,
            Heartbeat,
            MessageFromGoggles(MspPacket),
            DroneEvent(DroneEvent),
            Pass,
        }
        let my_action = tokio::select! {
            _ = update_tick.tick() => MyAction::Render,
            _ = heartbeat_tick.tick() => MyAction::Heartbeat,
            Some(r) = osd_rx.next() => {
                match r {
                    Ok(msg) => MyAction::MessageFromGoggles(msg),
                    Err(e) => {
                        tracing::error!("{e}");
                        MyAction::Pass
                    }
                }
            },
            drone_event = drone_events.recv() => {
                match drone_event {
                    Ok(msg) => MyAction::DroneEvent(msg),
                    Err(e) => {
                        tracing::error!("broadway error in osd: {e}");
                        MyAction::Pass
                    }
                }
            }
        };

        match my_action {
            MyAction::DroneEvent(evt) => match evt {
                DroneEvent::Armed => {
                    drone_status.armed = true;
                }
                DroneEvent::Disarmed => {
                    drone_status.armed = false;
                }
                DroneEvent::FlightModeChanged(_old_fm, new_fm) => {
                    drone_status.flight_mode = new_fm;
                }
                DroneEvent::BatteryState(bs) => {
                    drone_status.batt_percent = bs.batt_percent;
                    drone_status.batt_voltage = bs.batt_voltage;
                    drone_status.timestamp = bs.timestamp;
                }
                DroneEvent::CommLost => {
                    drone_status = DroneStatus {
                        armed: drone_status.armed, //keep assuming armed to not stop goggle video recording upon mavlink loss
                        ..Default::default()
                    }
                }
                _ => {}
            },
            MyAction::Pass => {}
            MyAction::Heartbeat => {
                if let Some(osd_tx) = osd_tx.as_mut() {
                    osd_tx
                        .send(displayport_messages::DisplayportMessage::Heartbeat.make_msp_packet())
                        .await?;

                    //send arming status message to vtx/goggles to make them go full power and start recording video
                    use multiwii_serial_protocol_v2::MspCommandCode as CC;
                    use packed_struct::prelude::PackedStruct;

                    let telemsg = multiwii_serial_protocol_v2::structs::MspStatus {
                        cycle_time: 0,
                        i2c_errors: 0,
                        sensors: multiwii_serial_protocol_v2::structs::MspAvailableSensors {
                            sonar: false,
                            gps: false,
                            mag: false,
                            baro: false,
                            acc: false,
                        },
                        null1: 0,
                        flight_mode: if drone_status.armed { 1 } else { 0 },
                        profile: 0,
                        system_load: 0,
                    };
                    osd_tx
                    .send(MspPacket {
                        cmd: CC::MSP_STATUS as u16,
                        direction:
                            multiwii_serial_protocol_v2::MspPacketDirection::FromFlightController,
                        data: telemsg.pack().to_vec(),
                    })
                    .await?;
                }
            }
            MyAction::Render => {
                let state = {
                    // Only in this scope do we keep the lock on `flo`.
                    flo.borrow_and_update().clone()
                };
                let packets = {
                    // Lock the canvas while we redraw it.
                    let mut canvas = canvas_arc.lock().unwrap();
                    canvas.clear();

                    canvas.print(b"FLO", 0, 0, Align::Left);

                    draw_battery(&mut canvas, &drone_status, -1, -1, Align::Right);

                    let fm_str = match drone_status.flight_mode {
                        FlightMode::Position => b"POS",
                        FlightMode::Hold => b"HLD",
                        FlightMode::Manual => b"MAN",
                        FlightMode::Altitude => b"ALT",
                        _ => b"???",
                    };
                    canvas.print(fm_str, -1, 0, Align::Right);

                    //display tracking/bee marker
                    if state.tracking_mode == TrackingMode::ClosedLoop {
                        //render a distance-dependent blob
                        //FIXME: add parallax correction here
                        let dist = if state.bee_dist.0.is_nan() || state.bee_dist.0 == 0.0 {
                            None
                        } else {
                            Some(state.bee_dist)
                        };
                        let (x, y) = cal.angles_to_px(
                            state.motor_state.pan_enc,
                            state.motor_state.tilt_enc,
                            dist,
                        );
                        let ((chx, chy), inscreen) =
                            cal.constrain_charpos_f(cal.px_to_charpos(x, y));
                        let (chx_i, chy_i) = (chx.round() as i32, chy.round() as i32);

                        let mut n: usize = if state.bee_dist.0.is_nan() || state.bee_dist.0 == 0.0 {
                            1
                        } else {
                            config.blob.convert(state.bee_dist)
                        };
                        if inscreen {
                            n += 1 //compensate for "do not render directly over bee"
                        };
                        let chars = osd_utils::char_blob(&cal, cal.charposf_to_px(chx, chy), n);
                        for (bchx, bchy) in chars {
                            if inscreen && bchx == chx_i && bchy == chy_i {
                                continue; //do not render directly over bee
                            }
                            let ch = osd_utils::arrow_towards(&cal, (bchx, bchy), (x, y));
                            canvas.draw_char(ch, bchx, bchy);
                        }

                        //draw numerical distance
                        let mut s = format!("{:.2}", state.bee_dist.0.clamp(-99.99, 99.99))
                            .to_ascii_uppercase()
                            .as_bytes()
                            .to_vec();
                        s.push(osd_utils::SYM_M); // add "m" for meters
                        canvas.print(&s, 0, cal.osd_char_h - 1, Align::Left);
                    } else {
                        //render a character indicating where flo is pointing
                        let ((chx, chy), _) = cal.constrain_charpos(cal.angles_to_charpos(
                            state.motor_state.pan_enc,
                            state.motor_state.tilt_enc,
                            None,
                        ));

                        let ch = match state.tracking_mode {
                            TrackingMode::AcquiringLock => b'?',
                            TrackingMode::ClosedLoop => unreachable!(),
                            TrackingMode::ManualOpenLoop => b'#',
                            TrackingMode::SuspendedClosedLoop => b'?',
                        };
                        canvas.draw_char(ch, chx, chy);
                    }

                    if osd_tx.is_some() {
                        // In case of true OSD, build the packets to send.
                        Some(canvas.make_packets())
                    } else {
                        // No need to compute packets for emulated OSD.
                        None
                    }
                };

                // We have now finished rendering the canvas.

                // In case of true OSD, finalize the packets and send out the serial port.
                if let Some(osd_tx) = osd_tx.as_mut() {
                    if let Some(ser_packets) = packets {
                        for pkt in ser_packets.into_iter() {
                            osd_tx.feed(pkt).await?;
                        }
                        osd_tx.flush().await?;
                    }
                }
            }
            MyAction::MessageFromGoggles(_) => {} //FIXME: parse and listen for canvas size
        }
    }
}

// fn draw(s: &[u8], chx: i32, chy: i32) -> Result<MspPacket> {
//     Ok(displayport_messages::DisplayportMessage::WriteString(
//         displayport_messages::WriteStringPayload::from_bytestring(s, chy as u8, chx as u8),
//     )
//     .make_msp_packet())
// }

fn draw_battery(
    canvas: &mut OsdCache,
    drone_status: &DroneStatus,
    chx: i32,
    chy: i32,
    align: Align,
) {
    let s = if elapsed(drone_status.timestamp) > 5.0 {
        // TODO: blink as warning
        b"NO SIG".to_vec()
    } else {
        let batt_char_f = (drone_status.batt_percent / 100.0 * 6.0)
            .round()
            .clamp(0.0, 6.0);
        let batt_char = osd_utils::SYM_BATT_FULL + 6 - batt_char_f as u8;
        let mut s = Vec::with_capacity(6);
        s.push(batt_char);
        s.extend(
            format!("{:.2}V", drone_status.batt_voltage.clamp(0.0, 9.99))
                .to_ascii_uppercase()
                .as_bytes(),
        );
        s
    };

    canvas.print(&s, chx, chy, align)
}

#[test]
fn test_packed_msp_status() {
    use packed_struct::prelude::PackedStruct;

    let telemsg_armed = multiwii_serial_protocol_v2::structs::MspStatus {
        cycle_time: 0,
        i2c_errors: 0,
        sensors: multiwii_serial_protocol_v2::structs::MspAvailableSensors {
            sonar: false,
            gps: false,
            mag: false,
            baro: false,
            acc: false,
        },
        null1: 0,
        flight_mode: 1, // armed
        profile: 0,
        system_load: 0,
    };
    let data_armed = telemsg_armed.pack();
    assert_eq!(&data_armed[..], &[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]);

    // flight_mode 0
    let telemsg_not_armed = multiwii_serial_protocol_v2::structs::MspStatus {
        cycle_time: 0,
        i2c_errors: 0,
        sensors: multiwii_serial_protocol_v2::structs::MspAvailableSensors {
            sonar: false,
            gps: false,
            mag: false,
            baro: false,
            acc: false,
        },
        null1: 0,
        flight_mode: 0, // not armed
        profile: 0,
        system_load: 0,
    };
    let data_not_armed = telemsg_not_armed.pack();
    assert_eq!(
        &data_not_armed[..],
        &[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    );
}
