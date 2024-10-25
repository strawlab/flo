use flo_core::{osd_structs::Align, sq, Angle, FloatType};
use osd_displayport::displayport_messages;

#[allow(unused)]
mod osd_symbols {
    //see https://betaflight.com/docs/development/osd-glyps

    ///also the first in a total of 16 arrows (ccw)
    pub(crate) const SYM_OVER_HOME: u8 = 0x05;
    pub(crate) const SYM_STICK_OVERLAY_SPRITE_MID: u8 = 0x09;
    pub(crate) const SYM_M: u8 = 0x0c;
    pub(crate) const SYM_ARROW_SOUTH: u8 = 0x60;
    /// a crosshair
    pub(crate) const SYM_AH_CENTER: u8 = 0x73;
    ///progress bar pieces
    pub(crate) const SYM_PB_START: u8 = 0x8A;
    pub(crate) const SYM_PB_FULL: u8 = 0x8B;
    pub(crate) const SYM_PB_HALF: u8 = 0x8C;
    pub(crate) const SYM_PB_EMPTY: u8 = 0x8D;
    pub(crate) const SYM_PB_END: u8 = 0x8E;
    pub(crate) const SYM_PB_CLOSE: u8 = 0x8F;
    ///total 7 symbols, from full to empty
    pub(crate) const SYM_BATT_FULL: u8 = 0x90;
}
use multiwii_serial_protocol_v2::MspPacket;
pub(crate) use osd_symbols::*;

///return arrow character closest to the given direction (0 angle ls ccw from rightward)
pub(crate) fn arrow_char(dir: Angle) -> u8 {
    let turn = core::f64::consts::TAU as FloatType;
    let dir_rel_south = Angle(dir.0 + turn / 4.0).constrained_unsigned().0 + turn; //+turn to be super duper sure it's positive, because remainder operator has stupid habit of being negative for negative numbers
    let char_index = (dir_rel_south / turn * 16.0).round() as u32;
    (char_index % 16) as u8 + SYM_ARROW_SOUTH
}

///returns arrow character pointing towards a given screen point
pub(crate) fn arrow_towards(
    cal: &flo_core::osd_structs::LoadedFpvCameraOSDCalibration,
    charpos: (i32, i32),
    screenpos: (FloatType, FloatType),
) -> u8 {
    let charpos_px = cal.charpos_to_px(charpos.0, charpos.1);
    let dir = FloatType::atan2(-(screenpos.1 - charpos_px.1), screenpos.0 - charpos_px.0);
    arrow_char(Angle(dir))
}

///creates a list of character coordinates sorted by proximity to given screen coordinates
pub(crate) fn char_blob(
    cal: &flo_core::osd_structs::LoadedFpvCameraOSDCalibration,
    screenpos: (FloatType, FloatType),
    n: usize,
) -> Vec<(i32, i32)> {
    let (x, y) = screenpos;
    let mut all_chars: Vec<(i32, i32)> =
        Vec::with_capacity((cal.osd_char_h * cal.osd_char_w) as usize); //all chars of the osd canvas
    for chy in 0..cal.osd_char_h {
        for chx in 0..cal.osd_char_w {
            all_chars.push((chx, chy));
        }
    }
    all_chars.sort_by(|&a, &b| {
        let (x1, y1) = cal.charpos_to_px(a.0, a.1);
        let (x2, y2) = cal.charpos_to_px(b.0, b.1);
        let d1 = sq(x1 - x) + sq(y1 - y);
        let d2 = sq(x2 - x) + sq(y2 - y);
        d1.partial_cmp(&d2).unwrap()
    });
    let mut chars: Vec<(i32, i32)> = Vec::new();
    chars.extend_from_slice(&all_chars[0..n.min(all_chars.len())]);
    chars
}

#[derive(Clone)]
pub(crate) struct OsdCache {
    pub(crate) chars: Vec<u8>,
    w: i32,
    h: i32,
}

///a character canvas that can then group adjacent characters into strings to send them efficiently
impl OsdCache {
    pub(crate) fn new(w: i32, h: i32) -> Self {
        let mut me = Self {
            chars: Vec::with_capacity((w * h) as usize),
            w,
            h,
        };
        me.chars.resize((w * h) as usize, 0_u8);
        me
    }

    pub(crate) fn clear(&mut self) {
        self.chars.fill(0_u8);
    }

    pub(crate) fn draw_char(&mut self, ch: u8, mut x: i32, mut y: i32) {
        //support for negative indexing like python
        if x < 0 {
            x += self.w
        };
        if y < 0 {
            y += self.h
        };
        if x < 0 || y < 0 || x > self.w - 1 || y > self.h - 1 {
            tracing::error!("OsdCache: character coordinates out of range: {} {}", x, y);
        } else {
            self.chars[(x + self.w * y) as usize] = ch;
        }
    }

    ///supports negative indexing like python
    pub(crate) fn print(&mut self, s: &[u8], mut x: i32, y: i32, align: Align) {
        if align == Align::Right {
            x -= s.len() as i32 - 1;
        }
        for ch in s {
            self.draw_char(*ch, x, y);
            x += 1;
        }
    }

    pub(crate) fn make_packets(&mut self) -> Vec<MspPacket> {
        const MSP_OVERHEAD: i32 = 9; //how many extra bytes have to be sent in addition to the characters

        let mut packets = Vec::<MspPacket>::new();

        let mut push_packet = |x0: i32, x1: i32, y: i32| {
            let ch0 = x0 + y * self.w;
            let ch1 = x1 + y * self.w;
            packets.push(
                displayport_messages::DisplayportMessage::WriteString(
                    displayport_messages::WriteStringPayload::from_bytestring(
                        &self.chars[ch0 as usize..ch1 as usize],
                        y as u8,
                        x0 as u8,
                    ),
                )
                .make_msp_packet(),
            );
        };

        for y in 0..self.h {
            let mut packet_range: Option<(i32, i32)> = None;
            for x in 0..self.w {
                if self.chars[(x + y * self.w) as usize] != 0 {
                    if let Some((ref mut _x0, ref mut x1)) = packet_range {
                        *x1 = x + 1;
                    } else {
                        packet_range = Some((x, x + 1));
                    }
                } else if let Some((x0, x1)) = packet_range {
                    if x - x1 > MSP_OVERHEAD {
                        push_packet(x0, x1, y);
                        packet_range = None;
                    }
                }
            }
            if let Some((x0, x1)) = packet_range {
                push_packet(x0, x1, y);
                //packet_range = None;
            }
        }

        packets
    }
}
