use std::io::Write;

use flo_core::{Angle, FloatType, osd_structs::Align, sq};
use osd_displayport::displayport_messages;

mod osd_symbols {
    //see https://betaflight.com/docs/development/osd-glyps

    pub const SYM_M: u8 = 0x0c;
    pub const SYM_STICK_OVERLAY_VERTICAL: u8 = 0x16;
    pub const SYM_STICK_OVERLAY_HORIZONTAL: u8 = 0x17;
    /// Also the first in a total of 16 arrows (CCW).
    pub const SYM_ARROW_SOUTH: u8 = 0x60;
    pub const X75: u8 = 0x75;
    pub const X76: u8 = 0x76;
    pub const X77: u8 = 0x77;
    pub const X78: u8 = 0x78;
    /// Total 7 symbols, from full to empty.
    pub const SYM_BATT_FULL: u8 = 0x90;
}
use multiwii_serial_protocol_v2::MspPacket;
pub use osd_symbols::*;

/// Returns the arrow character closest to the given direction (0 angle is CCW from rightward).
fn arrow_char(dir: Angle) -> u8 {
    let turn = core::f64::consts::TAU as FloatType;
    let dir_rel_south = Angle(dir.0 + turn / 4.0).constrained_unsigned().0 + turn; //+turn to be super duper sure it's positive, because remainder operator has stupid habit of being negative for negative numbers
    let char_index = (dir_rel_south / turn * 16.0).round() as u32;
    (char_index % 16) as u8 + SYM_ARROW_SOUTH
}

/// Returns the arrow character pointing towards a given screen point.
pub fn arrow_towards(
    cal: &flo_core::osd_structs::LoadedFpvCameraOSDCalibration,
    charpos: (i32, i32),
    screenpos: (FloatType, FloatType),
) -> u8 {
    let charpos_px = cal.charpos_to_px(charpos.0, charpos.1);
    let dir = FloatType::atan2(-(screenpos.1 - charpos_px.1), screenpos.0 - charpos_px.0);
    arrow_char(Angle(dir))
}

/// Returns character coordinates sorted by proximity to the given screen coordinates.
pub fn char_blob(
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

fn serialize_chars<S>(chars: &[u8], serializer: S) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    // use serde::ser::Error;
    use base64::Engine;

    let mut enc = flate2::write::DeflateEncoder::new(Vec::new(), flate2::Compression::fast());
    enc.write_all(chars).unwrap();
    let compressed = enc.finish().unwrap();
    let base64_buf = base64::engine::general_purpose::STANDARD.encode(&compressed);
    serializer.serialize_str(&base64_buf)
}

fn deserialize_chars<'de, D>(deserializer: D) -> Result<Vec<u8>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use base64::Engine;
    use serde::{Deserialize, de::Error};

    let base64_encoded = String::deserialize(deserializer)?;
    let compressed = base64::engine::general_purpose::STANDARD
        .decode(&base64_encoded)
        .map_err(|_decode_error| {
            D::Error::invalid_value(
                serde::de::Unexpected::Str(&base64_encoded),
                &"base64 encoded string",
            )
        })?;

    let mut writer = Vec::new();
    let mut deflater = flate2::write::DeflateDecoder::new(writer);
    deflater.write_all(&compressed[..]).unwrap();
    writer = deflater.finish().unwrap();

    Ok(writer)
}

#[derive(Debug, PartialEq, Clone, serde::Serialize, serde::Deserialize)]
pub struct OsdCache {
    #[serde(
        serialize_with = "serialize_chars",
        deserialize_with = "deserialize_chars"
    )]
    pub chars: Vec<u8>,
    pub w: i32,
    pub h: i32,
}

/// A character canvas that groups adjacent characters into strings for efficient sending.
impl OsdCache {
    pub fn new(w: i32, h: i32) -> Self {
        let chars = vec![0; (w * h) as usize];
        Self { chars, w, h }
    }

    pub fn clear(&mut self) {
        self.chars.fill(0_u8);
    }

    pub fn draw_char(&mut self, ch: u8, mut x: i32, mut y: i32) {
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

    /// Supports negative indexing (like Python).
    pub fn print(&mut self, s: &[u8], mut x: i32, y: i32, align: Align) {
        if align == Align::Right {
            x -= s.len() as i32 - 1;
        }
        for ch in s {
            self.draw_char(*ch, x, y);
            x += 1;
        }
    }

    pub fn make_packets(&mut self) -> Vec<MspPacket> {
        const MSP_OVERHEAD: i32 = 9; //how many extra bytes have to be sent in addition to the characters

        let mut packets = Vec::<MspPacket>::new();

        packets.push(displayport_messages::DisplayportMessage::ClearScreen.make_msp_packet());

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
                } else if let Some((x0, x1)) = packet_range
                    && x - x1 > MSP_OVERHEAD
                {
                    push_packet(x0, x1, y);
                    packet_range = None;
                }
            }
            if let Some((x0, x1)) = packet_range {
                push_packet(x0, x1, y);
                //packet_range = None;
            }
        }

        packets.push(displayport_messages::DisplayportMessage::DrawScreen.make_msp_packet());

        packets
    }
}

/// Build a 30×16 canvas with every kind of OSD glyph (corner labels, the 16
/// arrow directions, the battery ramp, and the misc symbols) so a viewer can
/// confirm the font/rendering pipeline is working without flo connected.
pub fn test_pattern() -> OsdCache {
    use flo_core::osd_structs::Align;

    let mut canvas = OsdCache::new(30, 16);
    canvas.print(b"TOP LEFT", 0, 0, Align::Left);
    canvas.print(b"TOP RIGHT", -1, 0, Align::Right);
    canvas.print(b"BOTTOM LEFT", 0, -1, Align::Left);
    canvas.print(b"BOTTOM RIGHT", -1, -1, Align::Right);

    let arrows: Vec<u8> = (SYM_ARROW_SOUTH..SYM_ARROW_SOUTH + 16).collect();
    canvas.print(&arrows, 0, 1, Align::Left);

    let batts: Vec<u8> = (SYM_BATT_FULL..SYM_BATT_FULL + 6).collect();
    canvas.print(&batts, 0, 2, Align::Left);

    canvas.print(
        &[SYM_STICK_OVERLAY_VERTICAL, SYM_STICK_OVERLAY_HORIZONTAL],
        0,
        3,
        Align::Left,
    );
    canvas.print(&[X75, X76, X77, X78], 0, 4, Align::Left);

    canvas
}

/// A minimal canvas standing in for flo's live OSD when no fresh update has
/// arrived recently, so a dead or dropped control link is visible on the
/// display instead of leaving the corner silently blank.
pub fn flo_disconnected_pattern() -> OsdCache {
    use flo_core::osd_structs::Align;

    let mut canvas = OsdCache::new(20, 1);
    canvas.print(b"FLO DISCONNECTED", 0, 0, Align::Left);
    canvas
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_serde_rt() {
        let mut canvas = OsdCache::new(30, 16);
        canvas.print(b"TOP LEFT", 0, 0, flo_core::osd_structs::Align::Left);
        let arrows: Vec<u8> = (SYM_ARROW_SOUTH..SYM_ARROW_SOUTH + 16).collect();
        canvas.print(&arrows, 0, 1, flo_core::osd_structs::Align::Left);
        let batts: Vec<u8> = (SYM_BATT_FULL..SYM_BATT_FULL + 6).collect();
        canvas.print(&batts, 0, 2, flo_core::osd_structs::Align::Left);
        canvas.print(
            &[SYM_STICK_OVERLAY_VERTICAL, SYM_STICK_OVERLAY_HORIZONTAL],
            0,
            3,
            flo_core::osd_structs::Align::Left,
        );
        canvas.print(
            &[X75, X76, X77, X78],
            0,
            4,
            flo_core::osd_structs::Align::Left,
        );
        canvas.print(b"TOP RIGHT", -1, 0, flo_core::osd_structs::Align::Right);
        canvas.print(b"BOTTOM LEFT", 0, -1, flo_core::osd_structs::Align::Left);
        canvas.print(b"BOTTOM RIGHT", -1, -1, flo_core::osd_structs::Align::Right);

        let encoded = serde_json::to_string(&canvas).unwrap();
        let decoded = serde_json::from_str(&encoded).unwrap();
        assert_eq!(canvas, decoded);
    }

    // This test data ensures the format of the serialized data remains stable.
    const RAW_CHARS: &[u8] = b"abc\0def\xF0\x9F\x98\x80";
    const RAW_W: i32 = 12;
    const RAW_H: i32 = 34;
    const ENCODED: &str = "{\"chars\":\"S0xKZkhJTfswf0YDAA==\",\"w\":12,\"h\":34}";

    #[test]
    fn test_decode() {
        let x = serde_json::from_str::<OsdCache>(ENCODED).unwrap();
        assert_eq!(&x.chars, RAW_CHARS);
        assert_eq!(x.w, RAW_W);
        assert_eq!(x.h, RAW_H);
    }

    #[test]
    fn test_encode() {
        let raw = OsdCache {
            chars: RAW_CHARS.to_vec(),
            w: RAW_W,
            h: RAW_H,
        };
        let encoded = serde_json::to_string(&raw).unwrap();
        // Don't compare the raw encoded string, as json formatting may change
        // slightly. Rather decode it and check the source data is unchanged.
        let x = serde_json::from_str::<OsdCache>(&encoded).unwrap();
        assert_eq!(&x.chars, RAW_CHARS);
        assert_eq!(x.w, RAW_W);
        assert_eq!(x.h, RAW_H);
    }
}
