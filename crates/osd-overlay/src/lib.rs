//! Render an [`osd_utils::OsdCache`] character grid onto an RGB image.
//!
//! Used both at runtime by `camshow` (overlay on the live feed) and offline by
//! `burn-osd` (bake the OSD into a recorded video).

use eyre::Result;
use machine_vision_formats::{ImageMutStride, pixel_format};
use rusttype::{Scale, point};

use osd_utils::{
    SYM_ARROW_SOUTH, SYM_BATT_FULL, SYM_M, SYM_STICK_OVERLAY_HORIZONTAL,
    SYM_STICK_OVERLAY_VERTICAL, X75, X76, X77, X78,
};

const FONT_X_SCALE: f32 = 135.0;
const FONT_Y_SCALE: f32 = 55.0;

const WHITE: [u8; 3] = [255, 255, 255];
const BLACK: [u8; 3] = [0, 0, 0];

/// Loaded TrueType fonts for OSD rendering. The bold copy is drawn first as a
/// black outline; the light copy is drawn on top in white.
pub struct OsdFonts {
    pub bold: rusttype::Font<'static>,
    pub light: rusttype::Font<'static>,
}

impl OsdFonts {
    pub fn load() -> Self {
        const BOLD: &[u8] = include_bytes!("../IBMPlexMono-Bold.ttf");
        const LIGHT: &[u8] = include_bytes!("../IBMPlexMono-Thin.ttf");
        Self {
            bold: rusttype::Font::try_from_bytes(BOLD).expect("bundled bold font is valid"),
            light: rusttype::Font::try_from_bytes(LIGHT).expect("bundled light font is valid"),
        }
    }
}

/// Map OSD glyph indices (0..=255) to Unicode characters that the bundled
/// fonts can render.
pub type GlyphTable = [char; 256];

/// Build the OSD-byte → char table once. The bytes that don't have explicit
/// overrides map to themselves (ASCII passthrough); 0 is treated as a space
/// so empty cells render blank.
pub fn build_glyph_table() -> GlyphTable {
    let mut table = ['\0'; 256];
    for (i, slot) in table.iter_mut().enumerate() {
        *slot = i as u8 as char;
    }
    table[0] = ' ';

    let arrows = [
        '↓', '↓', '↘', '↘', '→', '→', '↗', '↗', '↑', '↑', '↖', '↖', '←', '←', '↙', '↙',
    ];
    for (i, ch) in arrows.iter().enumerate() {
        table[SYM_ARROW_SOUTH as usize + i] = *ch;
    }

    let batteries = ['█', '▉', '▆', '▄', '▃', '▂', '▁'];
    for (i, ch) in batteries.iter().enumerate() {
        table[SYM_BATT_FULL as usize + i] = *ch;
    }

    table[SYM_M as usize] = 'm';
    table[SYM_STICK_OVERLAY_VERTICAL as usize] = '|';
    table[SYM_STICK_OVERLAY_HORIZONTAL as usize] = '-';
    table[X75 as usize] = '^';
    table[X76 as usize] = 'v';
    table[X77 as usize] = '>';
    table[X78 as usize] = '<';

    table
}

/// Convert a row of OSD bytes to its renderable string form.
pub fn osd_row_to_string(row: &[u8], table: &GlyphTable) -> String {
    row.iter().map(|b| table[*b as usize]).collect()
}

/// Stamp a single text row onto the image at `rownum`'s vertical offset.
pub fn stamp_row(
    image: &mut dyn ImageMutStride<pixel_format::RGB8>,
    rownum: usize,
    fonts: &OsdFonts,
    text: &str,
) -> Result<()> {
    let scale = Scale {
        x: FONT_X_SCALE,
        y: FONT_Y_SCALE,
    };
    let v_metrics = fonts.light.v_metrics(scale);
    let line_gap = if v_metrics.line_gap == 0.0 {
        v_metrics.ascent * 1.5
    } else {
        v_metrics.line_gap
    };

    let x0 = 20.0;
    let y0 = 20.0 + rownum as f32 * line_gap + v_metrics.ascent;

    for glyph in fonts.light.layout(text, scale, point(x0, y0)) {
        let Some(bb) = glyph.pixel_bounding_box() else {
            continue;
        };

        // Black bold outline.
        let bold = fonts
            .bold
            .glyph(glyph.id())
            .scaled(scale)
            .positioned(glyph.position());
        let bb_bold = bold.pixel_bounding_box().unwrap_or(bb);
        bold.draw(|x, y, v| {
            blend_pixel(
                image,
                x + bb_bold.min.x as u32,
                y + bb_bold.min.y as u32,
                BLACK,
                v,
            );
        });

        // White light foreground.
        glyph.draw(|x, y, v| {
            blend_pixel(image, x + bb.min.x as u32, y + bb.min.y as u32, WHITE, v);
        });
    }

    Ok(())
}

/// Stamp every row of an [`osd_utils::OsdCache`] onto an image.
pub fn stamp_canvas(
    image: &mut dyn ImageMutStride<pixel_format::RGB8>,
    canvas: &osd_utils::OsdCache,
    fonts: &OsdFonts,
    table: &GlyphTable,
) -> Result<()> {
    for (rownum, row) in canvas.chars.chunks_exact(canvas.w as usize).enumerate() {
        let text = osd_row_to_string(row, table);
        stamp_row(image, rownum, fonts, &text)?;
    }
    Ok(())
}

fn blend_pixel(
    image: &mut dyn ImageMutStride<pixel_format::RGB8>,
    x: u32,
    y: u32,
    color: [u8; 3],
    coverage: f32,
) {
    if x >= image.width() || y >= image.height() {
        return;
    }
    let stride = image.stride();
    let buf = image.buffer_mut_ref().data;
    let offset = stride * y as usize + x as usize * 3;

    let a = coverage as f64;
    let inv = 1.0 - a;
    for c in 0..3 {
        let old = buf[offset + c] as f64;
        let new = (old * inv + color[c] as f64 * a).round();
        buf[offset + c] = new.clamp(0.0, 255.0) as u8;
    }
}
