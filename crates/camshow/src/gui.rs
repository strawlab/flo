//! egui app that draws the latest webcam frame.

use std::sync::{Arc, mpsc};

use eframe::egui::{self, Color32, ColorImage, TextureHandle, TextureOptions};
use machine_vision_formats::{ImageData, pixel_format::RGB8};
use tokio::sync::watch;

use crate::state::DisplayFrame;

pub(crate) struct CamshowApp {
    frame_rx: watch::Receiver<Option<DisplayFrame>>,
    egui_ctx_tx: Option<mpsc::Sender<egui::Context>>,
    texture: Option<TextureHandle>,
    last_recording: bool,
}

impl CamshowApp {
    pub(crate) fn new(
        frame_rx: watch::Receiver<Option<DisplayFrame>>,
        egui_ctx_tx: mpsc::Sender<egui::Context>,
    ) -> Self {
        Self {
            frame_rx,
            egui_ctx_tx: Some(egui_ctx_tx),
            texture: None,
            last_recording: false,
        }
    }

    fn refresh_texture(&mut self, ctx: &egui::Context) {
        if !matches!(self.frame_rx.has_changed(), Ok(true)) {
            return;
        }
        let display = self.frame_rx.borrow_and_update();
        let Some(display) = display.as_ref() else {
            return;
        };
        self.last_recording = display.recording;

        let im = display.frame.borrow();
        let Some(rgb) = im.as_static::<RGB8>() else {
            tracing::warn!("frame is not RGB8; skipping render");
            return;
        };

        let size = [rgb.width() as usize, rgb.height() as usize];
        let texture = self.texture.get_or_insert_with(|| {
            ctx.load_texture(
                "camshow",
                egui::ImageData::Color(Arc::new(ColorImage::new(size, vec![Color32::TRANSPARENT]))),
                TextureOptions::default(),
            )
        });

        texture.set(
            ColorImage::from_rgb(size, rgb.image_data()),
            TextureOptions::default(),
        );
    }
}

impl eframe::App for CamshowApp {
    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        tracing::info!("camshow GUI exiting");
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let is_fullscreen = ctx.input(|i| i.viewport().fullscreen.unwrap_or(false));

        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) && is_fullscreen {
            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(false));
        }

        if ctx.input(|i| i.key_pressed(egui::Key::F) && i.modifiers.ctrl) && !is_fullscreen {
            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(true));
        }

        if let Some(sender) = self.egui_ctx_tx.take() {
            let _ = sender.send(ctx.clone());
        }

        self.refresh_texture(ctx);

        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(Color32::BLACK))
            .show(ctx, |ui| {
                let panel_rect = ui.max_rect();
                if let Some(tex) = self.texture.as_ref() {
                    ui.add(egui::Image::new(tex).shrink_to_fit());
                } else {
                    ui.centered_and_justified(|ui| {
                        ui.label("waiting for camera…");
                    });
                }
                let painter = ui.painter();
                if self.last_recording {
                    painter.text(
                        panel_rect.right_top() + egui::vec2(-12.0, 12.0),
                        egui::Align2::RIGHT_TOP,
                        "● REC",
                        egui::FontId::proportional(20.0),
                        Color32::from_rgb(255, 60, 60),
                    );
                }
                let hint = if is_fullscreen {
                    "Fullscreen mode: Press Esc to exit"
                } else {
                    "Windowed mode: Press Ctrl-F for fullscreen"
                };
                let font_id = egui::FontId::proportional(16.0);
                let anchor = panel_rect.left_bottom() + egui::vec2(12.0, -12.0);
                painter.text(
                    anchor,
                    egui::Align2::LEFT_BOTTOM,
                    hint,
                    font_id,
                    Color32::WHITE,
                );
            });
    }
}
