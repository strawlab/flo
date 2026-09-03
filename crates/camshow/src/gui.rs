//! The `--gui` output: an egui window that draws the latest OSD-stamped frame.

use std::{
    ops::ControlFlow,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
        mpsc as std_mpsc,
    },
    time::{Duration, Instant},
};

use eframe::egui::{self, Color32, ColorImage, TextureHandle, TextureOptions};
use eyre::Result;
use flo_core::DisplaySource;
use machine_vision_formats::{ImageData, pixel_format::RGB8};
use tokio::sync::watch;
use tracing::{debug, info, warn};

use crate::{
    sink::FrameSink,
    state::{DisplayFrame, Frame, Timestamp},
};

/// How long the GUI may go without painting before the capture thread reports
/// it. Generous, because a paint rate is only meaningful over several frames.
const PAINT_STALL_AFTER: Duration = Duration::from_secs(5);

/// The capture-thread half of the GUI wiring, from [`channels`].
pub(crate) struct GuiSinkConfig {
    display_tx: watch::Sender<Option<DisplayFrame>>,
    egui_ctx_rx: std_mpsc::Receiver<egui::Context>,
    passes: Arc<AtomicU64>,
}

/// The main-thread half of the GUI wiring, from [`channels`].
pub(crate) struct GuiHandles {
    display_rx: watch::Receiver<Option<DisplayFrame>>,
    egui_ctx_tx: std_mpsc::Sender<egui::Context>,
    display_source_tx: watch::Sender<DisplaySource>,
    passes: Arc<AtomicU64>,
}

/// Creates the channels connecting the capture thread to eframe on the main
/// thread: frames go one way, the egui context (available only once eframe has
/// built the app) comes back the other, and a pass counter goes with them so
/// the capture thread can tell whether eframe is still painting.
pub(crate) fn channels(
    display_source_tx: watch::Sender<DisplaySource>,
) -> (GuiSinkConfig, GuiHandles) {
    let (display_tx, display_rx) = watch::channel::<Option<DisplayFrame>>(None);
    let (egui_ctx_tx, egui_ctx_rx) = std_mpsc::channel();
    let passes = Arc::new(AtomicU64::new(0));
    (
        GuiSinkConfig {
            display_tx,
            egui_ctx_rx,
            passes: Arc::clone(&passes),
        },
        GuiHandles {
            display_rx,
            egui_ctx_tx,
            display_source_tx,
            passes,
        },
    )
}

/// Runs eframe on the calling thread, which must be the main thread. Returns
/// when the window closes (or when the capture thread closes it on shutdown).
pub(crate) fn run(handles: GuiHandles, windowed: bool) -> Result<()> {
    let eframe_opts = eframe::NativeOptions {
        window_builder: Some(Box::new(move |mut vb| {
            vb.fullscreen = Some(!windowed);
            vb.decorations = Some(true);
            // On Linux, monitor power cycles can emit tiny restored sizes.
            // Keep windowed mode usable by enforcing sensible bounds.
            vb.min_inner_size = Some(egui::vec2(640.0, 360.0));
            if windowed {
                vb.inner_size = Some(egui::vec2(1280.0, 720.0));
            }
            vb
        })),
        // Off deliberately. With vsync on, the GL swap throttles on the
        // compositor's frame callbacks, which a surface that is not being
        // composited (no monitor attached, output asleep) never receives —
        // that blocks this thread inside `swap_buffers` indefinitely, and a
        // blocked GUI thread cannot process the viewport close that shutdown
        // depends on. Nothing is lost by turning it off: what is drawn is a
        // camera feed that already paces itself on frame arrival.
        vsync: false,
        ..Default::default()
    };

    eframe::run_native(
        env!("CARGO_PKG_NAME"),
        eframe_opts,
        Box::new(move |_cc| {
            Ok(Box::new(CamshowApp::new(
                handles.display_rx,
                handles.egui_ctx_tx,
                handles.display_source_tx,
                handles.passes,
            )))
        }),
    )
    .map_err(|e| eyre::eyre!("eframe failed: {e}"))
}

/// Publishes frames to the egui app and nudges it to repaint.
pub(crate) struct GuiSink {
    display_tx: watch::Sender<Option<DisplayFrame>>,
    egui_ctx: egui::Context,
    /// Bumped by [`CamshowApp::update`] on the main thread, read here on the
    /// capture thread. Whether eframe is painting cannot be observed from
    /// inside `update`, which by definition only runs when it is.
    passes: Arc<AtomicU64>,
    last_progress_check: Instant,
    passes_at_last_check: u64,
    stalled: bool,
}

impl GuiSink {
    /// Blocks until eframe hands over its context, which it does on its first
    /// `update`. Fails if eframe never got that far (e.g. no display), which
    /// drops the sender side.
    pub(crate) fn build(cfg: GuiSinkConfig) -> Result<Self> {
        let egui_ctx = cfg
            .egui_ctx_rx
            .recv()
            .map_err(|e| eyre::eyre!("never received egui context: {e}"))?;
        Ok(Self {
            display_tx: cfg.display_tx,
            egui_ctx,
            passes: cfg.passes,
            last_progress_check: Instant::now(),
            passes_at_last_check: 0,
            stalled: false,
        })
    }

    /// Reports, from off the GUI thread, whether the GUI is still painting.
    ///
    /// Painting is driven by whatever is compositing our surface, not by us: a
    /// surface nobody is displaying (no monitor attached, output asleep) can
    /// stop being painted entirely while capture, recording and streaming
    /// carry on unaffected. With no screen to look at, that state has no
    /// symptom at all until shutdown, when the viewport close this sink queues
    /// is never acted on and the process has to be killed. So say it out loud.
    ///
    /// Only the transitions are logged at `info`/`warn`: a line every few
    /// seconds forever would be noise, and the interesting fact is the change.
    fn check_paint_progress(&mut self) {
        let elapsed = self.last_progress_check.elapsed();
        if elapsed < PAINT_STALL_AFTER {
            return;
        }
        let passes = self.passes.load(Ordering::Relaxed);
        let painted = passes - self.passes_at_last_check;
        match (painted == 0, self.stalled) {
            (true, false) => {
                warn!(
                    "GUI has not painted for {:.1}s (is anything displaying it?); \
                     capture and recording are unaffected",
                    elapsed.as_secs_f32()
                );
                self.stalled = true;
            }
            (false, true) => {
                info!(
                    "GUI is painting again ({painted} passes in {:.1}s)",
                    elapsed.as_secs_f32()
                );
                self.stalled = false;
            }
            _ => debug!(
                "GUI painted {painted} passes in {:.1}s",
                elapsed.as_secs_f32()
            ),
        }
        self.last_progress_check = Instant::now();
        self.passes_at_last_check = passes;
    }

    /// Closes the egui viewport, so eframe's main loop returns. Called once
    /// shutdown has been requested from elsewhere (a signal, or the TCP
    /// server ending).
    pub(crate) fn close_viewport_hook(&self) -> Box<dyn FnOnce() + Send> {
        let egui_ctx = self.egui_ctx.clone();
        Box::new(move || {
            egui_ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        })
    }
}

impl FrameSink for GuiSink {
    fn on_frame(
        &mut self,
        frame: Frame,
        _timestamp: Timestamp,
        recording: bool,
    ) -> ControlFlow<()> {
        let display = DisplayFrame { frame, recording };
        if self.display_tx.send(Some(display)).is_err() {
            debug!("GUI dropped; capture loop exiting");
            return ControlFlow::Break(());
        }
        self.egui_ctx.request_repaint();
        // Asking for a repaint is not the same as getting one, and this
        // thread is the only one in a position to notice the difference.
        self.check_paint_progress();
        ControlFlow::Continue(())
    }

    fn finish(&mut self) {}
}

struct CamshowApp {
    frame_rx: watch::Receiver<Option<DisplayFrame>>,
    egui_ctx_tx: Option<std_mpsc::Sender<egui::Context>>,
    texture: Option<TextureHandle>,
    last_recording: bool,
    display_source_tx: watch::Sender<DisplaySource>,
    /// Bumped once per pass so [`GuiSink::check_paint_progress`], on the
    /// capture thread, can see whether this one is still being driven.
    passes: Arc<AtomicU64>,
}

impl CamshowApp {
    fn new(
        frame_rx: watch::Receiver<Option<DisplayFrame>>,
        egui_ctx_tx: std_mpsc::Sender<egui::Context>,
        display_source_tx: watch::Sender<DisplaySource>,
        passes: Arc<AtomicU64>,
    ) -> Self {
        Self {
            frame_rx,
            egui_ctx_tx: Some(egui_ctx_tx),
            texture: None,
            last_recording: false,
            display_source_tx,
            passes,
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
        self.passes.fetch_add(1, Ordering::Relaxed);

        let is_fullscreen = ctx.input(|i| i.viewport().fullscreen.unwrap_or(false));

        if let Some(source) = ctx.input(display_source_hotkey) {
            let _ = self.display_source_tx.send(source);
        }

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
                    let image = egui::Image::new(tex).shrink_to_fit();
                    if is_fullscreen {
                        let image_size = image.calc_size(panel_rect.size(), Some(tex.size_vec2()));
                        image.paint_at(ui, centered_rect(panel_rect, image_size));
                    } else {
                        ui.add(image);
                    }
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

fn centered_rect(outer: egui::Rect, size: egui::Vec2) -> egui::Rect {
    egui::Rect::from_center_size(outer.center(), size)
}

fn display_source_hotkey(input: &egui::InputState) -> Option<DisplaySource> {
    [
        egui::Key::Num0,
        egui::Key::Space,
        egui::Key::Num1,
        egui::Key::Num2,
    ]
    .into_iter()
    .find(|&key| input.key_pressed(key))
    .and_then(display_source_for_key)
}

fn display_source_for_key(key: egui::Key) -> Option<DisplaySource> {
    match key {
        egui::Key::Num0 | egui::Key::Space => Some(DisplaySource::Webcam),
        egui::Key::Num1 => Some(DisplaySource::StrandCamMain),
        egui::Key::Num2 => Some(DisplaySource::StrandCamSecondary),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_source_hotkeys_select_the_expected_views() {
        let cases = [
            (egui::Key::Num0, DisplaySource::Webcam),
            (egui::Key::Space, DisplaySource::Webcam),
            (egui::Key::Num1, DisplaySource::StrandCamMain),
            (egui::Key::Num2, DisplaySource::StrandCamSecondary),
        ];

        for (key, expected) in cases {
            assert_eq!(display_source_for_key(key), Some(expected));
        }
    }

    #[test]
    fn fullscreen_image_is_centered_in_the_panel() {
        let panel = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(1920.0, 1080.0));
        let image = centered_rect(panel, egui::vec2(1440.0, 1080.0));

        assert_eq!(image.left(), 240.0);
        assert_eq!(image.right(), 1680.0);
        assert_eq!(image.top(), 0.0);
        assert_eq!(image.bottom(), 1080.0);
    }
}
