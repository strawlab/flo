//! Web-based .floz file viewer.
//!
//! Displays motor positions (pan + tilt over time) and distance over time as
//! interactive Plotly scatter plots.  The archive is parsed locally in the
//! browser — no data is uploaded.
use std::collections::HashMap;

use gloo_file::{File, callbacks::FileReader};

use wasm_bindgen::prelude::*;
use web_sys::console::log_1;

use yew::prelude::*;

use ads_webasm::components::file_input::FileInput;

// -----------------------------------------------------------------------------

const MOTOR_POSITIONS_PLOT_ID: &str = "plot-motor-positions";
const DISTANCE_PLOT_ID: &str = "plot-distance";

// -----------------------------------------------------------------------------

#[derive(Default)]
pub enum MaybeValidFlozFile {
    #[default]
    NotLoaded,
    ParseFail(String),
    Valid(ValidFlozFile),
}

pub struct ValidFlozFile {
    pub filename: String,
    filesize: u64,
    pub motor_positions: Vec<flo_core::MotorPositionResult>,
    pub tracking_states: Vec<flo_core::SaveTrackingState>,
}

// -----------------------------------------------------------------------------

struct Model {
    readers: HashMap<String, FileReader>,
    floz_file: MaybeValidFlozFile,
    render_error: Option<String>,
    render_after_next_paint: bool,
    why_busy: WhyBusy,
}

pub enum Msg {
    RenderAll,
    FileChanged(File),
    Loaded(String, Vec<u8>),
}

enum WhyBusy {
    NotBusy,
    LoadingFile(String),
    DrawingPlots,
}

impl Component for Model {
    type Message = Msg;
    type Properties = ();

    fn create(_ctx: &Context<Self>) -> Self {
        Self {
            floz_file: MaybeValidFlozFile::default(),
            readers: HashMap::default(),
            render_error: None,
            render_after_next_paint: false,
            why_busy: WhyBusy::NotBusy,
        }
    }

    fn update(&mut self, ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::RenderAll => {
                self.render_error = None;
                update_plots(self);
                self.why_busy = WhyBusy::NotBusy;
            }
            Msg::Loaded(filename, rbuf) => {
                self.why_busy = WhyBusy::DrawingPlots;
                let filesize = rbuf.len() as u64;

                log_1(
                    &format!("[floz-viewer] loading: {filename} ({filesize} bytes)").into(),
                );

                let cur = zip_or_dir::ZipDirArchive::from_zip(
                    std::io::Cursor::new(rbuf),
                    filename.clone(),
                )
                .unwrap_throw();

                // Log archive contents so parse failures are easy to diagnose.
                match cur.list_paths::<std::path::PathBuf>(None) {
                    Ok(paths) => {
                        log_1(
                            &format!(
                                "[floz-viewer] archive has {} entries:",
                                paths.len()
                            )
                            .into(),
                        );
                        for p in &paths {
                            log_1(&format!("  {}", p.display()).into());
                        }
                    }
                    Err(e) => {
                        log_1(
                            &format!("[floz-viewer] could not list archive entries: {e}").into(),
                        );
                    }
                }

                self.readers.remove(&filename);
                let file = match floz_parser::floz_parse(cur) {
                    Ok(archive) => {
                        log_1(
                            &format!(
                                "[floz-viewer] parsed OK: {} motor samples, {} tracking samples",
                                archive.motor_positions.len(),
                                archive.tracking_states.len(),
                            )
                            .into(),
                        );
                        let title = format!("{filename} - FLOZ Viewer");
                        web_sys::window()
                            .unwrap()
                            .document()
                            .unwrap()
                            .set_title(&title);

                        MaybeValidFlozFile::Valid(ValidFlozFile {
                            filename,
                            filesize,
                            motor_positions: archive.motor_positions,
                            tracking_states: archive.tracking_states,
                        })
                    }
                    Err(e) => {
                        log_1(&format!("[floz-viewer] parse error: {e:#}").into());
                        web_sys::window()
                            .unwrap()
                            .document()
                            .unwrap()
                            .set_title("FLOZ Viewer");

                        MaybeValidFlozFile::ParseFail(e.to_string())
                    }
                };

                self.floz_file = file;
                self.render_error = None;
                // Always dismiss the spinner, even when parsing fails.
                self.why_busy = WhyBusy::NotBusy;
                self.render_after_next_paint =
                    matches!(self.floz_file, MaybeValidFlozFile::Valid(_));
            }
            Msg::FileChanged(file) => {
                let filename = file.name();
                self.render_error = None;
                self.why_busy = WhyBusy::LoadingFile(filename.clone());
                let link = ctx.link().clone();
                let filename2 = filename.clone();
                let reader = gloo_file::callbacks::read_as_bytes(&file, move |res| {
                    link.send_message(Msg::Loaded(
                        filename2,
                        res.expect("failed to read file"),
                    ))
                });
                self.readers.insert(filename, reader);
            }
        }
        true
    }

    fn rendered(&mut self, ctx: &Context<Self>, _first_render: bool) {
        if self.render_after_next_paint {
            self.render_after_next_paint = false;
            ctx.link().send_message(Msg::RenderAll);
        }
    }

    fn view(&self, ctx: &Context<Self>) -> Html {
        use crate::MaybeValidFlozFile::*;

        let file_state_part = match &self.floz_file {
            Valid(fd) => summary_panel(fd),
            NotLoaded => {
                html! {
                    <section class="empty-state">
                        <h2>{"Open a .floz file"}</h2>
                        <p>{"The archive is parsed locally in this browser."}</p>
                    </section>
                }
            }
            ParseFail(e) => {
                html! {
                    <section class="empty-state error-state">
                        <h2>{"Parsing failed"}</h2>
                        <p>{"This file could not be read as a FLOZ archive."}</p>
                        <p>{e}</p>
                    </section>
                }
            }
        };

        let render_error_part = if let Some(err) = &self.render_error {
            html! {
                <section class="empty-state error-state">
                    <h2>{"Plot rendering failed"}</h2>
                    <p>{err}</p>
                </section>
            }
        } else {
            html! { <></> }
        };

        let plots_part = if matches!(self.floz_file, Valid(_)) {
            plot_panels()
        } else {
            html! { <></> }
        };

        let (spinner_div_class, spinner_msg) = match &self.why_busy {
            WhyBusy::NotBusy => ("display-none", "".to_string()),
            WhyBusy::LoadingFile(filename) => {
                ("compute-modal", format!("Loading file: \"{filename}\""))
            }
            WhyBusy::DrawingPlots => ("compute-modal", "Drawing plots".to_string()),
        };

        html! {
            <div id="page-container">
                <div class={spinner_div_class}>
                    <div class="compute-modal-inner">
                        <p>{spinner_msg}</p>
                        <div class="lds-ellipsis">
                            <div></div><div></div><div></div><div></div>
                        </div>
                    </div>
                </div>
                <div id="content-wrap">
                    <header class="app-header">
                        <div>
                            <h1>{"FLOZ Viewer"}</h1>
                            <p>{"Explore local .floz archives with interactive plots."}</p>
                        </div>
                        <FileInput
                            button_text={"Select a .floz file"}
                            accept={".floz"}
                            multiple=false
                            on_changed={ctx.link().callback(|files: Vec<File>| {
                                assert_eq!(files.len(), 1);
                                let file = files.into_iter().next().unwrap();
                                Msg::FileChanged(file)
                            })}
                        />
                    </header>
                    <main class="viewer-main">
                        {file_state_part}
                        {render_error_part}
                        {plots_part}
                    </main>
                    <footer id="footer">{format!("Viewer date: {} (revision {})",
                                        env!("GIT_DATE"),
                                        env!("GIT_HASH"))}
                    </footer>
                </div>
            </div>
        }
    }
}

fn summary_panel(fd: &ValidFlozFile) -> Html {
    let filesize = bytesize_str(fd.filesize);
    let motor_samples = format!("{}", fd.motor_positions.len());
    let tracking_samples = format!("{}", fd.tracking_states.len());

    let duration_str = {
        let t0 = fd.motor_positions.first().map(|r| r.local);
        let t1 = fd.motor_positions.last().map(|r| r.local);
        match (t0, t1) {
            (Some(t0), Some(t1)) => {
                let dur = t1.signed_duration_since(t0);
                let secs = dur.num_milliseconds() as f64 / 1000.0;
                format!("{secs:.1} s")
            }
            _ => "—".to_string(),
        }
    };

    html! {
        <section class="panel">
            <div class="panel-heading">
                <h2>{"Archive Summary"}</h2>
                <p>{&fd.filename}</p>
            </div>
            <div class="metric-grid">
                {metric_card("File size", filesize)}
                {metric_card("Duration", duration_str)}
                {metric_card("Motor samples", motor_samples)}
                {metric_card("Tracking samples", tracking_samples)}
            </div>
        </section>
    }
}

fn plot_panels() -> Html {
    html! {
        <>
            <section class="panel">
                <div class="panel-heading">
                    <h2>{"Motor Positions"}</h2>
                    <p>{"Pan and tilt encoder angles over time. Drag to pan; scroll to zoom."}</p>
                </div>
                <article class="plot-card">
                    <div id={MOTOR_POSITIONS_PLOT_ID} class="plot"></div>
                </article>
            </section>
            <section class="panel">
                <div class="panel-heading">
                    <h2>{"Distance"}</h2>
                    <p>{"Estimated and observed target distance over time."}</p>
                </div>
                <article class="plot-card">
                    <div id={DISTANCE_PLOT_ID} class="plot"></div>
                </article>
            </section>
        </>
    }
}

fn metric_card(label: &str, value: String) -> Html {
    html! {
        <article class="metric-card">
            <span>{label}</span>
            <strong>{value}</strong>
        </article>
    }
}

fn bytesize_str(bytes: u64) -> String {
    const KIB: u64 = 1024;
    const MIB: u64 = KIB * 1024;
    const GIB: u64 = MIB * 1024;
    if bytes >= GIB {
        format!("{:.2} GiB", bytes as f64 / GIB as f64)
    } else if bytes >= MIB {
        format!("{:.2} MiB", bytes as f64 / MIB as f64)
    } else if bytes >= KIB {
        format!("{:.1} KiB", bytes as f64 / KIB as f64)
    } else {
        format!("{bytes} B")
    }
}

// -----------------------------------------------------------------------------
// Plot rendering: convert Rust data → JS arrays, call into plots.js.

fn update_plots(model: &mut Model) {
    let MaybeValidFlozFile::Valid(fd) = &model.floz_file else {
        return;
    };

    // --- Motor positions ---
    {
        // Timestamps as ms-since-epoch (f64) for Plotly date axis.
        let times_ms: js_sys::Array = fd
            .motor_positions
            .iter()
            .map(|r| {
                let ms = r.local.timestamp_millis() as f64;
                JsValue::from_f64(ms)
            })
            .collect();

        let pan_vals: js_sys::Array = fd
            .motor_positions
            .iter()
            .map(|r| JsValue::from_f64(r.pan_enc.0))
            .collect();

        let tilt_vals: js_sys::Array = fd
            .motor_positions
            .iter()
            .map(|r| JsValue::from_f64(r.tilt_enc.0))
            .collect();

        if let Err(e) = plot_motor_positions(
            MOTOR_POSITIONS_PLOT_ID,
            &times_ms.into(),
            &pan_vals.into(),
            &tilt_vals.into(),
        ) {
            let msg = js_error_message("Motor positions plot failed", &e);
            model.render_error = Some(msg);
            return;
        }
    }

    // --- Distance ---
    {
        let times_ms: js_sys::Array = fd
            .tracking_states
            .iter()
            .map(|r| JsValue::from_f64(r.processed_timestamp.timestamp_millis() as f64))
            .collect();

        // est_dist: Option<f32> — use NaN for None so Plotly shows a gap.
        let est_dist_vals: js_sys::Array = fd
            .tracking_states
            .iter()
            .map(|r| {
                let v = r.est_dist.map(|v| v as f64).unwrap_or(f64::NAN);
                JsValue::from_f64(v)
            })
            .collect();

        let dist_obs_vals: js_sys::Array = fd
            .tracking_states
            .iter()
            .map(|r| JsValue::from_f64(r.dist_obs as f64))
            .collect();

        if let Err(e) = plot_distance(
            DISTANCE_PLOT_ID,
            &times_ms.into(),
            &est_dist_vals.into(),
            &dist_obs_vals.into(),
        ) {
            let msg = js_error_message("Distance plot failed", &e);
            model.render_error = Some(msg);
        }
    }
}

fn js_error_message(prefix: &str, value: &JsValue) -> String {
    if let Some(message) = value.as_string() {
        return format!("{prefix}: {message}");
    }
    if let Ok(message) = js_sys::Reflect::get(value, &"message".into())
        && let Some(message) = message.as_string()
    {
        return format!("{prefix}: {message}");
    }
    format!("{prefix}. See the browser console for details.")
}

// -----------------------------------------------------------------------------
// JS bindings

#[wasm_bindgen(module = "/js/launch_queue_support.js")]
extern "C" {
    fn launch_queue_set_consumer(f: &Closure<dyn FnMut(JsValue)>);
}

#[wasm_bindgen(module = "/js/plots.js")]
extern "C" {
    #[wasm_bindgen(catch, js_name = plotMotorPositions)]
    fn plot_motor_positions(
        container_id: &str,
        times_ms: &JsValue,
        pan_vals: &JsValue,
        tilt_vals: &JsValue,
    ) -> Result<(), JsValue>;

    #[wasm_bindgen(catch, js_name = plotDistance)]
    fn plot_distance(
        container_id: &str,
        times_ms: &JsValue,
        est_dist_vals: &JsValue,
        dist_obs_vals: &JsValue,
    ) -> Result<(), JsValue>;
}

// -----------------------------------------------------------------------------

pub fn main() {
    wasm_logger::init(wasm_logger::Config::new(log::Level::Info));
    let app_handle = yew::Renderer::<Model>::new().render();

    {
        // Register a PWA file-handler so clicking a .floz in the OS opens it.
        let boxed = Box::new(app_handle);
        let statik: &'static mut _ = Box::leak(boxed);
        let statik2 = statik.clone();

        let on_launch_params = Closure::new(move |launch_params: JsValue| {
            let files =
                js_sys::Reflect::get(&launch_params, &JsValue::from_str("files")).unwrap();

            let iterator = js_sys::try_iter(&files)
                .unwrap()
                .ok_or("need to pass iterable JS values")
                .unwrap();

            for res_file_js_value in iterator {
                let file_js_value = res_file_js_value.unwrap();

                let file_future = wasm_bindgen_futures::JsFuture::from(
                    file_js_value
                        .dyn_into::<web_sys::FileSystemFileHandle>()
                        .unwrap()
                        .get_file(),
                );

                let statik3 = statik2.clone();
                wasm_bindgen_futures::spawn_local(async move {
                    let file = file_future
                        .await
                        .unwrap()
                        .dyn_into::<web_sys::File>()
                        .unwrap();
                    statik3.send_message(Msg::FileChanged(file.into()));
                })
            }
        });

        launch_queue_set_consumer(&on_launch_params);
        on_launch_params.forget();
    }
}
