//! Web-based .floz file viewer.
//!
//! Displays motor positions (pan + tilt over time) and distance over time as
//! interactive Plotly scatter plots.  The archive is parsed locally in the
//! browser — no data is uploaded.
use std::collections::HashMap;

use gloo_file::{File, callbacks::FileReader};

use wasm_bindgen::prelude::*;

use yew::prelude::*;

use ads_webasm::components::file_input::FileInput;

// -----------------------------------------------------------------------------

const MOTOR_POSITIONS_PLOT_ID: &str = "plot-motor-positions";
const DISTANCE_PLOT_ID: &str = "plot-distance";

/// Maximum number of points sent to Plotly per trace.  For larger datasets we
/// decimate with a uniform stride so rendering stays interactive.
const MAX_PLOT_POINTS: usize = 10_000;

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

                let cur = zip_or_dir::ZipDirArchive::from_zip(
                    std::io::Cursor::new(rbuf),
                    filename.clone(),
                )
                .unwrap_throw();

                self.readers.remove(&filename);
                let file = match floz_parser::floz_parse(cur) {
                    Ok(archive) => {
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
                    link.send_message(Msg::Loaded(filename2, res.expect("failed to read file")))
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
        <section class="panel">
            <div class="panel-heading">
                <h2>{"Time Series"}</h2>
                <p>{"Drag to pan; scroll to zoom. Both plots share the time axis."}</p>
            </div>
            <article class="plot-card">
                <p class="plot-label">{"Motor positions — pan & tilt encoder angles"}</p>
                <div id={MOTOR_POSITIONS_PLOT_ID} class="plot"></div>
                <p class="plot-label">{"Distance — estimated (green) and observed (grey)"}</p>
                <div id={DISTANCE_PLOT_ID} class="plot"></div>
            </article>
        </section>
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
// Plot rendering: convert Rust data → JS typed arrays, call into plots.js.
//
// Performance strategy for large datasets:
//
// 1. Uniform decimation — keep at most MAX_PLOT_POINTS samples per trace.
//    For a million-point file this reduces Plotly's work by ~100×.
//
// 2. Float64Array instead of js_sys::Array of JsValue — a single bulk
//    memcopy into wasm-bindgen's typed-array wrapper rather than allocating
//    one JS object per sample.  Typically another 10–50× faster.
//
// 3. Raw ms-since-epoch numbers for timestamps — eliminates per-point
//    `new Date(...).toISOString()` conversions in JS (~10× savings there).

fn make_float64_array(data: &[f64]) -> js_sys::Float64Array {
    let arr = js_sys::Float64Array::new_with_length(data.len() as u32);
    arr.copy_from(data);
    arr
}

fn update_plots(model: &mut Model) {
    let MaybeValidFlozFile::Valid(fd) = &model.floz_file else {
        return;
    };

    // --- Motor positions ---
    {
        let n = fd.motor_positions.len();
        let stride = (n / MAX_PLOT_POINTS).max(1);
        let cap = n / stride + 1;

        let mut times = Vec::with_capacity(cap);
        let mut pan = Vec::with_capacity(cap);
        let mut tilt = Vec::with_capacity(cap);

        for r in fd.motor_positions.iter().step_by(stride) {
            times.push(r.local.timestamp_millis() as f64);
            pan.push(r.pan_enc.0);
            tilt.push(r.tilt_enc.0);
        }

        if let Err(e) = plot_motor_positions(
            MOTOR_POSITIONS_PLOT_ID,
            &make_float64_array(&times).into(),
            &make_float64_array(&pan).into(),
            &make_float64_array(&tilt).into(),
        ) {
            let msg = js_error_message("Motor positions plot failed", &e);
            model.render_error = Some(msg);
            return;
        }
    }

    // --- Distance ---
    {
        let n = fd.tracking_states.len();
        let stride = (n / MAX_PLOT_POINTS).max(1);
        let cap = n / stride + 1;

        let mut times = Vec::with_capacity(cap);
        let mut est_dist = Vec::with_capacity(cap);
        let mut dist_obs = Vec::with_capacity(cap);

        for r in fd.tracking_states.iter().step_by(stride) {
            times.push(r.processed_timestamp.timestamp_millis() as f64);
            // Use NaN for None so Plotly renders a gap rather than zero.
            est_dist.push(r.est_dist.map(|v| v as f64).unwrap_or(f64::NAN));
            dist_obs.push(r.dist_obs as f64);
        }

        if let Err(e) = plot_distance(
            DISTANCE_PLOT_ID,
            &make_float64_array(&times).into(),
            &make_float64_array(&est_dist).into(),
            &make_float64_array(&dist_obs).into(),
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
            let files = js_sys::Reflect::get(&launch_params, &JsValue::from_str("files")).unwrap();

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
