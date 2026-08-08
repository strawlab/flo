use console_error_panic_hook::set_once as set_panic_hook;

use std::{fmt, net::SocketAddr};

use gloo_events::EventListener;
use gloo_timers::callback::Interval;
use wasm_bindgen::{JsCast, JsValue, prelude::*};
use wasm_bindgen_futures::JsFuture;
use web_sys::{Event, EventSource, Gamepad, GamepadEvent, HtmlInputElement, MessageEvent};

use yew::prelude::*;

use yew_tincture::components::{Button, TypedInput, TypedInputStorage};

use flo_core::*;

mod recording_path;
use recording_path::RecordingPathWidget;

use ads_webasm::components::ConnectDevice;

const JOYGAIN: f64 = 1.0;

#[derive(Clone, PartialEq)]
struct AngleDegrees(FloatType);

impl std::str::FromStr for AngleDegrees {
    type Err = std::num::ParseFloatError;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        let v = FloatType::from_str(s)?;
        Ok(AngleDegrees(v))
    }
}

impl std::fmt::Display for AngleDegrees {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Clone, PartialEq)]
struct DistanceMeters(FloatType);

impl std::str::FromStr for DistanceMeters {
    type Err = std::num::ParseFloatError;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        let v = FloatType::from_str(s)?;
        Ok(DistanceMeters(v))
    }
}

impl std::fmt::Display for DistanceMeters {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Clone, PartialEq)]
struct Seconds(FloatType);

impl std::str::FromStr for Seconds {
    type Err = std::num::ParseFloatError;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        let v = FloatType::from_str(s)?;
        Ok(Seconds(v))
    }
}

impl std::fmt::Display for Seconds {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

struct App {
    floz_recording_path: Option<RecordingPath>,
    webcam_recording_path: Option<RecordingPath>,
    pan_center_degrees: TypedInputStorage<AngleDegrees>,
    tilt_center_degrees: TypedInputStorage<AngleDegrees>,
    distance_center: TypedInputStorage<DistanceMeters>,
    distance_corr_m: TypedInputStorage<DistanceMeters>,
    precapture_seconds: TypedInputStorage<Seconds>,
    last_state: Option<flo_core::DeviceState>,
    cfg: Option<flo_core::FloControllerConfig>,
    strand_cameras: Vec<flo_core::StrandCamProxyInfo>,
    es: EventSource,
    _listeners: Vec<EventListener>,
    query_gamepad_interval: Option<Interval>,
    last_gamepad_timestamp: f64,
    /// Set once the server broadcasts that it is shutting down.
    server_quit: bool,
    adding_rtp_target: bool,
    new_rtp_target: String,
    new_rtp_bitrate: String,
    new_rtp_target_error: Option<String>,
    /// The address field of the add-target modal. `autofocus` does nothing here
    /// because Yew sets the attribute after the element is already in the
    /// document, so the field is focused explicitly in `rendered`.
    new_rtp_target_ref: NodeRef,
    /// Set when the modal opens; cleared once its address field has the focus.
    focus_new_rtp_target: bool,
    rtp_target_pending_removal: Option<String>,
}

enum Msg {
    SetHomePosition,
    SwitchToOpenLoop,
    SetHomePositionFromCurrent,
    SwitchToClosedLoop,
    /// Event Source ready. We got new data.
    EsReady(Box<Result<BuiEventData, serde_path_to_error::Error<serde_json::Error>>>),
    GamepadConnected((bool, Gamepad)),
    GamepadInterval,
    SendMessageFetchState(FetchState),
    DoRecordMotorPositionsFloz(bool),
    SetRecordTrackingCamMp4(bool),
    DoPreCaptureRecord,
    SetPreCaptureSeconds,
    SetDistanceCorrection,
    AdjustFocus(i32),
    SetDisplaySource(DisplaySource),
    ShowAddRtpTarget,
    CancelAddRtpTarget,
    SetNewRtpTarget(String),
    SetNewRtpBitrate(String),
    AddRtpTarget,
    SetRtpTargetBitrate(String, String),
    ConfirmRemoveRtpTarget(String),
    CancelRemoveRtpTarget,
    RemoveRtpTarget,
    /// The server broadcast that it is shutting down.
    ServerQuit,
    RenderView,
}

// -----------------------------------------------------------------------------

pub enum FetchState {
    Fetching,
    Success,
    Failed(FetchError),
}

// -----------------------------------------------------------------------------

/// Something wrong has occurred while fetching an external resource.
#[derive(Debug, Clone, PartialEq)]
pub struct FetchError {
    err: JsValue,
}
impl std::fmt::Display for FetchError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> fmt::Result {
        std::fmt::Debug::fmt(&self.err, f)
    }
}
impl std::error::Error for FetchError {}

impl From<JsValue> for FetchError {
    fn from(value: JsValue) -> Self {
        Self { err: value }
    }
}

// -----------------------------------------------------------------------------

impl Component for App {
    type Message = Msg;
    type Properties = ();

    fn create(ctx: &Context<Self>) -> Self {
        let es = EventSource::new(flo_core::EVENTS_PATH)
            .map_err(|js_value: JsValue| {
                let err: js_sys::Error = js_value.dyn_into().unwrap_throw();
                err
            })
            .unwrap_throw();
        let mut _listeners = Vec::new();

        {
            // event source listener
            let data_callback = ctx.link().callback(|bufstr: String| {
                let jd = &mut serde_json::Deserializer::from_str(&bufstr);
                let result: Result<BuiEventData, _> = serde_path_to_error::deserialize(jd);
                Msg::EsReady(Box::new(result))
            });
            _listeners.push(EventListener::new(&es, EVENT_NAME, move |event: &Event| {
                let event = event.dyn_ref::<MessageEvent>().unwrap_throw();
                let text = event.data().as_string().unwrap_throw();
                data_callback.emit(text);
            }));
        }

        {
            // gamepad connected and disconnected listeners
            let window = gloo_utils::window();
            let gamepad_connected_callback =
                ctx.link()
                    .callback(|(connected, gamepad): (bool, Gamepad)| {
                        Msg::GamepadConnected((connected, gamepad))
                    });
            let gamepad_connected_callback2 = gamepad_connected_callback.clone();
            _listeners.push(EventListener::new(
                &window,
                "gamepadconnected",
                move |event: &Event| {
                    let event = event.dyn_ref::<GamepadEvent>().unwrap_throw();
                    let gamepad = event.gamepad().unwrap_throw();
                    gamepad_connected_callback2.emit((true, gamepad));
                },
            ));
            _listeners.push(EventListener::new(
                &window,
                "gamepaddisconnected",
                move |event: &Event| {
                    let event = event.dyn_ref::<GamepadEvent>().unwrap_throw();
                    let gamepad = event.gamepad().unwrap_throw();
                    gamepad_connected_callback.emit((false, gamepad));
                },
            ));
        }

        {
            // Listen for the server-is-quitting event so this browser shows the
            // "FLO has quit" screen and stops trying to reconnect, even if the
            // shutdown was initiated elsewhere.
            let quit_callback = ctx.link().callback(|_| Msg::ServerQuit);
            _listeners.push(EventListener::new(
                &es,
                FLO_QUIT_EVENT_NAME,
                move |_event: &Event| {
                    quit_callback.emit(());
                },
            ));
        }

        let link = ctx.link().clone();
        _listeners.push(EventListener::new(&es, "error", move |_event: &Event| {
            // Trigger a UI redraw on error, because we won't get any state
            // updates from the server which would otherwise cause a redraw.
            link.send_message(Msg::RenderView);
        }));

        Self {
            es,
            floz_recording_path: None,
            webcam_recording_path: None,
            pan_center_degrees: TypedInputStorage::from_initial(AngleDegrees(0.0)),
            tilt_center_degrees: TypedInputStorage::from_initial(AngleDegrees(0.0)),
            distance_center: TypedInputStorage::from_initial(DistanceMeters(0.5)),
            distance_corr_m: TypedInputStorage::from_initial(DistanceMeters(0.082)),
            precapture_seconds: TypedInputStorage::from_initial(Seconds(0.0)),

            last_state: None,
            cfg: None,
            strand_cameras: Vec::new(),
            _listeners,
            query_gamepad_interval: None,
            last_gamepad_timestamp: 0.0,
            server_quit: false,
            adding_rtp_target: false,
            new_rtp_target: String::new(),
            new_rtp_bitrate: flo_core::DEFAULT_RTP_BITRATE_KBPS.to_string(),
            new_rtp_target_error: None,
            new_rtp_target_ref: NodeRef::default(),
            focus_new_rtp_target: false,
            rtp_target_pending_removal: None,
        }
    }

    fn rendered(&mut self, _ctx: &Context<Self>, _first_render: bool) {
        if self.focus_new_rtp_target
            && let Some(input) = self.new_rtp_target_ref.cast::<HtmlInputElement>()
        {
            let _ = input.focus();
            self.focus_new_rtp_target = false;
        }
    }

    fn update(&mut self, ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::RenderView => {}
            Msg::ServerQuit => {
                // The server is shutting down. Show the "has quit" screen and
                // close the event stream so the browser stops reconnecting.
                self.server_quit = true;
                self.es.close();
            }
            Msg::GamepadConnected((_connected, _gamepad)) => {
                if self.n_connected_gamepads() > 0 {
                    let handle = {
                        let link = ctx.link().clone();
                        Interval::new(20, move || link.send_message(Msg::GamepadInterval))
                    };
                    self.query_gamepad_interval = Some(handle);
                } else {
                    self.query_gamepad_interval = None;
                }
            }
            Msg::GamepadInterval => {
                if let Some(cfg) = self.cfg.clone() {
                    self.handle_gamepad_interval(ctx, cfg);
                }
            }
            Msg::SendMessageFetchState(_fetch_state) => {
                return false;
            }
            Msg::DoRecordMotorPositionsFloz(val) => {
                let msg = flo_core::FloCommand::SetRecordingState(val);
                self.send_message(msg, ctx);
                return false; // Don't update DOM, do that when backend notifies us of new state.
            }
            // Msg::Sawtooth => {
            //     self.set_device_mode(flo_core::DeviceMode::SawtoothTest((1, 10)), ctx);
            //     return false;
            // }
            // Msg::VoltageFollower12 => {
            //     self.set_device_mode(flo_core::DeviceMode::VoltageFollower12, ctx);
            //     return false;
            // }
            // Msg::VoltageFollower13 => {
            //     self.set_device_mode(flo_core::DeviceMode::VoltageFollower13, ctx);
            //     return false;
            // }
            Msg::SetRecordTrackingCamMp4(val) => {
                let msg = flo_core::FloCommand::SetRecordTrackingCamMp4(val);
                self.send_message(msg, ctx);
                return false; // Don't update DOM; wait for backend state.
            }
            Msg::DoPreCaptureRecord => {
                let msg = flo_core::FloCommand::StartPreCaptureRecording;
                self.send_message(msg, ctx);
                return false; // Don't update DOM; wait for backend state.
            }
            Msg::SetPreCaptureSeconds => {
                if let Ok(secs) = self.precapture_seconds.parsed() {
                    let msg = flo_core::FloCommand::SetPreCaptureSeconds(secs.0);
                    self.send_message(msg, ctx);
                }
                return false; // Don't update DOM; wait for backend state.
            }
            Msg::SetDistanceCorrection => {
                if let Ok(dist_corr) = self.distance_corr_m.parsed() {
                    let msg = flo_core::FloCommand::SetDistCorr(dist_corr.0);
                    self.send_message(msg, ctx);
                }
            }

            Msg::AdjustFocus(change) => {
                let msg = flo_core::FloCommand::AdjustFocus(change);
                self.send_message(msg, ctx);
            }
            Msg::SetDisplaySource(source) => {
                self.send_message(flo_core::FloCommand::SetDisplaySource(source), ctx);
                return false;
            }
            Msg::ShowAddRtpTarget => {
                self.adding_rtp_target = true;
                self.focus_new_rtp_target = true;
            }
            Msg::CancelAddRtpTarget => {
                self.adding_rtp_target = false;
                self.focus_new_rtp_target = false;
                self.new_rtp_target.clear();
                self.new_rtp_bitrate = flo_core::DEFAULT_RTP_BITRATE_KBPS.to_string();
                self.new_rtp_target_error = None;
            }
            Msg::SetNewRtpTarget(target) => {
                self.new_rtp_target = target;
                self.new_rtp_target_error = None;
            }
            Msg::SetNewRtpBitrate(bitrate) => {
                self.new_rtp_bitrate = bitrate;
                self.new_rtp_target_error = None;
            }
            Msg::AddRtpTarget => {
                match parse_new_rtp_target(&self.new_rtp_target, &self.new_rtp_bitrate) {
                    Ok((target, bitrate_kbps)) => {
                        self.send_message(
                            flo_core::FloCommand::AddRtpTarget {
                                target,
                                bitrate_kbps,
                            },
                            ctx,
                        );
                        self.adding_rtp_target = false;
                        self.focus_new_rtp_target = false;
                        self.new_rtp_target.clear();
                        self.new_rtp_bitrate = flo_core::DEFAULT_RTP_BITRATE_KBPS.to_string();
                        self.new_rtp_target_error = None;
                    }
                    Err(error) => {
                        self.new_rtp_target_error = Some(error);
                    }
                }
            }
            Msg::SetRtpTargetBitrate(target, bitrate) => {
                if let Ok(bitrate_kbps) = bitrate.parse::<u32>()
                    && bitrate_kbps > 0
                {
                    self.send_message(
                        flo_core::FloCommand::SetRtpTargetBitrate {
                            target,
                            bitrate_kbps,
                        },
                        ctx,
                    );
                }
            }
            Msg::ConfirmRemoveRtpTarget(target) => {
                self.rtp_target_pending_removal = Some(target);
            }
            Msg::CancelRemoveRtpTarget => {
                self.rtp_target_pending_removal = None;
            }
            Msg::RemoveRtpTarget => {
                if let Some(target) = self.rtp_target_pending_removal.take() {
                    self.send_message(flo_core::FloCommand::RemoveRtpTarget(target), ctx);
                }
            }

            Msg::SetHomePosition => {
                if let Ok(pan) = self.pan_center_degrees.parsed()
                    && let Ok(tilt) = self.tilt_center_degrees.parsed()
                    && let Ok(distance) = self.distance_center.parsed()
                {
                    let msg = flo_core::FloCommand::SetHomePosition((
                        Some(Angle::from_degrees(pan.0)),
                        Some(Angle::from_degrees(tilt.0)),
                        Some(RadialDistance::new(distance.0)),
                    ));
                    self.send_message(msg, ctx);
                }
                return false; // don't update DOM, do that on return
            }
            Msg::SwitchToOpenLoop => {
                self.send_message(flo_core::FloCommand::SwitchToOpenLoop, ctx);
                return false; // don't update DOM, do that on return
            }
            Msg::SetHomePositionFromCurrent => {
                self.send_message(flo_core::FloCommand::SetHomePositionFromCurrent, ctx);
                return false; // don't update DOM, do that on return
            }
            Msg::SwitchToClosedLoop => {
                let msg = flo_core::FloCommand::SwitchMode(
                    flo_core::DeviceMode::AcquiringLock,
                    flo_core::ModeChangeReason::Operator,
                );
                self.send_message(msg, ctx);
                return false; // don't update DOM, do that on return
            }
            Msg::EsReady(response) => {
                let response = *response; // unbox
                match response {
                    Ok(bui_event_data) => {
                        match bui_event_data {
                            BuiEventData::DeviceState(from_device) => {
                                let new_state: DeviceState = from_device;
                                // TODO: check timestamps before setting these?

                                let home_pan_deg = new_state.home_position.0.degrees();
                                let home_tilt_deg = new_state.home_position.1.degrees();
                                self.pan_center_degrees
                                    .set_if_not_focused(AngleDegrees(home_pan_deg));
                                self.tilt_center_degrees
                                    .set_if_not_focused(AngleDegrees(home_tilt_deg));
                                self.distance_center.set_if_not_focused(DistanceMeters(
                                    new_state.home_position.2.0,
                                ));

                                self.floz_recording_path = new_state.floz_recording_path.clone();
                                self.webcam_recording_path =
                                    new_state.webcam_recording_path.clone();
                                self.precapture_seconds
                                    .set_if_not_focused(Seconds(new_state.precapture_window_secs));
                                self.last_state = Some(new_state);
                            }
                            BuiEventData::Config(cfg) => {
                                if let Some(focus_config) = cfg.focus_motor_config.as_ref() {
                                    let distance_corr_m = focus_config.cal.distance_offset;
                                    self.distance_corr_m
                                        .set_if_not_focused(DistanceMeters(distance_corr_m.0));
                                }

                                self.cfg = Some(cfg);
                            }
                            BuiEventData::StrandCameras(cameras) => {
                                self.strand_cameras = cameras;
                            }
                        }
                    }
                    Err(e) => {
                        log::error!("{}", e);
                    }
                };
            }
        }
        true
    }

    fn view(&self, ctx: &Context<Self>) -> Html {
        if self.server_quit {
            // Render only the "has quit" screen. Rendering nothing else stops
            // the rest of the UI (and any of its timers) from running against
            // the now-gone server.
            return self.server_quit_dialog();
        }
        html! {
            <div>
                <h1>{"FLO"}</h1>
                {self.disconnected_dialog()}
                <div style="text-align: center;"><ConnectDevice /></div>
                { self.browser_info() }
                <div class="border-1px">
                    <h2>{"Info"}</h2>
                    { self.info_div() }
                </div>
                <div class="border-1px">
                    <h2>{"Mode"}</h2>
                    <div>
                        <RecordingPathWidget
                        label="Record .floz file"
                        value={self.floz_recording_path.clone()}
                        ontoggle={ctx.link().callback(|checked| {Msg::DoRecordMotorPositionsFloz(checked)})}
                        />
                    </div>
                    { self.tracking_cam_mp4_view(ctx) }
                    { self.precapture_view(ctx) }
                    <div class="button-holder">
                        <Button title="Set Home" onsignal={ctx.link().callback(|_| Msg::SetHomePositionFromCurrent)}/>
                    </div>
                    <div class="button-holder">
                        <Button title="Go Home" onsignal={ctx.link().callback(|_| Msg::SwitchToOpenLoop)}/>
                    </div>
                    <div class="button-holder">
                        <Button title="Track" onsignal={ctx.link().callback(|_| Msg::SwitchToClosedLoop)}/>
                    </div>
                </div>
                <div class="border-1px">
                    <h2>{"Cameras"}</h2>
                    { self.camera_links() }
                    { self.camshow_display_view(ctx) }
                </div>
                <div class="border-1px">
                    <h2>{"H.264 RTP Targets"}</h2>
                    { self.rtp_targets_view(ctx) }
                </div>
                { self.add_rtp_target_dialog(ctx) }
                { self.remove_rtp_target_dialog(ctx) }
                <div class="border-1px">
                    <h2>{"Home Position"}</h2>
                    <div class="my-padding">
                        <label>{"PAN (degrees)"}
                            <TypedInput<AngleDegrees>
                                storage={self.pan_center_degrees.clone()}
                                placeholder={"pan"}
                                on_input={ctx.link().callback(|_| Msg::SetHomePosition)}
                                />
                        </label>
                    </div>
                    <div class="my-padding">
                        <label>{"TILT (degrees)"}
                            <TypedInput<AngleDegrees>
                                storage={self.tilt_center_degrees.clone()}
                                placeholder={"tilt"}
                                on_input={ctx.link().callback(|_| Msg::SetHomePosition)}
                                />
                        </label>
                    </div>
                    <div class="my-padding">
                        <label>{"FOCUS DISTANCE (meters)"}
                            <TypedInput<DistanceMeters>
                                storage={self.distance_center.clone()}
                                placeholder={"focus"}
                                on_input={ctx.link().callback(|_| Msg::SetHomePosition)}
                                />
                        </label>
                    </div>
                    <div class="my-padding">
                        <label>{"focus correction (m)"}
                            <TypedInput<DistanceMeters>
                                storage={self.distance_corr_m.clone()}
                                placeholder={"m"}
                                on_input={ctx.link().callback(|_| Msg::SetDistanceCorrection)}
                                />
                        </label>
                    </div>
                    <div class="button-holder">
                        <Button title="---" onsignal={ctx.link().callback(|_| Msg::AdjustFocus(-3))}/>
                        <Button title="--" onsignal={ctx.link().callback(|_| Msg::AdjustFocus(-2))}/>
                        <Button title="-" onsignal={ctx.link().callback(|_| Msg::AdjustFocus(-1))}/>
                        <Button title="0" onsignal={ctx.link().callback(|_| Msg::AdjustFocus(0))}/>
                        <Button title="+" onsignal={ctx.link().callback(|_| Msg::AdjustFocus(1))}/>
                        <Button title="++" onsignal={ctx.link().callback(|_| Msg::AdjustFocus(2))}/>
                        <Button title="+++" onsignal={ctx.link().callback(|_| Msg::AdjustFocus(3))}/>
                    </div>
                </div>

                <div class="border-1px">
                    <h2>{"Device State"}</h2>

                    { self.view_state() }
                </div>
            </div>
        }
    }
}

impl App {
    fn camshow_display_view(&self, ctx: &Context<Self>) -> Html {
        let selected = self
            .last_state
            .as_ref()
            .map(|state| state.display_source)
            .unwrap_or_default();
        let has_main = self
            .strand_cameras
            .iter()
            .any(|camera| camera.role == StrandCamRole::Main);
        let has_secondary = self
            .strand_cameras
            .iter()
            .any(|camera| camera.role == StrandCamRole::Secondary);

        let choice = |label: &'static str, source: DisplaySource, enabled: bool| {
            html! {
                <label class="camshow-source-choice">
                    <input
                        type="radio"
                        name="camshow-display-source"
                        checked={selected == source}
                        disabled={!enabled}
                        onchange={ctx.link().callback(move |_| Msg::SetDisplaySource(source))}
                        />
                    {label}
                </label>
            }
        };

        html! {
            <div class="my-padding">
                <h3>{"Camshow display"}</h3>
                <div class="camshow-source-choices">
                    {choice("FPV webcam", DisplaySource::Webcam, true)}
                    {choice("Main tracking camera", DisplaySource::StrandCamMain, has_main)}
                    {choice(
                        "Secondary tracking camera",
                        DisplaySource::StrandCamSecondary,
                        has_secondary,
                    )}
                </div>
            </div>
        }
    }

    fn rtp_targets_view(&self, ctx: &Context<Self>) -> Html {
        let targets = self
            .last_state
            .as_ref()
            .map(|state| &state.rtp_targets)
            .cloned()
            .unwrap_or_default();
        html! {
            <div class="my-padding">
                if targets.is_empty() {
                    <p>{"camshow is not currently streaming H.264 to any targets."}</p>
                } else {
                    <ul class="rtp-target-list">
                        { for targets.into_iter().map(|target| {
                            let addr = target.addr.to_string();
                            let addr_for_bitrate = addr.clone();
                            let addr_for_delete = addr.clone();
                            html! {
                                <li key={addr.clone()}>
                                    <code>{addr}</code>
                                    <label class="rtp-target-bitrate">
                                        <input
                                            type="number"
                                            min="1"
                                            value={target.bitrate_kbps.to_string()}
                                            onchange={ctx.link().callback(move |event: Event| {
                                                Msg::SetRtpTargetBitrate(
                                                    addr_for_bitrate.clone(),
                                                    event.target_unchecked_into::<HtmlInputElement>().value(),
                                                )
                                            })}
                                            />
                                        {" kbps"}
                                    </label>
                                    <button
                                        class="btn rtp-target-delete"
                                        onclick={ctx.link().callback(move |_| Msg::ConfirmRemoveRtpTarget(addr_for_delete.clone()))}
                                        >{"Delete"}</button>
                                </li>
                            }
                        }) }
                    </ul>
                }
                <button class="btn" onclick={ctx.link().callback(|_| Msg::ShowAddRtpTarget)}>
                    {"Add target"}
                </button>
            </div>
        }
    }

    fn add_rtp_target_dialog(&self, ctx: &Context<Self>) -> Html {
        if !self.adding_rtp_target {
            return html! {};
        }
        // Enter accepts, Escape dismisses. The modal has no <form>, so without
        // this the only way out is the mouse.
        let onkeydown =
            ctx.link()
                .batch_callback(|event: KeyboardEvent| match event.key().as_str() {
                    "Enter" => Some(Msg::AddRtpTarget),
                    "Escape" => Some(Msg::CancelAddRtpTarget),
                    _ => None,
                });
        html! {
            <div class="modal-container rtp-target-modal" onkeydown={onkeydown}>
                <h2>{"Add H.264 RTP target"}</h2>
                <p>{"Enter the destination as host:port (for example, 192.168.1.20:5600)."}</p>
                <label class="rtp-target-address">
                    {"Destination"}
                    <input
                        type="text"
                        ref={self.new_rtp_target_ref.clone()}
                        placeholder={"192.168.1.20:5600"}
                        value={self.new_rtp_target.clone()}
                        oninput={ctx.link().callback(|event: InputEvent| {
                            Msg::SetNewRtpTarget(event.target_unchecked_into::<HtmlInputElement>().value())
                        })}
                        />
                </label>
                <label class="rtp-target-bitrate">
                    {"Bitrate "}
                    <input
                        type="number"
                        min="1"
                        value={self.new_rtp_bitrate.clone()}
                        oninput={ctx.link().callback(|event: InputEvent| {
                            Msg::SetNewRtpBitrate(event.target_unchecked_into::<HtmlInputElement>().value())
                        })}
                        />
                    {" kbps"}
                </label>
                if let Some(error) = &self.new_rtp_target_error {
                    <p class="rtp-target-error">{error}</p>
                }
                <div class="button-holder">
                    <button class="btn" onclick={ctx.link().callback(|_| Msg::AddRtpTarget)}>{"Add"}</button>
                    <button class="btn" onclick={ctx.link().callback(|_| Msg::CancelAddRtpTarget)}>{"Cancel"}</button>
                </div>
            </div>
        }
    }

    fn remove_rtp_target_dialog(&self, ctx: &Context<Self>) -> Html {
        let Some(target) = self.rtp_target_pending_removal.as_deref() else {
            return html! {};
        };
        html! {
            <div class="modal-container rtp-target-modal">
                <h2>{"Remove H.264 RTP target?"}</h2>
                <p>{format!("Stop streaming H.264 to {target}?")}</p>
                <div class="button-holder">
                    <button class="btn" onclick={ctx.link().callback(|_| Msg::RemoveRtpTarget)}>{"Remove"}</button>
                    <button class="btn" onclick={ctx.link().callback(|_| Msg::CancelRemoveRtpTarget)}>{"Cancel"}</button>
                </div>
            </div>
        }
    }

    fn camera_links(&self) -> Html {
        if self.strand_cameras.is_empty() {
            return html! { <p>{"No cameras configured."}</p> };
        }

        fn role_str(role: &StrandCamRole) -> &'static str {
            match role {
                StrandCamRole::Main => " (Main)",
                StrandCamRole::Secondary => " (Secondary)",
            }
        }

        html! {
            <ul>
                { for self.strand_cameras.iter().map(|camera| {
                    html! {
                        <li key={camera.name.clone()}>
                            <a href={camera.proxy_prefix.clone()}>{&camera.name}</a> {role_str(&camera.role)}
                        </li>
                    }
                }) }
            </ul>
        }
    }

    /// Whether the tracking cameras' MP4 recordings follow the `.floz`
    /// recording. With this on, one record command — the button above, the
    /// post-trigger button, or arming over MAVLink — saves every source for
    /// the same span.
    fn tracking_cam_mp4_view(&self, ctx: &Context<Self>) -> Html {
        let checked = self
            .last_state
            .as_ref()
            .map(|state| state.record_tracking_cam_mp4)
            .unwrap_or_default();
        html! {
            <div class="my-padding">
                <label>
                    <input
                        type="checkbox"
                        checked={checked}
                        onchange={ctx.link().callback(move |_| Msg::SetRecordTrackingCamMp4(!checked))}
                        />
                    {" Also record tracking camera .mp4 files with the .floz"}
                </label>
            </div>
        }
    }

    /// Pre-capture ("post-trigger") controls. Setting the window above zero
    /// makes the writer continuously buffer recent data in RAM. The separate
    /// "Post-trigger record" button starts a recording that begins with that
    /// buffered window, letting the operator capture an event that already
    /// happened. (The normal record button above is unaffected and records
    /// from now.) The readout shows how many seconds are currently buffered.
    fn precapture_view(&self, ctx: &Context<Self>) -> Html {
        let buffered = self
            .last_state
            .as_ref()
            .map(|s| s.precapture_buffered_secs)
            .unwrap_or(0.0);
        let window = self
            .last_state
            .as_ref()
            .map(|s| s.precapture_window_secs)
            .unwrap_or(0.0);
        let recording = self.floz_recording_path.is_some();
        let status = if window <= 0.0 {
            "disabled".to_string()
        } else if recording {
            "recording".to_string()
        } else {
            format!("buffered: {buffered:.1} s ready")
        };
        // With no window there is nothing to flush, so the button would just
        // duplicate the record toggle above while looking like it captured the
        // past. Say so rather than let it silently do the wrong thing.
        let disabled = window <= 0.0 || recording;
        html! {
            <div class="my-padding">
                <label>{"Pre-capture buffer (seconds, 0 disables) "}
                    <TypedInput<Seconds>
                        storage={self.precapture_seconds.clone()}
                        placeholder={"seconds"}
                        on_input={ctx.link().callback(|_| Msg::SetPreCaptureSeconds)}
                        />
                </label>
                <span style="margin-left: 0.5em;">{status}</span>
                <div class="button-holder">
                    <Button
                        title="Post-trigger record"
                        disabled={disabled}
                        onsignal={ctx.link().callback(|_| Msg::DoPreCaptureRecord)}
                        />
                </div>
            </div>
        }
    }

    fn handle_gamepad_interval(&mut self, ctx: &Context<Self>, cfg: FloControllerConfig) {
        for js_val in gloo_utils::window()
            .navigator()
            .get_gamepads()
            .unwrap_throw()
            .iter()
            .filter(|js_val| !js_val.is_null())
        {
            let gamepad = js_val.dyn_ref::<web_sys::Gamepad>().unwrap_throw();
            if !gamepad.connected() {
                // if gamepad is not connected, skip it and go to next.
                continue;
            }
            let timestamp = gamepad.timestamp();
            if timestamp <= self.last_gamepad_timestamp {
                // no new joystick data
                continue;
            }
            let buttons: Vec<bool> = gamepad
                .buttons()
                .iter()
                .map(|val| {
                    let button = val.dyn_ref::<web_sys::GamepadButton>().unwrap_throw();
                    button.pressed()
                })
                .take(4)
                .collect();
            let axes: Vec<f64> = gamepad
                .axes()
                .iter()
                .map(|val| val.as_f64().unwrap_throw())
                .take(2)
                .collect();
            // log::info!("buttons: {:?}, axes: {:?}", buttons, axes);
            if buttons.len() < 4 {
                panic!("expected minimum 4 gamepad buttons");
            }
            if axes.len() < 2 {
                panic!("expected minimum 2 gamepad axes");
            }

            self.last_gamepad_timestamp = timestamp;

            if buttons[0] {
                ctx.link().send_message(Msg::SwitchToClosedLoop)
            }
            if buttons[1] {
                ctx.link().send_message(Msg::SwitchToOpenLoop)
            }

            {
                let minmax = (
                    cfg.pan_motor_config.endpoint_low.degrees(),
                    cfg.pan_motor_config.endpoint_high.degrees(),
                );
                self.pan_center_degrees
                    .modify(|val| {
                        val.0 += (JOYGAIN * axes[0]) as FloatType;
                        let (min, max) = minmax;
                        if val.0 <= min {
                            val.0 = min
                        };
                        if val.0 >= max {
                            val.0 = max
                        }
                    })
                    .unwrap_throw();
            }

            {
                let minmax = (
                    cfg.tilt_motor_config.endpoint_low.degrees(),
                    cfg.tilt_motor_config.endpoint_high.degrees(),
                );

                self.tilt_center_degrees
                    .modify(|val| {
                        val.0 += (-JOYGAIN * axes[1]) as FloatType;
                        let (min, max) = minmax;
                        if val.0 <= min {
                            val.0 = min
                        };
                        if val.0 >= max {
                            val.0 = max
                        }
                    })
                    .unwrap_throw();
            }

            // let pan = self.pan_center_degrees.get().unwrap_throw();
            // let tilt = self.tilt_center_degrees.get().unwrap_throw();
            // log::info!("pan: {pan}, tilt: {tilt}");

            // We got data from one gamepad, end the loop now.
            break;
        }
    }
    // fn set_device_mode(&mut self, new_mode: flo_core::DeviceMode, ctx: &Context<Self>) {
    //     match &self.last_state {
    //         None => {
    //             // do nothing
    //         }
    //         Some(s) => {
    //             let mut initial = s.device_config.clone();
    //             initial.mode = new_mode;
    //             let msg = flo_core::ToDevice::SetDevConfig(initial);
    //             self.send_message(msg, ctx);
    //         }
    //     };
    // }

    fn disconnected_dialog(&self) -> Html {
        // 0: connecting, 1: open, 2: closed
        if self.es.ready_state() == 1 {
            html! {
               <div>
                 { "" }
               </div>
            }
        } else {
            html! {
                <div class="modal-container">
                    <h1> { "Web browser not connected to FLO" } </h1>
                    <p>{ format!("Connection State: {:?}", ReadyState::from(self.es.ready_state())) }</p>
                    <p>{ "Please restart FLO and reload this webpage" }</p>
                </div>
            }
        }
    }

    fn server_quit_dialog(&self) -> Html {
        html! {
            <div class="modal-container">
                <h1> { "FLO has quit" } </h1>
                <p>{ "The FLO server has shut down. Restart FLO and reload this \
                      webpage to reconnect." }</p>
            </div>
        }
    }

    fn n_connected_gamepads(&self) -> usize {
        let mut count = 0;
        for js_val in gloo_utils::window()
            .navigator()
            .get_gamepads()
            .unwrap_throw()
            .iter()
            .filter(|js_val| !js_val.is_null())
        {
            let gamepad = js_val.dyn_ref::<web_sys::Gamepad>().unwrap_throw();
            if gamepad.connected() {
                count += 1;
            }
        }
        count
    }

    fn browser_info(&self) -> Html {
        let gamepad = if self.query_gamepad_interval.is_some() {
            "🕹"
        } else {
            "\u{200b}" // unicode zero width space character
        };
        let ready_state = match ReadyState::from(self.es.ready_state()) {
            ReadyState::Closed => "Connection: ❌",
            ReadyState::Open => "\u{200b}", // unicode zero width space character
            ReadyState::Connecting => "Connection: ⋯",
        };
        html! {
            <div>
                { ready_state }{ gamepad }
            </div>
        }
    }

    fn info_div(&self) -> Html {
        if let Some(ref state) = self.last_state {
            let (distance, disparity) = match state.stereopsis_state.as_ref() {
                Some(ss) => (format!("{:.2}m", ss.dist), format!("{:.2}px", ss.dx)),
                None => ("\u{200b}".to_string(), "\u{200b}".to_string()), // unicode zero width space character
            };
            let pan_deg = format!("{:.1}°", state.cached_motors.pan.degrees());
            let tilt_deg = format!("{:.1}°", state.cached_motors.tilt.degrees());
            let cam_state = state.cam_stale.as_msg();
            html! {
                <div>
                    <div class="qqblock">
                       <div class="qqkey">{ "Mode" }</div>
                       <div class="qqvalue"> {format!("{}", state.mode) } </div>
                    </div>
                    <div class="qqblock">
                        <div class="qqkey">{ "Current Cam Data?" }</div>
                        <div class="qqvalue"> {cam_state} </div>
                    </div>
                    <div class="qqblock">
                        <div class="qqkey">{ "Distance" }</div>
                        <div class="qqvalue"> {distance} </div>
                    </div>
                    <div class="qqblock">
                        <div class="qqkey">{ "Disparity" }</div>
                        <div class="qqvalue"> {disparity} </div>
                    </div>

                    <div class="qqblock">
                        <div class="qqkey">{ "Pan" }</div>
                        <div class="qqvalue"> {pan_deg} </div>
                    </div>

                    <div class="qqblock">
                        <div class="qqkey">{ "Tilt" }</div>
                        <div class="qqvalue"> {tilt_deg} </div>
                    </div>

                    { gps_origin_div(&state.gps_origin) }
                </div>
            }
        } else {
            html! {}
        }
    }

    fn view_state(&self) -> Html {
        if let Some(ref state) = self.last_state {
            let state_string = serde_yaml::to_string(state).unwrap();
            html! {
                <div>
                    <p>{"Device ID: "}{format!("{:?}",state.device_id)}</p>
                    <p>{"Mode: "}{format!("{:?}",state.mode)}</p>
                    <div class="preformatted">
                        {state_string}
                    </div>
                </div>
            }
        } else {
            html! {<div></div>}
        }
    }

    fn send_message(&self, data: flo_core::FloCommand, ctx: &Context<Self>) {
        ctx.link().send_future(async move {
            match post_message(&data).await {
                Ok(()) => Msg::SendMessageFetchState(FetchState::Success),
                Err(err) => Msg::SendMessageFetchState(FetchState::Failed(err)),
            }
        });
    }
}

// -----------------------------------------------------------------------------

async fn post_message(msg: &flo_core::FloCommand) -> Result<(), FetchError> {
    use web_sys::{Request, RequestInit, Response};
    let opts = RequestInit::new();
    opts.set_method("POST");
    opts.set_cache(web_sys::RequestCache::NoStore);
    let buf = serde_json::to_string(&msg).unwrap_throw();
    opts.set_body(&JsValue::from_str(&buf));
    let headers = web_sys::Headers::new().unwrap_throw();
    headers
        .append("Content-Type", "application/json")
        .unwrap_throw();
    opts.set_headers(&headers);

    let url = "callback";
    let request = Request::new_with_str_and_init(url, &opts)?;

    let window = gloo_utils::window();
    let resp_value = JsFuture::from(window.fetch_with_request(&request)).await?;
    let resp: Response = resp_value.dyn_into().unwrap_throw();

    let text = JsFuture::from(resp.text()?).await?;
    let _text_string = text.as_string().unwrap_throw();
    Ok(())
}

// -----------------------------------------------------------------------------

/// The flight controller's local-position origin: what FLO asked for, what the
/// flight controller reports, and whether they agree.
///
/// Everything FLO derives from `LOCAL_POSITION_NED` is relative to this point,
/// so an origin that did not take hold has to be visible rather than buried in
/// a log file.
fn gps_origin_div(status: &GpsOriginStatus) -> Html {
    if status.check == GpsOriginCheck::NotRequested && status.reported.is_none() {
        // No flight controller, or no origin configured and none reported.
        return html! {};
    }
    let value = match status.reported {
        Some(origin) => origin.to_string(),
        None => "waiting for flight controller".to_string(),
    };
    let (note, class) = match status.check {
        GpsOriginCheck::NotRequested => (String::new(), "gps-origin-ok"),
        GpsOriginCheck::Awaiting => (
            "not yet confirmed by the flight controller".to_string(),
            "gps-origin-warn",
        ),
        GpsOriginCheck::Confirmed => ("✓ confirmed".to_string(), "gps-origin-ok"),
        GpsOriginCheck::Mismatched => (
            match status.requested {
                Some(requested) => {
                    format!("⚠ DID NOT STICK — configured origin is {requested}")
                }
                None => "⚠ did not stick".to_string(),
            },
            "gps-origin-error",
        ),
    };
    html! {
        <div class="qqblock">
            <div class="qqkey">{ "GPS origin (lat, lon, alt)" }</div>
            <div class="qqvalue">{ value }</div>
            if !note.is_empty() {
                <div class={class}>{ note }</div>
            }
        </div>
    }
}

/// Validate the add-target modal's two fields.
///
/// On success the trimmed destination and its bitrate are returned; on failure
/// the message to show in the modal. The destination must be a numeric address
/// and port because nothing in the browser can resolve a name for `camshow`.
fn parse_new_rtp_target(target: &str, bitrate: &str) -> Result<(String, u32), String> {
    let target = target.trim();
    if target.is_empty() {
        return Err("Enter a destination, such as 192.168.1.20:5600.".to_owned());
    }
    if target.parse::<SocketAddr>().is_err() {
        return Err(format!(
            "\"{target}\" is not a numeric address and port, such as 192.168.1.20:5600."
        ));
    }
    match bitrate.trim().parse::<u32>() {
        Ok(bitrate_kbps) if bitrate_kbps > 0 => Ok((target.to_owned(), bitrate_kbps)),
        _ => Err("The bitrate must be a whole number of kbps greater than zero.".to_owned()),
    }
}

// -----------------------------------------------------------------------------

pub fn main() {
    set_panic_hook();
    wasm_logger::init(wasm_logger::Config::default());
    yew::Renderer::<App>::new().render();
}

#[derive(Debug)]
enum ReadyState {
    Connecting,
    Open,
    Closed,
}

impl From<u16> for ReadyState {
    fn from(orig: u16) -> ReadyState {
        // https://developer.mozilla.org/en-US/docs/Web/API/EventSource/readyState
        match orig {
            0 => ReadyState::Connecting,
            1 => ReadyState::Open,
            2 => ReadyState::Closed,
            other => panic!("unknown ReadyState: {other}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::parse_new_rtp_target;

    #[test]
    fn accepts_a_numeric_address_and_port() {
        assert_eq!(
            parse_new_rtp_target(" 192.168.1.20:5600 ", "4000"),
            Ok(("192.168.1.20:5600".to_owned(), 4000))
        );
    }

    #[test]
    fn accepts_a_bracketed_ipv6_address() {
        assert_eq!(
            parse_new_rtp_target("[::1]:5600", "1"),
            Ok(("[::1]:5600".to_owned(), 1))
        );
    }

    #[test]
    fn rejects_an_empty_or_unresolvable_destination() {
        assert!(parse_new_rtp_target("  ", "4000").is_err());
        // A name cannot be resolved in the browser, so it is rejected here
        // rather than silently dropped by the controller.
        assert!(parse_new_rtp_target("groundstation:5600", "4000").is_err());
        assert!(parse_new_rtp_target("192.168.1.20", "4000").is_err());
    }

    #[test]
    fn rejects_a_zero_or_unparseable_bitrate() {
        assert!(parse_new_rtp_target("192.168.1.20:5600", "0").is_err());
        assert!(parse_new_rtp_target("192.168.1.20:5600", "").is_err());
        assert!(parse_new_rtp_target("192.168.1.20:5600", "fast").is_err());
    }
}
