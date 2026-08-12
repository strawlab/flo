use console_error_panic_hook::set_once as set_panic_hook;

use std::{collections::BTreeMap, fmt, net::SocketAddr};

use gloo_events::EventListener;
use gloo_timers::callback::Interval;
use wasm_bindgen::{JsCast, JsValue, prelude::*};
use wasm_bindgen_futures::JsFuture;
use web_sys::{Event, EventSource, Gamepad, GamepadEvent, HtmlInputElement, MessageEvent};

use yew::prelude::*;

use yew_tincture::components::{Button, TypedInput, TypedInputStorage};

use flo_core::*;

mod mobile;

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

/// An H.264 encoder bitrate in kbps, as typed by the operator.
#[derive(Clone, Debug, PartialEq)]
struct Kbps(u32);

/// Why a typed bitrate was rejected.
///
/// `TypedInput` needs the parse error to be `Clone`, which `ParseIntError`
/// already is — but zero has to be rejected too, so both cases end up here.
#[derive(Clone, Debug, PartialEq)]
struct KbpsParseError;

impl std::fmt::Display for KbpsParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "expected a whole number of kbps greater than zero")
    }
}

impl std::error::Error for KbpsParseError {}

impl std::str::FromStr for Kbps {
    type Err = KbpsParseError;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.trim().parse::<u32>() {
            Ok(kbps) if kbps > 0 => Ok(Kbps(kbps)),
            _ => Err(KbpsParseError),
        }
    }
}

impl std::fmt::Display for Kbps {
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
    /// What the running program was built from, sent once on connect. Empty
    /// until it arrives, which is why the footer falls back to this frontend's
    /// own build.
    component_versions: Vec<flo_core::ComponentVersion>,
    es: EventSource,
    _listeners: Vec<EventListener>,
    query_gamepad_interval: Option<Interval>,
    last_gamepad_timestamp: f64,
    /// Set once the server broadcasts that it is shutting down.
    server_quit: bool,
    /// How many events have arrived. Only its parity is used, to restart the
    /// indicator's animation on each one.
    data_events: u64,
    /// When the last event arrived, from `Date::now()` in milliseconds.
    last_data_ms: Option<f64>,
    /// Redraws the liveness indicator once a second.
    ///
    /// Nothing else would: when the server goes quiet there are no events to
    /// re-render on, which is exactly when the indicator has something to say.
    _liveness_interval: Interval,
    /// One editable bitrate per current RTP target, keyed by destination.
    ///
    /// The state broadcast arrives several times a second, so the field cannot
    /// be driven straight from `last_state`: re-rendering would overwrite a
    /// half-typed number. As with the home-position fields, the storage takes
    /// the server's value only while the input is unfocused.
    rtp_bitrates: BTreeMap<SocketAddr, TypedInputStorage<Kbps>>,
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
    /// Whether the phone view is showing, from the URL fragment. See
    /// [`MOBILE_HASH`].
    mobile: bool,
    /// What the machine running FLO calls itself, sent once on connect. `None`
    /// until it arrives, or when the server could not determine a name.
    hostname: Option<String>,
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
    SetRtpTargetBitrate(SocketAddr, Kbps),
    SetRtpTargetEnabled(SocketAddr, bool),
    SetRtpSendEnabled(bool),
    ConfirmRemoveRtpTarget(String),
    CancelRemoveRtpTarget,
    RemoveRtpTarget,
    /// The server broadcast that it is shutting down.
    ServerQuit,
    /// The URL fragment changed, which is how the phone view is entered and
    /// left.
    HashChanged,
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

        {
            // The two views are one app, so switching between them is a
            // fragment change rather than a page load: the event stream, and
            // everything received over it, survives the switch.
            let link = ctx.link().clone();
            _listeners.push(EventListener::new(
                &gloo_utils::window(),
                "hashchange",
                move |_event: &Event| {
                    link.send_message(Msg::HashChanged);
                },
            ));
        }

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
            component_versions: Vec::new(),
            _listeners,
            query_gamepad_interval: None,
            last_gamepad_timestamp: 0.0,
            server_quit: false,
            data_events: 0,
            last_data_ms: None,
            _liveness_interval: {
                let link = ctx.link().clone();
                Interval::new(1_000, move || link.send_message(Msg::RenderView))
            },
            rtp_bitrates: BTreeMap::new(),
            adding_rtp_target: false,
            new_rtp_target: String::new(),
            new_rtp_bitrate: flo_core::DEFAULT_RTP_BITRATE_KBPS.to_string(),
            new_rtp_target_error: None,
            new_rtp_target_ref: NodeRef::default(),
            focus_new_rtp_target: false,
            rtp_target_pending_removal: None,
            mobile: mobile_hash_is_set(),
            hostname: None,
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
            Msg::HashChanged => {
                self.mobile = mobile_hash_is_set();
            }
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
            Msg::SetRtpTargetBitrate(target, Kbps(bitrate_kbps)) => {
                self.send_message(
                    flo_core::FloCommand::SetRtpTargetBitrate {
                        target: target.to_string(),
                        bitrate_kbps,
                    },
                    ctx,
                );
                return false; // Don't update DOM; wait for backend state.
            }
            Msg::SetRtpTargetEnabled(target, enabled) => {
                self.send_message(
                    flo_core::FloCommand::SetRtpTargetEnabled {
                        target: target.to_string(),
                        enabled,
                    },
                    ctx,
                );
                return false; // Don't update DOM; wait for backend state.
            }
            Msg::SetRtpSendEnabled(enable) => {
                self.send_message(flo_core::FloCommand::SetRtpSendEnabled(enable), ctx);
                return false; // Don't update DOM; wait for backend state.
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
                        // Any event means the link is carrying data, whichever
                        // kind it is.
                        self.data_events = self.data_events.wrapping_add(1);
                        self.last_data_ms = Some(js_sys::Date::now());
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

                                // Drop the storage of any target that is gone
                                // and add one for any target that is new, so
                                // the map always mirrors the target list.
                                self.rtp_bitrates.retain(|addr, _| {
                                    new_state
                                        .rtp_targets
                                        .iter()
                                        .any(|config| &config.target.addr == addr)
                                });
                                for config in &new_state.rtp_targets {
                                    let bitrate = Kbps(config.target.bitrate_kbps);
                                    self.rtp_bitrates
                                        .entry(config.target.addr)
                                        .or_insert_with(|| {
                                            TypedInputStorage::from_initial(bitrate.clone())
                                        })
                                        .set_if_not_focused(bitrate);
                                }

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
                            BuiEventData::Versions(versions) => {
                                self.component_versions = versions;
                            }
                            BuiEventData::Hostname(hostname) => {
                                // The tab's title, not only the heading: it is
                                // how a tab is picked out of a row of them, and
                                // what a bookmark of this FLO is called. The
                                // same title in both views — which machine this
                                // is does not change with how it is being
                                // looked at.
                                gloo_utils::document()
                                    .set_title(&page_title(Some(&hostname), None));
                                self.hostname = Some(hostname);
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
        if self.mobile {
            return self.mobile_view(ctx);
        }
        html! {
            <div>
                {self.disconnected_dialog()}
                <header class="app-header">
                    { self.app_title() }
                    { self.browser_info() }
                    <span class="app-header-connect">
                        // Same page, different view: the fragment is picked up
                        // without a reload, so nothing is reconnected.
                        <a class="btn" href={format!("#{MOBILE_HASH}")}>{"Phone"}</a>
                        <ConnectDevice />
                    </span>
                </header>
                <div class="border-1px">
                    // The indicator belongs to the whole panel: without fresh
                    // data every readout below is a stale one.
                    <h2 class="panel-heading">{"Info"}{ self.liveness_indicator() }</h2>
                    { self.info_div() }
                </div>
                <div class="border-1px">
                    <h2>{"Mode"}</h2>
                    <div class="my-padding recording-row">
                        <RecordingPathWidget
                        label="Record .floz file"
                        value={self.floz_recording_path.clone()}
                        ontoggle={ctx.link().callback(|checked| {Msg::DoRecordMotorPositionsFloz(checked)})}
                        />
                    </div>
                    { self.tracking_cam_mp4_view(ctx) }
                    { self.precapture_view(ctx) }
                    <div class="button-holder mode-actions">
                        <Button title="Set Home" onsignal={ctx.link().callback(|_| Msg::SetHomePositionFromCurrent)}/>
                        <Button title="Go Home" onsignal={ctx.link().callback(|_| Msg::SwitchToOpenLoop)}/>
                        <Button title="Track" onsignal={ctx.link().callback(|_| Msg::SwitchToClosedLoop)}/>
                    </div>
                </div>
                <div class="border-1px">
                    <h2>{"Cameras"}</h2>
                    { self.camera_links() }
                    { self.camshow_display_view(ctx) }
                </div>
                { self.mavlink_view() }
                <div class="border-1px">
                    <h2>{"H.264 RTP Targets"}</h2>
                    { self.rtp_targets_view(ctx) }
                </div>
                { self.add_rtp_target_dialog(ctx) }
                { self.remove_rtp_target_dialog(ctx) }
                <div class="border-1px">
                    <h2>{"Home Position"}</h2>
                    <label class="field-row">{"PAN (degrees)"}
                        <TypedInput<AngleDegrees>
                            storage={self.pan_center_degrees.clone()}
                            placeholder={"pan"}
                            on_input={ctx.link().callback(|_| Msg::SetHomePosition)}
                            />
                    </label>
                    <label class="field-row">{"TILT (degrees)"}
                        <TypedInput<AngleDegrees>
                            storage={self.tilt_center_degrees.clone()}
                            placeholder={"tilt"}
                            on_input={ctx.link().callback(|_| Msg::SetHomePosition)}
                            />
                    </label>
                    <label class="field-row">{"FOCUS DISTANCE (meters)"}
                        <TypedInput<DistanceMeters>
                            storage={self.distance_center.clone()}
                            placeholder={"focus"}
                            on_input={ctx.link().callback(|_| Msg::SetHomePosition)}
                            />
                    </label>
                    <label class="field-row">{"focus correction (m)"}
                        <TypedInput<DistanceMeters>
                            storage={self.distance_corr_m.clone()}
                            placeholder={"m"}
                            on_input={ctx.link().callback(|_| Msg::SetDistanceCorrection)}
                            />
                    </label>
                    <div class="button-holder focus-actions">
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
                <footer id="footer">
                    { self.versions_view() }
                </footer>
            </div>
        }
    }
}

impl App {
    /// The heading both views carry: FLO, and which machine's FLO this is.
    ///
    /// The name is the machine's own, reported by the server (see
    /// [`BuiEventData::Hostname`]); with several FLOs around, "FLO" alone does
    /// not say which one this browser is driving. Until it arrives, and on a
    /// machine that reports no name, the heading is just FLO.
    fn app_title(&self) -> Html {
        html! {
            <h1 class="app-title">
                // The space is a text node rather than a gap between two boxes
                // so that the heading is still "FLO strawbot" when it is read
                // aloud or copied, not "FLOstrawbot".
                {"FLO "}
                if let Some(hostname) = &self.hostname {
                    <span class="app-title-host">{hostname}</span>
                }
            </h1>
        }
    }

    /// What the running program was built from, one line per component.
    ///
    /// Sent by the server rather than compiled in, because the interesting
    /// answer is usually not this frontend's own build: FLO is often one
    /// component of a binary built in another repository, with extensions from
    /// elsewhere again, and only each of those can report its own revision.
    /// Until that arrives, this frontend's build is all there is to show.
    fn versions_view(&self) -> Html {
        if self.component_versions.is_empty() {
            return html! {
                <span class="version-line">{format!(
                    "FLO web UI {} (revision {})",
                    env!("CARGO_PKG_VERSION"),
                    env!("GIT_HASH"),
                )}</span>
            };
        }
        html! {
            <>
            { for self.component_versions.iter().map(|component| html! {
                <span class="version-line" key={component.name.clone()}>
                    { component.to_string() }
                </span>
            }) }
            </>
        }
    }

    /// The FPV webcam's entry in [`Self::camera_links`], or `None` when camshow
    /// is not configured and there is therefore no webcam to preview.
    ///
    /// The preview costs work in camshow and in flo for as long as the page is
    /// up, so it is something the operator opens beside the main UI and closes
    /// again. A plain link rather than a scripted popup, so popup blockers
    /// leave it alone and the window goes where they want it.
    fn webcam_preview_link(&self) -> Option<Html> {
        let configured = self
            .cfg
            .as_ref()
            .and_then(|cfg| cfg.osd_config.as_ref())
            .is_some_and(|osd| osd.camshow_addr.is_some());
        if !configured {
            return None;
        }
        Some(html! {
            <li key="fpv-webcam">
                <a
                    href={format!("/{}", flo_core::WEBCAM_PREVIEW_PATH)}
                    target="_blank"
                    rel="noopener"
                >{"FPV webcam"}</a>
                {" (Preview)"}
            </li>
        })
    }

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

    /// The current RTP destinations, each with an editable bitrate.
    ///
    /// The rows come from `rtp_bitrates` rather than from `last_state` so that
    /// what is drawn and what the input fields hold cannot drift apart. The
    /// bitrate is sent when the field loses the focus or on Enter, which is
    /// what makes a typed value stick across the state broadcasts that arrive
    /// while it is being typed.
    /// Whether one destination is switched on. Unknown addresses read as on,
    /// which is what a destination is when it is added.
    fn rtp_target_enabled(&self, addr: &SocketAddr) -> bool {
        self.last_state
            .as_ref()
            .and_then(|state| {
                state
                    .rtp_targets
                    .iter()
                    .find(|config| &config.target.addr == addr)
            })
            .map(|config| config.enabled)
            .unwrap_or(true)
    }

    fn rtp_targets_view(&self, ctx: &Context<Self>) -> Html {
        // Defaults to on: without state from the server yet, the switch shows
        // what a running FLO does, rather than reading as "off" for a moment.
        let send_enabled = self
            .last_state
            .as_ref()
            .map(|state| state.rtp_send_enabled)
            .unwrap_or(true);
        html! {
            <div class="my-padding">
                <label class="check-row rtp-send-all">
                    <input
                        type="checkbox"
                        checked={send_enabled}
                        onchange={ctx.link().callback(move |_| Msg::SetRtpSendEnabled(!send_enabled))}
                        />
                    <span>{"Send to all targets"}</span>
                </label>
                if self.rtp_bitrates.is_empty() {
                    <p>{"camshow is not currently streaming H.264 to any targets."}</p>
                } else {
                    if !send_enabled {
                        <p class="rtp-send-off">{"Sending is off. Every target below is kept, \
                                                  with its own setting, and streaming resumes \
                                                  when this is switched back on."}</p>
                    }
                    <ul class="rtp-target-list">
                        { for self.rtp_bitrates.iter().map(|(addr, bitrate)| {
                            let addr_string = addr.to_string();
                            let addr_for_bitrate = *addr;
                            let addr_for_enable = *addr;
                            let addr_for_delete = addr_string.clone();
                            // Each destination carries its own switch; the one
                            // above is the master over all of them.
                            let enabled = self.rtp_target_enabled(addr);
                            html! {
                                <li key={addr_string.clone()} class="rtp-target-item">
                                    <label class="rtp-target-enable">
                                        <input
                                            type="checkbox"
                                            checked={enabled}
                                            onchange={ctx.link().callback(move |_| {
                                                Msg::SetRtpTargetEnabled(addr_for_enable, !enabled)
                                            })}
                                            />
                                    </label>
                                    <code class={classes!("rtp-target-addr", (!enabled).then_some("rtp-target-off"))}>{addr_string}</code>
                                    // The bitrate and Delete stay together as
                                    // one group, so on a narrow screen they
                                    // wrap below the address as a unit.
                                    <div class="rtp-target-controls">
                                        <label class="rtp-target-bitrate">
                                            <TypedInput<Kbps>
                                                storage={bitrate.clone()}
                                                placeholder={"kbps"}
                                                on_send_valid={ctx.link().callback(move |kbps| {
                                                    Msg::SetRtpTargetBitrate(addr_for_bitrate, kbps)
                                                })}
                                                />
                                            {" kbps"}
                                        </label>
                                        <button
                                            class="btn rtp-target-delete"
                                            onclick={ctx.link().callback(move |_| Msg::ConfirmRemoveRtpTarget(addr_for_delete.clone()))}
                                            >{"Delete"}</button>
                                    </div>
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
            <div class="modal-container" onkeydown={onkeydown}>
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
            <div class="modal-container">
                <h2>{"Remove H.264 RTP target?"}</h2>
                <p>{format!("Stop streaming H.264 to {target}?")}</p>
                <div class="button-holder">
                    <button class="btn" onclick={ctx.link().callback(|_| Msg::RemoveRtpTarget)}>{"Remove"}</button>
                    <button class="btn" onclick={ctx.link().callback(|_| Msg::CancelRemoveRtpTarget)}>{"Cancel"}</button>
                </div>
            </div>
        }
    }

    /// Every camera the operator can go and look at: the tracking cameras, and
    /// the FPV webcam alongside them.
    ///
    /// The webcam is reached by a different route — a standalone preview page
    /// rather than a proxied strand-cam UI — but that is our plumbing, not
    /// something the operator should have to think about. From their side it is
    /// one more camera to open, so it is one more entry in this list rather
    /// than a button styled differently from its neighbours.
    ///
    /// Every one of them opens in a new tab. Navigating away in this one would
    /// tear down the event stream and this UI with it — possibly mid-flight —
    /// and a camera is something the operator looks at *beside* the controls,
    /// not instead of them.
    fn camera_links(&self) -> Html {
        fn role_str(role: &StrandCamRole) -> &'static str {
            match role {
                StrandCamRole::Main => " (Main)",
                StrandCamRole::Secondary => " (Secondary)",
            }
        }

        let webcam_preview = self.webcam_preview_link();
        if self.strand_cameras.is_empty() && webcam_preview.is_none() {
            return html! { <p>{"No cameras configured."}</p> };
        }

        html! {
            <ul class="camera-list">
                { for self.strand_cameras.iter().map(|camera| {
                    html! {
                        <li key={camera.name.clone()}>
                            <a
                                href={camera.proxy_prefix.clone()}
                                target="_blank"
                                rel="noopener"
                            >{&camera.name}</a> {role_str(&camera.role)}
                        </li>
                    }
                }) }
                { webcam_preview.unwrap_or_default() }
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
            <label class="check-row">
                <input
                    type="checkbox"
                    checked={checked}
                    onchange={ctx.link().callback(move |_| Msg::SetRecordTrackingCamMp4(!checked))}
                    />
                <span>{"Also record tracking camera .mp4 files with the .floz"}</span>
            </label>
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
            <div>
                <label class="field-row">{"Pre-capture buffer (seconds, 0 disables)"}
                    <TypedInput<Seconds>
                        storage={self.precapture_seconds.clone()}
                        placeholder={"seconds"}
                        on_input={ctx.link().callback(|_| Msg::SetPreCaptureSeconds)}
                        />
                </label>
                <div class="button-holder">
                    <Button
                        title="Post-trigger record"
                        disabled={disabled}
                        onsignal={ctx.link().callback(|_| Msg::DoPreCaptureRecord)}
                        />
                    <span class="precapture-status">{status}</span>
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
            <span class="app-header-status">
                { ready_state }{ gamepad }
            </span>
        }
    }

    /// A pulse for every event that arrives, and how long it has been since the
    /// last one once they stop.
    ///
    /// The dot restarts its animation on each event, so a working link ticks
    /// visibly. This says something the header's indicator cannot: that
    /// indicator reports whether the event stream is *open*, which it remains
    /// when the server has stopped sending — a stalled FLO behind a healthy
    /// connection would otherwise leave every readout below frozen at its last
    /// value with nothing to say so.
    fn liveness_indicator(&self) -> Html {
        let Some(last_data_ms) = self.last_data_ms else {
            return html! {
                <span class="live">
                    <span class="live-dot live-dot-idle"></span>{"waiting for data"}
                </span>
            };
        };
        let age_secs = (js_sys::Date::now() - last_data_ms) / 1000.0;
        // FLO echoes its state once a second, so three missed ticks is a
        // silence rather than a slow tick.
        if age_secs > STALE_DATA_SECS {
            return html! {
                <span class="live live-stale">
                    <span class="live-dot live-dot-stale"></span>
                    {format!("no data for {age_secs:.0} s")}
                </span>
            };
        }
        // Two animations, alternated, because restarting one means handing the
        // element a different animation to run.
        let pulse = if self.data_events.is_multiple_of(2) {
            "live-pulse-a"
        } else {
            "live-pulse-b"
        };
        html! {
            <span class="live">
                <span class={classes!("live-dot", pulse)}></span>{"live"}
            </span>
        }
    }

    fn info_div(&self) -> Html {
        let Some(state) = self.last_state.as_ref() else {
            return html! {};
        };
        let (distance, disparity) = match state.stereopsis_state.as_ref() {
            Some(ss) => (format!("{:.2}m", ss.dist), format!("{:.2}px", ss.dx)),
            None => ("\u{200b}".to_string(), "\u{200b}".to_string()), // unicode zero width space character
        };
        let pan_deg = format!("{:.1}°", state.cached_motors.pan.degrees());
        let tilt_deg = format!("{:.1}°", state.cached_motors.tilt.degrees());
        html! {
            <div class="qqgrid">
                { qqblock("Mode", state.mode.to_string()) }
                { qqblock("Current Cam Data?", state.cam_stale.as_msg().to_string()) }
                { qqblock("Distance", distance) }
                { qqblock("Disparity", disparity) }
                { qqblock("Pan", pan_deg) }
                { qqblock("Tilt", tilt_deg) }
            </div>
        }
    }

    /// The MAVLINK section: what the flight controller reports, laid out like
    /// the Info section above it.
    ///
    /// Nothing is rendered when there is no flight controller, which is also
    /// when the server sends no MAVLink state at all.
    fn mavlink_view(&self) -> Html {
        let Some(mavlink) = self
            .last_state
            .as_ref()
            .and_then(|state| state.mavlink.as_ref())
        else {
            return html! {};
        };

        let flight_mode = match mavlink.custom_mode {
            Some(custom_mode) => flight_mode_label(custom_mode),
            None => NO_DATA.to_string(),
        };
        let rtk = match &mavlink.gnss_rtk_mode {
            Some(mode) => mode.label().to_string(),
            None => NO_DATA.to_string(),
        };
        // Each triple is shown as one readout: all three numbers come from the
        // same message, so they are present or absent together, and keeping them
        // on one line is both how they are read and one row instead of two on a
        // phone.
        let offset = match mavlink.local_position {
            Some(position) => position.horizontal_offset_str(),
            None => NO_DATA.to_string(),
        };
        let position = match mavlink.local_position {
            Some(position) => format!(
                "{:.1}, {:.1}, {:.1} m",
                position.north_m, position.east_m, position.down_m
            ),
            None => NO_DATA.to_string(),
        };
        let attitude = match mavlink.attitude {
            Some(attitude) => format!(
                "{:.1}°, {:.1}°, {:.1}°",
                attitude.yaw_heading_deg(),
                attitude.pitch_deg(),
                attitude.roll_deg()
            ),
            None => NO_DATA.to_string(),
        };

        html! {
            <div class="border-1px">
                <h2>{"MAVLINK"}</h2>
                <div class="qqgrid">
                    { qqblock("Flight mode", flight_mode) }
                    { qqblock("RTK", rtk) }
                    { qqblock("Horizontal offset", offset) }
                    { qqblock_wide("Local position (N, E, D)", position) }
                    { qqblock_wide("Attitude (yaw, pitch, roll)", attitude) }
                    { gps_origin_div(&mavlink.gps_origin) }
                </div>
            </div>
        }
    }

    fn view_state(&self) -> Html {
        if let Some(ref state) = self.last_state {
            let state_string = serde_yaml::to_string(state).unwrap();
            html! {
                <div>
                    <div class="qqgrid">
                        <div class="qqblock">
                            <div class="qqkey">{"Device ID"}</div>
                            <div class="qqvalue">{format!("{:?}",state.device_id)}</div>
                        </div>
                        <div class="qqblock">
                            <div class="qqkey">{"Mode"}</div>
                            <div class="qqvalue">{format!("{:?}",state.mode)}</div>
                        </div>
                    </div>
                    // The full dump is long enough to bury everything below it
                    // on a phone, so it starts collapsed.
                    <details>
                        <summary>{"Full state (YAML)"}</summary>
                        <div class="preformatted">
                            {state_string}
                        </div>
                    </details>
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

/// Shown in place of a value the flight controller has not reported yet.
const NO_DATA: &str = "—";

/// The URL fragment that selects the phone view (see [`mobile`]).
///
/// A fragment rather than a path: it needs no route on the server, and the
/// browser hands it to the app already running instead of loading the page
/// again. Leaving the view is the empty fragment, so the phone view is also one
/// press of Back away.
const MOBILE_HASH: &str = "mobile";

/// Whether `hash` — a URL fragment as `Location::hash` reports it, leading `#`
/// included — selects the phone view.
fn is_mobile_hash(hash: &str) -> bool {
    hash.strip_prefix('#').unwrap_or(hash) == MOBILE_HASH
}

/// Whether the phone view is what the current URL asks for.
fn mobile_hash_is_set() -> bool {
    gloo_utils::window()
        .location()
        .hash()
        .is_ok_and(|hash| is_mobile_hash(&hash))
}

/// How long without an event counts as the data having stopped.
const STALE_DATA_SECS: f64 = 3.0;

/// One key-and-value readout, as the Info and MAVLINK sections are built from.
fn qqblock(key: &str, value: String) -> Html {
    html! {
        <div class="qqblock">
            <div class="qqkey">{ key }</div>
            <div class="qqvalue">{ value }</div>
        </div>
    }
}

/// A readout whose value is too long to share a row with another.
fn qqblock_wide(key: &str, value: String) -> Html {
    html! {
        <div class="qqblock qqblock-wide">
            <div class="qqkey">{ key }</div>
            <div class="qqvalue">{ value }</div>
        </div>
    }
}

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
        <div class="qqblock qqblock-wide">
            <div class="qqkey">{ "GPS origin (lat, lon, alt)" }</div>
            <div class="qqvalue">{ value }</div>
            if !note.is_empty() {
                <div class={class}>{ note }</div>
            }
            if let Some(origin) = status.reported {
                { map_links(origin.latitude_deg(), origin.longitude_deg()) }
            }
        </div>
    }
}

/// Zoom for the OpenStreetMap link: close enough to tell individual buildings
/// apart, wide enough to place the origin in its surroundings.
const MAP_ZOOM: u32 = 17;

/// The two map URLs for `lat`/`lon`.
///
/// Seven decimals is the resolution `GPS_GLOBAL_ORIGIN` carries, so the link
/// points at the same place the readout above it shows.
fn map_urls(lat: FloatType, lon: FloatType) -> (String, String) {
    let lat = format!("{lat:.7}");
    let lon = format!("{lon:.7}");
    (
        format!("https://www.openstreetmap.org/?mlat={lat}&mlon={lon}#map={MAP_ZOOM}/{lat}/{lon}"),
        format!("https://www.google.com/maps/search/?api=1&query={lat},{lon}"),
    )
}

/// Show the origin on a map, so it can be checked against where FLO actually
/// stands instead of being read as bare degrees.
///
/// Both open in a new tab: following a link in this one would tear down the
/// event stream and the UI along with it, possibly mid-flight.
fn map_links(lat: FloatType, lon: FloatType) -> Html {
    let (osm, google) = map_urls(lat, lon);
    html! {
        <div class="map-links">
            <a href={osm} target="_blank" rel="noopener">{"OSM"}</a>
            <a href={google} target="_blank" rel="noopener">{"Google"}</a>
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
    match bitrate.parse::<Kbps>() {
        Ok(Kbps(bitrate_kbps)) => Ok((target.to_owned(), bitrate_kbps)),
        Err(_) => Err("The bitrate must be a whole number of kbps greater than zero.".to_owned()),
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
    use super::{Kbps, is_mobile_hash, map_urls, parse_new_rtp_target};

    #[test]
    fn the_phone_view_is_selected_by_its_own_fragment_only() {
        // `Location::hash` includes the `#`, but the link in the header and the
        // constant it is built from do not, so both spellings must work.
        assert!(is_mobile_hash("#mobile"));
        assert!(is_mobile_hash("mobile"));
        // Anything else is the full UI, including the empty fragment a bare
        // `#` link leaves behind — that link is how the phone view is left.
        assert!(!is_mobile_hash(""));
        assert!(!is_mobile_hash("#"));
        assert!(!is_mobile_hash("#mobile-something"));
    }

    #[test]
    fn builds_both_map_urls_for_an_origin() {
        // OpenStreetMap needs the position twice: `mlat`/`mlon` drop the marker
        // and the fragment sets what the map is looking at. Without the
        // fragment it opens at the last place that browser was looking.
        let (osm, google) = map_urls(48.0021341, 7.8341234);
        assert_eq!(
            osm,
            "https://www.openstreetmap.org/?mlat=48.0021341&mlon=7.8341234\
             #map=17/48.0021341/7.8341234"
        );
        assert_eq!(
            google,
            "https://www.google.com/maps/search/?api=1&query=48.0021341,7.8341234"
        );
    }

    #[test]
    fn a_southwestern_origin_keeps_its_sign() {
        // Negative degrees are what a west-of-Greenwich or southern-hemisphere
        // origin looks like; both services take them as-is, so nothing here may
        // drop the minus.
        let (osm, google) = map_urls(-33.8688, -151.2093);
        assert!(osm.contains("mlat=-33.8688000&mlon=-151.2093000"));
        assert!(google.ends_with("query=-33.8688000,-151.2093000"));
    }

    #[test]
    fn parses_a_bitrate_and_rejects_zero() {
        // The field is trimmed because the operator types into it directly.
        assert_eq!(" 4000 ".parse::<Kbps>(), Ok(Kbps(4000)));
        // Zero would stop the encoder, so it is a parse error and shows up as
        // an invalid field rather than being sent.
        assert!("0".parse::<Kbps>().is_err());
        assert!("-1".parse::<Kbps>().is_err());
        assert!("4.5".parse::<Kbps>().is_err());
        assert!("".parse::<Kbps>().is_err());
    }

    #[test]
    fn a_bitrate_round_trips_through_its_display() {
        // `TypedInputStorage` formats with `Display` and reads back with
        // `FromStr`, so the two have to agree.
        assert_eq!(Kbps(4000).to_string().parse::<Kbps>(), Ok(Kbps(4000)));
    }

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
