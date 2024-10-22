#![recursion_limit = "1024"]

// TODO: keep an actively updated list of all recently heard devices and allow
// the user to select which one to interact with.

use std::fmt;

use gloo_events::EventListener;
use gloo_timers::callback::Interval;
use wasm_bindgen::{prelude::*, JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;
use web_sys::{Event, EventSource, Gamepad, GamepadEvent, MessageEvent};

use yew::prelude::*;

use yew_tincture::components::{Button, TypedInput, TypedInputStorage};

use flo_core::*;

mod recording_path;
use recording_path::RecordingPathWidget;

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

struct App {
    this_recording_path: Option<RecordingPath>,
    pan_center_degrees: TypedInputStorage<AngleDegrees>,
    tilt_center_degrees: TypedInputStorage<AngleDegrees>,
    distance_center: TypedInputStorage<DistanceMeters>,
    distance_corr_m: TypedInputStorage<DistanceMeters>,
    last_state: Option<flo_core::DeviceState>,
    cfg: Option<flo_core::FloControllerConfig>,
    es: EventSource,
    _listeners: Vec<EventListener>,
    query_gamepad_interval: Option<Interval>,
    last_gamepad_timestamp: f64,
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
    DoRecordMotorPositionsCsv(bool),
    SetDistanceCorrection,
    AdjustFocus(i32),
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

        let link = ctx.link().clone();
        _listeners.push(EventListener::new(&es, "error", move |_event: &Event| {
            // Trigger a UI redraw on error, because we won't get any state
            // updates from the server which would otherwise cause a redraw.
            link.send_message(Msg::RenderView);
        }));

        Self {
            es,
            this_recording_path: None,
            pan_center_degrees: TypedInputStorage::from_initial(AngleDegrees(0.0)),
            tilt_center_degrees: TypedInputStorage::from_initial(AngleDegrees(0.0)),
            distance_center: TypedInputStorage::from_initial(DistanceMeters(0.5)),
            distance_corr_m: TypedInputStorage::from_initial(DistanceMeters(0.082)),

            last_state: None,
            cfg: None,
            _listeners,
            query_gamepad_interval: None,
            last_gamepad_timestamp: 0.0,
        }
    }

    fn update(&mut self, ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::RenderView => {}
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
                if let Some(cfg) = self.cfg.as_ref().map(Clone::clone) {
                    self.handle_gamepad_interval(ctx, cfg);
                }
            }
            Msg::SendMessageFetchState(_fetch_state) => {
                return false;
            }
            Msg::DoRecordMotorPositionsCsv(val) => {
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

            Msg::SetHomePosition => {
                if let Ok(pan) = self.pan_center_degrees.parsed() {
                    if let Ok(tilt) = self.tilt_center_degrees.parsed() {
                        if let Ok(distance) = self.distance_center.parsed() {
                            let msg = flo_core::FloCommand::SetHomePosition((
                                Some(Angle::from_degrees(pan.0)),
                                Some(Angle::from_degrees(tilt.0)),
                                Some(RadialDistance::new(distance.0)),
                            ));
                            self.send_message(msg, ctx);
                        }
                    }
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
                                    new_state.home_position.2 .0,
                                ));

                                self.this_recording_path = new_state.recording_path.clone();
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
        html! {
            <div>
                <h1>{"FLO"}</h1>
                {self.disconnected_dialog()}
                { self.browser_info() }
                <div class="border-1px">
                    <h2>{"Info"}</h2>
                    { self.info_div() }
                </div>
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
                    <h2>{"Mode"}</h2>
                    // <div class="button-holder">
                    //     <Button title="Sawtooth" onsignal={ctx.link().callback(|_| Msg::Sawtooth)}/>
                    //     <Button title="Voltage Follower ADC1, ADC2" onsignal={ctx.link().callback(|_| Msg::VoltageFollower12)}/>
                    //     <Button title="Voltage Follower ADC1, ADC3" onsignal={ctx.link().callback(|_| Msg::VoltageFollower13)}/>
                    // </div>
                    <div>
                        <RecordingPathWidget
                        label="Record motor positions .csv file"
                        value={self.this_recording_path.clone()}
                        ontoggle={ctx.link().callback(|checked| {Msg::DoRecordMotorPositionsCsv(checked)})}
                        />
                    </div>
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
                    <h2>{"Device State"}</h2>

                    { self.view_state() }
                </div>
            </div>
        }
    }
}

impl App {
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

            if buttons[0] == true {
                ctx.link().send_message(Msg::SwitchToClosedLoop)
            }
            if buttons[1] == true {
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
                Some(ss) => (format!("{:.2}m", ss.dist), format!("{}px", ss.dx)),
                None => ("\u{200b}".to_string(), "\u{200b}".to_string()), // unicode zero width space character
            };
            let pan_deg = format!("{:.1}°", state.cached_motors.pan.degrees());
            let tilt_deg = format!("{:.1}°", state.cached_motors.tilt.degrees());
            html! {
                <div>
                    <div class="qqblock">
                       <div class="qqkey">{ "Mode" }</div>
                       <div class="qqvalue"> {format!("{}", state.mode) } </div>
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


                </div>
            }
        } else {
            "".to_html()
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

#[wasm_bindgen(start)]
pub fn run_app() {
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
