//! The phone view: the handful of controls that are reached for while standing
//! in a field holding a phone, and nothing else.
//!
//! Not a second frontend but a second view of the same one, selected by the
//! `#mobile` fragment. It therefore shares the event stream, the command
//! plumbing and the state this app already holds, and switching between the two
//! views costs no page load. Everything the full UI shows — the readouts, the
//! home position, the RTP targets, the state dump — is deliberately absent:
//! this is the view for someone who wants to start a recording and point FLO at
//! something without reading anything.

use yew::prelude::*;

use yew_tincture::components::Button;

use strand_cam_bui_components::CamPreview;

use flo_core::StrandCamRole;

use crate::{App, Msg};

/// Aspect ratio for the preview's placeholder box before the first frame
/// arrives. The canvas itself is sized from the image once there is one, so
/// this only keeps the layout from jumping on a camera that is not 4:3.
const PREVIEW_ASPECT_STYLE: &str = "aspect-ratio: 4 / 3;";

impl App {
    pub(crate) fn mobile_view(&self, ctx: &Context<Self>) -> Html {
        html! {
            <div class="mobile">
                {self.disconnected_dialog()}
                <header class="app-header">
                    { self.app_title() }
                    { self.mobile_status() }
                    <a class="btn app-header-connect" href="#">{"Full UI"}</a>
                </header>
                { self.mobile_record_view(ctx) }
                <div class="border-1px">
                    <div class="button-holder mode-actions">
                        <Button title="Set Home" onsignal={ctx.link().callback(|_| Msg::SetHomePositionFromCurrent)}/>
                        <Button title="Go Home" onsignal={ctx.link().callback(|_| Msg::SwitchToOpenLoop)}/>
                        <Button title="Track" onsignal={ctx.link().callback(|_| Msg::SwitchToClosedLoop)}/>
                    </div>
                </div>
                { self.mobile_distance() }
                { self.mobile_gnss_status() }
                { self.mobile_preview() }
            </div>
        }
    }

    /// How far away the thing being tracked is, as stereopsis measures it.
    ///
    /// The one number worth having out here: it says whether FLO has a subject
    /// at a plausible range, and it is what the focus follows. Shown at the
    /// readout size the full UI uses rather than as another line of small text,
    /// because it is read at arm's length.
    ///
    /// An em dash when there is no estimate — a monocular deployment, or no
    /// pair of detections yet. Deliberately not the home position's distance,
    /// which is a setting rather than a measurement.
    fn mobile_distance(&self) -> Html {
        let distance = match self
            .last_state
            .as_ref()
            .and_then(|state| state.stereopsis_state.as_ref())
        {
            Some(stereopsis) => format!("{:.2}m", stereopsis.dist),
            None => crate::NO_DATA.to_string(),
        };
        html! {
            <div class="border-1px">
                { crate::qqblock("Distance", distance) }
            </div>
        }
    }

    /// Primary receiver fix quality, shown only for MAVLink deployments.
    fn mobile_gnss_status(&self) -> Html {
        let Some(mavlink) = self
            .last_state
            .as_ref()
            .and_then(|state| state.mavlink.as_ref())
        else {
            return html! {};
        };
        html! {
            <div class="border-1px">
                { crate::qqblock("GNSS status", crate::format_gnss_status(mavlink)) }
            </div>
        }
    }

    /// The one readout the phone view keeps: which mode FLO is in, beside the
    /// liveness dot.
    ///
    /// Without the mode there is no way to tell whether the Track button took
    /// effect, and without the dot a frozen mode reads as a mode.
    fn mobile_status(&self) -> Html {
        let mode = match self.last_state.as_ref() {
            Some(state) => state.mode.to_string(),
            None => "\u{2014}".to_string(), // em dash
        };
        html! {
            <span class="mobile-status">
                { self.liveness_indicator() }
                <span class="mobile-mode">{ mode }</span>
            </span>
        }
    }

    /// The record control: one button that starts and stops the `.floz`, and the
    /// switch for the tracking cameras' MP4s alongside it.
    ///
    /// The MP4 switch is the same server-side setting the full UI offers, so the
    /// two views cannot disagree about it; it defaults to on because that is
    /// what a running FLO defaults to.
    fn mobile_record_view(&self, ctx: &Context<Self>) -> Html {
        let recording = self.floz_recording_path.is_some();
        let mp4 = self
            .last_state
            .as_ref()
            .map(|state| state.record_tracking_cam_mp4)
            .unwrap_or_default();
        html! {
            <div class="border-1px">
                <div class={classes!("button-holder", "mobile-record", recording.then_some("mobile-recording"))}>
                    <Button
                        title={if recording { "Stop saving" } else { "Save FLOZ" }.to_string()}
                        onsignal={ctx.link().callback(move |_| Msg::DoRecordMotorPositionsFloz(!recording))}
                        />
                </div>
                <label class="check-row">
                    <input
                        type="checkbox"
                        checked={mp4}
                        onchange={ctx.link().callback(move |_| Msg::SetRecordTrackingCamMp4(!mp4))}
                        />
                    <span>{"Also save tracking camera .mp4 files"}</span>
                </label>
            </div>
        }
    }

    /// A live view of the main tracking camera, small enough to leave the
    /// buttons above it in reach of a thumb.
    ///
    /// Mounted only while this view is on screen: the component closes its
    /// stream when it goes away, so the camera stops encoding preview frames as
    /// soon as the operator switches back to the full UI.
    fn mobile_preview(&self) -> Html {
        let Some(main) = self
            .strand_cameras
            .iter()
            .find(|camera| camera.role == StrandCamRole::Main)
        else {
            return html! {
                <div class="border-1px mobile-preview">
                    <p>{"No main tracking camera."}</p>
                </div>
            };
        };
        html! {
            <div class="border-1px mobile-preview">
                <CamPreview
                    proxy_prefix={main.proxy_prefix.clone()}
                    aspect_style={Some(PREVIEW_ASPECT_STYLE.to_string())}
                    />
            </div>
        }
    }
}
