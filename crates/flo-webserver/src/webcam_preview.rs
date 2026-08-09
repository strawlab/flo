//! The webcam preview: a standalone page showing what camshow's FPV camera
//! sees, and the JPEG behind it.
//!
//! Deliberately not part of the main UI. camshow owns the webcam, so every
//! preview frame costs a stamp and a downscale there, a JPEG encode here, and
//! bandwidth on whatever link the browser is at the far end of. None of that
//! is worth paying for a panel nobody is looking at, so the whole path is
//! demand-driven: this module records when a preview image was last fetched,
//! and the producer connects to camshow only while that is recent. With the
//! page closed, no frames are produced anywhere.
//!
//! This half deals only in already-encoded bytes; the producer side owns the
//! link and the encoding.

use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

/// How long after a fetch the preview is still considered wanted.
///
/// Longer than the page's poll interval by enough that one slow round trip, or
/// a browser throttling a background tab for a moment, does not tear the link
/// down and immediately rebuild it.
const DEMAND_WINDOW: Duration = Duration::from_secs(5);

/// A preview image older than this is not served: better an empty pane that
/// says so than a frozen one the operator reads as current.
const STALE_AFTER: Duration = Duration::from_secs(3);

/// Shared handle on the newest preview image, held by both the producer that
/// fills it and the HTTP handlers that serve it.
#[derive(Clone, Default)]
pub struct WebcamPreview {
    inner: Arc<Inner>,
}

#[derive(Default)]
struct Inner {
    latest: Mutex<Option<Snapshot>>,
    /// When a preview image was last asked for. `None` until the first fetch,
    /// which is why the producer starts out idle.
    last_demand: Mutex<Option<Instant>>,
}

struct Snapshot {
    jpeg: Arc<Vec<u8>>,
    received_at: Instant,
}

impl std::fmt::Debug for WebcamPreview {
    /// Deliberately says nothing about the image held: `AppState` is `Debug`,
    /// and a few hundred kilobytes of JPEG in a log line helps nobody.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("WebcamPreview")
            .field("wanted", &self.wanted())
            .field("has_frame", &self.snapshot().is_some())
            .finish()
    }
}

impl WebcamPreview {
    pub fn new() -> Self {
        Self::default()
    }

    /// Store a freshly encoded image, replacing whatever came before.
    pub fn set_jpeg(&self, jpeg: Vec<u8>) {
        *self.lock_latest() = Some(Snapshot {
            jpeg: Arc::new(jpeg),
            received_at: Instant::now(),
        });
    }

    /// The newest image, if one arrived recently enough to be worth showing.
    /// Does *not* record demand: call [`Self::note_demand`] for that, so
    /// internal reads cannot keep the producer alive on their own.
    pub fn snapshot(&self) -> Option<Arc<Vec<u8>>> {
        let latest = self.lock_latest();
        let snapshot = latest.as_ref()?;
        (snapshot.received_at.elapsed() < STALE_AFTER).then(|| Arc::clone(&snapshot.jpeg))
    }

    /// Record that someone asked for a preview image just now.
    pub fn note_demand(&self) {
        *self.lock_demand() = Some(Instant::now());
    }

    /// Whether anyone has asked recently enough that frames are worth
    /// producing. The producer polls this; nothing else should need it.
    pub fn wanted(&self) -> bool {
        self.lock_demand()
            .is_some_and(|at| at.elapsed() < DEMAND_WINDOW)
    }

    /// Drop the held image. Called when the producer's link goes away, so a
    /// reopened page does not flash the last frame of a previous session.
    pub fn clear(&self) {
        *self.lock_latest() = None;
    }

    // A poisoned lock here means a panic while holding it, which for these two
    // tiny critical sections cannot leave inconsistent state. Recovering beats
    // taking the web server down with it.
    fn lock_latest(&self) -> std::sync::MutexGuard<'_, Option<Snapshot>> {
        self.inner
            .latest
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    fn lock_demand(&self) -> std::sync::MutexGuard<'_, Option<Instant>> {
        self.inner
            .last_demand
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

/// The standalone preview page: an image tag and just enough script to keep it
/// current.
///
/// Served as a string rather than added to the single-page app because that is
/// the whole point of it — a window the operator opens, watches, and closes,
/// costing nothing while shut. It carries no styling from the main UI and no
/// wasm.
///
/// `PREVIEW_IMAGE_PATH` is substituted by [`preview_page_html`] so the page and
/// the router cannot disagree about where the image lives.
const PREVIEW_PAGE_TEMPLATE: &str = r#"<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="color-scheme" content="light dark">
<title>FLO webcam preview</title>
<style>
  body { margin: 0; background: #000; color: #adadad;
         font-family: system-ui, sans-serif; display: flex;
         flex-direction: column; height: 100vh; }
  #view { flex: 1; min-height: 0; display: flex; align-items: center;
          justify-content: center; }
  /* Deliberately sets no `display`: the image is hidden until the first frame
     arrives by the `hidden` attribute instead. A rule here would have to be
     undone from the script, and clearing an inline style only lets the rule
     apply again. */
  #frame { max-width: 100%; max-height: 100%; }
  #status { padding: 0.5em; font-size: 0.85rem; text-align: center; }
</style>
</head>
<body>
<div id="view"><img id="frame" alt="webcam preview" hidden></div>
<div id="status">Connecting&hellip;</div>
<script>
  const frame = document.getElementById('frame');
  const status = document.getElementById('status');
  // Poll rather than stream: camshow only offers frames at 5 Hz, so asking
  // more often would just burn requests. Each response is a complete image,
  // so a dropped one costs nothing but that frame.
  const INTERVAL_MS = 200;
  let inFlight = false;

  async function tick() {
    // Skip if the previous request has not come back: on a slow link, piling
    // up requests would make the lag worse, not the frame rate better.
    if (inFlight || document.hidden) { return; }
    inFlight = true;
    try {
      const resp = await fetch('PREVIEW_IMAGE_PATH?t=' + Date.now(), { cache: 'no-store' });
      if (resp.ok) {
        const blob = await resp.blob();
        const url = URL.createObjectURL(blob);
        const previous = frame.src;
        frame.src = url;
        frame.hidden = false;
        status.textContent = '';
        // Release the previous blob only once the new one is in place, so the
        // image never blanks between frames.
        if (previous.startsWith('blob:')) { URL.revokeObjectURL(previous); }
      } else {
        frame.hidden = true;
        status.textContent = 'No webcam frames. Is camshow running?';
      }
    } catch (e) {
      status.textContent = 'Lost contact with FLO.';
    } finally {
      inFlight = false;
    }
  }

  tick();
  setInterval(tick, INTERVAL_MS);
</script>
</body>
</html>
"#;

/// The preview page, with the image route filled in.
pub(crate) fn preview_page_html() -> String {
    PREVIEW_PAGE_TEMPLATE.replace(
        "PREVIEW_IMAGE_PATH",
        &format!("/{}", flo_core::WEBCAM_PREVIEW_IMAGE_PATH),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_preview_is_unwanted_until_someone_asks() {
        let preview = WebcamPreview::new();
        assert!(!preview.wanted(), "nothing should be produced by default");
        preview.note_demand();
        assert!(preview.wanted());
    }

    #[test]
    fn reading_a_snapshot_does_not_itself_create_demand() {
        let preview = WebcamPreview::new();
        preview.set_jpeg(vec![0xff, 0xd8]);
        assert!(preview.snapshot().is_some());
        assert!(
            !preview.wanted(),
            "only a fetch counts, or the producer would never idle"
        );
    }

    #[test]
    fn the_newest_image_replaces_the_previous_one() {
        let preview = WebcamPreview::new();
        preview.set_jpeg(vec![1]);
        preview.set_jpeg(vec![2]);
        assert_eq!(preview.snapshot().unwrap().as_slice(), &[2]);
    }

    #[test]
    fn clearing_leaves_nothing_to_serve() {
        let preview = WebcamPreview::new();
        preview.set_jpeg(vec![1]);
        preview.clear();
        assert!(preview.snapshot().is_none());
    }

    /// The image starts hidden and is revealed once frames arrive. Those two
    /// must be the same mechanism: hiding it with a `display: none` rule in the
    /// stylesheet and revealing it by clearing the inline `style.display`
    /// cannot work, because removing the inline declaration just lets the rule
    /// apply again. That failed silently — the page fetched and decoded every
    /// frame correctly and simply never showed one.
    #[test]
    fn the_image_is_revealed_by_whatever_hides_it() {
        let page = preview_page_html();
        assert!(
            page.contains("frame.hidden = false"),
            "the image must be revealed by the attribute that hides it"
        );
        assert!(
            !page.contains("display: none"),
            "a stylesheet rule hiding the image would outlive any inline reset"
        );
    }

    #[test]
    fn the_page_points_at_the_image_route() {
        let page = preview_page_html();
        assert!(
            page.contains(&format!("'/{}?t='", flo_core::WEBCAM_PREVIEW_IMAGE_PATH)),
            "the page must fetch the route the server actually serves"
        );
        assert!(
            !page.contains("PREVIEW_IMAGE_PATH"),
            "the placeholder must be substituted"
        );
    }
}
