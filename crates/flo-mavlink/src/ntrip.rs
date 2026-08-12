use std::{collections::VecDeque, time::Duration};

use eyre::{Context, Result};
use mavlink::MavHeader;

use flo_core::SaveToDiskMsg;

const RATE_WINDOW: Duration = Duration::from_secs(5);
const BYTES_PER_KILOBYTE: f64 = 1_000.0;

#[derive(Default)]
pub(crate) struct RollingByteRate {
    samples: VecDeque<(tokio::time::Instant, usize)>,
    total_bytes: usize,
}

impl RollingByteRate {
    pub(crate) fn record(&mut self, now: tokio::time::Instant, bytes: usize) {
        self.samples.push_back((now, bytes));
        self.total_bytes += bytes;
        self.expire(now);
    }

    pub(crate) fn kbps(&mut self, now: tokio::time::Instant) -> f64 {
        self.expire(now);
        self.total_bytes as f64 / RATE_WINDOW.as_secs_f64() / BYTES_PER_KILOBYTE
    }

    fn expire(&mut self, now: tokio::time::Instant) {
        while self.samples.front().is_some_and(|(received_at, _)| {
            now.saturating_duration_since(*received_at) >= RATE_WINDOW
        }) {
            let (_, bytes) = self.samples.pop_front().unwrap();
            self.total_bytes -= bytes;
        }
    }
}

/// A zero-sized type which is never created to indicate that Ok(_) never
/// happens.
#[derive(Debug)]
pub(crate) enum NeverOk {}

fn parse_authority(auth: &http::uri::Authority) -> Result<(String, Option<(String, String)>)> {
    // Replace when https://github.com/hyperium/http/pull/399 is merged.
    let auth_vec = auth.as_str().split("@").collect::<Vec<_>>();
    match auth_vec.len() {
        1 => {
            // Only "host:port"
            let host_port = auth_vec[0].to_string();
            Ok((host_port, None))
        }
        2 => {
            // "username:password@host:port"
            let user_pass = auth_vec[0];
            let host_port = auth_vec[1].to_string();
            let up = user_pass.split(":").collect::<Vec<_>>();
            if up.len() != 2 {
                eyre::bail!("Could not parse username and password from URL");
            }
            let username = up[0].to_string();
            let password = up[1].to_string();
            Ok((host_port, Some((username, password))))
        }
        _ => {
            eyre::bail!("Expected zero or one '@' symbols in authority");
        }
    }
}

/// Take NTRIP URL and return a URL with credentials from environment variables
/// if available.
fn fix_url(ntrip_url: &str) -> Result<String> {
    let uri: http::Uri = ntrip_url
        .parse()
        .with_context(|| format!("While parsing NTRIP URL \"{ntrip_url}\"."))?;

    let parts = uri.into_parts();
    let (host_port, mut user_pass) = if let Some(auth) = &parts.authority {
        parse_authority(auth)?
    } else {
        eyre::bail!("No authority section of URL");
    };

    let env_username = std::env::var("NTRIP_USERNAME");
    let env_password = std::env::var("NTRIP_PASSWORD");

    if let (Ok(user), Ok(pass)) = (env_username, env_password) {
        // Use environment variables if available.
        tracing::info!("Using NTRIP credentials from environment variables.");
        user_pass = Some((user, pass));
    };

    let auth = if let Some(user_pass) = user_pass {
        let auth_str = format!("{}:{}@{}", user_pass.0, user_pass.1, host_port);
        http::uri::Authority::from_maybe_shared(auth_str)?
    } else {
        http::uri::Authority::from_maybe_shared(host_port)?
    };

    let uri = http::uri::Builder::new()
        .scheme(parts.scheme.unwrap())
        .authority(auth)
        .path_and_query(parts.path_and_query.unwrap())
        .build()?;

    Ok(format!("{uri}"))
}

/// Infinite loop which connects to an NTRIP server and sends RTCM data
/// to the autopilot.
///
/// A robust connection to the NTRIP server is made. If the connection is lost,
/// it will be re-established automatically. The function will run indefinitely.
/// Stream RTCM3 corrections from an NTRIP caster to the flight controller,
/// recording them on the way past.
///
/// The recording is the base station half of a post-processed kinematic (PPK)
/// solution: the flight controller's own log can hold what the rover measured,
/// but the corrections it was measured against exist only in this stream. See
/// [`flo_core::NTRIP_RTCM_FNAME`].
pub(crate) async fn ntrip_loop(
    ntrip_url: String,
    mavconn_tx: tokio::sync::mpsc::Sender<(MavHeader, mavlink::ardupilotmega::MavMessage)>,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    rate_tx: tokio::sync::mpsc::UnboundedSender<usize>,
    header: MavHeader,
) -> Result<NeverOk> {
    let fixed_url =
        fix_url(&ntrip_url).with_context(|| format!("While fixing NTRIP URL \"{ntrip_url}\"."))?;
    let lowlevel_client =
        robust_ntrip_client::RobustNtripClient::new(&fixed_url, Default::default()).await?;
    let mut ntrip = robust_ntrip_client::ParsingNtripClient::new(lowlevel_client);
    let mut sequence_number: u8 = 0;
    loop {
        let rtcm = ntrip.next().await?;
        let bytes: Vec<u8> = rtcm.into();
        if bytes.is_empty() {
            tracing::error!("Received empty RTCM data from NTRIP server, skipping.");
            continue;
        }
        rate_tx.send(bytes.len())?;

        // Record before the size check below, not after: a frame too large for
        // MAVLink never reaches the flight controller, but it is still a valid
        // base station observation and post-processing wants it.
        floz_logger.send(SaveToDiskMsg::NtripRtcm(bytes.clone()))?;

        let n_frags = bytes.len().div_ceil(180);
        if n_frags > 4 {
            tracing::error!(
                "RTCM data is too long: {} bytes. See https://github.com/mavlink/mavlink/issues/2109.",
                bytes.len()
            );
            continue;
        }
        for (frag_idx, frag_bytes) in bytes[..].chunks(180).enumerate() {
            let mut data = [0u8; 180];
            data[..frag_bytes.len()].copy_from_slice(frag_bytes);
            let len = frag_bytes.len().try_into().unwrap();

            // flags LSB
            let mut flags = if n_frags == 1 { 0 } else { 1 };
            // fragment ID
            flags |= (frag_idx as u8 & 0x03) << 1;
            // sequence number
            flags |= (sequence_number & 0x1F) << 3;

            let data = mavlink::ardupilotmega::MavMessage::GPS_RTCM_DATA(
                mavlink::ardupilotmega::GPS_RTCM_DATA_DATA { flags, len, data },
            );
            mavconn_tx.send((header, data)).await?;
        }

        sequence_number = sequence_number.wrapping_add(1);
    }
}

#[cfg(test)]
mod tests {
    use super::{RATE_WINDOW, RollingByteRate};

    #[test]
    fn byte_rate_uses_only_the_preceding_five_seconds() {
        let start = tokio::time::Instant::now();
        let mut rate = RollingByteRate::default();

        rate.record(start, 2_000);
        assert!((rate.kbps(start) - 0.4).abs() < f64::EPSILON);

        rate.record(start + RATE_WINDOW / 2, 3_000);
        assert!((rate.kbps(start + RATE_WINDOW / 2) - 1.0).abs() < f64::EPSILON);

        assert!((rate.kbps(start + RATE_WINDOW) - 0.6).abs() < f64::EPSILON);
        assert_eq!(rate.kbps(start + RATE_WINDOW + RATE_WINDOW / 2), 0.0);
    }
}
