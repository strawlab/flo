use std::{
    collections::VecDeque,
    path::{Path, PathBuf},
    time::Duration,
};

use eyre::{Context, Result};
use mavlink::MavHeader;

use flo_core::SaveToDiskMsg;

const RATE_WINDOW: Duration = Duration::from_secs(5);
const BYTES_PER_KILOBYTE: f64 = 1_000.0;
const CREDENTIALS_DIRECTORY_ENV: &str = "CREDENTIALS_DIRECTORY";
const NTRIP_USERNAME_CREDENTIAL: &str = "ntrip_username";
const NTRIP_PASSWORD_CREDENTIAL: &str = "ntrip_password";
const NTRIP_PASSWORD_ENV: &str = "NTRIP_PASSWORD";
const NTRIP_USERNAME_ENV: &str = "NTRIP_USERNAME";

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

/// Read an NTRIP credential systemd made available to this service.
///
/// A missing credential is not an error because FLO may not be running under
/// systemd, or may still use the legacy environment variable. Other failures
/// are reported rather than silently falling back from a credential the
/// service manager attempted to provide.
fn read_systemd_ntrip_credential(
    credentials_directory: Option<&Path>,
    credential_name: &str,
) -> Result<Option<String>> {
    let Some(credentials_directory) = credentials_directory else {
        return Ok(None);
    };
    let path = credentials_directory.join(credential_name);
    match std::fs::read_to_string(&path) {
        Ok(value) => Ok(Some(value.trim_end_matches(['\r', '\n']).to_owned())),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e).with_context(|| format!("reading systemd credential {}", path.display())),
    }
}

/// Take an NTRIP URL and override its credentials from the supplied sources.
/// Each systemd credential takes precedence over its legacy environment value,
/// which in turn takes precedence over the corresponding URL field.
fn fix_url_with_credentials(
    ntrip_url: &str,
    env_username: Option<String>,
    env_password: Option<String>,
    credential_username: Option<String>,
    credential_password: Option<String>,
) -> Result<String> {
    let uri: http::Uri = ntrip_url
        .parse()
        .with_context(|| format!("While parsing NTRIP URL \"{ntrip_url}\"."))?;

    let parts = uri.into_parts();
    let (host_port, mut user_pass) = if let Some(auth) = &parts.authority {
        parse_authority(auth)?
    } else {
        eyre::bail!("No authority section of URL");
    };

    let has_systemd_credentials = credential_username.is_some() || credential_password.is_some();
    let has_environment_credentials = env_username.is_some() || env_password.is_some();
    let (url_username, url_password) = user_pass.unzip();
    let username = credential_username.or(env_username).or(url_username);
    let password = credential_password.or(env_password).or(url_password);
    user_pass = match (username, password) {
        (Some(username), Some(password)) => Some((username, password)),
        (None, None) => None,
        (Some(_), None) => eyre::bail!("NTRIP username was supplied without a password"),
        (None, Some(_)) => eyre::bail!("NTRIP password was supplied without a username"),
    };

    if has_systemd_credentials {
        tracing::info!("Using NTRIP credentials supplied by systemd.");
    } else if has_environment_credentials {
        tracing::info!("Using legacy NTRIP credentials from environment variables.");
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

/// Take an NTRIP URL and add credentials supplied by systemd or, for backwards
/// compatibility, environment variables.
fn fix_url(ntrip_url: &str) -> Result<String> {
    let credentials_directory = std::env::var_os(CREDENTIALS_DIRECTORY_ENV).map(PathBuf::from);
    let credential_username =
        read_systemd_ntrip_credential(credentials_directory.as_deref(), NTRIP_USERNAME_CREDENTIAL)?;
    let credential_password =
        read_systemd_ntrip_credential(credentials_directory.as_deref(), NTRIP_PASSWORD_CREDENTIAL)?;
    fix_url_with_credentials(
        ntrip_url,
        std::env::var(NTRIP_USERNAME_ENV).ok(),
        std::env::var(NTRIP_PASSWORD_ENV).ok(),
        credential_username,
        credential_password,
    )
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
    use std::sync::atomic::{AtomicU64, Ordering};

    use super::*;

    struct TestDirectory(PathBuf);

    impl TestDirectory {
        fn new() -> Self {
            static NEXT_ID: AtomicU64 = AtomicU64::new(0);
            loop {
                let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
                let path = std::env::temp_dir()
                    .join(format!("flo-ntrip-test-{}-{id}", std::process::id()));
                match std::fs::create_dir(&path) {
                    Ok(()) => return Self(path),
                    Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
                    Err(e) => panic!("creating test directory {}: {e}", path.display()),
                }
            }
        }

        fn path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for TestDirectory {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn systemd_credentials_override_legacy_environment_credentials() {
        let fixed = fix_url_with_credentials(
            "http://url-user:url-password@caster.example:2101/mount",
            Some("environment-user".to_owned()),
            Some("environment-password".to_owned()),
            Some("credential-user".to_owned()),
            Some("credential-password".to_owned()),
        )
        .unwrap();

        assert_eq!(
            fixed,
            "http://credential-user:credential-password@caster.example:2101/mount"
        );
    }

    #[test]
    fn systemd_password_can_use_the_username_from_the_url() {
        let fixed = fix_url_with_credentials(
            "http://url-user:obsolete-password@caster.example:2101/mount",
            None,
            None,
            None,
            Some("credential-password".to_owned()),
        )
        .unwrap();

        assert_eq!(
            fixed,
            "http://url-user:credential-password@caster.example:2101/mount"
        );
    }

    #[test]
    fn legacy_environment_credentials_still_work() {
        let fixed = fix_url_with_credentials(
            "http://caster.example:2101/mount",
            Some("legacy-user".to_owned()),
            Some("legacy-password".to_owned()),
            None,
            None,
        )
        .unwrap();

        assert_eq!(
            fixed,
            "http://legacy-user:legacy-password@caster.example:2101/mount"
        );
    }

    #[test]
    fn credential_file_trailing_newlines_are_not_part_of_the_values() {
        let directory = TestDirectory::new();
        std::fs::write(
            directory.path().join(NTRIP_USERNAME_CREDENTIAL),
            "credential-user\n",
        )
        .unwrap();
        std::fs::write(
            directory.path().join(NTRIP_PASSWORD_CREDENTIAL),
            "credential-password\r\n",
        )
        .unwrap();

        assert_eq!(
            read_systemd_ntrip_credential(Some(directory.path()), NTRIP_USERNAME_CREDENTIAL)
                .unwrap(),
            Some("credential-user".to_owned())
        );
        assert_eq!(
            read_systemd_ntrip_credential(Some(directory.path()), NTRIP_PASSWORD_CREDENTIAL)
                .unwrap(),
            Some("credential-password".to_owned())
        );
    }

    #[test]
    fn absent_systemd_credential_allows_the_legacy_fallback() {
        let directory = TestDirectory::new();
        assert_eq!(
            read_systemd_ntrip_credential(Some(directory.path()), NTRIP_USERNAME_CREDENTIAL)
                .unwrap(),
            None
        );
    }

    #[test]
    fn unreadable_systemd_credential_does_not_silently_fall_back() {
        let directory = TestDirectory::new();
        std::fs::create_dir(directory.path().join(NTRIP_PASSWORD_CREDENTIAL)).unwrap();

        assert!(
            read_systemd_ntrip_credential(Some(directory.path()), NTRIP_PASSWORD_CREDENTIAL)
                .is_err()
        );
    }

    #[test]
    fn systemd_password_without_any_username_is_rejected() {
        let result = fix_url_with_credentials(
            "http://caster.example:2101/mount",
            None,
            None,
            None,
            Some("credential-password".to_owned()),
        );

        assert!(result.is_err());
    }

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
