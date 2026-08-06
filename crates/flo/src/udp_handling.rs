use color_eyre::eyre::{self, Result};
use tokio_util::udp::UdpFramed;
use tracing as log;

use crate::udp_codec::FloControllerUdpCodec;

/// Open UDP socket for [FloControllerUdpCodec] messages from the network.
///
/// The opened socket receives unicast messages sent to the device, e.g. from
/// Strand Camera.
pub(crate) async fn setup_udp<A>(udp_addr: A) -> Result<UdpFramed<FloControllerUdpCodec>>
where
    A: tokio::net::ToSocketAddrs + std::fmt::Display + Clone,
{
    use eyre::WrapErr;

    let udp_socket = tokio::net::UdpSocket::bind(udp_addr.clone())
        .await
        .with_context(|| format!("When binding UDP address \"{}\"", udp_addr))?;

    let udp_src_addr = udp_socket.local_addr()?;
    if let Some(sock_addrs) = flo_core::all_addrs(udp_src_addr) {
        for sock_addr in sock_addrs.into_iter() {
            log::info!("Using unicast UDP for listening {sock_addr}");
        }
    } else {
        log::info!(
            "Using unspecified unicast UDP for listening {}",
            udp_src_addr
        );
    }

    // Wrap our socket in tokio decode/encode streams.
    let udp_framed = UdpFramed::new(udp_socket, FloControllerUdpCodec::default());

    Ok(udp_framed)
}
