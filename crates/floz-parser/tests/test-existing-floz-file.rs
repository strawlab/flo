const FNAME: &str = "flo20260519_181751.289609672.floz";
const URL_BASE: &str = "https://strawlab.org/flo-testing-assets";
const SHA256SUM: &str = "91724dc55813faebc4ec9e719e8624cf6b8ffaa463f296dc753ed25fa691b8c7";

#[test]
fn test_existing_floz_file() {
    // Installs the global tracing subscriber
    tracing_subscriber::fmt::init();

    let local_fname: String = format!("scratch/{}", FNAME);

    download_verify::download_verify(
        format!("{}/{}", URL_BASE, FNAME).as_str(),
        &local_fname,
        &download_verify::Hash::Sha256(SHA256SUM.into()),
    )
    .unwrap();
}
