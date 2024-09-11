#[cfg(target_os = "macos")]
extern crate cc;

#[cfg(target_os = "macos")]
fn main() {
    // User can set env var MACOSX_DEPLOYMENT_TARGET="12.0" to silence compiler
    // warnings. Could also call `.cargo_warnings(false)` on cc:Build to silence
    // console. I didn't do that here because I don't want to deviate from
    // plain-vanilla. (Furthermore, this is all macOS only.)
    println!("cargo:rerun-if-changed=src/uuid-macos.c");
    println!("cargo:rustc-link-lib=framework=CoreFoundation");
    println!("cargo:rustc-link-lib=framework=IOKit");
    cc::Build::new()
        .file("src/uuid-macos.c")
        .compile("uuid-macos");
}

#[cfg(not(target_os = "macos"))]
fn main() {}
