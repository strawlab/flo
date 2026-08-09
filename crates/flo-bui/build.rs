/// Set the environment variable `GIT_HASH` to the current git revision so the
/// BUI footer can report which build it is serving.
fn git_hash() -> String {
    let output = match std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
    {
        Ok(output) if output.status.success() => output,
        // Building from a source archive rather than a checkout, or without
        // git installed. A footer saying "unknown" beats failing the build.
        _ => return "unknown".to_string(),
    };
    let git_hash = String::from_utf8_lossy(&output.stdout).trim().to_string();
    if git_hash.is_empty() {
        "unknown".to_string()
    } else {
        git_hash
    }
}

fn main() {
    println!("cargo:rustc-env=GIT_HASH={}", git_hash());
}
