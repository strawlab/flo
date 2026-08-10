//! Records the revision this crate was built from. See `crates/flo/build.rs`
//! for why each component has to do this for itself.

fn main() {
    watch_git(std::path::Path::new("../../.git"));

    let git_hash = run_git(&["rev-parse", "HEAD"])
        .filter(|hash| !hash.is_empty())
        .unwrap_or_else(|| "unknown".to_owned());
    println!("cargo:rustc-env=GIT_HASH={git_hash}");

    // Empty output from `--porcelain` means a clean tree. Untracked files are
    // excluded, as `git describe --dirty` excludes them: build output sitting
    // in the working directory is not a modification of what was built. An error is reported
    // as "unknown" rather than clean: claiming a build came from committed
    // sources when we could not check is the one wrong answer.
    let dirty = match run_git(&["status", "--porcelain", "--untracked-files=no"]) {
        Some(out) => {
            if out.is_empty() {
                "false"
            } else {
                "true"
            }
        }
        None => "unknown",
    };
    println!("cargo:rustc-env=GIT_DIRTY={dirty}");
}

fn run_git(args: &[&str]) -> Option<String> {
    let output = std::process::Command::new("git").args(args).output().ok()?;
    if !output.status.success() {
        return None;
    }
    String::from_utf8(output.stdout)
        .ok()
        .map(|text| text.trim().to_owned())
}

/// Ask cargo to rerun this script when the recorded revision could change.
///
/// `.git/HEAD` alone is not enough, and the gap is easy to miss: on a commit
/// `HEAD` still reads `ref: refs/heads/<branch>` and does not change at all --
/// the branch's ref file does. Watching only `HEAD` therefore bakes in a stale
/// revision until something else forces a rebuild, which for a provenance
/// record is worse than having none.
///
/// `src` is watched too so that editing the source recomputes the dirty flag;
/// naming any path at all opts out of cargo's default "rerun when a package
/// file changes".
fn watch_git(git_dir: &std::path::Path) {
    let head = git_dir.join("HEAD");
    println!("cargo:rerun-if-changed={}", head.display());
    println!("cargo:rerun-if-changed=src");
    // Follow `ref: refs/heads/<branch>` to the file that actually moves.
    if let Ok(contents) = std::fs::read_to_string(&head)
        && let Some(reference) = contents.strip_prefix("ref:")
    {
        println!(
            "cargo:rerun-if-changed={}",
            git_dir.join(reference.trim()).display()
        );
    }
}
