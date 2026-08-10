//! Records the revision this crate was built from, for `flo_core::ComponentVersion`.
//!
//! The same few lines appear in every FLO binary's build script, and in the
//! build script of anything that composes FLO from another repository: Cargo's
//! `rustc-env` reaches only the crate it belongs to, so each component has to
//! ask git for itself. There is no shared helper because the interesting case
//! -- a composing binary in a repository that does not depend on this workspace
//! -- could not use one.

fn main() {
    watch_git(std::path::Path::new("../../.git"));

    let git_hash = run_git(&["rev-parse", "HEAD"])
        .filter(|hash| !hash.is_empty())
        // An empty or failed result must not reach the version string. It used
        // to: this crate built `CARGO_PKG_VERSION` as `{version}+{hash}`, so a
        // missing hash produced `0.1.0+`, whose empty build-metadata segment is
        // not a valid semver -- the failure seen in strand-braid #27. The
        // trailing newline from `rev-parse` went in untrimmed, too.
        .unwrap_or_else(|| "unknown".to_owned());
    println!("cargo:rustc-env=GIT_HASH={git_hash}");

    // `--porcelain` prints one line per changed path and nothing at all for a
    // clean tree, so emptiness is the answer. Untracked files are excluded, as
    // `git describe --dirty` excludes them: a `target/` directory or an editor
    // scratch file is not a modification of the source that was built. An error is reported as "unknown"
    // rather than as clean, since claiming a build is reproducible when we
    // could not check is the one wrong answer here.
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

    // Keep the package version pointing at the source, for anything reading it
    // directly rather than through `ComponentVersion`.
    let version = format!("{}+{git_hash}", env!("CARGO_PKG_VERSION"));
    println!("cargo:rustc-env=CARGO_PKG_VERSION={version}");
}

/// Run a git command, returning its trimmed stdout, or `None` if git is absent,
/// fails, or says nothing useful.
fn run_git(args: &[&str]) -> Option<String> {
    let output = std::process::Command::new("git").args(args).output().ok()?;
    if !output.status.success() {
        return None;
    }
    let text = String::from_utf8(output.stdout).ok()?;
    let text = text.trim().to_owned();
    // An empty rev-parse is meaningless; an empty `status --porcelain` means
    // clean, so callers that care distinguish the two themselves.
    Some(text)
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
