# AGENTS.md — Coding agent guidance for flo

## Version control

- Every commit made by a model should have the model name in the commit summary
  and full message.
- Check if the checkout is setup to use `jj` by checking the output of `jj status`.
  If so, prefer `jj` commands over `git`.

## Style and tooling

- Follow `rustfmt` defaults; do not add `rustfmt.toml` overrides without
  discussion. Use `cargo fmt` to reformat code.
- When removing functionality, delete the code entirely. Do not comment it out.
- When using rust's string formatting, prefer to directly capture the argument
  (e.g. use `println!("{variable}");` over `println!("{}", variable);`).
- Warnings are errors (`-D warnings`). Fix all warnings before finishing.
- Use `tracing` (not `log`) for instrumentation. The workspace already depends
  on `tracing` and `tracing-subscriber`; do not add `env_logger` to new crates.
- Prefer workspace-level dependency versions declared in the root `Cargo.toml`
  `[workspace.dependencies]` table rather than pinning versions in leaf crates.
- The workspace uses `resolver = "3"`. Keep feature unification in mind when
  adding dependencies.
- Favor using best practices for maintainability (e.g. minimize redundancy, use
  `#[expect(xyz)]` instead of `#[allow(xyz)]` including a comment about why).
  Features from the most recent rust version can be used.
- New logic should have unit tests in the same file (`#[cfg(test)]` module) or
  in a `tests/` directory.
- Keep within-repo crate references separated by a newline below crate
  references from crates.io in Cargo.toml files and in lines of `use
  crate_name::function_name` at the top of rust source files.
- Any new crates should have a README.md file and if they have scripts, the
  scripts should be documented and known to run.
