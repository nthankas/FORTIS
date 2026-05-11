# stack CLI tests

Tests for the `./stack` script that lives at the repo root.

## Layout

The `stack` script intentionally lives at the repository root (not in this
directory) so the operator workflow is `./stack <cmd>`, not
`./tools/stack/stack <cmd>`. Only the tests live under `tools/stack/`.

## Running

Two runners are supported. Pick whichever is available locally; you do not
need both.

### Plain bash (default, zero dependencies)

```bash
./tools/stack/tests/run_tests.sh
```

Exits 0 on success, 1 on any failure. Reports per-test results to stdout.
This is the runner CI invokes and the runner you should use unless you
already have `bats-core` installed for another project.

### bats-core (optional)

If you have [bats-core](https://github.com/bats-core/bats-core) installed
on your `$PATH` (`command -v bats`), you can run:

```bash
bats tools/stack/tests/
```

…against any `*.bats` files that get added later. None ship today; the
plain-bash runner covers the same surface and avoids a system install.
Bats files would go alongside `run_tests.sh` in this directory.

## What is covered

The tests in `run_tests.sh` exercise:

- `./stack` with no args prints help (exit 0).
- `./stack help` and `./stack help up` print help (exit 0).
- `./stack <invalid>` errors with a hint pointing at `help` (exit 2).
- `./stack status` works with no `.env` (warns but exits 0).
- `./stack status` works with `.env` present (exits 0).
- `./stack switch <ref>` rewrites the `FORTIS_REF=` line in `.env`.
- `./stack switch` (no argument) exits non-zero.
- `./stack exec` defaults to the `fortis-dev` container on the cpu
  profile and `fortis-dev-gpu` on the gpu profile.

`git`, `docker`, and `ssh` are mocked by prepending a scratch directory of
fake binaries to `$PATH`. The mocks record their argv into a log file and
return 0 so the stack script's control flow runs end-to-end without
touching the real CLIs.

## Adding tests

Append a `test_<name>()` function in `run_tests.sh` and call it from the
"Run all tests" block at the bottom of the file. Use `expect_exit` /
`expect_contains` for the simple cases, or `make_sandbox_repo` +
`make_mock_bin` for tests that need an isolated working tree with mocked
external commands.
