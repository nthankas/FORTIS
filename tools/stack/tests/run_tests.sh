#!/usr/bin/env bash
# Plain-bash test runner for the ./stack CLI.
#
# Run from any working directory:
#     ./tools/stack/tests/run_tests.sh
#
# The runner picks up the stack script from the repo root (computed
# relative to this file) and exercises argument parsing, help output, and
# the dispatcher. Tests that would otherwise touch the real git or docker
# CLIs export shell-function shims that capture calls and short-circuit
# the action.
#
# This is the fallback runner. If bats-core (https://github.com/bats-core)
# is installed locally, run `bats tools/stack/tests/` instead - see
# tools/stack/tests/README.md.

set -u  # NOT -e: tests use explicit pass/fail tracking

# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------
TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${TESTS_DIR}/../../.." && pwd)"
STACK="${REPO_ROOT}/stack"

# Disable color in subprocess output for stable golden comparisons.
export NO_COLOR=1
# Tell stack not to prompt; tests must never block.
export STACK_ASSUME_YES=1

# -----------------------------------------------------------------------------
# Tracking
# -----------------------------------------------------------------------------
PASS=0
FAIL=0
FAILED_TESTS=()

# Use a per-run scratch dir for any temp files (mock PATH, fake repos, etc).
SCRATCH="$(mktemp -d -t stack-tests.XXXXXX)"
trap 'rm -rf "${SCRATCH}"' EXIT

pass() {
    PASS=$((PASS + 1))
    printf '  ok   %s\n' "$1"
}

fail() {
    FAIL=$((FAIL + 1))
    FAILED_TESTS+=("$1")
    printf '  FAIL %s\n' "$1"
    if [[ -n "${2:-}" ]]; then
        printf '       %s\n' "$2"
    fi
}

# Assert exit code from a stack invocation.
# Usage: expect_exit <expected_code> <test_name> -- <stack args...>
expect_exit() {
    local expected="$1"
    local name="$2"
    shift 2
    if [[ "${1:-}" == "--" ]]; then shift; fi
    local out actual=0
    out="$("${STACK}" "$@" 2>&1)" || actual=$?
    if [[ "${actual}" -eq "${expected}" ]]; then
        pass "${name} (exit=${actual})"
    else
        fail "${name}" "expected exit ${expected}, got ${actual}. output:
${out}"
    fi
}

# Assert stdout/stderr from a stack invocation contains a substring.
# Usage: expect_contains <substring> <test_name> -- <stack args...>
expect_contains() {
    local needle="$1"
    local name="$2"
    shift 2
    if [[ "${1:-}" == "--" ]]; then shift; fi
    local out
    out="$("${STACK}" "$@" 2>&1)" || true
    if [[ "${out}" == *"${needle}"* ]]; then
        pass "${name}"
    else
        fail "${name}" "did not find '${needle}' in output:
${out}"
    fi
}

# -----------------------------------------------------------------------------
# Helpers for tests that need an isolated working tree.
# -----------------------------------------------------------------------------
make_sandbox_repo() {
    # Create a self-contained directory containing a copy of the stack script
    # and a usable .env, plus a fake `git` and `docker` on PATH. Tests that
    # need to run dispatch logic without touching the real repo invoke this.
    local dir
    dir="$(mktemp -d -p "${SCRATCH}" sandbox.XXXX)"
    cp "${STACK}" "${dir}/stack"
    chmod +x "${dir}/stack"
    cp "${REPO_ROOT}/.env.example" "${dir}/.env"
    # Provide empty docker-compose stubs so dc() never fails on missing files.
    mkdir -p "${dir}/docker"
    printf 'services:\n  dev:\n    image: busybox\n' > "${dir}/docker/docker-compose.yml"
    printf 'services:\n  fortis-dev-gpu:\n    image: busybox\n' > "${dir}/docker/docker-compose.gpu.yml"
    printf '%s' "${dir}"
}

make_mock_bin() {
    # Build a directory of mock executables that record their argv to a log.
    # Returns the directory path (for prepending to PATH) and the log path on
    # stdout, newline separated.
    local dir log
    dir="$(mktemp -d -p "${SCRATCH}" mockbin.XXXX)"
    log="$(mktemp -p "${SCRATCH}" mocklog.XXXX)"
    for tool in git docker ssh; do
        cat > "${dir}/${tool}" <<EOF
#!/usr/bin/env bash
printf '%s %s\\n' "${tool}" "\$*" >> "${log}"
# Behave well enough that stack's helpers don't blow up.
case "${tool}" in
    git)
        case "\$1" in
            -C) shift 2 ;;
        esac
        case "\$1" in
            status) printf '' ;;
            symbolic-ref) printf 'main\\n' ;;
            rev-parse) printf 'abcdef0\\n' ;;
            fetch|checkout|pull) ;;
        esac
        ;;
    docker)
        case "\$1" in
            compose|exec) ;;
        esac
        ;;
esac
exit 0
EOF
        chmod +x "${dir}/${tool}"
    done
    printf '%s\n%s\n' "${dir}" "${log}"
}

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
test_no_args_prints_help() {
    expect_exit 0 "no args prints help" --
    expect_contains "stack - FORTIS deployment helper" "no args output is help" --
}

test_help_command() {
    expect_exit 0 "help command exits 0" -- help
    expect_contains "COMMANDS" "help command lists commands" -- help
}

test_help_subcommand() {
    expect_exit 0 "help up exits 0" -- help up
    expect_contains "Bring the FORTIS stack up" "help up shows up help" -- help up
}

test_invalid_command() {
    expect_exit 2 "invalid command exits 2" -- this-is-not-real
    expect_contains "Unknown command" "invalid command names error" -- this-is-not-real
    expect_contains "./stack help" "invalid command points at help" -- this-is-not-real
}

test_status_without_env() {
    # Run from a fresh sandbox with NO .env file.
    local dir
    dir="$(mktemp -d -p "${SCRATCH}" no-env.XXXX)"
    cp "${STACK}" "${dir}/stack"
    chmod +x "${dir}/stack"
    mkdir -p "${dir}/docker"
    # Empty compose files so dc ps doesn't error out fataly.
    printf 'services: {}\n' > "${dir}/docker/docker-compose.yml"
    printf 'services: {}\n' > "${dir}/docker/docker-compose.gpu.yml"

    local out rc=0
    out="$(cd "${dir}" && ./stack status 2>&1)" || rc=$?
    if [[ "${rc}" -eq 0 ]] && [[ "${out}" == *"MISSING"* ]]; then
        pass "status with no .env exits 0 and warns MISSING"
    else
        fail "status with no .env" "rc=${rc} out:
${out}"
    fi
}

test_status_with_env() {
    local dir
    dir="$(make_sandbox_repo)"
    local out rc=0
    out="$(cd "${dir}" && ./stack status 2>&1)" || rc=$?
    if [[ "${rc}" -eq 0 ]] && [[ "${out}" == *"loaded"* ]]; then
        pass "status with .env exits 0 and reports loaded"
    else
        fail "status with .env" "rc=${rc} out:
${out}"
    fi
}

test_switch_writes_env() {
    # Build a sandbox with mocked git/docker so the switch -> pull -> restart
    # chain doesn't try to touch the real world. Assert .env is rewritten.
    local dir
    dir="$(make_sandbox_repo)"
    local mockinfo mockbin
    mockinfo="$(make_mock_bin)"
    mockbin="$(printf '%s\n' "${mockinfo}" | head -n 1)"

    local out rc=0
    out="$(cd "${dir}" && PATH="${mockbin}:${PATH}" ./stack switch feat/test-branch 2>&1)" || rc=$?
    if [[ "${rc}" -eq 0 ]] && grep -q '^FORTIS_REF=feat/test-branch$' "${dir}/.env"; then
        pass "switch writes new FORTIS_REF to .env"
    else
        fail "switch writes new FORTIS_REF" "rc=${rc}, .env contents:
$(cat "${dir}/.env" 2>/dev/null)
captured stack output:
${out}"
    fi
}

test_switch_requires_ref() {
    expect_exit 1 "switch with no ref errors" -- switch
}

test_exec_default_container_cpu() {
    # cpu profile -> container 'fortis-dev'. We can't actually exec, but we
    # can override docker as a shell function so the call is captured.
    local dir
    dir="$(make_sandbox_repo)"
    # Force cpu profile in .env.
    sed -i.bak 's|^COMPOSE_PROFILE=.*|COMPOSE_PROFILE=cpu|' "${dir}/.env"
    rm -f "${dir}/.env.bak"

    # Replace `docker` with a recorder.
    local mockbin log
    mockbin="$(mktemp -d -p "${SCRATCH}" execbin.XXXX)"
    log="${mockbin}/calls.log"
    cat > "${mockbin}/docker" <<EOF
#!/usr/bin/env bash
printf '%s\\n' "\$*" >> "${log}"
exit 0
EOF
    chmod +x "${mockbin}/docker"

    local out rc=0
    out="$(cd "${dir}" && PATH="${mockbin}:${PATH}" ./stack exec 2>&1)" || rc=$?
    if [[ "${rc}" -eq 0 ]] && grep -q 'exec -it fortis-dev bash' "${log}"; then
        pass "exec defaults to fortis-dev for cpu profile"
    else
        fail "exec default cpu container" "rc=${rc} log:
$(cat "${log}" 2>/dev/null)
out: ${out}"
    fi
}

test_exec_default_container_gpu() {
    local dir
    dir="$(make_sandbox_repo)"
    sed -i.bak 's|^COMPOSE_PROFILE=.*|COMPOSE_PROFILE=gpu|' "${dir}/.env"
    rm -f "${dir}/.env.bak"

    local mockbin log
    mockbin="$(mktemp -d -p "${SCRATCH}" execbin.XXXX)"
    log="${mockbin}/calls.log"
    cat > "${mockbin}/docker" <<EOF
#!/usr/bin/env bash
printf '%s\\n' "\$*" >> "${log}"
exit 0
EOF
    chmod +x "${mockbin}/docker"

    local out rc=0
    out="$(cd "${dir}" && PATH="${mockbin}:${PATH}" ./stack exec 2>&1)" || rc=$?
    if [[ "${rc}" -eq 0 ]] && grep -q 'exec -it fortis-dev-gpu bash' "${log}"; then
        pass "exec defaults to fortis-dev-gpu for gpu profile"
    else
        fail "exec default gpu container" "rc=${rc} log:
$(cat "${log}" 2>/dev/null)
out: ${out}"
    fi
}

# -----------------------------------------------------------------------------
# Run all tests.
# -----------------------------------------------------------------------------
printf 'Running stack CLI tests (runner: plain bash)\n'
printf '==============================================\n'

test_no_args_prints_help
test_help_command
test_help_subcommand
test_invalid_command
test_status_without_env
test_status_with_env
test_switch_writes_env
test_switch_requires_ref
test_exec_default_container_cpu
test_exec_default_container_gpu

printf '\n----------------------------------------------\n'
printf 'Results: %d passed, %d failed\n' "${PASS}" "${FAIL}"
if [[ "${FAIL}" -gt 0 ]]; then
    printf 'Failed tests:\n'
    for t in "${FAILED_TESTS[@]}"; do
        printf '  - %s\n' "${t}"
    done
    exit 1
fi
exit 0
