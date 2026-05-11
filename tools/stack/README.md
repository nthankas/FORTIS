# `stack` - FORTIS deployment helper

A small bash CLI that wraps `git` + `docker compose` so the common FORTIS
deployment workflows (bring up the dev container, flip the Jetson to a
branch, exec into the container) are one-line operations.

The script itself lives at the **repository root** as `./stack`. This
directory holds tests and documentation for it; the script is at the root
so the operator workflow is `./stack <cmd>` instead of nested path typing.

## Quickstart

```bash
cp .env.example .env       # copy template, then edit for this machine
./stack up                 # bring the stack up
./stack exec               # open a shell inside the dev container
./stack status             # show what's running
./stack down               # tear it down
```

`.env` is gitignored. Every machine that runs the stack needs its own
`.env` file with the right `COMPOSE_PROFILE` (cpu / gpu) and `FORTIS_REF`.

## How it picks the compose profile

The CLI reads `COMPOSE_PROFILE` from `.env`:

| Profile | Compose files                                            | Service name      | Container name    |
|---------|----------------------------------------------------------|-------------------|-------------------|
| `cpu`   | `docker/docker-compose.yml`                              | `dev`             | `fortis-dev`      |
| `gpu`   | `docker/docker-compose.yml` + `docker/docker-compose.gpu.yml` | `fortis-dev-gpu` | `fortis-dev-gpu` |

`cpu` is the default. Pick `gpu` on machines with an NVIDIA GPU and
NVIDIA Container Toolkit installed (FORTIS PC, IdeaPad, Jetson Orin Nano).

## Command reference

Every subcommand also accepts `-h` / `--help`.

### `stack up`

Bring the stack up.

```bash
./stack up
```

What it does:
1. Loads `.env`.
2. Warns if HEAD does not match `FORTIS_REF` (no auto-checkout).
3. Warns if the working tree is dirty.
4. Runs `docker compose -f <files> up -d` for the active profile.

If containers are already up this is effectively a no-op; docker compose
starts any missing ones and leaves the rest alone.

### `stack down`

Stop and remove containers (`docker compose down`). Volumes are
preserved. Prompts for confirmation unless `--yes`.

```bash
./stack down
./stack down --yes
```

### `stack pull`

Update the workspace to `FORTIS_REF` from `.env`:

```bash
./stack pull             # refuses on dirty tree
./stack pull --force     # try anyway (git itself still refuses to clobber)
```

Equivalent to `git fetch --all --prune && git checkout <ref> && git pull
--ff-only` (the `pull --ff-only` only runs for branch refs, not tags).

### `stack restart [service]`

Restart one or every service in the active profile.

```bash
./stack restart                # all services
./stack restart fortis-dev     # only the cpu dev container
```

### `stack rebuild [service] [--no-cache]`

Rebuild images. `--no-cache` drops the docker layer cache; that path
prompts for confirmation unless `--yes`.

```bash
./stack rebuild
./stack rebuild fortis-dev
./stack rebuild --no-cache --yes
```

### `stack logs [service] [--follow]`

`docker compose logs` for one or every service, with optional follow.

```bash
./stack logs
./stack logs --follow
./stack logs fortis-dev -f
```

### `stack exec [service]`

Open an interactive `bash` in the running container. With no argument the
default is profile-aware:

| Profile | Default container |
|---------|--------------------|
| `cpu`   | `fortis-dev`       |
| `gpu`   | `fortis-dev-gpu`   |

```bash
./stack exec                  # cpu -> fortis-dev, gpu -> fortis-dev-gpu
./stack exec fortis-dev-gpu   # explicit
```

Wraps `docker exec -it <container> bash`. Requires the container to
already be running; run `./stack up` first if not.

### `stack status`

Compact summary of the local stack state:

- `.env` loaded vs missing.
- `FORTIS_REF` from `.env` vs actual `git HEAD` (warns on mismatch).
- Working tree clean / dirty.
- Active `COMPOSE_PROFILE`.
- `ROS_DOMAIN_ID` in use.
- `docker compose ps` for the active project (services, image tags, status).

Safe to run with no `.env`; warns and falls back to defaults.

```bash
./stack status
```

### `stack switch <ref>`

Set `FORTIS_REF=<ref>` in `.env`, run `stack pull`, then `stack restart`.

```bash
./stack switch main
./stack switch feat/some-branch
./stack switch v0.1.0
```

This is the primary workflow on the Jetson for flipping between demo
branches.

### `stack ssh`

SSH from the current machine to the Jetson, using `FORTIS_JETSON_HOST`
from `.env`:

```bash
./stack ssh                      # interactive shell
./stack ssh "./stack status"     # one-shot remote command
```

Detects when the local hostname already matches `FORTIS_JETSON_HOST` and
no-ops with a friendly message in that case, so the same `.env` works on
both the desk and the Jetson.

### `stack help [command]`

```bash
./stack help              # full help
./stack help up           # help for one command
./stack                   # same as ./stack help
```

## Typical workflows

### Dev desk -> Jetson branch flip

From your dev desktop (with `FORTIS_JETSON_HOST` set in your local `.env`):

```bash
./stack ssh "./stack switch feat/some-branch && ./stack status"
```

That logs into the Jetson, rewrites its `.env`, pulls the branch,
restarts containers, and prints the new status. One line.

### Bring up + work + tear down

```bash
./stack up
./stack exec
# ... work in /workspace inside the container ...
exit
./stack down
```

### Pre-demo: confirm everything matches

```bash
./stack status            # confirm FORTIS_REF matches HEAD, tree clean
./stack logs --follow     # tail logs while teammate exercises the robot
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `[stack] No .env file at ...` | First run on this machine. | `cp .env.example .env` and edit. |
| `[stack] Unknown COMPOSE_PROFILE=...` | Typo in `.env`. | Set `COMPOSE_PROFILE=cpu` or `gpu`. |
| `docker: 'compose' is not a docker command` | Old docker without the compose v2 plugin. | Install Docker 24+ with the compose plugin (see `docker/README.md`). |
| `[stack] Working tree is dirty.` blocking pull | Local uncommitted changes. | Commit / stash, or pass `--force`. |
| `stack ssh` no-ops on the Jetson | Expected: `FORTIS_JETSON_HOST` resolves to the local hostname. | Use the same `.env` shape on both machines. |
| `Error response from daemon: ... nvidia` on `stack up` | GPU profile selected without NVIDIA Container Toolkit. | Switch to `cpu`, or install the toolkit; see `docker/README.md`. |

## Fallback: when `stack` itself is broken

Everything `stack` does is achievable with `git` and `docker compose`
directly. If the script is broken or unavailable on a machine, the
underlying commands are:

```bash
# cpu profile equivalents
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml down
docker compose -f docker/docker-compose.yml logs --follow
docker exec -it fortis-dev bash

# gpu profile equivalents
docker compose -f docker/docker-compose.yml -f docker/docker-compose.gpu.yml up -d
docker exec -it fortis-dev-gpu bash
```

The CLI is a convenience layer, not a hard dependency. Treat it as such.

## Tests

See [`tests/README.md`](tests/README.md). TL;DR: `./tools/stack/tests/run_tests.sh`.
