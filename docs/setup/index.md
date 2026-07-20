# Setup overview & prerequisites

This section takes you from a bare set of machines to a live VAT system: a
Unitree Go2-W robot streaming a 360° point cloud and its pose to a PRISM-VGGT
mapping server, viewed on a laptop.

If you have **never used Linux**, don't worry — this page starts from opening a
terminal and explains each new idea the first time it appears. Read it top to
bottom before touching the per-machine pages.

!!! tip "The one thing to remember"
    You almost never run raw commands by hand. The whole project is driven by
    **`make`** targets. Run `make help` to list them and `make steps` to print
    the full bring-up order. When in doubt, `make help`.

---

## The three machines

VAT runs on three computers that talk to each other over a VPN (a private
network — see [VPN setup](development/vpn.md)). Each has a distinct job.

| Machine | What it is | What runs on it | Setup page |
|---|---|---|---|
| **SERVER** | A GPU box (needs an NVIDIA GPU + CUDA 12.8) | The **Zenoh router** (the hub everything connects to) + the **PRISM-VGGT mapping server** (turns camera frames into a 3D point cloud) | [Server](server.md) |
| **ROBOT** | The Jetson computer on the Unitree Go2-W | The RICOH Theta X 360° camera driver + a Docker container running the ROS↔Zenoh bridge, camera publisher, and pose estimator | [Robot](robot.md) |
| **CLIENT** | Your laptop | The **VisPy** 3D viewer (`make viewer`) + diagnostic tools | [Client](client.md) |

!!! note "You don't need all three to start"
    You can bring things up one machine at a time and test each stage with the
    diagnostic tools. The [Bring-up Runbook](../bringup.md) walks the staged
    sequence (transport → frames → body → poses → full viewer).

---

## How the project is wired: `make`, `uv`, and `vat.env`

Three ideas make the whole repo work. Learn these and everything else follows.

**`make`** is a task runner. A file named `Makefile` at the repo root defines
named "targets"; you run one with `make <target>` (for example `make router`).
Each target just runs a known-good command for you, so you don't memorise long
command lines.

**`uv`** is the Python tool this project uses. It creates an isolated Python
environment (a private folder of packages, so different components can't break
each other) and runs code inside it. **Never call bare `python`.** Always use
the `make` targets — under the hood they run `uv run python …` in the correct
environment. Each component has its own environment, created by its own sync
target:

```bash
make sync-router     # SERVER: Zenoh router env
make sync-mapping    # SERVER: PRISM-VGGT mapping env (needs GPU + CUDA)
make sync-client     # CLIENT: viewer + diagnostic tools env
make sync-robot      # ROBOT:  host-side camera-to-Zenoh helper env
make sync-docs       # ANY:    documentation (mkdocs) env
```

You rarely run these yourself: the service targets (`make router`,
`make mapping`, `make viewer`, …) call the matching `sync-*` first, so the
environment is created automatically on first use.

**`vat.env`** is a single file at the repo root holding all public configuration
(router IP address, robot name, camera settings, tuning knobs — no secrets).
The `Makefile` reads it (`include vat.env`) and passes every value to the
commands it runs. To change how the system behaves, you edit `vat.env`, not the
code.

!!! warning "Every machine needs the same `ROUTER_IP`"
    In `vat.env`, `ROUTER_IP` must be the VPN address of the machine running
    the router. Set the **same** value on the robot and the client. Getting
    this wrong is the most common "nothing connects" mistake.

!!! note "Do not edit `vat.env` casually"
    The defaults in `vat.env` are tuned. The only values you normally change per
    deployment are `ROUTER_IP` and, on the robot, `NET_IFACE` if auto-detect
    picks the wrong network card.

---

## Linux from zero (skip if you know this)

You run every command in a **terminal** — a text window where you type commands
and press Enter. On Ubuntu, open it from the app grid ("Terminal") or press
`Ctrl`+`Alt`+`T`.

A few concepts you'll meet:

- **`sudo`** — runs a command as the administrator ("superuser do"). It will ask
  for your password. You need it to install system software or load kernel
  modules.
- **`apt`** — Ubuntu's software installer. `sudo apt install <package>` downloads
  and installs a program system-wide.
- **PATH** — the list of folders the shell searches for programs. After you
  install a tool like `uv`, you may need to open a **new** terminal so your PATH
  picks it up.
- **cloning a git repo** — downloading a copy of the project's source code (and
  its history) with `git clone`.
- **a "service" / systemd** — a background program the system starts
  automatically (e.g. on boot). `systemctl` controls services; `journalctl`
  shows their logs. The robot and server use these to auto-start on boot.
- **environment variables** — named values (like `ROUTER_IP=...`) the shell
  passes to programs. `vat.env` is a file full of them.

### Install the base tools

On each machine, first update the package list, then install `git` (to clone the
repo) and `make` (to run the targets):

```bash
sudo apt update
sudo apt install -y git make
```

`uv` (the Python tool) is installed with its own script, then made available on
your PATH:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

!!! tip "Open a new terminal after installing `uv`"
    The installer adds `uv` to your PATH, but existing terminals won't see it
    until you open a new one. Verify with:
    ```bash
    uv --version
    ```
    If that prints a version number, you're set. If it says "command not found",
    close the terminal and open a fresh one, then try again.

The SERVER and ROBOT need extra software (Docker, NVIDIA drivers, the camera
libraries). Those are covered on their own pages — install them there.

---

## Get the code (clone with submodules)

The project lives on GitHub. It includes a **submodule** — a second git repo
(PRISM-VGGT) nested inside this one. You must clone with `--recurse-submodules`
so that nested repo comes down too, or the mapping server won't build.

```bash
git clone --recurse-submodules https://github.com/zRafaF/vat-monorepo.git
cd vat-monorepo
```

Already cloned without `--recurse-submodules`? Pull the submodule in after the
fact from the repo root:

```bash
git submodule update --init --recursive
```

!!! note "Where to clone it"
    Anywhere in your home folder is fine (e.g. `~/vat-monorepo`). Just be
    consistent, and if you set up auto-start services later, point them at the
    path you actually used. **All `make` commands are run from the repo root**
    (the `vat-monorepo` folder that contains the `Makefile` and `vat.env`)
    unless a step says otherwise.

Confirm it works:

```bash
make help
```

You should see the list of targets grouped by Setup / Services / Tests. If you
get `make: command not found`, install `make` (above). If you see
`vat.env: No such file`, you're not in the repo root — `cd` into it.

---

## Recommended order

Bring the machines up in this order so each has something to talk to:

1. **[Server](server.md)** first — it runs the router (the hub) that everything
   else dials, plus the mapping server.
2. **[Robot](robot.md)** next — the camera feed and the ROS↔Zenoh bridge.
3. **[Client](client.md)** last — the viewer and diagnostics, which read from
   the other two.

For development you'll also want the **[VPN](development/vpn.md)** (Tailscale) so
the three machines can reach each other, and the
**[Documentation](development/documentation.md)** guide if you edit these pages.

When all three are installed, follow the **[Bring-up Runbook](../bringup.md)**
for the exact staged start-up sequence and the diagnostic checks at each stage.
