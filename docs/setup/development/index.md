# Development setup

VAT is a **distributed** system: the robot, the server, and the client are three
separate machines that talk over a network. For a real deployment they'd be
spread across the internet, but for development and testing it's far easier to
put all three on one small private network (a VPN) so they can reach each other
by a stable address with no firewall hassle.

This section covers the two things you need for a comfortable dev loop:

- **[VPN](vpn.md)** — a [Tailscale](https://tailscale.com/) VPN (zero-config,
  built on WireGuard, free for personal use) that connects the robot, server,
  and your laptop, and lets you SSH between them. Set this up first.
- **[Documentation](documentation.md)** — how to preview, build, and deploy
  these docs (they're a MkDocs site) when you edit them.

The day-to-day workflow itself is the same `make` + `uv` + `vat.env` model used
everywhere in the project — see the [setup overview](../index.md#how-the-project-is-wired-make-uv-and-vatenv)
if you haven't read it. In short: every action is a `make` target, each component
has its own isolated `uv` environment, and all shared config lives in `vat.env`
at the repo root.

Once the VPN is up, follow the per-machine pages ([Server](../server.md),
[Robot](../robot.md), [Client](../client.md)) and then the
[Bring-up Runbook](../../bringup.md) to start everything in order.
