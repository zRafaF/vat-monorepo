# VPN network for easier development and testing

In a real deployment the parts of the system run on different machines,
probably connected over the internet. For development and testing it is much
easier to put them all on one VPN, which gives us finer routing and addressing
control. We use [Tailscale](https://tailscale.com/) — a zero-config VPN built on
the WireGuard protocol. It is free for personal use and easy to set up.

The result: every machine (robot, server, laptop) gets a stable `100.x.y.z`
address that works from any other machine on your tailnet, regardless of which
network it's physically on. You put those addresses in `vat.env` (`ROUTER_IP`)
and everything can reach the router.

!!! note "New terms"
    - **VPN** — a private network overlaid on top of the real internet, so your
      machines behave as if they were on the same LAN.
    - **tailnet** — your personal Tailscale network. Every device you log in
      joins it.
    - **SSH** — opening a remote terminal on another machine. Tailscale can do it
      for you with `--ssh`.

---

## Account creation and installation

First, create a free Tailscale account at
[login.tailscale.com](https://login.tailscale.com/start) (sign in with Google,
GitHub, Microsoft, etc.). You'll log every machine into this **same** account so
they all join one tailnet.

Then install and authenticate Tailscale on each of the three machines. The
install command is the same on any Linux machine (robot and server); the laptop
uses the app for its OS.

### On the robot and the server (Linux)

Install Tailscale with the official script (`sudo` runs it as administrator —
it will ask for your password):

```bash
curl -fsSL https://tailscale.com/install.sh | sh
```

Then bring it up. On the **robot** and **server** enable SSH access at the same
time so you can log in over the VPN:

```bash
sudo tailscale up --ssh
```

The first time, this prints a URL. Open it in any browser and log in to your
Tailscale account to authorize the machine. After that it's authenticated.

Verify the machine joined and see its address:

```bash
tailscale ip -4        # prints this machine's 100.x.y.z VPN address
tailscale status       # lists every machine on your tailnet
```

After the installation and authentication you should also see the new device in
the [Tailscale admin console](https://login.tailscale.com/admin/machines).

### On the laptop (client)

Install the Tailscale app for your OS from
[tailscale.com/download](https://tailscale.com/download) (Windows, macOS, or
Linux) and log in with the same account. On Linux you can use the same
`curl … | sh` install and then `sudo tailscale up` (the laptop doesn't need
`--ssh` unless you want to SSH *into* it).

### Autostart on boot

After the first authentication, the VPN is supposed to start automatically on
boot. To double-check on Linux (robot/server):

```bash
sudo systemctl is-enabled tailscaled
```

It should print `enabled`. If it prints `disabled`, enable it:

```bash
sudo systemctl enable --now tailscaled
```

For quick reference, see [my setup](#my-setup) for the IP addresses of the
devices on my network.

---

## My Setup
**Rafael's Note**

On my network I have the following devices:

| Name      | Description                          | IP Address                          |
| ----------- | ------------------------------------ | ------------------------------------ |
| `caramelo`       | My laptop  | `100.87.118.34` |
| `ubuntu-unitree-go2`       | The robot | `100.106.128.63` |
| `lab-dell-g16-7630`    | Laptop server I'm using for testing | `100.125.156.19` |


```bash
ssh unitree@ubuntu-unitree-go2
```
