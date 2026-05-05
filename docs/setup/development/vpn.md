# VPN network for easier development and testing

For a real world application of course we will have the different parts of the system running on different machines ans probabbly connected over the internet. For development and testing purposes it is easier to have just a vpn as that allows us finer routing and addressing control. For this we will be using [Tailscale](https://tailscale.com/) which is a zero config VPN that uses the WireGuard protocol. It is free for personal use and easy to set up.


## Account creation and installation
a


### On the robot
{{ lorem(3) }}.

### On the remote server

To access via ssh use 
```bash
sudo tailscale up --ssh
```