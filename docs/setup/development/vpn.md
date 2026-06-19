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

!!! tip

    After the first authentication the vpn is supposed to start automatically on boot. To double check you can run:
    ```bash
    sudo systemctl is-enabled tailscaled
    ```

After the installation and authentication you should see the new device in the Tailscale admin console.

For quick reference, see [my setup](./vpn.md#my-setup) for the IP addresses of the devices on my network.

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