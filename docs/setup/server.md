# Server setup


## Zenoh

To install Zenoh router you can follow the detailed instructions is the [Zenoh documentation](https://zenoh.io/docs/getting-started/installation/).

On Ubuntu, you can use the following commands to install the Zenoh router:

```bash
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
```

``` bash
echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null
```

```bash
sudo apt update
```

```bash
sudo apt install zenoh
```

Then you can start the Zenoh router with this command:

```bash
zenohd
```

!!! Note
    We are not using docker because the documentation states that "Docker doesn’t support UDP multicast between a container and its host" so a bare metal installation is recommended.


### Bridge plugin
As we now have the repository added to the source list installing other zenoh plugins is as easy as installing any other package. For the ROS 2 bridge plugin you can run:

```bash
sudo apt install zenoh-plugin-ros2dds
```