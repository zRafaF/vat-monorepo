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
zenohd --listen tcp/0.0.0.0:7447 --rest-http-port 39522
```

!!! Note
    We are not using docker because the documentation states that "Docker doesn’t support UDP multicast between a container and its host" so a bare metal installation is recommended.


### Setting up auto start for Zenoh router

To set up auto start for the Zenoh router, you can create a systemd service file. Here are the steps to do that:

First find where the zenohd binary is located:

```bash
which zenohd

# /usr/bin/zenohd
```

Then create a systemd service file for the Zenoh router:

```bash
sudo nano /etc/systemd/system/zenohd.service
```

Add the following content to the file:

```ini
[Unit]
Description=Zenoh Router
After=network.target

[Service]
Type=simple
User=lab # replace with your username
ExecStart=/usr/bin/zenohd --listen tcp/0.0.0.0:7447 --rest-http-port 39522
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Now enable and start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable zenohd
sudo systemctl start zenohd
```

To check if the service is running, you can use:

```bash
curl http://localhost:39522/@/router/local/info
```