# Robot setup


## Bridge node

To get the ros topics from the robot to be available on the Zenoh network we have created a bridge node that uses the Zenoh Python API to forward the topics.

!!! note
    This is probably a temporary solution, there are two other easy options:
    1. Use a docker environment with a newer version of ros that supports python 10 (in this case humble) and then connect the forwarder from it.
    2. Utilize the Zenoh c version and create a ros node in c that forwards the topics to the Zenoh network. This would be more efficient and would not require a separate python environment, but it would require more development time.

    > There is also a third option which is to use the Zenoh ROS 2 bridge plugin, but it is currently not working with the robot setup. More details on that can be found in the failure section below. 


The current bridge architecture is composed of two parts in two different python version:

### Python 3.8
This is the script that connects to the ros topics, the communication to the other script that does the forwarding is done via a tcp socket. This script is using python 3.8 because that is the version that is compatible with ros foxy, which is the version we are using for the robot component.

As it doesnt use external libraries it can be run with:

```bash
python ros_to_tcp.py
```


### Python 3.9

This is the script that connects to the Zenoh network and forwards the topics received from the other script. This script is using python 3.9 because that is the version that comes with the unitree go 2 robot but can easily be changed to a newer python version that is compatible with the zenoh python api.

#### Python virtual environment
To start a new venv and install the dependencies, run the following commands:

```bash
# Ensure you are on the robot directory
cd robot

/usr/bin/python3.9 -m venv venv

source venv/bin/activate

pip install -r requirements.txt
```

To run the script, make sure to activate the virtual environment and then run:

```bash
source venv/bin/activate

python tcp_to_zenoh.py
```

---



??? failure "[failure] Zenoh ROS 2 bridge plugin"
    The Zenoh ROS 2 bridge plugin is currently not working with the robot setup. The external ros nodes are getting discovered and forwarded to the bridge, but the robot component is not able to be detected. It is still unsure if that is a shared memory problem or something else.

    Trying building it from source with the shared memory transport enabled caused many problems with the cmake version and the dependencies. Thus the current version of the bridge described above can be seen as a workaround to get the robot component working, but it is not ideal. The ideal solution would be to have the bridge plugin supplying the topics in a transparent manner.

    ??? note "Previous version of the documentatin"
        One problem with the compiled binaries of the Zenoh ROS 2 bridge plugin is that they are not using shared memory transport, which is required for the robot component to work. To solve this issue, we need to compile the plugin from source.

        Installing dependencies:
        ```bash
        sudo apt install libacl1-dev libncurses5-dev
        ```

        !!! Tip
            If using the unitree go 2 robot you might encounter problems with the default version of cmake that comes with ubuntu 20.04. To circumvent this issue you can try and install a newer version side by side just for the build process.

            ```bash
            pip3 install cmake==3.26.4
            ```

            Point the Build to the New CMake

            ```bash

            # Add the pip binary folder to the start of your PATH
            export PATH=$HOME/.local/bin:$PATH

            # Verify the version (it should now say 3.26.4)
            cmake --version
            ```

            ```bash
            # If needed, clean previous build artifacts with `cargo clean` before rebuilding

            ROS_DISTRO=foxy cargo build --release -p zenoh-bridge-ros2dds --features dds_shm
            ```


        ```bash
        sudo ROS_DISTRO=foxy zenoh-bridge-ros2dds -c ./DEFAULT_CONFIG.json5 -r 8001
        ```
