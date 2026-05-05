# Robot setup


## Python virtual environment
You will need a venv with `python>=3.9`, the unitree go 2 comes native with `3.8` and `3.9`.

To start a new venv and install the dependencies, run the following commands:
```bash
# Ensure you are on the robot directory
cd robot

/usr/bin/python3.9 -m venv venv

source venv/bin/activate

pip install -r requirements.txt
```

## Zenoh ROS 2 bridge

<!-->
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
-->

```bash
sudo ROS_DISTRO=foxy zenoh-bridge-ros2dds -c ./DEFAULT_CONFIG.json5 -r 8001
```