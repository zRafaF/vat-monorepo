


## Setting up the insta360 ros driver


For detailed instructions on setting up the Insta360 ROS driver, please refer to the [Insta360 ROS Driver Repository](https://github.com/ai4ce/insta360_ros_driver). The repository contains comprehensive documentation and step-by-step guides to help you get started with the Insta360 camera in your ROS environment.

First you will need to have the ros system enabled, on the go2 you can do this by running the following command:

``` bash
source ~/unitree_ros2/setup.sh
```

> For more info please check the official documentation [here](https://support.unitree.com/home/en/developer/ROS2_service)


``` bash
# do this inside the robot directory
git clone -b humble https://github.com/ai4ce/insta360_ros_driver

```

Then you will need your Insta360 camera's SDK, you will need an account and request access. You can download it from here: https://www.insta360.com/sdk/record

![alt text](assets/image.png)


> [!TIP]
> You can right click on the download button and select "Copy Link Address" to get the direct download link for the SDK, which you can use in the terminal with `wget` or `curl` to download it directly to your robot. It should look something like this: `https://wassets.insta360.com/common/<my_key>/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip`


You will then need to unzip the SDK and follow the instructions in the `README.md` file in the `insta360_ros_driver` repository to build and run the ROS driver for the Insta360 camera. This will allow you to stream video data from the camera into your ROS ecosystem, where you can process it using Zenoh for efficient data handling and communication.




    