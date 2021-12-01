# ROS2 LEAP MOTION DRIVER

(Partial) ROS2 driver for the Leap Motion Controller

## REQUIREMENTS

You should have [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html) installed on your device and the [Leap Motion SDK](https://developer.leapmotion.com/tracking-software-download) for Linux.

## FEATURES

Currently, this ROS2 package supports publishing raw camera images from the controller, basic visualization using RViz and a pointcloud2 generated from [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) node. Hand tracking features are still not ported. 


## INSTALLATION

**1.** Download the SDK from [Leap Motion](https://developer.leapmotion.com/tracking-software-download).

**2.** Uncompress the tar file and install the deb file inside it.
```bash
    gunzip -c Leap_Motion_SDK_Linux_2.3.1.tgz | tar xvf -
    cd LeapDeveloperKit_2.3.1+31549_linux/
    sudo dpkg -i Leap-2.3.1+31549-x64.deb  # or Leap-2.3.1+31549-x86.deb
```

**3.** Colcon seems to have problems finding `libLeap.so`. This is the quickiest (and dirtiest) way to fix it: copying it.
```bash
    sudo cp LeapSDK/lib/x64/libLeap.so /usr/lib
    # or sudo cp LeapSDK/lib/x86/libLeap.so /usr/lib
```

**4.** Clone the repo to your workspace.

```bash
    cd ~/catkin_ws/src
    git clone https://github.com/TaISLab/leap_motion.git
```

**5.** Copy the Leap Config file from the repo to the system-wide location:

```bash
    sudo cp leap_motion/config/leapd.conf /var/.Leap\ Motion/config.json
```

**6.** Configure Leap Motion systemd service

- Create a service file

```bash
   cd /lib/systemd/system
   sudo nano leapd.service
```

- The file should be like this:
```bash
[Unit]

Description=LeapMotion Daemon

After=syslog.target

[Service]

Type=simple

ExecStart=/usr/sbin/leapd --run --config=/var/.Leap\ Motion/config.json

[Install]

WantedBy=multi-user.target
``` 

- Link it:
```bash
   sudo ln -s /lib/systemd/system/leapd.service/etc/systemd/system/leapd.service
   systemctl daemon-reload
```

- Now this should start the service:
```bash
   sudo service leapd start
```

- You can check the status with:
```bash
   sudo service leapd status
```

- And see te output with this command:
```bash
   Visualizer
```

**7.** Compile the ros2 package:

```bash
    cd ~/catkin_ws/
    colcon build --symlink-install --packages-select leap_motion
```

### Usage

**1.** (OPTIONAL) If it gives you an error about the leap daemon not running, stop the LeapControlPanel have a look [here](https://forums.leapmotion.com/t/error-in-leapd-malloc/4271/13) and use the following command:

```bash
sudo service leapd restart
```

**2.** Source your current catkin workspace.

```bash
source ~/catkin_ws/install/setup.bash
```

**3.** Launch `camera.launch.py` file. 

```bash
ros2 launch leap_motion camera.launch.py
```

**4.** You can use Rviz config file `leap_camera.rviz` to see the output

### Foreword
Check Jeff Gensler's [ROS2 Image Pipeline Tutorial](https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329) to learn a bit more about the disparity images, great reading!
