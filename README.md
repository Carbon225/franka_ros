# ROS integration for Franka Emika research robots

## Docker-RT setup

Docker-RT is a minimal OS for the Raspberry Pi 5, preinstalled with Docker and the Franka stack on ROS Noetic.

### Installation

1. Download it [here](https://aghedupl-my.sharepoint.com/:f:/g/personal/carbon_agh_edu_pl/IgCXMDQIZ4zvRJ1E5bF92basAS1JBcYGyLqWRrZn94JOHu8?e=CLu7w8)

2. Install [Raspberry Pi Imager](https://www.raspberrypi.com/software/)

3. Select the downloaded `docker-rt-x.y.z.img` (Choose OS -> Use custom) and flash it to your SD card/USB drive

4. Insert the SD card/USB drive into your Raspberry Pi 5 and power it on

### First boot (one time only)

1. Connect to the WiFi network `DOCKER-RT` with the password `12345678`

2. Connect via SSH `ssh root@192.168.42.1` with the password `1234`

3. Install the Franka ROS docker image with `docker load -i /usr/share/docker-rt/franka_ros.tar`

    This might take a few minutes.

### Connect to the Web UI

1. (On the PC) SSH with port forwarding `sudo ssh -L 127.0.0.1:443:192.168.1.100:443 root@192.168.42.1 sleep inf`

    This will forward the local port 443 to the robot's port 443.

2. Open a supported browser and navigate to `https://127.0.0.1`

3. Login and enable the FCI

### Connect to the robot

1. SSH to the Raspberry

2. Ping the robot with `ping 192.168.1.100`

3. Move the robot to the starting position with `docker run -it --rm --net=host --privileged franka_ros communication_test 192.168.1.100`

### Running the ROS container

1. Start the ROS container with `docker run -it --rm --net=host --privileged franka_ros`

2. Run the desired ROS launch file, for example

    ```bash
    roslaunch franka_example_controllers cartesian_impedance_external.launch
    ```

3. In a new terminal, start a new container and inspect the robot's state

    ```bash
    docker run -it --rm --net=host --privileged franka_ros
    rostopic list
    rostopic echo /franka_state_controller/franka_states
    ```

### Use ROS from your PC

1. Connect to the DOCKER-RT WiFi network

2. Point ROS to the Raspberry Pi

    ```bash
    # On your PC
    export ROS_MASTER_URI=http://192.168.42.1:11311
    export ROS_IP=192.168.42.x # Enter your machine's IP address in the DOCKER-RT network
    ```

3. List the available topics

    ```bash
    rostopic list
    ```
