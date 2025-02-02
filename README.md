# micro-ROS ESP32 Example

## TODO: Make separate REPO for adjusted micro-ROS component (with UART transport, menuconfig options, etc.)

## Build Process

1. Clone this repository
2. Inside the component folder (create it if not existant), clone the [micro-ros component](https://github.com/micro-ROS/micro_ros_espidf_component)
3. In the root of the repository, run `idf.py set-target esp32` to set the target to ESP32
4. Then run `idf.py menuconfig` to configure the project
  - In "micro-ROS Settings" -> "WiFi Configuration" set the SSID and password of your WiFi network
  - Also in "micro-ROS Settings", set the "Agent IP" and "Agent Port" to the IP and port of your micro-ROS agent (you can use `ifconfig` to get the IP of the machine running the agent)
  - Then press "Q" to exit the menuconfig and save the configuration (you will be prompted to do so)
5. Run `idf.py build` to build the project
6. Run `idf.py flash monitor` to flash the project to the ESP32 and open the serial monitor
7. If everything is correctly configured, you should see the ESP32 connecting to the micro-ROS agent

## Notes

- The micro-ROS agent must be running in the same network as the ESP32
  - Use `docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888` to run the agent in the same network
- You can then run another container with ros to interact with the system
  - Use `docker run -it --rm --net=host osrf/ros:jazzy-desktop` to run a container with ROS
  - `ros2 node list` should show the ESP32 node
  - `ros2 param list` should show the parameters of the ESP32 node
  - Parameters can be retreiaved and set using `ros2 param get` and `ros2 param set`
- If you want to change the transport to UART, you can do so using these steps:
  1. Adjust "DRMW_UXRCE_TRANSPORT" in the "app-colcon.meta" file to "uart"
  2. Clean the components using `idf.py clean-microros` and `idf.py clean`
  3. Again using `menuconfig`, in "micro-ROS example-app settings" toggle "Use Custom Transport" to "ON"
  4. In "micro-ROS Settings" -> "micro-ROS network interface select" set it to "Micro XRCE-DDS over UART. Check int32_publisher_custom_transport"
  5. Then again save and exit the menuconfig
  6. Now build and flash the project again, the micro-ROS agent must be running with the UART transport enabled (`docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -v6`)
- Custom message definitions can be added as a package in "components/micro_ros_espidf_component/extra_packages"
  - After changing anything within the micro-ros component, you must run `idf.py clean-microros` to clean the component before building the project again
  
## TODO
- Improve the README
- Resolve TODOs in the code
- Utilize Docker Compose for running the agent and ROS
- Integrate the Dev Container for VSCode
- Integrate with Gazebo simulation