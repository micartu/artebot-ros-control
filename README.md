# ArteBot ROS Control

ROS2 Node for controlling a ArteBot via ROS Control interfaces.

## Setup

From a WaveShare ArteBot robot, clone this repository and open it over SSH. Install the Dev Containers extension and allow it to build the Docker container and open the workspace in that container.

Then build the code using `colcon build` from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build
```

## Testing the code

Once built, you can run the test function to check that the library can be loaded correctly as follows:

```bash
source install/setup.bash
./build/artebot_control/test_artebot_system
```

The test should pass with log messages showing a successful initialization of the control library.

## Running the code

Once built, it should be possible to launch the artebot controller using the provided launch file. Note that this will only run on a ArteBot with the i2c-1 bus available.

```bash
source install/setup.bash
ros2 launch artebot_control artebot.launch.py
```

After this, the robot will respond to commands sent with `TwistStamped` type sent on the `/cmd_vel` topic.

If you want to run mock hardware for any reason, such as not having a ArteBot to run the code on, you can enable the flag as follows:

```bash
ros2 launch artebot_control artebot.launch.py use_mock_hardware:=true
```

## License

The code in this repository is covered by the MIT license in the [LICENSE](./LICENSE) file. However, four files are included from the [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) repository, and so are covered by the [ROS2_CONTROL_LICENSE](./ROS2_CONTROL_LICENSE) file instead. These four files are as follows:

1. [artebot.launch.py](./bringup/launch/artebot.launch.py)
2. [artebot_system.cpp](./hardware/src/artebot_system.cpp)
3. [artebot_system.hpp](./hardware/include/artebot_control/artebot_system.hpp)
4. [visibility_control.h](./hardware/include/artebot_control/visibility_control.h)
