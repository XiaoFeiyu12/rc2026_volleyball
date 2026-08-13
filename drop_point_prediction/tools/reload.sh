# 方便每次改代码调参数编译使用
colcon build --parallel-workers 4
source install/setup.bash
ros2 launch volleyball_robot bring_up.launch.py
