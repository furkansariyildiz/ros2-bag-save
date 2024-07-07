# ros2-bag-save
ROS2 Bag saver package 

###To build ros2-bag-save package
```bash
cd ~ros2_ws/src && git clone git@github.com:furkansariyildiz/ros2-bag-save.git
colcon build --symlink-install --packages-select ros2_bag_save
```

### Run this package via ros2 run command (you may need to import ROS2 parameters which are imported on YAML file)
```bash
cd ~ros2_ws && source install/setup.bash
ros2 run ros2_bag_save ros2_bag_save.py
```

### Run this package via ros2 launch command
```bash
cd ~ros2_ws && source install/setup.bash
ros2 launch ros2_bag_save ros2_bag_save.launch.py
```