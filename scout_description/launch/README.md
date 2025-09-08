## 启动仿真示意

``` bash
ros2 launch scout_description gz_sim_scout.launch.py
```

**发送控制命令**

``` bash
ros2 topic pub --rate 10 /diff_drive_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.7}, angular: {z: 1.0}}"
```
