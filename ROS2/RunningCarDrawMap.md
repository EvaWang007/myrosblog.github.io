# Gazebo+Rviz2+Slamtool小车在三维环境移动自主绘制二维栅格地图
## 工作流程
![f1f57f5e5a02bb8402341dc1ac4f9442](https://github.com/user-attachments/assets/be853696-4f4e-44c5-b604-e30aee9aa175)
### *step1* 
    下载SLAM工具（ROS1对应gmapping,ROS2对应slam-tools)
### *step2*
    启动Gazebo仿真环境，先加载模型告诉ros2现在使用的模型再运行launch脚本，这个launch是已经把小车的walk放进去了所以显示的是小车在3D环境里面跑
    export TURTLEBOT3_MODEL=burger
    ros2 launch my_turtle_demo sim_all.launch.py
### *step3*
    打开一个新终端，启动 slam_toolbox。它会订阅小车的激光雷达（Lidar）和里程计（Odom）数据，实时生成栅格地图
    export TURTLEBOT3_MODEL=burger
    ros2 launch slam_toolbox online_sync_launch.py
    对该命令做出解释，该命令的启动部分是ros2 launch，是先去ros2的安装目录里面找slam-_toolbox然后再在该包里面找online_sync_launch文件
    该命令会产生的反应是参数读取：slam_toolbox 读取环境变量，知道自己现在是在为一台 burger 小车服务
    建立连接：节点开始在后台监听两个核心数据：
    /scan 话题：获取激光雷达扫描到的障碍物距离。
    /odom 话题：获取小车轮子转动产生的里程计位姿。
    标变换（TF）：算法通过计算，确定小车在地图空间中的位置。
    地图发布：算法不断对比新旧雷达数据，修正位姿，并持续发布 /map 话题。这个话题就是你最终在 Rviz2 中看到的黑白格子图
## 注意事项
 ### *可以通过请打开一个新终端，检查以下两个话题是否有数据跳动：
        雷达数据： ros2 topic echo /scan
        里程计数据： ros2 topic echo /odom
        结果判断： 如果终端没有数据输出，说明你的 Gazebo 仿真环境 没有正常运行，或者小车模型没有配置好传感器。请确保 Gazebo 窗口里能看到激光雷达的红线或蓝线
 ### *通过打开 ros2 topic list检查目前已经打开的话题，检查是否有/map
 ### *打不开Rviz2的时候可以手动先开，然后手动配置Rviz2的正确姿态（前提，已经有list到了/map话题），配置方式如下
      Global Options -> Fixed Frame: 必须设置为 map。如果下拉菜单没有 map，手动输入单词 map。
      Map 组件 -> Topic: 确保选择的是 /map，且 Reliability Policy 尝试在 Reliable 和 Best Effort 之间切换一下。
      RobotModel: 如果小车显示为红色或报错，通常是 Fixed Frame 没选对
     
 ### *Slam-tools和Gazebo的数据发布的时间一定要同步，在 Gazebo 仿真中，如果 SLAM 节点认为现在是 2026 年，而 Gazebo 发布的数据时间戳是从 0 秒开始的（仿真时间），节点会认为这些数据全部过期而丢弃导致加载不出地图
      将step3的第二行命令换成：ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true
 
    
    

    
    
    

    

