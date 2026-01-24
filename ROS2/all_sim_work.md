# 问题描述1：Gazebo（仿真环境）、RViz2（可视化）和 SLAM（算法）之间的时间不一致数据接收错位

## 🔐Solution 1: 开启 use_sim_time 参数 (最关键)
这是解决问题的根本。确保所有节点（包括 SLAM-toolbox和RViz2和Gazebo和机器人驱动模块）都将 use_sim_time 参数设置为 True。ROS2系统中，一般由Gazebo提供时钟源，需要先启动Gazebo env这样系统里面才有/clock话题，然后启动slam-tools处理数据和机器人驱动跑图，最后启动Rviz2可视化。这里的Rviz2只是一个订阅者（Supscription)并不负责发布任何消息，因此逻辑上应该最后启动。我发现最好解决方式就是在launch文件把所有环境变量和文件执行指定明白，防止出现这个先读了那个没读到时候加载不出来。

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    # 1. 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 2. 环境变量 (确保 Gazebo 知道我们要加载哪辆车)
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_bringup = get_package_share_directory('turtlebot3_bringup')
    # 如果你安装了 slam_toolbox，获取其路径
    pkg_slam = get_package_share_directory('slam_toolbox')

    # 3. 启动 Gazebo
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. 启动 SLAM-toolbox (新增：这是绘图的关键！)
    start_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_slam, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    # 5. 启动 RViz2
    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'rviz2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 6. 你的控制节点
    run_walker = Node(
        package='my_turtle_demo',
        executable='walker_node', 
        output='screen',
        parameters=[{'use_sim_time': True}] # 强制 True
    )

    return LaunchDescription([
        set_model,
        start_gazebo,
        start_slam,  # 确保 SLAM 被加入
        start_rviz,
        run_walker
    ])
```
采用以上的定义全局环境变量`use_sim_time = LaunchConfiguration('use_sim_time', default='true')`的形式统一控制时间；显示包含slam话题防止/map收不到slam-tools的发出的话题导致加载不出map;
TF 树闭环：start_gazebo 提供odom -> base，start_slam提供 map -> odom。这样RViz里的Fixed Frame设置为 map 时，一切才能连通。

# 问题描述2：slam-tool可以接收到数据但是无法绘图（TF树断裂问题）
### 🔐Solution 2:在 RViz2 左侧最上方的 Global Options 里：Fixed Frame 必须设置为 map。
如果下拉菜单里没有 map，手动输入 map。如果输入后报错变红，说明 SLAM 节点还没发布 map -> odom的TF；
在 RViz2 左侧配置栏找到 Map -> Topic。将 Durability Policy 改为 Transient Local。SLAM 发布的地图属于“静态数据”，频率很低，Transient Local 允许 RViz2 在启动后获取之前已经发布过的地图

# 查错常用语句☘️
1. 检查TF树：` ros2 run tf2_tools view_frames `查看各个结点链之间是不是符合要求，尤其是是map是否存在断裂
2. 检查地图话题是否有数据：` ros2 topic echo /map --tail 1 `如果没有数据输出，说明 SLAM 节点配置有问题，没在发图
3. 检查雷达话题是否正常：` ros2 topic hz /scan ` ,如果没有频率，说明仿真环境里的雷达坏了，SLAM 没素材画图
4. 查看slam内部统计：` ros2 topic status /map `， 查看发布者(publisher)是否存在矛盾，有时候scan的发布会抢占/map话题导致本来应该由slam-tools发布的出现矛盾
5. 查看所有话题： `ros2 topic list `

































