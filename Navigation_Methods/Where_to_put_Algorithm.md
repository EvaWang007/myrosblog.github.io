# 😙我应该怎么用自己的路径规划算法来替代Nav2中自带的路径规划策略😙
## 1.机器人在robot_patrol->house_patrol中实现对机器人寻址和导航
  In conclusion，这个文件的作用可以概括为：创建self.navigator对象，使用create_pose_stamped方法把输入的坐标点转换为欧拉坐标系中包含位置+姿态的完整位姿消息即way_points，然后把这个way_points给到follow_waypoints方法，在follow_waypoints中调用self.navigator的自带方法followWaypoints让小车根据输入坐标点导航。所以其实关键点就是这个followWaypoints，它是nav2包的一个类BasicNavigator的一个方法，专门用来计算坐标点再给小车进行导航，我们现在去看看里面到底是个什么玩意。

## 2.在nav2_simple_commander.robot_navigator中寻找关键的路径规划部分BasicNavigator类
   根据文件开头`from nav2_simple_commander.robot_navigator import BasicNavigator`，我试图去share的nav2_simple_commander里面找robot_navigator但是无果，里面都是launch文件和预编译文件，后来发现底层的源码(.py)文件在`/opt/ros/humble/lib/python3.10/site-packages/nav2_simple_commander`里面，于是我在里头找到了`robot_navigator.py`并查看。

## 3.robot_navigation是怎么实现导航的
   BasicNavigator 本质上是一个 ROS 2 Action Client 集成管理器。它的核心逻辑可以概括为：“只下指令，不干活”。
接口封装：它把复杂的 ROS 2 Action（异步动作）封装成了简单的 Python 函数（如 goToPose, followWaypoints）。
任务分发：当你调用 followWaypoints 时，它会创建一个 FollowWaypoints.Goal，然后通过 self.follow_waypoints_client 发送给后端的 Nav2 Server。
状态监控：它通过 _feedbackCallback 实时接收进度，通过 isTaskComplete 轮询任务是否结束。
规划器的关系：注意代码中的 _getPathImpl 方法，它向 compute_path_to_pose 服务发请求。在后端，处理这个请求的就是 Planner Server。

## 4.所以真正接收坐标点信息选择并运行算法的是_getPathImpl()方法
   _getPathImpl 扮演的是 Action Client（客户端） 的角色。
数据打包：代码中 goal_msg = ComputePathToPose.Goal() 这一行是在填订单。它告诉后端：起点 (start)、终点 (goal) 以及 指定规划器 ID (planner_id)。

异步发送：send_goal_async(goal_msg) 将这个订单通过 ROS 2 的 Action 通信机制发送出去。

等待结果：rclpy.spin_until_future_complete 会阻塞 Python 脚本，直到后端的“工厂”算好路径并返回。

🔯Planner Server：后端的“工厂经理”
当你运行 Nav2 时，有一个节点叫 planner_server。它是一个 C++ 节点，专门负责处理上述请求。
它的工作逻辑如下：

监听请求：它一直在后台运行，等待 ComputePathToPose 类型的订单。

解析订单：当它收到 goal_msg 后，它会看一眼订单里的 planner_id（对应你 YAML 里的插件别名）。
调度插件：如果 planner_id 是 "GridBased"，它会去它的“工具箱”里寻找对应的插件。

🔐：如果你的 YAML 配置了 plugin: "nav2_astar_planner/AStarPlanner"，那么经理就会把起点、终点和当前的代价地图 (Costmap) 全部交给你的 A* 插件。但是目前我们默认的配置是这样的`def _getPathImpl(self, start, goal, planner_id='', use_start=False)`，这说明我们现在的planner_id是一个空字符，后端会默认找 planner_plugins 列表里的第一个（比如 GridBased）。

## 5.所以要去planner_plugins.xml中给我们自己要写的算法注册一个`户口`，让它可以被找到，同时可以在`/opt/ros/humble/share/nav2_bringup/params`中可以查看到。
  打开params.yaml我们现在可以看到
  ```
  planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
  ```
  这说明目前使用的是 NavfnPlanner，由于 use_astar: false，它目前运行的是 Dijkstra 算法，我们希望改动之后的A*规划算法插件变成：
  ```
  planner_server:
  ros__parameters:
    planner_plugins: ["MyCoolAStar"] # 绰号
    MyCoolAStar:
      plugin: "my_astar_planner/MyAStar" # 指向户口名
 ```
这样系统就会自动调用我们自己写的算法对小车的路径进行规划

  
  


















