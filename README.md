# ros_multi_target_navigation
使用ros+movebase进行多目标点巡航。



## 起

最近的工作开始转向机器人方面的研究了，当前需要解决的一个子任务是让四足机器人可以定点巡航(即预先给定几个机器人需要到达的位置，让机器人依次到达这些特定的点位)。对于一个刚踏入机器人领域的小白来说，最快的方式就是借鉴(抄代码)，在网上搜索了一番后发现了一个让我相对完整且通俗易懂的实现([multi_target_navigation](https://github.com/wjjcdy/multi_target_navigation))，于是决定在此基础上修改，将其实现成我所需要的形式。



## 承

阅读和测试该仓库([multi_target_navigation](https://github.com/wjjcdy/multi_target_navigation))代码的时候发现，该仓库将目标点设置到.yaml文件中，然后从.yaml文件中读取需要到达的特定点位，但是在程序读取这些点位的时候，发现读取点位的位置和设置的不一样，因此机器人会到达错误的位置，甚至超过地图的边界，此外还发现一个yaw角度转换到四元数的错误。尝试使用chatgpt帮我纠正错误，同时对原始代码进行了如下优化，增加了一些其他功能：

(1) 用户可以通过注释或者打开`multi_target.h`文件中的

```c++
#define LOOP_NAVIGATION   // 用于判定是否循环执行目标点的标志，打开执行循环，注释只执行设定的几个点
```

参数，可以选择让程序是否在设置的目标点中循环执行。



(2)用户可以设置执行一个目标点的最大时长，通过修改`multi_target.h`下的

```c++
ros::Duration max_goal_time_ = ros::Duration(100.0);  // 设置目标点超时时长，即超过100s未到达目标点，即跳过该点，执行下一个目标点
```

来实现，如果当前机器人在100s内无法到达该点，机器人会跳过该点，继续执行下一个目标点。因为有些情况下机器人多次调整无法到达，机器人会陷入僵局，通过该设置，可以稍微避免这种情况。



(3)用户可以通过修改配置参数`config.yaml`文件，设定需要到达的目标点参数，目标点个数，确认到达目标点位可接受的阈值等

```yaml
goal_poses:
  - {x: -4.38975, y: -1.60161, theta: -34.14118}
  - {x: -4.73559, y: 1.11929, theta: 80.66521}
  - {x: 2.22491, y: 2.41651, theta: -76.65672}
  - {x: 1.13148, y: -0.54157, theta: 104.49494}
# x和y分别表示2d地图中的x和y的坐标，theta表示yaw角度(yaw角度用角度制表示)

pose_number: 4 # 表示当前设置4个目标点

position_tolerance: 0.3 # 位置误差容忍阈值（米）

angle_tolerance_deg: 15.0 # 角度误差容忍阈值（度）
```

因为我在实际测试的时候，使用的是四足机器人，发现如果不增加一些到达目标点的误差范围，会让机器人在一个位置反复调整，从而超时或者一直无法到达该点（因为四足机器人稍微挪动就会超过该位置或者角度对不上）。因此我增加了误差阈值的判断，在当前机器人到达的位姿与设定目标点的位姿在设定的误差范围内时，即设定机器人已到达该点，从而可以让机器人继续行进到下一个目标点。（该设置适用于那些精确度没有那么高的巡航任务）

此外，请确保目标点数和目标位姿的数目保持一致。



## 转

本仓库代码的具体使用方式如下：

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/zhahoi/ros_multi_target_navigation.git
$ cd ..
$ source devel/setup.bash
$ roslaunch navigation_test navi_test.launch
```

注：使用前请确保启动movebase。



## 合

本仓库是我在学习ROS和机器人之后上传的第一个仓库，希望后面可以继续学习更多相关的知识。也希望本仓库可以帮助那些刚入门机器人的新手，如果觉得本仓库的代码质量还不错的话，麻烦给一个star或者fork，这是我开源自己代码最大的动力。以上。



## Reference

-[multi_target_navigation](https://github.com/wjjcdy/multi_target_navigation)
