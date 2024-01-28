# ROS入门

## 安装ROS noetic

第一步，设置sources.list：

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

第二步，设置key

```shell
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

第三步，更新package：

```shell
sudo apt update
```

第四步，安装ROS：

```shell
sudo apt install ros-noetic-desktop-full
```

第五步，更新bash：

```shell
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

第六步，安装依赖：

```shell
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

第七步，初始化：

```shell
sudo rosdep init
rosdep update
```

这只是提取了官方文档中的关键部分，完整文档参见：

- [noetic/Installation/Ubuntu - ROS Wiki](https://wiki.ros.org/noetic/Installation/Ubuntu)

## ROS初探

首先，打开一个终端，启动roscore：

```shell
roscore
```

然后，再打开一个终端，运行turtle：

```shell
rosrun turtlesim turtlesim_node
```

最后，打开一个终端，为turtle启用键盘控制：

```shell
rosrun turtlesim turtle_teleop_key
```

然后，你应该可以使用方向键操作这个小海龟了：

![](images/2024-01-15-10-04-58-image.png)

## 项目创建与Vscode环境配置

首先，创建一个文件夹用于放置项目，并在其中创建src文件夹，例如：

```shell
mkdir ros
cd ros
mkdir src
```

然后，使用以下指令创建项目：

```shell
catkin_make
```

在工作目录（这里是ros目录）下，使用以下指令打开vscode：

```shell
~/ros$ code .
```

打开后，推荐安装以下拓展：

![](images/2024-01-15-10-16-42-image.png)

安装后如果没有生效就多关闭重开几次，之后应该会在`.vscode`文件夹下看到自动生成的`c_cpp_properties.json`和`settings.json`。

然后，右键src文件夹，选择`Create Catkin Package`：

![](images/2024-01-15-10-24-04-image.png)

设置包名，这里叫`lesson`；再输入依赖项，这里我们需要`roscpp std_msgs tf2_ros`。

在`lesson/src`目录下创建你的源文件，比如`lesson.cpp`：

## 代码实例

现在来看很多人都没有做的进阶算法ROS操作题：

> 在ROS操作系统中实现以下程序：给定坐标系A和B，坐标系A到坐标系B之间的位置变换（x，y，z）为（1，-2，0），角度变换（roll，pitch，yaw）为（-90°，0，45°）；在坐标系A下有一个位姿点M，位置坐标（x，y，z）为（1，1，1），角度（roll，pitch，yaw）为（0，0，0）
> 
> a) 发布坐标系A和坐标系B之间的静态坐标系变换；（10分）
> b) 以geometry_msgs::PoseStamped为类型，发布位姿点M（在A坐标系下）的位姿话题信息；（10分）
> c) 以geometry_msgs::PoseStamped为类型，发布位姿点N（在B坐标系下）的位姿话题信息，使得点N与点M在空间中重合；（10分）
> d) 在ROS的可视化界面rviz中显示坐标系A和B之间的TF变换以及点M、点N的位姿信息，截图附在答案中；（10分）

第一题，发布坐标系A和坐标系B之间的静态坐标系变换：

```cpp
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lesson");

    // 发布坐标系A和坐标系B之间的静态坐标系变换
    geometry_msgs::TransformStamped transform_stamped;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    tf2::Transform transform;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "A";
    transform_stamped.child_frame_id = "B";
    transform.setOrigin(tf2::Vector3(1, -2, 0));
    transform.setRotation(tf2::Quaternion(M_PI / 4, 0, M_PI / -2));
    transform_stamped.transform = tf2::toMsg(transform);
    static_broadcaster.sendTransform(transform_stamped);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
```

然后，在`CMakeLists.txt`的最后增加以下两行：

```cmake
add_executable(lesson src/lesson.cpp)
target_link_libraries(lesson ${catkin_LIBRARIES})
```

用`Ctrl+Shift+B`中的`catkin_make: build`来编译项目。

用`Ctrl+P`中的`>ROS: Start`启动roscore，，之后，使用`>Ros: Run`启动包。

**这个启动会export所有环境变量，但似乎有些问题，我没找到解决办法，但不影响使用就是了……**

现在，我们可以使用`rviz`查看静态坐标系变换了，在其中添加一个TF：

![](images/2024-01-15-11-55-44-image.png)

![](images/2024-01-15-11-56-49-image.png)

第二题，以geometry_msgs::PoseStamped为类型，发布位姿点M（在A坐标系下）的位姿话题信息：

```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lesson");

    // 发布坐标系A和坐标系B之间的静态坐标系变换
    // 省略

    // 以geometry_msgs::PoseStamped为类型，发布位姿点M（在A坐标系下）的位姿话题信息
    geometry_msgs::PoseStamped M;
    M.header.frame_id = "A";
    M.pose.position.x = 1;
    M.pose.position.y = 1;
    M.pose.position.z = 1;
    M.pose.orientation.w = 1;

    ros::NodeHandle nh;
    ros::Publisher publisher_M = nh.advertise<geometry_msgs::PoseStamped>("M", 10);
    ros::Rate rate(10);
    while (ros::ok())
    {
        M.header.stamp = ros::Time::now();
        publisher_M.publish(M);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
```

![](images/2024-01-15-12-01-42-image.png)

第三题，以geometry_msgs::PoseStamped为类型，发布位姿点N（在B坐标系下）的位姿话题信息，使得点N与点M在空间中重合：

```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lesson");

    // 发布坐标系A和坐标系B之间的静态坐标系变换
    // 以geometry_msgs::PoseStamped为类型，发布位姿点M（在A坐标系下）的位姿话题信息
    // 省略

    // 以geometry_msgs::PoseStamped为类型，发布位姿点N（在B坐标系下）的位姿话题信息，使得点N与点M在空间中重合
    geometry_msgs::PoseStamped N;
    geometry_msgs::TransformStamped transform_inversed;
    N.header.frame_id = "B";
    tf2::fromMsg(transform_stamped.transform, transform);
    transform_inversed.transform = tf2::toMsg(transform.inverse());
    tf2::doTransform(M.pose, N.pose, transform_inversed);

    ros::NodeHandle nh;
    ros::Publisher publisher_M = nh.advertise<geometry_msgs::PoseStamped>("M", 10);
    ros::Publisher publisher_N = nh.advertise<geometry_msgs::PoseStamped>("N", 10);
    ros::Rate rate(10);
    while (ros::ok())
    {
        M.header.stamp = ros::Time::now();
        publisher_M.publish(M);
        N.header.stamp = ros::Time::now();
        publisher_N.publish(N);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
```

这里主要的关键点在坐标系转换，我课上会讲。

![](images/2024-01-15-12-04-55-image.png)

第四题的截图已经在各个步骤中给出。文档里只给出了大致的代码解释，详细解释我会在课上讲述。

## 自定义消息

ROS自带很多消息类型，但它们不一定可以满足我们的需要，此时就需要我们自定义一个消息类型了，接下来会简单教大家如何使用。

首先，新建一个名为lesson_msg的包，依赖为`roscpp std_msgs`，在其中新建msg文件夹，在文件夹中新建一个lesson.msg文件，输入以下内容：

```
int32 int32_msg
string string_msg
```

前者是消息类型，后者是消息的名字，消息类型支持嵌套，即可以使用ROS自带的部分消息类型。之后，在package.xml中添加以下两行：

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

CMakeLists.txt中也有不少需要修改的部分，详细如下：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation # 这一行需要添加
)
# 以下内容需要取消注释并按照需求修改
add_message_files(
  FILES
  lesson.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)
```

然后就可以编译了，编译完成后可以在`devel/include`下看到编译后的头文件，之后记得用`Ctrl+P`中的`>ROS: Update C++ Properties`更新依赖。

## 消息的发送

我们创建一个名为lesson_publisher的包，依赖为`roscpp std_msgs lesson_msg`，在src文件夹中创建lesson_publisher.cpp，并在CMakeLists.txt末尾新增以下两行：

```cmake
add_executable(lesson_publisher src/lesson_publisher.cpp)
target_link_libraries(lesson_publisher ${catkin_LIBRARIES})
```

然后编写lesson_publisher.cpp：

```cpp
#include <ros/ros.h>
#include "lesson_msg/lesson.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lesson_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<lesson_msg::lesson>("lesson_topic", 100);
    ros::Rate rate(10);
    unsigned int count = 0;
    while (ros::ok())
    {
        lesson_msg::lesson msg;
        msg.lesson_uint32 = count++;
        msg.lesson_string = "Hello, world! " + std::to_string(count);
        ROS_INFO("%s", msg.lesson_string.c_str());
        pub.publish(msg);
        ros::spinOnce;
        rate.sleep();
    }
    return 0;
}
```

用`Ctrl+Shift+B`中的`catkin_make: build`来编译项目。

用`Ctrl+P`中的`>ROS: Start`启动roscore，之后，使用`>Ros: Run`启动包。

```
zzh@Moon-R9000P:~/ros$ rosrun lesson_publisher lesson_publisher 
[ INFO] [1706430599.286841193]: Hello, world! 1
[ INFO] [1706430599.386918214]: Hello, world! 2
[ INFO] [1706430599.486943971]: Hello, world! 3
[ INFO] [1706430599.587006954]: Hello, world! 4
[ INFO] [1706430599.686953161]: Hello, world! 5
[ INFO] [1706430599.786933171]: Hello, world! 6
[ INFO] [1706430599.887183610]: Hello, world! 7
[ INFO] [1706430599.987185551]: Hello, world! 8
[ INFO] [1706430600.086967210]: Hello, world! 9
[ INFO] [1706430600.186933321]: Hello, world! 10
```

可以使用`rostopic list`查看所有正在发布的消息，在工作目录新建一个终端，使用`source devel/setup.bash`更新依赖，然后可以使用`rostopic echo /lesson_topic`查看消息：

```
zzh@Moon-R9000P:~/ros$ rostopic echo /lesson_topic
lesson_uint32: 2699
lesson_string: "Hello, world! 2700"
---
lesson_uint32: 2700
lesson_string: "Hello, world! 2701"
---
lesson_uint32: 2701
lesson_string: "Hello, world! 2702"
---
lesson_uint32: 2702
lesson_string: "Hello, world! 2703"
---
lesson_uint32: 2703
lesson_string: "Hello, world! 2704"
---
```

## 消息的接收

我们创建一个名为lesson_subscriber的包，依赖为`roscpp std_msgs lesson_msg`，在src文件夹中创建lesson_subscriber.cpp，并在CMakeLists.txt末尾新增以下两行：

```cmake
add_executable(lesson_subscriber src/lesson_subscriber.cpp)
target_link_libraries(lesson_subscriber ${catkin_LIBRARIES})
```

然后编写lesson_subscriber.cpp：

```cpp
#include <ros/ros.h>
#include "lesson_msg/lesson.h"

void topicCallback(lesson_msg::lesson msg)
{
    ROS_INFO("[%d] %s", msg.lesson_uint32, msg.lesson_string.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lesson_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("lesson_topic", 100, topicCallback);
    ros::spin();
    return 0;
}
```

用`Ctrl+Shift+B`中的`catkin_make: build`来编译项目。

用`Ctrl+P`中的`>ROS: Start`启动roscore，之后，使用`>Ros: Run`启动包。

左边是publisher，右边是subscriber，它们成功一起运行了。

![](images/2024-01-28-16-49-07.png)

这时使用`rqt_graph`来看一下消息图：

![](images/2024-01-28-16-52-38.png)
