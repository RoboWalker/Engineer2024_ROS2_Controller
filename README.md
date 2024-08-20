# RoboWalker 2024 工程机器人ROS2上位机

## 写在前面

RoboWalker战队的工程机器人在区域赛的时候取得了不错的战绩，从区域赛开始基本上稳定单场黄金矿工，并且获得了区域赛东部赛区场均经济最高、单场经济最高。在复活赛，各个强队的工程机器人水平迎头赶上，而我们的工程机器人也出现了一些机械、电路上的战损情况，导致机器人在赛场上的表现有所下滑，不过每场的表现也还算可以接受。现开源工程机器人上位机代码，希望和各队伍多多交流。

此外，工程机器人机械结构、自定义控制器机械结构、自定义控制器代码和工程机器人下位机代码也将陆续开源。

### 贡献者

上位机代码的贡献者包括：

| 姓名   | 介绍             | 贡献                                                      |
| ------ | ---------------- | --------------------------------------------------------- |
| 薛佳龙 | 2024赛季正式队员 | 完成整体框架的构建，完成仿真、FK/IK、有限状态机相关代码。 |
| 李辰阳 | 2024赛季正式队员 | 完成机械臂自动执行动作的相关代码，以及大部分后期调试。    |
| 唐煜程 | 2024赛季正式队员 | 调试与自定义控制器（图传串口）的通信接口。                |

## 安装

### ROS2

建议从官方安装或从“鱼香ROS”安装。

**建议使用 Ubuntu22.04 对应的ROS2版本，各个版本播放rosbag相关的api不太一样。当然，相关的代码你也可以自行进行修改。**

### Mujoco

使用mujoco作为仿真环境。

解压仓库中的`.mujoco.zip`文件，将解压出来的`.mujoco`文件夹放到`HOME`目录下面。

在`.bashrc`中加入以下两句话：

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin:$LD_LIBRARY_PATH
```

### lapack

IKFast依赖于lapack，直接使用apt安装即可

```bash
sudo apt install liblapack-dev
```

## 编译和运行

### 硬件部署

连接自定义控制器：

自定义控制器使用图传串口连接+usb转ttl，串口名称（设备描述文件）命名为ttyPL2302（之前用的调试线是pl2302，名字懒得改了，图传串口实际上用的是cp2102转换器），需要在`/etc/udev/rules.d`中创建规则文件。后期将改成图传串口。

连接工程机器人stm32下位机：

工程机器人的stm32使用板载USB与上位机连接，表现为USB虚拟串口的形式，串口名称（设备描述文件）命名为ttyEngineer。同样需要在`/etc/udev/rules.d`中创建规则文件。

### 编译

在ROS2的工作空间下使用colcon编译

```bash
colcon build
```

### 运行

首先按照ROS2的要求source

```bash
source install/local_setup.bash
```

如果要使用仿真环境，

```bash
ros2 launch engineer_arm_sim sim.launch.py
```

如果要在硬件上运行，

```bash
ros2 launch engineer_hw hw.launch.py
```

注意：即使在仿真环境下，你仍然需要连接自定义控制器。

### 自启动

将`autorun.sh`，`env_setup`放在家目录（注意修改文件里面的家目录，家目录必须用绝对路径，不能用~代替），设置`chmod +x`。

将`autorun_engineer.service` 放在`/etc/systemd/system`。

运行

```bash
sudo systemctl daemon-reload
sudo systemctl enable autorun_engineer.service
```

之后，可以进行启动和停止：

```bash
sudo systemctl start autorun_engineer.service #启动
sudo systemctl stop autorun_engineer.service #停止
```

下次启动计算机时，程序将会以systemd服务的形式自启动。

## 代码仓库介绍

代码仓库以ROS2为基础，包含多个ROS2功能包。下面一一进行介绍。

### engineer_msg

专门用来定义话题和服务文件。

| 类型 | 名称              | 介绍                         |
| ---- | ----------------- | ---------------------------- |
| 话题 | EndEffector.msg   | 机械臂末端的坐标和欧拉角     |
| 话题 | JointCommand.msg  | 对机械臂关节电机的命令       |
| 话题 | JointState.msg    | 机械臂关节电机的状态         |
| 话题 | SucketState.msg   | 吸盘的状态                   |
| 话题 | LiftState.msg     | 抬升机构升高/降低            |
| 话题 | RobotArmState.msg | 机械臂运行状态               |
| 服务 | FSMCommand.srv    | 对有限状态机的命令（输入）   |
| 服务 | FSMState.srv      | 请求获得有限状态机的当前状态 |

### customctrl_serial_communication

从自定义控制器接受数据，通过ROS2话题发送给机械臂控制器节点。

#### 通信协议

通信协议按照比赛官方图传串口的要求，进行如下设计：

```c++
uint8_t head = 0xA5;
uint16_t data_length = 30;
uint8_t seq; //序列号
uint8_t CRC8; //对前四个字节的CRC8校验
uint16_t cmd_id = 0x0302;
float x;		//自定义控制器的末端x
float y;		//自定义控制器的末端y
float z;		//自定义控制器的末端z
float roll;		//自定义控制器的末端roll
float pitch;	//自定义控制器的末端pitch
float yaw;		//自定义控制器的末端yaw
uint8_t buttons;        //自定义控制器按键 (bit0-->bit2)
    bool sucket;        //气泵状态
    bool lift_rise;     //机械臂升高
    bool lift_fall;     //机械臂降低
uint8_t reserved;
uint8_t reserved;
uint8_t reserved;
uint8_t reserved;
uint8_t reserved;
uint16_t CRC16;	//CRC16整包校验
```

#### 话题

| 名称                    | 类型                           | 发布/订阅 | 介绍                                                         |
| ----------------------- | ------------------------------ | --------- | ------------------------------------------------------------ |
| target_end_effector     | engineer_msg::msg::EndEffector | publisher | 机械臂的目标末端位置，包括xyz和rpy。注意，控制器节点不一定会服从。 |
| customctrl_sucket_state | engineer_msg::msg::SucketState | publisher | 自定义控制器发送的目标吸盘状态。注意，控制器节点不一定会服从。 |

### engineer_arm_hw

直接与工程机器人下位机通信的节点，通信方式为问答式，通信频率由下位机决定（暂定为50Hz）

#### 通信协议

通信协议暂定为

上位机 --> 下位机：

```c++
uint8_t head = 0xAB;
float joint1;		//角度，rad
float joint2;		
float joint3;		
float joint4;		
float joint5;	
float joint6;		
uint8_t buttons;        //自定义控制器按键 (bit0-->bit2)
    bool sucket;        //气泵状态
    bool lift_rise;     //机械臂升高
    bool lift_fall;     //机械臂降低
	bool auto_actions_prepare; //是否准备开始自动动作
	bool robotarm_ready; //机械臂是否准备好（是否已经到达了指定位置）
uint8_t mode;   //机械臂当前的控制模式（自定义控制器，自动取矿，小资源岛，.......）
uint32_t reserved;
uint16_t CRC16;	//CRC16整包校验。
```

下位机 --> 上位机：

```c++
uint8_t head = 0xAC;
float joint1;		//角度，rad
float joint2;		
float joint3;		
float joint4;		
float joint5;	
float joint6;		
uint8_t alive_status;	//机械臂是否存活（上电）
uint8_t auto_actions_request;   //机械臂自动动作请求
uint8_t lift_position;     //抬升实际角度
uint8_t reserved;
uint8_t reserved;
uint8_t reserved;
uint16_t CRC16;	//CRC16整包校验。
```

#### 话题

| 名称           | 类型                             | 发布/订阅  | 介绍                                                         |
| -------------- | -------------------------------- | ---------- | ------------------------------------------------------------ |
| joint_state    | engineer_msg::msg::JointState    | publisher  | 关节角度，rad。                                              |
| robotarm_state | engineer_msg::msg::RobotArmState | publisher  | 机械臂状态：下位机要求的自动动作码，是否到达指定初始位置，是否存活 |
| auto_command   | engineer_msg::msg::RobotArmState | subscriber | 需要执行的自动动作码                                         |
| joint_command  | engineer_msg::msg::JointCommand  | subscriber | 关节目标角度，rad                                            |
| sucket_command | engineer_msg::msg::SucketState   | subscriber | 吸盘目标状态                                                 |
| lift_command   | engineer_msg::msg::LiftState     | subscriber | 升降机构命令                                                 |

### engineer_arm_sim

机械臂仿真节点，基于Mujoco。

只进行运动学仿真（或者说，只进行动画显示）。发过来的角度指令会被立即执行，电机的速度可视为无穷大。

（写了PID和MIT控制电机的代码，仿真里面效果不好，放弃使用了x）

### engineer_arm_controller

代码中最核心的节点，进行整个机械臂的运动学控制和状态管理。

#### 话题

| 名称                    | 类型                            | 发布/订阅  | 介绍                                                         |
| ----------------------- | ------------------------------- | ---------- | ------------------------------------------------------------ |
| joint_state             | engineer_msg::msg::JointState   | subscriber | 当前关节角度，rad。                                          |
| target_end_effector     | engineer_msg::msg::EndEffector  | subscriber | 机械臂的目标末端位置，包括xyz和rpy。                         |
| customctrl_sucket_state | engineer_msg::msg::SucketState  | subscriber | 从自定义控制器获取的吸盘目标状态。仅在自定义控制器模式生效时才会执行。 |
| joint_command           | engineer_msg::msg::JointCommand | publisher  | 关节目标角度，rad                                            |
| sucket_command          | engineer_msg::msg::SucketState  | publisher  | 吸盘目标状态                                                 |
| now_end_effector        | engineer_msg::msg::EndEffector  | publisher  | 当前机械臂的末端位置                                         |

#### 服务

| 名称        | 类型                          | 介绍                           |
| ----------- | ----------------------------- | ------------------------------ |
| fsm_command | engineer_msg::srv::FSMCommand | 向有限状态机发送状态切换的命令 |
| fsm_state   | engineer_msg::srv::FSMState   | 从有限状态机获取当前的状态     |

#### FSM

FSM即有限状态机，用于管理机械臂控制器的状态。有限状态机接受指令（在功能包的`main.cpp`，编写以ROS2服务形式接受指令的回调函数），实现状态的切换。每一个状态包括四个函数：

`enter()`：在从其它状态切换到当前状态时，这个函数会被执行。该函数最好在50ms内运行完成。一般写进入该状态之前需要进行的预处理操作。

`run()`：当前状态的主函数。在当前状态生效时，该函数每20ms运行一次。所以，不要在该函数内进行较长的延时操作。例如，如果要编写自动取矿的代码，可以在`enter()`函数里面创建一个执行机械臂动作序列的线程，然后在`run()`函数里面通过轮询的方式等待动作序列完成，最后在`exit()`函数里清除这个线程。

`exit()`：离开当前状态时，这个函数会被执行。

`checkTransition(FSMCommand command)`：检查command，并返回在当前command下状态机将会进入的下一个状态。

如果要添加一个新的状态，你需要实现的内容：

1. 定义这个状态的类，从父类`FSMState`中继承
2. 完成`enter()`，`run()`，`exit()`，`checkTransition()`方法
3. 在`enumClass.hpp`定义状态的名称以及转换指令。
4. 完善FSM的`_stateList`，在FSM的构造函数中实例化这个新状态，并完善`getNextState`方法。
5. 修改其它所有状态的`checkTransition()`方法，定义其他状态是否可以转换到这个状态。
6. 在`main.cpp`中完善FSM相关的ROS2服务回调函数。

#### Kinematics_Controller

基于IKFast的机械臂解算器，包含正运动学解算（从当前关节角度获取当前末端位姿），逆运动学解算（从目标末端位姿获取目标关节角度）。

`ikfast_solver.cpp`：使用IKFast工具，通过urdf文件自动生成，是当前机械臂构型下的逆运动学库。代码非常多，但是是自动生成的，所以**不需要看懂，大概用法参阅Kinematics_Controller.cpp即可**。具体请参阅[运动学求解器IKFast](https://zhuanlan.zhihu.com/p/97364652)。IKFast生成器比较难配置，如果你想重新生成相关这个库文件，可以使用[小鱼（鱼香ROS）提供的Docker](https://zhuanlan.zhihu.com/p/567243492)。

`Kinematics_Controller.cpp`：解算器的具体实现。除IKFast外，正运动学通过ROS2自带的KDL库实现，姿态角的相关操作则通过Eigen实现。

`inverseKinematicsGround()`：抓取地面矿石的模式，根据控制器发来的末端目标欧拉角判断是否执行，一个简单的五自由度解算。

#### 自动动作

自动动作主要实现自动从矿仓取矿、放置和自动抓取小资源岛中的矿石等。

基本原理是事先录制rosbag，使用rosbag2_storage相关工具来解析，并按照动作执行的顺序，播放bag中的内容，可以根据实际情况播放到中途中断、或倍速播放等
