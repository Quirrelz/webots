# 基于 Webots 的视觉定点搬运系统

本项目基于 Webots 仿真平台，设计并实现了一套视觉定点搬运系统。系统通过摄像头识别不同颜色的目标物体，利用六自由度机械臂完成抓取与放置，并结合移动机器人路径规划，实现目标物体在不同工位之间的自动运输。

项目综合使用了 Webots 仿真、Python 控制器、OpenCV 图像处理、A* 路径规划、多机器人通信调度等技术，可用于机器人学课程设计、移动机器人路径规划实验和机械臂视觉抓取仿真学习。

## 项目功能

- Webots 机器人仿真场景搭建
- 红、绿、蓝三类目标物体识别
- 基于 HSV 阈值分割的视觉检测
- 六自由度机械臂抓取与放置控制
- ROSbot XL 移动机器人定点运输
- 基于栅格地图的 A* 路径规划
- 多辆小车与多台机械臂协同调度
- JSON 配置文件管理任务参数
- A* 路径规划结果可视化

## 项目结构

```text
机械臂视觉搬运
├── ReadMe.md
├── visualize_astar_paths.py
├── maps
│   └── obstacle_grid.json
├── controllers
│   ├── task_config.json
│   ├── car_base.py
│   ├── msg_manager
│   │   └── msg_manager.py
│   ├── start_arm
│   │   └── start_arm.py
│   ├── start_camera
│   │   └── start_camera.py
│   ├── red_car
│   │   └── red_car.py
│   ├── green_car
│   │   └── green_car.py
│   ├── blue_car
│   │   └── blue_car.py
│   ├── red_target_arm
│   │   └── red_target_arm.py
│   ├── green_target_arm
│   │   └── green_target_arm.py
│   ├── blue_target_arm
│   │   └── blue_target_arm.py
│   ├── red_target_camera
│   │   └── red_target_camera.py
│   ├── green_target_camera
│   │   └── green_target_camera.py
│   └── blue_target_camera
│       └── blue_target_camera.py
├── worlds
│   ├── 05六自由度机械臂视觉小场景搬运-气动.wbt
│   └── 06六自由度机械臂视觉小场景搬运-夹爪.wbt
├── protos
│   └── robodyno_webots
└── 标定表
    └── 07六自由度机械臂视觉场景搬运-标定表.xlsx
```

## 系统架构

系统采用模块化设计，主要包括仿真环境模块、视觉识别模块、机械臂控制模块、移动机器人控制模块、路径规划模块和任务调度模块。

### 1. 仿真环境模块

通过 Webots 世界文件构建仿真场景，包含移动小车、六自由度机械臂、摄像头、障碍物、目标物体和放置区域等对象。

主要目录：

```text
worlds
protos
```

### 2. 视觉识别模块

摄像头控制器使用 OpenCV 对图像进行处理，通过 HSV 颜色阈值识别红色、绿色和蓝色目标物体，并提取目标中心点坐标。

主要文件：

```text
controllers/start_camera/start_camera.py
controllers/red_target_camera/red_target_camera.py
controllers/green_target_camera/green_target_camera.py
controllers/blue_target_camera/blue_target_camera.py
```

### 3. 机械臂控制模块

机械臂控制器根据视觉识别结果，通过手眼标定关系将图像坐标转换为机械臂坐标，并控制六自由度机械臂完成抓取和放置。

主要文件：

```text
controllers/start_arm/start_arm.py
controllers/red_target_arm/red_target_arm.py
controllers/green_target_arm/green_target_arm.py
controllers/blue_target_arm/blue_target_arm.py
```

### 4. 移动机器人控制模块

三辆移动小车共用基础控制类 `BaseCarController`，实现位置获取、路径规划、路径跟踪、姿态调整和任务反馈。

主要文件：

```text
controllers/car_base.py
controllers/red_car/red_car.py
controllers/green_car/green_car.py
controllers/blue_car/blue_car.py
```

### 5. 路径规划模块

系统使用栅格地图描述障碍物环境，并采用 A* 算法为小车规划从起点到目标点的路径。

地图文件：

```text
maps/obstacle_grid.json
```

A* 算法使用欧几里得距离作为启发式函数：

```python
def heuristic(self, a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])
```

### 6. 任务调度模块

消息管理器负责读取任务配置文件，并通过 Webots 的 Emitter 和 Receiver 设备向小车和机械臂发送任务指令，实现多机器人协同工作。

主要文件：

```text
controllers/msg_manager/msg_manager.py
controllers/task_config.json
```

## 任务流程

系统运行后，整体任务流程如下：

1. 各控制器初始化；
2. 消息管理器检测小车和目标机械臂是否准备完成；
3. 按照配置文件中的红、绿、蓝顺序启动搬运任务；
4. 起始机械臂识别并抓取指定颜色物体；
5. 小车接收目标点位并进行 A* 路径规划；
6. 小车沿规划路径移动到目标工位；
7. 目标端摄像头识别物体位置；
8. 目标端机械臂完成抓取和放置；
9. 小车返回指定位置；
10. 所有任务完成后，系统输出完成状态。

## 运行环境

建议使用以下环境运行项目：

- Webots R2023b 或更高版本
- Python 3.x
- Robodyno 1.7.1 或更高版本
- OpenCV
- NumPy
- Matplotlib，可选，用于保存路径规划图片
- Tkinter，可选，用于 Matplotlib 不可用时显示路径图

## 安装依赖

可根据实际 Python 环境安装依赖：

```bash
pip install robodyno opencv-python numpy matplotlib
```

如果使用 Anaconda，也可以执行：

```bash
conda install numpy matplotlib
pip install opencv-python robodyno
```

## Webots 场景运行方法

1. 打开 Webots；
2. 选择 `File -> Open World`；
3. 打开项目中的世界文件：

```text
worlds/06六自由度机械臂视觉小场景搬运-夹爪.wbt
```

4. 启动仿真运行；
5. 系统会自动加载各机器人控制器并执行搬运任务。

## A* 路径规划可视化

项目提供了路径规划可视化脚本：

```text
visualize_astar_paths.py
```

运行方式：

```bash
python visualize_astar_paths.py
```

在 Windows 中可使用完整路径运行：

```powershell
F:\Anaconda\python.exe "F:\Chrome Download\07Webots的视觉定点搬运\机械臂视觉搬运\visualize_astar_paths.py"
```

如果 Matplotlib 可用，可以保存图片：

```bash
python visualize_astar_paths.py --save astar_paths.png
```

该脚本会读取 `maps/obstacle_grid.json` 和 `controllers/task_config.json`，绘制红、绿、蓝三辆小车对应目标点的 A* 原始规划路径。若 Matplotlib 导入失败，脚本会自动切换到 Tkinter 窗口显示模式。

## 配置文件说明

任务配置文件位于：

```text
controllers/task_config.json
```

其中主要配置内容包括：

- 任务执行顺序
- 小车编号
- 目标机械臂编号
- 目标颜色
- 小车目标位姿
- 小车返回位姿
- 视觉稳定等待时间
- 连续未检测次数阈值
- 下一辆车启动延时

通过修改该文件，可以调整系统任务执行顺序和目标点位置。

## 项目特点

- 使用 Webots 构建完整机器人仿真环境
- 支持多小车、多机械臂协同工作
- 采用模块化控制器设计，结构清晰
- 使用 JSON 消息实现多控制器通信
- 使用 A* 算法完成障碍物环境下路径规划
- 具备视觉识别、抓取、运输和放置完整闭环

## 当前不足

- 视觉识别主要依赖 HSV 阈值，对光照变化较敏感；
- 手眼标定采用线性映射方式，精度仍有提升空间；
- A* 规划路径存在一定锯齿现象，路径平滑性不足；
- 小车接近目标点时可能出现短时间抖动；
- 当前路径规划主要面向静态障碍物，尚未加入动态避障机制。

## 后续改进方向

- 引入路径平滑算法，减少 A* 路径锯齿；
- 优化小车终点控制策略，降低到达目标点时的抖动；
- 使用更规范的手眼标定方法提高抓取精度；
- 引入深度学习目标检测算法，提高视觉识别鲁棒性；
- 增加动态避障功能；
- 增加可视化监控界面，实时显示任务状态、小车位置和识别结果。

## 参考资料

[1] Robodyno. Robodyno开发者文档[EB/OL]. [2026-06-19]. http://101.42.250.169/1.7.3/.

[2] 陈白帆, 宋德臻. 移动机器人[M]. 北京: 清华大学出版社, 2021.

[3] Robotway. Webots仿真设计教程[EB/OL]. [2026-06-19]. https://www.robotway.com/col.jsp?id=183&.

[4] 蔡自兴, 谢斌. 机器人学[M]. 4版. 北京: 清华大学出版社, 2022.

[5] Cyberbotics. Webots Cloud[EB/OL]. [2026-06-19]. https://webots.cloud/.
#   w e b o t s  
 