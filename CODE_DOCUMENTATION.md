# 代码功能文档

本文档简要说明每个代码文件和函数的关键功能、输入输出。

---

## 📁 kinova_is_sim/motion_mapper.py

**文件功能**：将Vision Pro手部追踪数据映射为Kinova机械臂控制指令

### `SimpleMotionMapper` 类

**关键作用**：实现Vision Pro到机械臂的运动映射和手势识别

#### `__init__(position_scale=2.5, workspace_offset=None, use_head_relative=True)`
- **功能**：初始化运动映射器
- **输入**：
  - `position_scale`: 位置缩放系数（默认2.5，值越大机械臂移动越多）
  - `workspace_offset`: 机械臂工作空间中心偏移 [x,y,z]
  - `use_head_relative`: 是否使用头部相对控制模式
- **输出**：无
- **关键作用**：设置控制参数和初始化状态变量

#### `detect_fist(finger_matrices)`
- **功能**：检测手是否握拳
- **输入**：`finger_matrices` - 手指关节变换矩阵 (25, 4, 4)
- **输出**：`bool` - True表示握拳
- **关键作用**：通过计算指尖到手腕的平均距离判断握拳（<10cm为握拳）

#### `detect_open_hand(finger_matrices)`
- **功能**：检测手是否完全张开（紧急停止手势）
- **输入**：`finger_matrices` - 手指关节变换矩阵 (25, 4, 4)
- **输出**：`bool` - True表示张开
- **关键作用**：通过计算指尖到手腕的平均距离判断张开（>15cm为张开）

#### `rotation_matrix_to_quaternion(rotation_matrix)`
- **功能**：将旋转矩阵转换为四元数
- **输入**：`rotation_matrix` - 3x3旋转矩阵
- **输出**：`np.array` - 四元数 [x, y, z, w]
- **关键作用**：用于将手部姿态转换为机械臂可接受的四元数格式

#### `rotation_matrix_to_euler(rotation_matrix)`
- **功能**：将旋转矩阵转换为欧拉角
- **输入**：`rotation_matrix` - 3x3旋转矩阵
- **输出**：`np.array` - 欧拉角 [roll, pitch, yaw] (弧度)
- **关键作用**：备用的姿态表示方法

#### `map_to_robot(vp_data)`
- **功能**：核心映射函数，将Vision Pro数据转换为机械臂指令
- **输入**：`vp_data` - 字典，包含：
  - `'head'`: 头部变换矩阵 (1, 4, 4)
  - `'right_wrist'`: 右手腕变换矩阵 (1, 4, 4)
  - `'left_wrist'`: 左手腕变换矩阵 (1, 4, 4)
  - `'right_fingers'`: 右手指关节矩阵 (25, 4, 4)
  - `'left_fingers'`: 左手指关节矩阵 (25, 4, 4)
  - `'right_pinch_distance'`: 右手捏合距离 (float)
  - `'left_pinch_distance'`: 左手捏合距离 (float)
- **输出**：
  - `None` - 控制未激活
  - `{'emergency_stop': True}` - 紧急停止
  - `dict` - 机械臂指令：
    - `'position'`: [x, y, z] 目标位置
    - `'orientation'`: [x, y, z, w] 四元数姿态
    - `'gripper'`: 0.0-1.0 夹爪开度
    - `'speed_scale'`: 速度系数
- **关键作用**：
  1. 检测左手手势（捏合=启停，张开=紧急停止，握拳=精细模式）
  2. 计算右手相对于头部的位置（头部相对模式）或绝对位置
  3. 应用缩放系数将手部运动映射到机械臂运动
  4. 提取右手姿态并转换为四元数
  5. 根据右手捏合距离计算夹爪开度

---

## 📁 kinova_is_sim/kinova_controller.py

**文件功能**：使用PyBullet控制Kinova机械臂，包含逆运动学求解和关节控制

### `KinovaController` 类

**关键作用**：封装机械臂控制逻辑，提供高层接口

#### `__init__(robot_id, end_effector_link_index, num_joints=7)`
- **功能**：初始化Kinova控制器
- **输入**：
  - `robot_id`: PyBullet机器人ID
  - `end_effector_link_index`: 末端执行器link索引
  - `num_joints`: 可控关节数量（默认7）
- **输出**：无
- **关键作用**：获取关节信息、关节限制、夹爪关节索引

#### `move_to_pose(position, orientation, speed_scale=1.0)`
- **功能**：移动末端执行器到目标位姿
- **输入**：
  - `position`: [x, y, z] 目标位置
  - `orientation`: [x, y, z, w] 四元数姿态
  - `speed_scale`: 速度缩放系数 (0.0-1.0)
- **输出**：`bool` - True表示IK求解成功
- **关键作用**：
  1. 调用PyBullet的IK求解器计算关节角度
  2. 发送关节位置控制指令
  3. 应用速度限制

#### `control_gripper(gripper_command)`
- **功能**：控制夹爪开合
- **输入**：`gripper_command` - 0.0（闭合）到1.0（张开）
- **输出**：无
- **关键作用**：将0-1的开度映射到夹爪关节的实际角度范围

#### `get_end_effector_pose()`
- **功能**：获取当前末端执行器位姿
- **输入**：无
- **输出**：`(position, orientation)` - 位置和四元数姿态
- **关键作用**：读取仿真中末端执行器的实际状态

#### `get_joint_states()`
- **功能**：获取当前关节状态
- **输入**：无
- **输出**：`list` - 关节位置列表
- **关键作用**：读取所有可控关节的当前角度

#### `reset_to_home()`
- **功能**：将机械臂重置到零位
- **输入**：无
- **输出**：无
- **关键作用**：设置所有关节角度为0

#### `stop()`
- **功能**：紧急停止，保持当前位置
- **输入**：无
- **输出**：无
- **关键作用**：设置最大速度为0，锁定当前关节角度

---

## 📁 kinova_is_sim/simulator.py

**文件功能**：使用PyBullet创建和管理机械臂仿真环境

### `KinovaSimulator` 类

**关键作用**：管理仿真环境、加载机器人模型、提供可视化

#### `__init__(gui=True, kinova_urdf_path=None)`
- **功能**：初始化PyBullet仿真器
- **输入**：
  - `gui`: True=GUI模式，False=无头模式
  - `kinova_urdf_path`: Kinova URDF文件路径（可选）
- **输出**：无
- **关键作用**：
  1. 启动PyBullet物理引擎
  2. 设置重力和时间步长
  3. 加载地面和机器人模型
  4. 创建夹爪可视化

#### `_load_kinova()`
- **功能**：加载Kinova机器人URDF模型
- **输入**：无
- **输出**：无
- **关键作用**：
  1. 尝试多个路径查找Kinova URDF
  2. 如果找不到，加载Kuka IIWA作为替代
  3. 自动识别末端执行器link

#### `_find_end_effector_link()`
- **功能**：自动查找末端执行器link索引
- **输入**：无
- **输出**：无
- **关键作用**：通过link名称关键词（ee、gripper、tool等）识别末端执行器

#### `_print_robot_info()`
- **功能**：打印机器人关节信息
- **输入**：无
- **输出**：无（打印到控制台）
- **关键作用**：调试辅助，显示所有关节的类型、名称、限制

#### `step()`
- **功能**：推进仿真一步
- **输入**：无
- **输出**：无
- **关键作用**：执行一次物理计算步（1/240秒）

#### `reset()`
- **功能**：重置仿真
- **输入**：无
- **输出**：无
- **关键作用**：清除所有物体，重新加载环境

#### `close()`
- **功能**：关闭仿真器
- **输入**：无
- **输出**：无
- **关键作用**：断开PyBullet连接，释放资源

#### `add_debug_axes(position, orientation, length=0.1)`
- **功能**：在指定位姿绘制坐标轴
- **输入**：
  - `position`: [x, y, z] 位置
  - `orientation`: [x, y, z, w] 四元数
  - `length`: 坐标轴长度
- **输出**：无
- **关键作用**：可视化调试工具，显示目标位姿

#### `add_debug_sphere(position, radius=0.02, color=[1,0,0], lifetime=0.1)`
- **功能**：在指定位置添加调试球体
- **输入**：
  - `position`: [x, y, z]
  - `radius`: 半径
  - `color`: RGB颜色
  - `lifetime`: 持续时间（秒）
- **输出**：`body_id` - 创建的物体ID
- **关键作用**：可视化调试工具，标记空间点

#### `_create_gripper_visualization()`
- **功能**：创建夹爪的可视化表示
- **输入**：无
- **输出**：无
- **关键作用**：
  1. 创建两个蓝色半透明盒子代表夹爪指
  2. 存储可视化物体ID

#### `update_gripper_visualization(gripper_state=0.5)`
- **功能**：更新夹爪可视化的位置和开度
- **输入**：`gripper_state` - 0.0（闭合）到1.0（张开）
- **输出**：无
- **关键作用**：
  1. 获取末端执行器当前位姿
  2. 根据夹爪开度计算两个指的位置
  3. 更新可视化物体的位置和姿态

#### `get_robot_id()`
- **功能**：获取机器人ID
- **输入**：无
- **输出**：`int` - PyBullet机器人ID
- **关键作用**：供控制器使用

#### `get_end_effector_link_index()`
- **功能**：获取末端执行器link索引
- **输入**：无
- **输出**：`int` - link索引
- **关键作用**：供控制器使用

---

## 📁 main_teleoperation.py

**文件功能**：主遥操作程序，连接Vision Pro数据流和机械臂控制

### `main()` 函数

**关键作用**：整合所有模块，实现完整的遥操作流程

**流程**：

1. **参数解析**：
   - `--vp_ip`: Vision Pro IP地址（必需）
   - `--position_scale`: 位置缩放系数（默认2.5）
   - `--workspace_offset`: 工作空间偏移（默认[0.5, 0.3, 0.5]）
   - `--control_freq`: 控制频率Hz（默认100）
   - `--no_head_relative`: 禁用头部相对模式
   - `--kinova_urdf`: Kinova URDF路径（可选）
   - `--no_gui`: 无GUI模式

2. **初始化Vision Pro连接**：
   - 创建`VisionProStreamer`对象
   - 等待数据流就绪（最多5秒）
   - 验证数据完整性

3. **初始化仿真环境**：
   - 创建`KinovaSimulator`
   - 获取机器人ID和末端执行器索引

4. **初始化控制器和映射器**：
   - 创建`KinovaController`
   - 创建`SimpleMotionMapper`（配置缩放、偏移、头部相对模式）

5. **主控制循环**（运行在指定频率）：
   - 获取Vision Pro最新数据
   - 调用映射器生成机械臂指令
   - 处理紧急停止
   - 执行位姿控制（IK求解）
   - 控制夹爪
   - 更新夹爪可视化
   - 绘制调试坐标轴
   - 推进仿真
   - 保持循环频率

6. **清理**：
   - Ctrl+C退出
   - 关闭仿真器

**关键输入**：Vision Pro数据流（gRPC）

**关键输出**：机械臂关节控制指令（PyBullet）

---

## 📁 test_kinova_sim.py

**文件功能**：独立测试脚本，无需Vision Pro即可测试仿真和控制

### `generate_circular_trajectory(center, radius, num_points=100)`
- **功能**：生成圆形轨迹点
- **输入**：
  - `center`: [x, y, z] 圆心
  - `radius`: 半径
  - `num_points`: 轨迹点数量
- **输出**：`list` - 轨迹点列表
- **关键作用**：用于测试机械臂跟踪轨迹的能力

### `main()` 函数

**关键作用**：执行5个测试用例

**测试用例**：

1. **测试1：回零位**
   - 调用`reset_to_home()`
   - 等待1秒稳定

2. **测试2：移动到目标位置**
   - 目标位置：[0.5, 0.3, 0.5]
   - 目标姿态：[0, 1, 0, 0]（朝下）
   - 显示位置误差

3. **测试3：圆形轨迹跟踪**
   - 圆心：[0.5, 0.3, 0.5]
   - 半径：0.1m
   - 200个点
   - 测试连续运动能力

4. **测试4：夹爪控制**
   - 闭合（gripper=0.0）
   - 等待0.5秒
   - 张开（gripper=1.0）

5. **测试5：不同姿态**
   - 朝下：[0, 1, 0, 0]
   - 倾斜：[0, 0, 1, 0]
   - 侧向：[0.707, 0, 0.707, 0]

**输入**：无需外部输入

**输出**：控制台输出测试结果和进度

---

## 数据流图

```
Vision Pro (ARKit)
      ↓ gRPC (100Hz)
VisionProStreamer (avp_stream)
      ↓ vp_data (dict)
SimpleMotionMapper
  ├─ 手势识别
  ├─ 头部相对计算
  ├─ 位置缩放
  └─ 姿态转换
      ↓ robot_command (dict)
KinovaController
  ├─ 逆运动学求解
  ├─ 关节控制
  └─ 夹爪控制
      ↓ joint_commands
PyBullet仿真
  ├─ 物理计算
  ├─ 机械臂可视化
  └─ 夹爪可视化
```

---

## 关键参数说明

### 位置缩放 (position_scale)
- **默认值**：2.5
- **含义**：手部移动距离 × position_scale = 机械臂移动距离
- **示例**：
  - `position_scale=2.5`: 手移动20cm → 臂移动50cm
  - `position_scale=5.0`: 手移动10cm → 臂移动50cm（更灵敏）
  - `position_scale=1.0`: 手移动50cm → 臂移动50cm（1:1映射）

### 工作空间偏移 (workspace_offset)
- **默认值**：[0.5, 0.3, 0.5] (米)
- **含义**：机械臂工作空间的中心点
- **作用**：将手部相对坐标转换到机械臂绝对坐标

### 控制频率 (control_freq)
- **默认值**：100 Hz
- **范围**：10-200 Hz
- **建议**：
  - 实时性要求高：100-200 Hz
  - 网络延迟大：50-100 Hz
  - 无GUI模式：可以更高

### 头部相对模式 (use_head_relative)
- **默认值**：True（启用）
- **启用时**：手部位置相对于头部计算，用户可以在任何位置站立
- **禁用时**：手部绝对位置，用户需要站在固定位置

---

## 常见问题

### Q1: 如何调整机械臂的灵敏度？
**A**: 修改`--position_scale`参数。值越大，手部小动作产生更大的机械臂移动。

### Q2: 如何让机械臂移动到不同的工作区域？
**A**: 调整`--workspace_offset`参数，例如`--workspace_offset 0.6 0.4 0.6`。

### Q3: 控制循环频率受什么限制？
**A**:
- Vision Pro数据流：90-100 Hz
- 网络延迟：通常20-100ms
- PyBullet仿真：240 Hz物理步
- 建议控制频率≤100 Hz

### Q4: 头部相对模式的优势是什么？
**A**:
- 用户可以自由移动站位
- 不需要精确的空间标定
- 更符合直觉的控制方式

---

## 扩展开发指南

### 添加新的手势识别
1. 在`SimpleMotionMapper`中添加检测函数
2. 在`map_to_robot()`中调用检测函数
3. 根据手势修改输出指令

### 添加新的控制模式
1. 在`SimpleMotionMapper.__init__()`中添加模式标志
2. 在`map_to_robot()`中添加模式切换逻辑
3. 在`main_teleoperation.py`中添加命令行参数

### 集成真实Kinova机械臂
1. 替换`KinovaController`中的PyBullet调用为Kinova SDK调用
2. 实现相同的接口：`move_to_pose()`, `control_gripper()`
3. 添加安全限制和错误处理

---

**文档版本**: 1.0
**最后更新**: 2024
