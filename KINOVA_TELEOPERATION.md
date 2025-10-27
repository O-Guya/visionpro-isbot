# Vision Pro to Kinova Teleoperation

使用Apple Vision Pro遥操作控制Kinova机械臂的系统。

## 功能特性

✅ **右手完全控制**
- 右手腕位置 → 机械臂末端位置（6自由度）
- 右手腕姿态 → 机械臂末端姿态
- 右手捏合 → 夹爪开合

✅ **左手辅助功能**
- 左手捏合：启用/暂停控制
- 左手张开：紧急停止
- 左手握拳：精细控制模式（慢速+小幅度）

✅ **实时性能**
- 100Hz控制频率
- gRPC低延迟数据传输
- PyBullet高性能物理仿真

---

## 系统架构

```
Vision Pro (ARKit)
        ↓ gRPC (100Hz)
VisionProStreamer
        ↓
SimpleMotionMapper
  - 位置映射
  - 姿态映射
  - 手势识别
        ↓
KinovaController
  - 逆运动学
  - 关节控制
        ↓
PyBullet仿真
```

---

## 安装步骤

### 1. 安装依赖

```bash
# 安装Kinova相关依赖
pip install -r requirements_kinova.txt

# 或手动安装
pip install pybullet scipy
```

### 2. 获取Kinova URDF模型（可选）

本系统可以使用任何机械臂URDF模型。如果没有Kinova URDF，系统会自动使用PyBullet内置的Kuka IIWA作为替代。

**选项A：使用Kinova Gen3 URDF**
```bash
# 下载Kinova Gen3 URDF（需要从Kinova官网或GitHub获取）
# 示例：https://github.com/Kinovarobotics/ros_kortex

# 将URDF文件放在项目目录
cp path/to/kinova_gen3.urdf ./
```

**选项B：使用默认机械臂**
```bash
# 不需要额外操作，系统会自动使用Kuka IIWA作为演示
```

---

## 使用方法

### 1. 启动Vision Pro端

在Vision Pro上运行Tracking Streamer应用，确保它正在广播数据。

### 2. 获取Vision Pro IP地址

在Vision Pro的设置中查看Wi-Fi连接信息，获取IP地址（例如：`10.31.181.201`）

### 3. 运行遥操作程序

```bash
python main_teleoperation.py --vp_ip <Vision_Pro_IP>
```

**示例：**
```bash
python main_teleoperation.py --vp_ip 10.31.181.201
```

### 4. 开始控制

1. **激活控制**：左手捏合（拇指和食指接触）
2. **移动机械臂**：移动右手，机械臂末端会跟随
3. **旋转末端**：旋转右手手腕，机械臂末端会同步旋转
4. **抓取物体**：右手捏合，夹爪闭合
5. **松开物体**：右手张开，夹爪打开
6. **精细模式**：左手握拳，进入慢速精细控制
7. **停止控制**：左手捏合，或左手完全张开（紧急停止）

---

## 命令行参数

```bash
python main_teleoperation.py [OPTIONS]

必需参数：
  --vp_ip IP                Vision Pro的IP地址

可选参数：
  --no_gui                  无GUI模式（更快，无可视化）
  --position_scale SCALE    位置缩放因子（默认：0.5）
                           例如：0.5表示手移动1m，机械臂移动0.5m
  --workspace_offset X Y Z  工作空间偏移量（默认：0.5 0.3 0.5）
  --kinova_urdf PATH        Kinova URDF文件路径（可选）

示例：
  # 基本使用
  python main_teleoperation.py --vp_ip 10.31.181.201

  # 更小的运动幅度（更精细）
  python main_teleoperation.py --vp_ip 10.31.181.201 --position_scale 0.3

  # 指定URDF文件
  python main_teleoperation.py --vp_ip 10.31.181.201 --kinova_urdf kinova_gen3.urdf

  # 无GUI模式（更快）
  python main_teleoperation.py --vp_ip 10.31.181.201 --no_gui
```

---

## 项目结构

```
visionpro-isbot/
├── kinova_is_sim/              # Kinova仿真模块（新增）
│   ├── __init__.py
│   ├── motion_mapper.py        # 运动映射器
│   ├── kinova_controller.py    # Kinova控制器
│   └── simulator.py            # PyBullet仿真器
├── avp_stream/                 # Vision Pro数据流（已有）
│   ├── streamer.py
│   └── grpc_msg/
├── main_teleoperation.py       # 主程序（新增）
├── requirements_kinova.txt     # 依赖文件（新增）
└── KINOVA_TELEOPERATION.md     # 本文档
```

---

## 技术细节

### 坐标系转换

- **Vision Pro**：YUP坐标系（Y轴向上）
- **机械臂**：ZUP坐标系（Z轴向上）
- 转换矩阵已在`avp_stream/streamer.py`中实现

### 位置映射

```python
# 右手腕位置
right_wrist_pos = vp_data['right_wrist'][0, :3, 3]

# 应用缩放和偏移
target_position = right_wrist_pos * position_scale + workspace_offset
```

### 姿态映射

```python
# 右手腕旋转矩阵（3x3）
right_wrist_rotation = vp_data['right_wrist'][0, :3, :3]

# 转换为四元数
target_orientation = rotation_matrix_to_quaternion(right_wrist_rotation)
```

### 夹爪映射

```python
# 右手捏合距离
pinch_distance = vp_data['right_pinch_distance']

# 映射到夹爪开合度（0=闭合，1=张开）
if pinch_distance < 0.02:    # 2cm
    gripper = 0.0  # 完全闭合
elif pinch_distance > 0.05:  # 5cm
    gripper = 1.0  # 完全张开
else:
    gripper = (pinch_distance - 0.02) / 0.03  # 线性插值
```

---

## 故障排查

### 问题1：无法连接到Vision Pro

**症状**：`Failed to connect to Vision Pro`

**解决方案**：
1. 检查Vision Pro和电脑是否在同一Wi-Fi网络
2. 确认Vision Pro的Tracking Streamer应用正在运行
3. 验证IP地址是否正确
4. 尝试ping Vision Pro：`ping <IP地址>`

### 问题2：PyBullet无法加载Kinova URDF

**症状**：`WARNING: Could not find Kinova URDF file!`

**解决方案**：
1. 这是正常的，系统会自动使用Kuka IIWA作为替代
2. 如果需要Kinova模型，下载URDF并通过`--kinova_urdf`参数指定
3. 或将URDF文件命名为`kinova_gen3.urdf`并放在项目根目录

### 问题3：机械臂运动不流畅

**解决方案**：
1. 降低`position_scale`参数（例如0.3或0.2）
2. 使用左手握拳进入精细控制模式
3. 确保网络延迟低（<50ms）

### 问题4：逆运动学失败

**症状**：`Warning: IK failed for target pose`

**解决方案**：
1. 目标位置超出机械臂工作空间
2. 调整`workspace_offset`参数
3. 减小`position_scale`
4. 避免让机械臂达到奇异位姿

---

## 性能优化

### 提升控制频率

```bash
# 使用无GUI模式
python main_teleoperation.py --vp_ip <IP> --no_gui
```

### 降低网络延迟

1. 使用5GHz Wi-Fi（而非2.4GHz）
2. 减少网络中其他设备流量
3. 确保Vision Pro和电脑之间无障碍物

### 调整控制参数

```python
# 在motion_mapper.py中调整
position_scale = 0.3        # 更小的缩放 = 更精细控制
workspace_offset = [0.5, 0.3, 0.5]  # 调整工作空间中心
```

---

## 下一步开发

### 短期改进
- [ ] 添加数据记录功能（轨迹示教）
- [ ] 实现平滑滤波（卡尔曼滤波）
- [ ] 添加工作空间可视化边界
- [ ] 实现多种控制模式切换

### 中期改进
- [ ] 添加力反馈支持
- [ ] 实现双臂协作控制
- [ ] 集成真实Kinova机械臂SDK
- [ ] 添加碰撞检测

### 长期愿景
- [ ] VR可视化（在Vision Pro中显示机械臂状态）
- [ ] 机器学习模仿学习
- [ ] 多用户协作遥操作

---

## 常见问题 (FAQ)

**Q: 需要真实的Kinova机械臂吗？**
A: 不需要。本系统使用PyBullet仿真，可以在没有真实机械臂的情况下测试和开发。

**Q: 可以使用其他机械臂吗？**
A: 可以。只需提供URDF文件，系统支持任何机械臂模型。

**Q: 控制延迟是多少？**
A: 在良好的Wi-Fi环境下，端到端延迟约50-100ms。

**Q: 如何提高控制精度？**
A: 使用左手握拳进入精细模式，或降低`position_scale`参数。

**Q: 支持力反馈吗？**
A: 目前不支持。这是未来的开发方向。

---

## 贡献

欢迎提交Issue和Pull Request！

---

## 许可证

与主项目相同。

---

## 联系方式

如有问题，请在GitHub上提Issue。
