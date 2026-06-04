# 功能包: behavior_selector

## 1. 功能概述

*   **一句话总结**: 基于RQT的图形界面行为选择器，为多智能体飞行系统提供任务模式切换的GUI控制面板
*   **核心节点**: `behavior_selector_node.py`
*   **算法类型**: 基于有限状态机的行为选择器，通过ROS服务实现全局飞行模式的切换和管理

## 2. 依赖关系

*   **主要依赖项**:
    *   `rospy`: Python客户端库，用于ROS节点通信
    *   `rqt_gui`: RQT图形界面框架核心库
    *   `rqt_gui_py`: RQT Python插件支持库
    *   `snapstack_msgs`: 包含QuadFlightMode等自定义消息类型
    *   `message_generation`: 用于生成自定义服务消息
    *   `std_msgs`: 标准ROS消息类型
    *   `python_qt_binding`: Qt图形界面Python绑定

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

此功能包不订阅任何话题，主要通过GUI交互和服务调用工作。

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `/globalflightmode` | `snapstack_msgs/QuadFlightMode` | 事件驱动 | 发布全局飞行模式状态，支持GO、LAND、KILL三种模式 |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: 无
*   **发布的TF**: 无

### 3.4 提供的服务 (Services)

| 服务名称 | 服务类型 | 描述 |
| :--- | :--- | :--- |
| `/change_mode` | `behavior_selector/MissionModeChange` | 接收模式切换请求，支持START(1)、END(2)、KILL(3)三种操作 |

### 3.5 自定义消息和服务

**服务类型: MissionModeChange**
```
# 请求 (Request)
uint8 mode          # 模式类型
uint8 START  = 1    # 开始任务
uint8 END    = 2    # 结束任务
uint8 KILL   = 3    # 紧急停止
---
# 响应 (Response)
bool RECEIVED       # 确认接收
```

## 4. 核心算法原理

### 4.1 有限状态机设计

behavior_selector采用了简单而有效的有限状态机(FSM)模式来管理飞行器的行为状态。该系统包含两个主要状态：

- **NOT_FLYING (0)**: 非飞行状态
- **FLYING (1)**: 飞行状态

### 4.2 状态转换逻辑

系统的状态转换遵循以下规则：

#### 状态转换图
```
[NOT_FLYING] --START--> [FLYING]
[FLYING] ----END-----> [NOT_FLYING] (通过LAND模式)
[ANY_STATE] --KILL---> [NOT_FLYING] (紧急停止)
```

#### 具体转换逻辑

1. **启动流程 (START)**:
   ```python
   if req.mode == req.START:
       self.status = FLYING
       self.flightevent.mode = QuadFlightMode.GO
       self.sendEvent()
   ```

2. **正常结束流程 (END)**:
   ```python
   if req.mode == req.END and self.status == FLYING:
       self.flightevent.mode = QuadFlightMode.LAND
       self.sendEvent()
   ```

3. **紧急停止流程 (KILL)**:
   ```python
   if req.mode == req.KILL:
       self.status = NOT_FLYING
       self.flightevent.mode = QuadFlightMode.KILL
       self.sendEvent()
   ```

### 4.3 安全机制

- **状态保护**: END命令只有在FLYING状态下才会执行，避免重复降落
- **紧急处理**: KILL命令可以在任何状态下执行，提供紧急停止能力
- **消息时间戳**: 每次发布消息时都会更新时间戳，确保消息时效性

### 4.4 GUI交互机制

RQT插件通过以下机制与后端节点通信：

1. **按钮映射**: 三个GUI按钮分别映射到START、END、KILL操作
2. **服务调用**: 按钮按下时通过ROS服务异步调用后端节点
3. **视觉反馈**: 按钮采用颜色编码（绿色START、蓝色END、红色E-STOP）

## 5. 使用与配置

### 5.1 启动示例

```bash
# 启动完整的GUI界面，包括行为选择器和后端服务
roslaunch behavior_selector gui.launch
```

### 5.2 单独启动后端服务

```bash
# 仅启动行为选择服务节点
rosrun behavior_selector behavior_selector_node.py
```

### 5.3 手动模式切换

```bash
# 使用命令行调用服务进行模式切换
rosservice call /change_mode "mode: 1"  # 启动
rosservice call /change_mode "mode: 2"  # 结束
rosservice call /change_mode "mode: 3"  # 紧急停止
```

### 5.4 监控飞行模式

```bash
# 监控全局飞行模式发布
rostopic echo /globalflightmode
```

## 6. 文件结构分析

### 6.1 核心文件

- **scripts/behavior_selector_node.py**: 主要后端服务节点，实现状态机逻辑
- **src/rqt_pkg/button_module.py**: RQT插件实现，提供图形界面
- **resource/MissionModePlugin.ui**: Qt Designer设计的用户界面文件
- **srv/MissionModeChange.srv**: 自定义服务消息定义

### 6.2 配置文件

- **launch/gui.launch**: 启动文件，同时启动RQT界面和后端服务
- **plugin.xml**: RQT插件注册配置文件
- **cfg/default.perspective**: RQT界面布局配置文件

### 6.3 构建配置

- **package.xml**: ROS功能包描述文件
- **CMakeLists.txt**: 构建配置文件
- **setup.py**: Python包安装配置

## 7. 设计特点

### 7.1 模块化设计

- **前后端分离**: GUI插件和服务节点分离，支持独立部署
- **标准接口**: 遵循ROS标准，易于集成到现有系统
- **可扩展性**: 支持添加新的飞行模式和状态

### 7.2 安全性考虑

- **状态验证**: 确保状态转换的合法性
- **紧急机制**: 提供无条件的紧急停止功能
- **消息确认**: 服务调用提供确认反馈

### 7.3 用户体验

- **直观界面**: 大按钮设计，颜色编码清晰
- **即时反馈**: 操作后立即发布状态消息
- **工具提示**: 按钮提供操作说明

## 8. 适用场景

### 8.1 多智能体系统

- **集中控制**: 为多个无人机提供统一的行为控制入口
- **任务协调**: 支持团队任务的开始、结束和紧急中止

### 8.2 实验和测试

- **实验控制**: 为机器人实验提供标准化的启停控制
- **安全测试**: 紧急停止功能确保测试安全性

### 8.3 演示和教学

- **操作简化**: 简单的GUI降低操作门槛
- **状态可视**: 清晰的状态反馈有助于理解系统行为

## 9. 扩展可能性

### 9.1 功能扩展

- **更多模式**: 可以添加悬停、巡航等更多飞行模式
- **状态监控**: 增加更详细的系统状态显示
- **任务规划**: 集成简单的任务序列规划功能

### 9.2 界面优化

- **状态指示**: 添加LED状态指示器
- **音频反馈**: 增加操作确认音效
- **快捷键**: 支持键盘快捷操作

### 9.3 系统集成

- **日志记录**: 集成操作日志和审计功能
- **权限控制**: 添加用户权限和访问控制
- **远程控制**: 支持网络远程操作

## 10. 总结

behavior_selector是一个设计简洁但功能完备的飞行器行为控制系统。它通过标准的ROS接口和直观的图形界面，为多智能体飞行系统提供了可靠的全局模式控制能力。该系统在保证安全性的同时，具有良好的可扩展性和易用性，适合在各种无人机应用场景中使用。