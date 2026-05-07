<!-- ──────────────── HEADER ──────────────── -->
<h1 align="center">⚙️ Contact Node</h1>
<p align="center"><em>串联 ROS 2 自瞄算法与下位机固件的高可靠通信枢纽</em></p>

<p align="center">
  <a href="#-项目概览">📘 项目概览</a> •
  <a href="#-功能亮点">✨ 功能亮点</a> •
  <a href="#-消息流转">📡 消息流转</a> •
  <a href="#-参数配置">⚙️ 参数配置</a> •
  <a href="#-构建与运行">🚀 构建与运行</a> •
  <a href="#-通信协议备注">🛰️ 协议备注</a> •
  <a href="#-todo--建议方向">🧭 TODO</a>
</p>

---

<!-- ──────────────── BADGES ──────────────── -->
<p align="center">
  <img alt="ROS2" src="https://img.shields.io/badge/ROS2-Humble-blueviolet?style=flat-square">
  <img alt="Serial" src="https://img.shields.io/badge/Serial-115200-orange?style=flat-square">
  <img alt="Status" src="https://img.shields.io/badge/status-Active-success?style=flat-square">
</p>

---

## 📘 项目概览
>
> Contact Node 基于 ROS 2 构建，用于将 <code>/autoaim/target</code> 话题中的自瞄指令实时发送到下位机
> 控制板，并把姿态、射击及模式信息回传到 ROS 系统。通过可配置的控制中间件和稳定的串口
> 管理，自动瞄准系统可以与嵌入式固件保持多路数据同步。

---

## ✨ 功能亮点

| 特性 | 说明 |
| ---- | ---- |
| **ROS 2 ↔ MCU 桥接** | 订阅 <code>gary_msgs/msg/AutoAIM</code>，向下位机发送姿态；回传关节角、四元数、颜色识别、射击状态等话题。 |
| **可组合控制中间件** | 依次执行单位换算、PID 补偿、低通滤波与安全限幅，可按需替换回调。 |
| **异步串口驱动** | <code>CommPort</code> 常驻监听 <code>/dev/ttyACM0</code>，掉线时自动扫描 STMicroelectronics 设备重连。 |
| **结构化二进制协议** | <code>TxMsg</code>/<code>RxMsg</code> 使用 packed 结构体保持与固件一致的帧格式与校验位。 |
| **参数化行为** | <code>config/contact_node.yaml</code> 管理 PID、滤波与日志开关，运行时即可调参。 |
| **开箱即用** | <code>launch/contact_node.launch.py</code> 载入默认参数并启动节点，方便测试部署。 |

---

## 📡 消息流转

<h3>输入链路</h3>
<ol>
  <li>订阅 <code>/autoaim/target</code>（<code>gary_msgs/msg/AutoAIM</code>）。</li>
  <li>将 pitch/yaw 从弧度转换为角度。</li>
  <li>结合最新回传数据运行 PID 控制器。</li>
  <li>对航向应用一阶低通滤波，对俯仰/航向做安全限幅。</li>
  <li>写入 <code>TxContent</code>（帧头 <code>0xA3</code>、姿态字段、目标状态位）并通过 <code>CommPort</code> 发送。</li>
</ol>

<h3>输出链路</h3>
<p>按 <code>rx.timerwall_ms</code> 的 Wall Timer 周期，将下位机回传解析并发布：</p>
<ul>
  <li><code>/autoaim/status</code>（<code>gary_msgs/msg/AutoAIM</code>）</li>
  <li><code>/joint_states</code>（<code>sensor_msgs/msg/JointState</code>）</li>
  <li><code>/quaternion</code>（<code>geometry_msgs/msg/Quaternion</code>）</li>
  <li><code>/color</code>（<code>std_msgs/msg/Int32</code>）</li>
  <li><code>/autoaim/mode</code>（<code>std_msgs/msg/Int32</code>）</li>
  <li><code>/autoaim/decision</code>（<code>std_msgs/msg/Int32</code>）</li>
  <li><code>/shoot_data</code>（<code>gary_msgs/msg/ShootData</code>）</li>
</ul>

---

## ⚙️ 参数配置

<p>参数集中在 <code>config/contact_node.yaml</code>，由 <code>Params</code> 结构声明：</p>
<table>
  <thead>
    <tr>
      <th>参数键</th>
      <th>含义</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><code>pid.yaw.*</code> / <code>pid.pitch.*</code></td>
      <td>俯仰与航向 PID 增益、积分限幅，对应 <code>pid.h</code> 控制器。</td>
    </tr>
    <tr>
      <td><code>filter.yaw_cutoff</code></td>
      <td>航向低通滤波器的截止频率（Hz），影响 <code>LowPassFilter</code> 响应速度。</td>
    </tr>
    <tr>
      <td><code>rx.timerwall_ms</code></td>
      <td>接收定时器周期（毫秒），决定回传消息刷新频率。</td>
    </tr>
    <tr>
      <td><code>rx.verbose</code> / <code>tx.verbose</code></td>
      <td>控制 RX/TX 日志输出的详细程度。</td>
    </tr>
  </tbody>
</table>
<p>修改参数后重新启动节点即可生效；PID 目标角度由实时消息动态赋值。</p>

---

## 🚀 构建与运行

```bash
# 构建
colcon build --packages-select contact

# 环境变量
source install/setup.bash

# 启动节点
ros2 launch contact contact_node.launch.py
```

<p>确保 <code>/dev/ttyACM0</code> 指向下位机串口。如需使用其他设备，可在 <code>CommPort::CommPort()</code> 中修改或结合 TODO 中的参数化方案。</p>

<h3>依赖组件</h3>
<ul>
  <li>ROS 2（rclcpp 及常用消息包）</li>
  <li><code>gary_msgs</code></li>
  <li><code>serial</code> 库（随仓库提供于 <code>serial/lib</code>）</li>
  <li><code>spdlog</code></li>
</ul>

---

## 🛰️ 通信协议备注

<ul>
  <li>下位机回传帧以 <code>0x3A</code> 开头、<code>0xAA</code> 结尾。</li>
  <li>上行命令帧使用帧头 <code>0x3A</code>；<code>Checksum.cpp</code> 提供 CRC8/CRC16 工具，当前默认保留校验位以兼容固件。</li>
  <li><code>RxMsg::loadMsgAndPublish()</code> 将二进制字段直接映射到 ROS 消息（四元数、关节状态、颜色、射击决策等）。</li>
</ul>

---

## 🧭 TODO / 建议方向

<ul>
  <li>[ ] 接入 CRC 校验流程，提升帧错误检测与容错能力。</li>
  <li>[ ] 设计基于 Launch/rostest 的串口数据模拟测试，覆盖关键消息流。</li>
  <li>[ ] 支持运行时 PID 调参，可通过参数事件回调或诊断话题发布。</li>
  <li>[ ] 补充协议文档，如帧格式示意或状态机时序图，便于跨团队协作。</li>
</ul>

---

<p align="center"><sub>© 2025 XJTLU GMaster</sub></p>
