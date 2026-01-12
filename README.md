# Quadrotor-Trajectory-Tracking-PD-Feedforward-Simulink
# 基于前馈补偿PD的四旋翼无人机轨迹跟踪控制 (Simulink仿真)

## 1. 项目背景与简介

本项目为 **物联网控制技术** 课程设计内容。核心工作是基于 MATLAB/Simulink 平台，复现了学术论文 **《基于前馈补偿的PD四旋翼无人机轨迹跟踪控制》** 中的控制算法。

针对四旋翼无人机在飞行过程中面临的非线性、强耦合以及抗干扰问题，本项目构建了完整的 **六自由度动力学模型**，并采用了双闭环控制策略。其中，**外环（位置环）** 采用基于前馈补偿的 PD 控制算法，负责计算期望的姿态角；**内环（姿态环）** 采用 PD 控制算法，负责快速跟踪期望姿态。

特别说明：代码为个人复现与改进版本，旨在验证论文理论的有效性，可能存在参数差异和代码缺陷，如有问题欢迎指正。


## 2. 复现细节与改进创新

在复现原论文核心算法的基础上，本项目着重改进了测试验证策略。原论文通常仅使用单一轨迹进行验证，而本项目在 `wzctrl.m` 中扩展了四种不同类型的测试轨迹，以全面评估算法在不同工况下的鲁棒性：

**高度跃升 (Altitude Step)**
测试无人机在垂直方向上的阶跃响应能力，重点观察系统的超调量与稳态误差。

**螺旋上升 (Spiral Ascent)**
这是原论文中的基准测试轨迹，用于验证 X、Y、Z 三轴联动时的跟踪精度。

**8字形飞行 (Figure-8)**
测试无人机在水平面内进行连续转弯时的耦合机动性能，考验前馈补偿对动态误差的抑制能力。

**矩形巡航 (Square Patrol)**
模拟直角转弯路径，测试系统对位置突变信号（非平滑路径）的瞬态响应能力。



## 3. 项目文件结构说明

**matlab2024b.slx (仿真模型)**
这是项目的主入口文件，包含了完整的 Simulink 框图，集成了无人机物理模型、控制器模块以及信号连接。**请注意，需使用 MATLAB 2024b 或更高版本打开此文件。**

**wzctrl.m (位置控制器/外环)**
这是本项目改动最大的核心脚本。它不仅包含了 PD 控制律和前馈项的计算逻辑，还集成了上述四种测试轨迹的生成算法，是控制系统的核心大脑。

**ztctrl.m (姿态控制器/内环)**
该脚本负责接收来自位置环的期望角度（Roll, Pitch）和偏航角（Yaw），并计算出电机所需的升力力矩，确保姿态的快速响应。

**UAV.m (被控对象模型)**
基于牛顿-欧拉方程建立的四旋翼无人机六自由度非线性动力学方程（S-Function），用于在仿真环境中模拟真实的物理特性。

**TD1.m / TD2.m (跟踪微分器)**
用于对输入信号进行平滑处理，并提取高质量的微分信号（速度/加速度），辅助控制器减少噪声干扰。



## 4. 参考与致谢

本项目的核心算法来源于 **刘栩粼, 胡德清** 的论文 **《基于前馈补偿的PD四旋翼无人机轨迹跟踪控制》**，模型基础构建于常规四旋翼动力学建模理论。



---

# Quadrotor-Trajectory-Tracking-PD-Feedforward-Simulink

# Quadrotor UAV Trajectory Tracking Control based on PD with Feedforward Compensation (Simulink Simulation)

## 1. Project Background & Introduction

This project is the coursework for the **IoT Control Technology** course. The core objective is to reproduce the control algorithm from the academic paper **"Trajectory Tracking Control for Quadrotor UAV Based on PD with Feedforward Compensation"** using the MATLAB/Simulink platform.

Addressing the issues of nonlinearity, strong coupling, and interference faced by quadrotor UAVs during flight, this project constructs a complete **Six-Degrees-of-Freedom (6-DOF) Dynamic Model** and adopts a dual-loop control strategy. Specifically, the **Outer Loop (Position Loop)** employs a PD control algorithm based on feedforward compensation to calculate the desired attitude angles; the **Inner Loop (Attitude Loop)** employs a PD control algorithm to ensure rapid tracking of the desired attitude.

**Note**: This code is a personal reproduction and improvement version intended to verify the validity of the paper's theory. There may be parameter differences or code defects. Feedback and corrections are welcome.

---

## 2. Reproduction Details & Improvements

Building upon the reproduction of the core algorithm from the original paper, this project emphasizes improvements in the verification strategy. While the original paper typically uses a single trajectory for verification, this project expands `wzctrl.m` to include four different types of test trajectories to comprehensively evaluate the algorithm's robustness under various operating conditions:

**Altitude Step**
Tests the step response capability of the UAV in the vertical direction, focusing on observing the system's overshoot and steady-state error.

**Spiral Ascent**

This is the benchmark test trajectory from the original paper, used to verify the tracking accuracy during X, Y, and Z three-axis linkage.

**Figure-8 Flight**

Tests the coupling maneuverability of the UAV during continuous turning in the horizontal plane, challenging the feedforward compensation's ability to suppress dynamic errors.

**Square Patrol (Rectangular Cruise)**
Simulates a path with right-angle turns to test the system's transient response capability to position mutation signals (non-smooth paths).

---

## 3. Project File Structure Description

**matlab2024b.slx (Simulation Model)**
This is the main entry file of the project, containing the complete Simulink block diagram that integrates the UAV physical model, controller modules, and signal connections. **Please note: This file must be opened using MATLAB 2024b or a newer version.**

**wzctrl.m (Position Controller / Outer Loop)**
This is the core script with the most significant modifications in this project. It contains not only the calculation logic for the PD control law and feedforward terms but also integrates the generation algorithms for the four test trajectories mentioned above. It serves as the "brain" of the control system.

**ztctrl.m (Attitude Controller / Inner Loop)**
This script is responsible for receiving the desired angles (Roll, Pitch) and Yaw angle from the position loop and calculating the required lift torques for the motors to ensure rapid attitude response.

**UAV.m (Controlled Object Model)**
A Six-Degrees-of-Freedom nonlinear dynamic equation (S-Function) of the quadrotor UAV established based on Newton-Euler equations, used to simulate real physical characteristics in the simulation environment.

**TD1.m / TD2.m (Tracking Differentiators)**
Used to smooth input signals and extract high-quality differential signals (velocity/acceleration) to assist the controller in reducing noise interference.

---

## 4. References & Acknowledgements

The core algorithm of this project is derived from the paper **"Trajectory Tracking Control for Quadrotor UAV Based on PD with Feedforward Compensation"** by **Liu Xulin and Hu Deqing**. The model foundation is built upon standard quadrotor dynamic modeling theory.

If you have questions regarding the mathematical derivations or other contents in the code, it is recommended to refer to the **PDF Paper** and compare it with the detailed comments in the code for understanding.

如果您对代码中的数学推导等内容有疑问，建议参考 **PDF 论文** 以及代码中的详细注释进行对照理解。


---
