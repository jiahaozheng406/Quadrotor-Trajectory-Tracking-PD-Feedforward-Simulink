function [sys,x0,str,ts,simStateCompliance] = TD1(t,x,u,flag)
% TD1: 三阶非线性跟踪微分器 S-Function
% 功能：接收一个输入信号，输出该信号的一阶导数和二阶导数
% 特点：采用非线性反馈控制律，具有有限时间收敛特性

switch flag
  case 0
    % 初始化：设置输入输出维度、状态数量等
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    % 导数计算：核心算法，计算状态变量的微分方程
    sys=mdlDerivatives(t,x,u);
  case 2
    % 离散状态更新（本项目未使用）
    sys=mdlUpdate(t,x,u);
  case 3
    % 输出函数：将状态量映射到模块输出
    sys=mdlOutputs(t,x,u);
  case 4
    % 计算下一次采样时间（变步长使用，此处未使用）
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9
    % 仿真结束时的清理工作
    sys=mdlTerminate(t,x,u);
  otherwise
    % 错误处理
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

%% ================== 初始化函数 ==================
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 3; % 定义3个连续状态：x(1)估计值, x(2)一阶导, x(3)二阶导
sizes.NumDiscStates  = 0; % 无离散状态
sizes.NumOutputs     = 3; % 定义3个输出
sizes.NumInputs      = 1; % 定义1个输入（即目标跟踪信号）
sizes.DirFeedthrough = 1; % 存在直通分量（在输出函数中直接用到了输入u）
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes);
x0  = [0 0 0];            % 状态初始值初始化为0
str = [];
ts  = [0 0];              % 连续采样时间
simStateCompliance = 'UnknownSimState';

%% ================== 导数计算函数 (核心算法) ==================
function sys=mdlDerivatives(t,x,u)
% 参数设置
ebs = 0.10; % 滤波因子 (epsilon)。越小跟踪越快但噪声敏感，越大滤波效果越好但相位滞后。

% 输入信号
vt = u(1);  % 目标信号 v(t)

% --- 非线性反馈控制律计算 ---
% 这是一种基于高阶滑模或有限时间控制理论的微分器设计
% x(1) 追踪 v(t)
% x(2) 追踪 v'(t)
% x(3) 追踪 v''(t)

% 临时变量1：与一阶导数相关的非线性项 (类似阻尼项)
temp1 = (abs(ebs*x(2))^(9/7))*sign(ebs*x(2));

% 临时变量2：位置误差综合项 (x(1) - vt) + 阻尼
temp2 = x(1) - vt + temp1;
% 对误差项进行非线性变换 (分数幂 1/3)
temp2 = abs(temp2)^(1/3)*sign(temp2);

% 临时变量3：与二阶导数相关的非线性项 (分数幂 3/5)
temp3 = abs(ebs^2*x(3))^(3/5)*sign(ebs^2*x(3));

% --- 状态方程定义 ---
% dx1 = x2
sys(1) = x(2); 

% dx2 = x3
sys(2) = x(3); 

% dx3 = f(...) 高阶非线性反馈，迫使 x(1) 追踪 vt
% 这里的公式结构使得系统在有限时间内收敛
sys(3) = (-2^(3/5)*4*temp2 - 4*temp3) * 1/ebs^3;

%% ================== 离散更新函数 (未使用) ==================
function sys=mdlUpdate(t,x,u)
sys = [];

%% ================== 输出函数 ==================
function sys=mdlOutputs(t,x,u)
v = u(1); % 获取原始输入信号

% 输出1：直接输出原始输入信号 (用于后续控制器对比或前馈)
% 注意：有些TD设计这里输出的是滤波后的 x(1)，但此处代码选择输出原始输入 u(1)
sys(1) = v;   

% 输出2：输出估计的一阶导数 (速度)
sys(2) = x(2);

% 输出3：输出估计的二阶导数 (加速度)
sys(3) = x(3);

%% ================== 其他系统函数 ==================
function sys=mdlGetTimeOfNextVarHit(t,x,u)
sys = []; % 变步长设置为空

function sys=mdlTerminate(t,x,u)
sys = []; % 结束无特殊操作
