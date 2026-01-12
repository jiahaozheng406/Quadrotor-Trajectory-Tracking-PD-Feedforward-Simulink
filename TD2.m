function [sys,x0,str,ts,simStateCompliance] = TD1(t,x,u,flag)
% 这里的函数名保持为 TD1
% 功能：三阶非线性跟踪微分器 (S-Function)
% 作用：对输入信号进行快速跟踪，并提取一阶和二阶导数，参数 ebs=0.04 (较快收敛模式)

switch flag
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case 2
    sys=mdlUpdate(t,x,u);
  case 3
    sys=mdlOutputs(t,x,u);
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

%% ================== 1. 初始化函数 ==================
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 3; % 连续状态变量个数：3个 (x1跟踪信号, x2跟踪一阶导, x3跟踪二阶导)
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3; % 输出个数：3个 (原信号，一阶导，二阶导)
sizes.NumInputs      = 1; % 输入个数：1个 (需要跟踪的目标信号)
sizes.DirFeedthrough = 1; % 是否存在直通 (输出函数直接使用了输入u)
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes);
x0  = [0 0 0];            % 状态初始值 [x1, x2, x3] 初始化为0
str = [];
ts  = [0 0];              % 连续采样时间
simStateCompliance = 'UnknownSimState';

%% ================== 2. 导数计算函数 (核心算法) ==================
function sys=mdlDerivatives(t,x,u)
% ebs (epsilon): 滤波因子/时间尺度参数
% 值越小(0.04)，系统的频带越宽，跟踪越快，但对噪声的抑制能力会下降
ebs = 0.04; 

vt = u(1); % 输入信号 v(t)

% --- 非线性反馈律 (有限时间收敛控制) ---
% 这里的非线性函数形式 (abs^k * sign) 是为了保证系统状态快速收敛且无颤振

% 构造中间项1：与速度 x(2) 相关的非线性阻尼项
temp1 = (abs(ebs*x(2))^(9/7))*sign(ebs*x(2));

% 构造中间项2：位置误差项 (x(1) - vt) 加上 temp1
temp2 = x(1) - vt + temp1;
% 对误差项进行非线性变换 (1/3次幂)
temp2 = abs(temp2)^(1/3)*sign(temp2);

% 构造中间项3：与加速度 x(3) 相关的非线性项 (3/5次幂)
temp3 = abs(ebs^2*x(3))^(3/5)*sign(ebs^2*x(3));

% --- 状态方程 (积分链) ---
% dx1 = x2
sys(1) = x(2);

% dx2 = x3
sys(2) = x(3);

% dx3 = u_control (非线性控制律)
% 计算三阶导数，迫使 x(1) -> vt
sys(3) = (-2^(3/5)*4*temp2 - 4*temp3) * 1/ebs^3;

%% ================== 3. 离散更新 (未使用) ==================
function sys=mdlUpdate(t,x,u)
sys = [];

%% ================== 4. 输出函数 ==================
function sys=mdlOutputs(t,x,u)
v = u(1);

% 输出端口定义：
sys(1) = v;    % Port 1: 原始输入信号 (Pass-through)
sys(2) = x(2); % Port 2: 估计的一阶导数 (速度)
sys(3) = x(3); % Port 3: 估计的二阶导数 (加速度)

%% ================== 5. 其他辅助函数 ==================
function sys=mdlGetTimeOfNextVarHit(t,x,u)
sys = [];

function sys=mdlTerminate(t,x,u)
sys = [];
