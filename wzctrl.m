function [sys,x0,str,ts,simStateCompliance] = wzctrl(t,x,u,flag)
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

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 16;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
sys = [];

function sys=mdlUpdate(t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u)
%% [核心修改区域] 定义螺旋上升轨迹
% 参数设置：半径R=2米，角速度omega=0.5rad/s，上升速度vz=0.5m/s
R = 2;
omega = 0.5;
vz = 0.5;

% X轴：余弦运动 (x = R*cos(wt))
x1d   = R * cos(omega*t);
dx1d  = -R * omega * sin(omega*t);
ddx1d = -R * omega^2 * cos(omega*t);

% Y轴：正弦运动 (y = R*sin(wt))
yd    = R * sin(omega*t);
dyd   = R * omega * cos(omega*t);
ddyd  = -R * omega^2 * sin(omega*t);

% Z轴：匀速上升 (z = vz*t)
% 设定一个上限，比如飞到10米就停在10米
if t < 20
    zd   = vz * t;
    dzd  = vz;
    ddzd = 0;
else
    zd   = 20 * vz; % 保持在最后的高度
    dzd  = 0;
    ddzd = 0;
end

%% 获取当前状态
x1 = u(10);  dx1 = u(11);
y = u(12);   dy = u(13);
z = u(14);   dz = u(15);
phi = u(16);

%% 定义误差
xe = x1 - x1d;  dxe = dx1 - dx1d; % 注意：这里我补全了X轴误差定义
ye = y - yd;    dye = dy - dyd;   % 注意：这里我补全了Y轴误差定义
ze = z - zd;    dze = dz - dzd;

%% 无人机物理参数
m = 1.8;  g = 9.8;  K3 = 0.01;

%% 控制器参数
kdx = 5; kpx = 5;
kdy = 5; kpy = 5;
kdz = 5; kpz = 5;

%% 位置环控制律 (PD + 前馈)
% X轴控制 (包含加速度前馈 ddx1d)
u1x = -kpx*xe - kdx*dxe + ddx1d; 

% Y轴控制 (包含加速度前馈 ddyd)
u1y = -kpy*ye - kdy*dye + ddyd; 

% Z轴控制 (包含重力g + 加速度前馈ddzd + 阻力补偿)
u1z = -kpz*ze - kdz*dze + g + ddzd + K3/m*dzd;

%% 姿态解算器 (防奇异保护版)
if abs(u1z) < 0.001, u1z = 0.001; end

% 计算中间变量X
X_val = (cos(phi)*cos(phi)*u1x + cos(phi)*sin(phi)*u1y)/u1z;

% 期望俯仰角 (限制幅度)
if X_val > 1
    thetad = pi/2;
elseif X_val < -1
    thetad = -pi/2;
else
    thetad = asin(X_val);
end

% 期望偏航角 (协调转弯策略)
psid = atan((sin(phi)*cos(phi)*u1x - cos(phi)*cos(phi)*u1y)/u1z);

% 总升力
u1 = u1z/(cos(phi)*cos(psid));

%% 模块输出
sys(1) = u1;
sys(2) = thetad;
sys(3) = psid;

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;   
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];