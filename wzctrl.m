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
sizes.NumOutputs     = 3;   % 输出: 升力u1, 期望俯仰theta_d, 期望偏航psi_d
sizes.NumInputs      = 16;  % 输入: 包含无人机所有状态量
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


%%                 轨迹生成模块 (四选一)

%% --- 场景 1: 高度跃升测试 (Altitude Step) --- 
% 目的：测试Z轴的阶跃响应速度和超调量
%{
    % 水平位置保持在原点
    x1d = 0; dx1d = 0; ddx1d = 0;
    yd  = 0; dyd  = 0; ddyd  = 0;

    % Z轴：前5秒在0米，5秒时瞬间设定为5米
    if t < 5
        zd = 0; dzd = 0; ddzd = 0;
    else
        zd = 5; dzd = 0; ddzd = 0; 
    end
%}

%% --- 场景 2: 螺旋上升 (Spiral Ascent) --- 
% 目的：论文基准测试，验证三轴联动的跟踪精度
%
    R = 2;          % 半径
    omega = 0.5;    % 角速度
    vz = 0.5;       % 上升速度

    % X轴：余弦运动
    x1d   = R * cos(omega*t);
    dx1d  = -R * omega * sin(omega*t);
    ddx1d = -R * omega^2 * cos(omega*t);

    % Y轴：正弦运动
    yd    = R * sin(omega*t);
    dyd   = R * omega * cos(omega*t);
    ddyd  = -R * omega^2 * sin(omega*t);

    % Z轴：匀速上升，最高到10米
    if t < 20
        zd   = vz * t;
        dzd  = vz;
        ddzd = 0;
    else
        zd   = 10; dzd = 0; ddzd = 0;
    end
%

%% --- 场景 3: 8字形飞行 (Figure-8) --- 
% 目的：测试无人机在水平面做复杂曲线时的机动性
%{
    A = 5;      % 幅度
    w = 0.4;    % 频率

    % X轴：左右摆动 A*sin(wt)
    x1d   = A * sin(w*t);
    dx1d  = A * w * cos(w*t);
    ddx1d = -A * w^2 * sin(w*t);

    % Y轴：倍频摆动形成8字 A/2*sin(2wt)
    yd    = (A/2) * sin(2*w*t);
    dyd   = A * w * cos(2*w*t);
    ddyd  = -2 * A * w^2 * sin(2*w*t);

    % Z轴：定高悬停
    zd = 5; dzd = 0; ddzd = 0;
%}

%% --- 场景 4: 矩形巡航 (Square Patrol) ---
% 目的：测试对直角转弯（突变信号）的响应能力
%{
    period = 20;    % 跑一圈的时间
    lt = mod(t, period); 
    v = 1.0;        % 巡航速度

    zd = 5; dzd = 0; ddzd = 0; % 高度固定

    % 分四段走矩形：(0,0)->(0,5)->(5,5)->(5,0)->(0,0)
    if lt < 5
        x1d = 0; dx1d=0; ddx1d=0;
        yd = v*lt; dyd=v; ddyd=0;
    elseif lt < 10
        x1d = v*(lt-5); dx1d=v; ddx1d=0;
        yd = 5; dyd=0; ddyd=0;
    elseif lt < 15
        x1d = 5; dx1d=0; ddx1d=0;
        yd = 5 - v*(lt-10); dyd=-v; ddyd=0;
    else
        x1d = 5 - v*(lt-15); dx1d=-v; ddx1d=0;
        yd = 0; dyd=0; ddyd=0;
    end
%}

%% =============================================================
%%                   控制器核心算法 (PD + 前馈)
%% =============================================================

% 1. 获取当前状态 (从输入u读取)
x1  = u(10); dx1 = u(11);
y   = u(12); dy  = u(13);
z   = u(14); dz  = u(15);
phi = u(16);

% 2. 计算误差 (Error)
xe  = x1 - x1d;  dxe = dx1 - dx1d;
ye  = y  - yd;   dye = dy  - dyd;
ze  = z  - zd;   dze = dz  - dzd;

% 3. 物理参数
m = 1.8;  g = 9.8;  K3 = 0.01;

% 4. 控制器参数 (根据需要微调)
kdx = 5; kpx = 5;
kdy = 5; kpy = 5;
kdz = 5; kpz = 5;

% 5. 位置控制律计算
% X轴 (含加速度前馈)
u1x = -kpx*xe - kdx*dxe + ddx1d;

% Y轴 (含加速度前馈)
u1y = -kpy*ye - kdy*dye + ddyd;

% Z轴 (含重力补偿 + 加速度前馈 + 阻力补偿)
u1z = -kpz*ze - kdz*dze + g + ddzd + K3/m*dzd;

% 6. 姿态解算 (虚拟控制量 -> 实际角度)
% 防止分母为0
if abs(u1z) < 0.001, u1z = 0.001; end

% 计算中间变量
X_temp = (cos(phi)*cos(phi)*u1x + cos(phi)*sin(phi)*u1y)/u1z;

% 期望俯仰角 (theta_d) - 增加限幅防止翻车
if X_temp > 1
    thetad = pi/2;
elseif X_temp < -1
    thetad = -pi/2;
else
    thetad = asin(X_temp);
end

% 期望偏航角 (psi_d)
psid = atan((sin(phi)*cos(phi)*u1x - cos(phi)*cos(phi)*u1y)/u1z);

% 计算总升力 U1
u1 = u1z/(cos(phi)*cos(psid));

%% 7. 模块输出
sys(1) = u1;
sys(2) = thetad;
sys(3) = psid;

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;    
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];
