%% sim_reference.m
%  纯 MATLAB 复现 STM32 main_hil.c 的控制器 + 运动学，生成 x_sim / y_sim。
%  轨迹、增益、步长、初始状态都与 STM32 完全一致 —— 这样和 stm32_log.csv 对比才有效。
%  运行后会得到 x_sim, y_sim（供 compare_stm32_vs_sim.m 叠加）。

clear;

%% ---- 与 STM32 完全一致的参数 ----
L   = 0.254;        % 轴距
DT  = 0.02;         % 控制周期 50Hz
R   = 1.0;          % 参考圆半径
Vd  = 0.5;          % 目标线速度（对应 STM32 默认值，不转编码器时）
Tend = 12;          % 仿真时长(s)，跑够一圈多

% 初始状态：与 main_hil.c 里一致
% 标准起点用 (0.7,-0.2,pi/2)；做"2m大误差"演示时改成 (-1.0, 0.0, pi/2)
x  = 0.7;  y = -0.2;  th = pi/2;

N = round(Tend/DT);
x_sim  = zeros(1,N);  y_sim  = zeros(1,N);
xd_log = zeros(1,N);  yd_log = zeros(1,N);
t_log  = (0:N-1)*DT;

%% ---- 主循环：和 STM32 一模一样 ----
for k = 1:N
    t = (k-1)*DT;

    % 参考圆轨迹
    wd  = Vd / R;
    phi = wd * t;
    xd  = R*cos(phi);
    yd  = R*sin(phi);
    thd = phi + pi/2;

    % 反步法控制器（与 controller.c / 曲线跟踪控制器函数.txt 同一套）
    [v, fic, omega] = backstepping_ctrl_local(xd, yd, thd, wd, Vd, x, y, th, L);

    % 运动学积分（自行车模型，后轴参考）
    th = th + (v/L)*tan(fic)*DT;
    th = atan2(sin(th), cos(th));
    x  = x + v*cos(th)*DT;
    y  = y + v*sin(th)*DT;

    x_sim(k)=x; y_sim(k)=y; xd_log(k)=xd; yd_log(k)=yd;
end

%% ---- 快速自检图 ----
figure('Name','MATLAB 复现');
plot(xd_log,yd_log,'k--'); hold on; plot(x_sim,y_sim,'r-','LineWidth',1.5);
axis equal; grid on; legend('参考','MATLAB跟踪'); title('sim\_reference 自检');

disp('已生成 x_sim, y_sim，可直接运行 compare_stm32_vs_sim.m 叠加 STM32 数据。');

%% ===== 本地控制器函数（从 曲线跟踪控制器函数.txt 整理，与 STM32 一致）=====
function [v, fic, omega] = backstepping_ctrl_local(xd,yd,thd,wd,Vd,x,y,th,L)
    k1=5.0; k2=6.0; k3=3.0; k4=20.0;

    xe = xd-x;  ye = yd-y;  the = atan2(sin(thd-th),cos(thd-th));
    e1 =  cos(th)*xe + sin(th)*ye;
    e2 = -sin(th)*xe + cos(th)*ye;
    etheta = the;
    vr = Vd;  wr = wd;

    c=cos(etheta/2); s=sin(etheta/2);
    omega = 2*k3*e2*vr*c + k4*s + wr;
    omega_dot = 2*k3*((-omega*e1 + vr*sin(etheta))*vr)*c ...
              - k3*e2*vr*s*(wr-omega) + 0.5*k4*c*(wr-omega);
    den=sqrt(1+omega^2);
    v = vr*cos(etheta) + k1*omega*e1*(omega/den) - k1*vr*sin(etheta)*(omega/den) ...
        - k1*omega_dot*e2/den^3 + k2*(e1 - k1*e2*(omega/den));

    if abs(v)>5,     v=sign(v)*5;     end
    if abs(omega)>5, omega=sign(omega)*5; end
    if abs(v)<0.001, fic=0; else, fic=atan(omega*L/v); end
    if abs(fic)>pi/4, fic=sign(fic)*pi/4; end
end
