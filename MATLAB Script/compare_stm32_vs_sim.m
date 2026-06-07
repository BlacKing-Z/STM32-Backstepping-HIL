%% compare_stm32_vs_sim.m
%  把 STM32(HIL) 跑出的轨迹 与 MATLAB 复现(sim_reference) 叠加对比。
%
%  步骤：
%   1) 串口助手 115200 8N1 打开板子 COM 口，勾选保存日志为 stm32_log.csv
%      （固件已自带表头 + 整数 CSV 行）。
%   2) 把 sim_reference.m 里的初始状态改成与本次烧录一致（标准起点 / 2m大误差起点）。
%   3) 直接运行本脚本：它会先跑 sim_reference 生成 x_sim/y_sim，再叠加 STM32 数据。

clear; clc;

%% ---- MATLAB 复现，得到 x_sim / y_sim ----
run('sim_reference.m');   % 与 STM32 同轨迹/同增益/同步长

%% ---- 读取 STM32 日志 ----
T = readtable('stm32_log.csv');
T = rmmissing(T);                   % 关键：去掉串口可能抓到的半行残缺数据
t   = T.t_ms     / 1000;
xd  = T.xd_mm    / 1000;
yd  = T.yd_mm    / 1000;
x   = T.x_mm     / 1000;
y   = T.y_mm     / 1000;
fic = T.fic_mdeg / 1000;            % 舵机指令角(deg)

%% ---- 轨迹对比图 ----
figure('Name','Trajectory tracking: STM32 vs MATLAB');
plot(xd, yd, 'k--', 'LineWidth', 1.2); hold on; grid on; axis equal;
plot(x_sim, y_sim, 'r-',  'LineWidth', 2.0);
plot(x,     y,     'b.',  'MarkerSize', 6);
legend('Reference','MATLAB (sim)','STM32 (HIL)','Location','best');
xlabel('x (m)'); ylabel('y (m)');
title('Backstepping trajectory tracking: STM32 vs MATLAB');

%% ---- 跟踪误差 vs 时间----
err = hypot(x - xd, y - yd);
ss  = mean(err(end-round(numel(err)/5):end), 'omitnan');   % 末段稳态误差
figure('Name','Tracking error');
plot(t, err*100, 'b-', 'LineWidth', 1.3); grid on;
xlabel('t (s)'); ylabel('position error (cm)');
title(sprintf('max err = %.1f cm,  steady-state err = %.2f cm', max(err)*100, ss*100));

fprintf('Max tracking error      : %.2f cm\n', max(err)*100);
fprintf('Steady-state error (avg): %.2f cm\n', ss*100);
fprintf('Servo command range     : [%.1f, %.1f] deg\n', min(fic), max(fic));
