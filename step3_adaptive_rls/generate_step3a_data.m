%% GENERATE_STEP3A_DATA.M - Step 3A 专用纯净质量阶跃基准时域数据生成脚本
% =========================================================================
% 工况设计依据（用户审阅核准）:
% 1. d_load = 0.0 m (中心载荷，严格消除转动惯量变化与平动-偏转惯性耦合扰动)
% 2. delta_fric = 0.0 (左右导轨摩擦严格对称，彻底排除非对称摩擦引起的偏转分量)
% 3. Kf_L = Kf_R = Kf_nom = 0.0061979 N/count (标称推力系数作为仿真已知真值)
% 4. 纯净质量阶跃:
%    - t < 3.3 s:  delta_m = 4.5 kg -> M_tot = 13.1 + 4.5 = 17.6 kg
%    - t >= 3.3 s: delta_m = 0.5 kg -> M_tot = 13.1 + 0.5 = 13.6 kg
% 5. 换向时序:
%    - t in [0.0, 3.0] s: 正向加速、巡航与减速停靠
%    - t in [3.0, 3.5] s: 静止保持区 (3.3s 进行准静态卸载)
%    - t in [3.5, 6.8] s: 返程加速、巡航与减速停靠 (3.5s 开始有返程加速度激励)
% 6. 同步生成高精度连续位移与 8192 线光电编码器量化位移 (1.21 um 分辨率)
% =========================================================================

clear; clc; close all;

ws_dir = fileparts(pwd);
addpath(fullfile(ws_dir, 'step1_baseline_c0'));
addpath(fullfile(ws_dir, 'step2_advanced_controllers'));

[ctrl, mech, plant] = param_init();

Ts = 0.001; % 1 ms
traj = trajectory_reciprocating(7.0, Ts, 1.0, 0.6, 1.5);
N_steps = length(traj.t);
t_half_idx = find(traj.t >= traj.t_half, 1);

% 基线控制器采用 C2a 固定名义参数
ctrl_c2_base.M_hat = 0.95 * diag([mech.mG_nom, mech.J_alpha_nom]);
ctrl_c2_base.B_hat = 0.90 * diag([plant.b_nom * 2.0, plant.B_alpha]);
ctrl_c2_base.K_hat = diag([0.0, plant.K_alpha]);
ctrl_c2_base.A_hat = diag([plant.fc_nom * 2.0, 0.0]);
ctrl_c2_base.Gamma = diag([30.0, 20.0]);
ctrl_c2_base.K1 = diag([45.0, 30.0]);
ctrl_c2_base.K2 = diag([12.0, 6.0]);
ctrl_c2_base.phi = [0.03; 0.03];
ctrl_c2_base.eps_v = 0.01;
ctrl_c2_base.eps_alpha = 0.01;
ctrl_c2_base.I_fw_limit = 16000.0;
ctrl_c2_base.enable_ff = true;

Imax = 4500.0; % 标称电流限幅

% 编码器当量: 1 rev = 8192 counts, 减速比 N=19, 分度圆半径 rp=0.03m
% 1 count 对应线位移: 2*pi*rp / (N * 8192)
dy_ecd = (2.0 * pi * mech.rp) / (mech.N * ctrl.ecd_cpr); % 约 1.2109e-6 m (1.21 um)

% 仿真状态初始化
x = zeros(4, 1);
log_t = traj.t;
log_yG = zeros(N_steps, 1);
log_alpha = zeros(N_steps, 1);
log_vG = zeros(N_steps, 1);
log_alpha_dot = zeros(N_steps, 1);
log_iL_cmd = zeros(N_steps, 1);
log_iR_cmd = zeros(N_steps, 1);
log_FG_applied = zeros(N_steps, 1);
log_M_true = zeros(N_steps, 1);
log_yG_quant = zeros(N_steps, 1);

t_switch = 3.30;
d_load_zero = 0.0;
delta_fric_zero = 0.0;

fprintf('>>> 正在生成 Step 3A 专用纯净质量阶跃仿真时域数据 (d=0, delta_fric=0) ...\n');

for k = 1:N_steps
    tk = traj.t(k);
    
    % 分段载荷切换
    if tk < t_switch
        dm_k = 4.5; % 正向 4.5 kg
    else
        dm_k = 0.5; % 返程 0.5 kg
    end
    M_tot_k = mech.mG_nom + dm_k;
    
    yG_c = x(1); alpha_c = x(2);
    yG_dot_c = x(3); alpha_dot_c = x(4);
    
    q_curr = [yG_c; alpha_c];
    qdot_curr = [yG_dot_c; alpha_dot_c];
    qd_curr = [traj.y(k); 0.0];
    qdot_d_curr = [traj.ydot(k); 0.0];
    qddot_d_curr = [traj.yddot(k); 0.0];
    
    % C2a 鲁棒滑模控制输出
    [iL_cmd, iR_cmd, ~] = controller_c2a_robust( ...
        q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
        ctrl_c2_base, mech.Le, mech.Kf, mech.Kf, Imax);
    
    % 实际施加推力 (对向安装符号: FG = FL + FR = Kf*iL - Kf*iR)
    FG_applied_k = mech.Kf * (iL_cmd - iR_cmd);
    
    % 真实物理对象推演
    x = gantry_dynamics_step(x, iL_cmd, iR_cmd, mech, plant, ...
        dm_k, d_load_zero, delta_fric_zero, Ts, mech.Kf, mech.Kf);
    
    % 8192 线编码器量化位移
    ecd_counts = round(yG_c / dy_ecd);
    yG_quant_k = ecd_counts * dy_ecd;
    
    log_yG(k) = yG_c;
    log_alpha(k) = alpha_c;
    log_vG(k) = yG_dot_c;
    log_alpha_dot(k) = alpha_dot_c;
    log_iL_cmd(k) = iL_cmd;
    log_iR_cmd(k) = iR_cmd;
    log_FG_applied(k) = FG_applied_k;
    log_M_true(k) = M_tot_k;
    log_yG_quant(k) = yG_quant_k;
end

% 导出专用数据文件
data_step3a.t = log_t;
data_step3a.dt = Ts;
data_step3a.yG = log_yG;
data_step3a.alpha = log_alpha;
data_step3a.vG = log_vG;
data_step3a.alpha_dot = log_alpha_dot;
data_step3a.iL_cmd = log_iL_cmd;
data_step3a.iR_cmd = log_iR_cmd;
data_step3a.FG_applied = log_FG_applied;
data_step3a.M_true = log_M_true;
data_step3a.yG_quant = log_yG_quant;
data_step3a.dy_ecd = dy_ecd;
data_step3a.t_switch = t_switch;
data_step3a.t_reverse = traj.t_half; % 3.5s

save('data_step3a.mat', 'data_step3a');
fprintf('[OK] Step 3A 专用数据已成功保存至 data_step3a.mat (共 %d 个采样点)\n', N_steps);
fprintf('     - 正向真值总质量: M_tot = %.1f kg (t < 3.3s)\n', mech.mG_nom + 4.5);
fprintf('     - 返程真值总质量: M_tot = %.1f kg (t >= 3.3s)\n', mech.mG_nom + 0.5);
fprintf('     - 编码器单步分辨率: %.3f um, 速度量化步长: %.3f mm/s\n', dy_ecd*1e6, (dy_ecd/Ts)*1e3);
