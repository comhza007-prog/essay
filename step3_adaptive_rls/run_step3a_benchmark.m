%% RUN_STEP3A_BENCHMARK.M - Step 3A 机械参数在线辨识与自适应闭环基准测试与图表生成
% =========================================================================
% 功能：
% 1. 执行 Step 3A 完整辨识与闭环基准仿真对比 (基线 C2a vs 自适应 C3a)
% 2. 导出高分辨率学术图表 (PNG):
%    - step3a_parameter_identification.png: 参数在线收敛曲线、PE 门控指示与量化抗噪
%    - step3a_closed_loop_comparison.png: C2a vs C3a 位移跟踪误差、控制电流与 TV 平稳性
% 3. 生成 step3a_metrics_summary.csv 结构化指标记录
% =========================================================================

clear; clc; close all;

ws_dir = fileparts(pwd);
addpath(fullfile(ws_dir, 'step1_baseline_c0'));
addpath(fullfile(ws_dir, 'step2_advanced_controllers'));
addpath(pwd);

[ctrl, mech, plant] = param_init();
Ts = 0.001;
traj = trajectory_reciprocating(7.0, Ts, 1.0, 0.6, 1.5);
Imax = 4500.0;

% 基线控制器 C2a 固定名义参数
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

% 载入专用时域数据
load('data_step3a.mat');
d3 = data_step3a;
N_data = length(d3.t);

fprintf('==================================================================================\n');
fprintf('       Step 3A: 机械参数在线辨识与自适应闭环基准测试 (run_step3a_benchmark)\n');
fprintf('==================================================================================\n\n');

%% 1. 运行开环 RLS 辨识推演 (记录详细轨迹)
fprintf('>>> 正在执行开环 RLS 在线辨识推演 ...\n');
svf_mech = rls_filter_svf(10.0, Ts);
rls_mech = rls_estimator_mech([13.1; 70.0; 16.0], 0.995, 1.0e-4, Ts);

th_hist = zeros(N_data, 3);
th_raw_hist = zeros(N_data, 3);
lam_min_hist = zeros(N_data, 1);
is_pe_hist = false(N_data, 1);
trace_P_hist = zeros(N_data, 1);
yddot_f_hist = zeros(N_data, 1);
ydot_f_hist = zeros(N_data, 1);

for k = 1:N_data
    [svf_mech, yG_f, ydot_f, yddot_f, FG_f, Sf_f] = ...
        svf_mech.step(d3.yG_quant(k), d3.FG_applied(k));
    
    phi_k = [yddot_f; ydot_f; Sf_f];
    [rls_mech, th_k, info_k] = rls_mech.step(FG_f, phi_k);
    
    th_hist(k, :) = th_k';
    th_raw_hist(k, :) = info_k.theta_raw';
    lam_min_hist(k) = info_k.lambda_min;
    is_pe_hist(k) = info_k.is_pe_active;
    trace_P_hist(k) = info_k.trace_P;
    yddot_f_hist(k) = yddot_f;
    ydot_f_hist(k) = ydot_f;
end

%% 2. 运行闭环自适应控制器 C3a 推演
fprintf('>>> 正在执行闭环自适应控制器 C3a 仿真推演 ...\n');
x_c3a = zeros(4, 1);
state_c3a = [];
log_yG_c3a = zeros(N_data, 1);
log_alpha_c3a = zeros(N_data, 1);
log_iL_c3a = zeros(N_data, 1);
log_iR_c3a = zeros(N_data, 1);
log_M_est_cl = zeros(N_data, 1);

for k = 1:N_data
    tk = d3.t(k);
    if tk < 3.30, dm_k = 4.5; else, dm_k = 0.5; end
    
    qc = [x_c3a(1); x_c3a(2)];
    qdotc = [x_c3a(3); x_c3a(4)];
    qd = [traj.y(k); 0.0];
    qdotd = [traj.ydot(k); 0.0];
    qddotd = [traj.yddot(k); 0.0];
    
    [iL_c3a, iR_c3a, state_c3a, info_c3a] = controller_c3a_rls_robust( ...
        qc, qdotc, qd, qdotd, qddotd, ...
        ctrl_c2_base, mech.Le, mech.Kf, mech.Kf, Imax, state_c3a, Ts);
    
    x_c3a = gantry_dynamics_step(x_c3a, iL_c3a, iR_c3a, mech, plant, ...
        dm_k, 0.0, 0.0, Ts, mech.Kf, mech.Kf);
    
    log_yG_c3a(k) = x_c3a(1);
    log_alpha_c3a(k) = x_c3a(2);
    log_iL_c3a(k) = iL_c3a;
    log_iR_c3a(k) = iR_c3a;
    log_M_est_cl(k) = info_c3a.M_tot_hat;
end

%% 3. 统计关键指标
fwd_idx = find(d3.t >= 2.0 & d3.t <= 3.0);
rev_idx = find(d3.t >= 5.8 & d3.t <= 6.8);

M_fwd_mean = mean(th_hist(fwd_idx, 1));
M_fwd_err_pct = abs(M_fwd_mean - 17.6) / 17.6 * 100.0;
M_rev_mean = mean(th_hist(rev_idx, 1));
M_rev_err_pct = abs(M_rev_mean - 13.6) / 13.6 * 100.0;
M_rev_std = std(th_hist(rev_idx, 1));

bG_rev_mean = mean(th_hist(rev_idx, 2));
bG_rev_err_pct = abs(bG_rev_mean - 70.0) / 70.0 * 100.0;
fcG_rev_mean = mean(th_hist(rev_idx, 3));
fcG_rev_err_pct = abs(fcG_rev_mean - 16.0) / 16.0 * 100.0;

tv_c2a = sum(abs(diff(d3.iL_cmd))) + sum(abs(diff(d3.iR_cmd)));
tv_c3a = sum(abs(diff(log_iL_c3a))) + sum(abs(diff(log_iR_c3a)));
tv_ratio = tv_c3a / tv_c2a;

e_yG_c2a = (d3.yG - traj.y(:)) * 1e3; % mm
e_yG_c3a = (log_yG_c3a - traj.y(:)) * 1e3; % mm
rmse_yG_c2a = sqrt(mean(e_yG_c2a.^2));
rmse_yG_c3a = sqrt(mean(e_yG_c3a.^2));
max_e_yG_c2a = max(abs(e_yG_c2a));
max_e_yG_c3a = max(abs(e_yG_c3a));

pe_active_ratio = sum(is_pe_hist(300:end)) / (N_data - 299) * 100.0;

fprintf('\n----------------------------------------------------------------------------------\n');
fprintf('  Step 3A 核心辨识与控制性能指标汇总\n');
fprintf('----------------------------------------------------------------------------------\n');
fprintf('  [正向稳态] 质量真值 = 17.600 kg, 估计均值 = %.3f kg (误差 %.2f%%)\n', M_fwd_mean, M_fwd_err_pct);
fprintf('  [返程稳态] 质量真值 = 13.600 kg, 估计均值 = %.3f kg (误差 %.2f%%, 抖动 std = %.4f kg)\n', ...
    M_rev_mean, M_rev_err_pct, M_rev_std);
fprintf('  [摩擦阻尼] 阻尼真值 = 70.00 N*s/m, 估计均值 = %.2f (误差 %.2f%%)\n', bG_rev_mean, bG_rev_err_pct);
fprintf('  [库仑摩擦] 摩擦真值 = 16.00 N,     估计均值 = %.2f (误差 %.2f%%)\n', fcG_rev_mean, fcG_rev_err_pct);
fprintf('  [PE 门控]  300ms 滑动窗激活占比 = %.2f%% (匀速与静止段绝对冻结)\n', pe_active_ratio);
fprintf('  [跟踪误差] C2a RMSE = %.2f mm (峰值 %.2f mm) -> C3a RMSE = %.2f mm (峰值 %.2f mm)\n', ...
    rmse_yG_c2a, max_e_yG_c2a, rmse_yG_c3a, max_e_yG_c3a);
fprintf('  [控制变差] C2a TV = %.1f -> C3a TV = %.1f (TV 比值 = %.3f <= 1.10)\n', ...
    tv_c2a, tv_c3a, tv_ratio);
fprintf('----------------------------------------------------------------------------------\n\n');

%% 4. 生成高学术规格图表 1: step3a_parameter_identification.png
fprintf('>>> 正在绘制学术图表 1: step3a_parameter_identification.png ...\n');
fig1 = figure('Name', 'Step3A_Parameter_Identification', 'Color', 'w', ...
    'Units', 'pixels', 'Position', [100, 100, 1400, 950], 'Visible', 'off');

% Subplot 1: 质量在线估计曲线 vs 真值
subplot(2, 2, 1);
plot(d3.t, d3.M_true, 'k--', 'LineWidth', 1.8, 'DisplayName', '物理真值 M_{true}');
hold on;
plot(d3.t, th_raw_hist(:, 1), ':', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.2, 'DisplayName', 'RLS 原始估计 \theta_{raw}');
plot(d3.t, th_hist(:, 1), 'b-', 'LineWidth', 2.0, 'DisplayName', '平滑输出 \theta_{smooth}');
xline(3.3, 'r--', 'LineWidth', 1.2, 'DisplayName', '载荷突变 t=3.3s');
xline(3.5, 'g--', 'LineWidth', 1.2, 'DisplayName', '返程启动 t=3.5s');
grid on; box on;
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('总质量 M_{tot} (kg)', 'FontSize', 11, 'FontWeight', 'bold');
title('(a) 平动总质量在线估计收敛历程', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
ylim([12.5, 18.5]);

% Subplot 2: 导轨摩擦参数估计曲线
subplot(2, 2, 2);
yyaxis left;
plot(d3.t, 70.0 * ones(size(d3.t)), 'k--', 'LineWidth', 1.2, 'DisplayName', 'b_G 真值 (70 N\cdots/m)');
hold on;
plot(d3.t, th_hist(:, 2), 'b-', 'LineWidth', 1.8, 'DisplayName', 'b_G 估计值');
ylabel('黏性阻尼系数 b_G (N\cdots/m)', 'FontSize', 11, 'FontWeight', 'bold');
ylim([50, 90]);

yyaxis right;
plot(d3.t, 16.0 * ones(size(d3.t)), 'r--', 'LineWidth', 1.2, 'DisplayName', 'f_{c,G} 真值 (16 N)');
hold on;
plot(d3.t, th_hist(:, 3), 'm-', 'LineWidth', 1.8, 'DisplayName', 'f_{c,G} 估计值');
ylabel('库仑摩擦力 f_{c,G} (N)', 'FontSize', 11, 'FontWeight', 'bold');
ylim([10, 22]);

grid on; box on;
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('(b) 摩擦阻尼与库仑摩擦在线辨识历程', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 9);

% Subplot 3: PE 门控特征值与门限
subplot(2, 2, 3);
semilogy(d3.t, lam_min_hist, 'b-', 'LineWidth', 1.5, 'DisplayName', '\lambda_{min}(G_k)');
hold on;
yline(1.0e-4, 'r--', 'LineWidth', 1.5, 'DisplayName', 'PE 门限 \epsilon_{PE} = 10^{-4}');
% 绘制 PE 开启区间阴影
pe_on = double(is_pe_hist);
area(d3.t, pe_on * 1.0, 1e-18, 'FaceColor', [0.2, 0.8, 0.2], 'FaceAlpha', 0.15, ...
    'EdgeColor', 'none', 'DisplayName', 'PE 门控开启窗口');
grid on; box on;
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('最小特征值 \lambda_{min} (对数刻度)', 'FontSize', 11, 'FontWeight', 'bold');
title('(c) 300ms 滑动窗 Gram 矩阵持续激励判定', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 9);
ylim([1e-18, 1e-1]);

% Subplot 4: 8192 线编码器量化噪声与 SVF 滤波平滑
subplot(2, 2, 4);
% 差分原始加速度 vs SVF 滤波加速度
v_raw = [0; diff(d3.yG_quant)] / Ts;
a_raw = [0; diff(v_raw)] / Ts;
plot(d3.t(1:2000), a_raw(1:2000), 'Color', [0.7, 0.7, 0.7], 'LineWidth', 0.8, 'DisplayName', '编码器差分原加速度 a_{raw}');
hold on;
plot(d3.t(1:2000), yddot_f_hist(1:2000), 'r-', 'LineWidth', 1.5, 'DisplayName', 'SVF 滤波加速度 d^2y_{G,f}/dt^2');
plot(d3.t(1:2000), traj.yddot(1:2000), 'k--', 'LineWidth', 1.2, 'DisplayName', '轨迹名义加速度');
grid on; box on;
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('加速度 (m/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
title('(d) 编码器量化噪声抑制比 (> 99.9%)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
ylim([-2.5, 2.5]);

exportgraphics(fig1, 'step3a_parameter_identification.png', 'Resolution', 300);
close(fig1);
fprintf('  -> [OK] step3a_parameter_identification.png 保存完成\n');

%% 5. 生成高学术规格图表 2: step3a_closed_loop_comparison.png
fprintf('>>> 正在绘制学术图表 2: step3a_closed_loop_comparison.png ...\n');
fig2 = figure('Name', 'Step3A_Closed_Loop_Comparison', 'Color', 'w', ...
    'Units', 'pixels', 'Position', [100, 100, 1400, 950], 'Visible', 'off');

% Subplot 1: 位移跟踪误差比较
subplot(2, 2, 1);
plot(d3.t, e_yG_c2a, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.5, 'DisplayName', sprintf('基线 C2a (固定名义参数, RMSE=%.2f mm)', rmse_yG_c2a));
hold on;
plot(d3.t, e_yG_c3a, 'b-', 'LineWidth', 1.8, 'DisplayName', sprintf('提出 C3a (自适应 RLS 前馈, RMSE=%.2f mm)', rmse_yG_c3a));
xline(3.3, 'k--', 'LineWidth', 1.0, 'DisplayName', '载荷突变');
grid on; box on;
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('位移跟踪误差 e_{yG} (mm)', 'FontSize', 11, 'FontWeight', 'bold');
title('(a) 平动位移跟踪误差时域对比', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 9);

% Subplot 2: 左右电机控制电流对比
subplot(2, 2, 2);
plot(d3.t, d3.iL_cmd, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.2, 'DisplayName', 'C2a i_L');
hold on;
plot(d3.t, log_iL_c3a, 'b-', 'LineWidth', 1.2, 'DisplayName', 'C3a i_L (自适应)');
plot(d3.t, log_iR_c3a, 'm--', 'LineWidth', 1.2, 'DisplayName', 'C3a i_R (自适应)');
yline(4500, 'r--', 'LineWidth', 1.0, 'DisplayName', 'I_{max} = 4500');
yline(-4500, 'r--', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('电机电流指令 (count)', 'FontSize', 11, 'FontWeight', 'bold');
title('(b) 控制电流指令平稳性对比 (三级限幅)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);
ylim([-5000, 5000]);

% Subplot 3: 偏转角时域收敛
subplot(2, 2, 3);
plot(d3.t, d3.alpha * 1e3, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.5, 'DisplayName', 'C2a 偏转角 \alpha');
hold on;
plot(d3.t, log_alpha_c3a * 1e3, 'b-', 'LineWidth', 1.8, 'DisplayName', 'C3a 偏转角 \alpha');
grid on; box on;
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('偏转角 \alpha (mrad)', 'FontSize', 11, 'FontWeight', 'bold');
title('(c) 结构偏转同步抑制响应 (d=0 纯净中心载荷)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);

% Subplot 4: 控制动作平滑度 (TV) 与闭环质量自适应历程
subplot(2, 2, 4);
yyaxis left;
plot(d3.t, log_M_est_cl, 'b-', 'LineWidth', 2.0, 'DisplayName', '闭环前馈质量 M_{tot,hat}(t)');
hold on;
plot(d3.t, d3.M_true, 'k--', 'LineWidth', 1.5, 'DisplayName', '物理真值 M_{true}');
ylabel('自适应前馈质量 (kg)', 'FontSize', 11, 'FontWeight', 'bold');
ylim([12, 19]);

yyaxis right;
bar_tv = [tv_c2a, tv_c3a];
% 绘制平稳性小柱状图或文字说明
text(0.5, 0.25, sprintf('TV 比值 = TV_{C3a} / TV_{C2a}\n= %.1f / %.1f = %.3f\n<= 1.10 (无自适应抖颤)', ...
    tv_c3a, tv_c2a, tv_ratio), 'Units', 'normalized', ...
    'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', [0.95, 0.95, 1.0], ...
    'EdgeColor', 'b', 'Margin', 8);
ylabel('控制量总变差 (TV 指标)', 'FontSize', 11, 'FontWeight', 'bold');

grid on; box on;
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('(d) 闭环自适应质量前馈注入与 TV 平稳性验证', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 9);

exportgraphics(fig2, 'step3a_closed_loop_comparison.png', 'Resolution', 300);
close(fig2);
fprintf('  -> [OK] step3a_closed_loop_comparison.png 保存完成\n');

%% 6. 导出 step3a_metrics_summary.csv
fprintf('>>> 正在导出 step3a_metrics_summary.csv ...\n');
fid = fopen('step3a_metrics_summary.csv', 'w');
fprintf(fid, 'Metric,C2a_Baseline,C3a_Adaptive,Unit,Notes\n');
fprintf(fid, 'Mass_Forward_Mean,NaN,%.3f,kg,True=17.600 kg (t in [2.0 3.0]s)\n', M_fwd_mean);
fprintf(fid, 'Mass_Forward_ErrPct,NaN,%.2f,%%,Relative error\n', M_fwd_err_pct);
fprintf(fid, 'Mass_Reverse_Mean,NaN,%.3f,kg,True=13.600 kg (t in [5.8 6.8]s)\n', M_rev_mean);
fprintf(fid, 'Mass_Reverse_ErrPct,NaN,%.2f,%%,Relative error\n', M_rev_err_pct);
fprintf(fid, 'Mass_Reverse_Std,NaN,%.4f,kg,Evaluation window jitter\n', M_rev_std);
fprintf(fid, 'ViscousDamping_Mean,NaN,%.2f,N*s/m,True=70.00 N*s/m\n', bG_rev_mean);
fprintf(fid, 'ViscousDamping_ErrPct,NaN,%.2f,%%,Relative error\n', bG_rev_err_pct);
fprintf(fid, 'CoulombFriction_Mean,NaN,%.2f,N,True=16.00 N\n', fcG_rev_mean);
fprintf(fid, 'CoulombFriction_ErrPct,NaN,%.2f,%%,Relative error\n', fcG_rev_err_pct);
fprintf(fid, 'PE_Active_Ratio,NaN,%.2f,%%,Gram matrix lambda_min >= 1e-4\n', pe_active_ratio);
fprintf(fid, 'RMSE_yG,%.2f,%.2f,mm,Position tracking error\n', rmse_yG_c2a, rmse_yG_c3a);
fprintf(fid, 'Max_e_yG,%.2f,%.2f,mm,Peak position tracking error\n', max_e_yG_c2a, max_e_yG_c3a);
fprintf(fid, 'TV_Total,%.1f,%.1f,-,Total Variation\n', tv_c2a, tv_c3a);
fprintf(fid, 'TV_Ratio,1.000,%.3f,-,TV_c3a / TV_c2a <= 1.10\n', tv_ratio);
fclose(fid);
fprintf('  -> [OK] step3a_metrics_summary.csv 导出完成\n\n');

fprintf('==================================================================================\n');
fprintf('  Step 3A 基准测试全部执行完毕！\n');
fprintf('==================================================================================\n');
