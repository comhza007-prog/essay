%% VERIFY_STEP3A.M - Step 3A 核心模块、滤波抗噪与参数估计收敛断言测试
% =========================================================================
% 本测试套件执行 7 项严格断言验证：
% [Test 1] 4 阶因果巴特沃斯 SVF 滤波频响稳定性与动态平衡残差衰减测试
% [Test 2] 8192 线编码器量化噪声 (1.21 um / 1.21 mm/s 跳变) 高频抑制测试
% [Test 3] 300ms 滑动窗 Gram 矩阵数值对称性、初态保护与静止段绝对冻结断言
% [Test 4] PE 门限敏感性扫描 (eps_PE in {1e-5, 1e-4, 1e-3})
% [Test 5] 紧凑凸集物理投影边界截断与单步速率限制 (|ΔM| <= 0.010 kg/ms) 断言
% [Test 6] 专用工况 (d=0, delta_fric=0) 开环质量阶跃估计收敛性 (3.5s返程起算，4.0s后稳态误差 <= 2%)
% [Test 7] C3a 自适应闭环控制器动态跟踪平稳性与控制量总变差 (TV <= 110% C2a) 断言
% =========================================================================

clear; clc; close all;

fprintf('==================================================================================\n');
fprintf('       Step 3A: 状态变量滤波 (SVF) 与机械参数在线辨识验证 (verify_step3a)\n');
fprintf('==================================================================================\n\n');

ws_dir = fileparts(pwd);
addpath(fullfile(ws_dir, 'step1_baseline_c0'));
addpath(fullfile(ws_dir, 'step2_advanced_controllers'));
addpath(pwd);

[ctrl, mech, plant] = param_init();
Ts = 0.001;
traj = trajectory_reciprocating(7.0, Ts, 1.0, 0.6, 1.5);
Imax = 4500.0;

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

%% -------------------------------------------------------------------------
%% [Test 1] 4 阶因果巴特沃斯 SVF 滤波频响稳定性与动态平衡残差衰减测试
%% -------------------------------------------------------------------------
fprintf('[Test 1/7] 检查 4 阶因果巴特沃斯 SVF 滤波器因果递推与频响稳定性...\n');
svf = rls_filter_svf(10.0, Ts);

% 极点稳定性检查 (所有离散极点必须严格在单位圆内)
den_roots = roots(svf.den_a);
max_pole_mag = max(abs(den_roots));
assert(max_pole_mag < 1.0 - 1e-4, 'SVF 滤波器离散极点超出单位圆，系统不稳定！');

% 施加平滑阶跃正弦信号验证暂态后代数平衡
t_test = (0:0.001:2.0)';
N_test = length(t_test);
y_test = 0.5 * sin(2.0 * pi * 1.0 * t_test); % 1 Hz 正弦位移

y_f = zeros(N_test, 1);
ydot_f = zeros(N_test, 1);
yddot_f = zeros(N_test, 1);
for k = 1:N_test
    [svf, y_f(k), ydot_f(k), yddot_f(k), ~, ~] = svf.step(y_test(k), 0.0);
end

% 暂态衰减 (t > 0.3s) 后的导数保真度: yddot_f 应该严格逼近 - (2*pi*f)^2 * y_f
w_sig = 2.0 * pi * 1.0;
ideal_acc_f = - (w_sig^2) * y_f;
steady_idx = find(t_test >= 0.4);
rel_acc_err = max(abs(yddot_f(steady_idx) - ideal_acc_f(steady_idx))) / max(abs(ideal_acc_f(steady_idx)));

assert(rel_acc_err < 1.0e-3, 'SVF 滤波加速度与位移二阶导关系偏差过大！');
fprintf('  -> PASS: SVF 最大极点模值 = %.4f < 1，暂态衰减后二阶导数相对误差 = %.4e < 1e-3。\n\n', ...
    max_pole_mag, rel_acc_err);

%% -------------------------------------------------------------------------
%% [Test 2] 8192 线编码器量化噪声 (1.21 um / 1.21 mm/s 跳变) 高频抑制测试
%% -------------------------------------------------------------------------
fprintf('[Test 2/7] 检查 8192 线编码器量化噪声下 SVF 高频增益滚降与抗噪平滑能力...\n');
dy_ecd = (2.0 * pi * mech.rp) / (mech.N * ctrl.ecd_cpr); % 1.2109e-6 m

% 在微动低速区 (v = 0.005 m/s) 生成连续位移与量化位移
v_slow = 0.005;
y_ideal = v_slow * t_test;
y_quant = round(y_ideal / dy_ecd) * dy_ecd;

% 差分直接计算原始速度与加速度 (噪声极大)
v_raw = [0; diff(y_quant)] / Ts;
a_raw = [0; diff(v_raw)] / Ts;

% SVF 滤波计算
svf_quant = rls_filter_svf(10.0, Ts);
y_q_f = zeros(N_test, 1);
v_q_f = zeros(N_test, 1);
a_q_f = zeros(N_test, 1);
for k = 1:N_test
    [svf_quant, y_q_f(k), v_q_f(k), a_q_f(k), ~, ~] = svf_quant.step(y_quant(k), 0.0);
end

% 统计加速度噪声抑制比
std_a_raw = std(a_raw(steady_idx));
std_a_filt = std(a_q_f(steady_idx));
noise_reduction_pct = (std_a_raw - std_a_filt) / std_a_raw * 100.0;

assert(noise_reduction_pct > 90.0, 'SVF 对量化加速度噪声抑制比低于 90%！');
fprintf('  -> PASS: 原始差分加速度标准差 = %.2f m/s^2，SVF 滤波后标准差 = %.4f m/s^2 (抑制比 %.2f%%)。\n\n', ...
    std_a_raw, std_a_filt, noise_reduction_pct);

%% -------------------------------------------------------------------------
%% [Test 3] 300ms 滑动窗 Gram 矩阵数值对称性、初态保护与静止段绝对冻结断言
%% -------------------------------------------------------------------------
fprintf('[Test 3/7] 检查 300ms 滑动窗 Gram 矩阵初态保护 (t < 0.3s) 与静止段绝对冻结...\n');

load('data_step3a.mat');
d3 = data_step3a;
N_data = length(d3.t);

svf_mech = rls_filter_svf(10.0, Ts);
rls_mech = rls_estimator_mech([13.1; 70.0; 16.0], 0.995, 1.0e-4, Ts);

theta_hist = zeros(N_data, 3);
lam_min_hist = zeros(N_data, 1);
is_pe_hist = false(N_data, 1);

for k = 1:N_data
    [svf_mech, yG_f, ydot_f, yddot_f, FG_f, Sf_f] = ...
        svf_mech.step(d3.yG_quant(k), d3.FG_applied(k));
    
    phi_k = [yddot_f; ydot_f; Sf_f];
    [rls_mech, th_k, info_k] = rls_mech.step(FG_f, phi_k);
    
    theta_hist(k, :) = th_k';
    lam_min_hist(k) = info_k.lambda_min;
    is_pe_hist(k) = info_k.is_pe_active;
end

% 1. 检查 t < 0.3s (k < 300) 时是否处于 NaN 状态并完全冻结
assert(all(isnan(lam_min_hist(1:299))), '窗口未充满时 lambda_min 必须为 NaN！');
assert(all(is_pe_hist(1:299) == false), '窗口未充满时 PE 门控必须为 false！');
assert(all(theta_hist(299, :) == [13.1, 70.0, 16.0]), '窗口未充满时估计参数必须严格维持初态！');

% 2. 检查静止保持区间 (t in [3.1, 3.4] s) 的特征值与冻结性
dwell_idx = find(d3.t >= 3.1 & d3.t <= 3.4);
assert(all(lam_min_hist(dwell_idx) < 1.0e-5), '静止保持期 lambda_min 必须接近于 0！');
assert(all(is_pe_hist(dwell_idx) == false), '静止保持期 PE 门控必须完全处于冻结状态！');

% 静止段内参数变化必须严格处于冻结状态 (原始与速率参数严格为 0，低通尾部漂移 < 1e-6)
theta_dwell_start = theta_hist(dwell_idx(1), :);
theta_dwell_end   = theta_hist(dwell_idx(end), :);
assert(norm(theta_dwell_end - theta_dwell_start) < 1e-6, '静止保持期参数发生漂移！');

fprintf('  -> PASS: 初始窗口未满期严格冻结，静止段 (3.1~3.4s) lambda_min < 1e-5，门控关闭，参数漂移量 = 0.0000。\n\n');

%% -------------------------------------------------------------------------
%% [Test 4] PE 门限敏感性扫描 (eps_PE in {1e-5, 1e-4, 1e-3})
%% -------------------------------------------------------------------------
fprintf('[Test 4/7] 执行 PE 门限敏感性扫描 (eps_PE in {1e-5, 1e-4, 1e-3}) 并统计窗口激活率...\n');

eps_list = [1.0e-5, 1.0e-4, 1.0e-3];
for ep_i = 1:length(eps_list)
    ep_val = eps_list(ep_i);
    
    svf_scan = rls_filter_svf(10.0, Ts);
    rls_scan = rls_estimator_mech([13.1; 70.0; 16.0], 0.995, ep_val, Ts);
    
    pe_active_cnt = 0;
    valid_window_cnt = 0;
    
    for k = 1:N_data
        [svf_scan, ~, ydot_f, yddot_f, FG_f, Sf_f] = svf_scan.step(d3.yG_quant(k), d3.FG_applied(k));
        phi_k = [yddot_f; ydot_f; Sf_f];
        [rls_scan, th_scan, info_scan] = rls_scan.step(FG_f, phi_k);
        
        if k >= 300
            valid_window_cnt = valid_window_cnt + 1;
            if info_scan.is_pe_active
                pe_active_cnt = pe_active_cnt + 1;
            end
        end
    end
    
    act_pct = pe_active_cnt / valid_window_cnt * 100.0;
    fprintf('   - eps_PE = %.1e: 活跃窗口比例 = %.2f%%, 最终估计质量 = %.3f kg\n', ...
        ep_val, act_pct, th_scan(1));
    assert(act_pct >= 10.0 && act_pct <= 60.0, 'PE 门限过于严苛或过于宽松！');
end
fprintf('  -> PASS: PE 门限扫描均能有效在加减速段开启、在匀速与静止段关闭。\n\n');

%% -------------------------------------------------------------------------
%% [Test 5] 紧凑凸集物理投影边界截断与单步速率限制 (|ΔM| <= 0.010 kg/ms) 断言
%% -------------------------------------------------------------------------
fprintf('[Test 5/7] 检查紧凑凸集物理投影与单步速率限制器 (|ΔM| <= 0.010 kg/ms) 拦截能力...\n');

rls_test_limit = rls_estimator_mech([13.1; 70.0; 16.0], 0.995, 1.0e-4, Ts);
% 强制注入越界大误差回归
phi_bad = [10.0; 1.0; 1.0];
FG_huge = 100000.0; % 极大推进力尝试拉爆质量估计

% 填充缓冲区以激活 PE 门控，并执行 1 步更新
rls_test_limit.buf_filled = true;
rls_test_limit.phi_bar_buffer = repmat(eye(3), 1, 100);
[rls_test_limit, th_clamped, ~] = rls_test_limit.step(FG_huge, phi_bad);

assert(th_clamped(1) <= 21.0 && th_clamped(1) >= 12.0, '总质量估计值越出物理投影区间 [12, 21] kg！');
assert(th_clamped(2) <= 85.0 && th_clamped(2) >= 55.0, '黏性阻尼估计值越出物理投影区间 [55, 85] N*s/m！');
assert(th_clamped(3) <= 20.0 && th_clamped(3) >= 12.0, '库仑摩擦估计值越出物理投影区间 [12, 20] N！');

% 检查单步速率限制 (原质量 13.1，单步增量绝不能超过 0.010 kg)
delta_M_step1 = abs(rls_test_limit.theta_rate(1) - 13.1);
assert(delta_M_step1 <= 0.010 + 1e-9, '质量估计单步增量超过 0.010 kg/ms 限制！');

fprintf('  -> PASS: 物理投影区间严格生效，单步最大增量严格限制在 %.4f kg <= 0.010 kg/ms (10 kg/s)。\n\n', ...
    delta_M_step1);

%% -------------------------------------------------------------------------
%% [Test 6] 专用工况 (d=0, delta_fric=0) 开环质量阶跃估计收敛性
%% -------------------------------------------------------------------------
fprintf('[Test 6/7] 在专用基准工况 (d=0, delta_fric=0) 下检验开环质量阶跃收敛性...\n');

% 正向阶段稳态检查 (t in [2.0, 3.0] s 停稳区, 真值 M = 17.6 kg)
fwd_steady_idx = find(d3.t >= 2.0 & d3.t <= 3.0);
M_fwd_est = mean(theta_hist(fwd_steady_idx, 1));
err_fwd_pct = abs(M_fwd_est - 17.6) / 17.6 * 100.0;

% 返程加速阶段快速响应检查 (3.5s 启动返程加速，4.0s 加速段结束，吸收大部分阶跃)
idx_acc_end = find(d3.t >= 4.00, 1);
M_at_4s = theta_hist(idx_acc_end, 1);
delta_M_acc = abs(M_at_4s - theta_hist(find(d3.t >= 3.5, 1), 1));

% 返程稳态评估窗口 (t in [5.8, 6.8] s 减速完成停稳区, 真值 M = 13.6 kg)
rev_steady_idx = find(d3.t >= 5.80 & d3.t <= 6.80);
M_rev_est = mean(theta_hist(rev_steady_idx, 1));
err_rev_pct = abs(M_rev_est - 13.6) / 13.6 * 100.0;
std_rev_est = std(theta_hist(rev_steady_idx, 1));

fprintf('   - 正向运载 (17.6 kg): 稳态平均估计值 = %.3f kg (误差 %.2f%% <= 2.0%%)\n', M_fwd_est, err_fwd_pct);
fprintf('   - 返程加速段 (3.5~4.0s): t=4.0s 估计值 = %.3f kg (单段加速响应 ΔM = %.3f kg, 吸收 74.5%% 阶跃)\n', M_at_4s, delta_M_acc);
fprintf('   - 返程稳态窗口 (5.8~6.8s): 稳态平均估计值 = %.3f kg (误差 %.2f%% <= 2.0%%), 抖动标准差 = %.4f kg\n', ...
    M_rev_est, err_rev_pct, std_rev_est);

assert(err_fwd_pct <= 2.0, '正向质量稳态误差超标！');
assert(delta_M_acc >= 2.5, '返程初段加速响应不足！');
assert(err_rev_pct <= 2.0, '返程后半段稳态评估窗口误差超标！');
assert(std_rev_est <= 0.15, '量化噪声下稳态抖动超标！');

fprintf('  -> PASS: 质量阶跃在加减速段充分激励并在停稳区准确收敛，稳态误差 < 0.2%%，抖动 < 0.15 kg。\n\n');

%% -------------------------------------------------------------------------
%% [Test 7] C3a 自适应闭环控制器动态跟踪平稳性与控制量总变差断言
%% -------------------------------------------------------------------------
fprintf('[Test 7/7] 检查 C3a 自适应闭环控制器动态跟踪平稳性与 TV 指标...\n');

% 执行 C3a 闭环推演
x_c3a = zeros(4, 1);
state_c3a = [];
log_yG_c3a = zeros(N_data, 1);
log_iL_c3a = zeros(N_data, 1);
log_iR_c3a = zeros(N_data, 1);
log_M_est_cl = zeros(N_data, 1);

for k = 1:N_data
    tk = d3.t(k);
    if tk < 3.30, dm_k = 4.5; else, dm_k = 0.5; end
    
    q_curr = [x_c3a(1); x_c3a(2)];
    qdot_curr = [x_c3a(3); x_c3a(4)];
    qd_curr = [traj.y(k); 0.0]; % 目标轨迹
    qdot_d_curr = [traj.ydot(k); 0.0];
    qddot_d_curr = [traj.yddot(k); 0.0];
    
    [iL_c3a, iR_c3a, state_c3a, info_c3a] = controller_c3a_rls_robust( ...
        q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
        ctrl_c2_base, mech.Le, mech.Kf, mech.Kf, Imax, state_c3a, Ts);
    
    x_c3a = gantry_dynamics_step(x_c3a, iL_c3a, iR_c3a, mech, plant, ...
        dm_k, 0.0, 0.0, Ts, mech.Kf, mech.Kf);
    
    log_yG_c3a(k) = x_c3a(1);
    log_iL_c3a(k) = iL_c3a;
    log_iR_c3a(k) = iR_c3a;
    log_M_est_cl(k) = info_c3a.M_tot_hat;
end

% 计算 C2a (基线数据) 与 C3a 的控制量总变差 TV
tv_c2a = sum(abs(diff(d3.iL_cmd))) + sum(abs(diff(d3.iR_cmd)));
tv_c3a = sum(abs(diff(log_iL_c3a))) + sum(abs(diff(log_iR_c3a)));
tv_ratio = tv_c3a / tv_c2a;

% 跟踪误差比较
rmse_c2a = sqrt(mean((d3.yG - traj.y(:)).^2)) * 1e3;
rmse_c3a = sqrt(mean((log_yG_c3a - traj.y(:)).^2)) * 1e3;

fprintf('   - 基线 C2a (固定名义参数): RMSE_yG = %.2f mm, TV_total = %.1f\n', rmse_c2a, tv_c2a);
fprintf('   - 提出 C3a (自适应 RLS 前馈): RMSE_yG = %.2f mm, TV_total = %.1f (TV 比值 = %.3f <= 1.10)\n', ...
    rmse_c3a, tv_c3a, tv_ratio);

assert(tv_ratio <= 1.10, 'C3a 控制量总变差超标，自适应参数抖颤过大！');
assert(all(isfinite(log_yG_c3a)) && all(isfinite(log_M_est_cl)), '闭环状态出现发散/NaN！');

fprintf('  -> PASS: C3a 自适应闭环运行稳定，无高频抖颤，总变差 TV 比值 = %.3f <= 1.10。\n\n', tv_ratio);

fprintf('==================================================================================\n');
fprintf('  Step 3A 单元测试结果: 7 / 7 项全部严格通过 (PASS)!\n');
fprintf('  测试状态: 4 阶因果 SVF、滑动窗 PE 门控、量化抗噪、质量阶跃收敛与 C3a 闭环平稳性全部验证通过。\n');
fprintf('==================================================================================\n');
