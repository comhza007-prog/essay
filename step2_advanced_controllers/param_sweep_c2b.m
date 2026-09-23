%% PARAM_SWEEP_C2B.M - C2b 动态抗饱和参数全面扫描与消融分析
% =========================================================================
% 本脚本针对强限流饱和工况 (Imax = 4500 counts)，扫描抗饱和参数矩阵：
%   K_aw in [0, 1, 2, 5, 10, 20]
%   lambda_aw in [2, 5, 10, 20, 40]
% 特别注意：K_aw = 0 为完全同结构、仅关闭动态补偿的严格消融组 (等价于加了 16000 限幅的 C2a)。
%
% 统计全套学术评价指标：
% 1. 轨迹跟踪与同步: RMSE_yG, IAE_yG, Max_Sync, RMSE_Sync, IAE_sync, Max_Alpha
% 2. 抗饱和过程动态: 
%    - 退出饱和后最大超调 (Overshoot_yG, Post_Sat_Max_Sync)
%    - 退出饱和后恢复时间 (T_recovery)
%    - 滑模面范数峰值与时间积分 (Max_Sigma, Integral_Sigma)
%    - 控制量总变差 (TV_iR)
%    - 饱和残差时间积分与峰值 (Integral_Delta_i, Max_Abs_Delta_i, Max_Delta_i_Norm)
%    - 饱和比例 (Sat_Ratio)
% =========================================================================

clear; clc; close all;

addpath(fullfile(pwd, '..', 'step1_baseline_c0'));
[ctrl, mech, plant] = param_init();

Ts = ctrl.Ts;                     % 0.001 s
t_span = 3.5;
y_target = 1.0;
v_max = 0.6;
a_max = 1.5;
traj = trajectory_gen(t_span, Ts, y_target, v_max, a_max);
N_steps = length(traj.t);

% 物理不对称环境 (严苛工况: 偏心 3.0kg, 摩擦差 30%)
env.delta_m = 3.0;
env.d_load = 0.28;
env.delta_fric = 0.30;
env.Kf_L = mech.Kf;
env.Kf_R = mech.Kf;

Imax = 4500.0; % 严格限流

% 名义模型参数
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
ctrl_c2_base.aw_mode = 'external';

% 参数扫描范围
k_aw_grid = [0.0, 1.0, 2.0, 5.0, 10.0, 20.0];
lambda_aw_grid = [2.0, 5.0, 10.0, 20.0, 40.0];

results_sweep = [];
sweep_count = 0;

fprintf('========================================================================================================\n');
fprintf('                             C2b 抗饱和参数网格扫描与消融测试 (Imax = 4500)\n');
fprintf('========================================================================================================\n');
fprintf('%-8s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s\n', ...
    'K_aw', 'lambda_aw', 'RMSE_yG(mm)', 'IAE_yG', 'Max_Sync(mm)', 'IAE_sync', 'Max_Sigma', 'TV_iR', 'Int_Delta', 'Sat_Ratio');
fprintf('--------------------------------------------------------------------------------------------------------\n');

for ki = 1:length(k_aw_grid)
    for li = 1:length(lambda_aw_grid)
        sweep_count = sweep_count + 1;
        k_val = k_aw_grid(ki);
        lam_val = lambda_aw_grid(li);
        
        ctrl_test = ctrl_c2_base;
        ctrl_test.K_aw = k_val * eye(2);
        ctrl_test.lambda_aw = lam_val;
        
        x = zeros(4, 1);
        state_aw.z_aw = [0.0; 0.0];
        
        log_yG = zeros(1, N_steps);
        log_alpha = zeros(1, N_steps);
        log_yL = zeros(1, N_steps);
        log_yR = zeros(1, N_steps);
        log_iL_cmd = zeros(1, N_steps);
        log_iR_cmd = zeros(1, N_steps);
        log_iL_ideal = zeros(1, N_steps);
        log_iR_ideal = zeros(1, N_steps);
        log_delta_iL = zeros(1, N_steps);
        log_delta_iR = zeros(1, N_steps);
        log_sat = false(1, N_steps);
        log_sigma = zeros(2, N_steps);
        
        for k = 1:N_steps
            yG_c = x(1); alpha_c = x(2);
            yG_dot_c = x(3); alpha_dot_c = x(4);
            
            yL_c = yG_c - 0.5 * mech.Le * alpha_c;
            yR_c = yG_c + 0.5 * mech.Le * alpha_c;
            
            yd_t = traj.y(k);
            q_curr = [yG_c; alpha_c];
            qdot_curr = [yG_dot_c; alpha_dot_c];
            qd_curr = [yd_t; 0.0];
            qdot_d_curr = [traj.ydot(k); 0.0];
            qddot_d_curr = [traj.yddot(k); 0.0];
            
            [iL_cmd, iR_cmd, state_aw, info] = controller_c2b_aw( ...
                q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                ctrl_test, mech.Le, env.Kf_L, env.Kf_R, Imax, state_aw, Ts);
            
            x = gantry_dynamics_step(x, iL_cmd, iR_cmd, mech, plant, ...
                env.delta_m, env.d_load, env.delta_fric, Ts, env.Kf_L, env.Kf_R);
            
            log_yG(k) = yG_c;
            log_alpha(k) = alpha_c;
            log_yL(k) = yL_c;
            log_yR(k) = yR_c;
            log_iL_cmd(k) = iL_cmd;
            log_iR_cmd(k) = iR_cmd;
            log_iL_ideal(k) = info.i_request(1);
            log_iR_ideal(k) = info.i_request(2);
            log_delta_iL(k) = info.Delta_i_external(1);
            log_delta_iR(k) = info.Delta_i_external(2);
            log_sat(k) = info.is_sat;
            log_sigma(:, k) = info.sigma;
        end
        
        eyG = (log_yG - traj.y) * 1e3;   % mm
        esync = (log_yR - log_yL) * 1e3; % mm
        sigma_norm = sqrt(log_sigma(1,:).^2 + log_sigma(2,:).^2);
        delta_norm = sqrt(log_delta_iL.^2 + log_delta_iR.^2);
        
        % 标量综合指标计算
        rmse_yG = sqrt(mean(eyG.^2));
        iae_yG = trapz(traj.t, abs(eyG));
        max_sync = max(abs(esync));
        rmse_sync = sqrt(mean(esync.^2));
        iae_sync = trapz(traj.t, abs(esync));
        max_alpha = max(abs(log_alpha)) * 1e3;
        
        max_sigma = max(sigma_norm);
        int_sigma = trapz(traj.t, sigma_norm);
        tv_iR = sum(abs(diff(log_iR_cmd)));
        tv_iR_ideal = sum(abs(diff(log_iR_ideal)));
        int_delta_i = trapz(traj.t, delta_norm);
        sat_ratio = mean(log_sat) * 100.0;
        
        max_abs_delta = max([abs(log_delta_iL), abs(log_delta_iR)], [], 'all');
        max_delta_norm = max(delta_norm);
        
        % 退出饱和后超调与恢复时间
        last_sat_idx = find(log_sat, 1, 'last');
        if isempty(last_sat_idx)
            t_recovery = 0.0;
            overshoot_yG = 0.0;
            post_sat_max_sync = max_sync;
        else
            t_exit_sat = traj.t(last_sat_idx);
            overshoot_yG = max(0, max(log_yG) - y_target) * 1e3;
            post_sat_max_sync = max(abs(esync(last_sat_idx:end)));
            
            err_tail = abs(log_yG - y_target) * 1e3;
            idx_settle = find(err_tail > 1.0, 1, 'last');
            if isempty(idx_settle) || idx_settle <= last_sat_idx
                t_recovery = 0.0;
            else
                t_recovery = traj.t(min(idx_settle + 1, N_steps)) - t_exit_sat;
            end
        end
        
        res.k_aw = k_val;
        res.lambda_aw = lam_val;
        res.rmse_yG = rmse_yG;
        res.iae_yG = iae_yG;
        res.max_sync = max_sync;
        res.rmse_sync = rmse_sync;
        res.iae_sync = iae_sync;
        res.max_alpha = max_alpha;
        res.max_sigma = max_sigma;
        res.int_sigma = int_sigma;
        res.tv_iR = tv_iR;
        res.tv_iR_ideal = tv_iR_ideal;
        res.int_delta_i = int_delta_i;
        res.sat_ratio = sat_ratio;
        res.max_abs_delta = max_abs_delta;
        res.max_delta_norm = max_delta_norm;
        res.overshoot_yG = overshoot_yG;
        res.post_sat_max_sync = post_sat_max_sync;
        res.t_recovery = t_recovery;
        
        results_sweep = [results_sweep; res];
        
        fprintf('%8.1f | %10.1f | %10.3f | %10.3f | %12.4f | %10.4f | %10.4f | %10.1f | %10.1f | %9.1f%%\n', ...
            k_val, lam_val, rmse_yG, iae_yG, max_sync, iae_sync, max_sigma, tv_iR, int_delta_i, sat_ratio);
    end
end

fprintf('========================================================================================================\n\n');

%% 保存扫描结果到 CSV
sweep_table = struct2table(results_sweep);
writetable(sweep_table, 'c2b_param_sweep.csv');
save('c2b_param_sweep.mat', 'results_sweep', 'sweep_table');
fprintf('[OK] 参数扫描结果已保存至 c2b_param_sweep.csv 与 c2b_param_sweep.mat\n');
