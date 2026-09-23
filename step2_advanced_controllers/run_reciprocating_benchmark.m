%% RUN_RECIPROCATING_BENCHMARK.M - 往复换向全工况基准对比测试 (2x2消融矩阵 + 工程基线)
% =========================================================================
% 本脚本在往复换向工况 (0 -> 1.0m -> 0.0m, 总时长 7.0s) 下，考核 6 种控制器：
% 1. C0: 独立级联 PID (工程基线)
% 2. C1: 传统交叉耦合 CCC (速度补偿基线)
% 3. C2a: 广义误差面鲁棒控制 (独立限幅截断, 无抗饱和)
% 4. C2a-SyncAlloc: 广义误差面鲁棒控制 (推力空间同步优先约束分配, 无抗饱和)
% 5. C2b: 动态抗饱和鲁棒控制 (独立限幅截断, 提出)
% 6. C2b-SyncAlloc: 动态抗饱和 + 推力空间同步优先约束分配 (提出复合方法)
%
% 构成 2x2 因子完全消融体系：
%                    独立截断 (Independent)       同步优先分配 (SyncAlloc)
%   无动态抗饱和      C2a                         C2a-SyncAlloc
%   含动态抗饱和      C2b                         C2b-SyncAlloc
%
% 双实验工况：
% - Group 1: Imax = 16000 counts (充裕无饱和基准校验)
% - Group 2: Imax = 4500 counts  (强限流往复换向深度考核)
% =========================================================================

if ~exist('is_test_runner', 'var')
    clear; clc; close all;
end

%% 1. 参数加载与往复轨迹规划
addpath(fullfile(pwd, '..', 'step1_baseline_c0'));
[ctrl, mech, plant] = param_init();

Ts = ctrl.Ts;                     % 0.001 s
t_span_total = 7.0;               % 7.0 s
y_target = 1.0;                   % 1.0 m
v_max = 0.6;                      % 0.6 m/s
a_max = 1.5;                      % 1.5 m/s^2

traj = trajectory_reciprocating(t_span_total, Ts, y_target, v_max, a_max);
N_steps = length(traj.t);
t_half_idx = round(traj.t_half / Ts) + 1;

m_to_ecd = (mech.N * ctrl.ecd_cpr) / (2.0 * pi * mech.rp);
ms_to_rpm = (60.0 * mech.N) / (2.0 * pi * mech.rp);

%% 2. 统一物理模型非对称环境
env.delta_m = 3.0;                % 右侧偏心质量 3.0 kg
env.d_load = 0.28;                % 偏心距 0.28 m
env.delta_fric = 0.30;            % 右侧摩擦增加 30%
env.Kf_L = mech.Kf;
env.Kf_R = mech.Kf;

%% 3. 控制器特定参数配置
% C1: 传统交叉耦合
ctrl_c1 = ctrl;
ctrl_c1.Kp_sync = 3.5;            % 1/s
ctrl_c1.Kd_sync = 0.08;

% C2 基础参数 (名义模型注入 5%~10% 失配)
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

% C2b 抗饱和增益
ctrl_c2b_base = ctrl_c2_base;
ctrl_c2b_base.K_aw = 20.0 * eye(2);
ctrl_c2b_base.lambda_aw = 20.0;
ctrl_c2b_base.aw_mode = 'external';

%% 4. 定义对比实验矩阵 (2组限流 x 6种控制器 = 12项实验)
groups(1).name = 'Group 1: 充裕电流限幅 (Imax = 16000)';
groups(1).Imax = 16000.0;

groups(2).name = 'Group 2: 严格电流限幅 (Imax = 4500)';
groups(2).Imax = 4500.0;

ctrl_names = {'C0 (独立级联 PID)', ...
              'C1 (传统交叉耦合 CCC)', ...
              'C2a (误差面鲁棒 - 独立截断)', ...
              'C2a-SyncAlloc (误差面鲁棒 - 同步优先分配)', ...
              'C2b (动态抗饱和 - 独立截断)', ...
              'C2b-SyncAlloc (动态抗饱和 - 同步优先分配)'};

ctrl_colors = {[0.0, 0.45, 0.74], ...      % 经典蓝
               [0.85, 0.33, 0.10], ...      % 活力橙
               [0.47, 0.67, 0.19], ...      % 草绿 (C2a)
               [0.10, 0.65, 0.60], ...      % 青绿 (C2a-SyncAlloc)
               [0.49, 0.18, 0.56], ...      % 紫罗兰 (C2b)
               [0.80, 0.00, 0.50]};        % 洋红 (C2b-SyncAlloc)

line_styles = {'-', '-', '--', '-', '--', '-'};
line_widths = [1.2, 1.2, 1.3, 1.4, 1.4, 1.6];

results_recip = struct([]);
csv_records = {};

exp_count = 0;

for g_idx = 1:length(groups)
    Imax_current = groups(g_idx).Imax;
    fprintf('\n>>> 正在运行往复基准测试: %s <<<\n', groups(g_idx).name);
    
    for c_id = 1:6
        exp_count = exp_count + 1;
        fprintf('  - 正在推演: %s ...\n', ctrl_names{c_id});
        
        x = zeros(4, 1);
        
        % 初始化 PID 状态
        s_ang_L = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, ctrl.ang_base_max_out, ctrl.ang_max_iout);
        s_ang_R = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, ctrl.ang_base_max_out, ctrl.ang_max_iout);
        s_spd_L = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, ctrl.spd_max_out, ctrl.spd_max_iout);
        s_spd_R = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, ctrl.spd_max_out, ctrl.spd_max_iout);
        states_c1.pid_ang_L = s_ang_L; states_c1.pid_ang_R = s_ang_R;
        states_c1.pid_spd_L = s_spd_L; states_c1.pid_spd_R = s_spd_R;
        
        % 初始化抗饱和状态
        state_c2b.z_aw = [0.0; 0.0];
        state_c2b_sync.z_aw = [0.0; 0.0];
        
        % 记录缓存
        log_yG = zeros(1, N_steps);
        log_alpha = zeros(1, N_steps);
        log_yL = zeros(1, N_steps);
        log_yR = zeros(1, N_steps);
        log_iL_cmd = zeros(1, N_steps);
        log_iR_cmd = zeros(1, N_steps);
        log_iL_ideal = zeros(1, N_steps);
        log_iR_ideal = zeros(1, N_steps);
        log_delta_iL_alloc = zeros(1, N_steps);
        log_delta_iR_alloc = zeros(1, N_steps);
        log_delta_iL_ext = zeros(1, N_steps);
        log_delta_iR_ext = zeros(1, N_steps);
        log_delta_iL_tot = zeros(1, N_steps);
        log_delta_iR_tot = zeros(1, N_steps);
        log_sat_alloc = zeros(1, N_steps);
        log_sat_ext = zeros(1, N_steps);
        log_sat_any = zeros(1, N_steps);
        
        log_Talpha_des = zeros(1, N_steps);
        log_Talpha_alloc = zeros(1, N_steps);
        log_FG_des = zeros(1, N_steps);
        log_FG_alloc = zeros(1, N_steps);
        log_z_aw = zeros(2, N_steps);
        
        for k = 1:N_steps
            yG_c = x(1); alpha_c = x(2);
            yG_dot_c = x(3); alpha_dot_c = x(4);
            
            yL_c = yG_c - 0.5 * mech.Le * alpha_c;
            yR_c = yG_c + 0.5 * mech.Le * alpha_c;
            vL_c = yG_dot_c - 0.5 * mech.Le * alpha_dot_c;
            vR_c = yG_dot_c + 0.5 * mech.Le * alpha_dot_c;
            
            yd_t = traj.y(k);
            tk = traj.t(k);
            
            % 往复双程速度梯度限幅 (在两个单程起步与终点均平滑约束)
            ecd_L = yL_c * m_to_ecd;
            ecd_tar_L = yd_t * m_to_ecd;
            err_rev = abs(ecd_tar_L - ecd_L) / ctrl.ecd_cpr;
            if tk < traj.t_half
                stroke_dist = abs(ecd_L);
            else
                stroke_dist = abs(ecd_L - y_target * m_to_ecd);
            end
            dyn_max = min(err_rev, stroke_dist / ctrl.ecd_cpr) * ctrl.ang_gradient_slope;
            dyn_max = max(ctrl.ang_gradient_min_out, min(ctrl.ang_gradient_max_out, dyn_max));
            
            q_curr = [yG_c; alpha_c];
            qdot_curr = [yG_dot_c; alpha_dot_c];
            qd_curr = [yd_t; 0.0];
            qdot_d_curr = [traj.ydot(k); 0.0];
            qddot_d_curr = [traj.yddot(k); 0.0];
            
            switch c_id
                case 1
                    % --- C0: 独立级联 PID ---
                    ecd_R = -yR_c * m_to_ecd;
                    ecd_tar_R = -yd_t * m_to_ecd;
                    s_ang_L.max_out = dyn_max; s_ang_R.max_out = dyn_max;
                    
                    [rpm_L, s_ang_L] = pid_calc(s_ang_L, ecd_L, ecd_tar_L);
                    [rpm_R, s_ang_R] = pid_calc(s_ang_R, ecd_R, ecd_tar_R);
                    [iL_raw, s_spd_L, iL_unsat] = pid_calc(s_spd_L,  vL_c * ms_to_rpm, rpm_L);
                    [iR_raw, s_spd_R, iR_unsat] = pid_calc(s_spd_R, -vR_c * ms_to_rpm, rpm_R);
                    
                    [iL_cmd, iR_cmd, Delta_i_ext, is_sat_ext] = saturation_model.hard_sat(iL_raw, iR_raw, Imax_current);
                    i_request = [iL_unsat; iR_unsat];
                    iL_ideal = i_request(1);
                    iR_ideal = i_request(2);
                    Delta_i_alloc = [0.0; 0.0];
                    is_sat_alloc = false;
                    Delta_i_tot = [iL_cmd; iR_cmd] - i_request;
                    is_sat_any = any(abs(Delta_i_tot) > 1e-6);
                    
                case 2
                    % --- C1: 传统交叉耦合 CCC ---
                    [iL_cmd, iR_cmd, states_c1, info_c1] = controller_c1_ccc( ...
                        yL_c, yR_c, vL_c, vR_c, yd_t, dyn_max, ctrl_c1, mech, Imax_current, states_c1);
                    iL_ideal = info_c1.i_request(1);
                    iR_ideal = info_c1.i_request(2);
                    Delta_i_alloc = [0.0; 0.0];
                    is_sat_alloc = false;
                    Delta_i_ext = info_c1.Delta_i_external;
                    Delta_i_tot = info_c1.Delta_i_total;
                    is_sat_ext = info_c1.is_sat_external;
                    is_sat_any = info_c1.is_sat_any;
                    
                case 3
                    % --- C2a: 广义误差面鲁棒 (独立截断) ---
                    [iL_cmd, iR_cmd, info_c2a] = controller_c2a_robust( ...
                        q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                        ctrl_c2_base, mech.Le, env.Kf_L, env.Kf_R, Imax_current);
                    iL_ideal = info_c2a.i_request(1);
                    iR_ideal = info_c2a.i_request(2);
                    Delta_i_alloc = [0.0; 0.0];
                    is_sat_alloc = false;
                    Delta_i_ext = info_c2a.Delta_i_external;
                    Delta_i_tot = info_c2a.Delta_i_total;
                    is_sat_ext = info_c2a.is_sat_external;
                    is_sat_any = info_c2a.is_sat_any;
                    
                    log_Talpha_des(k) = info_c2a.v_beta(2);
                    log_Talpha_alloc(k) = info_c2a.v_actual(2);
                    log_FG_des(k) = info_c2a.v_beta(1);
                    log_FG_alloc(k) = info_c2a.v_actual(1);
                    
                case 4
                    % --- C2a-SyncAlloc: 广义误差面鲁棒 (推力空间同步优先分配) ---
                    [iL_cmd, iR_cmd, info_c2a_sync] = controller_c2a_sync_alloc( ...
                        q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                        ctrl_c2_base, mech.Le, env.Kf_L, env.Kf_R, Imax_current);
                    iL_ideal = info_c2a_sync.i_request(1);
                    iR_ideal = info_c2a_sync.i_request(2);
                    Delta_i_alloc = info_c2a_sync.Delta_i_alloc;
                    Delta_i_ext = info_c2a_sync.Delta_i_external; % [0.0; 0.0], 无下游硬件截断
                    Delta_i_tot = info_c2a_sync.Delta_i_total;
                    is_sat_alloc = info_c2a_sync.is_sat_alloc;
                    is_sat_ext = info_c2a_sync.is_sat_external;
                    is_sat_any = info_c2a_sync.is_sat_any;
                    
                    log_Talpha_des(k) = info_c2a_sync.v_beta(2);
                    log_Talpha_alloc(k) = info_c2a_sync.v_actual(2);
                    log_FG_des(k) = info_c2a_sync.v_beta(1);
                    log_FG_alloc(k) = info_c2a_sync.v_actual(1);
                    
                case 5
                    % --- C2b: 动态抗饱和 (独立截断) ---
                    [iL_cmd, iR_cmd, state_c2b, info_c2b] = controller_c2b_aw( ...
                        q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                        ctrl_c2b_base, mech.Le, env.Kf_L, env.Kf_R, Imax_current, state_c2b, Ts);
                    iL_ideal = info_c2b.i_request(1);
                    iR_ideal = info_c2b.i_request(2);
                    Delta_i_alloc = [0.0; 0.0];
                    is_sat_alloc = false;
                    Delta_i_ext = info_c2b.Delta_i_external;
                    Delta_i_tot = info_c2b.Delta_i_total;
                    is_sat_ext = info_c2b.is_sat_external;
                    is_sat_any = info_c2b.is_sat_any;
                    
                    log_Talpha_des(k) = info_c2b.v_cmd(2);
                    log_Talpha_alloc(k) = info_c2b.v_actual(2);
                    log_FG_des(k) = info_c2b.v_cmd(1);
                    log_FG_alloc(k) = info_c2b.v_actual(1);
                    log_z_aw(:, k) = info_c2b.z_aw;
                    
                case 6
                    % --- C2b-SyncAlloc: 动态抗饱和 + 推力空间同步优先分配 (提出复合方法) ---
                    [iL_cmd, iR_cmd, state_c2b_sync, info_c2b_sync] = controller_c2b_sync_alloc( ...
                        q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                        ctrl_c2b_base, mech.Le, env.Kf_L, env.Kf_R, Imax_current, state_c2b_sync, Ts);
                    iL_ideal = info_c2b_sync.i_request(1);
                    iR_ideal = info_c2b_sync.i_request(2);
                    Delta_i_alloc = info_c2b_sync.Delta_i_alloc;
                    Delta_i_ext = info_c2b_sync.Delta_i_external; % [0.0; 0.0], 无下游硬件截断
                    Delta_i_tot = info_c2b_sync.Delta_i_total;
                    is_sat_alloc = info_c2b_sync.is_sat_alloc;
                    is_sat_ext = info_c2b_sync.is_sat_external;
                    is_sat_any = info_c2b_sync.is_sat_any;
                    
                    log_Talpha_des(k) = info_c2b_sync.v_cmd(2);
                    log_Talpha_alloc(k) = info_c2b_sync.v_actual(2);
                    log_FG_des(k) = info_c2b_sync.v_cmd(1);
                    log_FG_alloc(k) = info_c2b_sync.v_actual(1);
                    log_z_aw(:, k) = info_c2b_sync.z_aw;
            end
            
            % 动力学推演
            x = gantry_dynamics_step(x, iL_cmd, iR_cmd, mech, plant, ...
                env.delta_m, env.d_load, env.delta_fric, Ts, env.Kf_L, env.Kf_R);
            
            log_yG(k) = yG_c;
            log_alpha(k) = alpha_c;
            log_yL(k) = yL_c;
            log_yR(k) = yR_c;
            log_iL_cmd(k) = iL_cmd;
            log_iR_cmd(k) = iR_cmd;
            log_iL_ideal(k) = iL_ideal;
            log_iR_ideal(k) = iR_ideal;
            log_delta_iL_alloc(k) = Delta_i_alloc(1);
            log_delta_iR_alloc(k) = Delta_i_alloc(2);
            log_delta_iL_ext(k) = Delta_i_ext(1);
            log_delta_iR_ext(k) = Delta_i_ext(2);
            log_delta_iL_tot(k) = Delta_i_tot(1);
            log_delta_iR_tot(k) = Delta_i_tot(2);
            log_sat_alloc(k) = is_sat_alloc;
            log_sat_ext(k) = is_sat_ext;
            log_sat_any(k) = is_sat_any;
        end
        
        % 存储结构
        results_recip(exp_count).group_id = g_idx;
        results_recip(exp_count).group_name = groups(g_idx).name;
        results_recip(exp_count).Imax = Imax_current;
        results_recip(exp_count).ctrl_id = c_id;
        results_recip(exp_count).ctrl_name = ctrl_names{c_id};
        results_recip(exp_count).color = ctrl_colors{c_id};
        results_recip(exp_count).line_style = line_styles{c_id};
        results_recip(exp_count).line_width = line_widths(c_id);
        
        results_recip(exp_count).t = traj.t;
        results_recip(exp_count).yG = log_yG;
        results_recip(exp_count).alpha = log_alpha;
        results_recip(exp_count).esync = (log_yR - log_yL) * 1e3; % mm
        results_recip(exp_count).eyG = (log_yG - traj.y) * 1e3;    % mm
        results_recip(exp_count).iL_cmd = log_iL_cmd;
        results_recip(exp_count).iR_cmd = log_iR_cmd;
        results_recip(exp_count).iL_ideal = log_iL_ideal;
        results_recip(exp_count).iR_ideal = log_iR_ideal;
        results_recip(exp_count).sat_ratio_alloc = mean(log_sat_alloc) * 100.0;
        results_recip(exp_count).sat_ratio_ext = mean(log_sat_ext) * 100.0;
        results_recip(exp_count).sat_ratio_tot = mean(log_sat_any) * 100.0;
        
        % 标量与阶段品质指标
        eyG = results_recip(exp_count).eyG;
        esync = results_recip(exp_count).esync;
        
        results_recip(exp_count).rmse_yG = sqrt(mean(eyG.^2));
        results_recip(exp_count).iae_yG = trapz(traj.t, abs(eyG));
        results_recip(exp_count).max_sync = max(abs(esync));
        results_recip(exp_count).rmse_sync = sqrt(mean(esync.^2));
        results_recip(exp_count).iae_sync = trapz(traj.t, abs(esync));
        results_recip(exp_count).max_alpha = max(abs(log_alpha)) * 1e3; % mrad
        
        % 分阶段峰值同步误差与停靠超调
        esync_fwd = esync(1:t_half_idx);
        esync_rev = esync(t_half_idx:end);
        results_recip(exp_count).max_sync_fwd = max(abs(esync_fwd));
        results_recip(exp_count).max_sync_rev = max(abs(esync_rev));
        
        % 阶段 1 (正向目标 1.0m 停靠超调)
        yG_fwd = log_yG(1:t_half_idx);
        results_recip(exp_count).overshoot_fwd = max(0, max(yG_fwd) - y_target) * 1e3; % mm
        
        % 阶段 2 (反向目标 0.0m 停靠超调)
        yG_rev = log_yG(t_half_idx:end);
        results_recip(exp_count).overshoot_rev = max(0, -min(yG_rev)) * 1e3; % mm
        
        % 阶段 1 恢复/调节时间 (进入 1.0m +-1mm 容差带)
        tail_fwd = abs(yG_fwd - y_target) * 1e3;
        idx_out_fwd = find(tail_fwd > 1.0, 1, 'last');
        if isempty(idx_out_fwd) || idx_out_fwd >= length(yG_fwd)
            results_recip(exp_count).settle_fwd = NaN;
        else
            results_recip(exp_count).settle_fwd = traj.t(idx_out_fwd + 1);
        end
        
        % 阶段 2 恢复/调节时间 (进入 0.0m +-1mm 容差带)
        tail_rev = abs(yG_rev - 0.0) * 1e3;
        idx_out_rev = find(tail_rev > 1.0, 1, 'last');
        if isempty(idx_out_rev) || idx_out_rev >= length(yG_rev)
            results_recip(exp_count).settle_rev = NaN;
        else
            results_recip(exp_count).settle_rev = traj.t(t_half_idx + idx_out_rev);
        end
        
        % 控制量总变差 TV
        results_recip(exp_count).tv_iR = sum(abs(diff(log_iR_cmd)));
        
        % 饱和与分配缺额残差积分
        delta_alloc_norm_t = sqrt(log_delta_iL_alloc.^2 + log_delta_iR_alloc.^2);
        delta_ext_norm_t = sqrt(log_delta_iL_ext.^2 + log_delta_iR_ext.^2);
        delta_tot_norm_t = sqrt(log_delta_iL_tot.^2 + log_delta_iR_tot.^2);
        results_recip(exp_count).int_delta_alloc = trapz(traj.t, delta_alloc_norm_t);
        results_recip(exp_count).int_delta_ext = trapz(traj.t, delta_ext_norm_t);
        results_recip(exp_count).int_delta_tot = trapz(traj.t, delta_tot_norm_t);
        
        % 力矩保真度偏差
        if c_id >= 3
            delta_Talpha_t = abs(log_Talpha_alloc - log_Talpha_des);
            results_recip(exp_count).max_delta_Talpha = max(delta_Talpha_t);
            results_recip(exp_count).int_delta_Talpha = trapz(traj.t, delta_Talpha_t);
        else
            results_recip(exp_count).max_delta_Talpha = NaN;
            results_recip(exp_count).int_delta_Talpha = NaN;
        end
        
        res = results_recip(exp_count);
        
        % 格式化存入 CSV
        csv_records{end+1} = {groups(g_idx).name, Imax_current, res.ctrl_name, ...
            res.rmse_yG, res.iae_yG, res.max_sync, res.max_sync_fwd, res.max_sync_rev, res.iae_sync, ...
            res.overshoot_fwd, res.overshoot_rev, res.settle_fwd, res.settle_rev, ...
            res.sat_ratio_alloc, res.sat_ratio_ext, res.sat_ratio_tot, ...
            res.int_delta_alloc, res.int_delta_ext, res.int_delta_tot, ...
            res.max_delta_Talpha, res.tv_iR};
    end
end

%% 5. 数据保存: MAT 文件与 CSV 报告
save('reciprocating_results.mat', 'results_recip');

csv_fid = fopen('reciprocating_results_summary.csv', 'w');
fprintf(csv_fid, 'GroupName,Imax,Controller,RMSE_yG_mm,IAE_yG,Max_Sync_mm,Max_Sync_Fwd_mm,Max_Sync_Rev_mm,IAE_sync,Overshoot_Fwd_mm,Overshoot_Rev_mm,Settle_Fwd_s,Settle_Rev_s,Sat_Ratio_Alloc_pct,Sat_Ratio_Ext_pct,Sat_Ratio_Tot_pct,Int_Delta_Alloc,Int_Delta_Ext,Int_Delta_Tot,Max_Delta_Talpha_Nm,TV_iR\n');
for r_i = 1:length(csv_records)
    row = csv_records{r_i};
    fprintf(csv_fid, '%s,%.1f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.4f,%.1f\n', ...
        row{1}, row{2}, row{3}, row{4}, row{5}, row{6}, row{7}, row{8}, row{9}, row{10}, row{11}, row{12}, row{13}, row{14}, row{15}, row{16}, row{17}, row{18}, row{19}, row{20}, row{21});
end
fclose(csv_fid);
fprintf('\n[OK] 往复基准测试数据已保存至 reciprocating_results.mat 与 reciprocating_results_summary.csv\n');

%% 6. 控制台打印格式化汇总评价表
fprintf('\n======================================================================================================================================================================================================================\n');
fprintf('                                              往复换向基准测试 (0 -> 1.0m -> 0.0m, 7.0s) 客观综合评价表 (2x2 完全消融体系)\n');
fprintf('======================================================================================================================================================================================================================\n');
fprintf('%-10s | %-26s | %-8s | %-11s | %-10s | %-10s | %-8s | %-8s | %-11s | %-10s | %-10s | %-8s | %-8s\n', ...
    '工况', '控制器(Controller)', 'RMSE_yG', 'Max_Sync(mm)', '正向MaxSync', '反向MaxSync', '正向超调', '反向超调', '∫||Δi_alloc||', '∫||Δi_ext||', '∫||Δi_tot||', '力矩缺额', 'TV_iR');
fprintf('----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
for exp_i = 1:length(results_recip)
    r = results_recip(exp_i);
    if isnan(r.max_delta_Talpha)
        t_gap_str = '   N/A  ';
    else
        t_gap_str = sprintf('%8.4f', r.max_delta_Talpha);
    end
    fprintf('%-10s | %-26s | %8.3f | %12.4f | %11.4f | %11.4f | %7.2fmm | %7.2fmm | %13.1f | %10.1f | %10.1f | %8s | %8.1f\n', ...
        sprintf('Imax=%.0f', r.Imax), r.ctrl_name, r.rmse_yG, r.max_sync, r.max_sync_fwd, r.max_sync_rev, ...
        r.overshoot_fwd, r.overshoot_rev, r.int_delta_alloc, r.int_delta_ext, r.int_delta_tot, t_gap_str, r.tv_iR);
    if mod(exp_i, 6) == 0
        fprintf('----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
    end
end
fprintf('======================================================================================================================================================================================================================\n\n');

%% 7. 绘制两套独立的六控制器对比图 (Imax=16000 与 Imax=4500)
for g_idx = 1:2
    Imax_val = groups(g_idx).Imax;
    fig = figure('Color', 'w', 'Position', [60, 60, 1280, 800]);
    idx_offset = (g_idx - 1) * 6;
    
    % (a) 质心往复位移跟踪
    subplot(2, 2, 1); hold on; grid on; box on;
    plot(traj.t, traj.y, 'k:', 'LineWidth', 1.6, 'DisplayName', '期望往复轨迹 y_d');
    for c_id = 1:6
        res_curr = results_recip(idx_offset + c_id);
        plot(res_curr.t, res_curr.yG, 'Color', res_curr.color, 'LineStyle', res_curr.line_style, ...
            'LineWidth', res_curr.line_width, 'DisplayName', res_curr.ctrl_name);
    end
    xlabel('时间 t (s)'); ylabel('质心位移 y_G (m)');
    title(sprintf('(a) 往复位移跟踪响应 (I_{max}=%.0f)', Imax_val));
    legend('Location', 'Southeast', 'FontSize', 7.5);
    
    % (b) 左右同步误差对比
    subplot(2, 2, 2); hold on; grid on; box on;
    for c_id = 1:6
        res_curr = results_recip(idx_offset + c_id);
        plot(res_curr.t, res_curr.esync, 'Color', res_curr.color, 'LineStyle', res_curr.line_style, ...
            'LineWidth', res_curr.line_width, 'DisplayName', res_curr.ctrl_name);
    end
    xlabel('时间 t (s)'); ylabel('同步误差 y_R - y_L (mm)');
    title(sprintf('(b) 往复全周期同步误差对比 (I_{max}=%.0f)', Imax_val));
    legend('Location', 'Northeast', 'FontSize', 7.5);
    
    % (c) 横梁偏转偏角 alpha
    subplot(2, 2, 3); hold on; grid on; box on;
    for c_id = 1:6
        res_curr = results_recip(idx_offset + c_id);
        plot(res_curr.t, res_curr.alpha * 1e3, 'Color', res_curr.color, 'LineStyle', res_curr.line_style, ...
            'LineWidth', res_curr.line_width, 'DisplayName', res_curr.ctrl_name);
    end
    xlabel('时间 t (s)'); ylabel('横梁偏角 \alpha (mrad)');
    title(sprintf('(c) 横梁偏角响应 (I_{max}=%.0f)', Imax_val));
    legend('Location', 'Northeast', 'FontSize', 7.5);
    
    % (d) 右电机电流指令响应
    subplot(2, 2, 4); hold on; grid on; box on;
    yline( Imax_val, 'r:', 'LineWidth', 1.3, 'DisplayName', sprintf('+I_{max} (%.0f)', Imax_val));
    yline(-Imax_val, 'r:', 'LineWidth', 1.3, 'DisplayName', sprintf('-I_{max} (-%.0f)', Imax_val));
    for c_id = 1:6
        res_curr = results_recip(idx_offset + c_id);
        plot(res_curr.t, res_curr.iR_cmd, 'Color', res_curr.color, 'LineStyle', res_curr.line_style, ...
             'LineWidth', res_curr.line_width, 'DisplayName', sprintf('%s 指令', res_curr.ctrl_name));
    end
    xlabel('时间 t (s)'); ylabel('右电机电流指令 (counts)');
    title(sprintf('(d) 右电机电流指令全周期响应 (I_{max}=%.0f)', Imax_val));
    legend('Location', 'Southeast', 'FontSize', 7);
    
    img_name = sprintf('benchmark_reciprocating_Imax%.0f.png', Imax_val);
    saveas(fig, img_name);
    fprintf('[OK] 往复对比图表已保存为 %s\n', img_name);
end

%% 辅助函数
function s = init_pid_struct(kp, ki, kd, max_out, max_iout)
    s.Kp = kp;
    s.Ki = ki;
    s.Kd = kd;
    s.max_out = max_out;
    s.max_iout = max_iout;
    s.error = [0.0, 0.0, 0.0];
    s.Dbuf = [0.0, 0.0, 0.0];
    s.Iout = 0.0;
    s.out = 0.0;
    s.set = 0.0;
    s.fdb = 0.0;
end
