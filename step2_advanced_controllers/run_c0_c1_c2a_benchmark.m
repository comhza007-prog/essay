%% RUN_C0_C1_C2A_BENCHMARK.M - C0 vs C1 vs C2a-noFF vs C2a vs C2b 统一限流基准对比 (V4.0 规范版)
% =========================================================================
% 本脚本完成两组独立限流条件下的严格横向对比：
% 1. 组别 1 (无饱和充裕工况): Imax = 16000 (C0, C1, C2a-noFF, C2a, C2b)
% 2. 组别 2 (强限流饱和工况): Imax = 4500  (C0, C1, C2a-noFF, C2a, C2b)
%
% 五大控制器对照体系：
% - C0       : 独立级联 PID (比赛工程基线, 固件限幅 16000, 积分限幅 2000)
% - C1       : 传统交叉耦合控制 CCC (物理前向速度坐标纠偏 + 动态速度限幅钳位)
% - C2a-noFF : 广义误差面鲁棒控制消融组 (关闭名义前馈 va=0, 验证前馈解耦价值)
% - C2a      : 广义误差面鲁棒控制 (含名义动力学前馈 + tanh 连续鲁棒反馈, 无抗饱和)
% - C2b      : 动态抗饱和鲁棒控制 (含跨周期保存的离散动态状态记忆 z_aw, 提出方法)
%
% 统一多级执行器限幅链条：
%   i_request -> sat(16000) [i_fw] -> sat(Imax) [i_actual]
% 双轨饱和统计指标：
%   1. 外部硬件限幅 (External Hard Saturation): Delta_i_external = i_actual - i_fw
%   2. 总不可实现请求 (Total Saturation):       Delta_i_total    = i_actual - i_request
% =========================================================================

if ~exist('is_test_runner', 'var')
    clear; clc; close all;
end

%% 1. 参数加载与测试轨迹生成
addpath(fullfile(pwd, '..', 'step1_baseline_c0'));
[ctrl, mech, plant] = param_init();

Ts = ctrl.Ts;                     % 0.001 s
t_span = 3.5;                     % 3.5 s

y_target = 1.0;                   % 1.0 m
v_max = 0.6;                      % 0.6 m/s
a_max = 1.5;                      % 1.5 m/s^2
traj = trajectory_gen(t_span, Ts, y_target, v_max, a_max);
N_steps = length(traj.t);

m_to_ecd = (mech.N * ctrl.ecd_cpr) / (2.0 * pi * mech.rp);
ms_to_rpm = (60.0 * mech.N) / (2.0 * pi * mech.rp);

%% 2. 统一物理模型非对称环境
env.delta_m = 3.0;                % 右侧偏心质量 3.0 kg
env.d_load = 0.28;                % 偏心距 0.28 m
env.delta_fric = 0.30;            % 右侧摩擦增加 30%
env.Kf_L = mech.Kf;
env.Kf_R = mech.Kf;

%% 3. 控制器特定参数配置
% C1: 传统交叉耦合参数
ctrl_c1 = ctrl;
ctrl_c1.Kp_sync = 3.5;            % 1/s
ctrl_c1.Kd_sync = 0.08;           % 无量纲

% C2a 基础参数
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

% C2a-noFF: 禁用名义动力学前馈消融组
ctrl_c2a_noff = ctrl_c2_base;
ctrl_c2a_noff.enable_ff = false;

% C2a: 启用名义动力学前馈
ctrl_c2a = ctrl_c2_base;
ctrl_c2a.enable_ff = true;

% C2b: 动态抗饱和鲁棒参数
ctrl_c2b = ctrl_c2a;
ctrl_c2b.K_aw = 20.0 * eye(2);     % 2x2 增益矩阵
ctrl_c2b.lambda_aw = 20.0;        % 衰减耗散因子 (1/s)
ctrl_c2b.I_fw_limit = 16000.0;    % 软件固件内部限幅 (counts)
ctrl_c2b.aw_mode = 'external';    % 外部硬件饱和残差驱动抗饱和动态

%% 4. 定义对比实验矩阵 (2组限流 x 5种控制器 = 10项实验)
groups(1).name = 'Group 1: 充裕电流限幅 (Imax = 16000)';
groups(1).Imax = 16000.0;

groups(2).name = 'Group 2: 严格电流限幅 (Imax = 4500)';
groups(2).Imax = 4500.0;

ctrl_names = {'C0 (独立级联 PID)', ...
              'C1 (传统交叉耦合 CCC)', ...
              'C2a-noFF (误差面鲁棒 - 无前馈消融)', ...
              'C2a (误差面鲁棒 - 名义前馈)', ...
              'C2b (动态抗饱和鲁棒 - 提出)'};

ctrl_colors = {[0.0, 0.45, 0.74], ...      % 经典蓝
               [0.85, 0.33, 0.10], ...      % 活力橙
               [0.64, 0.08, 0.18], ...      % 深红 (消融)
               [0.47, 0.67, 0.19], ...      % 草绿 (基线)
               [0.49, 0.18, 0.56]};        % 紫罗兰 (提出)

results_bench = struct([]);
csv_records = {};

exp_count = 0;

for g_idx = 1:length(groups)
    Imax_current = groups(g_idx).Imax;
    fprintf('\n>>> 正在运行 %s <<<\n', groups(g_idx).name);
    
    for c_id = 1:5
        exp_count = exp_count + 1;
        fprintf('  - 正在推演: %s ...\n', ctrl_names{c_id});
        
        x = zeros(4, 1);
        
        % 初始化 C0 / C1 PID 状态 (内部输出限幅固定为比赛控制器固有上限 16000 counts)
        s_ang_L = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, ctrl.ang_base_max_out, ctrl.ang_max_iout);
        s_ang_R = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, ctrl.ang_base_max_out, ctrl.ang_max_iout);
        s_spd_L = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, ctrl.spd_max_out, ctrl.spd_max_iout);
        s_spd_R = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, ctrl.spd_max_out, ctrl.spd_max_iout);
        states_c1.pid_ang_L = s_ang_L; states_c1.pid_ang_R = s_ang_R;
        states_c1.pid_spd_L = s_spd_L; states_c1.pid_spd_R = s_spd_R;
        
        % 初始化 C2b 动态抗饱和离散状态 (跨周期严格保存)
        state_c2b.z_aw = [0.0; 0.0];
        
        % 数据记录缓存
        log_yG = zeros(1, N_steps);
        log_alpha = zeros(1, N_steps);
        log_yL = zeros(1, N_steps);
        log_yR = zeros(1, N_steps);
        log_iL_cmd = zeros(1, N_steps);
        log_iR_cmd = zeros(1, N_steps);
        log_iL_ideal = zeros(1, N_steps);
        log_iR_ideal = zeros(1, N_steps);
        
        log_delta_iL_ext = zeros(1, N_steps);
        log_delta_iR_ext = zeros(1, N_steps);
        log_delta_iL_tot = zeros(1, N_steps);
        log_delta_iR_tot = zeros(1, N_steps);
        log_sat_ext = zeros(1, N_steps);
        log_sat_any = zeros(1, N_steps);
        
        % C2 族特有变量记录
        log_v_beta = zeros(2, N_steps);
        log_v_actual = zeros(2, N_steps);
        log_sigma = zeros(2, N_steps);
        log_va = zeros(2, N_steps);
        log_vr = zeros(2, N_steps);
        log_z_aw = zeros(2, N_steps);
        
        for k = 1:N_steps
            yG_c = x(1); alpha_c = x(2);
            yG_dot_c = x(3); alpha_dot_c = x(4);
            
            yL_c = yG_c - 0.5 * mech.Le * alpha_c;
            yR_c = yG_c + 0.5 * mech.Le * alpha_c;
            vL_c = yG_dot_c - 0.5 * mech.Le * alpha_dot_c;
            vR_c = yG_dot_c + 0.5 * mech.Le * alpha_dot_c;
            
            yd_t = traj.y(k);
            
            % 动态限幅 (chassis_calculate)
            ecd_L =  yL_c * m_to_ecd;
            ecd_tar_L = yd_t * m_to_ecd;
            err_rev = abs(ecd_tar_L - ecd_L) / ctrl.ecd_cpr;
            trav_rev = abs(ecd_L) / ctrl.ecd_cpr;
            dyn_max = min(err_rev, trav_rev) * ctrl.ang_gradient_slope;
            dyn_max = max(ctrl.ang_gradient_min_out, min(ctrl.ang_gradient_max_out, dyn_max));
            
            % 控制器分支解算
            if c_id == 1
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
                Delta_i_tot = [iL_cmd; iR_cmd] - i_request;
                is_sat_any = any(abs(Delta_i_tot) > 1e-6);
                
            elseif c_id == 2
                % --- C1: 传统交叉耦合 CCC (含物理速度限幅) ---
                [iL_cmd, iR_cmd, states_c1, info_c1] = controller_c1_ccc( ...
                    yL_c, yR_c, vL_c, vR_c, yd_t, dyn_max, ctrl_c1, mech, Imax_current, states_c1);
                iL_ideal = info_c1.i_request(1);
                iR_ideal = info_c1.i_request(2);
                Delta_i_ext = info_c1.Delta_i_external;
                Delta_i_tot = info_c1.Delta_i_total;
                is_sat_ext = info_c1.is_sat_external;
                is_sat_any = info_c1.is_sat_any;
                
            elseif c_id == 3
                % --- C2a-noFF: 广义误差面鲁棒控制消融组 (va = 0) ---
                q_curr = [yG_c; alpha_c];
                qdot_curr = [yG_dot_c; alpha_dot_c];
                qd_curr = [yd_t; 0.0];
                qdot_d_curr = [traj.ydot(k); 0.0];
                qddot_d_curr = [traj.yddot(k); 0.0];
                
                [iL_cmd, iR_cmd, info_c2a_noff] = controller_c2a_robust( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2a_noff, mech.Le, env.Kf_L, env.Kf_R, Imax_current);
                
                iL_ideal = info_c2a_noff.i_request(1);
                iR_ideal = info_c2a_noff.i_request(2);
                Delta_i_ext = info_c2a_noff.Delta_i_external;
                Delta_i_tot = info_c2a_noff.Delta_i_total;
                is_sat_ext = info_c2a_noff.is_sat_external;
                is_sat_any = info_c2a_noff.is_sat_any;
                
                log_v_beta(:, k) = info_c2a_noff.v_beta;
                log_v_actual(:, k) = info_c2a_noff.v_actual;
                log_sigma(:, k) = info_c2a_noff.sigma;
                log_va(:, k) = info_c2a_noff.va;
                log_vr(:, k) = info_c2a_noff.vr;
                
            elseif c_id == 4
                % --- C2a: 广义误差面鲁棒控制 (含名义前馈, 无抗饱和) ---
                q_curr = [yG_c; alpha_c];
                qdot_curr = [yG_dot_c; alpha_dot_c];
                qd_curr = [yd_t; 0.0];
                qdot_d_curr = [traj.ydot(k); 0.0];
                qddot_d_curr = [traj.yddot(k); 0.0];
                
                [iL_cmd, iR_cmd, info_c2a] = controller_c2a_robust( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2a, mech.Le, env.Kf_L, env.Kf_R, Imax_current);
                
                iL_ideal = info_c2a.i_request(1);
                iR_ideal = info_c2a.i_request(2);
                Delta_i_ext = info_c2a.Delta_i_external;
                Delta_i_tot = info_c2a.Delta_i_total;
                is_sat_ext = info_c2a.is_sat_external;
                is_sat_any = info_c2a.is_sat_any;
                
                log_v_beta(:, k) = info_c2a.v_beta;
                log_v_actual(:, k) = info_c2a.v_actual;
                log_sigma(:, k) = info_c2a.sigma;
                log_va(:, k) = info_c2a.va;
                log_vr(:, k) = info_c2a.vr;
                
            elseif c_id == 5
                % --- C2b: 动态抗饱和鲁棒控制 (含离散记忆状态 z_aw) ---
                q_curr = [yG_c; alpha_c];
                qdot_curr = [yG_dot_c; alpha_dot_c];
                qd_curr = [yd_t; 0.0];
                qdot_d_curr = [traj.ydot(k); 0.0];
                qddot_d_curr = [traj.yddot(k); 0.0];
                
                [iL_cmd, iR_cmd, state_c2b, info_c2b] = controller_c2b_aw( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2b, mech.Le, env.Kf_L, env.Kf_R, Imax_current, state_c2b, Ts);
                
                iL_ideal = info_c2b.i_request(1);
                iR_ideal = info_c2b.i_request(2);
                Delta_i_ext = info_c2b.Delta_i_external;
                Delta_i_tot = info_c2b.Delta_i_total;
                is_sat_ext = info_c2b.is_sat_external;
                is_sat_any = info_c2b.is_sat_any;
                
                log_v_beta(:, k) = info_c2b.v_beta;
                log_v_actual(:, k) = info_c2b.v_actual;
                log_sigma(:, k) = info_c2b.sigma;
                log_va(:, k) = info_c2b.va;
                log_vr(:, k) = info_c2b.vr;
                log_z_aw(:, k) = info_c2b.z_aw;
            end
            
            % 动力学单步推演
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
            
            log_delta_iL_ext(k) = Delta_i_ext(1);
            log_delta_iR_ext(k) = Delta_i_ext(2);
            log_delta_iL_tot(k) = Delta_i_tot(1);
            log_delta_iR_tot(k) = Delta_i_tot(2);
            log_sat_ext(k) = is_sat_ext;
            log_sat_any(k) = is_sat_any;
        end
        
        % 存储当前实验记录
        results_bench(exp_count).group_id = g_idx;
        results_bench(exp_count).group_name = groups(g_idx).name;
        results_bench(exp_count).Imax = Imax_current;
        results_bench(exp_count).ctrl_id = c_id;
        results_bench(exp_count).ctrl_name = ctrl_names{c_id};
        results_bench(exp_count).color = ctrl_colors{c_id};
        
        results_bench(exp_count).t = traj.t;
        results_bench(exp_count).yG = log_yG;
        results_bench(exp_count).alpha = log_alpha;
        results_bench(exp_count).esync = (log_yR - log_yL) * 1e3; % mm
        results_bench(exp_count).eyG = (log_yG - traj.y) * 1e3;    % mm
        results_bench(exp_count).iL_cmd = log_iL_cmd;
        results_bench(exp_count).iR_cmd = log_iR_cmd;
        results_bench(exp_count).iL_ideal = log_iL_ideal;
        results_bench(exp_count).iR_ideal = log_iR_ideal;
        results_bench(exp_count).delta_iL_ext = log_delta_iL_ext;
        results_bench(exp_count).delta_iR_ext = log_delta_iR_ext;
        results_bench(exp_count).delta_iL_tot = log_delta_iL_tot;
        results_bench(exp_count).delta_iR_tot = log_delta_iR_tot;
        results_bench(exp_count).sat_ratio_ext = mean(log_sat_ext) * 100.0;
        results_bench(exp_count).sat_ratio_tot = mean(log_sat_any) * 100.0;
        results_bench(exp_count).sat_ratio = results_bench(exp_count).sat_ratio_ext; % 保持兼容
        
        % 专有变量记录
        if c_id >= 3
            results_bench(exp_count).v_beta = log_v_beta;
            results_bench(exp_count).v_actual = log_v_actual;
            results_bench(exp_count).sigma = log_sigma;
            results_bench(exp_count).va = log_va;
            results_bench(exp_count).vr = log_vr;
        else
            results_bench(exp_count).v_beta = [];
            results_bench(exp_count).v_actual = [];
            results_bench(exp_count).sigma = [];
            results_bench(exp_count).va = [];
            results_bench(exp_count).vr = [];
        end
        if c_id == 5
            results_bench(exp_count).z_aw = log_z_aw;
        else
            results_bench(exp_count).z_aw = [];
        end
        
        % 统计标量指标
        eyG = results_bench(exp_count).eyG;
        esync = results_bench(exp_count).esync;
        
        results_bench(exp_count).rmse_yG = sqrt(mean(eyG.^2));
        results_bench(exp_count).iae_yG = trapz(traj.t, abs(eyG));
        results_bench(exp_count).max_sync = max(abs(esync));
        results_bench(exp_count).rmse_sync = sqrt(mean(esync.^2));
        results_bench(exp_count).iae_sync = trapz(traj.t, abs(esync));
        results_bench(exp_count).max_alpha = max(abs(log_alpha)) * 1e3; % mrad
        
        % 控制量总变差 (Total Variation)
        results_bench(exp_count).tv_iL = sum(abs(diff(log_iL_cmd)));
        results_bench(exp_count).tv_iR = sum(abs(diff(log_iR_cmd)));
        results_bench(exp_count).tv_i_total = results_bench(exp_count).tv_iL + results_bench(exp_count).tv_iR;
        
        % 双轨饱和残差指标 (外部硬件饱和 vs 总不可实现请求)
        delta_ext_norm_t = sqrt(log_delta_iL_ext.^2 + log_delta_iR_ext.^2);
        results_bench(exp_count).max_abs_delta_ext = max([abs(log_delta_iL_ext), abs(log_delta_iR_ext)], [], 'all');
        results_bench(exp_count).max_delta_ext_norm = max(delta_ext_norm_t);
        results_bench(exp_count).int_delta_ext = trapz(traj.t, delta_ext_norm_t);
        
        delta_tot_norm_t = sqrt(log_delta_iL_tot.^2 + log_delta_iR_tot.^2);
        results_bench(exp_count).max_abs_delta_tot = max([abs(log_delta_iL_tot), abs(log_delta_iR_tot)], [], 'all');
        results_bench(exp_count).max_delta_tot_norm = max(delta_tot_norm_t);
        results_bench(exp_count).int_delta_tot = trapz(traj.t, delta_tot_norm_t);
        
        % 兼容性旧字段
        results_bench(exp_count).max_abs_delta_i = results_bench(exp_count).max_abs_delta_ext;
        results_bench(exp_count).max_delta_i_norm = results_bench(exp_count).max_delta_ext_norm;
        results_bench(exp_count).int_delta_i = results_bench(exp_count).int_delta_ext;
        
        % 广义误差面指标
        if c_id >= 3
            sigma_norm_t = sqrt(log_sigma(1,:).^2 + log_sigma(2,:).^2);
            results_bench(exp_count).max_sigma = max(sigma_norm_t);
            results_bench(exp_count).int_sigma = trapz(traj.t, sigma_norm_t);
        else
            results_bench(exp_count).max_sigma = NaN;
            results_bench(exp_count).int_sigma = NaN;
        end
        
        % 调节时间判定 (进入 ±1.0mm 容差带)
        settle_tol = 1.0;
        err_tail = abs(results_bench(exp_count).yG - y_target) * 1e3;
        if err_tail(end) > settle_tol
            results_bench(exp_count).is_settled = false;
            results_bench(exp_count).settle_time = NaN;
        else
            idx_out = find(err_tail > settle_tol, 1, 'last');
            results_bench(exp_count).settle_time = traj.t(min(idx_out + 1, N_steps));
            results_bench(exp_count).is_settled = true;
        end
        
        res = results_bench(exp_count);
        
        if res.is_settled
            ts_val = res.settle_time;
        else
            ts_val = -1.0;
        end
        csv_records{end+1} = {groups(g_idx).name, Imax_current, res.ctrl_name, ...
            res.rmse_yG, res.iae_yG, res.max_sync, res.rmse_sync, res.iae_sync, res.max_alpha, ...
            res.sat_ratio_ext, res.sat_ratio_tot, res.max_delta_ext_norm, res.int_delta_ext, ...
            res.max_delta_tot_norm, res.int_delta_tot, res.tv_iR, res.max_sigma, ts_val};
    end
end

%% 5. 数据持久化保存: MAT 文件与 CSV 报告
save('results_bench.mat', 'results_bench');

% 写入 CSV 总结表
csv_fid = fopen('results_summary.csv', 'w');
fprintf(csv_fid, 'GroupName,Imax,Controller,RMSE_yG_mm,IAE_yG,Max_Sync_mm,RMSE_Sync_mm,IAE_sync,Max_Alpha_mrad,Sat_Ratio_Ext_pct,Sat_Ratio_Tot_pct,Max_Delta_Ext_Norm,Int_Delta_Ext,Max_Delta_Tot_Norm,Int_Delta_Tot,TV_iR,Max_Sigma,Settle_Ts_s\n');
for r_i = 1:length(csv_records)
    row = csv_records{r_i};
    fprintf(csv_fid, '%s,%.1f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f\n', ...
        row{1}, row{2}, row{3}, row{4}, row{5}, row{6}, row{7}, row{8}, row{9}, row{10}, row{11}, row{12}, row{13}, row{14}, row{15}, row{16}, row{17}, row{18});
end
fclose(csv_fid);
fprintf('\n[OK] 数据已保存至 results_bench.mat 与 results_summary.csv\n');

%% 6. 控制台打印格式化汇总指标表
fprintf('\n========================================================================================================================================================================\n');
fprintf('                           C0 vs C1 vs C2a-noFF vs C2a vs C2b 统一限流基准对比测试客观评价表 (严苛偏心 3.0kg + 摩擦差 30%%)\n');
fprintf('========================================================================================================================================================================\n');
fprintf('%-11s | %-24s | %-8s | %-11s | %-8s | %-8s | %-8s | %-10s | %-10s | %-8s | %-8s\n', ...
    '工况', '控制器(Controller)', 'RMSE_yG', 'Max_Sync(mm)', 'IAE_sync', 'Sat_Ext', 'Sat_Tot', '∫||Δi_ext||', '∫||Δi_tot||', 'TV_iR', 'Settle_Ts');
fprintf('------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
for exp_i = 1:length(results_bench)
    r = results_bench(exp_i);
    if r.is_settled
        ts_str = sprintf('%6.3f s', r.settle_time);
    else
        ts_str = 'Unsettled';
    end
    fprintf('%-11s | %-24s | %8.3f | %12.4f | %8.4f | %7.1f%% | %7.1f%% | %10.1f | %10.1f | %8.1f | %8s\n', ...
        sprintf('Imax=%.0f', r.Imax), r.ctrl_name, r.rmse_yG, r.max_sync, r.iae_sync, ...
        r.sat_ratio_ext, r.sat_ratio_tot, r.int_delta_ext, r.int_delta_tot, r.tv_iR, ts_str);
    if mod(exp_i, 5) == 0
        fprintf('------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
    end
end
fprintf('========================================================================================================================================================================\n\n');

%% 7. 绘制两套独立的五控制器对比图 (工况1: Imax=16000 与 工况2: Imax=4500)
line_styles = {'-', '-', '--', '-', '-'};
line_widths = [1.3, 1.3, 1.2, 1.4, 1.5];

for g_idx = 1:2
    Imax_val = groups(g_idx).Imax;
    fig = figure('Color', 'w', 'Position', [80, 80, 1200, 780]);
    idx_offset = (g_idx - 1) * 5;
    
    % (a) 质心平动位移
    subplot(2, 2, 1); hold on; grid on; box on;
    plot(traj.t, traj.y, 'k:', 'LineWidth', 1.6, 'DisplayName', '期望轨迹 y_d');
    for c_id = 1:5
        res_curr = results_bench(idx_offset + c_id);
        plot(res_curr.t, res_curr.yG, 'Color', res_curr.color, 'LineStyle', line_styles{c_id}, ...
            'LineWidth', line_widths(c_id), 'DisplayName', res_curr.ctrl_name);
    end
    xlabel('时间 t (s)'); ylabel('质心位移 y_G (m)');
    title(sprintf('(a) 质心平动轨迹跟踪 (I_{max}=%.0f)', Imax_val));
    legend('Location', 'Southeast', 'FontSize', 8);
    
    % (b) 左右同步误差
    subplot(2, 2, 2); hold on; grid on; box on;
    for c_id = 1:5
        res_curr = results_bench(idx_offset + c_id);
        plot(res_curr.t, res_curr.esync, 'Color', res_curr.color, 'LineStyle', line_styles{c_id}, ...
            'LineWidth', line_widths(c_id), 'DisplayName', res_curr.ctrl_name);
    end
    xlabel('时间 t (s)'); ylabel('同步误差 y_R - y_L (mm)');
    title(sprintf('(b) 左右同步误差对比 (I_{max}=%.0f)', Imax_val));
    legend('Location', 'Northeast', 'FontSize', 8);
    
    % (c) 横梁偏转偏角 alpha
    subplot(2, 2, 3); hold on; grid on; box on;
    for c_id = 1:5
        res_curr = results_bench(idx_offset + c_id);
        plot(res_curr.t, res_curr.alpha * 1e3, 'Color', res_curr.color, 'LineStyle', line_styles{c_id}, ...
            'LineWidth', line_widths(c_id), 'DisplayName', res_curr.ctrl_name);
    end
    xlabel('时间 t (s)'); ylabel('横梁偏角 \alpha (mrad)');
    title(sprintf('(c) 横梁偏角响应 (I_{max}=%.0f)', Imax_val));
    legend('Location', 'Northeast', 'FontSize', 8);
    
    % (d) 右电机指令响应与限幅削顶
    subplot(2, 2, 4); hold on; grid on; box on;
    yline(-Imax_val, 'r:', 'LineWidth', 1.4, 'DisplayName', sprintf('硬件上限 -%.0f', Imax_val));
    for c_id = 1:5
        res_curr = results_bench(idx_offset + c_id);
        plot(res_curr.t, res_curr.iR_cmd, 'Color', res_curr.color, 'LineStyle', line_styles{c_id}, ...
             'LineWidth', line_widths(c_id), 'DisplayName', sprintf('%s 指令', res_curr.ctrl_name));
        % 针对强限流工况，叠加画出 C2a 与 C2b 的未限幅理想请求虚线
        if g_idx == 2 && (c_id == 4 || c_id == 5)
            plot(res_curr.t, res_curr.iR_ideal, 'Color', res_curr.color, 'LineStyle', ':', 'LineWidth', 1.1, ...
                 'DisplayName', sprintf('%s 理想请求', res_curr.ctrl_name));
        end
    end
    xlabel('时间 t (s)'); ylabel('右电机电流指令 (counts)');
    title(sprintf('(d) 右电机指令响应与削顶缺额 (I_{max}=%.0f)', Imax_val));
    legend('Location', 'Southeast', 'FontSize', 7);
    
    img_name = sprintf('benchmark_c0_c1_c2a_Imax%.0f.png', Imax_val);
    saveas(fig, img_name);
    fprintf('[OK] 图表已保存为 %s\n', img_name);
end

%% 辅助函数: 初始化 PID 结构体
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
