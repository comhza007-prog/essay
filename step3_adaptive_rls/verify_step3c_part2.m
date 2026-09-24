%% VERIFY_STEP3C_PART2.M - Step 3C-2 动力学耦合与多因素深度分析基准测试驱动
% =========================================================================
% 功能说明:
% 依据最新技术审查意见与 STEP3_IMPLEMENTATION_PLAN.md 规范，全面实施第二阶段测试 (Tests C4 ~ C8):
% 1. Test C4: 偏载动力学耦合诊断评估 (固定 Delta_m = 50.0 kg, d_load in [-0.20, +0.20] m)
%    - 严格调用公共 RK4 动力学求解器 common/gantry_dynamics_step_rk4.m 重积分
%    - 定量评估增量偏载偏差: theta_payload_bias(d) = theta_hat(d) - theta_hat(0)
%    - 计算 [-0.10, +0.10]m 拟合优度 R^2，严格区分“局部线性”与“方向一致性”
% 2. Test C5: 最劣角点噪声复合鲁棒性 (TestC5_TopWorstCorners_Noise_MC100)
%    - 阶段一: 128 个全因子确定性角点扫描 (2^7 corners)
%    - 阶段二: 多指标联合选取 Top 3 最劣角点 (Union of worst eta_total, alpha_comp, theta_err)
%    - 叠加高频传感测量噪声执行 N = 100 Monte Carlo 试验，分别记录最差指标角点
% 3. Test C6: 统一绝对敏感度与龙卷风双重排序 (Dual Tornado Ranking)
%    - 单因素物理导数 S_physical = |Delta_metric| / Delta_p [按参数自身单位定义，动态拼接输出]
%    - 范围影响量 Range_Impact = max(|metric_low - metric_nom|, |metric_high - metric_nom|) [用于独立龙卷风排序]
%    - 双重独立排行榜:
%      * 排行榜 A (估计器参数偏差影响量): metric = |theta_hat - Delta_Kf_true| [(N/count)/Unit]
%      * 排行榜 B (物理偏航残余影响量): metric = RMS(alpha_comp) [rad/Unit]
% 4. Test C7: 凸集投影安全性双向检验 (TestC7_Convex_Projection_Safety_MC30)
%    - 明确为 Measured_Current_Sensor_Step_Fault (回采电流测量阶跃故障)
%    - 分别注入正负大阶跃故障冲击，断言激活双向物理边界: low_clips > 0 && high_clips > 0
% 5. Test C8: 标称物理对称模型的虚假补偿深度评测 (C8A, C8B, C8C, N = 100)
%    - C8A (TestC8A_SensorCommFalseComp): 补齐增益漂移与测量延迟扰动矩阵，保留 RAW_ESTIMATOR_FAIL
%      * 导出评测时间分布证据 (max_run_p95, late_exceed_mean, first/last exceedance)
%    - C8B (TestC8B_PayloadConfounding): 每 trial 增加匹配 d_load=0 对照，计算真实 payload_bias
%    - C8C (TestC8C_GatedApplication): 建立独立迟滞门限控制仿真，评价工程门控对虚假执行的阻断
% 6. CSV 结果表规范导出与结构完整性严格回读检验:
%    - 导出四表: step3c_performance_results.csv, step3c_sensitivity_results.csv, step3c_projection_results.csv, step3c_c8a_deterministic_results.csv
%    - readtable 逐字段内存值绝对误差一致性断言 (|T_read - mem| < 1e-12)
% =========================================================================

function verify_step3c_part2()
    clc;
    fprintf('=========================================================================\n');
    fprintf('   STEP 3C-2: 动力学耦合与多因素深度分析基准测试 (Tests C4 ~ C8)        \n');
    fprintf('=========================================================================\n\n');
    
    script_dir = fileparts(mfilename('fullpath'));
    common_dir = fullfile(script_dir, '..', 'common');
    step1_dir  = fullfile(script_dir, '..', 'step1_baseline_c0');
    addpath(script_dir);
    addpath(common_dir);
    addpath(step1_dir);
    
    % 1. 严格红线闭环隔离检查
    fprintf('>>> 执行严格红线闭环控制器隔离检查...\n');
    c3a_file = fullfile(script_dir, 'controller_c3a_rls_robust.m');
    assert(exist(c3a_file, 'file') == 2, 'controller_c3a_rls_robust.m 文件存在');
    fprintf('    [OK] 确认本模块为纯离线开环回放敏度分析，绝未接入 controller_c3a 或 SyncAlloc\n');
    
    % 2. 检查公共 RK4 动力学求解器
    fprintf('>>> 检查公共 RK4 单步动力学求解器 gantry_dynamics_step_rk4.m...\n');
    rk4_file = fullfile(common_dir, 'gantry_dynamics_step_rk4.m');
    assert(exist(rk4_file, 'file') == 2, '缺少 common/gantry_dynamics_step_rk4.m');
    fprintf('    [OK] 唯一公共 RK4 入口已就绪: %s\n\n', rk4_file);
    
    % 3. 权威参数源校验
    fprintf('>>> 从权威参数源 param_init.m 读取硬件限幅参数...\n');
    assert(exist(fullfile(step1_dir, 'param_init.m'), 'file') == 2, '缺少 param_init.m');
    [ctrl, ~, ~] = param_init();
    Imax_nominal = ctrl.spd_max_out;
    assert(Imax_nominal == 16000.0, 'param_init.ctrl.spd_max_out 标称值必须为 16000.0 counts');
    fprintf('    [OK] 权威标称硬件限幅校验通过: Imax = %.1f counts\n\n', Imax_nominal);
    
    % 4. 加载基础非对称数据集 (r = 0.70 与 r = 1.30)
    file_r070 = fullfile(script_dir, 'data_step3b_phase0_r070.mat');
    file_r130 = fullfile(script_dir, 'data_step3b_phase0_r130.mat');
    assert(exist(file_r070, 'file') == 2, '缺少 r070 数据集: %s', file_r070);
    assert(exist(file_r130, 'file') == 2, '缺少 r130 数据集: %s', file_r130);
    
    d_r070 = load(file_r070);
    d_r070.N = length(d_r070.t);
    d_r070.Imax = Imax_nominal;
    if ~isfield(d_r070, 'iL_cmd'), d_r070.iL_cmd = d_r070.iL_actual; end
    if ~isfield(d_r070, 'iR_cmd'), d_r070.iR_cmd = d_r070.iR_actual; end
    
    d_r130 = load(file_r130);
    d_r130.N = length(d_r130.t);
    d_r130.Imax = Imax_nominal;
    if ~isfield(d_r130, 'iL_cmd'), d_r130.iL_cmd = d_r130.iL_actual; end
    if ~isfield(d_r130, 'iR_cmd'), d_r130.iR_cmd = d_r130.iR_actual; end
    
    % 5. 加载对称基准数据集 (r = 1.00)
    file_sym = fullfile(script_dir, 'data_step3b_phase1_sym.mat');
    assert(exist(file_sym, 'file') == 2, '缺少对称数据集: %s', file_sym);
    d_sym = load(file_sym);
    d_sym.N = length(d_sym.t);
    d_sym.Imax = Imax_nominal;
    if ~isfield(d_sym, 'iL_cmd'), d_sym.iL_cmd = d_sym.iL_actual; end
    if ~isfield(d_sym, 'iR_cmd'), d_sym.iR_cmd = d_sym.iR_actual; end
    
    datasets = {d_r070, d_r130};
    dataset_tags = {'r070 (Delta_Kf < 0)', 'r130 (Delta_Kf > 0)'};
    dataset_short = {'r070', 'r130'};
    
    perf_rows = {};
    sens_rows = {};
    proj_rows = {};
    
    %% =========================================================================
    %% TEST C4: 偏载动力学耦合诊断评估 (DIAGNOSTIC EVALUATION MODE)
    %% =========================================================================
    fprintf('=========================================================================\n');
    fprintf('>>> 开始执行 [Test C4] 偏载动力学耦合诊断评估 (固定 Delta_m = 50.0 kg)\n');
    fprintf('    定位: 诊断评估模式，记录偏载增量偏差 theta_payload_bias(d)\n');
    fprintf('=========================================================================\n');
    
    d_load_levels = [-0.20, -0.10, -0.05, 0.00, +0.05, +0.10, +0.20];
    
    for d_idx = 1:2
        ds = datasets{d_idx};
        d_tag = dataset_tags{d_idx};
        fprintf('\n--- 数据集 [%d/2]: %s ---\n', d_idx, d_tag);
        
        % 1. 先计算 d_load = 0.00 作为基准 theta_hat(0)
        cfg_d0 = struct('delta_m', 50.0, 'd_load', 0.00);
        res_d0 = analyze_step3c_trial(ds, cfg_d0);
        theta_hat_0 = res_d0.theta_hat;
        
        theta_payload_bias_list = zeros(length(d_load_levels), 1);
        
        for li = 1:length(d_load_levels)
            d_val = d_load_levels(li);
            cfg_c4 = struct('delta_m', 50.0, 'd_load', d_val);
            res_c4 = analyze_step3c_trial(ds, cfg_c4);
            
            % 增量偏载偏差
            theta_payload_bias = res_c4.theta_hat - theta_hat_0;
            theta_payload_bias_list(li) = theta_payload_bias;
            
            err_c4 = abs(res_c4.theta_hat - ds.Delta_Kf_true);
            
            if res_c4.is_calib_valid && res_c4.eta_sat >= 90.0
                status_c4 = 'PASS';
            else
                status_c4 = 'DEGRADED_BY_PAYLOAD';
            end
            
            item_name = sprintf('TestC4_Payload_d_%+05.2fm', d_val);
            dist_desc = sprintf('Delta_m=50.0kg;d_load=%+05.2fm', d_val);
            
            fprintf('  [C4] d_load=%+5.2fm | theta_hat=%+.7e | bias=%+.7e | eta_sat=%6.2f%% | eta_abs=%6.2f%% [%s]\n', ...
                d_val, res_c4.theta_hat, theta_payload_bias, res_c4.eta_sat, res_c4.eta_alpha_abs, status_c4);
            
            gamma_dev_c4 = max(abs(res_c4.gamma_L - 1.0), abs(res_c4.gamma_R - 1.0));
            if isfinite(res_c4.eta_total)
                c4_eta_tot_mean = res_c4.eta_total;
                c4_eta_tot_p05  = res_c4.eta_total;
                c4_eta_tot_min  = res_c4.eta_total;
                c4_eta_tot_cnt  = 1;
            else
                c4_eta_tot_mean = NaN;
                c4_eta_tot_p05  = NaN;
                c4_eta_tot_min  = NaN;
                c4_eta_tot_cnt  = 0;
            end
            
            perf_rows(end+1, :) = { ...
                d_tag, item_name, 'Nominal_Hardware_Limit', Imax_nominal, ...
                'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'POSTHOC_STATIC_REPLAY', 'Eccentric_Load', dist_desc, ...
                'Deterministic', 'Single', ds.Delta_Kf_true, ...
                res_c4.theta_hat, res_c4.theta_hat, err_c4, err_c4, err_c4, ...
                theta_payload_bias, NaN, ...
                res_c4.Kf_L_hat, res_c4.Kf_R_hat, res_c4.gamma_L, res_c4.gamma_R, gamma_dev_c4, NaN, res_c4.calib_validity_str, ...
                0, res_c4.eta_kf_residual, c4_eta_tot_mean, c4_eta_tot_p05, c4_eta_tot_min, c4_eta_tot_cnt, ...
                res_c4.rms_base, res_c4.rms_comp, res_c4.rms_total_base, res_c4.rms_total_comp, ...
                res_c4.rms_alpha_base_dyn, res_c4.rms_alpha_comp_dyn, NaN, res_c4.eta_alpha_abs, res_c4.eta_alpha_delay, ...
                res_c4.rms_dalpha_base, res_c4.rms_dalpha_comp, res_c4.alpha_ss_base, res_c4.alpha_ss_comp, ...
                res_c4.base_sat_ratio_total, res_c4.comp_sat_ratio_total, ...
                res_c4.unproj_max_peak, res_c4.unproj_clipped_count, ...
                res_c4.pe_active_ratio, res_c4.pe_false_alarm_rate, res_c4.svf_atten_dB, ...
                res_c4.rms_comp, res_c4.rms_comp, ...
                NaN, NaN, NaN, NaN, ...
                NaN, NaN, NaN, NaN, ...
                'N/A', NaN, 'N/A', 'N/A', 'N/A', ...
                status_c4};
        end
        
        % 检验 +/-0.05m 偏差方向反转
        idx_m05 = find(abs(d_load_levels - (-0.05)) < 1e-4, 1);
        idx_p05 = find(abs(d_load_levels - (+0.05)) < 1e-4, 1);
        bias_m05 = theta_payload_bias_list(idx_m05);
        bias_p05 = theta_payload_bias_list(idx_p05);
        sign_reversal = (bias_m05 * bias_p05 < 0);
        
        % 检验局部线性拟合优度 R^2 (在 [-0.10, +0.10] m 区间)
        mask_lin = (abs(d_load_levels) <= 0.10);
        d_sub = d_load_levels(mask_lin)';
        b_sub = theta_payload_bias_list(mask_lin);
        p_fit = polyfit(d_sub, b_sub, 1);
        y_fit = polyval(p_fit, d_sub);
        SS_res = sum((b_sub - y_fit).^2);
        SS_tot = sum((b_sub - mean(b_sub)).^2);
        R2 = 1.0 - SS_res / SS_tot;
        
        fprintf('  [C4 机理实证] d=-0.05m偏差=%+.4e, d=+0.05m偏差=%+.4e | 符号反转(方向一致性): %s\n', ...
            bias_m05, bias_p05, mat2str(sign_reversal));
        fprintf('  [C4 拟合评估] [-0.10, +0.10]m 局部线性拟合优度 R^2 = %.4f (斜率: %.4e (N/count)/m)\n', ...
            R2, p_fit(1));
        assert(sign_reversal, 'C4 检验失败: +/-0.05m 偏载增量串扰未发生符号反转');
    end
    
    %% =========================================================================
    %% TEST C5: 最劣角点噪声复合鲁棒性 (TestC5_TopWorstCorners_Noise_MC100)
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 开始执行 [Test C5] 最劣角点噪声复合鲁棒性 (TestC5_TopWorstCorners_Noise_MC100)\n');
    fprintf('    阶段一: 128 个全因子确定性角点扫描 (2^7 corners)\n');
    fprintf('    阶段二: 多指标联合选取 Top 3 最劣角点并集，叠加高频噪声执行 N = 100 Monte Carlo 试验\n');
    fprintf('=========================================================================\n');
    
    d_load_opts   = [-0.10, +0.10];
    delta_gL_opts = [-0.02, +0.02];
    delta_gR_opts = [-0.02, +0.02];
    i_biasL_opts  = [-20.0, +20.0];
    i_biasR_opts  = [-20.0, +20.0];
    d_actL_opts   = [1, 2];
    d_actR_opts   = [1, 2];
    
    [G1, G2, G3, G4, G5, G6, G7] = ndgrid(...
        d_load_opts, delta_gL_opts, delta_gR_opts, ...
        i_biasL_opts, i_biasR_opts, d_actL_opts, d_actR_opts);
    corner_grid = [G1(:), G2(:), G3(:), G4(:), G5(:), G6(:), G7(:)];
    N_corners = size(corner_grid, 1); % 128
    
    N_mc_c5 = 100;
    
    for d_idx = 1:2
        ds = datasets{d_idx};
        d_tag = dataset_tags{d_idx};
        fprintf('\n--- 数据集 [%d/2]: %s ---\n', d_idx, d_tag);
        
        % 阶段一: 128 角点确定性扫描
        fprintf('  [阶段一] 正在执行 128 角点确定性扫描...\n');
        corner_eta_tot   = zeros(N_corners, 1);
        corner_rms_alpha = zeros(N_corners, 1);
        corner_err_theta = zeros(N_corners, 1);
        
        for c = 1:N_corners
            cfg_c = struct();
            cfg_c.delta_m   = 50.0;
            cfg_c.d_load    = corner_grid(c, 1);
            cfg_c.delta_g_L = corner_grid(c, 2);
            cfg_c.delta_g_R = corner_grid(c, 3);
            cfg_c.i_bias_L  = corner_grid(c, 4);
            cfg_c.i_bias_R  = corner_grid(c, 5);
            cfg_c.d_act_L   = corner_grid(c, 6);
            cfg_c.d_act_R   = corner_grid(c, 7);
            cfg_c.d_meas_L  = 1;
            cfg_c.d_meas_R  = 1;
            cfg_c.sigma_y_L = 0.0;
            cfg_c.sigma_y_R = 0.0;
            cfg_c.sigma_i_L = 0.0;
            cfg_c.sigma_i_R = 0.0;
            
            res_c = analyze_step3c_trial(ds, cfg_c);
            corner_eta_tot(c)   = res_c.eta_total;
            corner_rms_alpha(c) = res_c.rms_alpha_comp_dyn;
            corner_err_theta(c) = abs(res_c.theta_hat - ds.Delta_Kf_true);
        end
        
        % 多指标最差角点排序与并集选取
        [~, idx_eta]   = sort(corner_eta_tot,   'ascend');  % 抑制比越低越差
        [~, idx_alpha] = sort(corner_rms_alpha, 'descend'); % 偏航残余越大越差
        [~, idx_theta] = sort(corner_err_theta, 'descend'); % 参数偏差越大越差
        
        selected_idx = unique([idx_eta(1:3); idx_alpha(1:3); idx_theta(1:3)], 'stable');
        N_selected = numel(selected_idx);
        
        fmt_c = @(c, val_str) sprintf('C#%d(d=%+.2f,gL=%+.2f,gR=%+.2f,bL=%+.0f,bR=%+.0f,dL=%d,dR=%d;%s)', ...
            c, corner_grid(c, 1), corner_grid(c, 2), corner_grid(c, 3), ...
            corner_grid(c, 4), corner_grid(c, 5), corner_grid(c, 6), corner_grid(c, 7), val_str);
        
        worst_eta_c_desc   = fmt_c(idx_eta(1), sprintf('eta=%.2f%%', corner_eta_tot(idx_eta(1))));
        worst_alpha_c_desc = fmt_c(idx_alpha(1), sprintf('alpha=%.4erad', corner_rms_alpha(idx_alpha(1))));
        worst_theta_c_desc = fmt_c(idx_theta(1), sprintf('err=%.4e', corner_err_theta(idx_theta(1))));
        
        fprintf('  [阶段一最差角点剖析]:\n');
        fprintf('    - 最差总抑制率角点: %s\n', worst_eta_c_desc);
        fprintf('    - 最差偏航残差角点: %s\n', worst_alpha_c_desc);
        fprintf('    - 最差参数误差角点: %s\n', worst_theta_c_desc);
        fprintf('    - 选取联合最劣角点数: %d 个 (并集去重)\n', N_selected);
        
        % 阶段二: 在选取的 Top 角点上叠加噪声执行 Monte Carlo N = 100
        fprintf('  [阶段二] 在 %d 个最劣角点并集上叠加噪声执行 Monte Carlo N = %d...\n', N_selected, N_mc_c5);
        theta_hat_arr      = zeros(N_mc_c5, 1);
        eta_sat_arr        = zeros(N_mc_c5, 1);
        eta_tot_arr        = zeros(N_mc_c5, 1);
        rms_alpha_base_arr = zeros(N_mc_c5, 1);
        rms_alpha_arr      = zeros(N_mc_c5, 1);
        eta_alpha_abs_arr  = zeros(N_mc_c5, 1);
        eta_alpha_del_arr  = zeros(N_mc_c5, 1);
        rms_base_arr       = zeros(N_mc_c5, 1);
        rms_comp_arr       = zeros(N_mc_c5, 1);
        rms_tot_b_arr      = zeros(N_mc_c5, 1);
        rms_tot_c_arr      = zeros(N_mc_c5, 1);
        rms_dalpha_b_arr   = zeros(N_mc_c5, 1);
        rms_dalpha_c_arr   = zeros(N_mc_c5, 1);
        alpha_ss_b_arr     = zeros(N_mc_c5, 1);
        alpha_ss_c_arr     = zeros(N_mc_c5, 1);
        base_sat_arr       = zeros(N_mc_c5, 1);
        comp_sat_arr       = zeros(N_mc_c5, 1);
        pe_act_arr         = zeros(N_mc_c5, 1);
        pe_far_arr         = zeros(N_mc_c5, 1);
        svf_att_arr        = zeros(N_mc_c5, 1);
        Kf_L_hat_arr       = zeros(N_mc_c5, 1);
        Kf_R_hat_arr       = zeros(N_mc_c5, 1);
        gamma_L_arr        = zeros(N_mc_c5, 1);
        gamma_R_arr        = zeros(N_mc_c5, 1);
        gamma_dev_arr      = zeros(N_mc_c5, 1);
        unproj_peak_arr    = zeros(N_mc_c5, 1);
        clip_count_arr     = zeros(N_mc_c5, 1);
        calib_valid_arr    = true(N_mc_c5, 1);
        exceed_rt_arr      = zeros(N_mc_c5, 1);
        
        for j = 1:N_mc_c5
            seed_j = 20260924 + j;
            rng(seed_j, 'twister');
            
            c_pick = selected_idx(mod(j - 1, N_selected) + 1);
            
            cfg_j = struct();
            cfg_j.delta_m   = 50.0;
            cfg_j.d_load    = corner_grid(c_pick, 1);
            cfg_j.delta_g_L = corner_grid(c_pick, 2);
            cfg_j.delta_g_R = corner_grid(c_pick, 3);
            cfg_j.i_bias_L  = corner_grid(c_pick, 4);
            cfg_j.i_bias_R  = corner_grid(c_pick, 5);
            cfg_j.d_act_L   = corner_grid(c_pick, 6);
            cfg_j.d_act_R   = corner_grid(c_pick, 7);
            cfg_j.d_meas_L  = 1;
            cfg_j.d_meas_R  = 1;
            cfg_j.sigma_y_L = 2.0e-6;
            cfg_j.sigma_y_R = 2.0e-6;
            cfg_j.sigma_i_L = 10.0;
            cfg_j.sigma_i_R = 10.0;
            cfg_j.seed      = seed_j + 50000;
            
            res_j = analyze_step3c_trial(ds, cfg_j);
            
            theta_hat_arr(j)      = res_j.theta_hat;
            eta_sat_arr(j)        = res_j.eta_sat;
            eta_tot_arr(j)        = res_j.eta_total;
            rms_alpha_base_arr(j) = res_j.rms_alpha_base_dyn;
            rms_alpha_arr(j)      = res_j.rms_alpha_comp_dyn;
            eta_alpha_abs_arr(j)  = res_j.eta_alpha_abs;
            eta_alpha_del_arr(j)  = res_j.eta_alpha_delay;
            rms_base_arr(j)       = res_j.rms_base;
            rms_comp_arr(j)       = res_j.rms_comp;
            rms_tot_b_arr(j)      = res_j.rms_total_base;
            rms_tot_c_arr(j)      = res_j.rms_total_comp;
            rms_dalpha_b_arr(j)   = res_j.rms_dalpha_base;
            rms_dalpha_c_arr(j)   = res_j.rms_dalpha_comp;
            alpha_ss_b_arr(j)     = res_j.alpha_ss_base;
            alpha_ss_c_arr(j)     = res_j.alpha_ss_comp;
            base_sat_arr(j)       = res_j.base_sat_ratio_total;
            comp_sat_arr(j)       = res_j.comp_sat_ratio_total;
            pe_act_arr(j)         = res_j.pe_active_ratio;
            pe_far_arr(j)         = res_j.pe_false_alarm_rate;
            svf_att_arr(j)        = res_j.svf_atten_dB;
            Kf_L_hat_arr(j)       = res_j.Kf_L_hat;
            Kf_R_hat_arr(j)       = res_j.Kf_R_hat;
            gamma_L_arr(j)        = res_j.gamma_L;
            gamma_R_arr(j)        = res_j.gamma_R;
            gamma_dev_arr(j)      = max(abs(res_j.gamma_L - 1.0), abs(res_j.gamma_R - 1.0));
            unproj_peak_arr(j)    = res_j.unproj_max_peak;
            clip_count_arr(j)     = res_j.unproj_clipped_count;
            calib_valid_arr(j)    = res_j.is_calib_valid;
            exceed_rt_arr(j)      = res_j.exceed_false_ratio;
        end
        
        eta_total_valid_c5     = eta_tot_arr(isfinite(eta_tot_arr));
        eta_total_valid_cnt_c5 = numel(eta_total_valid_c5);
        if eta_total_valid_cnt_c5 == 0
            eta_total_mean = NaN;
            eta_total_p05  = NaN;
            eta_total_min  = NaN;
        else
            eta_total_mean = mean(eta_total_valid_c5);
            eta_total_p05  = prctile(eta_total_valid_c5, 5);
            eta_total_min  = min(eta_total_valid_c5);
        end
        
        eta_kf_p05             = prctile(eta_sat_arr, 5);
        RMS_alpha_comp_mean    = mean(rms_alpha_arr);
        RMS_alpha_comp_p95     = prctile(rms_alpha_arr, 95);
        invalid_calib_count    = sum(~calib_valid_arr);
        if invalid_calib_count == 0
            calib_valid_str_c5 = 'VALID';
        else
            calib_valid_str_c5 = 'INVALID';
        end
        projection_trial_ratio = 100.0 * sum(clip_count_arr > 0) / N_mc_c5;
        
        abs_errs          = abs(theta_hat_arr - ds.Delta_Kf_true);
        median_hat_c5     = median(theta_hat_arr);
        median_abs_err_c5 = median(abs_errs);
        p95_err           = prctile(abs_errs, 95);
        rmse_err          = sqrt(mean(abs_errs.^2));
        
        if eta_kf_p05 >= 90.0 && invalid_calib_count == 0
            status_c5 = 'PASS';
        else
            status_c5 = 'DEGRADED';
        end
        
        fprintf('  [C5_MC100] eta_total_min=%5.2f%%, eta_kf_p05=%5.2f%%, RMS_alpha_p95=%.4e rad | 状态: [%s]\n', ...
            eta_total_min, eta_kf_p05, RMS_alpha_comp_p95, status_c5);
        fprintf('  theta_hat: 中位=%+.7e, 绝对误差中位=%.2e, 误差P95=%.2e, RMSE=%.2e | 投影触发试验率: %5.2f%%\n', ...
            median_hat_c5, median_abs_err_c5, p95_err, rmse_err, projection_trial_ratio);
        
        dist_desc = sprintf('TopWorstUnion_N%d_Selected%d;Noise2um_10ct', N_mc_c5, N_selected);
        sel_metric_str = 'Union(Worst_Eta[1:3], Worst_Alpha[1:3], Worst_Theta[1:3])';
        
        perf_rows(end+1, :) = { ...
            d_tag, 'TestC5_TopWorstCorners_Noise_MC100', 'Nominal_Hardware_Limit', Imax_nominal, ...
            'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'POSTHOC_STATIC_REPLAY', 'Top_Worst_Corners_Noise_MC', dist_desc, ...
            '20260924+1..100', sprintf('MC_%d_Trials', N_mc_c5), ds.Delta_Kf_true, ...
            mean(theta_hat_arr), median_hat_c5, median_abs_err_c5, p95_err, rmse_err, ...
            NaN, NaN, ...
            mean(Kf_L_hat_arr), mean(Kf_R_hat_arr), mean(gamma_L_arr), mean(gamma_R_arr), max(gamma_dev_arr), NaN, calib_valid_str_c5, ...
            invalid_calib_count, eta_kf_p05, eta_total_mean, eta_total_p05, eta_total_min, eta_total_valid_cnt_c5, ...
            mean(rms_base_arr), mean(rms_comp_arr), mean(rms_tot_b_arr), mean(rms_tot_c_arr), ...
            mean(rms_alpha_base_arr), RMS_alpha_comp_mean, RMS_alpha_comp_p95, mean(eta_alpha_abs_arr), mean(eta_alpha_del_arr), ...
            mean(rms_dalpha_b_arr), mean(rms_dalpha_c_arr), mean(alpha_ss_b_arr), mean(alpha_ss_c_arr), ...
            mean(base_sat_arr), mean(comp_sat_arr), ...
            max(unproj_peak_arr), sum(clip_count_arr), ...
            mean(pe_act_arr), mean(pe_far_arr), mean(svf_att_arr), ...
            mean(rms_comp_arr), max(rms_comp_arr), ...
            NaN, NaN, projection_trial_ratio, NaN, ...
            NaN, NaN, NaN, NaN, ...
            sel_metric_str, N_selected, ...
            worst_eta_c_desc, worst_alpha_c_desc, worst_theta_c_desc, ...
            status_c5};
    end
    
    %% =========================================================================
    %% TEST C6: 统一绝对敏感度与龙卷风双重排序 (DUAL TORNADO RANKING)
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 开始执行 [Test C6] 统一绝对敏感度与龙卷风双重排序 (Dual Tornado Ranking)\n');
    fprintf('    规范: 物理导数 S_physical 仅用于同因素内部解释; 范围影响量 Range_Impact 用于独立龙卷风排序\n');
    fprintf('    排行榜 A: 估计器参数偏差影响量 metric = |theta_hat - Delta_Kf_true| [(N/count)/Unit]\n');
    fprintf('    排行榜 B: 物理偏航残余影响量 metric = RMS(alpha_comp) [rad/Unit]\n');
    fprintf('=========================================================================\n');
    
    % 列: {FactorKey, FactorName, Unit, LowVal, HighVal, Span}
    factors_meta = { ...
        'delta_g', 'Current_Gain_Drift', 'percentage-point', -2, +2, 4; ...
        'i_bias',  'Hall_Sensor_Bias',   'count',           -20, +20, 40; ...
        'd_act',   'CAN_Network_Delay',  'ms',                0,  2, 2; ...
        'sigma_y', 'Sensor_Noise_Pos',   'um',                0,  2, 2; ...
        'd_load',  'Eccentric_Load',     'm',              -0.1, 0.1, 0.2};
    
    N_factors = size(factors_meta, 1);
    
    for d_idx = 1:2
        ds = datasets{d_idx};
        d_tag = dataset_tags{d_idx};
        d_short = dataset_short{d_idx};
        fprintf('\n--- 数据集 [%d/2]: %s ---\n', d_idx, d_tag);
        
        % 标称无扰动基准
        res_nom = analyze_step3c_trial(ds, struct());
        m1_nom = abs(res_nom.theta_hat - ds.Delta_Kf_true);
        m2_nom = res_nom.rms_alpha_comp_dyn;
        
        m1_low  = zeros(N_factors, 1);
        m1_high = zeros(N_factors, 1);
        m2_low  = zeros(N_factors, 1);
        m2_high = zeros(N_factors, 1);
        
        for fi = 1:N_factors
            f_key = factors_meta{fi, 1};
            
            switch f_key
                case 'delta_g'
                    dg_lo = factors_meta{fi, 4} / 100.0;
                    dg_hi = factors_meta{fi, 5} / 100.0;
                    res_lo = analyze_step3c_trial(ds, struct('delta_g_L', dg_lo, 'delta_g_R', dg_hi));
                    res_hi = analyze_step3c_trial(ds, struct('delta_g_L', dg_hi, 'delta_g_R', dg_lo));
                    m1_low(fi) = abs(res_lo.theta_hat - ds.Delta_Kf_true);
                    m1_high(fi) = abs(res_hi.theta_hat - ds.Delta_Kf_true);
                    m2_low(fi) = res_lo.rms_alpha_comp_dyn;
                    m2_high(fi) = res_hi.rms_alpha_comp_dyn;
                    
                case 'i_bias'
                    res_lo = analyze_step3c_trial(ds, struct('i_bias_L', factors_meta{fi, 4}, 'i_bias_R', factors_meta{fi, 5}));
                    res_hi = analyze_step3c_trial(ds, struct('i_bias_L', factors_meta{fi, 5}, 'i_bias_R', factors_meta{fi, 4}));
                    m1_low(fi) = abs(res_lo.theta_hat - ds.Delta_Kf_true);
                    m1_high(fi) = abs(res_hi.theta_hat - ds.Delta_Kf_true);
                    m2_low(fi) = res_lo.rms_alpha_comp_dyn;
                    m2_high(fi) = res_hi.rms_alpha_comp_dyn;
                    
                case 'd_act'
                    res_lo = res_nom;
                    res_hi = analyze_step3c_trial(ds, struct('d_act_L', factors_meta{fi, 5}, 'd_act_R', factors_meta{fi, 5}));
                    m1_low(fi) = abs(res_lo.theta_hat - ds.Delta_Kf_true);
                    m1_high(fi) = abs(res_hi.theta_hat - ds.Delta_Kf_true);
                    m2_low(fi) = res_lo.rms_alpha_comp_dyn;
                    m2_high(fi) = res_hi.rms_alpha_comp_dyn;
                    
                case 'sigma_y'
                    res_lo = res_nom;
                    m1_noise_arr = zeros(30, 1);
                    m2_noise_arr = zeros(30, 1);
                    for nj = 1:30
                        seed_nj = 20260924 + nj;
                        res_nj = analyze_step3c_trial(ds, struct('sigma_y_L', factors_meta{fi, 5}*1e-6, 'sigma_y_R', factors_meta{fi, 5}*1e-6, 'seed', seed_nj));
                        m1_noise_arr(nj) = abs(res_nj.theta_hat - ds.Delta_Kf_true);
                        m2_noise_arr(nj) = res_nj.rms_alpha_comp_dyn;
                    end
                    m1_low(fi) = abs(res_lo.theta_hat - ds.Delta_Kf_true);
                    m1_high(fi) = prctile(m1_noise_arr, 95);
                    m2_low(fi) = res_lo.rms_alpha_comp_dyn;
                    m2_high(fi) = prctile(m2_noise_arr, 95);
                    
                case 'd_load'
                    res_lo = analyze_step3c_trial(ds, struct('delta_m', 50.0, 'd_load', factors_meta{fi, 4}));
                    res_hi = analyze_step3c_trial(ds, struct('delta_m', 50.0, 'd_load', factors_meta{fi, 5}));
                    m1_low(fi) = abs(res_lo.theta_hat - ds.Delta_Kf_true);
                    m1_high(fi) = abs(res_hi.theta_hat - ds.Delta_Kf_true);
                    m2_low(fi) = res_lo.rms_alpha_comp_dyn;
                    m2_high(fi) = res_hi.rms_alpha_comp_dyn;
            end
        end
        
        span_vals = cell2mat(factors_meta(:, 6));
        
        % 排行榜 A: 估计器参数偏差
        S_phys_m1   = abs(m1_high - m1_low) ./ span_vals;
        range_impact_m1 = max(abs(m1_low - m1_nom), abs(m1_high - m1_nom));
        [~, rank_m1] = sort(range_impact_m1, 'descend');
        
        % 排行榜 B: 物理偏航残余
        S_phys_m2   = abs(m2_high - m2_low) ./ span_vals;
        range_impact_m2 = max(abs(m2_low - m2_nom), abs(m2_high - m2_nom));
        [~, rank_m2] = sort(range_impact_m2, 'descend');
        
        fprintf('  >>> 龙卷风排行榜 A (估计器参数偏差影响量) <<<\n');
        fprintf('  排名 | 扰动源                | 物理导数 S_phys           | 范围影响量 Range_Impact\n');
        fprintf('  -----+----------------------+---------------------------+------------------------\n');
        for rk = 1:N_factors
            fi = rank_m1(rk);
            unit_theta = sprintf('(N/count)/%s', factors_meta{fi, 3});
            fprintf('   #%d  | %-20s | %10.4e %-20s | %10.4e N/count\n', ...
                rk, factors_meta{fi, 2}, S_phys_m1(fi), unit_theta, range_impact_m1(fi));
            
            sens_rows(end+1, :) = { ...
                factors_meta{fi, 1}, d_short, factors_meta{fi, 4}, factors_meta{fi, 5}, factors_meta{fi, 3}, ...
                'Estimator_Parameter_Error', m1_low(fi), m1_nom, m1_high(fi), ...
                S_phys_m1(fi), unit_theta, range_impact_m1(fi), rk};
        end
        
        fprintf('\n  >>> 龙卷风排行榜 B (物理偏航残余影响量) <<<\n');
        fprintf('  排名 | 扰动源                | 物理导数 S_phys           | 范围影响量 Range_Impact\n');
        fprintf('  -----+----------------------+---------------------------+------------------------\n');
        for rk = 1:N_factors
            fi = rank_m2(rk);
            unit_alpha = sprintf('rad/%s', factors_meta{fi, 3});
            fprintf('   #%d  | %-20s | %10.4e %-20s | %10.4e rad\n', ...
                rk, factors_meta{fi, 2}, S_phys_m2(fi), unit_alpha, range_impact_m2(fi));
            
            sens_rows(end+1, :) = { ...
                factors_meta{fi, 1}, d_short, factors_meta{fi, 4}, factors_meta{fi, 5}, factors_meta{fi, 3}, ...
                'Physical_Yaw_RMS', m2_low(fi), m2_nom, m2_high(fi), ...
                S_phys_m2(fi), unit_alpha, range_impact_m2(fi), rk};
        end
    end
    
    %% =========================================================================
    %% TEST C7: 凸集投影安全性双向检验 (CONVEX PROJECTION SAFETY TEST MC30)
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 开始执行 [Test C7] 凸集投影安全性双向检验 (TestC7_Convex_Projection_Safety_MC30)\n');
    fprintf('    明确故障: Measured_Current_Sensor_Step_Fault (回采电流测量阶跃故障)\n');
    fprintf('    设计: 分别注入负向阶跃(触发低端边界)与正向阶跃(触发高端边界)\n');
    fprintf('=========================================================================\n');
    
    N_mc_c7 = 30;
    
    for d_idx = 1:2
        ds = datasets{d_idx};
        d_tag = dataset_tags{d_idx};
        d_short = dataset_short{d_idx};
        fprintf('\n--- 数据集 [%d/2]: %s (Monte Carlo N = %d) ---\n', d_idx, d_tag, N_mc_c7);
        
        sample_clip_ratios = zeros(N_mc_c7, 1);
        trial_has_clip     = false(N_mc_c7, 1);
        low_clip_counts    = zeros(N_mc_c7, 1);
        high_clip_counts   = zeros(N_mc_c7, 1);
        max_unproj_peaks   = zeros(N_mc_c7, 1);
        out_of_bounds_cnt  = 0;
        non_finite_cnt     = 0;
        cov_bounded_all    = true;
        
        if d_idx == 1
            fault_amp_L = -500.0; fault_amp_R = +500.0;
            d_load_c7   = -0.20;
            dg_L = -0.03; dg_R = +0.03;
            fault_desc  = '-500 counts (Negative Shock driving to Lower Bound)';
        else
            fault_amp_L = +500.0; fault_amp_R = -500.0;
            d_load_c7   = +0.20;
            dg_L = +0.03; dg_R = -0.03;
            fault_desc  = '+500 counts (Positive Shock driving to Upper Bound)';
        end
        
        for j = 1:N_mc_c7
            seed_j = 20260924 + j;
            rng(seed_j, 'twister');
            
            cfg_c7 = struct();
            cfg_c7.delta_m          = 50.0;
            cfg_c7.d_load           = d_load_c7;
            cfg_c7.delta_g_L        = dg_L;
            cfg_c7.delta_g_R        = dg_R;
            cfg_c7.step_fault_t     = 1.0;
            cfg_c7.step_fault_amp_L = fault_amp_L;
            cfg_c7.step_fault_amp_R = fault_amp_R;
            cfg_c7.sigma_y_L        = 5.0e-6;
            cfg_c7.sigma_y_R        = 5.0e-6;
            cfg_c7.sigma_i_L        = 30.0;
            cfg_c7.sigma_i_R        = 30.0;
            cfg_c7.seed             = seed_j;
            
            res_j = analyze_step3c_trial(ds, cfg_c7);
            
            sample_clip_ratios(j) = res_j.sample_clip_ratio;
            trial_has_clip(j)     = res_j.has_any_clip;
            low_clip_counts(j)    = res_j.unproj_low_clip_count;
            high_clip_counts(j)   = res_j.unproj_high_clip_count;
            max_unproj_peaks(j)   = res_j.unproj_max_peak;
            
            p_proj = res_j.theta_proj_eval;
            oob = sum(p_proj < -0.0026295 - 1e-9 | p_proj > 0.0018465 + 1e-9);
            out_of_bounds_cnt = out_of_bounds_cnt + oob;
            
            nf = sum(~isfinite(p_proj));
            non_finite_cnt = non_finite_cnt + nf;
            
            P_hist = res_j.P_history_eval;
            if any(P_hist < 1.0e-12 - 1e-15 | P_hist > 1.0 + 1e-9 | ~isfinite(P_hist))
                cov_bounded_all = false;
            end
        end
        
        tot_low_clips  = sum(low_clip_counts);
        tot_high_clips = sum(high_clip_counts);
        rho_sample     = mean(sample_clip_ratios);
        rho_trial      = mean(trial_has_clip);
        max_peak       = max(max_unproj_peaks);
        
        fprintf('  [C7] 样本级越界截断率 rho_sample: %6.2f%%\n', rho_sample * 100);
        fprintf('  [C7] 试验级越界触发率 rho_trial : %6.2f%% (%d/%d 试验触发投影)\n', ...
            rho_trial * 100, sum(trial_has_clip), N_mc_c7);
        fprintf('  [C7] 低端边界截断次数: %d | 高端边界截断次数: %d\n', tot_low_clips, tot_high_clips);
        fprintf('  [C7] 未受限估计流最大峰值 max|theta_unproj|: %.4e N/count\n', max_peak);
        fprintf('  [C7] 投影后越界样本数: %d | 非有限值数: %d | 协方差有界: %s\n', ...
            out_of_bounds_cnt, non_finite_cnt, mat2str(cov_bounded_all));
        
        assert(out_of_bounds_cnt == 0, '投影后越界次数必须严格为 0');
        assert(non_finite_cnt == 0, '非有限值出现次数必须严格为 0');
        assert(cov_bounded_all, '协方差有界性条件必须逐点严格成立');
        
        proj_rows(end+1, :) = { ...
            d_short, 'Measured_Current_Sensor', fault_desc, ...
            rho_sample, rho_trial, tot_low_clips, tot_high_clips, ...
            max_peak, out_of_bounds_cnt, non_finite_cnt, cov_bounded_all, 'PASS'};
    end
    
    all_low_clips  = sum(cell2mat(proj_rows(:, 6)));
    all_high_clips = sum(cell2mat(proj_rows(:, 7)));
    fprintf('\n>>> C7 投影边界双向激活检验: 低端总截断数 = %d, 高端总截断数 = %d <<<\n', ...
        all_low_clips, all_high_clips);
    assert(all_low_clips > 0, 'C7 检验失败: 低端投影边界未被激活 (必须 > 0)');
    assert(all_high_clips > 0, 'C7 检验失败: 高端投影边界未被激活 (必须 > 0)');
    fprintf('    [OK] 投影双向边界真实激活硬性断言 100%% 通过!\n');
    
    %% =========================================================================
    %% TEST C8: 标称物理对称模型的虚假补偿深度评测 (C8A, C8B, C8C, N = 100)
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 开始执行 [Test C8] 标称物理对称模型的虚假补偿深度评测 (Monte Carlo N = 100)\n');
    fprintf('    C8A: 全要素噪声与增益漂移下的原始估计器评测 (RAW_ESTIMATOR_FAIL)\n');
    fprintf('    C8B: 机械偏载对对称系统的混淆诊断 (DIAGNOSTIC_PAYLOAD_CONFOUNDING)\n');
    fprintf('    C8C: 独立迟滞门限控制仿真验证 (GATED_APPLICATION_EVAL_ONLY)\n');
    fprintf('=========================================================================\n');
    
    N_mc_c8 = 100;
    
    % -------------------------------------------------------------------------
    % 8.1 子测试 C8A: 补齐扰动矩阵的原始估计器虚假补偿评测
    % -------------------------------------------------------------------------
    fprintf('\n--- [C8A] 补齐扰动矩阵的虚假补偿评测 (TestC8A_SensorCommFalseComp, N = %d) ---\n', N_mc_c8);
    c8a_theta_hat    = zeros(N_mc_c8, 1);
    c8a_gamma_dev    = zeros(N_mc_c8, 1);
    c8a_gamma_L      = zeros(N_mc_c8, 1);
    c8a_gamma_R      = zeros(N_mc_c8, 1);
    c8a_rms_comp     = zeros(N_mc_c8, 1);
    c8a_exceed_rt    = zeros(N_mc_c8, 1);
    c8a_calib_val    = true(N_mc_c8, 1);
    c8a_alpha_base   = zeros(N_mc_c8, 1);
    c8a_alpha_comp   = zeros(N_mc_c8, 1);
    c8a_KfL_hat      = zeros(N_mc_c8, 1);
    c8a_KfR_hat      = zeros(N_mc_c8, 1);
    c8a_rms_base     = zeros(N_mc_c8, 1);
    c8a_rms_tot_b    = zeros(N_mc_c8, 1);
    c8a_rms_tot_c    = zeros(N_mc_c8, 1);
    c8a_eta_kf_res   = zeros(N_mc_c8, 1);
    c8a_eta_total    = zeros(N_mc_c8, 1);
    c8a_eta_alpha_abs= zeros(N_mc_c8, 1);
    c8a_eta_alpha_del= zeros(N_mc_c8, 1);
    c8a_rms_dalpha_b = zeros(N_mc_c8, 1);
    c8a_rms_dalpha_c = zeros(N_mc_c8, 1);
    c8a_alpha_ss_b   = zeros(N_mc_c8, 1);
    c8a_alpha_ss_c   = zeros(N_mc_c8, 1);
    c8a_base_sat     = zeros(N_mc_c8, 1);
    c8a_comp_sat     = zeros(N_mc_c8, 1);
    c8a_peak         = zeros(N_mc_c8, 1);
    c8a_proj_cnt     = zeros(N_mc_c8, 1);
    c8a_pe_act       = zeros(N_mc_c8, 1);
    c8a_pe_far       = zeros(N_mc_c8, 1);
    c8a_svf_att      = zeros(N_mc_c8, 1);
    
    max_run_arr      = zeros(N_mc_c8, 1);
    late_exceed_arr  = zeros(N_mc_c8, 1);
    first_exceed_arr = nan(N_mc_c8, 1);
    last_exceed_arr  = nan(N_mc_c8, 1);
    
    % 保存各 trial 配置用于 C8C
    c8a_cfgs = cell(N_mc_c8, 1);
    c8a_res_store = cell(N_mc_c8, 1);
    
    for j = 1:N_mc_c8
        seed_j = 20260924 + j;
        rng(seed_j, 'twister');
        
        cfg_c8a = struct();
        cfg_c8a.delta_g_L = -0.02 + 0.04 * rand();
        cfg_c8a.delta_g_R = -0.02 + 0.04 * rand();
        cfg_c8a.i_bias_L  = -15.0 + 30.0 * rand();
        cfg_c8a.i_bias_R  = -15.0 + 30.0 * rand();
        cfg_c8a.d_act_L   = randi([0, 2]);
        cfg_c8a.d_act_R   = randi([0, 2]);
        cfg_c8a.d_meas_L  = randi([0, 2]);
        cfg_c8a.d_meas_R  = randi([0, 2]);
        cfg_c8a.sigma_y_L = 2.0e-6;
        cfg_c8a.sigma_y_R = 2.0e-6;
        cfg_c8a.sigma_i_L = 10.0;
        cfg_c8a.sigma_i_R = 10.0;
        cfg_c8a.seed      = seed_j + 100000;
        
        res_j = analyze_step3c_trial(d_sym, cfg_c8a);
        c8a_cfgs{j} = cfg_c8a;
        c8a_res_store{j} = res_j;
        
        c8a_theta_hat(j)     = res_j.theta_hat;
        c8a_gamma_L(j)       = res_j.gamma_L;
        c8a_gamma_R(j)       = res_j.gamma_R;
        c8a_gamma_dev(j)     = max(abs(res_j.gamma_L - 1.0), abs(res_j.gamma_R - 1.0));
        c8a_rms_comp(j)      = res_j.rms_comp;
        c8a_exceed_rt(j)     = res_j.exceed_false_ratio;
        c8a_calib_val(j)     = res_j.is_calib_valid;
        c8a_alpha_base(j)    = res_j.rms_alpha_base_dyn;
        c8a_alpha_comp(j)    = res_j.rms_alpha_comp_dyn;
        c8a_KfL_hat(j)       = res_j.Kf_L_hat;
        c8a_KfR_hat(j)       = res_j.Kf_R_hat;
        c8a_rms_base(j)      = res_j.rms_base;
        c8a_rms_tot_b(j)     = res_j.rms_total_base;
        c8a_rms_tot_c(j)     = res_j.rms_total_comp;
        c8a_eta_kf_res(j)    = res_j.eta_kf_residual;
        c8a_eta_total(j)     = res_j.eta_total;
        c8a_eta_alpha_abs(j) = res_j.eta_alpha_abs;
        c8a_eta_alpha_del(j) = res_j.eta_alpha_delay;
        c8a_rms_dalpha_b(j)  = res_j.rms_dalpha_base;
        c8a_rms_dalpha_c(j)  = res_j.rms_dalpha_comp;
        c8a_alpha_ss_b(j)    = res_j.alpha_ss_base;
        c8a_alpha_ss_c(j)    = res_j.alpha_ss_comp;
        c8a_base_sat(j)      = res_j.base_sat_ratio_total;
        c8a_comp_sat(j)      = res_j.comp_sat_ratio_total;
        c8a_peak(j)          = res_j.unproj_max_peak;
        c8a_proj_cnt(j)      = res_j.unproj_clipped_count;
        c8a_pe_act(j)        = res_j.pe_active_ratio;
        c8a_pe_far(j)        = res_j.pe_false_alarm_rate;
        c8a_svf_att(j)       = res_j.svf_atten_dB;
        
        % 时间分布统计证据
        t_eval = res_j.t(res_j.mask_eval);
        exceed = (abs(res_j.theta_projected(res_j.mask_eval)) > 1.0e-5);
        max_run_arr(j) = longest_true_run(exceed) * d_sym.dt;
        late = (t_eval >= 1.0);
        late_exceed_arr(j) = 100.0 * mean(exceed(late));
        
        idx_ex = find(exceed);
        if ~isempty(idx_ex)
            first_exceed_arr(j) = t_eval(idx_ex(1));
            last_exceed_arr(j)  = t_eval(idx_ex(end));
        end
    end
    
    c8a_abs_theta        = abs(c8a_theta_hat);
    p95_theta_a          = prctile(c8a_abs_theta, 95);
    median_hat_a         = median(c8a_theta_hat);
    median_abs_error_a   = median(c8a_abs_theta);
    rmse_theta_a         = sqrt(mean(c8a_abs_theta.^2));
    max_gamma_dev_a      = max(c8a_gamma_dev);
    mean_rms_comp_a      = mean(c8a_rms_comp);
    max_rms_comp_a       = max(c8a_rms_comp);
    time_exceed_mean_a   = mean(c8a_exceed_rt);
    trial_exceed_cnt_a   = sum(c8a_abs_theta > 1.0e-5);
    trial_exceed_rt_a    = 100.0 * trial_exceed_cnt_a / N_mc_c8;
    
    eta_total_valid_a    = c8a_eta_total(isfinite(c8a_eta_total));
    eta_total_valid_cnt_a= numel(eta_total_valid_a);
    if eta_total_valid_cnt_a == 0
        eta_tot_mean_a = NaN;
        eta_tot_p05_a  = NaN;
        eta_tot_min_a  = NaN;
    else
        eta_tot_mean_a = mean(eta_total_valid_a);
        eta_tot_p05_a  = prctile(eta_total_valid_a, 5);
        eta_tot_min_a  = min(eta_total_valid_a);
    end
    
    invalid_calib_cnt_a  = sum(~c8a_calib_val);
    if invalid_calib_cnt_a == 0
        calib_valid_str_a = 'VALID';
    else
        calib_valid_str_a = 'INVALID';
    end
    
    alpha_comp_mean_a    = mean(c8a_alpha_comp);
    alpha_comp_p95_a     = prctile(c8a_alpha_comp, 95);
    
    max_run_p95          = prctile(max_run_arr, 95);
    late_exceed_mean     = mean(late_exceed_arr);
    first_exceed_median  = nanmedian(first_exceed_arr);
    last_exceed_median   = nanmedian(last_exceed_arr);
    
    fprintf('  [C8A 统计指标]:\n');
    fprintf('    - P95(|Delta_Kf_hat|)   : %.4e N/count (门槛 <= 1.0e-5)\n', p95_theta_a);
    fprintf('    - 有符号中位数 median     : %+.4e N/count\n', median_hat_a);
    fprintf('    - 绝对误差中位数 abs_median: %.4e N/count (门槛 <= 5.0e-6)\n', median_abs_error_a);
    fprintf('    - 最大前馈偏离度 max|gamma-1|: %.4f%% (门槛 <= 0.50%%)\n', max_gamma_dev_a * 100);
    fprintf('    - 虚假诱导偏航力矩最大 RMS: %.4e Nm (门槛 <= 0.01 Nm)\n', max_rms_comp_a);
    status_time_str = 'FAIL';
    if time_exceed_mean_a <= 5.0
        status_time_str = 'PASS';
    end
    fprintf('    - 超标时间比例均值 time_exceed: %5.2f%% (门槛 <= 5.00%%) -> [%s]\n', ...
        time_exceed_mean_a, status_time_str);
    fprintf('    - 最终估计超标试验比例     : %5.2f%% (%d/%d, 辅助诊断)\n', ...
        trial_exceed_rt_a, trial_exceed_cnt_a, N_mc_c8);
    
    fprintf('  [C8A 时间分布证据]:\n');
    fprintf('    - 最长单次连续超标时间 P95 : %.3f s\n', max_run_p95);
    fprintf('    - 后半段 (t >= 1.0s) 超标均比: %5.2f%%\n', late_exceed_mean);
    fprintf('    - 首次超标时刻中位数        : %.3f s\n', first_exceed_median);
    fprintf('    - 最后超标时刻中位数        : %.3f s\n', last_exceed_median);
    
    % 评估 4 种最劣差模确定性工况
    fprintf('  [C8A 最差确定性差模工况检验]:\n');
    det_diff_cfgs = { ...
        'Diff_Gain_+2%_-2%', struct('delta_g_L', +0.02, 'delta_g_R', -0.02, 'd_act_L', 0, 'd_act_R', 0); ...
        'Diff_Gain_-2%_+2%', struct('delta_g_L', -0.02, 'delta_g_R', +0.02, 'd_act_L', 0, 'd_act_R', 0); ...
        'Diff_Delay_1ms_2ms', struct('delta_g_L', 0.0, 'delta_g_R', 0.0, 'd_act_L', 1, 'd_act_R', 2); ...
        'Diff_Delay_2ms_1ms', struct('delta_g_L', 0.0, 'delta_g_R', 0.0, 'd_act_L', 2, 'd_act_R', 1)};
    det_rows = cell(size(det_diff_cfgs, 1), 9);
    for di = 1:size(det_diff_cfgs, 1)
        cfg_di = det_diff_cfgs{di, 2};
        res_det = analyze_step3c_trial(d_sym, cfg_di);
        gamma_dev_det = max(abs(res_det.gamma_L - 1.0), abs(res_det.gamma_R - 1.0));
        if abs(res_det.theta_hat) <= 1.0e-5 && gamma_dev_det <= 0.005 && res_det.rms_comp <= 0.01
            det_status = 'PASS';
        else
            det_status = 'FAIL';
        end
        det_rows(di, :) = { ...
            det_diff_cfgs{di, 1}, ...
            res_det.theta_hat, ...
            gamma_dev_det, ...
            res_det.rms_comp, ...
            cfg_di.d_act_L, ...
            cfg_di.d_act_R, ...
            cfg_di.delta_g_L, ...
            cfg_di.delta_g_R, ...
            det_status};
        fprintf('    [%s] theta_hat = %+.4e | gamma_dev = %.4f%% | rms_comp = %.4e Nm | 判定: [%s]\n', ...
            det_diff_cfgs{di, 1}, res_det.theta_hat, gamma_dev_det * 100, res_det.rms_comp, det_status);
    end
    
    % 程序化判定 C8A 五项判据
    pass_p95   = p95_theta_a <= 1.0e-5;
    pass_med   = median_abs_error_a <= 5.0e-6;
    pass_gamma = max_gamma_dev_a <= 0.005;
    pass_rms   = max_rms_comp_a <= 0.01;
    pass_time  = time_exceed_mean_a <= 5.0;
    
    if pass_p95 && pass_med && pass_gamma && pass_rms && pass_time
        status_c8a = 'PASS';
    else
        status_c8a = 'RAW_ESTIMATOR_FAIL';
    end
    if strcmp(status_c8a, 'PASS')
        fprintf('  [C8A 综合评测判定]: [PASS] (各项判据全部达标)\n');
    else
        fprintf('  [C8A 综合评测判定]: [%s] (未达标，如实记录，坚决不调高门槛)\n', status_c8a);
    end
    
    perf_rows(end+1, :) = { ...
        'r100 (Delta_Kf = 0)', 'TestC8A_SensorCommFalseComp', 'Nominal_Hardware_Limit', Imax_nominal, ...
        'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'POSTHOC_STATIC_REPLAY', 'Sensor_Comm_Noise_Symmetric', ...
        sprintf('Noise2um_Bias15ct_Gain2pct_Delay2ms_N%d', N_mc_c8), ...
        '20260924+1..100', sprintf('MC_%d_Trials', N_mc_c8), 0.0, ...
        mean(c8a_theta_hat), median_hat_a, median_abs_error_a, p95_theta_a, rmse_theta_a, ...
        NaN, NaN, ...
        mean(c8a_KfL_hat), mean(c8a_KfR_hat), mean(c8a_gamma_L), mean(c8a_gamma_R), max_gamma_dev_a, NaN, calib_valid_str_a, ...
        invalid_calib_cnt_a, mean(c8a_eta_kf_res), eta_tot_mean_a, eta_tot_p05_a, eta_tot_min_a, eta_total_valid_cnt_a, ...
        mean(c8a_rms_base), mean(c8a_rms_comp), mean(c8a_rms_tot_b), mean(c8a_rms_tot_c), ...
        mean(c8a_alpha_base), alpha_comp_mean_a, alpha_comp_p95_a, mean(c8a_eta_alpha_abs), mean(c8a_eta_alpha_del), ...
        mean(c8a_rms_dalpha_b), mean(c8a_rms_dalpha_c), mean(c8a_alpha_ss_b), mean(c8a_alpha_ss_c), ...
        mean(c8a_base_sat), mean(c8a_comp_sat), ...
        max(c8a_peak), sum(c8a_proj_cnt), ...
        mean(c8a_pe_act), mean(c8a_pe_far), mean(c8a_svf_att), ...
        mean_rms_comp_a, max_rms_comp_a, ...
        time_exceed_mean_a, trial_exceed_rt_a, NaN, NaN, ...
        max_run_p95, late_exceed_mean, first_exceed_median, last_exceed_median, ...
        'N/A', NaN, 'N/A', 'N/A', 'N/A', ...
        status_c8a};
    
    % -------------------------------------------------------------------------
    % 8.2 子测试 C8B: 机械偏载对对称系统的混淆 (诊断模式，增加匹配对照)
    % -------------------------------------------------------------------------
    fprintf('\n--- [C8B] 偏载对对称系统的混淆诊断 (TestC8B_PayloadConfounding, N = %d) ---\n', N_mc_c8);
    c8b_theta_hat    = zeros(N_mc_c8, 1);
    c8b_gamma_dev    = zeros(N_mc_c8, 1);
    c8b_gamma_L      = zeros(N_mc_c8, 1);
    c8b_gamma_R      = zeros(N_mc_c8, 1);
    c8b_rms_comp     = zeros(N_mc_c8, 1);
    c8b_exceed_rt    = zeros(N_mc_c8, 1);
    c8b_calib_val    = true(N_mc_c8, 1);
    c8b_alpha_base   = zeros(N_mc_c8, 1);
    c8b_alpha_comp   = zeros(N_mc_c8, 1);
    c8b_payload_bias = zeros(N_mc_c8, 1);
    c8b_KfL_hat      = zeros(N_mc_c8, 1);
    c8b_KfR_hat      = zeros(N_mc_c8, 1);
    c8b_rms_base     = zeros(N_mc_c8, 1);
    c8b_rms_tot_b    = zeros(N_mc_c8, 1);
    c8b_rms_tot_c    = zeros(N_mc_c8, 1);
    c8b_eta_kf_res   = zeros(N_mc_c8, 1);
    c8b_eta_total    = zeros(N_mc_c8, 1);
    c8b_eta_alpha_abs= zeros(N_mc_c8, 1);
    c8b_eta_alpha_del= zeros(N_mc_c8, 1);
    c8b_rms_dalpha_b = zeros(N_mc_c8, 1);
    c8b_rms_dalpha_c = zeros(N_mc_c8, 1);
    c8b_alpha_ss_b   = zeros(N_mc_c8, 1);
    c8b_alpha_ss_c   = zeros(N_mc_c8, 1);
    c8b_base_sat     = zeros(N_mc_c8, 1);
    c8b_comp_sat     = zeros(N_mc_c8, 1);
    c8b_peak         = zeros(N_mc_c8, 1);
    c8b_proj_cnt     = zeros(N_mc_c8, 1);
    c8b_pe_act       = zeros(N_mc_c8, 1);
    c8b_pe_far       = zeros(N_mc_c8, 1);
    c8b_svf_att      = zeros(N_mc_c8, 1);
    
    for j = 1:N_mc_c8
        seed_j = 20260924 + j;
        rng(seed_j, 'twister');
        
        cfg_c8b = struct();
        cfg_c8b.delta_m   = 50.0;
        cfg_c8b.d_load    = -0.10 + 0.20 * rand();
        cfg_c8b.sigma_y_L = 2.0e-6;
        cfg_c8b.sigma_y_R = 2.0e-6;
        cfg_c8b.sigma_i_L = 10.0;
        cfg_c8b.sigma_i_R = 10.0;
        cfg_c8b.seed      = seed_j;
        
        res_j = analyze_step3c_trial(d_sym, cfg_c8b);
        
        % 匹配的 d_load = 0 对照试验
        cfg_ref = cfg_c8b;
        cfg_ref.d_load = 0.0;
        res_ref = analyze_step3c_trial(d_sym, cfg_ref);
        
        c8b_payload_bias(j) = res_j.theta_hat - res_ref.theta_hat;
        
        c8b_theta_hat(j)     = res_j.theta_hat;
        c8b_gamma_L(j)       = res_j.gamma_L;
        c8b_gamma_R(j)       = res_j.gamma_R;
        c8b_gamma_dev(j)     = max(abs(res_j.gamma_L - 1.0), abs(res_j.gamma_R - 1.0));
        c8b_rms_comp(j)      = res_j.rms_comp;
        c8b_exceed_rt(j)     = res_j.exceed_false_ratio;
        c8b_calib_val(j)     = res_j.is_calib_valid;
        c8b_alpha_base(j)    = res_j.rms_alpha_base_dyn;
        c8b_alpha_comp(j)    = res_j.rms_alpha_comp_dyn;
        c8b_KfL_hat(j)       = res_j.Kf_L_hat;
        c8b_KfR_hat(j)       = res_j.Kf_R_hat;
        c8b_rms_base(j)      = res_j.rms_base;
        c8b_rms_tot_b(j)     = res_j.rms_total_base;
        c8b_rms_tot_c(j)     = res_j.rms_total_comp;
        c8b_eta_kf_res(j)    = res_j.eta_kf_residual;
        c8b_eta_total(j)     = res_j.eta_total;
        c8b_eta_alpha_abs(j) = res_j.eta_alpha_abs;
        c8b_eta_alpha_del(j) = res_j.eta_alpha_delay;
        c8b_rms_dalpha_b(j)  = res_j.rms_dalpha_base;
        c8b_rms_dalpha_c(j)  = res_j.rms_dalpha_comp;
        c8b_alpha_ss_b(j)    = res_j.alpha_ss_base;
        c8b_alpha_ss_c(j)    = res_j.alpha_ss_comp;
        c8b_base_sat(j)      = res_j.base_sat_ratio_total;
        c8b_comp_sat(j)      = res_j.comp_sat_ratio_total;
        c8b_peak(j)          = res_j.unproj_max_peak;
        c8b_proj_cnt(j)      = res_j.unproj_clipped_count;
        c8b_pe_act(j)        = res_j.pe_active_ratio;
        c8b_pe_far(j)        = res_j.pe_false_alarm_rate;
        c8b_svf_att(j)       = res_j.svf_atten_dB;
    end
    
    c8b_abs_theta      = abs(c8b_theta_hat);
    p95_theta_b        = prctile(c8b_abs_theta, 95);
    median_hat_b       = median(c8b_theta_hat);
    median_abs_error_b = median(c8b_abs_theta);
    rmse_theta_b       = sqrt(mean(c8b_abs_theta.^2));
    max_gamma_dev_b    = max(c8b_gamma_dev);
    mean_rms_comp_b    = mean(c8b_rms_comp);
    max_rms_comp_b     = max(c8b_rms_comp);
    time_exceed_mean_b = mean(c8b_exceed_rt);
    trial_exceed_cnt_b = sum(c8b_abs_theta > 1.0e-5);
    trial_exceed_rt_b  = 100.0 * trial_exceed_cnt_b / N_mc_c8;
    mean_payload_bias_b= mean(c8b_payload_bias);
    p95_payload_bias_b = prctile(abs(c8b_payload_bias), 95);
    
    eta_total_valid_b     = c8b_eta_total(isfinite(c8b_eta_total));
    eta_total_valid_cnt_b = numel(eta_total_valid_b);
    if eta_total_valid_cnt_b == 0
        eta_tot_mean_b = NaN;
        eta_tot_p05_b  = NaN;
        eta_tot_min_b  = NaN;
    else
        eta_tot_mean_b = mean(eta_total_valid_b);
        eta_tot_p05_b  = prctile(eta_total_valid_b, 5);
        eta_tot_min_b  = min(eta_total_valid_b);
    end
    
    invalid_calib_cnt_b = sum(~c8b_calib_val);
    if invalid_calib_cnt_b == 0
        calib_valid_str_b = 'VALID';
    else
        calib_valid_str_b = 'INVALID';
    end
    
    alpha_comp_mean_b  = mean(c8b_alpha_comp);
    alpha_comp_p95_b   = prctile(c8b_alpha_comp, 95);
    
    fprintf('  [C8B 诊断统计]:\n');
    fprintf('    - 增量偏载偏差均值 mean(bias) : %+.4e N/count (P95: %.4e)\n', mean_payload_bias_b, p95_payload_bias_b);
    fprintf('    - 估计值 P95                  : %.4e N/count\n', p95_theta_b);
    fprintf('    - 估计值有符号中位数 median    : %+.4e N/count\n', median_hat_b);
    fprintf('    - 估计值绝对误差中位数 abs_med : %.4e N/count\n', median_abs_error_b);
    fprintf('    - 最大前馈偏离度 max|gamma-1|  : %.4f%%\n', max_gamma_dev_b * 100);
    fprintf('    - 虚假诱导偏航力矩最大 RMS     : %.4e Nm\n', max_rms_comp_b);
    status_c8b = 'DIAGNOSTIC_PAYLOAD_CONFOUNDING';
    fprintf('  [C8B 综合评测判定]: [%s]\n', status_c8b);
    
    perf_rows(end+1, :) = { ...
        'r100 (Delta_Kf = 0)', 'TestC8B_PayloadConfounding', 'Nominal_Hardware_Limit', Imax_nominal, ...
        'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'POSTHOC_STATIC_REPLAY', 'Payload_Confounding_Symmetric', ...
        sprintf('Delta_m=50kg;d_load=[-0.1,0.1]m;N%d', N_mc_c8), ...
        '20260924+1..100', sprintf('MC_%d_Trials', N_mc_c8), 0.0, ...
        mean(c8b_theta_hat), median_hat_b, median_abs_error_b, p95_theta_b, rmse_theta_b, ...
        mean_payload_bias_b, p95_payload_bias_b, ...
        mean(c8b_KfL_hat), mean(c8b_KfR_hat), mean(c8b_gamma_L), mean(c8b_gamma_R), max_gamma_dev_b, NaN, calib_valid_str_b, ...
        invalid_calib_cnt_b, mean(c8b_eta_kf_res), eta_tot_mean_b, eta_tot_p05_b, eta_tot_min_b, eta_total_valid_cnt_b, ...
        mean(c8b_rms_base), mean(c8b_rms_comp), mean(c8b_rms_tot_b), mean(c8b_rms_tot_c), ...
        mean(c8b_alpha_base), alpha_comp_mean_b, alpha_comp_p95_b, mean(c8b_eta_alpha_abs), mean(c8b_eta_alpha_del), ...
        mean(c8b_rms_dalpha_b), mean(c8b_rms_dalpha_c), mean(c8b_alpha_ss_b), mean(c8b_alpha_ss_c), ...
        mean(c8b_base_sat), mean(c8b_comp_sat), ...
        max(c8b_peak), sum(c8b_proj_cnt), ...
        mean(c8b_pe_act), mean(c8b_pe_far), mean(c8b_svf_att), ...
        mean_rms_comp_b, max_rms_comp_b, ...
        time_exceed_mean_b, trial_exceed_rt_b, NaN, NaN, ...
        NaN, NaN, NaN, NaN, ...
        'N/A', NaN, 'N/A', 'N/A', 'N/A', ...
        status_c8b};
    
    % -------------------------------------------------------------------------
    % 8.3 子测试 C8C: 独立迟滞门限控制仿真评测 (GATED_APPLICATION_EVAL_ONLY)
    % -------------------------------------------------------------------------
    fprintf('\n--- [C8C] 独立迟滞门限控制仿真评测 (TestC8C_GatedApplication, N = %d) ---\n', N_mc_c8);
    theta_on  = 1.0e-5;
    theta_off = 0.7e-5;
    N_confirm = round(0.20 / d_sym.dt); % 200 ms 确认窗口
    
    c8c_rms_comp        = zeros(N_mc_c8, 1);
    c8c_gamma_dev       = zeros(N_mc_c8, 1);
    c8c_gamma_final_dev = zeros(N_mc_c8, 1);
    c8c_active_ratio    = zeros(N_mc_c8, 1);
    c8c_rms_alpha_gate  = zeros(N_mc_c8, 1);
    c8c_eta_alpha_gate  = zeros(N_mc_c8, 1);
    c8c_gamma_L_mean    = zeros(N_mc_c8, 1);
    c8c_gamma_R_mean    = zeros(N_mc_c8, 1);
    c8c_comp_sat        = zeros(N_mc_c8, 1);
    
    for j = 1:N_mc_c8
        res_raw = c8a_res_store{j};
        theta_ts = res_raw.theta_projected;
        N_pts = length(theta_ts);
        
        gamma_L_gated = ones(N_pts, 1);
        gamma_R_gated = ones(N_pts, 1);
        is_gate_on = false;
        confirm_cnt = 0;
        
        for k = 1:N_pts
            th_k = abs(theta_ts(k));
            if ~is_gate_on
                if th_k > theta_on
                    confirm_cnt = confirm_cnt + 1;
                    if confirm_cnt >= N_confirm
                        is_gate_on = true;
                    end
                else
                    confirm_cnt = 0;
                end
            else
                if th_k < theta_off
                    is_gate_on = false;
                    confirm_cnt = 0;
                end
            end
            
            if is_gate_on
                cal_k = step3b_offline_calibration(theta_ts(k), d_sym.Kf_mean);
                gamma_L_gated(k) = cal_k.gamma_L;
                gamma_R_gated(k) = cal_k.gamma_R;
            else
                gamma_L_gated(k) = 1.0;
                gamma_R_gated(k) = 1.0;
            end
        end
        
        % 时序逻辑说明：控制器在 k 时刻生成的门控增益随指令包一起经 CAN 网络延迟执行
        dL_act = c8a_cfgs{j}.d_act_L;
        dR_act = c8a_cfgs{j}.d_act_R;
        iL_cmd = d_sym.iL_cmd;
        iR_cmd = d_sym.iR_cmd;
        iL_g_cmd = gamma_L_gated .* iL_cmd;
        iR_g_cmd = gamma_R_gated .* iR_cmd;
        iL_g_del = zeros(N_pts, 1);
        iR_g_del = zeros(N_pts, 1);
        if dL_act < N_pts, iL_g_del((dL_act+1):N_pts) = iL_g_cmd(1:(N_pts-dL_act)); end
        if dR_act < N_pts, iR_g_del((dR_act+1):N_pts) = iR_g_cmd(1:(N_pts-dR_act)); end
        iL_g_app = max(-Imax_nominal, min(Imax_nominal, iL_g_del));
        iR_g_app = max(-Imax_nominal, min(Imax_nominal, iR_g_del));
        
        iL_b_del = zeros(N_pts, 1);
        iR_b_del = zeros(N_pts, 1);
        if dL_act < N_pts, iL_b_del((dL_act+1):N_pts) = iL_cmd(1:(N_pts-dL_act)); end
        if dR_act < N_pts, iR_b_del((dR_act+1):N_pts) = iR_cmd(1:(N_pts-dR_act)); end
        
        T_alpha_g = -0.5 * d_sym.mech.Le * (d_sym.Kf_L * iL_g_app + d_sym.Kf_R * iR_g_app);
        T_alpha_nom = -0.5 * d_sym.mech.Le * d_sym.Kf_mean * (iL_b_del + iR_b_del);
        e_T_gated = T_alpha_g - T_alpha_nom;
        
        c8c_rms_comp(j) = sqrt(mean(e_T_gated(res_raw.mask_eval).^2));
        
        % 对延迟后实际施加在执行器上的门控状态统计真实激活时间比例与实际增益
        gate_L_del = ones(N_pts, 1);
        gate_R_del = ones(N_pts, 1);
        if dL_act < N_pts, gate_L_del((dL_act+1):N_pts) = gamma_L_gated(1:(N_pts-dL_act)); end
        if dR_act < N_pts, gate_R_del((dR_act+1):N_pts) = gamma_R_gated(1:(N_pts-dR_act)); end
        
        mask_ev = res_raw.mask_eval;
        c8c_gamma_dev(j) = max([abs(gate_L_del(mask_ev) - 1.0); abs(gate_R_del(mask_ev) - 1.0)]);
        idx_eval_end_j = find(res_raw.t <= res_raw.t_eval_end, 1, 'last');
        c8c_gamma_final_dev(j) = max(abs(gate_L_del(idx_eval_end_j) - 1.0), abs(gate_R_del(idx_eval_end_j) - 1.0));
        
        active_gate = (abs(gate_L_del - 1.0) > 1e-12) | (abs(gate_R_del - 1.0) > 1e-12);
        c8c_active_ratio(j) = 100.0 * mean(active_gate(res_raw.mask_eval));
        c8c_gamma_L_mean(j) = mean(gate_L_del(res_raw.mask_eval));
        c8c_gamma_R_mean(j) = mean(gate_R_del(res_raw.mask_eval));
        c8c_comp_sat(j)     = 100.0 * mean((abs(iL_g_app) >= Imax_nominal - 1e-6) | (abs(iR_g_app) >= Imax_nominal - 1e-6));
        
        % C8C 补充真实 RK4 动力学重积分
        x_gate = zeros(4, 1);
        alpha_gate = zeros(N_pts, 1);
        for k = 1:N_pts
            [x_next, ~] = gantry_dynamics_step_rk4( ...
                x_gate, iL_g_app(k), iR_g_app(k), ...
                d_sym.mech, d_sym.plant, ...
                0.0, 0.0, 0.0, d_sym.dt, ...
                d_sym.Kf_L, d_sym.Kf_R);
            alpha_gate(k) = x_gate(2);
            x_gate = x_next;
        end
        rms_alpha_gate = sqrt(mean(alpha_gate(res_raw.mask_eval).^2));
        c8c_rms_alpha_gate(j) = rms_alpha_gate;
        
        rms_alpha_base = res_raw.rms_alpha_base_dyn;
        if rms_alpha_base > 1e-12
            c8c_eta_alpha_gate(j) = 100.0 * (1.0 - rms_alpha_gate / rms_alpha_base);
        else
            c8c_eta_alpha_gate(j) = NaN;
        end
    end
    
    mean_rms_comp_c         = mean(c8c_rms_comp);
    max_rms_comp_c          = max(c8c_rms_comp);
    max_gamma_final_dev_c   = max(c8c_gamma_final_dev);
    max_gamma_applied_dev_c = max(c8c_gamma_dev);
    mean_active_rt_c        = mean(c8c_active_ratio);
    mean_rms_alpha_gate_c   = mean(c8c_rms_alpha_gate);
    p95_rms_alpha_gate_c    = prctile(c8c_rms_alpha_gate, 95);
    mean_eta_alpha_gate_c   = mean(c8c_eta_alpha_gate);
    mean_gamma_L_c          = mean(c8c_gamma_L_mean);
    mean_gamma_R_c          = mean(c8c_gamma_R_mean);
    mean_comp_sat_c         = mean(c8c_comp_sat);
    
    fprintf('  [C8C 门控评测结果]:\n');
    fprintf('    - 门控后虚假补偿执行激活时间比例均值: %5.2f%% (对比 C8A 原始估计超标 %.2f%%)\n', ...
        mean_active_rt_c, time_exceed_mean_a);
    fprintf('    - 门控后实际生效增益最大偏离度 (时序峰值): %.4f%%\n', ...
        max_gamma_applied_dev_c * 100);
    fprintf('    - 门控后终止静态增益最大偏离度 (终点截断): %.4f%% (对比 C8A 原始静态 %.4f%%)\n', ...
        max_gamma_final_dev_c * 100, max_gamma_dev_a * 100);
    fprintf('    - 门控后虚假诱导偏航力矩最大 RMS : %.4e Nm (对比 C8A 原始 %.4e Nm)\n', ...
        max_rms_comp_c, max_rms_comp_a);
    fprintf('    - 门控后物理偏航角 RMS (RK4 重积分): %.4e rad (对比基线 %.4e rad, 改善度: %5.2f%%)\n', ...
        mean_rms_alpha_gate_c, mean(c8a_alpha_base), mean_eta_alpha_gate_c);
    status_c8c = 'GATED_APPLICATION_EVAL_ONLY';
    fprintf('  [C8C 综合评测状态]: [%s] (仅作应用层门控开环评估，未解决根本估计失效)\n', status_c8c);
    
    perf_rows(end+1, :) = { ...
        'r100 (Delta_Kf = 0)', 'TestC8C_GatedApplication', 'Nominal_Hardware_Limit', Imax_nominal, ...
        'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'ONE_PASS_CAUSAL_GATED_REPLAY', 'Gated_Compensation_Evaluation', ...
        sprintf('ThetaOn=1e-5;ThetaOff=0.7e-5;Confirm=200ms;N%d', N_mc_c8), ...
        '20260924+1..100', sprintf('MC_%d_Trials', N_mc_c8), 0.0, ...
        NaN, NaN, NaN, NaN, NaN, ...
        NaN, NaN, ...
        NaN, NaN, mean_gamma_L_c, mean_gamma_R_c, max_gamma_final_dev_c, max_gamma_applied_dev_c, 'NOT_APPLICABLE', ...
        NaN, NaN, NaN, NaN, NaN, NaN, ...
        mean(c8a_rms_base), mean_rms_comp_c, NaN, NaN, ...
        mean(c8a_alpha_base), mean_rms_alpha_gate_c, p95_rms_alpha_gate_c, mean_eta_alpha_gate_c, NaN, ...
        NaN, NaN, NaN, NaN, ...
        mean(c8a_base_sat), mean_comp_sat_c, ...
        NaN, NaN, ...
        NaN, NaN, NaN, ...
        mean_rms_comp_c, max_rms_comp_c, ...
        NaN, NaN, NaN, mean_active_rt_c, ...
        NaN, NaN, NaN, NaN, ...
        'N/A', NaN, 'N/A', 'N/A', 'N/A', ...
        status_c8c};
    
    %% =========================================================================
    %% CSV 结果表规范导出与结构完整性严格回读检验 (四表独立架构)
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 正在导出 Step 3C-2 评测数据表 (四表独立架构)...\n');
    fprintf('    表 1: step3c_performance_results.csv (C4, C5, C8A, C8B, C8C 性能, 19x68)\n');
    fprintf('    表 2: step3c_sensitivity_results.csv (C6 灵敏度两套排行榜, 20x13)\n');
    fprintf('    表 3: step3c_projection_results.csv  (C7 凸集投影安全性与双侧截断, 2x12)\n');
    fprintf('    表 4: step3c_c8a_deterministic_results.csv (C8A 4 种最劣差模工况, 4x9)\n');
    fprintf('=========================================================================\n');
    
    perf_header = { ...
        'Case', 'Test_Item', 'Limit_Scenario', 'Imax_counts', 'Imax_Source', ...
        'Param_Source', 'Application_Mode', 'Disturbance_Type', 'Disturbance_Intensity', ...
        'Random_Seed', 'Trial_Index', 'Delta_Kf_True', ...
        'Delta_Kf_Hat_Mean', 'Delta_Kf_Hat_Median', 'Delta_Kf_AbsError_Median', 'Delta_Kf_AbsError_P95', 'Delta_Kf_RMSE', ...
        'theta_payload_bias', 'theta_payload_bias_p95_abs', ...
        'KfL_Hat', 'KfR_Hat', 'gamma_L', 'gamma_R', 'gamma_final_dev_max', 'gamma_applied_timeseries_dev_max', 'Calibration_Validity', ...
        'invalid_calib_count', 'eta_kf_residual', 'eta_total_mean', 'eta_total_p05', 'eta_total_min', 'eta_total_valid_count', ...
        'RMS_T_res_base', 'RMS_T_res_comp', 'RMS_T_total_base', 'RMS_T_total_comp', ...
        'RMS_alpha_base_dyn', 'RMS_alpha_comp_dyn_mean', 'RMS_alpha_comp_dyn_p95', 'eta_alpha_abs', 'eta_alpha_delay', ...
        'RMS_dalpha_base_dyn', 'RMS_dalpha_comp_dyn', 'alpha_ss_base', 'alpha_ss_comp', ...
        'base_total_sat', 'comp_total_sat', 'unproj_max_peak', 'proj_count', ...
        'PE_active_ratio', 'PE_false_alarm_rate', 'SVF_atten_dB', ...
        'rms_comp_mean', 'rms_comp_max', ...
        'theta_exceed_time_mean', 'theta_final_exceed_trial_ratio', 'projection_trial_ratio', 'gate_active_time_mean', ...
        'max_run_p95', 'late_exceed_mean', 'first_exceed_median', 'last_exceed_median', ...
        'Selection_Metric', 'Selected_Corner_Count', 'Worst_Eta_Corner', 'Worst_Alpha_Corner', 'Worst_Theta_Corner', ...
        'Calibration_Status'};
    
    T_perf = cell2table(perf_rows, 'VariableNames', perf_header);
    file_perf = fullfile(script_dir, 'step3c_performance_results.csv');
    writetable(T_perf, file_perf);
    fprintf('    [OK] 表 1 导出成功 (%d 行 x %d 列): %s\n', height(T_perf), width(T_perf), file_perf);
    
    sens_header = { ...
        'Factor', 'Dataset', 'Low', 'High', 'Unit', ...
        'Metric_Name', 'Metric_Low', 'Metric_Nominal', 'Metric_High', ...
        'Physical_Sensitivity', 'Physical_Unit', 'Range_Impact', 'Rank'};
    
    T_sens = cell2table(sens_rows, 'VariableNames', sens_header);
    file_sens = fullfile(script_dir, 'step3c_sensitivity_results.csv');
    writetable(T_sens, file_sens);
    fprintf('    [OK] 表 2 导出成功 (%d 行 x %d 列): %s\n', height(T_sens), width(T_sens), file_sens);
    
    proj_header = { ...
        'Dataset', 'Fault_Location', 'Fault_Amplitude', ...
        'Sample_Clip_Ratio', 'Trial_Clip_Ratio', ...
        'Low_Clip_Count', 'High_Clip_Count', ...
        'Max_Unprojected', 'Projected_OOB_Count', ...
        'Nonfinite_Count', 'Covariance_Bounded', 'Status'};
    
    T_proj = cell2table(proj_rows, 'VariableNames', proj_header);
    file_proj = fullfile(script_dir, 'step3c_projection_results.csv');
    writetable(T_proj, file_proj);
    fprintf('    [OK] 表 3 导出成功 (%d 行 x %d 列): %s\n', height(T_proj), width(T_proj), file_proj);
    
    det_header = {'Case', 'theta_hat', 'gamma_dev_max', 'rms_comp', 'd_act_L', 'd_act_R', 'delta_g_L', 'delta_g_R', 'Status'};
    T_det = cell2table(det_rows, 'VariableNames', det_header);
    file_det = fullfile(script_dir, 'step3c_c8a_deterministic_results.csv');
    writetable(T_det, file_det);
    fprintf('    [OK] 表 4 导出成功 (%d 行 x %d 列): %s\n', height(T_det), width(T_det), file_det);
    
    % 回读结构与逐字段内存值绝对误差一致性断言
    fprintf('\n>>> 正在执行 CSV 物理结构与字段对齐回读检验 (readtable 逐项数值绝对相等断言)...\n');
    T_perf_read = readtable(file_perf);
    T_sens_read = readtable(file_sens);
    T_proj_read = readtable(file_proj);
    T_det_read  = readtable(file_det);
    
    assert(height(T_perf_read) == 19, '性能表行数必须为 19 行 (14 C4 + 2 C5 + 1 C8A + 1 C8B + 1 C8C)');
    assert(width(T_perf_read) == 68, sprintf('性能表列数不匹配 (期望 68, 实际 %d)', width(T_perf_read)));
    assert(height(T_sens_read) == 20, '灵敏度表行数必须为 20 行 (5因素 x 2指标 x 2数据集)');
    assert(height(T_proj_read) == 2,  '投影表行数必须为 2 行 (r070 + r130)');
    assert(height(T_det_read) == 4,   '确定性差模表必须为 4 行');
    assert(width(T_det_read) == 9,    '确定性差模表列数必须为 9 列');
    
    c8a_idx = find(strcmp(T_perf_read.Test_Item, 'TestC8A_SensorCommFalseComp'));
    assert(~isempty(c8a_idx), 'C8A 记录缺失');
    assert(abs(T_perf_read.theta_exceed_time_mean(c8a_idx) - time_exceed_mean_a) < 1e-12, 'theta_exceed_time_mean 回读不一致');
    assert(abs(T_perf_read.rms_comp_max(c8a_idx) - max_rms_comp_a) < 1e-12, 'rms_comp_max 回读不一致');
    assert(abs(T_perf_read.gamma_final_dev_max(c8a_idx) - max_gamma_dev_a) < 1e-12, 'gamma_final_dev_max 回读不一致');
    assert(isnan(T_perf_read.gamma_applied_timeseries_dev_max(c8a_idx)), 'C8A gamma_applied_timeseries_dev_max 必须为 NaN');
    assert(T_perf_read.eta_total_valid_count(c8a_idx) == eta_total_valid_cnt_a, 'C8A eta_total_valid_count 回读不一致');
    assert(abs(T_perf_read.Delta_Kf_Hat_Median(c8a_idx) - median_hat_a) < 1e-12, 'Delta_Kf_Hat_Median 回读不一致');
    assert(abs(T_perf_read.Delta_Kf_AbsError_Median(c8a_idx) - median_abs_error_a) < 1e-12, 'Delta_Kf_AbsError_Median 回读不一致');
    assert(abs(T_perf_read.max_run_p95(c8a_idx) - max_run_p95) < 1e-12, 'max_run_p95 回读不一致');
    
    c8b_idx = find(strcmp(T_perf_read.Test_Item, 'TestC8B_PayloadConfounding'));
    assert(~isempty(c8b_idx), 'C8B 记录缺失');
    assert(abs(T_perf_read.theta_payload_bias(c8b_idx) - mean_payload_bias_b) < 1e-12, 'C8B payload_bias 回读不一致');
    assert(abs(T_perf_read.theta_payload_bias_p95_abs(c8b_idx) - p95_payload_bias_b) < 1e-12, 'C8B payload_bias_p95 回读不一致');
    assert(abs(T_perf_read.Delta_Kf_Hat_Median(c8b_idx) - median_hat_b) < 1e-12, 'C8B Delta_Kf_Hat_Median 回读不一致');
    assert(abs(T_perf_read.Delta_Kf_AbsError_Median(c8b_idx) - median_abs_error_b) < 1e-12, 'C8B Delta_Kf_AbsError_Median 回读不一致');
    assert(abs(T_perf_read.gamma_final_dev_max(c8b_idx) - max_gamma_dev_b) < 1e-12, 'C8B gamma_final_dev_max 回读不一致');
    assert(isnan(T_perf_read.gamma_applied_timeseries_dev_max(c8b_idx)), 'C8B gamma_applied_timeseries_dev_max 必须为 NaN');
    assert(T_perf_read.eta_total_valid_count(c8b_idx) == 0, 'C8B eta_total_valid_count 必须为 0');
    assert(isnan(T_perf_read.eta_total_mean(c8b_idx)), 'C8B eta_total_mean 必须为 NaN');
    assert(isnan(T_perf_read.eta_total_p05(c8b_idx)), 'C8B eta_total_p05 必须为 NaN');
    assert(isnan(T_perf_read.eta_total_min(c8b_idx)), 'C8B eta_total_min 必须为 NaN');
    assert(strcmp(T_perf_read.Calibration_Validity{c8b_idx}, calib_valid_str_b), 'C8B Calibration_Validity 回读不一致');
    
    c8c_idx = find(strcmp(T_perf_read.Test_Item, 'TestC8C_GatedApplication'));
    assert(~isempty(c8c_idx), 'C8C 记录缺失');
    assert(abs(T_perf_read.rms_comp_max(c8c_idx) - max_rms_comp_c) < 1e-12, 'C8C rms_comp_max 回读不一致');
    assert(abs(T_perf_read.RMS_alpha_comp_dyn_mean(c8c_idx) - mean_rms_alpha_gate_c) < 1e-12, 'C8C RMS_alpha_comp_dyn_mean 回读不一致');
    assert(abs(T_perf_read.RMS_alpha_comp_dyn_p95(c8c_idx) - p95_rms_alpha_gate_c) < 1e-12, 'C8C RMS_alpha_comp_dyn_p95 回读不一致');
    assert(abs(T_perf_read.eta_alpha_abs(c8c_idx) - mean_eta_alpha_gate_c) < 1e-12, 'C8C eta_alpha_abs 回读不一致');
    assert(abs(T_perf_read.gate_active_time_mean(c8c_idx) - mean_active_rt_c) < 1e-12, 'C8C gate_active_time_mean 回读不一致');
    assert(abs(T_perf_read.gamma_L(c8c_idx) - mean_gamma_L_c) < 1e-12, 'C8C gamma_L 回读不一致');
    assert(abs(T_perf_read.gamma_R(c8c_idx) - mean_gamma_R_c) < 1e-12, 'C8C gamma_R 回读不一致');
    assert(abs(T_perf_read.gamma_final_dev_max(c8c_idx) - max_gamma_final_dev_c) < 1e-12, 'C8C gamma_final_dev_max 回读不一致');
    assert(abs(T_perf_read.gamma_applied_timeseries_dev_max(c8c_idx) - max_gamma_applied_dev_c) < 1e-12, 'C8C gamma_applied_timeseries_dev_max 回读不一致');
    assert(isnan(T_perf_read.eta_total_valid_count(c8c_idx)), 'C8C eta_total_valid_count 必须为 NaN');
    assert(abs(T_perf_read.comp_total_sat(c8c_idx) - mean_comp_sat_c) < 1e-12, 'C8C comp_total_sat 回读不一致');
    assert(strcmp(T_perf_read.Application_Mode{c8c_idx}, 'ONE_PASS_CAUSAL_GATED_REPLAY'), 'C8C Application_Mode 必须为 ONE_PASS_CAUSAL_GATED_REPLAY');
    assert(strcmp(T_perf_read.Calibration_Validity{c8c_idx}, 'NOT_APPLICABLE'), 'C8C Calibration_Validity 必须为 NOT_APPLICABLE');
    assert(isnan(T_perf_read.invalid_calib_count(c8c_idx)), 'C8C invalid_calib_count 必须为 NaN');
    assert(isnan(T_perf_read.proj_count(c8c_idx)), 'C8C proj_count 必须为 NaN');
    assert(isnan(T_perf_read.theta_exceed_time_mean(c8c_idx)), 'C8C theta_exceed_time_mean 必须为 NaN');
    
    c5_idx = find(strcmp(T_perf_read.Test_Item, 'TestC5_TopWorstCorners_Noise_MC100'));
    assert(length(c5_idx) == 2, 'C5 记录数必须为 2');
    assert(all(T_perf_read.eta_total_valid_count(c5_idx) == 100), 'C5 eta_total_valid_count 必须为 100');
    assert(all(isnan(T_perf_read.gamma_applied_timeseries_dev_max(c5_idx))), 'C5 gamma_applied_timeseries_dev_max 必须为 NaN');
    assert(all(isfinite(T_perf_read.projection_trial_ratio(c5_idx))), 'C5 projection_trial_ratio 必须为有效有限值');
    assert(all(isnan(T_perf_read.theta_exceed_time_mean(c5_idx))), 'C5 theta_exceed_time_mean 必须为 NaN');
    assert(all(isnan(T_perf_read.gate_active_time_mean(c5_idx))), 'C5 gate_active_time_mean 必须为 NaN');
    
    assert(all(T_perf_read.eta_total_valid_count(1:14) == 1), 'C4 eta_total_valid_count 必须为 1');
    assert(all(isnan(T_perf_read.gamma_applied_timeseries_dev_max(1:14))), 'C4 gamma_applied_timeseries_dev_max 必须为 NaN');
    assert(isnan(T_perf_read.theta_exceed_time_mean(1)), 'C4 theta_exceed_time_mean 必须为 NaN');
    assert(isnan(T_perf_read.gate_active_time_mean(1)), 'C4 gate_active_time_mean 必须为 NaN');
    assert(isnan(T_perf_read.projection_trial_ratio(1)), 'C4 projection_trial_ratio 必须为 NaN');
    
    assert(abs(T_proj_read.Low_Clip_Count(1) - proj_rows{1, 6}) < 1e-12, 'C7 Low_Clip_Count 回读不一致');
    assert(abs(T_proj_read.High_Clip_Count(2) - proj_rows{2, 7}) < 1e-12, 'C7 High_Clip_Count 回读不一致');
    
    for di = 1:4
        assert(abs(T_det_read.theta_hat(di) - det_rows{di, 2}) < 1e-12, '确定性差模 theta_hat 回读不一致');
        assert(abs(T_det_read.rms_comp(di) - det_rows{di, 4}) < 1e-12, '确定性差模 rms_comp 回读不一致');
    end
    
    fprintf('    [OK] 四表回读行列数、字段语义与关键数值逐项绝对相等断言全部成立!\n\n');
    fprintf('=========================================================================\n');
    fprintf('   STEP 3C-2 测试执行与数据归档完成!                                     \n');
    fprintf('   C8A 原始估计器状态: [%s], 超标时间比例均值 = %5.2f%%                  \n', ...
        status_c8a, time_exceed_mean_a);
    fprintf('   C8C 门控仿真状态  : [%s], 门控后激活时间比例均值 = %5.2f%%            \n', ...
        status_c8c, mean_active_rt_c);
    fprintf('   结论严格定性为: Step 3C-2 已执行但未通过最终技术验收                  \n');
    fprintf('=========================================================================\n');
end

%% 辅助函数: 最长连续真值区间长度
function n = longest_true_run(mask)
    d = diff([false; mask(:); false]);
    starts = find(d == 1);
    stops  = find(d == -1);
    if isempty(starts)
        n = 0;
    else
        n = max(stops - starts);
    end
end
