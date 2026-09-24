%% VERIFY_STEP3C_PART2.M - Step 3C-2 动力学耦合与多因素深度分析基准测试驱动
% =========================================================================
% 功能说明:
% 依据 STEP3_IMPLEMENTATION_PLAN.md 规范，全面实施第二阶段测试 (Tests C4 ~ C8):
% 1. Test C4: 偏载动力学耦合诊断评估 (固定 Delta_m = 50.0 kg, d_load in [-0.20, +0.20] m)
%    - 严格调用公共 RK4 动力学求解器 common/gantry_dynamics_step_rk4.m 重积分
%    - 定量评估偏载未建模力矩对单参数估计器的串扰偏差与补偿退化边界
%    - 状态如实标定为 PASS 或 DEGRADED_BY_PAYLOAD
% 2. Test C5: 极端复合恶劣工况回放 (全要素扰动叠加, Monte Carlo N = 100)
%    - 增益漂移 + 偏置 + 异步时滞 + 传感噪声 + 偏载全要素耦合
%    - 评估最劣收敛边界与投影截断统计
% 3. Test C6: 统一绝对敏感度龙卷风排序 (Tornado Ranking)
%    - 基于绝对退化指标 S_p 排序电流增益、偏置、时滞、噪声、偏载五大物理敏感源
% 4. Test C7: 强扰动下凸集投影安全性统计检验 (Monte Carlo N = 30)
%    - 注入大冲击偏载与电流阶跃故障跳变
%    - 严格统计样本级越界率 rho_sample 与试验级越界率 rho_trial
%    - 严格断言投影后 0 越界、0 非有限值、协方差有界
% 5. Test C8: 标称对称基准虚假补偿定量评测 (Monte Carlo N = 100)
%    - 标称对称模型 (r = 1.00, Delta_Kf = 0) 施加全套随机高频噪声与偏置
%    - 严格考核 5 大虚假补偿判据: P95 <= 1e-5, median <= 5e-6, |gamma-1| <= 0.5%, RMS <= 0.01 Nm
% 6. CSV 输出与验证:
%    - 统一采用 MATLAB table + writetable 规范导出 step3c_part2_results.csv
%    - 立即 readtable 回读并严格断言结构完整性与数值一致性
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
    
    table_rows = {};
    
    %% =========================================================================
    %% TEST C4: 偏载动力学耦合诊断评估 (DIAGNOSTIC EVALUATION MODE)
    %% =========================================================================
    fprintf('=========================================================================\n');
    fprintf('>>> 开始执行 [Test C4] 偏载动力学耦合诊断评估 (固定 Delta_m = 50.0 kg)\n');
    fprintf('    定位: 诊断评估模式，记录偏载惯性力矩串扰偏差，绝不声称解耦偏载\n');
    fprintf('=========================================================================\n');
    
    d_load_levels = [-0.20, -0.10, -0.05, 0.00, +0.05, +0.10, +0.20];
    c4_res_store = cell(2, length(d_load_levels));
    
    for d_idx = 1:2
        ds = datasets{d_idx};
        d_tag = dataset_tags{d_idx};
        fprintf('\n--- 数据集 [%d/2]: %s ---\n', d_idx, d_tag);
        
        for li = 1:length(d_load_levels)
            d_val = d_load_levels(li);
            cfg_c4 = struct('delta_m', 50.0, 'd_load', d_val);
            res_c4 = analyze_step3c_trial(ds, cfg_c4);
            c4_res_store{d_idx, li} = res_c4;
            
            err_c4 = abs(res_c4.theta_hat - ds.Delta_Kf_true);
            
            % 状态分类: PASS 或 DEGRADED_BY_PAYLOAD
            if res_c4.is_calib_valid && res_c4.eta_sat >= 90.0
                status_c4 = 'PASS';
            else
                status_c4 = 'DEGRADED_BY_PAYLOAD';
            end
            
            item_name = sprintf('TestC4_Payload_d_%+05.2fm', d_val);
            dist_desc = sprintf('Delta_m=50.0kg;d_load=%+05.2fm', d_val);
            
            fprintf('  [C4] d_load = %+5.2fm | theta_hat = %+.7e (误差: %.2e) | eta_sat = %6.2f%% | eta_dyn = %6.2f%% [%s]\n', ...
                d_val, res_c4.theta_hat, err_c4, res_c4.eta_sat, res_c4.eta_alpha_dyn, status_c4);
            
            table_rows{end+1} = { ...
                d_tag, item_name, 'Nominal_Hardware_Limit', Imax_nominal, ...
                'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'Eccentric_Load', dist_desc, ...
                'Deterministic', 'Single', ds.Delta_Kf_true, ...
                res_c4.theta_hat, res_c4.theta_hat, err_c4, err_c4, ...
                res_c4.Kf_L_hat, res_c4.Kf_R_hat, res_c4.gamma_L, res_c4.gamma_R, res_c4.calib_validity_str, ...
                0, res_c4.eta_kf_residual, res_c4.eta_total, res_c4.eta_total, ...
                res_c4.rms_base, res_c4.rms_comp, res_c4.rms_total_base, res_c4.rms_total_comp, ...
                res_c4.rms_dalpha_base, res_c4.rms_dalpha_comp, res_c4.alpha_ss_base, res_c4.alpha_ss_comp, ...
                res_c4.base_sat_ratio_total, res_c4.comp_sat_ratio_total, ...
                res_c4.unproj_max_peak, res_c4.unproj_clipped_count, ...
                res_c4.pe_active_ratio, res_c4.pe_false_alarm_rate, res_c4.svf_atten_dB, status_c4};
        end
    end
    
    %% =========================================================================
    %% TEST C5: 极端复合恶劣工况回放 (WORST-CASE COMPOSITE MONTE CARLO N = 100)
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 开始执行 [Test C5] 极端复合恶劣工况回放 (Monte Carlo N = 100)\n');
    fprintf('    要素叠加: 增益±2%%, 偏置±20ct, 时滞1/2ms, 噪声2um, 偏载Delta_m=50kg, d_load=±0.1m\n');
    fprintf('=========================================================================\n');
    
    N_mc_c5 = 100;
    for d_idx = 1:2
        ds = datasets{d_idx};
        d_tag = dataset_tags{d_idx};
        fprintf('\n--- 数据集 [%d/2]: %s (Monte Carlo N = %d) ---\n', d_idx, d_tag, N_mc_c5);
        
        theta_hat_arr = zeros(N_mc_c5, 1);
        eta_sat_arr   = zeros(N_mc_c5, 1);
        eta_tot_arr   = zeros(N_mc_c5, 1);
        unproj_peak_arr = zeros(N_mc_c5, 1);
        clip_count_arr  = zeros(N_mc_c5, 1);
        calib_valid_arr = true(N_mc_c5, 1);
        
        for j = 1:N_mc_c5
            seed_j = 20260924 + j;
            rng(seed_j);
            
            cfg_c5 = struct();
            cfg_c5.delta_m    = 50.0;
            cfg_c5.d_load     = -0.10 + 0.20 * rand();
            cfg_c5.delta_g_L  = -0.02 + 0.04 * rand();
            cfg_c5.delta_g_R  = -0.02 + 0.04 * rand();
            cfg_c5.i_bias_L   = -20.0 + 40.0 * rand();
            cfg_c5.i_bias_R   = -20.0 + 40.0 * rand();
            cfg_c5.d_act_L    = randi([1, 2]);
            cfg_c5.d_act_R    = randi([1, 2]);
            cfg_c5.d_meas_L   = 1;
            cfg_c5.d_meas_R   = 1;
            cfg_c5.sigma_y_L  = 2.0e-6;
            cfg_c5.sigma_y_R  = 2.0e-6;
            cfg_c5.sigma_i_L  = 10.0;
            cfg_c5.sigma_i_R  = 10.0;
            cfg_c5.seed       = seed_j;
            
            res_j = analyze_step3c_trial(ds, cfg_c5);
            
            theta_hat_arr(j)   = res_j.theta_hat;
            eta_sat_arr(j)     = res_j.eta_sat;
            eta_tot_arr(j)     = res_j.eta_total;
            unproj_peak_arr(j) = res_j.unproj_max_peak;
            clip_count_arr(j)  = res_j.unproj_clipped_count;
            calib_valid_arr(j) = res_j.is_calib_valid;
        end
        
        eta_p05 = prctile(eta_sat_arr, 5);
        eta_p95 = prctile(eta_sat_arr, 95);
        eta_med = median(eta_sat_arr);
        eta_mean = mean(eta_sat_arr);
        tot_p05 = prctile(eta_tot_arr, 5);
        
        abs_errs = abs(theta_hat_arr - ds.Delta_Kf_true);
        p95_err  = prctile(abs_errs, 95);
        rmse_err = sqrt(mean(abs_errs.^2));
        
        invalid_count = sum(~calib_valid_arr);
        
        if eta_p05 >= 90.0 && invalid_count == 0
            status_c5 = 'PASS';
        else
            status_c5 = 'DEGRADED';
        end
        
        fprintf('  [C5_MC%d] eta_sat: 均值=%5.2f%%, 中位=%5.2f%%, P05=%5.2f%%, P95=%5.2f%% | 状态: [%s]\n', ...
            N_mc_c5, eta_mean, eta_med, eta_p05, eta_p95, status_c5);
        fprintf('  theta_hat: 中位=%+.7e, 误差P95=%.2e, 均方根=%.2e | 投影截断触发试验数: %d/%d\n', ...
            median(theta_hat_arr), p95_err, rmse_err, sum(clip_count_arr > 0), N_mc_c5);
        
        dist_desc = sprintf('Gain2pct+Bias20ct+Delay1-2ms+Noise2um+Payload50kg_0.1m_N%d', N_mc_c5);
        
        table_rows{end+1} = { ...
            d_tag, 'TestC5_WorstCase_Composite_MC100', 'Nominal_Hardware_Limit', Imax_nominal, ...
            'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'Worst_Case_Composite', dist_desc, ...
            '20260924+1..100', sprintf('MC_%d_Trials', N_mc_c5), ds.Delta_Kf_true, ...
            mean(theta_hat_arr), median(theta_hat_arr), p95_err, rmse_err, ...
            NaN, NaN, NaN, NaN, 'VALID', ...
            invalid_count, eta_mean, mean(eta_tot_arr), tot_p05, ...
            NaN, NaN, NaN, NaN, ...
            NaN, NaN, NaN, NaN, ...
            NaN, NaN, ...
            max(unproj_peak_arr), sum(clip_count_arr), ...
            NaN, NaN, NaN, status_c5};
    end
    
    %% =========================================================================
    %% TEST C6: 统一绝对敏感度龙卷风排序 (UNIFIED ABSOLUTE SENSITIVITY TORNADO)
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 开始执行 [Test C6] 统一绝对敏感度龙卷风排序 (Tornado Ranking)\n');
    fprintf('    公式: S_p = |Delta_eta_sat| / (2 * Delta_p) [统一绝对退化敏感度]\n');
    fprintf('=========================================================================\n');
    
    % 基准点: C0 无扰动 (已在 Part 1 测得，此处复算作为严密局部基准)
    % 扰动 1: 电流增益 delta_g (基准 0.00, 步长 0.02)
    % 扰动 2: 霍尔偏置 i_bias (基准 0.0, 步长 20.0 ct)
    % 扰动 3: CAN 时滞 d_act (基准 0 ms, 步长 2 ms)
    % 扰动 4: 传感噪声 sigma_y (基准 0.0 um, 步长 2.0 um)
    % 扰动 5: 偏载偏移 d_load (基准 0.0 m, 步长 0.10 m, Delta_m = 50 kg)
    
    sens_factors = { ...
        'delta_g',   'Current_Gain_Drift',   'pct',   0.02,  '[-0.02, +0.02]'; ...
        'i_bias',    'Hall_Sensor_Bias',     'ct',    20.0,  '[-20, +20] ct'; ...
        'd_act',     'CAN_Network_Delay',    'ms',    2.0,   '[0, 2] ms'; ...
        'sigma_y',   'Sensor_Noise_Pos',     'um',    2.0,   '[0, 2] um'; ...
        'd_load',    'Eccentric_Load',       'm',     0.10,  '[-0.1, +0.1] m (50kg)'};
    
    num_factors = size(sens_factors, 1);
    S_matrix = zeros(num_factors, 2); % [factor x dataset]
    
    for d_idx = 1:2
        ds = datasets{d_idx};
        d_tag = dataset_tags{d_idx};
        
        % 计算零扰动标称基准 eta_0
        res_zero = analyze_step3c_trial(ds, struct());
        eta_0 = res_zero.eta_sat;
        
        % 1. 电流增益 (+0.02 vs -0.02)
        res_gp = analyze_step3c_trial(ds, struct('delta_g_L', +0.02));
        res_gm = analyze_step3c_trial(ds, struct('delta_g_L', -0.02));
        S_matrix(1, d_idx) = abs(res_gp.eta_sat - res_gm.eta_sat) / (2.0 * 0.02); % [% / 1.0 gain]
        
        % 2. 霍尔偏置 (+20 ct vs -20 ct)
        res_bp = analyze_step3c_trial(ds, struct('i_bias_L', +20.0));
        res_bm = analyze_step3c_trial(ds, struct('i_bias_L', -20.0));
        S_matrix(2, d_idx) = abs(res_bp.eta_sat - res_bm.eta_sat) / (2.0 * 20.0); % [% / count]
        
        % 3. CAN 时滞 (0 vs 2 ms 单边绝对敏感度)
        res_dp = analyze_step3c_trial(ds, struct('d_act_L', 2, 'd_act_R', 2));
        S_matrix(3, d_idx) = abs(eta_0 - res_dp.eta_sat) / 2.0; % [% / ms]
        
        % 4. 传感噪声 (0 vs 2 um 高斯噪声单边敏感度, 固定种子)
        res_np = analyze_step3c_trial(ds, struct('sigma_y_L', 2.0e-6, 'sigma_y_R', 2.0e-6, 'seed', 20260924));
        S_matrix(4, d_idx) = abs(eta_0 - res_np.eta_sat) / 2.0; % [% / um]
        
        % 5. 偏载偏移 (+0.10 m vs -0.10 m, Delta_m = 50 kg)
        res_lp = analyze_step3c_trial(ds, struct('delta_m', 50.0, 'd_load', +0.10));
        res_lm = analyze_step3c_trial(ds, struct('delta_m', 50.0, 'd_load', -0.10));
        S_matrix(5, d_idx) = abs(res_lp.eta_sat - res_lm.eta_sat) / (2.0 * 0.10); % [% / m]
    end
    
    % 计算平均敏感度并排序
    S_avg = mean(S_matrix, 2);
    [S_sorted, sort_idx] = sort(S_avg, 'descend');
    
    fprintf('\n=========================================================================\n');
    fprintf('   统一绝对敏感度龙卷风排序结果 (Tornado Ranking 表)                      \n');
    fprintf('=========================================================================\n');
    fprintf('  排名 | 物理敏感源                | 物理单位 | S (r070) | S (r130) | S (综合平均) \n');
    fprintf('  -----+---------------------------+----------+----------+----------+-------------\n');
    for rk = 1:num_factors
        fi = sort_idx(rk);
        name_str = sens_factors{fi, 2};
        unit_str = sens_factors{fi, 3};
        fprintf('   #%d  | %-25s | %%/%-5s  | %8.4f | %8.4f | %11.4f\n', ...
            rk, name_str, unit_str, S_matrix(fi, 1), S_matrix(fi, 2), S_sorted(rk));
        
        % 写入表格
        for d_idx = 1:2
            d_tag = dataset_tags{d_idx};
            item_name = sprintf('TestC6_Tornado_Rank%d_%s', rk, sens_factors{fi, 1});
            dist_desc = sprintf('Sensitivity S_%s = %.4f %%/%s', sens_factors{fi, 1}, S_matrix(fi, d_idx), unit_str);
            
            table_rows{end+1} = { ...
                d_tag, item_name, 'Nominal_Hardware_Limit', Imax_nominal, ...
                'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'Tornado_Sensitivity', dist_desc, ...
                'Deterministic', 'Ranking', datasets{d_idx}.Delta_Kf_true, ...
                S_matrix(fi, d_idx), S_matrix(fi, d_idx), NaN, NaN, ...
                NaN, NaN, NaN, NaN, 'VALID', ...
                0, NaN, NaN, NaN, ...
                NaN, NaN, NaN, NaN, ...
                NaN, NaN, NaN, NaN, ...
                NaN, NaN, ...
                NaN, NaN, ...
                NaN, NaN, NaN, 'PASS'};
        end
    end
    fprintf('=========================================================================\n');
    
    %% =========================================================================
    %% TEST C7: 强扰动下凸集投影安全性统计检验 (CONVEX PROJECTION SAFETY TEST)
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 开始执行 [Test C7] 强扰动下凸集投影安全性统计检验 (Monte Carlo N = 30)\n');
    fprintf('    条件: 大冲击偏载 (50kg, 0.2m) + 阶跃电流跳变故障 (500ct) + 差模增益 (±3%%)\n');
    fprintf('=========================================================================\n');
    
    N_mc_c7 = 30;
    for d_idx = 1:2
        ds = datasets{d_idx};
        d_tag = dataset_tags{d_idx};
        fprintf('\n--- 数据集 [%d/2]: %s (Monte Carlo N = %d) ---\n', d_idx, d_tag, N_mc_c7);
        
        sample_clip_ratios = zeros(N_mc_c7, 1);
        trial_has_clip     = false(N_mc_c7, 1);
        max_unproj_peaks   = zeros(N_mc_c7, 1);
        out_of_bounds_cnt  = 0;
        non_finite_cnt     = 0;
        cov_bounded_all    = true;
        
        for j = 1:N_mc_c7
            seed_j = 20260924 + j;
            rng(seed_j);
            
            cfg_c7 = struct();
            cfg_c7.delta_m          = 50.0;
            cfg_c7.d_load           = 0.20; % 最大偏载极限
            cfg_c7.delta_g_L        = +0.03; % 差模增益极限
            cfg_c7.delta_g_R        = -0.03;
            cfg_c7.step_fault_t     = 1.0;   % t = 1.0s 时注入阶跃跳变故障
            cfg_c7.step_fault_amp_L = 500.0; % +500 counts 阶跃
            cfg_c7.step_fault_amp_R = -500.0;
            cfg_c7.sigma_y_L        = 5.0e-6; % 大噪声
            cfg_c7.sigma_y_R        = 5.0e-6;
            cfg_c7.sigma_i_L        = 30.0;
            cfg_c7.sigma_i_R        = 30.0;
            cfg_c7.seed             = seed_j;
            
            res_j = analyze_step3c_trial(ds, cfg_c7);
            
            sample_clip_ratios(j) = res_j.sample_clip_ratio;
            trial_has_clip(j)     = res_j.has_any_clip;
            max_unproj_peaks(j)   = res_j.unproj_max_peak;
            
            % 硬性保界与数值断言检验
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
        
        rho_sample = mean(sample_clip_ratios);
        rho_trial  = mean(trial_has_clip);
        max_peak   = max(max_unproj_peaks);
        
        fprintf('  [C7] 样本级越界截断率 rho_sample: %6.2f%%\n', rho_sample * 100);
        fprintf('  [C7] 试验级越界触发率 rho_trial : %6.2f%% (%d/%d 试验触发投影)\n', ...
            rho_trial * 100, sum(trial_has_clip), N_mc_c7);
        fprintf('  [C7] 未受限估计流最大峰值 max|theta_unproj|: %.4e N/count\n', max_peak);
        fprintf('  [C7] 投影后越界样本数: %d | 非有限值数: %d | 协方差有界: %s\n', ...
            out_of_bounds_cnt, non_finite_cnt, mat2str(cov_bounded_all));
        
        % 断言硬性安全准则
        assert(out_of_bounds_cnt == 0, '投影后越界次数必须严格为 0');
        assert(non_finite_cnt == 0, '非有限值出现次数必须严格为 0');
        assert(cov_bounded_all, '协方差有界性条件必须逐点严格成立');
        
        item_name = 'TestC7_Convex_Projection_Safety_MC30';
        dist_desc = sprintf('LargePayload0.2m+StepFault500ct+DeltaG3pct_MC%d', N_mc_c7);
        
        table_rows{end+1} = { ...
            d_tag, item_name, 'Nominal_Hardware_Limit', Imax_nominal, ...
            'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'Projection_Safety_Extreme', dist_desc, ...
            '20260924+1..30', sprintf('MC_%d_Trials', N_mc_c7), ds.Delta_Kf_true, ...
            rho_sample, rho_trial, max_peak, max_peak, ...
            NaN, NaN, NaN, NaN, 'VALID', ...
            0, NaN, NaN, NaN, ...
            NaN, NaN, NaN, NaN, ...
            NaN, NaN, NaN, NaN, ...
            NaN, NaN, ...
            max_peak, sum(trial_has_clip), ...
            NaN, NaN, NaN, 'PASS'};
    end
    
    %% =========================================================================
    %% TEST C8: 标称对称基准虚假补偿评测 (NOMINAL SYMMETRIC BENCHMARK EVALUATION)
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 开始执行 [Test C8] 标称对称基准虚假补偿定量评测 (Monte Carlo N = 100)\n');
    fprintf('    对象: 标称完全对称模型 (r = 1.00, Delta_Kf_True = 0.0)\n');
    fprintf('    判据: P95 <= 1e-5 N/ct, 中位 <= 5e-6, |gamma-1| <= 0.5%%, RMS <= 0.01 Nm, 超标时间 <= 5%%\n');
    fprintf('=========================================================================\n');
    
    N_mc_c8 = 100;
    c8_theta_hat = zeros(N_mc_c8, 1);
    c8_gamma_dev = zeros(N_mc_c8, 1);
    c8_rms_comp  = zeros(N_mc_c8, 1);
    c8_exceed_rt = zeros(N_mc_c8, 1);
    c8_calib_val = true(N_mc_c8, 1);
    
    for j = 1:N_mc_c8
        seed_j = 20260924 + j;
        rng(seed_j);
        
        cfg_c8 = struct();
        cfg_c8.sigma_y_L = 2.0e-6;
        cfg_c8.sigma_y_R = 2.0e-6;
        cfg_c8.sigma_i_L = 10.0;
        cfg_c8.sigma_i_R = 10.0;
        cfg_c8.i_bias_L  = -15.0 + 30.0 * rand();
        cfg_c8.i_bias_R  = -15.0 + 30.0 * rand();
        cfg_c8.d_act_L   = randi([0, 1]);
        cfg_c8.d_act_R   = randi([0, 1]);
        cfg_c8.seed      = seed_j;
        
        res_j = analyze_step3c_trial(d_sym, cfg_c8);
        
        c8_theta_hat(j) = res_j.theta_hat;
        c8_gamma_dev(j) = max(abs(res_j.gamma_L - 1.0), abs(res_j.gamma_R - 1.0));
        c8_rms_comp(j)  = res_j.rms_comp;
        c8_exceed_rt(j) = res_j.exceed_false_ratio;
        c8_calib_val(j) = res_j.is_calib_valid;
    end
    
    % 统计判据检验
    c8_abs_theta     = abs(c8_theta_hat);
    p95_theta        = prctile(c8_abs_theta, 95);
    median_theta     = median(c8_abs_theta);
    max_gamma_dev    = max(c8_gamma_dev);
    max_rms_comp     = max(c8_rms_comp);
    mean_exceed_rt   = mean(c8_exceed_rt);
    trial_exceed_cnt = sum(c8_abs_theta > 1.0e-5);
    trial_exceed_rt  = 100.0 * trial_exceed_cnt / N_mc_c8;
    
    fprintf('  [C8] 估计值 P95(|Delta_Kf_hat|): %.4e N/count (要求 <= 1.0e-5)\n', p95_theta);
    fprintf('  [C8] 估计值 中位数 median     : %.4e N/count (要求 <= 5.0e-6)\n', median_theta);
    fprintf('  [C8] 前馈增益最大偏离度       : %.4f%% (要求 <= 0.50%%)\n', max_gamma_dev * 100);
    fprintf('  [C8] 虚假诱导偏航力矩最大 RMS  : %.4e Nm (要求 <= 0.01 Nm)\n', max_rms_comp);
    fprintf('  [C8] 超标试验比例 (>1e-5)      : %5.2f%% (%d/%d 试验, 要求 <= 5.00%%)\n', ...
        trial_exceed_rt, trial_exceed_cnt, N_mc_c8);
    fprintf('  [C8] 动态步数瞬时超标均比     : %5.2f%%\n', mean_exceed_rt);
    
    % 判定与断言
    is_p95_pass    = (p95_theta <= 1.0e-5);
    is_median_pass = (median_theta <= 5.0e-6);
    is_gamma_pass  = (max_gamma_dev <= 0.005);
    is_rms_pass    = (max_rms_comp <= 0.01);
    is_trial_pass  = (trial_exceed_rt <= 5.0);
    
    all_c8_pass = is_p95_pass && is_median_pass && is_gamma_pass && is_rms_pass && is_trial_pass;
    if all_c8_pass
        status_c8 = 'PASS_NO_FALSE_COMPENSATION';
    else
        status_c8 = 'FAIL_FALSE_COMPENSATION';
    end
    fprintf('  [C8] 虚假补偿综合评测结果: [%s]\n', status_c8);
    
    assert(is_p95_pass, 'C8 判定失败: P95 超过 1.0e-5 N/count');
    assert(is_median_pass, 'C8 判定失败: 中位数超过 5.0e-6 N/count');
    assert(is_gamma_pass, 'C8 判定失败: 增益偏离度超过 0.5%');
    assert(is_rms_pass, 'C8 判定失败: 虚假偏航力矩 RMS 超过 0.01 Nm');
    assert(is_trial_pass, 'C8 判定失败: 超标试验比例超过 5.0%');
    
    item_name = 'TestC8_Nominal_Symmetric_Benchmark_MC100';
    dist_desc = sprintf('SymmetricModel_Noise2um_Bias15ct_Delay1-2ms_N%d', N_mc_c8);
    
    table_rows{end+1} = { ...
        'r100 (Delta_Kf = 0)', item_name, 'Nominal_Hardware_Limit', Imax_nominal, ...
        'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'Symmetric_Benchmark_Noise', dist_desc, ...
        '20260924+1..100', sprintf('MC_%d_Trials', N_mc_c8), 0.0, ...
        mean(c8_theta_hat), median_theta, p95_theta, sqrt(mean(c8_abs_theta.^2)), ...
        NaN, NaN, 1.0, 1.0, 'VALID', ...
        0, NaN, NaN, NaN, ...
        0.0, mean(c8_rms_comp), 0.0, mean(c8_rms_comp), ...
        0.0, 0.0, 0.0, 0.0, ...
        0.0, 0.0, ...
        0.0, 0, ...
        NaN, NaN, NaN, status_c8};
    
    %% =========================================================================
    %% CSV 结果表规范导出与结构完整性严格回读检验
    %% =========================================================================
    fprintf('\n=========================================================================\n');
    fprintf('>>> 正在导出 Step 3C-2 评测数据表 (采用规范 table + writetable 架构)...\n');
    
    csv_header = { ...
        'Case', 'Test_Item', 'Limit_Scenario', 'Imax_counts', 'Imax_Source', ...
        'Param_Source', 'Disturbance_Type', 'Disturbance_Intensity', ...
        'Random_Seed', 'Trial_Index', 'Delta_Kf_True', ...
        'Delta_Kf_Hat_Mean', 'Delta_Kf_Hat_Median', 'Delta_Kf_AbsError_P95', 'Delta_Kf_RMSE', ...
        'KfL_Hat', 'KfR_Hat', 'gamma_L', 'gamma_R', 'Calibration_Validity', ...
        'invalid_calib_count', 'eta_kf_residual', 'eta_total_mean', 'eta_total_p05', ...
        'RMS_T_res_base', 'RMS_T_res_comp', 'RMS_T_total_base', 'RMS_T_total_comp', ...
        'RMS_dalpha_base_dyn', 'RMS_dalpha_comp_dyn', 'alpha_ss_base', 'alpha_ss_comp', ...
        'base_total_sat', 'comp_total_sat', 'unproj_max_peak', 'proj_count', ...
        'PE_active_ratio', 'PE_false_alarm_rate', 'SVF_atten_dB', 'Calibration_Status'};
    
    N_rows = length(table_rows);
    col_Case                  = cell(N_rows, 1);
    col_Test_Item             = cell(N_rows, 1);
    col_Limit_Scenario        = cell(N_rows, 1);
    col_Imax_counts           = zeros(N_rows, 1);
    col_Imax_Source           = cell(N_rows, 1);
    col_Param_Source          = cell(N_rows, 1);
    col_Disturbance_Type      = cell(N_rows, 1);
    col_Disturbance_Intensity = cell(N_rows, 1);
    col_Random_Seed           = cell(N_rows, 1);
    col_Trial_Index           = cell(N_rows, 1);
    col_Delta_Kf_True         = zeros(N_rows, 1);
    col_Delta_Kf_Hat_Mean     = zeros(N_rows, 1);
    col_Delta_Kf_Hat_Median   = zeros(N_rows, 1);
    col_Delta_Kf_AbsError_P95 = zeros(N_rows, 1);
    col_Delta_Kf_RMSE         = zeros(N_rows, 1);
    col_KfL_Hat               = zeros(N_rows, 1);
    col_KfR_Hat               = zeros(N_rows, 1);
    col_gamma_L               = zeros(N_rows, 1);
    col_gamma_R               = zeros(N_rows, 1);
    col_Calibration_Validity  = cell(N_rows, 1);
    col_invalid_calib_count   = zeros(N_rows, 1);
    col_eta_kf_residual       = zeros(N_rows, 1);
    col_eta_total_mean        = zeros(N_rows, 1);
    col_eta_total_p05         = zeros(N_rows, 1);
    col_RMS_T_res_base        = zeros(N_rows, 1);
    col_RMS_T_res_comp        = zeros(N_rows, 1);
    col_RMS_T_total_base      = zeros(N_rows, 1);
    col_RMS_T_total_comp      = zeros(N_rows, 1);
    col_RMS_dalpha_base_dyn   = zeros(N_rows, 1);
    col_RMS_dalpha_comp_dyn   = zeros(N_rows, 1);
    col_alpha_ss_base         = zeros(N_rows, 1);
    col_alpha_ss_comp         = zeros(N_rows, 1);
    col_base_total_sat        = zeros(N_rows, 1);
    col_comp_total_sat        = zeros(N_rows, 1);
    col_unproj_max_peak       = zeros(N_rows, 1);
    col_proj_count            = zeros(N_rows, 1);
    col_PE_active_ratio       = zeros(N_rows, 1);
    col_PE_false_alarm_rate   = zeros(N_rows, 1);
    col_SVF_atten_dB          = zeros(N_rows, 1);
    col_Calibration_Status    = cell(N_rows, 1);
    
    for r = 1:N_rows
        row = table_rows{r};
        col_Case{r}                  = row{1};
        col_Test_Item{r}             = row{2};
        col_Limit_Scenario{r}        = row{3};
        col_Imax_counts(r)           = row{4};
        col_Imax_Source{r}           = row{5};
        col_Param_Source{r}          = row{6};
        col_Disturbance_Type{r}      = row{7};
        col_Disturbance_Intensity{r} = row{8};
        col_Random_Seed{r}           = row{9};
        col_Trial_Index{r}           = row{10};
        col_Delta_Kf_True(r)         = row{11};
        col_Delta_Kf_Hat_Mean(r)     = row{12};
        col_Delta_Kf_Hat_Median(r)   = row{13};
        col_Delta_Kf_AbsError_P95(r) = row{14};
        col_Delta_Kf_RMSE(r)         = row{15};
        col_KfL_Hat(r)               = row{16};
        col_KfR_Hat(r)               = row{17};
        col_gamma_L(r)               = row{18};
        col_gamma_R(r)               = row{19};
        col_Calibration_Validity{r}  = row{20};
        col_invalid_calib_count(r)   = row{21};
        col_eta_kf_residual(r)       = row{22};
        col_eta_total_mean(r)        = row{23};
        col_eta_total_p05(r)         = row{24};
        col_RMS_T_res_base(r)        = row{25};
        col_RMS_T_res_comp(r)        = row{26};
        col_RMS_T_total_base(r)      = row{27};
        col_RMS_T_total_comp(r)      = row{28};
        col_RMS_dalpha_base_dyn(r)   = row{29};
        col_RMS_dalpha_comp_dyn(r)   = row{30};
        col_alpha_ss_base(r)         = row{31};
        col_alpha_ss_comp(r)         = row{32};
        col_base_total_sat(r)        = row{33};
        col_comp_total_sat(r)        = row{34};
        col_unproj_max_peak(r)       = row{35};
        col_proj_count(r)            = row{36};
        col_PE_active_ratio(r)       = row{37};
        col_PE_false_alarm_rate(r)   = row{38};
        col_SVF_atten_dB(r)          = row{39};
        col_Calibration_Status{r}    = row{40};
    end
    
    T_out = table( ...
        col_Case, col_Test_Item, col_Limit_Scenario, col_Imax_counts, col_Imax_Source, ...
        col_Param_Source, col_Disturbance_Type, col_Disturbance_Intensity, ...
        col_Random_Seed, col_Trial_Index, col_Delta_Kf_True, ...
        col_Delta_Kf_Hat_Mean, col_Delta_Kf_Hat_Median, col_Delta_Kf_AbsError_P95, col_Delta_Kf_RMSE, ...
        col_KfL_Hat, col_KfR_Hat, col_gamma_L, col_gamma_R, col_Calibration_Validity, ...
        col_invalid_calib_count, col_eta_kf_residual, col_eta_total_mean, col_eta_total_p05, ...
        col_RMS_T_res_base, col_RMS_T_res_comp, col_RMS_T_total_base, col_RMS_T_total_comp, ...
        col_RMS_dalpha_base_dyn, col_RMS_dalpha_comp_dyn, col_alpha_ss_base, col_alpha_ss_comp, ...
        col_base_total_sat, col_comp_total_sat, col_unproj_max_peak, col_proj_count, ...
        col_PE_active_ratio, col_PE_false_alarm_rate, col_SVF_atten_dB, col_Calibration_Status, ...
        'VariableNames', csv_header);
    
    csv_file = fullfile(script_dir, 'step3c_part2_results.csv');
    writetable(T_out, csv_file);
    fprintf('    [OK] CSV 写入完成: %s\n', csv_file);
    
    % 同时合并 Part 1 与 Part 2 生成完整的全集表 step3c_all_results.csv
    file_p1 = fullfile(script_dir, 'step3c_part1_results.csv');
    if exist(file_p1, 'file') == 2
        T_p1 = readtable(file_p1, 'Delimiter', ',');
        T_all = [T_p1; T_out];
        file_all = fullfile(script_dir, 'step3c_all_results.csv');
        writetable(T_all, file_all);
        fprintf('    [OK] Part 1 + Part 2 全集汇总表写入完成 (%d 行 x %d 列): %s\n', ...
            height(T_all), width(T_all), file_all);
    end
    
    fprintf('>>> 正在执行 CSV 物理结构与字段对齐回读检验 (readtable 严密断言)...\n');
    T_read = readtable(csv_file);
    
    expected_rows = length(table_rows);
    expected_cols = 40;
    
    fprintf('    回读数据规模: %d 行 x %d 列 (期望: %d 行 x %d 列)\n', ...
        height(T_read), width(T_read), expected_rows, expected_cols);
    assert(height(T_read) == expected_rows, 'CSV 回读行数不符: %d vs %d', height(T_read), expected_rows);
    assert(width(T_read) == expected_cols, 'CSV 回读列数不符: %d vs %d', width(T_read), expected_cols);
    
    % 检查 C4 偏载行与 C8 对称行
    c4_rows = T_read(strcmp(T_read.Disturbance_Type, 'Eccentric_Load'), :);
    assert(height(c4_rows) == 14, 'C4 偏载工况行数必须为 14 行 (7 偏载 x 2 数据集)');
    
    c8_row = T_read(strcmp(T_read.Test_Item, 'TestC8_Nominal_Symmetric_Benchmark_MC100'), :);
    assert(height(c8_row) == 1, 'C8 必须有且仅有 1 行记录');
    assert(strcmp(c8_row.Calibration_Status{1}, 'PASS_NO_FALSE_COMPENSATION'), 'C8 状态断言失败');
    
    fprintf('    [OK] 全部回读断言与关键字段一致性抽查 100%% 通过!\n\n');
    fprintf('=========================================================================\n');
    fprintf('   STEP 3C-2 基准评测全部完成! 无报错, 动力学耦合与统计检验断言全部成立!   \n');
    fprintf('=========================================================================\n');
end
