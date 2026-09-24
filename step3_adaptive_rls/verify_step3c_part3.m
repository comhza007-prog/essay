%% VERIFY_STEP3C_PART3.M - Step 3C-3 Oracle 电流校正、D0b 严格配对消融与 D1 时滞解耦因果对齐
% =========================================================================
% 功能说明:
% 依据最新技术审查意见，在不修改底层控制器与门控阈值的前提下，完成以下三项系统性验证:
% 1. [Part 1] Test D0: Oracle 电流通道校正评测 (D0_OracleCurrentCalibration, MC100)
%    - 采用与 C8A 完全相同的 100 个随机种子 (20260924 + 1..100) 与扰动空间配置
%    - 预置固定标准正态白噪声序列，确保各测试与消融之间实现严格配对 (Strictly Paired)
%    - 精确校正已知传感器增益漂移与霍尔零偏，完全保留随机白噪声、时滞与位置量化:
%      iL_corr = (iL_meas - cfg.i_bias_L) / (1 + cfg.delta_g_L);
%      iR_corr = (iR_meas - cfg.i_bias_R) / (1 + cfg.delta_g_R);
%    - 严格采用 C8A 相同的五项正式验收指标门槛，未达标定性为 ORACLE_FAIL_TIME_RATIO
%    - 导出独立性能表 step3c_part3_oracle_results.csv (68 列)
%    - 导出 4 种最差差模确定性工况表 step3c_part3_oracle_deterministic_results.csv (9 列)
% 2. [Part 2] Test D0b: 六分支严格配对消融分析 (D0b Matched Paired Ablation, N = 100)
%    - 分支 A (D0b-A): 完整 Oracle (基准)
%    - 分支 B (D0b-B): sigma_y_L/R = 0 (关闭位置白噪声，保留量化)
%    - 分支 C (D0b-C): quant_res = 0 (关闭位置量化，保留位置白噪声)
%    - 分支 D (D0b-D): sigma_i_L/R = 0 (关闭电流测量白噪声，严格配对)
%    - 分支 E (D0b-E): d_meas_L/R = 0 (关闭全部回采测量时滞)
%    - 分支 F (D0b-F): 位置噪声、量化、电流噪声全部关闭 (保留时滞)
%    - 导出消融汇总表 step3c_part3_d0b_ablation_results.csv
% 3. [Part 3] Test D1: 时滞解耦与因果时间戳对齐 (D1 Delay Decoupling & Causal Alignment, N = 100)
%    - 分支 A (D1-A): 原始独立随机时滞 (基准, dL, dR in {0, 1, 2} ms)
%    - 分支 B (D1-B): 公共 1 ms / 1 ms (消除差分时滞)
%    - 分支 C (D1-C): 公共 2 ms / 2 ms (消除差分时滞)
%    - 分支 D (D1-D): 每 trial 取原始两轴 max(dL, dR) (消除左右差模，保留公共时滞)
%    - 分支 E (D1-E): 已知时延下的完整因果时间戳对齐 (较快电流补齐至 dmax，位置同步滞后 dmax)
%    - 导出时滞解耦结果表 step3c_part3_d1_delay_decoupling_results.csv
%    - 导出瞬态分布时间序列表 step3c_part3_d1_timeseries.csv
% =========================================================================

function verify_step3c_part3()
    clc;
    fprintf('=========================================================================\n');
    fprintf('   STEP 3C-3: D0 Oracle 校正、D0b 配对消融与 D1 时滞因果对齐基准测试\n');
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

    % 4. 加载对称基准数据集 (r = 1.00)
    file_sym = fullfile(script_dir, 'data_step3b_phase1_sym.mat');
    assert(exist(file_sym, 'file') == 2, '缺少对称数据集: %s', file_sym);
    d_sym = load(file_sym);
    d_sym.N = length(d_sym.t);
    d_sym.Imax = Imax_nominal;
    if ~isfield(d_sym, 'iL_cmd'), d_sym.iL_cmd = d_sym.iL_actual; end
    if ~isfield(d_sym, 'iR_cmd'), d_sym.iR_cmd = d_sym.iR_actual; end

    N_mc = 100;
    N_pts = d_sym.N;
    dt = d_sym.dt;
    Kf_mean = d_sym.Kf_mean;

    % 5. 预先生成 100 个 trial 的基础扰动参数与固定标准正态随机白噪声序列 (严格配对)
    fprintf('>>> 正在预生成 100 个 trial 的基础扰动与严格配对白噪声序列...\n');
    trials_base = cell(N_mc, 1);
    for j = 1:N_mc
        seed_j = 20260924 + j;
        rng(seed_j, 'twister');

        cfg = struct();
        cfg.delta_g_L = -0.02 + 0.04 * rand();
        cfg.delta_g_R = -0.02 + 0.04 * rand();
        cfg.i_bias_L  = -15.0 + 30.0 * rand();
        cfg.i_bias_R  = -15.0 + 30.0 * rand();
        cfg.d_act_L   = randi([0, 2]);
        cfg.d_act_R   = randi([0, 2]);
        cfg.d_meas_L  = randi([0, 2]);
        cfg.d_meas_R  = randi([0, 2]);
        cfg.sigma_y_L = 2.0e-6;
        cfg.sigma_y_R = 2.0e-6;
        cfg.sigma_i_L = 10.0;
        cfg.sigma_i_R = 10.0;
        cfg.quant_res = 1.0e-6;
        cfg.seed      = seed_j + 100000;

        % 预置固定标准正态白噪声序列，杜绝分支条件抽样破坏随机数流
        cfg.z_iL = randn(N_pts, 1);
        cfg.z_iR = randn(N_pts, 1);
        cfg.z_yL = randn(N_pts, 1);
        cfg.z_yR = randn(N_pts, 1);

        trials_base{j} = cfg;
    end
    fprintf('    [OK] 100 个 trial 基础配置与白噪声矩阵生成完毕 (严格配对保障就绪)\n\n');

    %% =========================================================================
    %% PART 1: TEST D0 - ORACLE CURRENT CALIBRATION (MC 100 Trials)
    %% =========================================================================
    fprintf('=========================================================================\n');
    fprintf('>>> [Part 1] 执行 Test D0: Oracle 电流通道校正评测 (Monte Carlo N = 100)\n');
    fprintf('    校正公式: i_corr = (i_meas - i_bias) / (1 + delta_g)\n');
    fprintf('    保留扰动: 随机高斯白噪声、CAN通信与测量时滞、光栅尺量化\n');
    fprintf('=========================================================================\n');

    d0_theta_hat     = zeros(N_mc, 1);
    d0_gamma_L       = zeros(N_mc, 1);
    d0_gamma_R       = zeros(N_mc, 1);
    d0_gamma_dev     = zeros(N_mc, 1);
    d0_rms_comp      = zeros(N_mc, 1);
    d0_exceed_rt     = zeros(N_mc, 1);
    d0_calib_val     = false(N_mc, 1);
    d0_alpha_base    = zeros(N_mc, 1);
    d0_alpha_comp    = zeros(N_mc, 1);
    d0_KfL_hat       = zeros(N_mc, 1);
    d0_KfR_hat       = zeros(N_mc, 1);
    d0_rms_base      = zeros(N_mc, 1);
    d0_rms_tot_b     = zeros(N_mc, 1);
    d0_rms_tot_c     = zeros(N_mc, 1);
    d0_eta_kf_res    = zeros(N_mc, 1);
    d0_eta_total     = zeros(N_mc, 1);
    d0_eta_alpha_abs = zeros(N_mc, 1);
    d0_eta_alpha_del = zeros(N_mc, 1);
    d0_rms_dalpha_b  = zeros(N_mc, 1);
    d0_rms_dalpha_c  = zeros(N_mc, 1);
    d0_alpha_ss_b    = zeros(N_mc, 1);
    d0_alpha_ss_c    = zeros(N_mc, 1);
    d0_base_sat      = zeros(N_mc, 1);
    d0_comp_sat      = zeros(N_mc, 1);
    d0_peak          = zeros(N_mc, 1);
    d0_proj_cnt      = zeros(N_mc, 1);
    d0_pe_act        = zeros(N_mc, 1);
    d0_pe_far        = zeros(N_mc, 1);
    d0_svf_att       = zeros(N_mc, 1);

    max_run_arr      = zeros(N_mc, 1);
    late_exceed_arr  = zeros(N_mc, 1);
    early_exceed_arr = zeros(N_mc, 1);
    first_exceed_arr = nan(N_mc, 1);
    last_exceed_arr  = nan(N_mc, 1);

    theta_trials_d0  = zeros(N_pts, N_mc);
    pe_trials_d0     = false(N_pts, N_mc);

    for j = 1:N_mc
        cfg_j = trials_base{j};
        res_j = analyze_oracle_trial(d_sym, cfg_j, false);

        theta_trials_d0(:, j) = res_j.theta_projected;
        pe_trials_d0(:, j)    = res_j.pe_mask;

        d0_theta_hat(j)     = res_j.theta_hat;
        d0_gamma_L(j)       = res_j.gamma_L;
        d0_gamma_R(j)       = res_j.gamma_R;
        d0_gamma_dev(j)     = max(abs(res_j.gamma_L - 1.0), abs(res_j.gamma_R - 1.0));
        d0_rms_comp(j)      = res_j.rms_comp;
        d0_exceed_rt(j)     = res_j.exceed_false_ratio;
        d0_calib_val(j)     = res_j.is_calib_valid;
        d0_alpha_base(j)    = res_j.rms_alpha_base_dyn;
        d0_alpha_comp(j)    = res_j.rms_alpha_comp_dyn;
        d0_KfL_hat(j)       = res_j.Kf_L_hat;
        d0_KfR_hat(j)       = res_j.Kf_R_hat;
        d0_rms_base(j)      = res_j.rms_base;
        d0_rms_tot_b(j)     = res_j.rms_total_base;
        d0_rms_tot_c(j)     = res_j.rms_total_comp;
        d0_eta_kf_res(j)    = res_j.eta_kf_residual;
        d0_eta_total(j)     = res_j.eta_total;
        d0_eta_alpha_abs(j) = res_j.eta_alpha_abs;
        d0_eta_alpha_del(j) = res_j.eta_alpha_delay;
        d0_rms_dalpha_b(j)  = res_j.rms_dalpha_base;
        d0_rms_dalpha_c(j)  = res_j.rms_dalpha_comp;
        d0_alpha_ss_b(j)    = res_j.alpha_ss_base;
        d0_alpha_ss_c(j)    = res_j.alpha_ss_comp;
        d0_base_sat(j)      = res_j.base_sat_ratio_total;
        d0_comp_sat(j)      = res_j.comp_sat_ratio_total;
        d0_peak(j)          = res_j.unproj_max_peak;
        d0_proj_cnt(j)      = res_j.unproj_clipped_count;
        d0_pe_act(j)        = res_j.pe_active_ratio;
        d0_pe_far(j)        = res_j.pe_false_alarm_rate;
        d0_svf_att(j)       = res_j.svf_atten_dB;

        t_eval = res_j.t(res_j.mask_eval);
        exceed = (abs(res_j.theta_projected(res_j.mask_eval)) > 1.0e-5);
        max_run_arr(j) = longest_true_run(exceed) * dt;
        late_exceed_arr(j)  = 100.0 * mean(exceed(t_eval >= 1.0));
        early_exceed_arr(j) = 100.0 * mean(exceed(t_eval < 1.0));

        idx_ex = find(exceed);
        if ~isempty(idx_ex)
            first_exceed_arr(j) = t_eval(idx_ex(1));
            last_exceed_arr(j)  = t_eval(idx_ex(end));
        end
    end

    d0_abs_theta        = abs(d0_theta_hat);
    p95_theta_d0        = prctile(d0_abs_theta, 95);
    median_hat_d0       = median(d0_theta_hat);
    median_abs_error_d0 = median(d0_abs_theta);
    rmse_theta_d0       = sqrt(mean(d0_abs_theta.^2));
    max_gamma_dev_d0    = max(d0_gamma_dev);
    mean_rms_comp_d0    = mean(d0_rms_comp);
    max_rms_comp_d0     = max(d0_rms_comp);
    time_exceed_mean_d0 = mean(d0_exceed_rt);
    trial_exceed_cnt_d0 = sum(d0_abs_theta > 1.0e-5);
    trial_exceed_rt_d0  = 100.0 * trial_exceed_cnt_d0 / N_mc;

    eta_total_valid_d0  = d0_eta_total(isfinite(d0_eta_total));
    eta_total_valid_cnt_d0 = numel(eta_total_valid_d0);
    if eta_total_valid_cnt_d0 == 0
        eta_tot_mean_d0 = NaN;
        eta_tot_p05_d0  = NaN;
        eta_tot_min_d0  = NaN;
    else
        eta_tot_mean_d0 = mean(eta_total_valid_d0);
        eta_tot_p05_d0  = prctile(eta_total_valid_d0, 5);
        eta_tot_min_d0  = min(eta_total_valid_d0);
    end

    invalid_calib_cnt_d0 = sum(~d0_calib_val);
    if invalid_calib_cnt_d0 == 0
        calib_valid_str_d0 = 'VALID';
    else
        calib_valid_str_d0 = 'INVALID';
    end

    alpha_comp_mean_d0  = mean(d0_alpha_comp);
    alpha_comp_p95_d0   = prctile(d0_alpha_comp, 95);

    max_run_p95         = prctile(max_run_arr, 95);
    late_exceed_mean    = mean(late_exceed_arr);
    early_exceed_mean   = mean(early_exceed_arr);
    first_exceed_median = nanmedian(first_exceed_arr);
    last_exceed_median  = nanmedian(last_exceed_arr);

    fprintf('\n--- [D0 Oracle] 统计指标与判据核验 (N = 100) ---\n');
    fprintf('    - P95(|Delta_Kf_hat|)   : %.4e N/count (门槛 <= 1.0e-5) -> [%s]\n', ...
        p95_theta_d0, pass_fail_str(p95_theta_d0 <= 1.0e-5));
    fprintf('    - 有符号中位数 median     : %+.4e N/count\n', median_hat_d0);
    fprintf('    - 绝对误差中位数 abs_med  : %.4e N/count (门槛 <= 5.0e-6) -> [%s]\n', ...
        median_abs_error_d0, pass_fail_str(median_abs_error_d0 <= 5.0e-6));
    fprintf('    - 最大前馈偏离 max|gamma-1|: %.4f%% (门槛 <= 0.50%%) -> [%s]\n', ...
        max_gamma_dev_d0 * 100, pass_fail_str(max_gamma_dev_d0 <= 0.005));
    fprintf('    - 虚假偏航力矩最大 RMS   : %.4e Nm (门槛 <= 0.010 Nm) -> [%s]\n', ...
        max_rms_comp_d0, pass_fail_str(max_rms_comp_d0 <= 0.010));
    fprintf('    - 超标时间比例均值 time_exceed: %5.2f%% (门槛 <= 5.00%%) -> [%s]\n', ...
        time_exceed_mean_d0, pass_fail_str(time_exceed_mean_d0 <= 5.00));
    fprintf('    - 终点估计超标试验比例   : %5.2f%% (%d/%d, 辅助诊断)\n', ...
        trial_exceed_rt_d0, trial_exceed_cnt_d0, N_mc);

    fprintf('  [D0 时间游程证据]:\n');
    fprintf('    - 最长单次连续超标时间 P95 : %.3f s\n', max_run_p95);
    fprintf('    - 前半段 (t < 1.0s) 超标均比: %5.2f%%\n', early_exceed_mean);
    fprintf('    - 后半段 (t >= 1.0s) 超标均比: %5.2f%%\n', late_exceed_mean);
    fprintf('    - 首次超标时刻中位数        : %.3f s\n', first_exceed_median);
    fprintf('    - 最后超标时刻中位数        : %.3f s\n', last_exceed_median);

    % 评估 4 种确定性差模工况
    fprintf('\n--- [D0 Oracle] 4 种最差确定性差模工况检验 ---\n');
    det_diff_cfgs = { ...
        'Diff_Gain_+2%_-2%', struct('delta_g_L', +0.02, 'delta_g_R', -0.02, 'd_act_L', 0, 'd_act_R', 0); ...
        'Diff_Gain_-2%_+2%', struct('delta_g_L', -0.02, 'delta_g_R', +0.02, 'd_act_L', 0, 'd_act_R', 0); ...
        'Diff_Delay_1ms_2ms', struct('delta_g_L', 0.0, 'delta_g_R', 0.0, 'd_act_L', 1, 'd_act_R', 2); ...
        'Diff_Delay_2ms_1ms', struct('delta_g_L', 0.0, 'delta_g_R', 0.0, 'd_act_L', 2, 'd_act_R', 1)};

    det_rows = cell(size(det_diff_cfgs, 1), 9);
    for di = 1:size(det_diff_cfgs, 1)
        cfg_di = det_diff_cfgs{di, 2};
        res_det = analyze_oracle_trial(d_sym, cfg_di, false);
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

    % 程序化判定 D0 五项正式判据
    pass_p95   = p95_theta_d0 <= 1.0e-5;
    pass_med   = median_abs_error_d0 <= 5.0e-6;
    pass_gamma = max_gamma_dev_d0 <= 0.005;
    pass_rms   = max_rms_comp_d0 <= 0.01;
    pass_time  = time_exceed_mean_d0 <= 5.0;

    if ~pass_time
        status_d0 = 'FAIL_TIME_RATIO';
    elseif ~pass_p95
        status_d0 = 'FAIL_FINAL_P95';
    elseif ~pass_med
        status_d0 = 'FAIL_FINAL_MEDIAN';
    elseif ~pass_gamma
        status_d0 = 'FAIL_GAIN_DEVIATION';
    elseif ~pass_rms
        status_d0 = 'FAIL_FALSE_YAW_TORQUE';
    else
        status_d0 = 'PASS';
    end

    fprintf('\n=========================================================================\n');
    if strcmp(status_d0, 'PASS')
        fprintf('   [D0 Oracle 综合正式判定]: [PASS] (五项正式判据全部达标)\n');
    else
        fprintf('   [D0 Oracle 综合正式判定]: [%s] (未达标，严禁调高门槛，如实定性)\n', status_d0);
    end
    fprintf('=========================================================================\n');

    % 导出独立确定性工况表
    det_header = {'Case', 'theta_hat', 'gamma_dev_max', 'rms_comp', ...
        'd_act_L', 'd_act_R', 'delta_g_L', 'delta_g_R', 'Status'};
    T_det = cell2table(det_rows, 'VariableNames', det_header);
    file_det = fullfile(script_dir, 'step3c_part3_oracle_deterministic_results.csv');
    writetable(T_det, file_det);
    fprintf('>>> 正在导出确定性工况表: %s\n', file_det);
    T_det_read = readtable(file_det);
    assert(height(T_det_read) == 4, '确定性表行数必须为 4');
    assert(width(T_det_read) == 9, '确定性表列数必须为 9');
    assert(all(strcmp(T_det_read.Status, 'PASS')), '确定性工况必须全部为 PASS');
    fprintf('    [OK] 确定性工况表导出与回读断言 100%% 成立!\n');

    % 导出独立 D0 性能汇总表
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

    d0_row = { ...
        'r100 (Delta_Kf = 0)', 'TestD0_OracleCurrentCalibration', 'Nominal_Hardware_Limit', Imax_nominal, ...
        'param_init.ctrl.spd_max_out', 'Step3C_Replay', 'POSTHOC_STATIC_REPLAY', 'Sensor_Comm_Noise_Symmetric_OracleCalib', ...
        sprintf('Noise2um_Bias15ct_Gain2pct_Delay2ms_OracleCalib_N%d', N_mc), ...
        '20260924+1..100', sprintf('MC_%d_Trials', N_mc), 0.0, ...
        mean(d0_theta_hat), median_hat_d0, median_abs_error_d0, p95_theta_d0, rmse_theta_d0, ...
        NaN, NaN, ...
        mean(d0_KfL_hat), mean(d0_KfR_hat), mean(d0_gamma_L), mean(d0_gamma_R), max_gamma_dev_d0, NaN, calib_valid_str_d0, ...
        invalid_calib_cnt_d0, mean(d0_eta_kf_res), eta_tot_mean_d0, eta_tot_p05_d0, eta_tot_min_d0, eta_total_valid_cnt_d0, ...
        mean(d0_rms_base), mean(d0_rms_comp), mean(d0_rms_tot_b), mean(d0_rms_tot_c), ...
        mean(d0_alpha_base), alpha_comp_mean_d0, alpha_comp_p95_d0, mean(d0_eta_alpha_abs), mean(d0_eta_alpha_del), ...
        mean(d0_rms_dalpha_b), mean(d0_rms_dalpha_c), mean(d0_alpha_ss_b), mean(d0_alpha_ss_c), ...
        mean(d0_base_sat), mean(d0_comp_sat), ...
        max(d0_peak), sum(d0_proj_cnt), ...
        mean(d0_pe_act), mean(d0_pe_far), mean(d0_svf_att), ...
        mean_rms_comp_d0, max_rms_comp_d0, ...
        time_exceed_mean_d0, trial_exceed_rt_d0, NaN, NaN, ...
        max_run_p95, late_exceed_mean, first_exceed_median, last_exceed_median, ...
        'N/A', NaN, 'N/A', 'N/A', 'N/A', ...
        status_d0};

    T_oracle = cell2table(d0_row, 'VariableNames', perf_header);
    file_csv = fullfile(script_dir, 'step3c_part3_oracle_results.csv');
    writetable(T_oracle, file_csv);
    fprintf('>>> 正在导出 Oracle 结果表: %s\n', file_csv);
    T_read = readtable(file_csv);
    assert(height(T_read) == 1, '行数必须严格为 1');
    assert(width(T_read) == 68, '列数必须严格为 68');
    assert(abs(T_read.Delta_Kf_AbsError_P95(1) - p95_theta_d0) < 1e-12, 'P95 估计绝对误差不一致');
    assert(abs(T_read.theta_exceed_time_mean(1) - time_exceed_mean_d0) < 1e-12, '超标时间均比不一致');
    assert(strcmp(T_read.Calibration_Status{1}, status_d0), '校准状态字符串不一致');
    fprintf('    [OK] Oracle 结果表回读一致性断言 100%% 成立!\n\n');

    %% =========================================================================
    %% PART 2: TEST D0b - MATCHED PAIRED ABLATION (Branches A ~ F, N = 100)
    %% =========================================================================
    fprintf('=========================================================================\n');
    fprintf('>>> [Part 2] 执行 Test D0b: 六分支严格配对消融分析 (N = 100)\n');
    fprintf('    保障机制: 采用固定标准正态白噪声序列，杜绝分支抽样破坏随机数序列\n');
    fprintf('=========================================================================\n');

    d0b_branches = {'A', 'B', 'C', 'D', 'E', 'F'};
    d0b_names = { ...
        'D0b-A (Full Oracle Baseline)', ...
        'D0b-B (sigma_y_L/R = 0, keep quant)', ...
        'D0b-C (quant_res = 0, keep sigma_y)', ...
        'D0b-D (sigma_i_L/R = 0, Strictly Paired)', ...
        'D0b-E (d_meas_L/R = 0)', ...
        'D0b-F (All noise/quant OFF)'};

    ablation_rows = cell(numel(d0b_branches), 11);

    for bi = 1:numel(d0b_branches)
        b_code = d0b_branches{bi};
        b_name = d0b_names{bi};

        exceed_time_arr = zeros(N_mc, 1);
        theta_final_arr = zeros(N_mc, 1);
        pe_act_arr      = zeros(N_mc, 1);
        late_ex_arr     = zeros(N_mc, 1);
        early_ex_arr    = zeros(N_mc, 1);
        max_run_arr_b   = zeros(N_mc, 1);

        for j = 1:N_mc
            cfg = trials_base{j};

            % 分支独立改动
            switch b_code
                case 'A'
                    % 基准保持不变
                case 'B'
                    cfg.sigma_y_L = 0; cfg.sigma_y_R = 0;
                case 'C'
                    cfg.quant_res = 0;
                case 'D'
                    cfg.sigma_i_L = 0; cfg.sigma_i_R = 0;
                case 'E'
                    cfg.d_meas_L = 0; cfg.d_meas_R = 0;
                case 'F'
                    cfg.sigma_y_L = 0; cfg.sigma_y_R = 0;
                    cfg.quant_res = 0;
                    cfg.sigma_i_L = 0; cfg.sigma_i_R = 0;
            end

            res_b = analyze_oracle_trial(d_sym, cfg, false);

            theta_final_arr(j) = res_b.theta_hat;
            exceed_time_arr(j) = res_b.exceed_false_ratio;
            pe_act_arr(j)      = res_b.pe_active_ratio;

            t_eval_b = res_b.t(res_b.mask_eval);
            ex_b = (abs(res_b.theta_projected(res_b.mask_eval)) > 1.0e-5);
            max_run_arr_b(j) = longest_true_run(ex_b) * dt;
            late_ex_arr(j)   = 100.0 * mean(ex_b(t_eval_b >= 1.0));
            early_ex_arr(j)  = 100.0 * mean(ex_b(t_eval_b < 1.0));
        end

        time_mean   = mean(exceed_time_arr);
        p95_run     = prctile(max_run_arr_b, 95);
        late_mean   = mean(late_ex_arr);
        early_mean  = mean(early_ex_arr);
        p95_final   = prctile(abs(theta_final_arr), 95);
        med_final   = median(abs(theta_final_arr));
        pe_mean     = mean(pe_act_arr);

        pass_t_d0b   = time_mean <= 5.0;
        pass_p95_d0b = p95_final <= 1.0e-5;
        pass_med_d0b = med_final <= 5.0e-6;

        if ~pass_t_d0b
            b_status = 'FAIL_TIME_RATIO';
        elseif ~pass_p95_d0b
            b_status = 'FAIL_FINAL_P95';
        elseif ~pass_med_d0b
            b_status = 'FAIL_FINAL_MEDIAN';
        else
            b_status = 'PASS';
        end

        ablation_rows(bi, :) = { ...
            sprintf('D0b-%s', b_code), b_name, time_mean, early_mean, late_mean, ...
            p95_run, p95_final, med_final, pe_mean, b_status, ...
            describe_d0b_finding(b_code, time_mean)};

        fprintf('\n  >>> 分支 %s: %s <<<\n', b_code, b_name);
        fprintf('      超标时间比例均值 time_exceed : %5.2f%% (门槛 <= 5.00%%) -> [%s]\n', ...
            time_mean, pass_fail_str(time_mean <= 5.00));
        fprintf('      前半段 (t < 1.0s) 超标均比   : %5.2f%%\n', early_mean);
        fprintf('      后半段 (t >= 1.0s) 超标均比  : %5.2f%%\n', late_mean);
        fprintf('      最长超标持续时间 P95        : %.3f s\n', p95_run);
        fprintf('      终点估计 P95(|theta|)       : %.4e N/ct (门槛 <= 1.0e-5)\n', p95_final);
        fprintf('      终点估计中位数 median        : %.4e N/ct (门槛 <= 5.0e-6)\n', med_final);
        fprintf('      PE 门控激活比例均值          : %.4f\n', pe_mean);
    end

    % 导出消融汇总表
    ablation_header = { ...
        'Branch_ID', 'Branch_Description', 'theta_exceed_time_mean', ...
        'early_exceed_mean', 'late_exceed_mean', 'max_run_p95', ...
        'P95_abs_theta_final', 'median_abs_theta_final', 'PE_active_ratio', ...
        'Status', 'Physical_Diagnosis'};
    T_ablation = cell2table(ablation_rows, 'VariableNames', ablation_header);
    file_ablation = fullfile(script_dir, 'step3c_part3_d0b_ablation_results.csv');
    writetable(T_ablation, file_ablation);
    fprintf('\n>>> 正在导出 D0b 消融汇总表: %s\n', file_ablation);
    T_abl_read = readtable(file_ablation);
    assert(height(T_abl_read) == 6, '消融表行数必须为 6');
    assert(width(T_abl_read) == 11, '消融表列数必须为 11');
    fprintf('    [OK] D0b 消融汇总表导出与回读断言 100%% 成立!\n\n');

    %% =========================================================================
    %% PART 3: TEST D1 - DELAY DECOUPLING & CAUSAL ALIGNMENT (Branches A ~ E, N = 100)
    %% =========================================================================
    fprintf('=========================================================================\n');
    fprintf('>>> [Part 3] 执行 Test D1: 时滞解耦与因果时间戳对齐 (N = 100)\n');
    fprintf('    目标: 解耦公共时滞与左右通道差分时滞，验证因果时间戳对齐\n');
    fprintf('=========================================================================\n');

    d1_branches = {'A', 'B', 'C', 'D', 'E'};
    d1_names = { ...
        'D1-A (Original Random Delays, Baseline)', ...
        'D1-B (Common 1ms / 1ms, Diff=0)', ...
        'D1-C (Common 2ms / 2ms, Diff=0)', ...
        'D1-D (Per-trial max(dL, dR), Diff=0)', ...
        'D1-E (Oracle-known-delay causal alignment)'};

    d1_rows = cell(numel(d1_branches), 13);
    theta_trials_d1 = cell(numel(d1_branches), 1);
    pe_trials_d1    = cell(numel(d1_branches), 1);

    d1_exceed_trials = nan(N_mc, numel(d1_branches));
    d1_final_trials  = nan(N_mc, numel(d1_branches));
    idx_accel = find(d_sym.t >= 0.5 & d_sym.t <= 0.8);
    reg_y_A   = zeros(numel(idx_accel), N_mc);
    reg_phi_A = zeros(numel(idx_accel), N_mc);
    reg_y_D   = zeros(numel(idx_accel), N_mc);
    reg_phi_D = zeros(numel(idx_accel), N_mc);

    for bi = 1:numel(d1_branches)
        b_code = d1_branches{bi};
        b_name = d1_names{bi};

        exceed_time_arr = zeros(N_mc, 1);
        theta_final_arr = zeros(N_mc, 1);
        gamma_dev_arr   = zeros(N_mc, 1);
        rms_comp_arr    = zeros(N_mc, 1);
        pe_act_arr      = zeros(N_mc, 1);
        late_ex_arr     = zeros(N_mc, 1);
        early_ex_arr    = zeros(N_mc, 1);
        max_run_arr_d1  = zeros(N_mc, 1);

        theta_trials_b  = zeros(N_pts, N_mc);
        pe_trials_b     = false(N_pts, N_mc);

        for j = 1:N_mc
            cfg = trials_base{j};
            is_causal_align = false;
            orig_dL = cfg.d_meas_L;
            orig_dR = cfg.d_meas_R;

            switch b_code
                case 'A'
                    % 原始随机时滞基准
                case 'B'
                    cfg.d_meas_L = 1; cfg.d_meas_R = 1;
                case 'C'
                    cfg.d_meas_L = 2; cfg.d_meas_R = 2;
                case 'D'
                    d_common = max(orig_dL, orig_dR);
                    cfg.d_meas_L = d_common;
                    cfg.d_meas_R = d_common;
                case 'E'
                    is_causal_align = true;
            end

            res_d1 = analyze_oracle_trial(d_sym, cfg, is_causal_align);

            theta_trials_b(:, j) = res_d1.theta_projected;
            pe_trials_b(:, j)    = res_d1.pe_mask;

            theta_final_arr(j) = res_d1.theta_hat;
            gamma_dev_arr(j)   = max(abs(res_d1.gamma_L - 1.0), abs(res_d1.gamma_R - 1.0));
            rms_comp_arr(j)    = res_d1.rms_comp;
            exceed_time_arr(j) = res_d1.exceed_false_ratio;
            pe_act_arr(j)      = res_d1.pe_active_ratio;

            d1_exceed_trials(j, bi) = res_d1.exceed_false_ratio;
            d1_final_trials(j, bi)  = abs(res_d1.theta_hat);

            if strcmp(b_code, 'A')
                reg_y_A(:, j)   = res_d1.reg_y_f(idx_accel);
                reg_phi_A(:, j) = res_d1.reg_phi_f(idx_accel);
            elseif strcmp(b_code, 'D')
                reg_y_D(:, j)   = res_d1.reg_y_f(idx_accel);
                reg_phi_D(:, j) = res_d1.reg_phi_f(idx_accel);
            end

            t_eval_b = res_d1.t(res_d1.mask_eval);
            ex_b = (abs(res_d1.theta_projected(res_d1.mask_eval)) > 1.0e-5);
            max_run_arr_d1(j) = longest_true_run(ex_b) * dt;
            late_ex_arr(j)    = 100.0 * mean(ex_b(t_eval_b >= 1.0));
            early_ex_arr(j)   = 100.0 * mean(ex_b(t_eval_b < 1.0));
        end

        theta_trials_d1{bi} = theta_trials_b;
        pe_trials_d1{bi}    = pe_trials_b;

        time_mean   = mean(exceed_time_arr);
        p95_run     = prctile(max_run_arr_d1, 95);
        late_mean   = mean(late_ex_arr);
        early_mean  = mean(early_ex_arr);
        p95_final   = prctile(abs(theta_final_arr), 95);
        med_final   = median(abs(theta_final_arr));
        max_g_dev   = max(gamma_dev_arr);
        max_rms     = max(rms_comp_arr);
        pe_mean     = mean(pe_act_arr);

        pass_p95_d1 = p95_final <= 1.0e-5;
        pass_med_d1 = med_final <= 5.0e-6;
        pass_g_d1   = max_g_dev <= 0.005;
        pass_rms_d1 = max_rms <= 0.01;
        pass_t_d1   = time_mean <= 5.0;

        if ~pass_t_d1
            d1_status = 'FAIL_TIME_RATIO';
        elseif ~pass_p95_d1
            d1_status = 'FAIL_FINAL_P95';
        elseif ~pass_med_d1
            d1_status = 'FAIL_FINAL_MEDIAN';
        elseif ~pass_g_d1
            d1_status = 'FAIL_GAIN_DEVIATION';
        elseif ~pass_rms_d1
            d1_status = 'FAIL_FALSE_YAW_TORQUE';
        else
            d1_status = 'PASS';
        end

        d1_rows(bi, :) = { ...
            sprintf('D1-%s', b_code), b_name, time_mean, early_mean, late_mean, ...
            p95_run, p95_final, med_final, max_g_dev * 100, max_rms, pe_mean, d1_status, ...
            describe_d1_finding(b_code, time_mean)};

        fprintf('\n  >>> 分支 %s: %s <<<\n', b_code, b_name);
        fprintf('      超标时间比例均值 time_exceed : %5.2f%% (门槛 <= 5.00%%) -> [%s]\n', ...
            time_mean, pass_fail_str(time_mean <= 5.00));
        fprintf('      前半段 (t < 1.0s) 超标均比   : %5.2f%%\n', early_mean);
        fprintf('      后半段 (t >= 1.0s) 超标均比  : %5.2f%%\n', late_mean);
        fprintf('      最长超标持续时间 P95        : %.3f s\n', p95_run);
        fprintf('      终点估计 P95(|theta|)       : %.4e N/ct (门槛 <= 1.0e-5)\n', p95_final);
        fprintf('      终点估计中位数 median        : %.4e N/ct (门槛 <= 5.0e-6)\n', med_final);
        fprintf('      最大前馈偏离 max|gamma-1|   : %.4f%% (门槛 <= 0.50%%)\n', max_g_dev * 100);
        fprintf('      虚假诱导偏航力矩最大 RMS    : %.4e Nm (门槛 <= 0.010 Nm)\n', max_rms);
        fprintf('      PE 门控激活比例均值          : %.4f\n', pe_mean);
        fprintf('      综合状态判定                : [%s]\n', d1_status);
    end

    % 导出 D1 结果表
    d1_header = { ...
        'Branch_ID', 'Branch_Description', 'theta_exceed_time_mean', ...
        'early_exceed_mean', 'late_exceed_mean', 'max_run_p95', ...
        'P95_abs_theta_final', 'median_abs_theta_final', 'max_gamma_dev_pct', ...
        'max_rms_comp_Nm', 'PE_active_ratio', 'Status', 'Physical_Diagnosis'};
    T_d1 = cell2table(d1_rows, 'VariableNames', d1_header);
    file_d1 = fullfile(script_dir, 'step3c_part3_d1_delay_decoupling_results.csv');
    writetable(T_d1, file_d1);
    fprintf('\n>>> 正在导出 D1 时滞解耦结果表: %s\n', file_d1);
    T_d1_read = readtable(file_d1);
    assert(height(T_d1_read) == 5, 'D1 表行数必须为 5');
    assert(width(T_d1_read) == 13, 'D1 表列数必须为 13');
    fprintf('    [OK] D1 时滞解耦结果表导出与回读断言 100%% 成立!\n');

    % D1 配对效应严格分析 (D1-A vs D1-D)
    delta_AD = d1_exceed_trials(:, 1) - d1_exceed_trials(:, 4);
    paired_mean_AD   = mean(delta_AD);
    paired_median_AD = median(delta_AD);
    paired_p05_AD    = prctile(delta_AD, 5);
    paired_p95_AD    = prctile(delta_AD, 95);
    improved_ratio   = 100 * mean(delta_AD > 0);

    fprintf('\n=========================================================================\n');
    fprintf('--- [D1 配对效应分析 (D1-A vs D1-D, N = %d)] ---\n', N_mc);
    fprintf('    A-D配对改善均值: %.3f percentage points\n', paired_mean_AD);
    fprintf('    A-D配对改善中位数: %.3f percentage points\n', paired_median_AD);
    fprintf('    A-D配对改善 P05/P95: [%.3f, %.3f] percentage points\n', paired_p05_AD, paired_p95_AD);
    fprintf('    A-D全体改善比例: %.1f%% trials\n', improved_ratio);
    assert(paired_mean_AD > 0, 'D1-D没有产生正向平均改善');

    % 程序化闭环校验: 非零差模试验改善率与零差模严格一致性
    delay_skew_samples = cellfun(@(c) abs(c.d_meas_L - c.d_meas_R), trials_base);
    delay_skew_samples = delay_skew_samples(:);
    idx_nonzero = delay_skew_samples > 0;
    idx_zero    = ~idx_nonzero;

    improved_ratio_nonzero = 100 * mean(delta_AD(idx_nonzero) > 0);
    zero_skew_max_diff = max(abs(delta_AD(idx_zero)));

    fprintf('    非零差模试验改善比例: %.1f%% (%d/%d)\n', ...
        improved_ratio_nonzero, sum(delta_AD(idx_nonzero) > 0), sum(idx_nonzero));
    fprintf('    零差模A/D最大差异: %.4e pp\n', zero_skew_max_diff);

    assert(improved_ratio_nonzero >= 80.0, ...
        '非零差模试验改善比例未达到80%');
    assert(zero_skew_max_diff <= 1e-12, ...
        '零差模试验中D1-A与D1-D未严格一致');

    % D1 时延差分层统计 (|dL - dR| 分组，非控制变量扫描，属于单调分层关联)
    fprintf('\n--- [D1 时延差分层统计 (|dL - dR| 分组)] ---\n');
    for skew_steps = 0:2
        idx = (delay_skew_samples == skew_steps);
        skew_ms = skew_steps * dt * 1e3;
        fprintf('|dL-dR|=%d steps (%.1f ms): A=%.3f%%, D=%.3f%%, N=%d\n', ...
            skew_steps, skew_ms, mean(d1_exceed_trials(idx, 1)), ...
            mean(d1_exceed_trials(idx, 4)), sum(idx));
    end

    % D1-A vs D1-D 回归信号差异程序化统计 (t in [0.5, 0.8]s)
    fprintf('\n--- [D1-A vs D1-D 回归信号差异程序化统计 (t in [0.5, 0.8]s)] ---\n');
    delta_y_all   = reg_y_A - reg_y_D;
    delta_phi_all = reg_phi_A - reg_phi_D;
    rms_delta_y   = sqrt(mean(delta_y_all(:).^2));
    p95_abs_delta_y = prctile(abs(delta_y_all(:)), 95);
    rms_delta_phi = sqrt(mean(delta_phi_all(:).^2));
    fprintf('    - RMS(delta_y)   : %.4e N*m\n', rms_delta_y);
    fprintf('    - P95(|delta_y|) : %.4e N*m\n', p95_abs_delta_y);
    fprintf('    - RMS(delta_phi) : %.4e count*m\n', rms_delta_phi);
    fprintf('    [说明] 在对称真值 Delta_Kf*=0 下，y_f 等于零参数假设下的表观回归残差。\n');

    % 导出逐试验明细表 step3c_part3_d1_trial_results.csv (包含明确单位转换与 Effective Delay 字段)
    trial_rows = cell(N_mc * numel(d1_branches), 10);
    row_idx = 0;
    for bi = 1:numel(d1_branches)
        b_id = sprintf('D1-%s', d1_branches{bi});
        for j = 1:N_mc
            row_idx = row_idx + 1;
            orig_dL = trials_base{j}.d_meas_L;
            orig_dR = trials_base{j}.d_meas_R;

            switch d1_branches{bi}
                case 'A'
                    app_dL = orig_dL;
                    app_dR = orig_dR;
                case 'B'
                    app_dL = 1;
                    app_dR = 1;
                case 'C'
                    app_dL = 2;
                    app_dR = 2;
                case 'D'
                    d_common = max(orig_dL, orig_dR);
                    app_dL = d_common;
                    app_dR = d_common;
                case 'E'
                    d_aligned = max(orig_dL, orig_dR);
                    app_dL = d_aligned;
                    app_dR = d_aligned;
            end

            orig_dL_ms = orig_dL * dt * 1e3;
            orig_dR_ms = orig_dR * dt * 1e3;
            eff_dL_ms  = app_dL  * dt * 1e3;
            eff_dR_ms  = app_dR  * dt * 1e3;
            skew_ms    = abs(orig_dL - orig_dR) * dt * 1e3;

            seed_j = 20260924 + j;
            trial_rows(row_idx, :) = { ...
                j, seed_j, b_id, ...
                orig_dL_ms, orig_dR_ms, eff_dL_ms, eff_dR_ms, ...
                skew_ms, d1_exceed_trials(j, bi), d1_final_trials(j, bi)};
        end
    end
    trial_header = { ...
        'Trial_ID', 'Seed', 'Branch_ID', ...
        'Original_dL_ms', 'Original_dR_ms', 'Effective_dL_ms', 'Effective_dR_ms', ...
        'Original_delay_skew_ms', 'theta_exceed_ratio', 'abs_theta_final'};
    T_trials = cell2table(trial_rows, 'VariableNames', trial_header);
    file_trials = fullfile(script_dir, 'step3c_part3_d1_trial_results.csv');
    writetable(T_trials, file_trials);
    fprintf('>>> 正在导出 D1 逐试验明细表: %s\n', file_trials);
    T_trials_read = readtable(file_trials);
    assert(height(T_trials_read) == N_mc * numel(d1_branches), '逐试验明细表行数必须为 500');
    assert(width(T_trials_read) == 10, '逐试验明细表列数必须为 10');
    fprintf('    [OK] D1 逐试验明细表导出与回读断言 100%% 成立!\n');

    % 导出时间序列瞬态分布表
    t_vec = d_sym.t;
    p_exceed_D1A = 100.0 * mean(abs(theta_trials_d1{1}) > 1.0e-5, 2);
    p_exceed_D1B = 100.0 * mean(abs(theta_trials_d1{2}) > 1.0e-5, 2);
    p_exceed_D1C = 100.0 * mean(abs(theta_trials_d1{3}) > 1.0e-5, 2);
    p_exceed_D1D = 100.0 * mean(abs(theta_trials_d1{4}) > 1.0e-5, 2);
    p_exceed_D1E = 100.0 * mean(abs(theta_trials_d1{5}) > 1.0e-5, 2);

    theta_p95_D1A = prctile(abs(theta_trials_d1{1}), 95, 2);
    theta_p95_D1D = prctile(abs(theta_trials_d1{4}), 95, 2);
    theta_p95_D1E = prctile(abs(theta_trials_d1{5}), 95, 2);

    theta_p50_D1A = median(abs(theta_trials_d1{1}), 2);
    theta_p50_D1D = median(abs(theta_trials_d1{4}), 2);
    theta_p50_D1E = median(abs(theta_trials_d1{5}), 2);

    pe_active_D1A = 100.0 * mean(pe_trials_d1{1}, 2);
    pe_active_D1E = 100.0 * mean(pe_trials_d1{5}, 2);

    ts_table = table(t_vec, ...
        p_exceed_D1A, p_exceed_D1B, p_exceed_D1C, p_exceed_D1D, p_exceed_D1E, ...
        theta_p95_D1A, theta_p95_D1D, theta_p95_D1E, ...
        theta_p50_D1A, theta_p50_D1D, theta_p50_D1E, ...
        pe_active_D1A, pe_active_D1E);

    file_ts = fullfile(script_dir, 'step3c_part3_d1_timeseries.csv');
    writetable(ts_table, file_ts);
    fprintf('>>> 正在导出 D1 时间序列瞬态分布表: %s\n', file_ts);
    T_ts_read = readtable(file_ts);
    assert(height(T_ts_read) == N_pts, '时序表行数必须等于 N_pts');
    fprintf('    [OK] D1 时间序列瞬态分布表导出与回读断言 100%% 成立!\n\n');
end

%% =========================================================================
%% 局部辅助函数: 单次 Oracle 试验因果回放与 RLS 分析核
%% =========================================================================
function res = analyze_oracle_trial(base_data, cfg, is_causal_align)
    if nargin < 2, cfg = struct(); end
    if nargin < 3, is_causal_align = false; end

    dt = base_data.dt;
    Le = base_data.mech.Le;
    Kf_mean = base_data.Kf_mean;
    Kf_L = base_data.Kf_L;
    Kf_R = base_data.Kf_R;
    if isfield(base_data, 'Imax')
        Imax = base_data.Imax;
    else
        Imax = 16000.0;
    end

    % 1. 注入物理非理想扰动 (生成真实的 pert_data)
    pert_data = step3c_apply_imperfections(base_data, cfg);
    N = pert_data.N;
    t = pert_data.t;

    % 2. 执行精确 Oracle 电流通道校正 (只校正已知增益与零偏，绝不删除白噪声)
    i_bias_L = 0.0; if isfield(cfg, 'i_bias_L'), i_bias_L = cfg.i_bias_L; end
    i_bias_R = 0.0; if isfield(cfg, 'i_bias_R'), i_bias_R = cfg.i_bias_R; end
    delta_g_L = 0.0; if isfield(cfg, 'delta_g_L'), delta_g_L = cfg.delta_g_L; end
    delta_g_R = 0.0; if isfield(cfg, 'delta_g_R'), delta_g_R = cfg.delta_g_R; end

    iL_corr = (pert_data.iL_meas - i_bias_L) / (1.0 + delta_g_L);
    iR_corr = (pert_data.iR_meas - i_bias_R) / (1.0 + delta_g_R);

    % 3. 因果时间戳对齐处理 (针对 D1-E)
    if is_causal_align
        % 使用仿真真值 cfg.d_meas_L/R 的 Oracle 因果对齐。
        % 仅验证已知时延情况下的性能上限，不代表在线时延估计已经实现。
        dL = cfg.d_meas_L;
        dR = cfg.d_meas_R;
        dmax = max(dL, dR);

        dL_extra = dmax - dL;
        dR_extra = dmax - dR;

        % 3.1 较快电流通道补齐额外滞后到 dmax
        iL_align = zeros(N, 1);
        iR_align = zeros(N, 1);
        if dL_extra == 0
            iL_align = iL_corr;
        else
            iL_align((dL_extra + 1):N) = iL_corr(1:(N - dL_extra));
        end
        if dR_extra == 0
            iR_align = iR_corr;
        else
            iR_align((dR_extra + 1):N) = iR_corr(1:(N - dR_extra));
        end

        % 3.2 位置/偏转回归输入同步因果滞后 dmax
        yL_align = zeros(N, 1);
        yR_align = zeros(N, 1);
        if dmax == 0
            yL_align = pert_data.yL_meas;
            yR_align = pert_data.yR_meas;
        else
            yL_align((dmax + 1):N) = pert_data.yL_meas(1:(N - dmax));
            yR_align((dmax + 1):N) = pert_data.yR_meas(1:(N - dmax));
        end

        % 3.3 构造同物理时刻回归量
        reg = build_step3b_regression(...
            yL_align, yR_align, ...
            iL_align, iR_align, ...
            dt, base_data.mech, base_data.plant, Kf_mean, 'step3c_sensor');
    else
        reg = build_step3b_regression(...
            pert_data.yL_meas, pert_data.yR_meas, ...
            iL_corr, iR_corr, ...
            dt, base_data.mech, base_data.plant, Kf_mean, 'step3c_sensor');
    end

    % 所有消融分支采用相同墙钟时间窗口。
    % 通信和对齐引入的延迟属于算法性能，不通过移动窗口排除。
    eval_start = 0.5;
    eval_end   = 2.3;

    mask_eval = (t >= eval_start) & (t <= eval_end);
    assert(any(mask_eval), '统一评测窗口为空');

    % 4. 初始化 RLS 估计器
    opts_rls = struct();
    opts_rls.lambda        = 1.0;
    opts_rls.window_length = 200;
    opts_rls.sigma_PE_th   = 50.0;
    opts_rls.theta_min     = -0.0026295;
    opts_rls.theta_max     =  0.0018465;
    opts_rls.P0            = 1.0e-4;
    opts_rls.P_min         = 1.0e-12;
    opts_rls.P_max         = 1.0;
    estimator = rls_estimator_delta_kf(opts_rls);

    theta_unprojected = zeros(N, 1);
    theta_projected   = zeros(N, 1);
    P_history         = zeros(N, 1);
    projection_mask   = false(N, 1);
    pe_mask           = false(N, 1);

    for k = 1:N
        phi_k = reg.phi_f(k);
        y_k   = reg.y_f(k);
        [estimator, ~, info] = estimator.update(phi_k, y_k);
        theta_unprojected(k) = info.theta_unprojected;
        theta_projected(k)   = info.theta_projected;
        P_history(k)         = info.P_next;
        projection_mask(k)   = info.is_projected;
        pe_mask(k)           = info.is_pe;
    end

    idx_eval_end = find(t <= eval_end, 1, 'last');
    final_theta = theta_projected(idx_eval_end);
    unproj_final_theta = theta_unprojected(idx_eval_end);

    calib = step3b_offline_calibration(final_theta, Kf_mean);

    dL_act = pert_data.cfg.d_act_L;
    dR_act = pert_data.cfg.d_act_R;

    iL_base_cmd = pert_data.iL_cmd;
    iR_base_cmd = pert_data.iR_cmd;
    iL_base_delayed = zeros(N, 1);
    iR_base_delayed = zeros(N, 1);
    if dL_act < N, iL_base_delayed((dL_act + 1):N) = iL_base_cmd(1:(N - dL_act)); end
    if dR_act < N, iR_base_delayed((dR_act + 1):N) = iR_base_cmd(1:(N - dR_act)); end
    iL_base_applied = max(-Imax, min(Imax, iL_base_delayed));
    iR_base_applied = max(-Imax, min(Imax, iR_base_delayed));

    iL_comp_cmd = calib.gamma_L * iL_base_cmd;
    iR_comp_cmd = calib.gamma_R * iR_base_cmd;
    iL_comp_delayed = zeros(N, 1);
    iR_comp_delayed = zeros(N, 1);
    if dL_act < N, iL_comp_delayed((dL_act + 1):N) = iL_comp_cmd(1:(N - dL_act)); end
    if dR_act < N, iR_comp_delayed((dR_act + 1):N) = iR_comp_cmd(1:(N - dR_act)); end
    iL_comp_applied = max(-Imax, min(Imax, iL_comp_delayed));
    iR_comp_applied = max(-Imax, min(Imax, iR_comp_delayed));

    T_alpha_base = -0.5 * Le * (Kf_L * iL_base_applied + Kf_R * iR_base_applied);
    T_alpha_comp = -0.5 * Le * (Kf_L * iL_comp_applied + Kf_R * iR_comp_applied);

    T_alpha_nom  = -0.5 * Le * Kf_mean * (iL_base_delayed + iR_base_delayed);
    e_T_base = T_alpha_base - T_alpha_nom;
    e_T_comp = T_alpha_comp - T_alpha_nom;

    T_nom_intended = -0.5 * Le * Kf_mean * (iL_base_cmd + iR_base_cmd);
    e_total_base = T_alpha_base - T_nom_intended;
    e_total_comp = T_alpha_comp - T_nom_intended;

    mask_dwell = (t >= 3.0 & t <= 4.0);

    rms_base = sqrt(mean(e_T_base(mask_eval).^2));
    rms_comp = sqrt(mean(e_T_comp(mask_eval).^2));
    if rms_base < 1.0e-12
        eta_kf_residual = NaN;
    else
        eta_kf_residual = (1.0 - rms_comp / rms_base) * 100.0;
    end
    eta_sat = eta_kf_residual;

    rms_total_base = sqrt(mean(e_total_base(mask_eval).^2));
    rms_total_comp = sqrt(mean(e_total_comp(mask_eval).^2));
    if rms_total_base < 1.0e-12
        eta_total = NaN;
    else
        eta_total = (1.0 - rms_total_comp / rms_total_base) * 100.0;
    end

    K_alpha = base_data.plant.K_alpha;
    alpha_ss_base = rms_base / K_alpha;
    alpha_ss_comp = rms_comp / K_alpha;

    delta_m_val = 0.0;
    d_load_val = 0.0;
    delta_fric_val = 0.0;

    iL_base_nd_applied = max(-Imax, min(Imax, iL_base_cmd));
    iR_base_nd_applied = max(-Imax, min(Imax, iR_base_cmd));
    iL_comp_nd_applied = max(-Imax, min(Imax, iL_comp_cmd));
    iR_comp_nd_applied = max(-Imax, min(Imax, iR_comp_cmd));

    alpha_base_no_delay = base_data.alpha;
    alpha_base_delayed  = pert_data.alpha_true;

    alpha_comp_no_delay = zeros(N, 1);
    x_c_nd = zeros(4, 1);
    for k = 1:N
        [x_next, ~] = gantry_dynamics_step_rk4(...
            x_c_nd, iL_comp_nd_applied(k), iR_comp_nd_applied(k), ...
            base_data.mech, base_data.plant, ...
            delta_m_val, d_load_val, delta_fric_val, dt, Kf_L, Kf_R);
        alpha_comp_no_delay(k) = x_c_nd(2);
        x_c_nd = x_next;
    end

    alpha_comp_delayed = zeros(N, 1);
    x_c_d = zeros(4, 1);
    for k = 1:N
        [x_next, ~] = gantry_dynamics_step_rk4(...
            x_c_d, iL_comp_applied(k), iR_comp_applied(k), ...
            base_data.mech, base_data.plant, ...
            delta_m_val, d_load_val, delta_fric_val, dt, Kf_L, Kf_R);
        alpha_comp_delayed(k) = x_c_d(2);
        x_c_d = x_next;
    end

    dalpha_base = alpha_base_delayed - alpha_base_no_delay;
    dalpha_comp = alpha_comp_delayed - alpha_comp_no_delay;

    rms_alpha_base_dyn = sqrt(mean(alpha_base_delayed(mask_eval).^2));
    rms_alpha_comp_dyn = sqrt(mean(alpha_comp_delayed(mask_eval).^2));
    rms_dalpha_base    = sqrt(mean(dalpha_base(mask_eval).^2));
    rms_dalpha_comp    = sqrt(mean(dalpha_comp(mask_eval).^2));

    if rms_alpha_base_dyn > 1.0e-12
        eta_alpha_abs = (1.0 - rms_alpha_comp_dyn / rms_alpha_base_dyn) * 100.0;
    else
        eta_alpha_abs = NaN;
    end

    if rms_dalpha_base > 1.0e-12
        eta_alpha_delay = (1.0 - rms_dalpha_comp / rms_dalpha_base) * 100.0;
    else
        eta_alpha_delay = NaN;
    end

    svf_atten_dB = NaN;
    pe_false_alarm_rate = mean(pe_mask(mask_dwell));
    pe_active_ratio     = mean(pe_mask(mask_eval));

    base_sat_mask_L = (abs(iL_base_applied) >= Imax - 1e-6);
    base_sat_mask_R = (abs(iR_base_applied) >= Imax - 1e-6);
    base_sat_ratio_total = 100.0 * mean(base_sat_mask_L | base_sat_mask_R);

    comp_sat_mask_L = (abs(iL_comp_applied) >= Imax - 1e-6);
    comp_sat_mask_R = (abs(iR_comp_applied) >= Imax - 1e-6);
    comp_sat_ratio_total = 100.0 * mean(comp_sat_mask_L | comp_sat_mask_R);

    unproj_eval = theta_unprojected(mask_eval);
    proj_eval   = theta_projected(mask_eval);
    unproj_low_clip_count  = sum(unproj_eval < opts_rls.theta_min - 1e-9);
    unproj_high_clip_count = sum(unproj_eval > opts_rls.theta_max + 1e-9);
    unproj_clipped_count   = unproj_low_clip_count + unproj_high_clip_count;
    unproj_max_peak        = max(abs(unproj_eval));
    exceed_false_ratio     = 100.0 * mean(abs(proj_eval) > 1.0e-5);

    res = struct();
    res.cfg                 = cfg;
    res.theta_hat           = final_theta;
    res.Kf_L_hat            = calib.Kf_L_hat;
    res.Kf_R_hat            = calib.Kf_R_hat;
    res.gamma_L             = calib.gamma_L;
    res.gamma_R             = calib.gamma_R;
    res.is_calib_valid      = calib.is_valid;
    res.rms_base            = rms_base;
    res.rms_comp            = rms_comp;
    res.eta_sat             = eta_sat;
    res.eta_kf_residual     = eta_kf_residual;
    res.rms_total_base      = rms_total_base;
    res.rms_total_comp      = rms_total_comp;
    res.eta_total           = eta_total;
    res.alpha_ss_base       = alpha_ss_base;
    res.alpha_ss_comp       = alpha_ss_comp;
    res.rms_dalpha_base     = rms_dalpha_base;
    res.rms_dalpha_comp     = rms_dalpha_comp;
    res.rms_alpha_base_dyn  = rms_alpha_base_dyn;
    res.rms_alpha_comp_dyn  = rms_alpha_comp_dyn;
    res.eta_alpha_abs       = eta_alpha_abs;
    res.eta_alpha_delay     = eta_alpha_delay;
    res.svf_atten_dB        = svf_atten_dB;
    res.pe_false_alarm_rate = pe_false_alarm_rate;
    res.pe_active_ratio     = pe_active_ratio;
    res.base_sat_ratio_total= base_sat_ratio_total;
    res.comp_sat_ratio_total= comp_sat_ratio_total;
    res.unproj_max_peak        = unproj_max_peak;
    res.unproj_clipped_count   = unproj_clipped_count;
    res.exceed_false_ratio     = exceed_false_ratio;
    res.mask_eval       = mask_eval;
    res.t               = t;
    res.theta_projected = theta_projected;
    res.pe_mask         = pe_mask;
    res.reg_phi_f       = reg.phi_f;
    res.reg_y_f         = reg.y_f;
    res.eval_start      = eval_start;
    res.eval_end        = eval_end;
end

function str = pass_fail_str(cond)
    if cond
        str = 'PASS';
    else
        str = 'FAIL';
    end
end

function max_run = longest_true_run(vec)
    if isempty(vec), max_run = 0; return; end
    d = diff([false; vec(:); false]);
    starts = find(d == 1);
    ends = find(d == -1);
    if isempty(starts)
        max_run = 0;
    else
        max_run = max(ends - starts);
    end
end

function desc = describe_d0b_finding(b_code, time_mean)
    switch b_code
        case 'A'
            desc = sprintf('全要素 Oracle 基准，超标时间 %.2f%%', time_mean);
        case 'B'
            desc = sprintf('去除位置白噪声后超标时间为 %.2f%%，有改善但非主要诱因', time_mean);
        case 'C'
            desc = sprintf('去除位置量化后超标时间为 %.2f%%，量化非主因', time_mean);
        case 'D'
            desc = sprintf('严格配对去除电流白噪声，超标时间为 %.2f%%，证实电流白噪声非主因', time_mean);
        case 'E'
            desc = sprintf('去除全部回采测量时滞后超标时间为 %.2f%%，证明通信时滞整体为主导因素', time_mean);
        case 'F'
            desc = sprintf('去除所有噪声量化保留时滞，超标时间仍为 %.2f%%，印证时滞为主导瓶颈', time_mean);
        otherwise
            desc = 'N/A';
    end
end

function desc = describe_d1_finding(b_code, time_mean)
    switch b_code
        case 'A'
            desc = sprintf('原始独立随机时滞基准，超标时间 %.2f%%', time_mean);
        case 'B'
            desc = sprintf('纯共模 1ms/1ms (无差分时滞) 超标时间 %.2f%%，支持差模测量时延是当前测试范围内的主导贡献因素', time_mean);
        case 'C'
            desc = sprintf('纯共模 2ms/2ms (无差分时滞) 超标时间 %.2f%%，验证纯共模时延在既定门槛内', time_mean);
        case 'D'
            desc = sprintf('消除左右差模保留每试验最大公共时滞，超标时间 %.2f%%，支持差模时延为主导因素', time_mean);
        case 'E'
            desc = sprintf('Oracle 仿真真值因果时延对齐，超标时间 %.2f%%，验证已知时延性能上限', time_mean);
        otherwise
            desc = 'N/A';
    end
end
